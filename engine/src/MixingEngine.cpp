#include "MixingEngine.h"
#include <algorithm>
#include <array>
#include <cmath>

namespace dsp {

MixingEngine::MixingEngine()
    : controlQueue(static_cast<size_t>(ControlQueueSize)),
      appliedQueue(static_cast<size_t>(AppliedQueueSize)) {
    // Identity patch: channel n hears input n until the desk says otherwise.
    convolutionQueue = std::make_unique<juce::dsp::ConvolutionMessageQueue>();
    for (int i = 0; i < MaxChannels; ++i) {
        channels[static_cast<size_t>(i)].inputIndex = i;
        channels[static_cast<size_t>(i)].amp = std::make_unique<GuitarAmp>(*convolutionQueue);
        channels[static_cast<size_t>(i)].bass = std::make_unique<BassRig>(*convolutionQueue);
        channels[static_cast<size_t>(i)].drums = std::make_unique<DrumRig>(*convolutionQueue);
        channels[static_cast<size_t>(i)].vocals = std::make_unique<VocalRig>(*convolutionQueue);
        auto& ch = channels[static_cast<size_t>(i)];
        ch.keys = std::make_unique<KeysRig>(*convolutionQueue);
        ch.brass = std::make_unique<LightRig>(*convolutionQueue, LightRig::Brass);
        ch.phin = std::make_unique<LightRig>(*convolutionQueue, LightRig::Phin);
        ch.khaen = std::make_unique<LightRig>(*convolutionQueue, LightRig::Khaen);
        ch.saw = std::make_unique<LightRig>(*convolutionQueue, LightRig::Saw);
    }
    reset();
}

void MixingEngine::serviceBackgroundWork() {
    for (auto& chan : channels) {
        if (chan.amp) chan.amp->serviceBackground();
        if (chan.bass) chan.bass->serviceBackground();
        if (chan.drums) chan.drums->serviceBackground();
        if (chan.vocals) chan.vocals->serviceBackground();
        if (chan.keys) chan.keys->serviceBackground();
        for (auto* light : { chan.brass.get(), chan.phin.get(), chan.khaen.get(), chan.saw.get() }) if (light) light->serviceBackground();
    }
}

void MixingEngine::prepare(double newSampleRate, int newMaxBlockSize) {
    sampleRate = newSampleRate;
    maxBlockSize = newMaxBlockSize;

    // The assistant's capture buffer, sized here so the audio thread never allocates.
    captureArmed.store(false);
    captureFull.store(false);
    const auto captureFrames = static_cast<size_t>(std::ceil(MaxCaptureSec * sampleRate)) + static_cast<size_t>(maxBlockSize);
    captureL.assign(captureFrames, 0.0f);
    captureR.assign(captureFrames, 0.0f);

    // Prepare channels
    for (int i = 0; i < MaxChannels; ++i) {
        channels[i].prepare(sampleRate, maxBlockSize);
        channels[i].name = "Ch " + std::to_string(i + 1);
    }

    // Prepare master bus
    master.prepare(sampleRate, maxBlockSize);

    // Prepare output delays
    for (int i = 0; i < MaxOutputs; ++i) {
        outputDelays[i].prepare(sampleRate, maxBlockSize);
        outputDelays[i].setDelayMs(outputDelayMs[static_cast<size_t>(i)]);
        speakers[static_cast<size_t>(i)].prepare(sampleRate, maxBlockSize);
    }

    // Prepare send returns
    fxBus.prepare(sampleRate, maxBlockSize);
    for (auto& m : monitors) m.prepare(sampleRate, maxBlockSize);

    // Pre-allocate real-time processing vectors
    mainBusL.assign(maxBlockSize, 0.0f);
    mainBusR.assign(maxBlockSize, 0.0f);
    fxReturnL.assign(maxBlockSize, 0.0f);
    fxReturnR.assign(maxBlockSize, 0.0f);
    subL.assign(maxBlockSize, 0.0f);
    subR.assign(maxBlockSize, 0.0f);
    mainL.assign(maxBlockSize, 0.0f);
    mainR.assign(maxBlockSize, 0.0f);
    postDspBuffer.assign(maxBlockSize, 0.0f);
    postFaderBuffer.assign(maxBlockSize, 0.0f);
    masterInterleaved.assign(2 * maxBlockSize, 0.0f);
    subInterleaved.assign(2 * maxBlockSize, 0.0f);
    mainInterleaved.assign(2 * maxBlockSize, 0.0f);

    for (int b = 0; b < MaxAuxBuses; ++b) {
        auxBuses[b].assign(maxBlockSize, 0.0f);
    }
    testBuffer.assign(static_cast<size_t>(maxBlockSize), 0.0f);

    reset();
    // Anything the desk changed while the device was stopped.
    serviceControls();
}

void MixingEngine::reset() {
    for (int i = 0; i < MaxChannels; ++i) {
        channels[i].reset();
    }
    master.reset();
    for (int i = 0; i < MaxOutputs; ++i) {
        outputDelays[i].reset();
        speakers[static_cast<size_t>(i)].reset();
    }
    fxBus.reset();
    // A test that was beeping when the device stopped does not resume half-way through.
    outputTone.stop();
    monitorTone.stop();
}

namespace {
/** Raise an atomic to at least `v` — the audio thread's half of a held meter. Never blocks. */
void raiseTo(std::atomic<float>& a, float v) {
    float cur = a.load(std::memory_order_relaxed);
    while (v > cur && !a.compare_exchange_weak(cur, v, std::memory_order_relaxed)) {}
}
} // namespace

void MixingEngine::processAudio(
    const float** inputs, 
    int channelsCount, 
    float** outputs, 
    int outputsCount, 
    int numSamples
) {
    /*
     * Flush denormals to zero for the whole block. Every IIR filter in this engine decays towards
     * zero after a signal stops, and on the way it passes through subnormal floats, which x86
     * processes up to a hundred times slower — the CPU load of a silent desk climbing until the
     * block misses its deadline, the moment a band stops playing. Restored on return, so nothing
     * outside the callback is affected.
     */
    juce::ScopedNoDenormals noDenormals;

    // Controls first, so the whole block is processed with one set of settings.
    serviceControls();

    // Safety clamp
    int activeSamples = std::min(numSamples, maxBlockSize);
    int activeInChannels = std::min(channelsCount, MaxChannels);
    int activeOutChannels = std::min(outputsCount, MaxOutputs);

    // The input jack meters (1.0.25): each device input as it arrived, before any channel touched it.
    for (int c = 0; c < activeInChannels; ++c) {
        if (inputs[c] == nullptr) continue;
        float peak = 0.0f;
        for (int i = 0; i < activeSamples; ++i) peak = std::max(peak, std::abs(inputs[c][i]));
        if (std::isfinite(peak)) raiseTo(inputPeak[static_cast<size_t>(c)], peak);
    }

    // 1. Clear accumulators
    std::fill(mainBusL.begin(), mainBusL.begin() + activeSamples, 0.0f);
    std::fill(mainBusR.begin(), mainBusR.begin() + activeSamples, 0.0f);
    for (int b = 0; b < MaxAuxBuses; ++b) {
        std::fill(auxBuses[b].begin(), auxBuses[b].begin() + activeSamples, 0.0f);
    }


    // 2. Process input channels
    // Every channel, not just the first `activeInChannels`: with the input patch, channel 7 can
    // hear input 1 on a two-input interface.
    /*
     * The sidechain's key is processed **first**, so the duck it computes is in hand when the
     * target is reached in the same block — a duck a block late lets the kick's attack through
     * on top of the bass, which is the one moment it exists to clear.
     */
    const bool duckActive = master.sidechainActive();
    std::array<int, MaxChannels> order {};
    {
        int k = 0;
        if (duckActive) order[static_cast<size_t>(k++)] = master.sidechainKey;
        for (int c = 0; c < MaxChannels; ++c) {
            if (duckActive && c == master.sidechainKey) continue;
            order[static_cast<size_t>(k++)] = c;
        }
    }
    bool keyDone = false;

    for (int k = 0; k < MaxChannels; ++k) {
        const int c = order[static_cast<size_t>(k)];
        // The key produced nothing this block (no input patched, or an absent device channel):
        // the envelope still decays, so a stopped kick does not leave the bass ducked.
        if (duckActive && !keyDone && c != master.sidechainKey) {
            master.duck.processSilence(activeSamples);
            keyDone = true;
        }
        auto& chan = channels[c];
        const int src = chan.inputIndex;
        if (src < 0 || src >= activeInChannels) {
            // No input. The meter still has to be told, or it freezes on the last reading of
            // whatever this channel heard before its patch changed — a frozen meter reads as a
            // live signal. Only for the channels the meters can show; the rest cost nothing.
            if (c < MeteredChannels) {
                std::fill(postFaderBuffer.begin(), postFaderBuffer.begin() + activeSamples, 0.0f);
                chan.metering.processBlock(postFaderBuffer.data(), activeSamples, 0.0f);
            }
            continue;
        }
        const float* chanIn = inputs[src];
        /**
         * A channel pointer can legitimately be null.
         *
         * `AudioIODeviceCallback` passes null for any channel inside the reported count that
         * the device's active-channel mask leaves off, and this engine asks for 32 inputs on
         * hardware that mostly has two. Dereferencing it is a crash on the audio thread,
         * which takes the whole show with it.
         *
         * Skipped rather than treated as silence on purpose: running an absent input through
         * the gate and compressor costs a full channel of DSP to produce zeros, times however
         * many of the 32 are not there.
         */
        if (chanIn == nullptr) continue;
        // Clamped here as well as at the control plane, deliberately: this is the one place
        // every writer must pass through, so a future setter that forgets cannot put an
        // unbounded number into a gain multiply. Same reasoning as `clampAmp` sitting between
        // every untrusted source and the amp chain in the web app.
        //
        // Targets, not values: the ramp is what stops a fader move being a step
        // discontinuity at a block boundary. See `Channel::trimGain`.
        chan.trimGain.setTargetValue(juce::Decibels::decibelsToGain(clampTrimDb(chan.trimDb)));
        chan.faderGain.setTargetValue(juce::Decibels::decibelsToGain(clampFaderDb(chan.faderDb)) * dcaGainFor(static_cast<int>(c)));
        chan.groupGain.setTargetValue(groupGainFor(static_cast<int>(c)));

        /*
         * Two passes, because the rack is a block process (oversampling, convolution) sitting
         * in the middle of a per-sample strip: trim → Ø → HPF → gate into `postDspBuffer`, then the
         * rack on that block, then the EQ, the dynamics, fader, sends and pan.
         *
         * The EQ is after the rack (1.0.27), as on the web desk. Before it, the rack's compressor and
         * limiter evened out whatever the EQ did — +12 dB on a low shelf measured 1–3 dB — so an EQ
         * move could not be heard as the size it was. Each smoother
         * is still advanced exactly once per sample — the trim in the first pass, the fader in
         * the second — so neither falls behind the block clock.
         */
        for (int i = 0; i < activeSamples; ++i) {
            const float trimGain = chan.trimGain.getNextValue();

            // Trim and phase invert
            float x = chanIn[i] * trimGain;
            if (chan.phaseInvert) {
                x = -x;
            }

            // High pass filter
            if (chan.hpfEnabled) {
                x = chan.hpf.processSample(x);
            }

            postDspBuffer[i] = chan.gate.processSample(x);
        }

        if (chan.amp && chan.amp->isEnabled()) chan.amp->process(postDspBuffer.data(), activeSamples);
        if (chan.bass && chan.bass->isEnabled()) chan.bass->process(postDspBuffer.data(), activeSamples);
        if (chan.drums && chan.drums->isEnabled()) chan.drums->process(postDspBuffer.data(), activeSamples);
        if (chan.vocals && chan.vocals->isEnabled()) chan.vocals->process(postDspBuffer.data(), activeSamples);
        if (chan.keys && chan.keys->isEnabled()) chan.keys->process(postDspBuffer.data(), activeSamples);
        for (auto* light : { chan.brass.get(), chan.phin.get(), chan.khaen.get(), chan.saw.get() })
            if (light && light->isEnabled()) light->process(postDspBuffer.data(), activeSamples);

        // Send gains once per block, not a `pow` per sample per bus — 32 channels × 10 buses was
        // 15 million of them a second at 48 kHz, for numbers that only change between blocks.
        std::array<float, MaxAuxBuses> sendGain {};
        for (int b = 0; b < MaxAuxBuses; ++b)
            if (chan.sends[static_cast<size_t>(b)].enabled)
                sendGain[static_cast<size_t>(b)] = juce::Decibels::decibelsToGain(chan.sends[static_cast<size_t>(b)].levelDb);

        bool blewUp = false;
        for (int i = 0; i < activeSamples; ++i) {
            const float faderGain = chan.faderGain.getNextValue();
            const float groupGain = chan.groupGain.getNextValue();
            // The strip EQ, after the rack (1.0.27).
            float x = chan.eq.processSample(postDspBuffer[i]);

            // Dynamics after the rack: De-esser, Compressor
            x = chan.deesser.processSample(x);
            x = chan.comp.processSample(x);

            // Low pass filter
            if (chan.lpfEnabled) {
                x = chan.lpf.processSample(x);
            }

            // The sidechain duck, before the fader — where the web desk puts it.
            if (duckActive && c == master.sidechainTarget) x *= master.duck.gainAt(i);

            // The alignment delay: the whole channel, so the sends leave aligned too. 0 = pass-through.
            x = chan.alignDelay.processSample(x);

            /*
             * A NaN or an infinity stops here, at the end of the strip, before it reaches a bus.
             * It can come in (a driver that hands over garbage) or be made inside (a filter that
             * went unstable); either way it is now in some filter's state and will come out of it
             * on every sample from here on. Zeroed, and the channel's DSP is cleared after the
             * block, so one bad sample costs one block of this channel rather than the show.
             */
            if (!std::isfinite(x)) { x = 0.0f; blewUp = true; }

            postDspBuffer[i] = x;
            
            // Fader gain
            float postFader = x * faderGain;
            postFaderBuffer[i] = postFader;

            // Route to Aux/FX sends
            for (int b = 0; b < MaxAuxBuses; ++b) {
                if (chan.sends[b].enabled) {
                    float sendSig = chan.sends[b].preFader ? x : postFader;
                    auxBuses[b][i] += sendSig * sendGain[static_cast<size_t>(b)];
                }
            }

            // Route to Main stereo bus
            if (chan.routedToMain) {
                float outL = 0.0f;
                float outR = 0.0f;
                // The subgroup scales the main-bus contribution only — the sends above left first.
                chan.panner.processSample(postFader * groupGain, outL, outR);
                mainBusL[i] += outL;
                mainBusR[i] += outR;
            }
        }

        if (blewUp) {
            chan.clearDspState();
            nonFinite.fetch_add(1, std::memory_order_relaxed);
        }

        // Process channel metering
        float maxGrDb = chan.comp.getGainReductionDb() + chan.gate.getGainReductionDb() + chan.deesser.getGainReductionDb();
        chan.metering.processBlock(postFaderBuffer.data(), activeSamples, maxGrDb);

        if (duckActive && c == master.sidechainKey) {
            master.duck.processKey(postFaderBuffer.data(), activeSamples);
            keyDone = true;
        }
    }

    // A monitor mix's test tone, into its bus before its EQ, fader and limiter — see `TestTone`.
    if (monitorTone.active() && static_cast<int>(testBuffer.size()) >= activeSamples) {
        const int m = monitorTone.target;
        if (monitorTone.fill(testBuffer.data(), activeSamples) && m >= 0 && m < MonitorAuxBuses)
            for (int i = 0; i < activeSamples; ++i) auxBuses[static_cast<size_t>(m)][static_cast<size_t>(i)] += testBuffer[static_cast<size_t>(i)];
    }

    // The six monitor mixes: each aux bus through its own EQ, fader and limiter, in place,
    // before the output patch reads it.
    for (int b = 0; b < MonitorAuxBuses; ++b)
        monitors[static_cast<size_t>(b)].process(auxBuses[static_cast<size_t>(b)].data(), activeSamples);

    // 3. Process Send/Return FX Bus (Reverb on ReverbAux, Delay on DelayAux)
    // Each effect hears its own send — see `ReverbDelay::processSends`.
    float* fxReturns[2] = { fxReturnL.data(), fxReturnR.data() };
    fxBus.processSends(auxBuses[ReverbAux].data(), auxBuses[DelayAux].data(), fxReturns, activeSamples);

    // Sum FX returns to Master bus
    for (int i = 0; i < activeSamples; ++i) {
        mainBusL[i] += fxReturnL[i];
        mainBusR[i] += fxReturnR[i];
    }

    // 4. Master processing (interleaved stereo processing)
    // Left & Right GEQ
    master.geqL.processBlock(mainBusL.data(), activeSamples);
    master.geqR.processBlock(mainBusR.data(), activeSamples);

    // Mono below the corner, after the GEQ and before the fader, suppressor and limiter —
    // the web desk's position, so the brickwall sees it.
    master.monoBass.process(mainBusL.data(), mainBusR.data(), activeSamples);

    /**
     * Master fader and mute, packed into the interleaved buffer in the same pass.
     *
     * Before the suppressor and the limiter on purpose — this is a mix decision, so
     * everything downstream that exists to protect the output has to see its result. After
     * the limiter it would be a way to push the desk straight back through a ceiling the
     * limiter had just enforced.
     */
    master.gain.setTargetValue(master.targetGain());
    for (int i = 0; i < activeSamples; ++i) {
        const float g = master.gain.getNextValue();
        masterInterleaved[2 * i] = mainBusL[i] * g;
        masterInterleaved[2 * i + 1] = mainBusR[i] * g;
    }

    // Feedback Suppressor (Stereo).
    //
    // `processBlockInterleaved`, not `processBlock`. The mono overload takes a count of
    // floats, so handing it this buffer and a frame count left the back half of every block
    // unsuppressed and ran one biquad bank over L and R at once. The `Limiter` call below
    // has always used the frame-count contract; these two now agree.
    master.suppressor.processBlockInterleaved(masterInterleaved.data(), activeSamples);

    // Master Limiter (Stereo True Peak lookahead)
    master.limiter.processBlock(masterInterleaved.data(), activeSamples);

    /**
     * **Deinterleave the finished master back into `mainBusL/R`, because that is what the
     * meters read — and until this line they were reading the wrong signal.**
     *
     * `mainBusL/R` last held the post-GEQ mix. The suppressor and the limiter both work on
     * `masterInterleaved`, in place, so the metering call further down was measuring the
     * master **before** the feedback suppressor, before the limiter and before the crossover
     * — under a panel captioned MASTER OUTPUT.
     *
     * Two consequences, and the second is the one that would have cost somebody a day:
     *
     *  - The true-peak reading could sit above the limiter's own ceiling, so the desk
     *    reported an overload it had already dealt with.
     *  - **The RTA drew the spectrum without the notches in it.** A suppressor holding a
     *    howl down at 2 kHz would leave a peak at 2 kHz on the analyser, on the one screen
     *    whose job is to show what the suppressor did — and every reading around it correct.
     *
     * Same family as the two taps already corrected in the web app: `/overview`'s RTA reading
     * pre-monitor-gain, and the recorder's input meters reading post-limiter. A convenient
     * tap is not a correct one.
     *
     * Pre-crossover on purpose: the split into sub and main is a loudspeaker-management
     * decision about four physical outputs, and "the master" is the stereo programme the desk
     * is sending, which is exactly what has just left the limiter.
     */
    for (int i = 0; i < activeSamples; ++i) {
        mainBusL[i] = masterInterleaved[2 * i];
        mainBusR[i] = masterInterleaved[2 * i + 1];
    }

    // 5. Master Crossover Sub/Main split
    master.crossover.processBlockInterleaved(
        masterInterleaved.data(), 
        subInterleaved.data(), 
        mainInterleaved.data(), 
        activeSamples
    );

    // Unpack Crossover bands
    for (int i = 0; i < activeSamples; ++i) {
        subL[i] = subInterleaved[2 * i];
        subR[i] = subInterleaved[2 * i + 1];
        mainL[i] = mainInterleaved[2 * i];
        mainR[i] = mainInterleaved[2 * i + 1];
    }

    // Process Master Metering (LUFS, Peak, RTA) — post-limiter, see the deinterleave above.
    const float* masterStereo[2] = { mainBusL.data(), mainBusR.data() };
    master.metering.processBlock(masterStereo, activeSamples, master.limiter.getGainReductionDb());

    // 6. Output routing with time-alignment delays
    //   crossover off (default): Out 0, 1 = full-range master L/R
    //   crossover on, 4+ outputs: Out 0, 1 = mains above the split, Out 2, 3 = sub below it
    // All other physical outputs are silent.

    /**
     * **Every output is silenced first, and that is not tidiness.**
     *
     * An `AudioIODeviceCallback` is handed output buffers that are *not* guaranteed to be
     * zeroed — they routinely hold the previous block, or whatever the driver left there.
     * Filling them is the callback's job. This engine asks for 32 outputs (16 before 1.0.19) and only ever wrote
     * four, so outputs 4..15 carried undefined content **straight to a PA**, and a device
     * with one output got nothing written at all while the code fell through both branches
     * in silence.
     *
     * `numSamples` rather than `activeSamples`: if a host ever hands over a longer block than
     * `prepare` was told about, the clamp above stops us *processing* the tail — it must not
     * leave that tail as garbage on the way out.
     */
    // The assistant's capture (1.0.26): the master as it left the limiter.
    if (captureArmed.load(std::memory_order_acquire)) {
        const int target = captureTarget.load(std::memory_order_relaxed);
        const int at = captureWritten.load(std::memory_order_relaxed);
        const int take = std::min(activeSamples, target - at);
        if (take > 0) {
            std::copy(mainBusL.begin(), mainBusL.begin() + take, captureL.begin() + at);
            std::copy(mainBusR.begin(), mainBusR.begin() + take, captureR.begin() + at);
            captureWritten.store(at + take, std::memory_order_relaxed);
        }
        if (at + std::max(take, 0) >= target) {
            captureArmed.store(false, std::memory_order_relaxed);
            captureFull.store(true, std::memory_order_release);
        }
    }

    for (int o = 0; o < activeOutChannels; ++o) {
        if (outputs[o] != nullptr) {
            std::fill(outputs[o], outputs[o] + numSamples, 0.0f);
        }
    }

    // An output's test beeps (see `TestTone`): added to that jack just before its speaker
    // processor, whichever routing branch below writes it — or into its silence if none does.
    int testOut = -1;
    if (outputTone.active() && static_cast<int>(testBuffer.size()) >= activeSamples) {
        const int which = outputTone.target;
        if (outputTone.fill(testBuffer.data(), activeSamples) && which >= 0 && which < activeOutChannels
            && outputs[which] != nullptr)
            testOut = which;
    }
    bool testAdded = false;
    const auto finish = [&](int o) {
        if (o == testOut && !testAdded) {
            for (int i = 0; i < activeSamples; ++i) outputs[o][i] += testBuffer[static_cast<size_t>(i)];
            testAdded = true;
        }
        finishOutput(o, outputs[o], activeSamples);
    };

    /*
     * Outputs 0/1 are the full-range master unless the sub/main split has been switched on.
     * See `MasterBus::crossoverEnabled` for the bug this replaced: with the split always on, a
     * stereo device that Windows opened as eight channels carried the sub band on the pair
     * anybody was listening to, and the programme on outputs nothing was plugged into.
     */
    if (outputPatched) {
        // The output patch: each physical output carries exactly what the desk chose for it.
        // A source two outputs share is copied to both; an output nothing was patched to is
        // the silence written above.
        for (int o = 0; o < activeOutChannels; ++o) {
            if (outputs[o] == nullptr) continue;
            const float* src = nullptr;
            switch (outputSources[static_cast<size_t>(o)]) {
                case OutputSource::MasterL: src = mainBusL.data(); break;
                case OutputSource::MasterR: src = mainBusR.data(); break;
                case OutputSource::MainL:   src = mainL.data(); break;
                case OutputSource::MainR:   src = mainR.data(); break;
                case OutputSource::SubL:    src = subL.data(); break;
                case OutputSource::SubR:    src = subR.data(); break;
                case OutputSource::Aux1: case OutputSource::Aux2: case OutputSource::Aux3:
                case OutputSource::Aux4: case OutputSource::Aux5: case OutputSource::Aux6:
                case OutputSource::Aux7: case OutputSource::Aux8:
                    src = auxBuses[static_cast<size_t>(static_cast<int>(outputSources[static_cast<size_t>(o)])
                                                       - static_cast<int>(OutputSource::Aux1))].data();
                    break;
                case OutputSource::None: break;
            }
            if (src == nullptr) continue;
            std::copy(src, src + activeSamples, outputs[o]);
            finish(o);
        }
    } else if (master.crossoverEnabled && activeOutChannels >= 4) {
        std::copy(mainL.begin(), mainL.begin() + activeSamples, outputs[0]);
        std::copy(mainR.begin(), mainR.begin() + activeSamples, outputs[1]);
        std::copy(subL.begin(), subL.begin() + activeSamples, outputs[2]);
        std::copy(subR.begin(), subR.begin() + activeSamples, outputs[3]);
        for (int o = 0; o < 4; ++o) finish(o);
    } else if (activeOutChannels >= 2) {
        // The master as it left the limiter — not sub + main re-summed, which is the same
        // signal only while the crossover is a perfect all-pass and costs two filters to prove.
        std::copy(mainBusL.begin(), mainBusL.begin() + activeSamples, outputs[0]);
        std::copy(mainBusR.begin(), mainBusR.begin() + activeSamples, outputs[1]);
        finish(0);
        finish(1);
    } else if (activeOutChannels == 1) {
        // A mono device gets the fold-down. Halved: L and R are correlated, and the limiter
        // has already run, so nothing downstream would catch +6 dB.
        for (int i = 0; i < activeSamples; ++i) {
            outputs[0][i] = 0.5f * (mainBusL[i] + mainBusR[i]);
        }
        finish(0);
    }
    // A jack nothing is routed to still beeps: finding an unpatched cable is half the point.
    if (testOut >= 0 && !testAdded) finish(testOut);
}

float MixingEngine::testPitchFor(const SpeakerProcessor::Settings& s) {
    const float lo = s.hpf.kind != SpeakerProcessor::FilterKind::Off ? s.hpf.hz : 20.0f;
    const float hi = s.lpf.kind != SpeakerProcessor::FilterKind::Off ? s.lpf.hz : 20000.0f;
    // Half an octave-ish inside each corner when the band allows it; the middle of it when it does not.
    const float a = lo * 1.5f, b = hi / 1.5f;
    const float hz = a <= b ? std::clamp(1000.0f, a, b) : std::sqrt(std::max(lo, 1.0f) * std::max(hi, 1.0f));
    return std::clamp(hz, 20.0f, 18000.0f);
}

void MixingEngine::finishOutput(int output, float* buffer, int numSamples) {
    auto& sp = speakers[static_cast<size_t>(output)];
    sp.process(buffer, numSamples);
    outputDelays[static_cast<size_t>(output)].processBlock(buffer, numSamples);
    // The output meter (1.0.24): what actually leaves the jack, after the delay.
    float peak = 0.0f;
    for (int i = 0; i < numSamples; ++i) peak = std::max(peak, std::abs(buffer[i]));
    if (std::isfinite(peak)) raiseTo(outputPeak[static_cast<size_t>(output)], peak);
    if (sp.isActive()) raiseTo(outputReduction[static_cast<size_t>(output)], sp.gainReductionDb());
}

float MixingEngine::takeOutputPeak(int output) {
    if (output < 0 || output >= MaxOutputs) return 0.0f;
    return outputPeak[static_cast<size_t>(output)].exchange(0.0f, std::memory_order_relaxed);
}

bool MixingEngine::startCapture(double seconds) {
    if (captureArmed.load() || captureFull.load() || captureL.empty() || !(seconds > 0.0)) return false;
    const int frames = static_cast<int>(std::min(seconds, MaxCaptureSec) * sampleRate);
    if (frames < 1) return false;
    captureTarget.store(frames, std::memory_order_relaxed);
    captureWritten.store(0, std::memory_order_relaxed);
    captureArmed.store(true, std::memory_order_release);
    return true;
}

bool MixingEngine::takeCapture(std::vector<float>& left, std::vector<float>& right) {
    if (!captureFull.load(std::memory_order_acquire)) return false;
    const int n = captureWritten.load(std::memory_order_relaxed);
    left.assign(captureL.begin(), captureL.begin() + n);
    right.assign(captureR.begin(), captureR.begin() + n);
    captureFull.store(false, std::memory_order_release);
    return true;
}

float MixingEngine::takeInputPeak(int input) {
    if (input < 0 || input >= MaxChannels) return 0.0f;
    return inputPeak[static_cast<size_t>(input)].exchange(0.0f, std::memory_order_relaxed);
}

float MixingEngine::takeOutputReductionDb(int output) {
    if (output < 0 || output >= MaxOutputs) return 0.0f;
    return outputReduction[static_cast<size_t>(output)].exchange(0.0f, std::memory_order_relaxed);
}

// =====================================================================================
// Cookbook → state-variable filter. The reasoning is on `EqShape` in the header.
// =====================================================================================
ParametricEQ::BandConfig MixingEngine::cookbookToSvf(const EqBandSetting& band, double fs) {
    constexpr double pi = 3.14159265358979323846;
    ParametricEQ::BandConfig out;
    out.enabled = band.enabled;
    out.gainDb = std::clamp(band.gainDb, FilterPrimitives::MinGainDb, FilterPrimitives::MaxGainDb);

    // Below Nyquist with margin: tan() of the prewarp runs to infinity at fs/2.
    const double top = std::min(static_cast<double>(FilterPrimitives::MaxFreqHz), 0.45 * fs);
    double f = std::clamp(static_cast<double>(band.frequencyHz),
                          static_cast<double>(FilterPrimitives::MinFreqHz), top);
    double q = band.q;
    const double k = std::pow(10.0, out.gainDb / 20.0);

    switch (band.shape) {
        case EqShape::Peaking:
            out.type = FilterPrimitives::Type::Peaking;
            // Cookbook bandwidth is symmetric in dB; the SVF's is set on the boosted side.
            q *= std::pow(10.0, std::abs(out.gainDb) / 40.0);
            break;
        case EqShape::LowShelf:
            out.type = FilterPrimitives::Type::LowShelf;
            // Move the pole so the midpoint lands where the cookbook puts it, prewarped.
            f = fs / pi * std::atan(std::tan(pi * f / fs) / std::pow(k, 0.25));
            break;
        case EqShape::HighShelf:
            out.type = FilterPrimitives::Type::HighShelf;
            f = fs / pi * std::atan(std::tan(pi * f / fs) * std::pow(k, 0.25));
            break;
    }
    out.frequencyHz = static_cast<float>(f);
    out.q = static_cast<float>(q);
    return out;
}

// =====================================================================================
// The /channel/<n>/... routing table. See the declaration for why it is here.
// =====================================================================================
namespace {
bool on(float v) { return v >= 0.5f; }
float limit(float lo, float hi, float v) { return std::clamp(v, lo, hi); }
/**
 * A whole number from the wire, inside [lo, hi], or false. The range is checked **before** the
 * cast: static_cast<int>(1e30f) is undefined behaviour, and a fuzzed or hostile message carries
 * exactly that. A fraction is a malformed message, not a request to round.
 */
bool wireInt(float v, int lo, int hi, int& out) {
    if (!std::isfinite(v) || v < static_cast<float>(lo) || v > static_cast<float>(hi)) return false;
    const int n = static_cast<int>(v);
    if (static_cast<float>(n) != v) return false;
    out = n;
    return true;
}
} // namespace

bool MixingEngine::setChannelParam(int index, std::string_view param, float value) {
    if (index < 0 || index >= MaxChannels) return false;
    if (!std::isfinite(value)) return false;
    auto& ch = channels[static_cast<size_t>(index)];

    // The guitar rack. Its own table, allocation-free; an impulse response it needs is built
    // later on the message thread (`serviceBackgroundWork`).
    if (param.substr(0, 4) == "amp/") return ch.amp && ch.amp->setParam(param.substr(4), value);
    if (param.substr(0, 5) == "bass/") return ch.bass && ch.bass->setParam(param.substr(5), value);
    if (param.substr(0, 6) == "drums/") return ch.drums && ch.drums->setParam(param.substr(6), value);
    if (param.substr(0, 7) == "vocals/") return ch.vocals && ch.vocals->setParam(param.substr(7), value);
    if (param.substr(0, 5) == "keys/") return ch.keys && ch.keys->setParam(param.substr(5), value);
    if (param.substr(0, 6) == "brass/") return ch.brass && ch.brass->setParam(param.substr(6), value);
    if (param.substr(0, 5) == "phin/") return ch.phin && ch.phin->setParam(param.substr(5), value);
    if (param.substr(0, 6) == "khaen/") return ch.khaen && ch.khaen->setParam(param.substr(6), value);
    if (param.substr(0, 4) == "saw/") return ch.saw && ch.saw->setParam(param.substr(4), value);

    if (param == "fader") { ch.faderDb = clampFaderDb(value); return true; }
    // `dca <mask>`: which DCAs this channel is in, bit n = DCA n+1. A fraction or a bit past the
    // eighth is a malformed message, not a request to round.
    if (param == "dca") {
        int mask = 0;
        if (!wireInt(value, 0, (1 << DcaCount) - 1, mask)) return false;
        ch.dcaMask = static_cast<uint32_t>(mask);
        return true;
    }
    // `group <n>`: 0 = no subgroup, 1..8. A fraction or a number past the eighth is malformed.
    if (param == "group") {
        int g = 0;
        if (!wireInt(value, 0, GroupCount, g)) return false;
        ch.group = g;
        return true;
    }
    if (param == "trim")  { ch.trimDb = clampTrimDb(value); return true; }
    if (param == "pan")   { ch.panner.setPan(value); return true; }
    // The engine has no `muted` flag; unrouting from the main bus is the only mute it has.
    if (param == "mute")  { ch.routedToMain = !on(value); return true; }
    if (param == "invert") { ch.phaseInvert = on(value); return true; }
    // `delay <ms>`: the channel's alignment delay, 0–50 ms — the web strip's own range.
    if (param == "delay") {
        ch.delayMs = limit(0.0f, Channel::MaxDelayMs, value);
        ch.alignDelay.setDelayMs(ch.delayMs);
        return true;
    }
    // The input is 0-based (unlike the channel in the address) and -1 means none. A fraction
    // is a malformed patch, not a request to round.
    if (param == "input") {
        int in = 0;
        if (!wireInt(value, NoInput, MaxChannels - 1, in)) return false;
        ch.inputIndex = in;
        return true;
    }

    // Post-fader sends into the two FX returns — ReverbAux is the reverb, DelayAux the delay (see
    // `processBlock`). Until these existed nothing could switch a send on, so the Master FX
    // page's reverb and delay were fed silence whatever they were set to. The value is the send
    // level in dB; at the floor the send is off rather than summing zeros.
    // The eight monitor auxes, post-fader like the FX sends: `send/aux/<1..8>` in dB, off at the
    // floor. They reach a speaker only through the output patch (`/output/<n>/source`).
    if (param.substr(0, 9) == "send/aux/" && param.size() >= 10) {
        const int b = param[9] - '1';
        if (b < 0 || b >= MonitorAuxBuses) return false;
        auto& s = ch.sends[static_cast<size_t>(b)];
        if (param.size() == 10) {
            s.levelDb = limit(-120.0f, 10.0f, value);
            s.enabled = s.levelDb > -120.0f;
            return true;
        }
        // `send/aux/<n>/pre`: before the fader (a monitor mix the FOH fader cannot move) or
        // after it. Apart from the level, so changing one never resets the other.
        if (param.substr(10) == "/pre") { s.preFader = on(value); return true; }
        return false;
    }
    if (param == "send/reverb" || param == "send/delay") {
        auto& s = ch.sends[static_cast<size_t>(param == "send/reverb" ? ReverbAux : DelayAux)];
        s.levelDb = limit(-120.0f, 10.0f, value);
        s.preFader = false;
        s.enabled = s.levelDb > -120.0f;
        return true;
    }

    if (param == "hpf/enabled") { ch.hpfEnabled = on(value); return true; }
    if (param == "hpf/freq") { ch.hpfHz = limit(FilterPrimitives::MinFreqHz, FilterPrimitives::MaxFreqHz, value); ch.applyFilters(); return true; }
    if (param == "hpf/q")    { ch.hpfQ = limit(FilterPrimitives::MinQ, FilterPrimitives::MaxQ, value); ch.applyFilters(); return true; }
    if (param == "lpf/enabled") { ch.lpfEnabled = on(value); return true; }
    if (param == "lpf/freq") { ch.lpfHz = limit(FilterPrimitives::MinFreqHz, FilterPrimitives::MaxFreqHz, value); ch.applyFilters(); return true; }
    if (param == "lpf/q")    { ch.lpfQ = limit(FilterPrimitives::MinQ, FilterPrimitives::MaxQ, value); ch.applyFilters(); return true; }

    // eq/<band>/<field>, band 1-based on the wire like the channel.
    if (param.size() > 5 && param.substr(0, 3) == "eq/" && param[4] == '/') {
        const int band = param[3] - '1';
        if (band < 0 || band >= ParametricEQ::NumBands) return false;
        auto& s = ch.eqSettings[static_cast<size_t>(band)];
        const auto field = param.substr(5);
        if (field == "shape") {
            int code = 0;
            if (!wireInt(value, 0, 2, code)) return false;
            s.shape = static_cast<EqShape>(code);
        }
        else if (field == "freq")    s.frequencyHz = limit(FilterPrimitives::MinFreqHz, FilterPrimitives::MaxFreqHz, value);
        else if (field == "q")       s.q = limit(FilterPrimitives::MinQ, FilterPrimitives::MaxQ, value);
        else if (field == "gain")    s.gainDb = limit(FilterPrimitives::MinGainDb, FilterPrimitives::MaxGainDb, value);
        else if (field == "enabled") s.enabled = on(value);
        else return false;
        ch.applyEqBand(band);
        return true;
    }

    if (param == "comp/enabled")   { ch.comp.setEnabled(on(value)); return true; }
    if (param == "comp/threshold") { ch.comp.setThreshold(limit(-80.0f, 0.0f, value)); return true; }
    if (param == "comp/ratio")     { ch.comp.setRatio(limit(1.0f, 100.0f, value)); return true; }
    if (param == "comp/attack")    { ch.comp.setAttack(limit(0.1f, 500.0f, value)); return true; }
    if (param == "comp/release")   { ch.comp.setRelease(limit(1.0f, 5000.0f, value)); return true; }
    if (param == "comp/knee")      { ch.comp.setKnee(limit(0.0f, 24.0f, value)); return true; }
    if (param == "comp/makeup")    { ch.comp.setMakeup(limit(-24.0f, 24.0f, value)); return true; }
    if (param == "comp/detection") {
        ch.comp.setDetectionMode(on(value) ? Compressor::DetectionMode::RMS : Compressor::DetectionMode::Peak);
        return true;
    }

    if (param == "gate/enabled")    { ch.gate.setEnabled(on(value)); return true; }
    if (param == "gate/threshold")  { ch.gate.setThreshold(limit(-100.0f, 0.0f, value)); return true; }
    if (param == "gate/ratio")      { ch.gate.setRatio(limit(1.0f, 100.0f, value)); return true; }
    if (param == "gate/attack")     { ch.gate.setAttack(limit(0.01f, 500.0f, value)); return true; }
    if (param == "gate/hold")       { ch.gate.setHold(limit(0.0f, 2000.0f, value)); return true; }
    if (param == "gate/release")    { ch.gate.setRelease(limit(1.0f, 5000.0f, value)); return true; }
    if (param == "gate/range")      { ch.gate.setRange(limit(-100.0f, 0.0f, value)); return true; }
    if (param == "gate/hysteresis") { ch.gate.setHysteresis(limit(0.0f, 20.0f, value)); return true; }

    if (param == "deesser/enabled")   { ch.deesser.setEnabled(on(value)); return true; }
    if (param == "deesser/threshold") { ch.deesser.setThreshold(limit(-80.0f, 0.0f, value)); return true; }
    if (param == "deesser/ratio")     { ch.deesser.setRatio(limit(1.0f, 100.0f, value)); return true; }
    if (param == "deesser/freq")      { ch.deesser.setFrequency(value); return true; } // clamps itself

    return false;
}

// =====================================================================================
// The master bus — moved here from Main.cpp unchanged in meaning, so it can go through the
// queue and be tested.
// =====================================================================================
bool MixingEngine::setMasterParam(std::string_view p, float value) {
    if (p == "gain")             { master.gainDb = clampFaderDb(value); return true; }
    if (p == "mute")             { master.muted = on(value); return true; }
    if (p == "limiter/enabled")  { master.limiter.setEnabled(on(value)); return true; }
    if (p == "limiter/ceiling")  { master.limiter.setCeiling(value); return true; }
    if (p == "crossover/enabled")   { master.crossoverEnabled = on(value); return true; }
    if (p == "crossover/frequency") { master.crossover.setFrequency(limit(40.0f, 250.0f, value)); return true; }

    // The low end — see `LowEnd.h`. Channels are 1-based on the wire and 0 means none.
    if (p == "sidechain/enabled") { master.sidechainEnabled = on(value); return true; }
    if (p == "sidechain/depth")   { master.duck.setDepthDb(value); return true; }
    if (p == "sidechain/key" || p == "sidechain/target") {
        // Rounded, as it always was — but only once it is known to fit an int.
        const long n = (value > -1.0f && value < static_cast<float>(MaxChannels) + 1.0f) ? std::lround(value) : 0;
        const int index = (n >= 1 && n <= MaxChannels) ? static_cast<int>(n) - 1 : -1;
        (p == "sidechain/key" ? master.sidechainKey : master.sidechainTarget) = index;
        return true;
    }
    if (p == "monobass/enabled")   { master.monoBass.setEnabled(on(value)); return true; }
    if (p == "monobass/frequency") { master.monoBass.setFrequency(value); return true; }

    if (p.size() > 4 && p.substr(0, 4) == "geq/") {
        int band = 0;
        for (char c : p.substr(4)) {
            if (c < '0' || c > '9') return false;
            band = band * 10 + (c - '0');
            if (band > 99) return false;
        }
        band -= 1; // 1-based on the wire
        if (band < 0 || band >= GraphicEQ::NumBands) return false;
        master.geqL.setBandGain(band, value);
        master.geqR.setBandGain(band, value);
        return true;
    }

    auto& fx = fxBus;
    if (p == "fx/reverb/enabled")  { fx.setReverbEnabled(on(value)); return true; }
    if (p == "fx/reverb/room")     { fx.setReverbRoomSize(value); return true; }
    if (p == "fx/reverb/damping")  { fx.setReverbDamping(value); return true; }
    if (p == "fx/reverb/width")    { fx.setReverbWidth(value); return true; }
    if (p == "fx/reverb/wet")      { fx.setReverbWetLevel(value); return true; }
    if (p == "fx/delay/enabled")   { fx.setDelayEnabled(on(value)); return true; }
    if (p == "fx/delay/time")      { fx.setDelayMs(value); return true; }
    if (p == "fx/delay/feedback")  { fx.setDelayFeedback(value); return true; }
    if (p == "fx/delay/wet")       { fx.setDelayWetLevel(value); return true; }
    if (p == "fx/delay/pingpong")  { fx.setDelayPingPong(on(value)); return true; }
    if (p == "fx/delay/hpf")       { fx.setDelayHpf(value); return true; }
    if (p == "fx/delay/lpf")       { fx.setDelayLpf(value); return true; }
    return false;
}

bool MixingEngine::setControl(std::string_view address, float value) {
    if (!std::isfinite(value)) return false;
    if (!address.empty() && address.front() == '/') address.remove_prefix(1);

    if (address.substr(0, 8) == "channel/") {
        auto rest = address.substr(8);
        int n = 0;
        size_t i = 0;
        while (i < rest.size() && rest[i] >= '0' && rest[i] <= '9' && i < 3) {
            n = n * 10 + (rest[i] - '0');
            ++i;
        }
        if (i == 0 || i >= rest.size() || rest[i] != '/') return false;
        return setChannelParam(n - 1, rest.substr(i + 1), value); // 1-based on the wire
    }
    if (address.substr(0, 7) == "master/") return setMasterParam(address.substr(7), value);
    if (address.substr(0, 11) == "suppressor/") return setSuppressorParam(address.substr(11), value);
    // `dca/<n>/gain|mute` — a DCA fader, 1-based on the wire.
    if (address.substr(0, 4) == "dca/") {
        auto rest = address.substr(4);
        if (rest.size() < 3 || rest[0] < '1' || rest[0] > '8' || rest[1] != '/') return false;
        return setDcaParam(rest[0] - '1', rest.substr(2), value);
    }
    // `group/<n>/gain|mute` — a subgroup fader, 1-based on the wire.
    if (address.substr(0, 6) == "group/") {
        auto rest = address.substr(6);
        if (rest.size() < 3 || rest[0] < '1' || rest[0] > '8' || rest[1] != '/') return false;
        return setGroupParam(rest[0] - '1', rest.substr(2), value);
    }
    // `aux/<n>/<param>` — a monitor mix's own master section, 1-based on the wire.
    if (address.substr(0, 4) == "aux/") {
        auto rest = address.substr(4);
        if (rest.size() < 3 || rest[0] < '1' || rest[0] > '9' || rest[1] != '/') return false;
        return setMonitorParam(rest[0] - '1', rest.substr(2), value);
    }
    // `output/<n>/source <code>` — the output patch, 1-based on the wire.
    if (address.substr(0, 7) == "output/") {
        auto rest = address.substr(7);
        int n = 0;
        size_t i = 0;
        while (i < rest.size() && rest[i] >= '0' && rest[i] <= '9' && i < 3) { n = n * 10 + (rest[i] - '0'); ++i; }
        if (i == 0 || i >= rest.size() || rest[i] != '/') return false;
        if (n < 1 || n > MaxOutputs) return false;
        if (rest.substr(i) != "/source") return setOutputParam(n - 1, rest.substr(i + 1), value);
        int code = 0;
        if (!wireInt(value, 0, OutputSourceCount - 1, code)) return false;
        setOutputSource(n - 1, static_cast<OutputSource>(code));
        return true;
    }
    return false;
}

/**
 * The feedback suppressor's controls, through the queue like everything else.
 *
 * They were applied on the OSC (message) thread, straight into a suppressor the audio thread was
 * running and a detection thread was updating: a plain `bool` for bypass, read every block from
 * another thread, and `clearDynamicSlots` writing trackers the detection thread owned. Here they
 * run at the top of a block, on the audio thread, and are journalled — so a restart brings them
 * back by the same path as every other setting, not a special case beside it.
 */
bool MixingEngine::setSuppressorParam(std::string_view p, float value) {
    auto& s = master.suppressor;
    if (p == "bypass")      { s.setEnabled(! on(value)); return true; }
    if (p == "sensitivity") { s.setSensitivity(limit(0.0f, 1.0f, value)); return true; }
    if (p == "max-dynamic-notches") {
        // A count: a fraction or a number outside 1..8 is a malformed message, not a request to round.
        int n = 0;
        if (!wireInt(value, 1, FeedbackSuppressor::NumDynamicSlots, n)) return false;
        s.setMaxDynamicNotches(n);
        return true;
    }
    // An action, not a setting: `serviceControls` does not journal it.
    if (p == "clear-dynamic") { s.clearDynamicSlots(); return true; }
    return false;
}

bool MixingEngine::setDcaParam(int dca, std::string_view p, float value) {
    if (dca < 0 || dca >= DcaCount || !std::isfinite(value)) return false;
    auto& d = dcas[static_cast<size_t>(dca)];
    if (p == "gain") { d.gainDb = clampFaderDb(value); return true; }
    if (p == "mute") { d.muted = value >= 0.5f; return true; }
    return false;
}

float MixingEngine::dcaGainFor(int index) const {
    if (index < 0 || index >= MaxChannels) return 1.0f;
    const uint32_t mask = channels[static_cast<size_t>(index)].dcaMask;
    if (mask == 0) return 1.0f;
    float db = 0.0f;
    for (int d = 0; d < DcaCount; ++d) {
        if ((mask & (1u << d)) == 0) continue;
        if (dcas[static_cast<size_t>(d)].muted) return 0.0f;
        db += dcas[static_cast<size_t>(d)].gainDb;
    }
    // A channel in several DCAs takes the sum; −60 dB on the scale is where a fader is off.
    return db <= -60.0f ? 0.0f : juce::Decibels::decibelsToGain(db);
}

bool MixingEngine::setGroupParam(int group, std::string_view p, float value) {
    if (group < 0 || group >= GroupCount || !std::isfinite(value)) return false;
    auto& g = groups[static_cast<size_t>(group)];
    if (p == "gain") { g.gainDb = clampFaderDb(value); return true; }
    if (p == "mute") { g.muted = value >= 0.5f; return true; }
    return false;
}

float MixingEngine::groupGainFor(int index) const {
    if (index < 0 || index >= MaxChannels) return 1.0f;
    const int g = channels[static_cast<size_t>(index)].group;
    if (g <= 0 || g > GroupCount) return 1.0f;
    const auto& grp = groups[static_cast<size_t>(g - 1)];
    if (grp.muted || grp.gainDb <= -60.0f) return 0.0f;
    return juce::Decibels::decibelsToGain(grp.gainDb);
}

bool MixingEngine::setMonitorParam(int mix, std::string_view p, float value) {
    if (mix < 0 || mix >= MonitorAuxBuses || !std::isfinite(value)) return false;
    auto& m = monitors[static_cast<size_t>(mix)];
    const auto on = [](float v) { return v >= 0.5f; };
    const auto limit = [](float lo, float hi, float v) { return std::clamp(v, lo, hi); };
    if (p == "gain") { m.gainDb = clampFaderDb(value); return true; }
    if (p == "mute") { m.muted = on(value); return true; }
    if (p == "eq/low")  { m.lowDb = limit(-15.0f, 15.0f, value); m.applyEq(); return true; }
    if (p == "eq/mid")  { m.midDb = limit(-15.0f, 15.0f, value); m.applyEq(); return true; }
    if (p == "eq/high") { m.highDb = limit(-15.0f, 15.0f, value); m.applyEq(); return true; }
    if (p == "limiter/enabled") { m.limiterEnabled = on(value); return true; }
    if (p == "limiter/ceiling") { m.ceilingDb = limit(-24.0f, 0.0f, value); return true; }
    // An action: 1.5 s of 1 kHz into this mix's bus. See `TestTone`.
    if (p == "test") { monitorTone.start(sampleRate, mix, 1000.0f, 0.0316f, 1, 1.5, 1.5); return true; }
    return false;
}

void MixingEngine::setOutputSource(int output, OutputSource source) {
    if (output < 0 || output >= MaxOutputs) return;
    if (! outputPatched) {
        // The first patch message replaces the legacy routing with an explicit table that
        // starts empty — the desk sends every output, so nothing is left from the old map.
        outputSources.fill(OutputSource::None);
        outputPatched = true;
    }
    outputSources[static_cast<size_t>(output)] = source;
}

/**
 * `output/<n>/delay <ms>` — the alignment delay, which the speaker page sets in ms or metres —
 * and `output/<n>/speaker/<param>`, the rest of the speaker processor.
 */
bool MixingEngine::setOutputParam(int output, std::string_view p, float value) {
    if (output < 0 || output >= MaxOutputs || !std::isfinite(value)) return false;
    // An action: beep this jack. Any value; see `TestTone`.
    if (p == "test") {
        int count = 1;
        double onS = 0.0, periodS = 0.0;
        outputBeeps(output, count, onS, periodS);
        outputTone.start(sampleRate, output, testPitchFor(speakers[static_cast<size_t>(output)].settings()),
                         0.063f, count, onS, periodS);
        return true;
    }
    if (p == "delay") {
        const float ms = std::clamp(value, 0.0f, MaxOutputDelayMs);
        outputDelayMs[static_cast<size_t>(output)] = ms;
        outputDelays[static_cast<size_t>(output)].setDelayMs(ms);
        return true;
    }
    if (p.substr(0, 8) == "speaker/") return speakers[static_cast<size_t>(output)].setParam(p.substr(8), value);
    return false;
}

MixingEngine::OutputSource MixingEngine::getOutputSource(int output) const {
    if (output < 0 || output >= MaxOutputs) return OutputSource::None;
    return outputSources[static_cast<size_t>(output)];
}

bool MixingEngine::postControl(std::string_view address, float value) {
    if (address.size() > MaxControlAddress) return false;
    int start1, size1, start2, size2;
    controlFifo.prepareToWrite(1, start1, size1, start2, size2);
    if (size1 + size2 < 1) {
        dropped.fetch_add(1);
        return false;
    }
    auto& slot = controlQueue[static_cast<size_t>(size1 > 0 ? start1 : start2)];
    std::fill(std::begin(slot.address), std::end(slot.address), '\0');
    std::copy(address.begin(), address.end(), slot.address);
    slot.value = value;
    controlFifo.finishedWrite(1);
    return true;
}

void MixingEngine::serviceControls() {
    // One consumer at a time. The audio thread never waits: if `prepare` holds it, this block
    // simply runs on the settings it already has.
    const juce::SpinLock::ScopedTryLockType guard(consumerLock);
    if (!guard.isLocked()) return;

    int start1, size1, start2, size2;
    controlFifo.prepareToRead(controlFifo.getNumReady(), start1, size1, start2, size2);
    for (int i = 0; i < size1; ++i) {
        const auto& c = controlQueue[static_cast<size_t>(start1 + i)];
        applyQueued(c);
    }
    for (int i = 0; i < size2; ++i) {
        const auto& c = controlQueue[static_cast<size_t>(start2 + i)];
        applyQueued(c);
    }
    controlFifo.finishedRead(size1 + size2);
}

void MixingEngine::applyQueued(const ControlRecord& c) noexcept {
    std::string_view a(c.address);
    if (!setControl(a, c.value)) { rejected.fetch_add(1); return; }
    if (!a.empty() && a.front() == '/') a.remove_prefix(1);
    // An action is done, not kept: a restart must not clear the notches all over again.
    if (a == "suppressor/clear-dynamic") return;
    // Test tones are actions too: a restart must not start beeping a jack.
    if (a.size() >= 5 && a.substr(a.size() - 5) == "/test") return;
    journalApplied(c);
}

void MixingEngine::journalApplied(const ControlRecord& record) noexcept {
    // Single producer: `serviceControls` runs under `consumerLock`, so only one thread is ever
    // here. Nothing below allocates or waits.
    int start1, size1, start2, size2;
    appliedFifo.prepareToWrite(1, start1, size1, start2, size2);
    if (size1 + size2 < 1) {
        appliedLost.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    appliedQueue[static_cast<size_t>(size1 > 0 ? start1 : start2)] = record;
    appliedFifo.finishedWrite(1);
}

int MixingEngine::drainApplied(ControlRecord* out, int maxCount) {
    if (out == nullptr || maxCount <= 0) return 0;
    int start1, size1, start2, size2;
    appliedFifo.prepareToRead(std::min(maxCount, appliedFifo.getNumReady()), start1, size1, start2, size2);
    int n = 0;
    for (int i = 0; i < size1; ++i) out[n++] = appliedQueue[static_cast<size_t>(start1 + i)];
    for (int i = 0; i < size2; ++i) out[n++] = appliedQueue[static_cast<size_t>(start2 + i)];
    appliedFifo.finishedRead(size1 + size2);
    return n;
}

} // namespace dsp
