#include "MixingEngine.h"
#include <algorithm>
#include <array>
#include <cmath>

namespace dsp {

MixingEngine::MixingEngine() : controlQueue(static_cast<size_t>(ControlQueueSize)) {
    // Identity patch: channel n hears input n until the desk says otherwise.
    convolutionQueue = std::make_unique<juce::dsp::ConvolutionMessageQueue>();
    for (int i = 0; i < MaxChannels; ++i) {
        channels[static_cast<size_t>(i)].inputIndex = i;
        channels[static_cast<size_t>(i)].amp = std::make_unique<GuitarAmp>(*convolutionQueue);
    }
    reset();
}

void MixingEngine::serviceBackgroundWork() {
    for (auto& chan : channels) {
        if (chan.amp) chan.amp->serviceBackground();
    }
}

void MixingEngine::prepare(double newSampleRate, int newMaxBlockSize) {
    sampleRate = newSampleRate;
    maxBlockSize = newMaxBlockSize;

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
    }

    // Prepare send returns
    fxBus.prepare(sampleRate, maxBlockSize);

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
    }
    fxBus.reset();
}

void MixingEngine::processAudio(
    const float** inputs, 
    int channelsCount, 
    float** outputs, 
    int outputsCount, 
    int numSamples
) {
    // Controls first, so the whole block is processed with one set of settings.
    serviceControls();

    // Safety clamp
    int activeSamples = std::min(numSamples, maxBlockSize);
    int activeInChannels = std::min(channelsCount, MaxChannels);
    int activeOutChannels = std::min(outputsCount, MaxOutputs);

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
        chan.faderGain.setTargetValue(juce::Decibels::decibelsToGain(clampFaderDb(chan.faderDb)));

        /*
         * Two passes, because the rack is a block process (oversampling, convolution) sitting
         * in the middle of a per-sample strip: trim → Ø → HPF → gate → EQ into `postDspBuffer`,
         * then the rack on that block, then the dynamics, fader, sends and pan. Each smoother
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

            x = chan.gate.processSample(x);
            postDspBuffer[i] = chan.eq.processSample(x);
        }

        if (chan.amp && chan.amp->isEnabled()) chan.amp->process(postDspBuffer.data(), activeSamples);

        for (int i = 0; i < activeSamples; ++i) {
            const float faderGain = chan.faderGain.getNextValue();
            float x = postDspBuffer[i];

            // Dynamics after the rack: De-esser, Compressor
            x = chan.deesser.processSample(x);
            x = chan.comp.processSample(x);

            // Low pass filter
            if (chan.lpfEnabled) {
                x = chan.lpf.processSample(x);
            }

            // The sidechain duck, before the fader — where the web desk puts it.
            if (duckActive && c == master.sidechainTarget) x *= master.duck.gainAt(i);

            postDspBuffer[i] = x;
            
            // Fader gain
            float postFader = x * faderGain;
            postFaderBuffer[i] = postFader;

            // Route to Aux/FX sends
            for (int b = 0; b < MaxAuxBuses; ++b) {
                if (chan.sends[b].enabled) {
                    float sendGain = juce::Decibels::decibelsToGain(chan.sends[b].levelDb);
                    float sendSig = chan.sends[b].preFader ? x : postFader;
                    auxBuses[b][i] += sendSig * sendGain;
                }
            }

            // Route to Main stereo bus
            if (chan.routedToMain) {
                float outL = 0.0f;
                float outR = 0.0f;
                chan.panner.processSample(postFader, outL, outR);
                mainBusL[i] += outL;
                mainBusR[i] += outR;
            }
        }

        // Process channel metering
        float maxGrDb = chan.comp.getGainReductionDb() + chan.gate.getGainReductionDb() + chan.deesser.getGainReductionDb();
        chan.metering.processBlock(postFaderBuffer.data(), activeSamples, maxGrDb);

        if (duckActive && c == master.sidechainKey) {
            master.duck.processKey(postFaderBuffer.data(), activeSamples);
            keyDone = true;
        }
    }

    // 3. Process Send/Return FX Bus (Reverb on aux 6, Delay on aux 7)
    // Each effect hears its own send — see `ReverbDelay::processSends`.
    float* fxReturns[2] = { fxReturnL.data(), fxReturnR.data() };
    fxBus.processSends(auxBuses[6].data(), auxBuses[7].data(), fxReturns, activeSamples);

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
     * Filling them is the callback's job. This engine asks for 16 outputs and only ever wrote
     * four, so outputs 4..15 carried undefined content **straight to a PA**, and a device
     * with one output got nothing written at all while the code fell through both branches
     * in silence.
     *
     * `numSamples` rather than `activeSamples`: if a host ever hands over a longer block than
     * `prepare` was told about, the clamp above stops us *processing* the tail — it must not
     * leave that tail as garbage on the way out.
     */
    for (int o = 0; o < activeOutChannels; ++o) {
        if (outputs[o] != nullptr) {
            std::fill(outputs[o], outputs[o] + numSamples, 0.0f);
        }
    }

    /*
     * Outputs 0/1 are the full-range master unless the sub/main split has been switched on.
     * See `MasterBus::crossoverEnabled` for the bug this replaced: with the split always on, a
     * stereo device that Windows opened as eight channels carried the sub band on the pair
     * anybody was listening to, and the programme on outputs nothing was plugged into.
     */
    if (master.crossoverEnabled && activeOutChannels >= 4) {
        std::copy(mainL.begin(), mainL.begin() + activeSamples, outputs[0]);
        std::copy(mainR.begin(), mainR.begin() + activeSamples, outputs[1]);
        std::copy(subL.begin(), subL.begin() + activeSamples, outputs[2]);
        std::copy(subR.begin(), subR.begin() + activeSamples, outputs[3]);
        for (int o = 0; o < 4; ++o) outputDelays[o].processBlock(outputs[o], activeSamples);
    } else if (activeOutChannels >= 2) {
        // The master as it left the limiter — not sub + main re-summed, which is the same
        // signal only while the crossover is a perfect all-pass and costs two filters to prove.
        std::copy(mainBusL.begin(), mainBusL.begin() + activeSamples, outputs[0]);
        std::copy(mainBusR.begin(), mainBusR.begin() + activeSamples, outputs[1]);
        outputDelays[0].processBlock(outputs[0], activeSamples);
        outputDelays[1].processBlock(outputs[1], activeSamples);
    } else if (activeOutChannels == 1) {
        // A mono device gets the fold-down. Halved: L and R are correlated, and the limiter
        // has already run, so nothing downstream would catch +6 dB.
        for (int i = 0; i < activeSamples; ++i) {
            outputs[0][i] = 0.5f * (mainBusL[i] + mainBusR[i]);
        }
        outputDelays[0].processBlock(outputs[0], activeSamples);
    }
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
} // namespace

bool MixingEngine::setChannelParam(int index, std::string_view param, float value) {
    if (index < 0 || index >= MaxChannels) return false;
    if (!std::isfinite(value)) return false;
    auto& ch = channels[static_cast<size_t>(index)];

    // The guitar rack. Its own table, allocation-free; an impulse response it needs is built
    // later on the message thread (`serviceBackgroundWork`).
    if (param.substr(0, 4) == "amp/") return ch.amp && ch.amp->setParam(param.substr(4), value);

    if (param == "fader") { ch.faderDb = clampFaderDb(value); return true; }
    if (param == "trim")  { ch.trimDb = clampTrimDb(value); return true; }
    if (param == "pan")   { ch.panner.setPan(value); return true; }
    // The engine has no `muted` flag; unrouting from the main bus is the only mute it has.
    if (param == "mute")  { ch.routedToMain = !on(value); return true; }
    if (param == "invert") { ch.phaseInvert = on(value); return true; }
    // The input is 0-based (unlike the channel in the address) and -1 means none. A fraction
    // is a malformed patch, not a request to round.
    if (param == "input") {
        const int in = static_cast<int>(value);
        if (static_cast<float>(in) != value || in < NoInput || in >= MaxChannels) return false;
        ch.inputIndex = in;
        return true;
    }

    // Post-fader sends into the two FX returns — aux 6 is the reverb, aux 7 the delay (see
    // `processBlock`). Until these existed nothing could switch a send on, so the Master FX
    // page's reverb and delay were fed silence whatever they were set to. The value is the send
    // level in dB; at the floor the send is off rather than summing zeros.
    if (param == "send/reverb" || param == "send/delay") {
        auto& s = ch.sends[param == "send/reverb" ? 6u : 7u];
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
            const int code = static_cast<int>(value);
            if (static_cast<float>(code) != value || code < 0 || code > 2) return false;
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
        const int n = static_cast<int>(std::lround(value));
        const int index = (n >= 1 && n <= MaxChannels) ? n - 1 : -1;
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
    return false;
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
        if (!setControl(std::string_view(c.address), c.value)) rejected.fetch_add(1);
    }
    for (int i = 0; i < size2; ++i) {
        const auto& c = controlQueue[static_cast<size_t>(start2 + i)];
        if (!setControl(std::string_view(c.address), c.value)) rejected.fetch_add(1);
    }
    controlFifo.finishedRead(size1 + size2);
}

} // namespace dsp
