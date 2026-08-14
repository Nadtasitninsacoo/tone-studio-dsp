#include "MixingEngine.h"
#include <algorithm>

namespace dsp {

MixingEngine::MixingEngine() {
    reset();
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
    for (int c = 0; c < activeInChannels; ++c) {
        auto& chan = channels[c];
        const float* chanIn = inputs[c];
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

        for (int i = 0; i < activeSamples; ++i) {
            // Advanced every sample, and both of them every sample even when only one is
            // moving: a smoother stepped on some paths and not others falls behind the block
            // clock and arrives late on the next move.
            const float trimGain = chan.trimGain.getNextValue();
            const float faderGain = chan.faderGain.getNextValue();

            // Trim and phase invert
            float x = chanIn[i] * trimGain;
            if (chan.phaseInvert) {
                x = -x;
            }

            // High pass filter
            if (chan.hpfEnabled) {
                x = chan.hpf.processSample(x);
            }

            // Dynamics: Gate, EQ, De-esser, Compressor
            x = chan.gate.processSample(x);
            x = chan.eq.processSample(x);
            x = chan.deesser.processSample(x);
            x = chan.comp.processSample(x);

            // Low pass filter
            if (chan.lpfEnabled) {
                x = chan.lpf.processSample(x);
            }

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
    }

    // 3. Process Send/Return FX Bus (Reverb on aux 6, Delay on aux 7)
    const float* fxSends[2] = { auxBuses[6].data(), auxBuses[7].data() };
    float* fxReturns[2] = { fxReturnL.data(), fxReturnR.data() };
    
    fxBus.processBlock(fxSends, fxReturns, activeSamples);

    // Sum FX returns to Master bus
    for (int i = 0; i < activeSamples; ++i) {
        mainBusL[i] += fxReturnL[i];
        mainBusR[i] += fxReturnR[i];
    }

    // 4. Master processing (interleaved stereo processing)
    // Left & Right GEQ
    master.geqL.processBlock(mainBusL.data(), activeSamples);
    master.geqR.processBlock(mainBusR.data(), activeSamples);

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
    // Typically:
    // Out 0, 1 = Sub L/R (outputs of crossover sub)
    // Out 2, 3 = Main L/R (outputs of crossover main)
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

    // Apply Output alignment Delay per physical output
    // Out 0: Sub Left
    // Out 1: Sub Right
    // Out 2: Main Left
    // Out 3: Main Right
    if (activeOutChannels >= 4) {
        std::copy(subL.begin(), subL.begin() + activeSamples, outputs[0]);
        std::copy(subR.begin(), subR.begin() + activeSamples, outputs[1]);
        std::copy(mainL.begin(), mainL.begin() + activeSamples, outputs[2]);
        std::copy(mainR.begin(), mainR.begin() + activeSamples, outputs[3]);

        outputDelays[0].processBlock(outputs[0], activeSamples);
        outputDelays[1].processBlock(outputs[1], activeSamples);
        outputDelays[2].processBlock(outputs[2], activeSamples);
        outputDelays[3].processBlock(outputs[3], activeSamples);
    } else if (activeOutChannels >= 2) {
        // Fallback for stereo output systems: mix sub and main back together
        for (int i = 0; i < activeSamples; ++i) {
            outputs[0][i] = subL[i] + mainL[i];
            outputs[1][i] = subR[i] + mainR[i];
        }
        outputDelays[0].processBlock(outputs[0], activeSamples);
        outputDelays[1].processBlock(outputs[1], activeSamples);
    } else if (activeOutChannels == 1) {
        /**
         * A mono device used to fall through both branches and get silence, which is
         * indistinguishable from a broken engine and is what a laptop's own speaker looks
         * like while somebody is setting up.
         *
         * Halved, because this is a fold-down of two bands *and* two sides: summing four
         * correlated signals at unity is +12 dB in the worst case, and the limiter has
         * already run by this point, so nothing downstream would catch it.
         */
        for (int i = 0; i < activeSamples; ++i) {
            outputs[0][i] = 0.5f * (subL[i] + subR[i] + mainL[i] + mainR[i]);
        }
        outputDelays[0].processBlock(outputs[0], activeSamples);
    }
}

} // namespace dsp
