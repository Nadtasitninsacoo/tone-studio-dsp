#include "MixingEngine.h"
#include <juce_core/juce_core.h>
#include <cmath>
#include <limits>
#include <vector>

/**
 * The engine had no tests at all — every suite covered one module, and every bug found in the
 * top-level graph lived in the wiring *between* them:
 *
 *  - The master chain handed the feedback suppressor an interleaved buffer with a frame
 *    count, and the suppressor's mono overload reads a float count. Both modules were
 *    individually correct.
 *  - Twelve of the sixteen outputs were never written **and never cleared**, so they carried
 *    whatever the driver left in them.
 *  - A device with one output fell through both routing branches and got silence.
 *
 * None of those is visible from inside a module, which is what this file is for.
 */
class MixingEngineTests : public juce::UnitTest
{
public:
    MixingEngineTests() : juce::UnitTest ("MixingEngine") {}

    static constexpr double kSampleRate = 48000.0;
    static constexpr int kBlock = 256;

    /** A scratch set of output buffers, deliberately pre-filled with something audible. */
    struct Outputs
    {
        explicit Outputs (int count, int frames, float fill = 0.9f)
            : storage ((size_t) count, std::vector<float> ((size_t) frames, fill))
        {
            for (auto& v : storage) ptrs.push_back (v.data());
        }
        std::vector<std::vector<float>> storage;
        std::vector<float*> ptrs;
        float** data() { return ptrs.data(); }
    };

    static float rms (const float* p, int from, int to)
    {
        double sum = 0.0;
        for (int i = from; i < to; ++i) sum += (double) p[i] * p[i];
        const int n = to - from;
        return n > 0 ? (float) std::sqrt (sum / n) : 0.0f;
    }

    void runTest() override
    {
        beginTest ("Unused outputs are silenced, not left as driver garbage");
        {
            dsp::MixingEngine engine;
            engine.prepare (kSampleRate, kBlock);

            std::vector<float> in ((size_t) kBlock, 0.25f);
            const float* inPtrs[1] = { in.data() };

            // Sixteen outputs, all pre-filled loud. The engine writes four.
            Outputs out (16, kBlock, 0.9f);
            engine.processAudio (inPtrs, 1, out.data(), 16, kBlock);

            bool tailSilent = true;
            for (int o = 4; o < 16; ++o)
                for (int i = 0; i < kBlock; ++i)
                    if (out.storage[(size_t) o][(size_t) i] != 0.0f) tailSilent = false;

            expect (tailSilent,
                    "outputs 4..15 are silent rather than carrying the previous block");
        }

        beginTest ("A mono device gets audio, not silence");
        {
            // One output used to match neither `>= 4` nor `>= 2`, so nothing was written and
            // nothing said so — indistinguishable from a dead engine.
            dsp::MixingEngine engine;
            engine.prepare (kSampleRate, kBlock);

            std::vector<float> in ((size_t) kBlock);
            for (int i = 0; i < kBlock; ++i)
                in[(size_t) i] = 0.5f * (float) std::sin (2.0 * juce::MathConstants<double>::pi
                                                          * 440.0 * i / kSampleRate);
            const float* inPtrs[1] = { in.data() };

            Outputs out (1, kBlock, 0.0f);
            engine.processAudio (inPtrs, 1, out.data(), 1, kBlock);

            expect (rms (out.storage[0].data(), 0, kBlock) > 1.0e-4f,
                    "a single-output device is fed the mono fold-down");
        }

        beginTest ("A null input channel is skipped, not dereferenced");
        {
            // JUCE passes null for channels inside the reported count that the device's
            // active mask leaves off, and this engine asks for 32 inputs.
            dsp::MixingEngine engine;
            engine.prepare (kSampleRate, kBlock);

            std::vector<float> in ((size_t) kBlock, 0.3f);
            const float* inPtrs[4] = { in.data(), nullptr, nullptr, in.data() };

            Outputs out (2, kBlock, 0.0f);
            engine.processAudio (inPtrs, 4, out.data(), 2, kBlock);
            expect (true, "no crash on the audio thread");
        }

        beginTest ("A master notch actually notches the frequency it names");
        {
            /**
             * The call-site half of the interleaved bug, and the first attempt at this test
             * **passed with the bug in place** — worth recording, because the reason is the
             * whole point.
             *
             * The obvious assertion was "the back half of the block is attenuated like the
             * front", since the mono overload walks `numFrames` floats out of `2 * numFrames`
             * and leaves the rest. But the front half is not attenuated either: with the
             * channel panned centre, L and R are equal, so reading the interleaved buffer as
             * a mono stream is the same tone at twice the sample rate — an octave away from
             * where the filter is sitting. **The notch misses entirely rather than landing on
             * half the block**, so front and back come out equal and a ratio test sees
             * nothing wrong.
             *
             * So compare against a run with no notch at all. That is the claim anyway: a
             * suppressor that reports a notch at 1 kHz has to remove 1 kHz from the master
             * output. Driven through the engine rather than the module, because
             * `FeedbackSuppressorTests` already covers the implementation and this is about
             * the wiring between the two.
             */
            auto runEngine = [] (bool withNotch)
            {
                dsp::MixingEngine engine;
                engine.prepare (kSampleRate, kBlock);

                auto& suppressor = engine.getMaster().suppressor;
                suppressor.setEnabled (true);
                if (withNotch) suppressor.setFixedNotch (0, 1000.0f, 15.0f, -18.0f);

                std::vector<float> in ((size_t) kBlock);
                Outputs out (2, kBlock, 0.0f);
                double phase = 0.0;
                const double step = 2.0 * juce::MathConstants<double>::pi * 1000.0 / kSampleRate;

                // Settle the smoothed coefficients, the crossover and the channel dynamics.
                for (int block = 0; block < 80; ++block)
                {
                    for (int i = 0; i < kBlock; ++i)
                    {
                        in[(size_t) i] = 0.5f * (float) std::sin (phase);
                        phase += step;
                    }
                    const float* inPtrs[1] = { in.data() };
                    engine.processAudio (inPtrs, 1, out.data(), 2, kBlock);
                }
                return rms (out.storage[0].data(), 0, kBlock);
            };

            const float clean   = runEngine (false);
            const float notched = runEngine (true);

            expect (clean > 1.0e-4f, "the tone reaches the master output at all");
            expect (notched < clean * 0.5f,
                    "an 18 dB notch at 1 kHz removes most of a 1 kHz tone from the output");
        }

        beginTest ("The master meter measures the master output, not the mix before it");
        {
            /**
             * The meters were fed `mainBusL/R`, which the suppressor and limiter never
             * touch — they work in place on `masterInterleaved`. So the true peak was read
             * *before* the limiter, under a panel captioned MASTER OUTPUT.
             *
             * Drive the master hard into a low ceiling and read the meter back. Pre-limiter
             * it reports the overload; post-limiter it reports what the desk is sending.
             */
            dsp::MixingEngine engine;
            engine.prepare (kSampleRate, kBlock);
            engine.getMaster().limiter.setEnabled (true);
            engine.getMaster().limiter.setCeiling (-6.0f);
            engine.getChannel (0).faderDb = 12.0f;

            std::vector<float> in ((size_t) kBlock);
            Outputs out (2, kBlock, 0.0f);
            double phase = 0.0;
            const double step = 2.0 * juce::MathConstants<double>::pi * 220.0 / kSampleRate;

            for (int block = 0; block < 60; ++block)
            {
                for (int i = 0; i < kBlock; ++i)
                {
                    in[(size_t) i] = 0.9f * (float) std::sin (phase);
                    phase += step;
                }
                const float* inPtrs[1] = { in.data() };
                engine.processAudio (inPtrs, 1, out.data(), 2, kBlock);
            }

            const float peakL = engine.getMaster().metering.getTruePeakDbL();
            expect (peakL > -60.0f, "the meter is reading something at all");
            // A ceiling of -6 dBFS plus inter-sample allowance. Pre-limiter this signal is
            // around +11 dBFS, so the discriminator is wide.
            expect (peakL < -2.0f,
                    "the reading respects the limiter the signal has already been through");
        }

        beginTest ("The RTA shows the master with its notches applied");
        {
            /**
             * The other half of the same tap, and the more expensive one to have got wrong:
             * a suppressor holding a howl down would have left the peak standing on the
             * analyser — on the one screen whose whole job is to show what it did.
             */
            auto rtaPeakDb = [] (bool withNotch)
            {
                dsp::MixingEngine engine;
                engine.prepare (kSampleRate, kBlock);
                auto& suppressor = engine.getMaster().suppressor;
                suppressor.setEnabled (true);
                if (withNotch) suppressor.setFixedNotch (0, 1000.0f, 15.0f, -18.0f);

                std::vector<float> in ((size_t) kBlock);
                Outputs out (2, kBlock, 0.0f);
                double phase = 0.0;
                const double step = 2.0 * juce::MathConstants<double>::pi * 1000.0 / kSampleRate;

                for (int block = 0; block < 80; ++block)
                {
                    for (int i = 0; i < kBlock; ++i)
                    {
                        in[(size_t) i] = 0.5f * (float) std::sin (phase);
                        phase += step;
                    }
                    const float* inPtrs[1] = { in.data() };
                    engine.processAudio (inPtrs, 1, out.data(), 2, kBlock);
                }

                const auto bands = engine.getMaster().metering.getRtaBains();
                float top = -200.0f;
                for (const auto b : bands) top = std::max (top, b);
                return top;
            };

            const float clean   = rtaPeakDb (false);
            const float notched = rtaPeakDb (true);

            expect (clean > -119.0f, "the RTA sees the tone");
            expect (notched < clean - 3.0f,
                    "and sees it reduced once the suppressor is holding that frequency down");
        }

        beginTest ("Fader and trim are bounded, and non-finite is rejected");
        {
            using ME = dsp::MixingEngine;

            expect (ME::clampFaderDb (1000.0f) == ME::MaxFaderDb, "a huge fader clamps");
            expect (ME::clampFaderDb (-1000.0f) == ME::MinFaderDb, "and a tiny one");
            expect (ME::clampTrimDb (1000.0f) == ME::MaxTrimDb, "same for trim");
            expect (ME::clampTrimDb (-1000.0f) == ME::MinTrimDb);
            expect (ME::clampFaderDb (0.0f) == 0.0f, "unity passes through untouched");
            expect (ME::clampTrimDb (-6.0f) == -6.0f, "and so does an ordinary value");

            /**
             * Rejected to unity rather than clamped. `std::clamp(NaN, lo, hi)` is NaN, and
             * NaN reaching a gain multiply silences the bus while every reading on screen
             * still looks correct — the correction `setMasterVolume` needed in the web app.
             */
            expect (ME::clampFaderDb (std::numeric_limits<float>::quiet_NaN()) == 0.0f);
            expect (ME::clampTrimDb (std::numeric_limits<float>::quiet_NaN()) == 0.0f);
            expect (ME::clampFaderDb (std::numeric_limits<float>::infinity()) == 0.0f);
            expect (ME::clampTrimDb (-std::numeric_limits<float>::infinity()) == 0.0f);

            expect (ME::MaxFaderDb < 24.0f,
                    "the ceiling is console travel, not something that can damage a driver");
        }

        beginTest ("An absurd fader written past the control plane still cannot run away");
        {
            /**
             * The clamp lives at the OSC boundary *and* where the gain is computed. This
             * writes the field directly — the boundary bypassed, exactly as a future setter
             * that forgets would — and asserts the output is still bounded.
             *
             * It matters because the master limiter can be switched off over the same
             * control plane, so nothing downstream would catch it.
             */
            dsp::MixingEngine engine;
            engine.prepare (kSampleRate, kBlock);
            engine.getMaster().limiter.setEnabled (false);
            engine.getChannel (0).faderDb = 400.0f;
            engine.getChannel (0).trimDb = 400.0f;

            std::vector<float> in ((size_t) kBlock, 0.5f);
            const float* inPtrs[1] = { in.data() };
            Outputs out (2, kBlock, 0.0f);
            engine.processAudio (inPtrs, 1, out.data(), 2, kBlock);

            float peak = 0.0f;
            for (int o = 0; o < 2; ++o)
                for (int i = 0; i < kBlock; ++i)
                    peak = std::max (peak, std::abs (out.storage[(size_t) o][(size_t) i]));

            // +12 fader and +24 trim on a 0.5 input is about 8.9 linear. Unclamped it would
            // be 10^40. The assertion is that it is a *number*, and a bounded one.
            expect (std::isfinite (peak), "the output is finite");
            expect (peak < 32.0f, "and within the clamped headroom rather than 1e40");
        }

        beginTest ("The master fader and mute reach the audio");
        {
            /**
             * They reached nothing at all. `bridge.js` translated `masterGain`/`masterMute`
             * into `/master/gain` and `/master/mute` and `Main.cpp` dropped both, so the web
             * app's master fader moved and reported success while changing no sound —
             * a control that does nothing, spread across two repositories.
             */
            auto runAt = [] (float masterDb, bool muted)
            {
                dsp::MixingEngine engine;
                engine.prepare (kSampleRate, kBlock);
                engine.getMaster().limiter.setEnabled (false);
                engine.getMaster().gainDb = masterDb;
                engine.getMaster().muted = muted;

                std::vector<float> in ((size_t) kBlock);
                Outputs out (2, kBlock, 0.0f);
                double phase = 0.0;
                const double step = 2.0 * juce::MathConstants<double>::pi * 440.0 / kSampleRate;
                // Several blocks so the ramp has arrived — the fader is smoothed.
                for (int block = 0; block < 20; ++block)
                {
                    for (int i = 0; i < kBlock; ++i)
                    {
                        in[(size_t) i] = 0.4f * (float) std::sin (phase);
                        phase += step;
                    }
                    const float* inPtrs[1] = { in.data() };
                    engine.processAudio (inPtrs, 1, out.data(), 2, kBlock);
                }
                return rms (out.storage[0].data(), 0, kBlock);
            };

            const float unity = runAt (0.0f, false);
            const float down  = runAt (-20.0f, false);
            const float mute  = runAt (0.0f, true);

            expect (unity > 1.0e-4f, "there is a signal at unity");
            expect (down < unity * 0.3f, "pulling the master down makes it quieter");
            expect (mute < 1.0e-6f, "and mute silences it");
        }

        beginTest ("A fader move is ramped, not stepped");
        {
            /**
             * `decibelsToGain` was called once per block and applied as a constant, so a
             * fader move landed as a step at a block boundary — a click per move, and a burst
             * of them while somebody rides a fader. `Panner` was already smoothed; trim and
             * fader were the odd ones out, which is what made it a bug rather than a design.
             *
             * **Two earlier versions of this test were wrong, and both are worth recording.**
             *
             * The first asserted the move had arrived within one block. The ramp is 20 ms —
             * 960 samples at 48 kHz — and the block was 256, so that was asserting the ramp
             * did *not* happen.
             *
             * The second used a 1024-sample block and compared the energy in its first
             * quarter against its last, expecting a step to make the two equal. **It passed
             * with the ramp removed.** Printing the profile in eighths showed why:
             * `0.0498 0.0151 0.0082 0.0019 0.0021 …` — a decay to the new level over about
             * 400 samples that was there either way, because a step in level is smeared by
             * the master GEQ and the Linkwitz-Riley crossover over the same timescale as the
             * ramp. The audio at the output physically cannot distinguish the two.
             *
             * So this reads the mechanism. After one 256-sample block of a 960-sample ramp
             * the gain must be strictly *between* the old value and the new one: still at the
             * old one means the smoother is never advanced, already at the new one means it
             * was set rather than ramped, and both of those are the bug.
             */
            dsp::MixingEngine engine;
            engine.prepare (kSampleRate, kBlock);
            engine.getChannel (0).faderDb = 0.0f;

            std::vector<float> in ((size_t) kBlock, 0.3f);
            const float* inPtrs[1] = { in.data() };
            Outputs out (2, kBlock, 0.0f);

            engine.processAudio (inPtrs, 1, out.data(), 2, kBlock);
            expect (std::abs (engine.getChannel (0).currentFaderGain() - 1.0f) < 1.0e-4f,
                    "unity to start with");

            engine.getChannel (0).faderDb = -40.0f;
            engine.processAudio (inPtrs, 1, out.data(), 2, kBlock);

            const float mid = engine.getChannel (0).currentFaderGain();
            const float target = juce::Decibels::decibelsToGain (-40.0f);

            expect (mid < 1.0f - 1.0e-4f,
                    "the ramp started — it is not still sitting at the old gain");
            expect (mid > target + 1.0e-4f,
                    "and it did not jump straight to the new one: that jump is the click");

            for (int block = 0; block < 5; ++block)
                engine.processAudio (inPtrs, 1, out.data(), 2, kBlock);
            expect (std::abs (engine.getChannel (0).currentFaderGain() - target) < 1.0e-4f,
                    "a ramp is a few milliseconds, not a fader that never gets there");
        }

        beginTest ("A block longer than prepare() was told about leaves no garbage");
        {
            // `activeSamples` clamps what gets processed. It must not leave the tail of the
            // output buffer as whatever was in it.
            dsp::MixingEngine engine;
            engine.prepare (kSampleRate, 64);

            std::vector<float> in ((size_t) kBlock, 0.2f);
            const float* inPtrs[1] = { in.data() };

            Outputs out (2, kBlock, 0.7f);
            engine.processAudio (inPtrs, 1, out.data(), 2, kBlock);

            bool tailClear = true;
            for (int o = 0; o < 2; ++o)
                for (int i = 64; i < kBlock; ++i)
                    if (out.storage[(size_t) o][(size_t) i] != 0.0f) tailClear = false;

            expect (tailClear, "the unprocessed tail is silence, not the previous contents");
        }
    }
};

static MixingEngineTests mixingEngineTests;
