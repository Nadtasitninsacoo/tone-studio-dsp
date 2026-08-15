#include "Metering.h"
#include <juce_core/juce_core.h>
#include "TestHelpers.h"

class MeteringTests : public juce::UnitTest
{
public:
    MeteringTests() : juce::UnitTest ("Metering") {}

    void runTest() override
    {
        beginTest ("Channel Metering Peaks");
        
        dsp::ChannelMetering chMeter;
        chMeter.prepare(48000.0);

        // Feed block of constant level (0.5f = -6 dBFS)
        int numSamples = 512;
        std::vector<float> buffer(numSamples, 0.5f);

        chMeter.processBlock(buffer.data(), numSamples, 0.0f);

        // Peak has an instant attack, so one block is enough.
        expectWithinAbsoluteTolerance(chMeter.getPeakDb(), -6.02f, 0.5f);

        /**
         * RMS integrates over 50 ms, and one 512-sample block at 48 kHz is 10.7 ms — so
         * after a single block it *must* read low. That is what an integrating meter is.
         * The original assertion expected the settled value immediately, which no correct
         * implementation can satisfy; it would only ever have passed against a meter that
         * did not integrate at all.
         *
         * This is a change to the test, not to the DSP, and the distinction is worth
         * recording: the first failure here (-39.8 dB) was a genuine bug — a per-sample
         * coefficient applied once per block — and fixing it revealed that the expectation
         * underneath was wrong as well.
         */
        const int blocksToSettle = static_cast<int>((0.5 * 48000.0) / numSamples);
        for (int i = 0; i < blocksToSettle; ++i)
            chMeter.processBlock(buffer.data(), numSamples, 0.0f);

        expectWithinAbsoluteTolerance(chMeter.getRmsDb(), -6.02f, 0.5f);

        beginTest("Master true peak releases in the time it says, not the block size times it");
        /**
         * `MasterMetering` still had the exact defect `ChannelMetering` was fixed for, in the
         * other half of the same file: `tpReleaseCoef` was `exp(-1 / (fs * 0.5))` — a
         * per-*sample* coefficient for a 500 ms release — applied once per **block**. At 48
         * kHz with 480-sample blocks that is a factor of 480: half a second becomes about
         * four minutes.
         *
         * It was found by running the engine with nothing plugged in and watching the
         * bridge: `masterL` sat at −51 dBFS and fell by 0.00006 over six seconds, on a desk
         * with no input. A meter that goes on reporting a signal minutes after it stopped is
         * the frozen-meter failure `STALE_MS` exists to prevent, arriving from inside the
         * engine rather than off the wire.
         */
        {
            dsp::MasterMetering master;
            const int block = 480;              // what this machine's device actually asks for
            master.prepare(48000.0, block);

            std::vector<float> loud((size_t) block, 0.5f);
            std::vector<float> quiet((size_t) block, 0.0f);
            const float* loudPtrs[2] = { loud.data(), loud.data() };
            const float* quietPtrs[2] = { quiet.data(), quiet.data() };

            master.processBlock(loudPtrs, block, 0.0f);
            expect(master.getTruePeakDbL() > -7.0f, "it reads the peak straight away");

            // 500 ms of silence is one release time constant, so ~-8.7 dB of decay. Two
            // seconds is four of them and must be well down.
            const int blocksInTwoSeconds = static_cast<int>(2.0 * 48000.0 / block);
            for (int i = 0; i < blocksInTwoSeconds; ++i)
                master.processBlock(quietPtrs, block, 0.0f);

            expect(master.getTruePeakDbL() < -30.0f,
                   "and two seconds of silence takes it down, rather than four minutes");

            // Under the bug it had barely moved. State that as its own assertion so a
            // regression names the right thing.
            expect(master.getTruePeakDbL() < -20.0f,
                   "a 500 ms release is not multiplied by the block size");
        }

        beginTest("True peak reads between the samples, which is what makes it a true peak");
        /**
         * **`truePeakDb*` was a sample peak for the whole life of this engine.**
         *
         * `peakL = std::max(peakL, std::abs(xl))` at base rate, under that name, behind
         * `getTruePeakDbL()`, published as `/meter/master`, and described in
         * `MixingEngine.cpp` as "Stereo True Peak lookahead". Its own comment claimed it
         * "uses oversampled peaks from the limiter, or calculates here" and did neither.
         *
         * The signal below is the textbook case, and it is one line of trigonometry away
         * from ordinary full-scale material rather than a corner case. A sine at exactly
         * fs/4 offset by 45° is sampled only at ±sin(45°) = ±0.7071, so **every sample sits
         * at −3.01 dBFS while the waveform between them reaches 0 dBFS.** A converter
         * reconstructs the curve, not the dots: this clips a real output while a sample-peak
         * meter reports 3 dB of headroom. 3 dB is also the most a two-point interpolation
         * can hide, which makes it the sharpest available test.
         */
        {
            dsp::MasterMetering master;
            const int block = 480;
            master.prepare(48000.0, block);

            std::vector<float> isp((size_t) block);
            for (int i = 0; i < block; ++i)
                isp[(size_t) i] = std::sin(juce::MathConstants<float>::halfPi * (float) i
                                           + juce::MathConstants<float>::pi * 0.25f);

            // The premise, asserted rather than assumed: every sample really is at -3 dBFS.
            float samplePeak = 0.0f;
            for (float v : isp) samplePeak = std::max(samplePeak, std::abs(v));
            expectWithinAbsoluteTolerance(
                juce::Decibels::gainToDecibels(samplePeak), -3.01f, 0.05f);

            const float* ptrs[2] = { isp.data(), isp.data() };
            // Several blocks: the interpolator is an FIR with latency, so the first block is
            // partly its fade-in. A meter that needed exactly one block would be measuring
            // the filter rather than the signal.
            for (int i = 0; i < 5; ++i) master.processBlock(ptrs, block, 0.0f);

            expect(master.getTruePeakDbL() > -1.0f,
                   juce::String("the peak between the samples is found, not just the ones on "
                                "them — got ")
                       + juce::String(master.getTruePeakDbL()) + " dBFS");

            // The old implementation returned the sample peak exactly. Name that number so a
            // regression to it fails with the reason already written in the message.
            expect(master.getTruePeakDbL() > -2.5f,
                   "a reading near -3.01 dBFS means it is back to measuring samples");
        }

        beginTest("And it invents no peaks on a signal that has none between its samples");
        /**
         * The other half, and the one that catches a broken interpolator.
         *
         * A meter that simply read high — a filter with gain, a window scanned twice, an
         * envelope that never falls — would pass the test above for entirely the wrong
         * reason. A low frequency is oversampled almost exactly, so true peak and sample
         * peak agree to a fraction of a dB and an overshooting measurement has nothing to
         * hide behind.
         */
        {
            dsp::MasterMetering master;
            const int block = 480;
            master.prepare(48000.0, block);

            std::vector<float> tone((size_t) block);
            for (int i = 0; i < block; ++i)
                tone[(size_t) i] = 0.5f // -6.02 dBFS
                                 * std::sin(juce::MathConstants<float>::twoPi * 100.0f
                                            * (float) i / 48000.0f);

            const float* ptrs[2] = { tone.data(), tone.data() };
            for (int i = 0; i < 20; ++i) master.processBlock(ptrs, block, 0.0f);

            expectWithinAbsoluteTolerance(master.getTruePeakDbL(), -6.02f, 0.3f);
        }
    }
};

static MeteringTests meteringTests;
