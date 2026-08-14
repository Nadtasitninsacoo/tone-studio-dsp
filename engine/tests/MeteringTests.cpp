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
    }
};

static MeteringTests meteringTests;
