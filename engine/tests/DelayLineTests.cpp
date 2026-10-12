#include "DelayLine.h"
#include <juce_core/juce_core.h>
#include <cmath>
#include "TestHelpers.h"

class DelayLineTests : public juce::UnitTest
{
public:
    DelayLineTests() : juce::UnitTest ("DelayLine") {}

    void runTest() override
    {
        beginTest ("Integer Delay Verification");
        
        dsp::DelayLine delay;
        delay.prepare(48000.0, 512);
        
        // 1 sample delay at 48kHz is: (1 / 48000) * 1000 = 0.020833 ms
        float oneSampleDelayMs = (1.0f / 48000.0f) * 1000.0f;
        delay.setDelayMs(oneSampleDelayMs);

        // Feed impulse
        float output = delay.processSample(1.0f);
        // Instant output is 0.0f (since it is delayed)
        expectWithinAbsoluteTolerance(output, 0.0f, 0.1f);
        
        // Feed silence, second sample should output the delayed 1.0f
        // Let it settle to handle delay smoothing
        float maxVal = 0.0f;
        for (int i = 0; i < 200; ++i) {
            output = delay.processSample(0.0f);
            maxVal = std::max(maxVal, std::abs(output));
        }
        
        // Expect delayed impulse to appear
        expectWithinAbsoluteTolerance(maxVal, 1.0f, 0.05f);

        /*
         * Under one sample of delay the interpolator must not read the slot after the write
         * position — the oldest audio in the ring. Fill the ring with +1, switch the signal to
         * −1, and at half a sample of delay every output after the first must be exactly −1:
         * the old four-point kernel put weight on that stale +1 and read ≈ −0.875.
         */
        beginTest ("A sub-sample delay never reads the stale end of the ring");
        dsp::DelayLine sub;
        sub.prepare(48000.0, 512);
        sub.setDelayMs(0.0f);
        for (int i = 0; i < 70000; ++i) sub.processSample(1.0f);
        sub.setDelayMs(0.5f * oneSampleDelayMs);
        float worst = 0.0f;
        for (int i = 0; i < 2000; ++i) {
            const float out = sub.processSample(-1.0f);
            if (i >= 2) worst = std::max(worst, std::abs(out + 1.0f));
        }
        expectLessThan(worst, 1.0e-6f, "a half-sample delay of a constant reads the constant");

        beginTest ("Zero delay is an exact pass-through");
        dsp::DelayLine zero;
        zero.prepare(48000.0, 512);
        zero.setDelayMs(0.0f);
        float zeroErr = 0.0f;
        for (int i = 0; i < 1000; ++i) {
            const float in = std::sin(0.05f * static_cast<float>(i));
            zeroErr = std::max(zeroErr, std::abs(zero.processSample(in) - in));
        }
        expectLessThan(zeroErr, 1.0e-7f, "delay 0 hands the input straight back");
    }
};

static DelayLineTests delayLineTests;
