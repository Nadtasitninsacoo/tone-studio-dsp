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
    }
};

static DelayLineTests delayLineTests;
