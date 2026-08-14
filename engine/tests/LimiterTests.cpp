#include "Limiter.h"
#include <juce_core/juce_core.h>
#include <cmath>
#include "TestHelpers.h"

class LimiterTests : public juce::UnitTest
{
public:
    LimiterTests() : juce::UnitTest ("Limiter") {}

    void runTest() override
    {
        beginTest ("Brickwall Ceiling Test");
        
        dsp::Limiter limiter;
        limiter.prepare(48000.0, 512);
        limiter.setEnabled(true);
        limiter.setThreshold(0.0f);
        limiter.setCeiling(-1.0f); // -1.0 dBFS ceiling
        limiter.setLookahead(2.0f);
        limiter.setRelease(50.0f);

        // Input stereo samples at +6 dBFS (amplitude = 2.0f)
        int numSamples = 1000;
        std::vector<float> buffer(2 * numSamples, 2.0f); // Stereo interleaved

        // Process block
        limiter.processBlock(buffer.data(), numSamples);

        // Verify output peak does not exceed the ceiling (-1.0 dBFS approx 0.891f)
        float maxPeak = 0.0f;
        for (float val : buffer) {
            maxPeak = std::max(maxPeak, std::abs(val));
        }

        float expectedCeilingGain = juce::Decibels::decibelsToGain(-1.0f);
        // Expect output peak to be capped exactly at ceiling
        expectWithinAbsoluteTolerance(maxPeak, expectedCeilingGain, 0.05f);
    }
};

static LimiterTests limiterTests;
