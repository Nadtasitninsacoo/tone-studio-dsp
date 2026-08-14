#include "ParametricEQ.h"
#include <juce_core/juce_core.h>
#include <cmath>
#include "TestHelpers.h"

class ParametricEQTests : public juce::UnitTest
{
public:
    ParametricEQTests() : juce::UnitTest ("ParametricEQ") {}

    void runTest() override
    {
        beginTest ("EQ Peaking Band Gain Test");
        
        dsp::ParametricEQ eq;
        eq.prepare(48000.0, 512);

        // Configure band 0: Peaking, 1000Hz, Q=1.0, Gain=+6dB
        dsp::ParametricEQ::BandConfig config;
        config.type = dsp::FilterPrimitives::Type::Peaking;
        config.frequencyHz = 1000.0f;
        config.q = 1.0f;
        config.gainDb = 6.0f;
        config.enabled = true;
        
        eq.setBandParameters(0, config);

        // Test with 1000Hz sine wave (center frequency)
        float freqHz = 1000.0f;
        float inputAmp = 0.5f;
        float outputMax = 0.0f;
        
        // Let it settle to bypass initial smoothing
        for (int i = 0; i < 2000; ++i)
        {
            float time = static_cast<float>(i) / 48000.0f;
            float sineSample = inputAmp * std::sin(2.0f * 3.14159265f * freqHz * time);
            float out = eq.processSample(sineSample);
            if (i > 1500) {
                outputMax = std::max(outputMax, std::abs(out));
            }
        }

        // Expected output amplitude is 0.5f * 10^(6/20) = 0.5 * 1.995 = ~1.0f
        float expectedAmp = inputAmp * std::pow(10.0f, 6.0f / 20.0f);
        expectWithinAbsoluteTolerance(outputMax, expectedAmp, 0.05f);
    }
};

static ParametricEQTests parametricEQTests;
