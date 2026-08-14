#include "ReverbDelay.h"
#include <juce_core/juce_core.h>

class ReverbDelayTests : public juce::UnitTest
{
public:
    ReverbDelayTests() : juce::UnitTest ("ReverbDelay") {}

    void runTest() override
    {
        beginTest ("Delay Send Output");
        
        dsp::ReverbDelay rv;
        rv.prepare(48000.0, 512);
        
        rv.setDelayEnabled(true);
        rv.setDelayParams(10.0f, 0.5f, 1.0f, false);
        
        int numSamples = 512;
        std::vector<float> inputL(numSamples, 1.0f);
        std::vector<float> inputR(numSamples, 1.0f);
        const float* inputs[2] = { inputL.data(), inputR.data() };
        
        std::vector<float> outputL(numSamples, 0.0f);
        std::vector<float> outputR(numSamples, 0.0f);
        float* outputs[2] = { outputL.data(), outputR.data() };

        // Process block
        rv.processBlock(inputs, outputs, numSamples);

        // Verify output is generated (not absolute silence)
        bool hasSound = false;
        for (int i = 0; i < numSamples; ++i) {
            if (std::abs(outputs[0][i]) > 0.0f) {
                hasSound = true;
                break;
            }
        }
        expect (hasSound);
    }
};

static ReverbDelayTests reverbDelayTests;
