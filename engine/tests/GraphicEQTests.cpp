#include "GraphicEQ.h"
#include <juce_core/juce_core.h>
#include <cmath>
#include "TestHelpers.h"

class GraphicEQTests : public juce::UnitTest
{
public:
    GraphicEQTests() : juce::UnitTest ("GraphicEQ") {}

    void runTest() override
    {
        beginTest ("ISO Band Gains");
        
        dsp::GraphicEQ geq;
        geq.prepare(48000.0, 512);

        // Test default gains are 0 dB
        for (int i = 0; i < dsp::GraphicEQ::NumBands; ++i) {
            expectWithinAbsoluteTolerance(geq.getBandGain(i), 0.0f, 0.01f);
        }

        // Set specific band gain and verify
        geq.setBandGain(17, 6.0f); // 1000 Hz band
        expectWithinAbsoluteTolerance(geq.getBandGain(17), 6.0f, 0.01f);
        
        // Pass sample through flat EQ and verify untouched
        geq.setBandGain(17, 0.0f);
        float input = 0.5f;
        float output = geq.processSample(input);
        expectWithinAbsoluteTolerance(output, input, 0.001f);
    }
};

static GraphicEQTests graphicEQTests;
