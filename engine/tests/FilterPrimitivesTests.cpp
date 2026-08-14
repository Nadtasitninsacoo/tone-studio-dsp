#include "FilterPrimitives.h"
#include <juce_core/juce_core.h>
#include <cmath>

class FilterPrimitivesTests : public juce::UnitTest
{
public:
    FilterPrimitivesTests() : juce::UnitTest ("FilterPrimitives") {}

    void runTest() override
    {
        beginTest ("Bypass Impulse Response");
        dsp::FilterPrimitives filter;
        filter.prepare(48000.0, 512);
        
        filter.setType(dsp::FilterPrimitives::Type::LowPass);
        filter.setParameters(1000.0f, 0.707f, 0.0f);
        
        // Test basic sample throughput
        float output = filter.processSample(1.0f);
        expect (!std::isnan(output));
        expect (!std::isinf(output));
        
        // Decay check
        for (int i = 0; i < 1000; ++i) {
            output = filter.processSample(0.0f);
        }
        expect (std::abs(output) < 0.01f);
    }
};

static FilterPrimitivesTests filterPrimitivesTests;
