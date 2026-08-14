#include "Panner.h"
#include <juce_core/juce_core.h>
#include <cmath>
#include "TestHelpers.h"

class PannerTests : public juce::UnitTest
{
public:
    PannerTests() : juce::UnitTest ("Panner") {}

    void runTest() override
    {
        beginTest ("3dB Panning Law");
        
        dsp::Panner panner;
        panner.prepare(48000.0);
        panner.setLaw(dsp::Panner::Law::Db3);

        // Center pan (0.0): Left and Right gains should be 0.707f (-3 dB)
        panner.setPan(0.0f);
        float outL = 0.0f;
        float outR = 0.0f;
        
        // Process a few times to let smoothing settle
        for (int i = 0; i < 2000; ++i) {
            panner.processSample(1.0f, outL, outR);
        }
        
        expectWithinAbsoluteTolerance(outL, 0.7071f, 0.01f);
        expectWithinAbsoluteTolerance(outR, 0.7071f, 0.01f);

        // Hard Left pan (-1.0): Left should be 1.0f, Right should be 0.0f
        panner.setPan(-1.0f);
        for (int i = 0; i < 2000; ++i) {
            panner.processSample(1.0f, outL, outR);
        }
        expectWithinAbsoluteTolerance(outL, 1.0f, 0.01f);
        expectWithinAbsoluteTolerance(outR, 0.0f, 0.01f);
    }
};

static PannerTests pannerTests;
