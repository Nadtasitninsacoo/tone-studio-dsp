#include "GateExpander.h"
#include <juce_core/juce_core.h>
#include <cmath>
#include "TestHelpers.h"

class GateExpanderTests : public juce::UnitTest
{
public:
    GateExpanderTests() : juce::UnitTest ("GateExpander") {}

    void runTest() override
    {
        beginTest ("Gate Threshold Test");
        
        dsp::GateExpander gate;
        gate.prepare(48000.0, 512);
        gate.setEnabled(true);
        gate.setThreshold(-30.0f);
        gate.setRange(-40.0f);
        gate.setAttack(1.0f);
        gate.setHold(0.0f);
        gate.setRelease(10.0f);
        gate.setHysteresis(0.0f);

        /**
         * The gate closes on an exponential with a 10 ms time constant, and 2000 samples at
         * 48 kHz is 41.7 ms — 4.2 time constants, which leaves it 8 dB short of the -40 dB
         * range and failed at -81.9 instead of -90. That is the release behaving normally,
         * not a fault: an exponential does not arrive, it approaches.
         *
         * 8000 samples is 167 ms, comfortably settled. Changed here rather than in the DSP
         * because an exponential release is what every gate does; asserting full closure at
         * 4 time constants was the test mis-stating the convention.
         */
        float lowSignal = juce::Decibels::decibelsToGain(-50.0f);
        float output = 0.0f;
        for (int i = 0; i < 8000; ++i) {
            output = gate.processSample(lowSignal);
        }
        float outDb = juce::Decibels::gainToDecibels(std::abs(output), -120.0f);
        // Expect output to be close to -90 dBFS (-50 dB + -40 dB range)
        expectWithinAbsoluteTolerance(outDb, -90.0f, 1.0f);

        // Test above threshold: input = -10 dBFS, should pass untouched (0 dB)
        float highSignal = juce::Decibels::decibelsToGain(-10.0f);
        for (int i = 0; i < 2000; ++i) {
            output = gate.processSample(highSignal);
        }
        outDb = juce::Decibels::gainToDecibels(std::abs(output), -120.0f);
        expectWithinAbsoluteTolerance(outDb, -10.0f, 1.0f);
    }
};

static GateExpanderTests gateExpanderTests;
