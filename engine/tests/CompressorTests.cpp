#include "Compressor.h"
#include <juce_core/juce_core.h>
#include <cmath>
#include "TestHelpers.h"

class CompressorTests : public juce::UnitTest
{
public:
    CompressorTests() : juce::UnitTest ("Compressor") {}

    void runTest() override
    {
        beginTest ("Compression Ratio Test");
        
        dsp::Compressor comp;
        comp.prepare(48000.0, 512);
        comp.setEnabled(true);
        comp.setThreshold(-20.0f);
        comp.setRatio(4.0f);
        comp.setAttack(1.0f);   // Fast attack
        comp.setRelease(100.0f);
        comp.setKnee(0.0f);     // Hard knee
        comp.setMakeup(0.0f);
        comp.setDetectionMode(dsp::Compressor::DetectionMode::Peak);

        // Input: -10 dBFS sine wave (10 dB above threshold)
        // Expected gain reduction: (10 - 10/4) = 7.5 dB
        float freqHz = 1000.0f;
        float inputAmp = juce::Decibels::decibelsToGain(-10.0f); // approx 0.3162f
        
        // Run for 1 second of audio to let the compressor settle
        float outputSample = 0.0f;
        for (int i = 0; i < 48000; ++i)
        {
            float time = static_cast<float>(i) / 48000.0f;
            float sineSample = inputAmp * std::sin(2.0f * 3.14159265f * freqHz * time);
            outputSample = comp.processSample(sineSample);
        }

        float grDb = comp.getGainReductionDb();
        
        // Assert that the gain reduction settled near -7.5 dB
        expectWithinAbsoluteTolerance(grDb, -7.5f, 0.5f);
    }
};

static CompressorTests compressorTests;
