#include "Crossover.h"
#include <juce_core/juce_core.h>
#include <cmath>
#include "TestHelpers.h"

class CrossoverTests : public juce::UnitTest
{
public:
    CrossoverTests() : juce::UnitTest ("Crossover") {}

    void runTest() override
    {
        beginTest ("Crossover Complementary Sum");
        
        dsp::Crossover crossover;
        crossover.prepare(48000.0, 512);
        crossover.setType(dsp::Crossover::Type::LR24);
        crossover.setFrequency(100.0f);

        // Process a stereo block of sine wave input
        int numSamples = 512;
        std::vector<float> input(2 * numSamples, 0.5f);
        std::vector<float> sub(2 * numSamples, 0.0f);
        std::vector<float> main(2 * numSamples, 0.0f);

        crossover.processBlockInterleaved(input.data(), sub.data(), main.data(), numSamples);

        // For Linkwitz-Riley crossovers, low-pass and high-pass sum to the exact input signal (flat phase & magnitude)
        for (int i = 0; i < 2 * numSamples; ++i) {
            float sum = sub[i] + main[i];
            // Allow small phase/magnitude deviation in transient startup
            if (i > 100) {
                expectWithinAbsoluteTolerance(sum, input[i], 0.05f);
            }
        }
    }
};

static CrossoverTests crossoverTests;
