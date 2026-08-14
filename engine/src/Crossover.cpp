#include "Crossover.h"
#include <algorithm>

namespace dsp {

Crossover::Crossover() {
    for (int ch = 0; ch < 2; ++ch) {
        for (int i = 0; i < 4; ++i) {
            lpFilters[ch][i].setType(FilterPrimitives::Type::LowPass);
            hpFilters[ch][i].setType(FilterPrimitives::Type::HighPass);
        }
    }
    reset();
}

void Crossover::prepare(double newSampleRate, int maxBlockSize) {
    sampleRate = newSampleRate;
    for (int ch = 0; ch < 2; ++ch) {
        for (int i = 0; i < 4; ++i) {
            lpFilters[ch][i].prepare(sampleRate, maxBlockSize);
            hpFilters[ch][i].prepare(sampleRate, maxBlockSize);
        }
    }
    updateFilters();
}

void Crossover::reset() {
    for (int ch = 0; ch < 2; ++ch) {
        for (int i = 0; i < 4; ++i) {
            lpFilters[ch][i].reset();
            hpFilters[ch][i].reset();
        }
    }
}

void Crossover::setFrequency(float val) {
    frequencyHz = std::clamp(val, 20.0f, 1000.0f);
    updateFilters();
}

void Crossover::setType(Type crossoverType) {
    type = crossoverType;
    updateFilters();
}

void Crossover::updateFilters() {
    const float Q_24 = 0.70710678f;
    const float Q_48_1 = 0.5411961f;
    const float Q_48_2 = 1.306563f;

    for (int ch = 0; ch < 2; ++ch) {
        if (type == Type::LR24) {
            // Stage 0 and 1
            lpFilters[ch][0].setParameters(frequencyHz, Q_24, 0.0f);
            lpFilters[ch][1].setParameters(frequencyHz, Q_24, 0.0f);
            hpFilters[ch][0].setParameters(frequencyHz, Q_24, 0.0f);
            hpFilters[ch][1].setParameters(frequencyHz, Q_24, 0.0f);
        } else { // LR48
            // Stage 0, 1, 2, 3
            lpFilters[ch][0].setParameters(frequencyHz, Q_48_1, 0.0f);
            lpFilters[ch][1].setParameters(frequencyHz, Q_48_2, 0.0f);
            lpFilters[ch][2].setParameters(frequencyHz, Q_48_1, 0.0f);
            lpFilters[ch][3].setParameters(frequencyHz, Q_48_2, 0.0f);
            
            hpFilters[ch][0].setParameters(frequencyHz, Q_48_1, 0.0f);
            hpFilters[ch][1].setParameters(frequencyHz, Q_48_2, 0.0f);
            hpFilters[ch][2].setParameters(frequencyHz, Q_48_1, 0.0f);
            hpFilters[ch][3].setParameters(frequencyHz, Q_48_2, 0.0f);
        }
    }
}

void Crossover::processBlockInterleaved(
    const float* inputBuffer, 
    float* subBuffer, 
    float* mainBuffer, 
    int numSamples
) {
    int numStages = (type == Type::LR24) ? 2 : 4;
    
    for (int i = 0; i < numSamples; ++i) {
        // Interleaved layout: Left is 2*i, Right is 2*i + 1
        float inL = inputBuffer[2 * i];
        float inR = inputBuffer[2 * i + 1];
        
        // Low-pass path (Sub)
        float subL = inL;
        float subR = inR;
        for (int stage = 0; stage < numStages; ++stage) {
            subL = lpFilters[0][stage].processSample(subL);
            subR = lpFilters[1][stage].processSample(subR);
        }
        
        // High-pass path (Main)
        float mainL = inL;
        float mainR = inR;
        for (int stage = 0; stage < numStages; ++stage) {
            mainL = hpFilters[0][stage].processSample(mainL);
            mainR = hpFilters[1][stage].processSample(mainR);
        }
        
        subBuffer[2 * i] = subL;
        subBuffer[2 * i + 1] = subR;
        
        mainBuffer[2 * i] = mainL;
        mainBuffer[2 * i + 1] = mainR;
    }
}

} // namespace dsp
