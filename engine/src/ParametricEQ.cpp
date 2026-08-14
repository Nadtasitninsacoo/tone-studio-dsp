#include "ParametricEQ.h"

namespace dsp {

ParametricEQ::ParametricEQ() {
    reset();
}

void ParametricEQ::prepare(double sampleRate, int maxBlockSize) {
    for (auto& filter : filters) {
        filter.prepare(sampleRate, maxBlockSize);
    }
}

void ParametricEQ::reset() {
    for (auto& filter : filters) {
        filter.reset();
    }
}

void ParametricEQ::setBandParameters(int bandIndex, const BandConfig& config) {
    if (bandIndex < 0 || bandIndex >= NumBands) return;

    filters[bandIndex].setType(config.type);
    filters[bandIndex].setParameters(config.frequencyHz, config.q, config.gainDb);
    bandEnabled[bandIndex] = config.enabled;
}

void ParametricEQ::setBandEnabled(int bandIndex, bool enabled) {
    if (bandIndex < 0 || bandIndex >= NumBands) return;
    bandEnabled[bandIndex] = enabled;
}

float ParametricEQ::processSample(float input) {
    float output = input;
    for (int i = 0; i < NumBands; ++i) {
        if (bandEnabled[i]) {
            output = filters[i].processSample(output);
        }
    }
    return output;
}

void ParametricEQ::processBlock(float* buffer, int numSamples) {
    for (int i = 0; i < numSamples; ++i) {
        buffer[i] = processSample(buffer[i]);
    }
}

} // namespace dsp
