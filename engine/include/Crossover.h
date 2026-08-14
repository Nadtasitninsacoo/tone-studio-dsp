#pragma once

#include "FilterPrimitives.h"
#include <array>

namespace dsp {

class Crossover {
public:
    enum class Type {
        LR24, // Linkwitz-Riley 24 dB/oct
        LR48  // Linkwitz-Riley 48 dB/oct
    };

    Crossover();
    ~Crossover() = default;

    void prepare(double sampleRate, int maxBlockSize);
    void reset();

    void setFrequency(float frequencyHz);
    void setType(Type crossoverType);

    /**
     * Process stereo block.
     * Takes stereo input, splits it, and writes to sub (low) and main (high) stereo buffers.
     * Note: buffers must be pre-allocated and at least of size numSamples.
     */
    void processBlockInterleaved(const float* inputBuffer, float* subBuffer, float* mainBuffer, int numSamples);

private:
    double sampleRate { 48000.0 };
    Type type { Type::LR24 };
    float frequencyHz { 100.0f };

    // Max 4 stages for LR48 (2 stages for LR24)
    // Indexes: 0 = Left Channel, 1 = Right Channel
    std::array<std::array<FilterPrimitives, 4>, 2> lpFilters;
    std::array<std::array<FilterPrimitives, 4>, 2> hpFilters;

    void updateFilters();
};

} // namespace dsp
