#pragma once

#include "FilterPrimitives.h"
#include <array>

namespace dsp {

class ParametricEQ {
public:
    static constexpr int NumBands = 6;

    struct BandConfig {
        FilterPrimitives::Type type { FilterPrimitives::Type::Peaking };
        float frequencyHz { 1000.0f };
        float q { 0.707f };
        float gainDb { 0.0f };
        bool enabled { false };
    };

    ParametricEQ();
    ~ParametricEQ() = default;

    void prepare(double sampleRate, int maxBlockSize);
    void reset();

    void setBandParameters(int bandIndex, const BandConfig& config);
    void setBandEnabled(int bandIndex, bool enabled);

    float processSample(float input);
    void processBlock(float* buffer, int numSamples);

private:
    std::array<FilterPrimitives, NumBands> filters;
    std::array<bool, NumBands> bandEnabled { false, false, false, false, false, false };
};

} // namespace dsp
