#include "FilterPrimitives.h"
#include <algorithm>

namespace dsp {

FilterPrimitives::FilterPrimitives() {
    reset();
}

void FilterPrimitives::prepare(double newSampleRate, int maxBlockSize) {
    sampleRate = newSampleRate;
    
    // Set default smoothing time (e.g., 20ms)
    smoothedG.reset(sampleRate, 0.020);
    smoothedR.reset(sampleRate, 0.020);
    smoothedK.reset(sampleRate, 0.020);
    
    reset();
    updateCoefficients();
    
    // Force smoothed values to current target immediately on prepare
    smoothedG.setCurrentAndTargetValue(smoothedG.getTargetValue());
    smoothedR.setCurrentAndTargetValue(smoothedR.getTargetValue());
    smoothedK.setCurrentAndTargetValue(smoothedK.getTargetValue());
}

void FilterPrimitives::reset() {
    s1 = 0.0f;
    s2 = 0.0f;
}

void FilterPrimitives::setType(Type newType) {
    type = newType;
    updateCoefficients();
}

void FilterPrimitives::setParameters(float frequencyHz, float q, float gainDb) {
    // The ranges live in the header so callers can clamp to the same numbers instead of
    // guessing and being quietly overruled here. See `MaxQ`.
    targetFreq = std::clamp(frequencyHz, MinFreqHz, MaxFreqHz);
    targetQ = std::clamp(q, MinQ, MaxQ);
    targetGainDb = std::clamp(gainDb, MinGainDb, MaxGainDb);
    updateCoefficients();
}

void FilterPrimitives::updateCoefficients() {
    // 1. Calculate g (pre-warped frequency)
    float wd = 2.0f * 3.1415926535f * targetFreq;
    float g = std::tan(wd / (2.0f * static_cast<float>(sampleRate)));
    
    // 2. Calculate linear gain K
    float K = std::pow(10.0f, targetGainDb / 20.0f);
    
    // 3. Calculate damping coefficient R
    float R = 0.0f;
    switch (type) {
        case Type::Peaking:
            if (targetGainDb >= 0.0f) {
                R = 1.0f / (2.0f * targetQ);
            } else {
                R = 1.0f / (2.0f * targetQ * K);
            }
            break;
            
        case Type::LowShelf:
        case Type::HighShelf:
            R = 1.0f / (2.0f * targetQ);
            break;
            
        default:
            R = 1.0f / (2.0f * targetQ);
            break;
    }
    
    // Set targets for smoothed values
    smoothedG.setTargetValue(g);
    smoothedR.setTargetValue(R);
    smoothedK.setTargetValue(K);
}

float FilterPrimitives::processSample(float input) {
    // Advance smoothed values
    float g = smoothedG.getNextValue();
    float R = smoothedR.getNextValue();
    float K = smoothedK.getNextValue();
    
    // State Variable Filter equations
    float D = 1.0f / (1.0f + 2.0f * R * g + g * g);
    float v3 = input - s2;
    float v1 = (g * v3 + s1) * D;
    float v2 = g * v1 + s2;
    
    // Update state
    s1 = 2.0f * v1 - s1;
    s2 = 2.0f * v2 - s2;
    
    // Generate outputs based on filter type
    switch (type) {
        case Type::LowPass:
            return v2;
            
        case Type::HighPass:
            return input - 2.0f * R * v1 - v2;
            
        case Type::BandPass:
            return v1;
            
        case Type::Notch:
            return input - 2.0f * R * v1;
            
        case Type::Peaking:
            return input + (K - 1.0f) * 2.0f * R * v1;
            
        case Type::LowShelf: {
            float sqrtK = std::sqrt(K);
            return input + (K - 1.0f) * v2 + (sqrtK - 1.0f) * 2.0f * R * v1;
        }
            
        case Type::HighShelf: {
            float hp = input - 2.0f * R * v1 - v2;
            float sqrtK = std::sqrt(K);
            return input + (K - 1.0f) * hp + (sqrtK - 1.0f) * 2.0f * R * v1;
        }
    }
    
    return input;
}

void FilterPrimitives::processBlock(float* buffer, int numSamples) {
    // Ensure ScopedNoDenormals is active (done in caller or here)
    for (int i = 0; i < numSamples; ++i) {
        buffer[i] = processSample(buffer[i]);
    }
}

} // namespace dsp
