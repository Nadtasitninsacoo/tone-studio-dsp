#pragma once

#include <juce_audio_basics/juce_audio_basics.h>
#include <cmath>

namespace dsp {

/**
 * A click-free Topology-Preserving Transform (TPT) State Variable Filter (SVF).
 * Operates in C++20 and JUCE.
 * Supports smooth transition of coefficients.
 */
class FilterPrimitives {
public:
    enum class Type {
        LowPass,
        HighPass,
        BandPass,
        Notch,
        Peaking,
        LowShelf,
        HighShelf
    };

    /**
     * The ranges `setParameters` clamps to, stated once and public.
     *
     * They were private literals, and `FeedbackSuppressor::setParams` clamped its own Q to
     * **5..50** before handing it here — where it met a ceiling of 18 and was silently
     * reduced. Asking a feedback suppressor for a Q of 50 and getting 18 is not a rounding
     * error: Q is how *narrow* the notch is, and a notch nearly three times wider than asked
     * for is the difference between removing a howl and removing a note somebody is singing.
     * That is the failure the whole feature exists to prevent, and nothing reported it.
     *
     * `MaxQ` is 50 rather than 18 now. This is a TPT state-variable filter, unconditionally
     * stable at high Q, so the old ceiling bought nothing — and since no caller could
     * previously get more than 18 past it, nothing that already exists changes.
     */
    static constexpr float MinFreqHz = 20.0f;
    static constexpr float MaxFreqHz = 20000.0f;
    static constexpr float MinQ = 0.1f;
    static constexpr float MaxQ = 50.0f;
    static constexpr float MinGainDb = -18.0f;
    static constexpr float MaxGainDb = 18.0f;

    FilterPrimitives();
    ~FilterPrimitives() = default;

    void prepare(double sampleRate, int maxBlockSize);
    void reset();

    void setType(Type newType);
    void setParameters(float frequencyHz, float q, float gainDb);

    /**
     * Process a single sample of audio.
     */
    float processSample(float input);

    /**
     * Process a block of audio.
     */
    void processBlock(float* buffer, int numSamples);

private:
    void updateCoefficients();

    double sampleRate { 48000.0 };
    Type type { Type::LowPass };

    // Targets for parameters
    float targetFreq { 1000.0f };
    float targetQ { 0.707f };
    float targetGainDb { 0.0f };

    // Smoothed values for filter coefficients to prevent clicks
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> smoothedG;
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> smoothedR;
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> smoothedK; // Linear gain

    // SVF state variables
    float s1 { 0.0f };
    float s2 { 0.0f };
};

} // namespace dsp
