#pragma once

#include <juce_audio_basics/juce_audio_basics.h>

namespace dsp {

class Panner {
public:
    enum class Law {
        Db3,  // -3 dB center gain (constant power)
        Db45  // -4.5 dB center gain
    };

    Panner();
    ~Panner() = default;

    void prepare(double sampleRate);
    void setLaw(Law panningLaw);
    
    /** Set pan value (-1.0 to +1.0) */
    void setPan(float pan);

    /**
     * Process a single mono sample into stereo Left/Right.
     */
    void processSample(float input, float& outputL, float& outputR);

    /**
     * Process a mono buffer into a stereo interleaved buffer.
     */
    void processBlock(const float* monoInput, float* stereoOutput, int numSamples);

private:
    Law law { Law::Db3 };
    float targetPan { 0.0f };
    
    // Smoothed values to avoid pan clicks
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> smoothedPan;

    void getGainFactors(float panVal, float& gainL, float& gainR) const;
};

} // namespace dsp
