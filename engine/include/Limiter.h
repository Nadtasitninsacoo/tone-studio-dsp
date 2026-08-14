#pragma once

#include <juce_audio_basics/juce_audio_basics.h>
#include <juce_dsp/juce_dsp.h>
#include <vector>
#include <memory>

namespace dsp {

class Limiter {
public:
    Limiter();
    ~Limiter() = default;

    void prepare(double sampleRate, int maxBlockSize);
    void reset();

    // Set parameters
    void setThreshold(float thresholdDb);
    void setLookahead(float lookaheadMs);
    void setRelease(float releaseMs);
    void setCeiling(float ceilingDb);
    void setEnabled(bool enabled);

    void processBlock(float* buffer, int numSamples);

private:
    /**
     * One chunk, never longer than prepare()'s maxBlockSize.
     *
     * processBlock() splits into these rather than clamping. Clamping was tried and it is
     * the worse failure by far: a host handing over a longer block than promised would have
     * the tail silently left unprocessed — a limiter that passes full-scale audio straight
     * through, which is the one thing a limiter must never do. Chunking costs nothing and
     * keeps the audio thread free of the heap either way.
     */
    void processChunk(float* buffer, int numSamples);

public:

    /** Get current gain reduction in dB for metering */
    float getGainReductionDb() const;

private:
    double sampleRate { 48000.0 };
    bool isEnabled { true };

    // Parameters
    float thresholdDb { 0.0f };
    float lookaheadMs { 2.0f };
    float releaseMs { 100.0f };
    float ceilingDb { -0.1f };

    // Oversampling (4x)
    std::unique_ptr<juce::dsp::Oversampling<float>> oversampler;

    // Lookahead delay buffer (for oversampled samples)
    std::vector<float> delayBuffer;

    // Deinterleave scratch, allocated in prepare().
    //
    // processBlock() used to declare these as locals, so every audio block called the heap
    // twice. The unit test caught it: 2048 bytes, which is exactly 512 floats. A malloc on
    // the audio thread can block on a lock held by any other thread, and the symptom is a
    // dropout under load that never reproduces on the bench.
    std::vector<float> scratchL;
    std::vector<float> scratchR;
    int writeIndex { 0 };
    int delayLength { 0 };

    // Dynamics state
    float currentGain { 1.0f };
    float lastGainReductionDb { 0.0f };

    void updateBuffers();
};

} // namespace dsp
