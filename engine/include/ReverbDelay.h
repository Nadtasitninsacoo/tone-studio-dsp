#pragma once

#include "FilterPrimitives.h"
#include <juce_audio_basics/juce_audio_basics.h>
#include <vector>

namespace dsp {

class ReverbDelay {
public:
    ReverbDelay();
    ~ReverbDelay() = default;

    void prepare(double sampleRate, int maxBlockSize);
    void reset();

    // Reverb parameters
    void setReverbEnabled(bool enabled);
    void setReverbParams(float roomSize, float damping, float width, float wetLevel);
    void setReverbRoomSize(float roomSize);
    void setReverbDamping(float damping);
    void setReverbWidth(float width);
    void setReverbWetLevel(float wetLevel);

    // Delay parameters
    void setDelayEnabled(bool enabled);
    void setDelayParams(float delayMs, float feedback, float wetLevel, bool pingPong);
    void setDelayMs(float delayMs);
    void setDelayFeedback(float feedback);
    void setDelayWetLevel(float wetLevel);
    void setDelayPingPong(bool pingPong);
    void setDelayFilters(float hpfHz, float lpfHz);
    void setDelayHpf(float hpfHz);
    void setDelayLpf(float lpfHz);

    /**
     * Process stereo send input.
     * Takes stereo input (summed from channel sends) and adds the wet processed reverb/delay output to the master bus.
     */
    void processBlock(const float** inputChannels, float** outputChannels, int numSamples);

private:
    double sampleRate { 48000.0 };

    // Reverb
    bool reverbEnabled { false };
    juce::Reverb reverb;
    juce::Reverb::Parameters reverbParams;

    // Delay
    bool delayEnabled { false };
    float targetDelayMs { 500.0f };
    float currentDelayMs { 500.0f };

    /**
     * Whether `currentDelayMs` has ever been set to a real target.
     *
     * Two bugs met here and the delay time was simply never applied. The smoothing ramp was
     * held in a *local* — `delaySamples` — that was recomputed from `currentDelayMs` at the
     * top of every block and never written back, so each block restarted from 500 ms and
     * crawled a few milliseconds toward the target before forgetting. And the starting point
     * was 500 ms whatever the caller asked for, so a 10 ms delay read from a part of the
     * buffer that had never been written: silence, which is exactly what the test saw.
     */
    bool delayPrimed { false };
    float feedback { 0.5f };
    float delayWetLevel { 0.5f };
    bool pingPongEnabled { false };

    // Delay line circular buffers (Left & Right)
    static constexpr int BufferSize = 131072; // ~2.7 seconds at 48kHz
    static constexpr int IndexMask = BufferSize - 1;
    
    std::vector<float> delayBufferL;
    std::vector<float> delayBufferR;

    // Per-block scratch, allocated in prepare(). Four heap calls per audio block used to
    // live in processBlock(); same defect as the Limiter's, caught by the same test.
    std::vector<float> wetL, wetR, revL, revR;

    /** One chunk, never longer than prepare()'s maxBlockSize. See Limiter for why. */
    void processChunk(const float** inputChannels, float** outputChannels, int numSamples);
    int writeIndex { 0 };

    // Filters in delay feedback loop
    FilterPrimitives feedbackHpfL;
    FilterPrimitives feedbackHpfR;
    FilterPrimitives feedbackLpfL;
    FilterPrimitives feedbackLpfR;
    float feedbackHpfHz { 200.0f };
    float feedbackLpfHz { 4000.0f };

    float readSample(const std::vector<float>& buffer, float delayInSamples) const;
};

} // namespace dsp
