#pragma once

#include <vector>

namespace dsp {

class DelayLine {
public:
    DelayLine();
    ~DelayLine() = default;

    void prepare(double sampleRate, int maxBlockSize);
    void reset();

    /** Set delay time in milliseconds (0 - 1000 ms) */
    void setDelayMs(float delayMs);
    
    /** Process a single sample of audio */
    float processSample(float input);
    
    void processBlock(float* buffer, int numSamples);

private:
    double sampleRate { 48000.0 };
    float targetDelayMs { 0.0f };
    float currentDelaySamples { 0.0f };

    /**
     * Whether `currentDelaySamples` has ever been given a real value.
     *
     * It started at zero and glided toward the target at 0.005 per sample, so the very first
     * sample was read at *zero* delay — `writeIndex - 0` is the sample just written, i.e. the
     * input handed straight back. A delay line that passes its input through for the first
     * few hundred samples is a comb filter on every start, and on the alignment delays this
     * class exists for it is simply the wrong number until it settles.
     *
     * The glide is still right for *changes* — it is what stops a delay change pitch-shifting
     * — but the first value is not a change, it is the starting point.
     */
    bool delayPrimed { false };

    // Circular buffer size must be power of 2
    static constexpr int BufferSize = 65536; 
    static constexpr int IndexMask = BufferSize - 1;
    
    std::vector<float> buffer;
    int writeIndex { 0 };

    float readSample(int index) const;
};

} // namespace dsp
