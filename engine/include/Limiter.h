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

    /**
     * How far below the requested ceiling the oversampled stage actually limits, so that the
     * hard clamp after `processSamplesDown` stays a backstop instead of becoming a clipper.
     *
     * **0.1 dB is measured, and the number it replaced was a guess that measurement
     * refuted.** The sequence is worth keeping because it is the whole argument for having a
     * bench at all.
     *
     * A real guitar through the running engine overshot its ceiling twice — 0.9 dB over −6,
     * 0.7 dB over −12. Moving the ceiling 6 dB moved the overshoot 0.2 dB, which ruled out
     * `setCeiling` and the dB→linear conversion (a scaling error would have doubled it) and
     * left the decimation filter as the suspect. This constant was set to **1.0 dB** to
     * cover that, which is sizing a fix to a symptom's magnitude before checking the
     * mechanism produces it.
     *
     * It does not. Four synthetic signals through this class alone — a silence-to-full-scale
     * burst, a square wave, clicks, and a full-scale sine at fs/4 — overshoot the working
     * ceiling by **about 0.03 dB**, not 0.9. The burst is the only one that reaches the
     * clamp at all. So 1 dB was ten times the measured need, and on steady material it was
     * a straight 1 dB of lost output: `Brickwall Ceiling Test` drives DC and landed at
     * exactly `ceiling − 1`.
     *
     * 0.1 dB takes the clamp to zero activations on every one of those four while costing a
     * tenth of a dB.
     *
     * **What this does NOT do is explain the 0.9 dB.** That was measured on a real signal
     * through the whole graph and is not reproduced here by a factor of thirty. Do not read
     * this constant as the fix for it. The leading remaining candidate is that
     * `filterHalfBandPolyphaseIIR` is a poor enough interpolator that the peaks the limiter
     * sees at 4× are not the real inter-sample peaks — in which case the clamp is what
     * actually holds the line, and the number to watch is the one from `MasterMetering`, now
     * that it measures true peak instead of `abs(x)`. Re-measure before touching this again.
     */
    static constexpr float kDownsampleHeadroomDb = 0.1f;

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
