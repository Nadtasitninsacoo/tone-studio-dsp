#pragma once

#include <juce_audio_basics/juce_audio_basics.h>
#include <juce_dsp/juce_dsp.h>
#include <array>
#include <atomic>
#include <vector>

namespace dsp {

/**
 * K-Weighting filter for BS.1770 LUFS measurement.
 * Cascade of High-Shelf and High-Pass.
 */
class KWeighting {
public:
    KWeighting();
    void prepare(double sampleRate);
    void reset();
    float processSample(float input);

private:
    // Biquad coefficients and state
    double b0_s { 1.0 }, b1_s { 0.0 }, b2_s { 0.0 };
    double a1_s { 0.0 }, a2_s { 0.0 };
    double x1_s { 0.0 }, x2_s { 0.0 }, y1_s { 0.0 }, y2_s { 0.0 };

    double b0_h { 1.0 }, b1_h { 0.0 }, b2_h { 0.0 };
    double a1_h { 0.0 }, a2_h { 0.0 };
    double x1_h { 0.0 }, x2_h { 0.0 }, y1_h { 0.0 }, y2_h { 0.0 };
};

/**
 * Handles metering for a single input channel or bus (Peak, RMS, Gain Reduction).
 */
class ChannelMetering {
public:
    ChannelMetering();
    ~ChannelMetering() = default;

    void prepare(double sampleRate);
    void reset();

    void processBlock(const float* buffer, int numSamples, float gainReductionDb);

    // Read values (atomic, non-realtime safe)
    float getPeakDb() const;
    float getRmsDb() const;
    float getGainReductionDb() const;

private:
    double sampleRate { 48000.0 };
    
    std::atomic<float> peakDb { -120.0f };
    std::atomic<float> rmsDb { -120.0f };
    std::atomic<float> maxGainReductionDb { 0.0f };

    float peakEnvelope { 0.0f };
    float rmsAccumulator { 0.0f };
    
    // Time constants
    // Recomputed per block from numSamples — see ChannelMetering::processBlock.
    float peakReleaseCoef { 0.0f };
    float rmsCoef { 0.0f };
    static constexpr float kPeakReleaseSec = 0.500f;
    static constexpr float kRmsWindowSec = 0.050f;
};

/**
 * Handles master bus metering: True Peak, Momentary / Short-term LUFS, and RTA.
 */
class MasterMetering {
public:
    static constexpr int NumRtaBands = 31;

    MasterMetering();
    ~MasterMetering() = default;

    void prepare(double sampleRate, int maxBlockSize);
    void reset();

    void processBlock(const float** stereoBuffer, int numSamples, float limiterGrDb);

    // Read values
    float getTruePeakDbL() const;
    float getTruePeakDbR() const;
    float getMomentaryLufs() const;
    float getShortTermLufs() const;
    float getLimiterGrDb() const;
    std::array<float, NumRtaBands> getRtaBains() const;

private:
    double sampleRate { 48000.0 };

    // Atomics for WebSocket/UI
    std::atomic<float> truePeakDbL { -120.0f };
    std::atomic<float> truePeakDbR { -120.0f };
    std::atomic<float> momentaryLufs { -120.0f };
    std::atomic<float> shortTermLufs { -120.0f };
    std::atomic<float> limiterGr { 0.0f };
    std::array<std::atomic<float>, NumRtaBands> rtaBandsAtom;

    // True Peak detector envelopes
    float tpL { 0.0f };
    float tpR { 0.0f };
    /**
     * How long the true-peak reading takes to fall, and the coefficient derived from it.
     *
     * Named beside the value it describes for the same reason `kPeakReleaseSec` is: this
     * coefficient used to be computed once in `prepare()` as a per-*sample* figure and then
     * applied once per block, which multiplied the release time by the block size — 500 ms
     * became about four minutes. Deriving it in `processBlock` from `numSamples` is the only
     * form that stays correct when the host changes the block size, which it does on every
     * device change.
     */
    static constexpr float kTruePeakReleaseSec = 0.500f;
    float tpReleaseCoef { 0.0f };

    // K-Weighting filters for Left & Right
    KWeighting kFilterL;
    KWeighting kFilterR;

    // LUFS sliding windows
    // Momentary: 400 ms window
    // Short-term: 3 sec window
    std::vector<float> msHistory;
    int msHistoryWriteIdx { 0 };
    int msWindowSizeSamples { 0 };
    double msSum { 0.0 };

    std::vector<float> stHistory;
    int stHistoryWriteIdx { 0 };
    int stWindowSizeSamples { 0 };
    double stSum { 0.0 };

    // RTA Analyzer
    /**
     * **1024 points was not enough to tell the bottom third of the plot apart.**
     *
     * At 48 kHz a 1024-point FFT has 46.9 Hz bins, and a 1/3-octave band at 63 Hz is 14.6 Hz
     * wide — narrower than a single bin. Several ISO bands therefore resolved to the *same*
     * bin and reported identical levels. Seen on a real guitar:
     *
     *     63Hz=-29.8  80Hz=-29.8  100Hz=-29.8  125Hz=-29.8  160Hz=-29.8
     *
     * Five bars, one number. The plot drew 31 distinct bars while the lowest ten carried no
     * information of their own — the same family as a meter tapped at the wrong point: the
     * picture and what it claims to measure are different things.
     *
     * 4096 gives 11.7 Hz bins, which separates every band down to 31.5 Hz. **20 Hz and 25 Hz
     * are still degenerate** and that is stated rather than hidden — resolving them needs
     * about 16k points, which at this update rate is a third of a second of latency for two
     * bars at the very bottom of a PA's range.
     *
     * The cost is a bigger, rarer spike on the audio thread: one 4096-point FFT every 85 ms
     * instead of a 1024-point one every 21 ms. The same work per sample, four times the
     * burst — comfortable inside a 10 ms block, and worth knowing before raising it further.
     */
    static constexpr int RtaFftOrder = 12; // 4096 points
    static constexpr int RtaFftSize = 1 << RtaFftOrder;
    juce::dsp::FFT rtaFft { RtaFftOrder };
    std::vector<float> rtaInputBuffer;
    std::vector<float> rtaFftBuffer;
    int rtaWriteIdx { 0 };

    void processRta();
};

} // namespace dsp
