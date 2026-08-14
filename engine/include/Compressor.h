#pragma once

#include "FilterPrimitives.h"

namespace dsp {

class Compressor {
public:
    enum class DetectionMode {
        Peak,
        RMS
    };

    Compressor();
    ~Compressor() = default;

    void prepare(double sampleRate, int maxBlockSize);
    void reset();

    // Set parameters
    void setThreshold(float thresholdDb);
    void setRatio(float ratio);
    void setAttack(float attackMs);
    void setRelease(float releaseMs);
    void setKnee(float kneeDb);
    void setMakeup(float makeupDb);
    void setAutoMakeup(bool enabled);
    void setDetectionMode(DetectionMode mode);
    void setEnabled(bool enabled);

    // Sidechain config
    void setSidechainFilter(bool hpfEnabled, float hpfHz, bool lpfEnabled, float lpfHz);

    float processSample(float input);
    void processBlock(float* buffer, int numSamples);

    /** Get current gain reduction in dB (for metering) */
    float getGainReductionDb() const;

private:
    double sampleRate { 48000.0 };
    bool isEnabled { false };

    // Parameters
    float thresholdDb { -20.0f };
    float ratio { 4.0f };
    float attackMs { 10.0f };
    float releaseMs { 200.0f };
    float kneeDb { 6.0f };
    float makeupDb { 0.0f };
    bool autoMakeup { false };
    DetectionMode detectionMode { DetectionMode::RMS };

    // Coefficients
    float attackCoef { 0.0f };
    float releaseCoef { 0.0f };
    float rmsCoef { 0.0f };

    // Sidechain filters
    FilterPrimitives scHpf;
    FilterPrimitives scLpf;
    bool scHpfEnabled { false };
    bool scLpfEnabled { false };

    // Runtime state
    float rmsSquare { 0.0f };
    float currentGain { 1.0f };
    float lastGainReductionDb { 0.0f };

    void updateTimeConstants();
};

} // namespace dsp
