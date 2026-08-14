#pragma once

#include "FilterPrimitives.h"

namespace dsp {

class GateExpander {
public:
    GateExpander();
    ~GateExpander() = default;

    void prepare(double sampleRate, int maxBlockSize);
    void reset();

    // Set parameters
    void setThreshold(float thresholdDb);
    void setRatio(float ratio); // e.g., 4.0 or larger for gating
    void setAttack(float attackMs);
    void setHold(float holdMs);
    void setRelease(float releaseMs);
    void setRange(float rangeDb); // Maximum attenuation (e.g. -40 dB)
    void setHysteresis(float hysteresisDb);
    
    // Sidechain config
    void setSidechainFilter(bool hpfEnabled, float hpfHz, bool lpfEnabled, float lpfHz);
    void setEnabled(bool enabled);

    float processSample(float input);
    void processBlock(float* buffer, int numSamples);

    /** Get current gain reduction in dB (for metering) */
    float getGainReductionDb() const;

private:
    double sampleRate { 48000.0 };
    bool isEnabled { false };

    // Parameters
    float thresholdDb { -45.0f };
    float ratio { 4.0f };
    float attackMs { 1.0f };
    float holdMs { 10.0f };
    float releaseMs { 100.0f };
    float rangeDb { -40.0f };
    float hysteresisDb { 2.0f };

    // Smoothing filter coefficients
    float attackCoef { 0.0f };
    float releaseCoef { 0.0f };
    float holdSamples { 0.0f };

    // Sidechain filters
    FilterPrimitives scHpf;
    FilterPrimitives scLpf;
    bool scHpfEnabled { false };
    bool scLpfEnabled { false };

    // Runtime state
    float envelope { 0.0f };
    float currentGain { 1.0f };
    bool isOpen { false };
    float holdTimer { 0.0f }; // remaining hold in samples
    float lastGainReductionDb { 0.0f };

    void updateTimeConstants();
};

} // namespace dsp
