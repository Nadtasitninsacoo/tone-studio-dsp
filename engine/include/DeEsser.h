#pragma once

#include "FilterPrimitives.h"

namespace dsp {

class DeEsser {
public:
    DeEsser();
    ~DeEsser() = default;

    void prepare(double sampleRate, int maxBlockSize);
    void reset();

    // Set parameters
    void setThreshold(float thresholdDb);
    void setRatio(float ratio);
    void setFrequency(float frequencyHz);
    void setListenMode(bool listen);
    void setEnabled(bool enabled);

    float processSample(float input);
    void processBlock(float* buffer, int numSamples);

    /** Get current sibilance attenuation in dB */
    float getGainReductionDb() const;

private:
    double sampleRate { 48000.0 };
    bool isEnabled { false };
    bool listenMode { false };

    // Parameters
    float thresholdDb { -20.0f };
    float ratio { 4.0f };
    float frequencyHz { 6000.0f };

    // Crossover filters (split-band)
    FilterPrimitives lpFilter;
    FilterPrimitives hpFilter;

    // Dynamics smoothing coefficients
    float attackCoef { 0.0f };
    float releaseCoef { 0.0f };

    // Runtime state
    float currentGain { 1.0f };

    /**
     * Peak envelope of the sibilance band, with instant attack and a slow decay.
     *
     * The detector read `std::abs(highBand)` sample by sample, which for any periodic signal
     * collapses to zero twice a cycle — so the computed reduction swung between full and
     * none at the band frequency and the smoothed gain settled somewhere in the middle. A
     * 6 kHz sine 10 dB over a 4:1 threshold should reduce by 7.5 dB; it reduced by 4.7,
     * because the detector was measuring the waveform rather than its level.
     *
     * An envelope is what every compressor puts in front of its gain computer, and it is
     * what makes the ratio mean what it says.
     */
    float detectorEnvelope { 0.0f };
    float detectorReleaseCoef { 0.0f };
    float lastGainReductionDb { 0.0f };

    void updateTimeConstants();
};

} // namespace dsp
