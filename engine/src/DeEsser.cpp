#include "DeEsser.h"
#include <algorithm>
#include <cmath>

namespace dsp {

DeEsser::DeEsser() {
    lpFilter.setType(FilterPrimitives::Type::LowPass);
    hpFilter.setType(FilterPrimitives::Type::HighPass);
    reset();
}

void DeEsser::prepare(double newSampleRate, int maxBlockSize) {
    sampleRate = newSampleRate;
    lpFilter.prepare(sampleRate, maxBlockSize);
    hpFilter.prepare(sampleRate, maxBlockSize);
    reset();
}

void DeEsser::reset() {
    currentGain = 1.0f;
    lastGainReductionDb = 0.0f;
    lpFilter.reset();
    hpFilter.reset();
    updateTimeConstants();
}

void DeEsser::setThreshold(float val) {
    thresholdDb = val;
}

void DeEsser::setRatio(float val) {
    ratio = std::max(1.0f, val);
}

void DeEsser::setFrequency(float val) {
    frequencyHz = std::clamp(val, 2000.0f, 12000.0f);
    lpFilter.setParameters(frequencyHz, 0.707f, 0.0f);
    hpFilter.setParameters(frequencyHz, 0.707f, 0.0f);
}

void DeEsser::setListenMode(bool listen) {
    listenMode = listen;
}

void DeEsser::setEnabled(bool enabled) {
    isEnabled = enabled;
}

void DeEsser::updateTimeConstants() {
    // De-esser uses very fast attack (e.g. 2ms) and release (e.g. 35ms)
    float attackSec = 0.002f;
    float releaseSec = 0.035f;
    
    attackCoef = 1.0f - std::exp(-1.0f / (static_cast<float>(sampleRate) * attackSec));
    releaseCoef = 1.0f - std::exp(-1.0f / (static_cast<float>(sampleRate) * releaseSec));
    // 20 ms envelope decay — long against a 6 kHz period, short against a syllable.
    detectorReleaseCoef = std::exp(-1.0f / (static_cast<float>(sampleRate) * 0.020f));
    detectorEnvelope = 0.0f;
    
    lpFilter.setParameters(frequencyHz, 0.707f, 0.0f);
    hpFilter.setParameters(frequencyHz, 0.707f, 0.0f);
}

float DeEsser::processSample(float input) {
    if (!isEnabled) {
        lastGainReductionDb = 0.0f;
        return input;
    }

    // 1. Split the band at sibilance crossover frequency
    float lowBand = lpFilter.processSample(input);
    float highBand = hpFilter.processSample(input);

    // 2. Measure the sibilance band's *level* — a peak envelope, not the waveform.
    //    Instant attack so a real 'sss' is caught on its first cycle; a 20 ms decay so the
    //    envelope rides the peak instead of following the signal down to zero crossings.
    float absVal = std::abs(highBand);
    if (absVal > detectorEnvelope) detectorEnvelope = absVal;
    else detectorEnvelope = absVal + detectorReleaseCoef * (detectorEnvelope - absVal);

    float highLevelDb = juce::Decibels::gainToDecibels(detectorEnvelope, -120.0f);

    // 3. Dynamic compression calculation on sibilance
    float gainReductionDb = 0.0f;
    if (highLevelDb > thresholdDb) {
        gainReductionDb = (1.0f / ratio - 1.0f) * (highLevelDb - thresholdDb);
    }

    // 4. Smooth gain factor
    float targetGain = juce::Decibels::decibelsToGain(gainReductionDb);
    if (targetGain < currentGain) {
        currentGain += attackCoef * (targetGain - currentGain);
    } else {
        currentGain += releaseCoef * (targetGain - currentGain);
    }

    float compressedHigh = highBand * currentGain;
    lastGainReductionDb = juce::Decibels::gainToDecibels(currentGain);

    // 5. Output based on listen mode
    if (listenMode) {
        return compressedHigh; // User wants to hear sibilance only
    }

    return lowBand + compressedHigh;
}

void DeEsser::processBlock(float* buffer, int numSamples) {
    for (int i = 0; i < numSamples; ++i) {
        buffer[i] = processSample(buffer[i]);
    }
}

float DeEsser::getGainReductionDb() const {
    return lastGainReductionDb;
}

} // namespace dsp
