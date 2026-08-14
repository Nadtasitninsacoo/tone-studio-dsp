#include "Compressor.h"
#include <algorithm>
#include <cmath>

namespace dsp {

Compressor::Compressor() {
    scHpf.setType(FilterPrimitives::Type::HighPass);
    scLpf.setType(FilterPrimitives::Type::LowPass);
    reset();
}

void Compressor::prepare(double newSampleRate, int maxBlockSize) {
    sampleRate = newSampleRate;
    scHpf.prepare(sampleRate, maxBlockSize);
    scLpf.prepare(sampleRate, maxBlockSize);
    reset();
}

void Compressor::reset() {
    rmsSquare = 0.0f;
    currentGain = 1.0f;
    lastGainReductionDb = 0.0f;
    scHpf.reset();
    scLpf.reset();
    updateTimeConstants();
}

void Compressor::setThreshold(float val) {
    thresholdDb = val;
}

void Compressor::setRatio(float val) {
    ratio = std::max(1.0f, val);
}

void Compressor::setAttack(float val) {
    attackMs = std::max(0.1f, val);
    updateTimeConstants();
}

void Compressor::setRelease(float val) {
    releaseMs = std::max(1.0f, val);
    updateTimeConstants();
}

void Compressor::setKnee(float val) {
    kneeDb = std::max(0.0f, val);
}

void Compressor::setMakeup(float val) {
    makeupDb = val;
}

void Compressor::setAutoMakeup(bool enabled) {
    autoMakeup = enabled;
}

void Compressor::setDetectionMode(DetectionMode mode) {
    detectionMode = mode;
}

void Compressor::setEnabled(bool enabled) {
    isEnabled = enabled;
}

void Compressor::setSidechainFilter(bool hpfEnabled, float hpfHz, bool lpfEnabled, float lpfHz) {
    scHpfEnabled = hpfEnabled;
    scLpfEnabled = lpfEnabled;
    scHpf.setParameters(hpfHz, 0.707f, 0.0f);
    scLpf.setParameters(lpfHz, 0.707f, 0.0f);
}

void Compressor::updateTimeConstants() {
    float attackSec = attackMs / 1000.0f;
    float releaseSec = releaseMs / 1000.0f;
    
    attackCoef = 1.0f - std::exp(-1.0f / (static_cast<float>(sampleRate) * attackSec));
    releaseCoef = 1.0f - std::exp(-1.0f / (static_cast<float>(sampleRate) * releaseSec));
    
    // RMS integration time constant (e.g. 50ms)
    rmsCoef = 1.0f - std::exp(-1.0f / (static_cast<float>(sampleRate) * 0.050f));
}

float Compressor::processSample(float input) {
    if (!isEnabled) {
        lastGainReductionDb = 0.0f;
        return input;
    }

    // 1. Process sidechain filters
    float scSignal = input;
    if (scHpfEnabled) {
        scSignal = scHpf.processSample(scSignal);
    }
    if (scLpfEnabled) {
        scSignal = scLpf.processSample(scSignal);
    }

    // 2. Measure envelope (Peak or RMS)
    float level = 0.0f;
    if (detectionMode == DetectionMode::RMS) {
        float sq = scSignal * scSignal;
        rmsSquare += rmsCoef * (sq - rmsSquare);
        level = std::sqrt(std::max(0.0f, rmsSquare));
    } else {
        level = std::abs(scSignal);
    }

    // Convert level to dB
    float levelDb = juce::Decibels::gainToDecibels(level, -120.0f);

    // 3. Static compression curve with soft knee
    float gainReductionDb = 0.0f;
    if (kneeDb > 0.0f) {
        if (levelDb > thresholdDb - kneeDb * 0.5f) {
            if (levelDb < thresholdDb + kneeDb * 0.5f) {
                float excess = levelDb - (thresholdDb - kneeDb * 0.5f);
                float compressionFactor = (1.0f / ratio - 1.0f) * excess * excess / (2.0f * kneeDb);
                gainReductionDb = compressionFactor;
            } else {
                gainReductionDb = (1.0f / ratio - 1.0f) * (levelDb - thresholdDb);
            }
        }
    } else {
        if (levelDb > thresholdDb) {
            gainReductionDb = (1.0f / ratio - 1.0f) * (levelDb - thresholdDb);
        }
    }

    // 4. Convert gain reduction to linear gain factor
    float targetGain = juce::Decibels::decibelsToGain(gainReductionDb);

    // 5. Smooth the gain factor (using attack and release)
    if (targetGain < currentGain) {
        currentGain += attackCoef * (targetGain - currentGain);
    } else {
        currentGain += releaseCoef * (targetGain - currentGain);
    }

    // 6. Apply makeup and auto-makeup
    float finalMakeupDb = makeupDb;
    if (autoMakeup) {
        float grAtZero = (1.0f / ratio - 1.0f) * (-thresholdDb);
        finalMakeupDb += -grAtZero * 0.5f;
    }

    float finalGain = currentGain * juce::Decibels::decibelsToGain(finalMakeupDb);
    
    lastGainReductionDb = juce::Decibels::gainToDecibels(currentGain);
    
    return input * finalGain;
}

void Compressor::processBlock(float* buffer, int numSamples) {
    for (int i = 0; i < numSamples; ++i) {
        buffer[i] = processSample(buffer[i]);
    }
}

float Compressor::getGainReductionDb() const {
    return lastGainReductionDb;
}

} // namespace dsp
