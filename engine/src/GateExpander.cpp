#include "GateExpander.h"
#include <algorithm>
#include <cmath>

namespace dsp {

GateExpander::GateExpander() {
    scHpf.setType(FilterPrimitives::Type::HighPass);
    scLpf.setType(FilterPrimitives::Type::LowPass);
    reset();
}

void GateExpander::prepare(double newSampleRate, int maxBlockSize) {
    sampleRate = newSampleRate;
    scHpf.prepare(sampleRate, maxBlockSize);
    scLpf.prepare(sampleRate, maxBlockSize);
    reset();
}

void GateExpander::reset() {
    envelope = 0.0f;
    currentGain = 1.0f;
    isOpen = true;
    holdTimer = 0.0f;
    lastGainReductionDb = 0.0f;
    scHpf.reset();
    scLpf.reset();
    updateTimeConstants();
}

void GateExpander::setThreshold(float val) {
    thresholdDb = val;
}

void GateExpander::setRatio(float val) {
    ratio = std::max(1.0f, val);
}

void GateExpander::setAttack(float val) {
    attackMs = std::max(0.1f, val);
    updateTimeConstants();
}

void GateExpander::setHold(float val) {
    holdMs = std::max(0.0f, val);
    updateTimeConstants();
}

void GateExpander::setRelease(float val) {
    releaseMs = std::max(1.0f, val);
    updateTimeConstants();
}

void GateExpander::setRange(float val) {
    rangeDb = std::min(0.0f, val);
}

void GateExpander::setHysteresis(float val) {
    hysteresisDb = std::max(0.0f, val);
}

void GateExpander::setSidechainFilter(bool hpfEnabled, float hpfHz, bool lpfEnabled, float lpfHz) {
    scHpfEnabled = hpfEnabled;
    scLpfEnabled = lpfEnabled;
    scHpf.setParameters(hpfHz, 0.707f, 0.0f);
    scLpf.setParameters(lpfHz, 0.707f, 0.0f);
}

void GateExpander::setEnabled(bool enabled) {
    isEnabled = enabled;
}

void GateExpander::updateTimeConstants() {
    float attackSec = attackMs / 1000.0f;
    float releaseSec = releaseMs / 1000.0f;
    
    attackCoef = 1.0f - std::exp(-1.0f / (static_cast<float>(sampleRate) * attackSec));
    releaseCoef = 1.0f - std::exp(-1.0f / (static_cast<float>(sampleRate) * releaseSec));
    holdSamples = (holdMs / 1000.0f) * static_cast<float>(sampleRate);
}

float GateExpander::processSample(float input) {
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

    // 2. Measure envelope level in dB
    float absVal = std::abs(scSignal);
    float inputDb = juce::Decibels::gainToDecibels(absVal, -120.0f);

    // 3. Threshold check with Hysteresis
    float openThresh = thresholdDb + hysteresisDb * 0.5f;
    float closeThresh = thresholdDb - hysteresisDb * 0.5f;

    if (inputDb > openThresh) {
        isOpen = true;
        holdTimer = holdSamples;
    } else if (inputDb < closeThresh) {
        if (holdTimer > 0.0f) {
            holdTimer -= 1.0f;
        } else {
            isOpen = false;
        }
    }

    // 4. Calculate target gain
    float targetGain = 1.0f;
    if (!isOpen) {
        float excessDb = closeThresh - inputDb;
        float gainReductionDb = -excessDb * (ratio - 1.0f);
        gainReductionDb = std::max(gainReductionDb, rangeDb);
        targetGain = juce::Decibels::decibelsToGain(gainReductionDb);
    }

    // 5. Apply time constants to gain smoothing
    if (targetGain > currentGain) {
        currentGain += attackCoef * (targetGain - currentGain);
    } else {
        currentGain += releaseCoef * (targetGain - currentGain);
    }

    // 6. Return output
    lastGainReductionDb = juce::Decibels::gainToDecibels(currentGain);
    return input * currentGain;
}

void GateExpander::processBlock(float* buffer, int numSamples) {
    for (int i = 0; i < numSamples; ++i) {
        buffer[i] = processSample(buffer[i]);
    }
}

float GateExpander::getGainReductionDb() const {
    return lastGainReductionDb;
}

} // namespace dsp
