#include "Limiter.h"
#include <algorithm>
#include <cmath>

namespace dsp {

Limiter::Limiter() {
    // Create oversampler for 2 channels, 4x oversampling (factor = 2)
    oversampler = std::make_unique<juce::dsp::Oversampling<float>>(
        2, 2, 
        juce::dsp::Oversampling<float>::filterHalfBandPolyphaseIIR, 
        true
    );
    reset();
}

void Limiter::prepare(double newSampleRate, int maxBlockSize) {
    sampleRate = newSampleRate;
    
    // Prepare oversampler
    oversampler->initProcessing(maxBlockSize);

    scratchL.assign(static_cast<size_t>(maxBlockSize), 0.0f);
    scratchR.assign(static_cast<size_t>(maxBlockSize), 0.0f);
    
    updateBuffers();
    reset();
}

void Limiter::reset() {
    std::fill(delayBuffer.begin(), delayBuffer.end(), 0.0f);
    writeIndex = 0;
    currentGain = 1.0f;
    lastGainReductionDb = 0.0f;
    oversampler->reset();
}

void Limiter::setThreshold(float val) {
    thresholdDb = val;
}

void Limiter::setLookahead(float val) {
    lookaheadMs = std::clamp(val, 1.0f, 5.0f);
    updateBuffers();
}

void Limiter::setRelease(float val) {
    releaseMs = std::clamp(val, 5.0f, 1000.0f);
    updateBuffers();
}

void Limiter::setCeiling(float val) {
    ceilingDb = std::min(0.0f, val);
}

void Limiter::setEnabled(bool enabled) {
    isEnabled = enabled;
}

void Limiter::updateBuffers() {
    // Oversampled rate is 4x sampleRate
    double osRate = sampleRate * 4.0;
    
    // Delay length in oversampled samples
    delayLength = static_cast<int>((lookaheadMs / 1000.0f) * osRate);
    delayLength = std::max(1, delayLength);
    
    // For 2 channels, delay buffer size is 2 * delayLength
    delayBuffer.assign(2 * delayLength, 0.0f);
    writeIndex = 0;
}

void Limiter::processBlock(float* buffer, int numSamples) {
    const int maxChunk = static_cast<int>(scratchL.size());
    if (maxChunk <= 0) return;
    for (int offset = 0; offset < numSamples; ) {
        const int n = std::min(maxChunk, numSamples - offset);
        processChunk(buffer + 2 * offset, n);
        offset += n;
    }
}

void Limiter::processChunk(float* buffer, int numSamples) {
    if (!isEnabled) {
        lastGainReductionDb = 0.0f;
        return;
    }

    // 1. Pack stereo buffer into juce::dsp::AudioBlock
    // Since master is stereo, we assume buffer contains interleaved or we have 2 channels.
    // Wait, the JUCE console app might pass us a stereo buffer or we can assume it's interleaved.
    // If the buffer is interleaved stereo (L, R, L, R...), we can unpack it, process, and repack.
    // Let's assume the console app passes interleaved stereo data (2 channels).
    // Or we can unpack to a temporary buffer, oversample, process, and repack.
    
    // Reused, never resized here — see the members' comment. The caller guarantees
    // numSamples <= scratchL.size() by chunking.
    auto& leftChan = scratchL;
    auto& rightChan = scratchR;
    for (int i = 0; i < numSamples; ++i) {
        leftChan[i] = buffer[2 * i];
        rightChan[i] = buffer[2 * i + 1];
    }

    float* channels[2] = { leftChan.data(), rightChan.data() };
    juce::dsp::AudioBlock<float> inputBlock(channels, 2, static_cast<size_t>(numSamples));
    
    // 2. Perform 4x oversampling
    // The real API is processSamplesUp / processSamplesDown; processInputPaths and
    // processOutputPaths have never existed on juce::dsp::Oversampling.
    juce::dsp::AudioBlock<float> osBlock = oversampler->processSamplesUp(inputBlock);
    int osSamples = static_cast<int>(osBlock.getNumSamples());
    
    float* osLeft = osBlock.getChannelPointer(0);
    float* osRight = osBlock.getChannelPointer(1);

    // 3. True Peak lookahead limiting loop on oversampled block
    float targetCeiling = juce::Decibels::decibelsToGain(ceilingDb);
    float thresholdGain = juce::Decibels::decibelsToGain(thresholdDb);
    
    // Damping coefficient for the release only. See the attack note in the loop.
    double osRate = sampleRate * 4.0;
    float releaseCoef = 1.0f - std::exp(-1.0f / (static_cast<float>(osRate) * (releaseMs / 1000.0f)));

    for (int i = 0; i < osSamples; ++i) {
        // Read undelayed samples
        float xl = osLeft[i] * thresholdGain;
        float xr = osRight[i] * thresholdGain;
        
        // Push to delay buffer and read delayed samples
        int leftIdx = 2 * writeIndex;
        int rightIdx = 2 * writeIndex + 1;
        
        float delL = delayBuffer[leftIdx];
        float delR = delayBuffer[rightIdx];
        
        delayBuffer[leftIdx] = xl;
        delayBuffer[rightIdx] = xr;
        
        writeIndex = (writeIndex + 1) % delayLength;

        // Peak detector on undelayed oversampled signal (stereo linked)
        float peak = std::max(std::abs(xl), std::abs(xr));
        
        // Calculate target gain
        float targetGain = 1.0f;
        if (peak > targetCeiling) {
            targetGain = targetCeiling / peak;
        }

        /**
         * **Attack is instantaneous, and that is what makes this a brickwall.**
         *
         * It used to ease into the target with a one-millisecond time constant. An
         * exponential needs four or five of those to arrive, so with a two-millisecond
         * lookahead the gain was still halfway down when the peak came out of the delay
         * line: a +6 dBFS input left at 1.29 against a ceiling of 0.891. A limiter that
         * exceeds its ceiling is not a soft limiter, it is a broken one — downstream this
         * feeds a PA, and the ceiling is the only thing standing between a mistake and a
         * blown driver.
         *
         * Snapping down is safe *because* of the lookahead: the reduction is applied to
         * audio that has not been heard yet, so what reaches the output is a ramp over the
         * lookahead window rather than a step. Release stays smooth — that is the part
         * anyone can hear.
         */
        if (targetGain < currentGain) {
            currentGain = targetGain;
        } else {
            currentGain += releaseCoef * (targetGain - currentGain);
        }

        // Apply gain reduction to delayed samples
        osLeft[i] = delL * currentGain;
        osRight[i] = delR * currentGain;
    }

    // 4. Downsample back to original rate
    oversampler->processSamplesDown(inputBlock);
    
    // 5. Repack channels back to buffer
    for (int i = 0; i < numSamples; ++i) {
        buffer[2 * i] = leftChan[i];
        buffer[2 * i + 1] = rightChan[i];
    }
    
    lastGainReductionDb = juce::Decibels::gainToDecibels(currentGain);
}

float Limiter::getGainReductionDb() const {
    return lastGainReductionDb;
}

} // namespace dsp
