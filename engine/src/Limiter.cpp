#include "Limiter.h"
#include <algorithm>
#include <cmath>

namespace dsp {

Limiter::Limiter() {
    /**
     * 2 channels, factor 2 → **4×**, with an **FIR** interpolator.
     *
     * It was `filterHalfBandPolyphaseIIR`, and that is what made the ceiling a claim rather
     * than a guarantee. The two are not interchangeable for this job:
     *
     * A true-peak limiter works by reconstructing what the waveform does *between* the
     * samples and holding that down. The polyphase IIR is cheap and phase-nonlinear, and it
     * reconstructs poorly — so the peaks this loop saw at 4× were not the peaks that are
     * really there. It limited what it could see, correctly, and the rest went out.
     *
     * Measured on a real guitar through the running engine, against a −6 dBFS ceiling: the
     * master came out at −5.3 dBFS true peak. The hard clamp below holds every *sample* to
     * the ceiling, and that is a genuine guarantee, but a sample-domain clamp says nothing
     * about the curve between two samples — flattening the tops adds harmonics and pushes
     * the inter-sample peaks further up. Sample peak and true peak are different quantities
     * and only one of them was being controlled.
     *
     * `filterHalfBandFIREquiripple` is the same interpolator `MasterMetering` measures
     * through, and that is the point: the limiter and the meter now reconstruct the signal
     * the same way, so the ceiling the limiter enforces is the ceiling the meter reports. A
     * limiter measured by a better interpolator than it uses will always appear to overshoot.
     *
     * **It costs latency**, and nothing in this engine compensates for it — that was already
     * true of the IIR and this makes it larger. `LimiterTests` prints the figure so a change
     * here is visible rather than discovered on a stage.
     */
    oversampler = std::make_unique<juce::dsp::Oversampling<float>>(
        2, 2,
        juce::dsp::Oversampling<float>::filterHalfBandFIREquiripple,
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
    //
    // **Two ceilings, and the gap between them is a measured fix rather than a safety
    // margin by taste.** `targetCeiling` is what the caller asked for and what the output
    // is held to at the end of this function. `workingCeiling` is what the loop below aims
    // at — a little lower, because `processSamplesDown` adds some of the reduction back.
    // See `kDownsampleHeadroomDb` in the header for the two runs that measured it.
    float targetCeiling = juce::Decibels::decibelsToGain(ceilingDb);
    float workingCeiling = juce::Decibels::decibelsToGain(ceilingDb - kDownsampleHeadroomDb);
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
        if (peak > workingCeiling) {
            targetGain = workingCeiling / peak;
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

    /**
     * 5. Repack, **and hold the output to the ceiling on the way out.**
     *
     * Until this clamp existed the function ended at the repack, and nothing had ever
     * looked at what the decimation filter produced. It was measured overshooting by
     * 0.7–0.9 dB with a real guitar at two different ceilings — see `kDownsampleHeadroomDb`.
     *
     * The attack note above says it plainly, about a different cause: *a limiter that
     * exceeds its ceiling is not a soft limiter, it is a broken one.* That fix took the
     * overshoot from +6.4 dB to +0.8. This is the rest of it, and it is the only line in
     * the class that makes the ceiling a **guarantee** rather than a target the maths aims
     * at — everything upstream operates at 4×, and the signal that leaves here does not.
     *
     * It is a hard clip, and that is the correct instrument here for two reasons: what it
     * removes is filter ringing rather than programme material, and `workingCeiling` is set
     * so that it almost never has anything to remove. A clamp that fires constantly is a
     * clipper wearing a limiter's name — the same category of lie as the meter that read
     * `abs(x)` under the name `truePeakDb`. If this starts biting, raise the headroom
     * constant rather than widening the clamp.
     */
    for (int i = 0; i < numSamples; ++i) {
        buffer[2 * i]     = std::clamp(leftChan[i],  -targetCeiling, targetCeiling);
        buffer[2 * i + 1] = std::clamp(rightChan[i], -targetCeiling, targetCeiling);
    }
    
    lastGainReductionDb = juce::Decibels::gainToDecibels(currentGain);
}

float Limiter::getGainReductionDb() const {
    return lastGainReductionDb;
}

float Limiter::getLatencySamples() const {
    // Before prepare() there is no oversampler and therefore no latency to report. Zero is
    // the truth here rather than a placeholder: nothing is processing yet.
    return oversampler != nullptr ? (float) oversampler->getLatencyInSamples() : 0.0f;
}

} // namespace dsp
