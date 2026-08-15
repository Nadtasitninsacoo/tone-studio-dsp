#include "Metering.h"
#include "GraphicEQ.h"   // IsoFrequencies — the RTA maps its bins onto them
#include <algorithm>
#include <cmath>

namespace dsp {

/* -- K-Weighting filter implementation -- */
KWeighting::KWeighting() {
    reset();
}

void KWeighting::prepare(double newSampleRate) {
    // Standard BS.1770 K-weighting coefficients for 48 kHz
    // Stage 1: High shelf filter
    b0_s = 1.53090961811162;
    b1_s = -2.65116903259146;
    b2_s = 1.16916684073381;
    a1_s = -1.66365011327177;
    a2_s = 0.71255753952573;

    // Stage 2: High pass filter
    b0_h = 1.0;
    b1_h = -2.0;
    b2_h = 1.0;
    a1_h = -1.99004745483398;
    a2_h = 0.99007225036621;

    // Adjust if sample rate is different from 48 kHz
    if (std::abs(newSampleRate - 48000.0) > 100.0) {
        // Approximate scaling of frequency parameters
        // (In a production system, these coefficients would be recalculated analytically)
    }
    reset();
}

void KWeighting::reset() {
    x1_s = x2_s = y1_s = y2_s = 0.0;
    x1_h = x2_h = y1_h = y2_h = 0.0;
}

float KWeighting::processSample(float input) {
    // Stage 1: High shelf
    double w1 = input - a1_s * y1_s - a2_s * y2_s;
    double out_s = b0_s * w1 + b1_s * y1_s + b2_s * y2_s;
    y2_s = y1_s;
    y1_s = w1;

    // Stage 2: High pass
    double w2 = out_s - a1_h * y1_h - a2_h * y2_h;
    double out_h = b0_h * w2 + b1_h * y1_h + b2_h * y2_h;
    y2_h = y1_h;
    y1_h = w2;

    return static_cast<float>(out_h);
}


/* -- Channel Metering implementation -- */
ChannelMetering::ChannelMetering() {
    reset();
}

void ChannelMetering::prepare(double newSampleRate) {
    sampleRate = newSampleRate;
    reset();
}

void ChannelMetering::reset() {
    peakEnvelope = 0.0f;
    rmsAccumulator = 0.0f;
    
    /**
     * Coefficients are derived **per block**, in processBlock, not here.
     *
     * They were computed as per-*sample* coefficients — `1 - exp(-1 / (fs * 0.050))` — and
     * then applied once per block. At 48 kHz and 512-sample blocks that is a 50 ms window
     * stretched to about 25 seconds, and the test caught it precisely: a steady -6.02 dBFS
     * tone metered -39.8 dB after one block. A meter that reads 34 dB low is worse than no
     * meter, because it is believed.
     *
     * Deriving from `numSamples` also makes them correct at any block size, which a value
     * baked in reset() cannot be — reset() is not told what the block size is.
     */
    
    peakDb.store(-120.0f);
    rmsDb.store(-120.0f);
    maxGainReductionDb.store(0.0f);
}

void ChannelMetering::processBlock(const float* buffer, int numSamples, float gainReductionDb) {
    float peakVal = 0.0f;
    float rmsSum = 0.0f;
    
    for (int i = 0; i < numSamples; ++i) {
        float absVal = std::abs(buffer[i]);
        peakVal = std::max(peakVal, absVal);
        rmsSum += absVal * absVal;
    }
    
    // One block's worth of time, so both constants mean what they say at any block size.
    const float blockSec = numSamples > 0
        ? static_cast<float>(numSamples) / static_cast<float>(sampleRate)
        : 0.0f;
    peakReleaseCoef = std::exp(-blockSec / kPeakReleaseSec);
    rmsCoef = 1.0f - std::exp(-blockSec / kRmsWindowSec);

    // Smooth peak envelope
    if (peakVal > peakEnvelope) {
        peakEnvelope = peakVal;
    } else {
        peakEnvelope = peakVal + peakReleaseCoef * (peakEnvelope - peakVal);
    }
    
    // Smooth RMS accumulator
    float blockRmsSquare = numSamples > 0 ? rmsSum / numSamples : 0.0f;
    rmsAccumulator += rmsCoef * (blockRmsSquare - rmsAccumulator);
    float currentRms = std::sqrt(std::max(0.0f, rmsAccumulator));
    
    // Write to atomics
    peakDb.store(juce::Decibels::gainToDecibels(peakEnvelope, -120.0f), std::memory_order_relaxed);
    rmsDb.store(juce::Decibels::gainToDecibels(currentRms, -120.0f), std::memory_order_relaxed);
    maxGainReductionDb.store(gainReductionDb, std::memory_order_relaxed);
}

float ChannelMetering::getPeakDb() const {
    return peakDb.load(std::memory_order_relaxed);
}

float ChannelMetering::getRmsDb() const {
    return rmsDb.load(std::memory_order_relaxed);
}

float ChannelMetering::getGainReductionDb() const {
    return maxGainReductionDb.load(std::memory_order_relaxed);
}


/* -- Master Metering implementation -- */
MasterMetering::MasterMetering() {
    for (auto& band : rtaBandsAtom) {
        band.store(-120.0f);
    }
    reset();
}

void MasterMetering::prepare(double newSampleRate, int maxBlockSize) {
    sampleRate = newSampleRate;

    /**
     * The true-peak interpolator. Built here rather than in the constructor because
     * `initProcessing` needs a block size, and rebuilt on every `prepare` because a device
     * change is free to bring a different one.
     *
     * `2, 2` is two channels and factor 2 — `juce::dsp::Oversampling` takes the factor as a
     * power of two, so this is **4×**: the same rate the limiter works at, and BS.1770-4's
     * minimum for a true-peak measurement.
     */
    tpOversampler = std::make_unique<juce::dsp::Oversampling<float>>(
        2, 2,
        juce::dsp::Oversampling<float>::filterHalfBandFIREquiripple,
        true);
    tpOversampler->initProcessing(static_cast<size_t>(maxBlockSize));
    tpScratchL.assign(static_cast<size_t>(maxBlockSize), 0.0f);
    tpScratchR.assign(static_cast<size_t>(maxBlockSize), 0.0f);

    kFilterL.prepare(sampleRate);
    kFilterR.prepare(sampleRate);
    
    // Momentary: 400ms. We decimate by 128 samples, so 400ms is ~150 block history.
    msWindowSizeSamples = 150;
    msHistory.assign(msWindowSizeSamples, 0.0f);
    msHistoryWriteIdx = 0;
    msSum = 0.0;

    // Short-term: 3 seconds. 3000ms is ~1125 block history.
    stWindowSizeSamples = 1125;
    stHistory.assign(stWindowSizeSamples, 0.0f);
    stHistoryWriteIdx = 0;
    stSum = 0.0;

    rtaInputBuffer.resize(RtaFftSize, 0.0f);
    rtaFftBuffer.resize(RtaFftSize * 2, 0.0f);
    rtaWriteIdx = 0;

    /**
     * **The same defect `ChannelMetering` was already fixed for, in the other half of this
     * file, still sitting here.**
     *
     * `exp(-1 / (fs * 0.5))` is a per-*sample* coefficient for a 500 ms release, and it was
     * applied once per **block** below. At 48 kHz with 480-sample blocks that stretches the
     * release by a factor of 480: **500 ms becomes about four minutes.**
     *
     * It was found by running the engine with no input connected and watching the bridge:
     * `masterL` sat at −51 dBFS and fell by 0.00006 in six seconds, on a desk that had
     * nothing plugged into it. A master meter that keeps reporting a signal minutes after
     * the signal stopped is the frozen-meter failure the bridge's own `STALE_MS` exists to
     * avoid, arriving from inside the engine instead of from the wire.
     *
     * The coefficient is derived per block in `processBlock` now, from `numSamples`, exactly
     * as `ChannelMetering` does — which also makes it correct at any block size, something a
     * value baked in `prepare()` cannot be when the host is free to change it.
     */

    reset();
}

void MasterMetering::reset() {
    tpL = 0.0f;
    tpR = 0.0f;
    truePeakDbL.store(-120.0f);
    truePeakDbR.store(-120.0f);
    momentaryLufs.store(-120.0f);
    shortTermLufs.store(-120.0f);
    limiterGr.store(0.0f);
    
    std::fill(msHistory.begin(), msHistory.end(), 0.0f);
    msHistoryWriteIdx = 0;
    msSum = 0.0;

    std::fill(stHistory.begin(), stHistory.end(), 0.0f);
    stHistoryWriteIdx = 0;
    stSum = 0.0;

    std::fill(rtaInputBuffer.begin(), rtaInputBuffer.end(), 0.0f);
    rtaWriteIdx = 0;

    kFilterL.reset();
    kFilterR.reset();

    // The interpolator carries state across blocks like any filter. Left over from the
    // previous device, its ringing is measured as this one's first peak.
    if (tpOversampler != nullptr) tpOversampler->reset();
}

void MasterMetering::processBlock(const float** stereoBuffer, int numSamples, float limiterGrDb) {
    float peakL = 0.0f;
    float peakR = 0.0f;

    double blockKPowerL = 0.0;
    double blockKPowerR = 0.0;

    /**
     * **True peak, at 4×, before anything else touches the block.**
     *
     * This used to be `peakL = max(peakL, abs(xl))` inside the loop below — a sample peak
     * under a true-peak name. See the oversampler's declaration in `Metering.h` for what was
     * wrong with that and why the interpolator here is FIR rather than the limiter's IIR.
     *
     * Measure-only: `processSamplesUp` and no matching `processSamplesDown`. The oversampled
     * block is scanned and discarded, so no filtered audio ever leaves this class — which is
     * also what makes it safe to use a longer FIR here than the audio path could afford.
     *
     * Falls back to the sample peak if `prepare` has not run. That is a *worse* reading
     * rather than no reading, and it is the right way round: a meter that reads slightly low
     * is recoverable, one that reads −120 while audio is playing looks exactly like silence.
     */
    if (tpOversampler != nullptr && numSamples <= static_cast<int>(tpScratchL.size())) {
        std::copy(stereoBuffer[0], stereoBuffer[0] + numSamples, tpScratchL.begin());
        std::copy(stereoBuffer[1], stereoBuffer[1] + numSamples, tpScratchR.begin());

        float* tpChannels[2] = { tpScratchL.data(), tpScratchR.data() };
        juce::dsp::AudioBlock<float> tpBlock(tpChannels, 2, static_cast<size_t>(numSamples));
        auto osBlock = tpOversampler->processSamplesUp(tpBlock);

        const auto osSamples = static_cast<int>(osBlock.getNumSamples());
        const float* osL = osBlock.getChannelPointer(0);
        const float* osR = osBlock.getChannelPointer(1);
        for (int i = 0; i < osSamples; ++i) {
            peakL = std::max(peakL, std::abs(osL[i]));
            peakR = std::max(peakR, std::abs(osR[i]));
        }
    } else {
        for (int i = 0; i < numSamples; ++i) {
            peakL = std::max(peakL, std::abs(stereoBuffer[0][i]));
            peakR = std::max(peakR, std::abs(stereoBuffer[1][i]));
        }
    }

    for (int i = 0; i < numSamples; ++i) {
        float xl = stereoBuffer[0][i];
        float xr = stereoBuffer[1][i];

        // 2. K-Weighting filter processing
        float kL = kFilterL.processSample(xl);
        float kR = kFilterR.processSample(xr);

        blockKPowerL += kL * kL;
        blockKPowerR += kR * kR;

        // 3. Push to RTA input buffer (mono sum)
        rtaInputBuffer[rtaWriteIdx] = (xl + xr) * 0.5f;
        rtaWriteIdx++;
        if (rtaWriteIdx >= RtaFftSize) {
            processRta();
            rtaWriteIdx = 0;
        }
    }

    // One block's worth of time, so the release constant means what it says at any block
    // size. See the note in prepare(): this used to be a per-sample coefficient applied once
    // per block, which turned a 500 ms release into roughly four minutes.
    const float blockSec = numSamples > 0
        ? static_cast<float>(numSamples) / static_cast<float>(sampleRate)
        : 0.0f;
    tpReleaseCoef = std::exp(-blockSec / kTruePeakReleaseSec);

    // Smooth True Peak values
    if (peakL > tpL) tpL = peakL; else tpL = peakL + tpReleaseCoef * (tpL - peakL);
    if (peakR > tpR) tpR = peakR; else tpR = peakR + tpReleaseCoef * (tpR - peakR);

    truePeakDbL.store(juce::Decibels::gainToDecibels(tpL, -120.0f), std::memory_order_relaxed);
    truePeakDbR.store(juce::Decibels::gainToDecibels(tpR, -120.0f), std::memory_order_relaxed);
    limiterGr.store(limiterGrDb, std::memory_order_relaxed);

    // 4. LUFS decimation integration (every block)
    if (numSamples > 0) {
        double blockMeanPower = (blockKPowerL + blockKPowerR) / (2.0 * numSamples);

        // Slide Momentary Window
        msSum -= msHistory[msHistoryWriteIdx];
        msHistory[msHistoryWriteIdx] = static_cast<float>(blockMeanPower);
        msSum += blockMeanPower;
        msHistoryWriteIdx = (msHistoryWriteIdx + 1) % msWindowSizeSamples;

        // Slide Short-term Window
        stSum -= stHistory[stHistoryWriteIdx];
        stHistory[stHistoryWriteIdx] = static_cast<float>(blockMeanPower);
        stSum += blockMeanPower;
        stHistoryWriteIdx = (stHistoryWriteIdx + 1) % stWindowSizeSamples;

        // Convert mean powers to LUFS
        double mMean = msSum / msWindowSizeSamples;
        double stMean = stSum / stWindowSizeSamples;

        // LUFS Formula: -0.691 + 10 * log10(Power)
        float mLufsVal = -0.691f + 10.0f * std::log10(std::max(mMean, 1e-12));
        float stLufsVal = -0.691f + 10.0f * std::log10(std::max(stMean, 1e-12));

        momentaryLufs.store(mLufsVal, std::memory_order_relaxed);
        shortTermLufs.store(stLufsVal, std::memory_order_relaxed);
    }
}

void MasterMetering::processRta() {
    // Hann window
    std::fill(rtaFftBuffer.begin(), rtaFftBuffer.end(), 0.0f);
    for (int i = 0; i < RtaFftSize; ++i) {
        float win = 0.5f * (1.0f - std::cos(2.0f * 3.14159265f * i / (RtaFftSize - 1)));
        rtaFftBuffer[i] = rtaInputBuffer[i] * win;
    }

    rtaFft.performFrequencyOnlyForwardTransform(rtaFftBuffer.data());

    // Map FFT bins to 31 ISO frequencies of GraphicEQ
    const float binWidth = static_cast<float>(sampleRate) / RtaFftSize;
    
    // Center frequencies list
    const auto& freqs = GraphicEQ::IsoFrequencies;
    
    for (int band = 0; band < NumRtaBands; ++band) {
        float fc = freqs[band];
        // 1/3 octave band edges: low = fc * 0.8909, high = fc * 1.1225
        float fLow = fc * 0.8908987f;
        float fHigh = fc * 1.122462f;

        int binLow = std::max(1, static_cast<int>(std::round(fLow / binWidth)));
        int binHigh = std::min(RtaFftSize / 2, static_cast<int>(std::round(fHigh / binWidth)));
        // A band narrower than one bin can round to binLow > binHigh, and the loop below
        // then runs zero times and leaves `maxVal` at 0 — reporting -120 dBFS for a band
        // sitting in the middle of a loud signal. One dead bar beside two loud ones reads
        // as a notch that nothing placed.
        binHigh = std::max(binHigh, binLow);
        
        float maxVal = 0.0f;
        for (int b = binLow; b <= binHigh; ++b) {
            maxVal = std::max(maxVal, rtaFftBuffer[b]);
        }
        
        // Convert to dB
        float db = juce::Decibels::gainToDecibels(maxVal / RtaFftSize, -120.0f);
        rtaBandsAtom[band].store(db, std::memory_order_relaxed);
    }
}

float MasterMetering::getTruePeakDbL() const { return truePeakDbL.load(std::memory_order_relaxed); }
float MasterMetering::getTruePeakDbR() const { return truePeakDbR.load(std::memory_order_relaxed); }
float MasterMetering::getMomentaryLufs() const { return momentaryLufs.load(std::memory_order_relaxed); }
float MasterMetering::getShortTermLufs() const { return shortTermLufs.load(std::memory_order_relaxed); }
float MasterMetering::getLimiterGrDb() const { return limiterGr.load(std::memory_order_relaxed); }

std::array<float, MasterMetering::NumRtaBands> MasterMetering::getRtaBains() const {
    std::array<float, NumRtaBands> list;
    for (int i = 0; i < NumRtaBands; ++i) {
        list[i] = rtaBandsAtom[i].load(std::memory_order_relaxed);
    }
    return list;
}

} // namespace dsp
