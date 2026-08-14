#include "FeedbackSuppressor.h"
#include <algorithm>
#include <cmath>

namespace dsp {

/**
 * ---------------------------------------------------------------------------
 * THE DETECTION THREAD DOES NOT RUN WHILE ITS BUFFERS ARE BEING REBUILT
 *
 * It used to be started **in the constructor**, before `prepare()` had sized a single
 * buffer. Two things followed, and neither is a rare race — both are certainties on an
 * ordinary startup:
 *
 *  - `prepare()` calls `resize()` on `fifoBuffer`, `fftBuffer` and `windowBuffer` while
 *    `run()` is reading them. A `std::vector` resize reallocates, so the reader is left
 *    holding freed memory: a **use-after-free on every device start and every device
 *    change** — which on Windows includes every USB re-enumeration.
 *  - Before that first `prepare()` every one of those buffers is empty, so anything the
 *    thread did with them indexed a zero-length vector.
 *
 * `stopThread` rather than a lock because none of this is a real-time path — `prepare` and
 * `reset` are called from device start/stop — and a lock would have to be held by `run()`
 * across the FFT, where the audio thread's FIFO writer would become the one waiting on it.
 * JUCE's `stopThread` signals and then `notify()`s, so the `wait(50)` in `run()` returns at
 * once; the timeout is a backstop, not the expected cost.
 * ------------------------------------------------------------------------- */

FeedbackSuppressor::FeedbackSuppressor()
    : juce::Thread("FeedbackSuppressorThread") {
    clearAllSlots();
    // Deliberately **not** started here — see the block above. There is nothing for it to
    // read until prepare() has sized its buffers, and starting it early is the bug.
}

FeedbackSuppressor::~FeedbackSuppressor() {
    stopThread(2000);
}

void FeedbackSuppressor::prepare(double newSampleRate, int maxBlockSize) {
    // Nothing may be reading these while they are reallocated.
    stopThread(2000);

    sampleRate = newSampleRate;

    // Allocate buffers
    fifoBuffer.resize(FftSize * 4, 0.0f);
    fftBuffer.resize(FftSize * 2, 0.0f);
    windowBuffer.resize(FftSize, 0.0f);
    // The detection thread's read window, so processDetection() does not allocate.
    detectionWindow.resize(FftSize, 0.0f);
    // One block's mono sum for the stereo path's detection feed. `std::max` because a device
    // can report a block size of 0 before it has started, and a zero-length scratch would
    // silently switch detection off on the interleaved path.
    monoScratch.assign(static_cast<size_t>(std::max(1, maxBlockSize)), 0.0f);
    
    // Create Hann window
    for (int i = 0; i < FftSize; ++i) {
        windowBuffer[i] = 0.5f * (1.0f - std::cos(2.0f * 3.14159265358979f * i / (FftSize - 1)));
    }
    
    // Prepare notch filters. Both banks — see `processBlockInterleaved`.
    for (auto& filter : filters) {
        filter.prepare(sampleRate, maxBlockSize);
    }
    for (auto& filter : filtersR) {
        filter.prepare(sampleRate, maxBlockSize);
    }

    clearState();

    // Now, and only now, is there something for it to read.
    // JUCE 7 moved this enum onto Thread itself; `juce::Priority` does not exist.
    // Background priority keeps FFT detection off the audio thread's back.
    startThread(juce::Thread::Priority::background);
}

void FeedbackSuppressor::reset() {
    /**
     * `reset()` is reachable on its own — `MasterBus::reset()` runs it from
     * `audioDeviceStopped`, and `MixingEngine`'s constructor runs it too. It refills
     * `fifoBuffer` and rewinds the FIFO's indices, both of which the detection thread is
     * reading, so it takes the same treatment as `prepare()`.
     *
     * Conditional on the thread already running, so a `reset()` before the first `prepare()`
     * does not start a thread against buffers that do not exist yet.
     */
    const bool wasRunning = isThreadRunning();
    if (wasRunning) stopThread(2000);

    clearState();

    if (wasRunning) startThread(juce::Thread::Priority::background);
}

void FeedbackSuppressor::clearState() {
    // Both callers stop the thread first. Nothing here is safe to run beside it.
    fifo.reset();
    std::fill(fifoBuffer.begin(), fifoBuffer.end(), 0.0f);
    for (auto& filter : filters) {
        filter.reset();
    }
    for (auto& filter : filtersR) {
        filter.reset();
    }
}

void FeedbackSuppressor::setEnabled(bool enabled) {
    isEnabled = enabled;
}

void FeedbackSuppressor::setDetectionEnabled(bool enabled) {
    detectionEnabled = enabled;
}

void FeedbackSuppressor::setParams(float q, float depthDb, float holdMs, float releaseMs) {
    // Against the filter's own range, not a second opinion about it. This used to clamp to
    // 5..50 while `FilterPrimitives` clamped to 0.1..18, so half of that range was fiction.
    notchQ = std::clamp(q, 5.0f, FilterPrimitives::MaxQ);
    notchDepthDb = std::clamp(depthDb, -24.0f, -6.0f);
    holdTimeMs = std::max(100.0f, holdMs);
    releaseTimeMs = std::max(100.0f, releaseMs);
}

void FeedbackSuppressor::clearDynamicSlots() {
    for (int i = NumFixedSlots; i < TotalSlots; ++i) {
        configs[i].enabled.store(false, std::memory_order_relaxed);
        configs[i].gainDb.store(0.0f, std::memory_order_relaxed);
    }
    
    // Reset trackers
    for (auto& tracker : dynamicTrackers) {
        tracker.active = false;
        tracker.frequency = 0.0f;
        tracker.currentDepthDb = 0.0f;
    }
}

void FeedbackSuppressor::clearAllSlots() {
    for (int i = 0; i < TotalSlots; ++i) {
        configs[i].enabled.store(false, std::memory_order_relaxed);
        configs[i].frequency.store(0.0f, std::memory_order_relaxed);
        configs[i].q.store(15.0f, std::memory_order_relaxed);
        configs[i].gainDb.store(0.0f, std::memory_order_relaxed);
    }
    clearDynamicSlots();
}

float FeedbackSuppressor::processSample(float input) {
    if (!isEnabled) {
        return input;
    }

    // Push sample to FIFO for detection thread (non-blocking write).
    // `fifoBuffer` is empty until prepare() has run, and a host is free to call process
    // before prepare — the guard is what makes that a no-op rather than a stray write.
    if (detectionEnabled.load(std::memory_order_relaxed) && !fifoBuffer.empty()) {
        int start1, size1, start2, size2;
        fifo.prepareToWrite(1, start1, size1, start2, size2);
        if (size1 > 0) {
            fifoBuffer[start1] = input;
            fifo.finishedWrite(1);
        }
    }

    // Apply notches in series
    float output = input;
    for (int i = 0; i < TotalSlots; ++i) {
        // Read configuration atomically (or use local cached state updated per block)
        // For per-sample processing, we read the activeSlots cached state
        if (activeSlots[i]) {
            output = filters[i].processSample(output);
        }
    }
    return output;
}

void FeedbackSuppressor::updateActiveSlots() {
    // Once per block, from the audio thread. The configs are atomics written by the
    // detection thread; the filters themselves are touched here and nowhere else.
    for (int i = 0; i < TotalSlots; ++i) {
        if (configs[i].enabled.load(std::memory_order_relaxed)) {
            const float freq = configs[i].frequency.load(std::memory_order_relaxed);
            const float q = configs[i].q.load(std::memory_order_relaxed);
            const float gain = configs[i].gainDb.load(std::memory_order_relaxed);

            filters[i].setType(FilterPrimitives::Type::Notch);
            filters[i].setParameters(freq, q, gain);
            filtersR[i].setType(FilterPrimitives::Type::Notch);
            filtersR[i].setParameters(freq, q, gain);
            activeSlots[i] = true;
        } else {
            activeSlots[i] = false;
        }
    }
}

void FeedbackSuppressor::pushToDetection(const float* mono, int numSamples) {
    if (!detectionEnabled.load(std::memory_order_relaxed)) return;
    if (fifoBuffer.empty()) return;

    int start1, size1, start2, size2;
    fifo.prepareToWrite(numSamples, start1, size1, start2, size2);
    // Drop the block rather than write part of one. A half-filled window has a step in the
    // middle of it, and a step is broadband — which is exactly what the peak-ratio test is
    // hunting for, so a partial write manufactures detections.
    if (size1 + size2 < numSamples) return;
    for (int i = 0; i < size1; ++i) fifoBuffer[start1 + i] = mono[i];
    for (int i = 0; i < size2; ++i) fifoBuffer[start2 + i] = mono[size1 + i];
    fifo.finishedWrite(numSamples);
}

void FeedbackSuppressor::processBlock(float* buffer, int numSamples) {
    if (!isEnabled) return;

    updateActiveSlots();
    pushToDetection(buffer, numSamples);

    for (int i = 0; i < numSamples; ++i) {
        float output = buffer[i];
        for (int s = 0; s < TotalSlots; ++s) {
            if (activeSlots[s]) {
                output = filters[s].processSample(output);
            }
        }
        buffer[i] = output;
    }
}

void FeedbackSuppressor::processBlockInterleaved(float* buffer, int numFrames) {
    if (!isEnabled) return;
    if (numFrames <= 0) return;

    updateActiveSlots();

    /**
     * Detection is fed the mono sum, written into the scratch buffer `prepare` sized.
     *
     * Not a local `std::vector`: this is the audio thread, where a heap allocation can block
     * on a lock another thread holds — the defect `Limiter` and `ReverbDelay` were each
     * fixed for once already. The sum rather than one side because a howl is a room
     * resonance and arrives on both, and one FFT is half the work of two.
     */
    if (detectionEnabled.load(std::memory_order_relaxed) && !monoScratch.empty()) {
        const int n = std::min(numFrames, static_cast<int>(monoScratch.size()));
        for (int i = 0; i < n; ++i) {
            monoScratch[i] = 0.5f * (buffer[2 * i] + buffer[2 * i + 1]);
        }
        pushToDetection(monoScratch.data(), n);
    }

    // Two banks over two signals. One bank over an interleaved stream is the bug this
    // function exists to replace — see the header.
    for (int i = 0; i < numFrames; ++i) {
        float l = buffer[2 * i];
        float r = buffer[2 * i + 1];
        for (int s = 0; s < TotalSlots; ++s) {
            if (activeSlots[s]) {
                l = filters[s].processSample(l);
                r = filtersR[s].processSample(r);
            }
        }
        buffer[2 * i] = l;
        buffer[2 * i + 1] = r;
    }
}

void FeedbackSuppressor::run() {
    const int sleepTimeMs = 50;
    while (!threadShouldExit()) {
        if (detectionEnabled.load(std::memory_order_relaxed)) {
            processDetection();
        }
        updateDynamicSlots(static_cast<float>(sleepTimeMs));
        wait(sleepTimeMs);
    }
}

void FeedbackSuppressor::processDetection() {
    int numReady = fifo.getNumReady();
    if (numReady < FftSize) return;

    // Nothing has been sized yet. prepare() owns every buffer here, and the detection thread
    // is started in the constructor — so this runs before prepare() on every startup.
    if (fifoBuffer.empty() || fftBuffer.empty() || windowBuffer.empty() || detectionWindow.empty()) {
        return;
    }

    // Read FftSize samples from FIFO into the window prepare() sized. This used to be a
    // local `std::vector<float>(FftSize)` — 2048 floats off the heap every 50 ms, for a
    // buffer whose size never changes.
    int start1, size1, start2, size2;
    fifo.prepareToRead(FftSize, start1, size1, start2, size2);
    if (size1 + size2 >= FftSize) {
        for (int i = 0; i < size1; ++i) detectionWindow[i] = fifoBuffer[start1 + i];
        for (int i = 0; i < size2; ++i) detectionWindow[size1 + i] = fifoBuffer[start2 + i];
        fifo.finishedRead(FftSize);
    } else {
        return;
    }

    // Apply Hann window and copy to FFT buffer
    std::fill(fftBuffer.begin(), fftBuffer.end(), 0.0f);
    for (int i = 0; i < FftSize; ++i) {
        fftBuffer[i] = detectionWindow[i] * windowBuffer[i];
    }

    // Run FFT
    fft.performFrequencyOnlyForwardTransform(fftBuffer.data());

    // Freq of bin i is: i * sampleRate / FftSize
    const float binWidth = static_cast<float>(sampleRate) / FftSize;
    const int minBin = static_cast<int>(100.0f / binWidth); // Check above 100 Hz
    const int maxBin = static_cast<int>(15000.0f / binWidth); // Check below 15 kHz
    const int searchRange = 15; // Local average window

    // Find peaks
    for (int i = minBin; i <= maxBin; ++i) {
        float magnitude = fftBuffer[i];
        
        // Ensure it is a local maximum
        if (magnitude > fftBuffer[i - 1] && magnitude > fftBuffer[i + 1] && magnitude > fftBuffer[i - 2] && magnitude > fftBuffer[i + 2]) {
            // Calculate local average
            float sum = 0.0f;
            int count = 0;
            for (int k = i - searchRange; k <= i + searchRange; ++k) {
                if (k >= 0 && k < FftSize / 2 && std::abs(k - i) > 2) {
                    sum += fftBuffer[k];
                    count++;
                }
            }
            float localAvg = count > 0 ? sum / count : 1.0f;
            
            // Peak ratio (linear scale, e.g. 10.0 corresponds to 20 dB above average)
            float ratio = magnitude / std::max(localAvg, 1e-6f);
            
            // Min magnitude threshold to avoid feedback on silence
            float levelDb = juce::Decibels::gainToDecibels(magnitude / FftSize, -120.0f);
            
            /**
             * Both thresholds were hardcoded (ratio > 12, level > -45 dB), so the web UI's
             * Sensitivity slider had nothing to move. Mapped here rather than in the bridge:
             * the bridge translates addresses, it does not own the DSP's tuning.
             *
             * Higher sensitivity lowers both bars — a smaller peak counts, and a quieter one.
             */
            const float sens = juce::jlimit(0.0f, 1.0f, sensitivity.load(std::memory_order_relaxed));
            const float ratioThreshold = juce::jmap(sens, 20.0f, 5.0f);
            const float levelThresholdDb = juce::jmap(sens, -35.0f, -55.0f);

            if (ratio > ratioThreshold && levelDb > levelThresholdDb) {
                float freq = i * binWidth;
                addNotch(freq);
            }
        }
    }
}

void FeedbackSuppressor::addNotch(float freq) {
    // 1. Check if we already have a notch near this frequency
    for (int i = 0; i < TotalSlots; ++i) {
        if (configs[i].enabled.load(std::memory_order_relaxed)) {
            float activeFreq = configs[i].frequency.load(std::memory_order_relaxed);
            // If within 3% of frequency, reset its hold time
            if (std::abs(activeFreq - freq) / activeFreq < 0.03f) {
                if (i >= NumFixedSlots) {
                    int dynamicIdx = i - NumFixedSlots;
                    dynamicTrackers[dynamicIdx].holdTimerMs = holdTimeMs.load(std::memory_order_relaxed);
                    dynamicTrackers[dynamicIdx].currentDepthDb = notchDepthDb.load(std::memory_order_relaxed);
                    configs[i].gainDb.store(dynamicTrackers[dynamicIdx].currentDepthDb, std::memory_order_relaxed);
                }
                return;
            }
        }
    }

    // 2. Find an empty dynamic slot, within the ceiling the operator set.
    const int limit = juce::jlimit(1, NumDynamicSlots, maxDynamicNotches.load(std::memory_order_relaxed));

    int active = 0;
    for (int i = 0; i < limit; ++i)
        if (dynamicTrackers[i].active) ++active;

    int slotToUse = -1;
    for (int i = 0; i < limit; ++i) {
        if (!dynamicTrackers[i].active) {
            slotToUse = i;
            break;
        }
    }

    // If all slots are full, overwrite the one with the lowest hold timer
    if (slotToUse == -1) {
        float minHold = 1e9f;
        for (int i = 0; i < limit; ++i) {
            if (dynamicTrackers[i].holdTimerMs < minHold) {
                minHold = dynamicTrackers[i].holdTimerMs;
                slotToUse = i;
            }
        }
    }

    if (slotToUse != -1) {
        // Set tracker
        dynamicTrackers[slotToUse].active = true;
        dynamicTrackers[slotToUse].frequency = freq;
        dynamicTrackers[slotToUse].currentDepthDb = notchDepthDb.load(std::memory_order_relaxed);
        dynamicTrackers[slotToUse].holdTimerMs = holdTimeMs.load(std::memory_order_relaxed);
        dynamicTrackers[slotToUse].releaseTimerMs = releaseTimeMs.load(std::memory_order_relaxed);

        // Update atomic configs
        int configIdx = NumFixedSlots + slotToUse;
        configs[configIdx].frequency.store(freq, std::memory_order_relaxed);
        configs[configIdx].q.store(notchQ.load(std::memory_order_relaxed), std::memory_order_relaxed);
        configs[configIdx].gainDb.store(dynamicTrackers[slotToUse].currentDepthDb, std::memory_order_relaxed);
        configs[configIdx].enabled.store(true, std::memory_order_relaxed);
    }
}

void FeedbackSuppressor::updateDynamicSlots(float deltaMs) {
    for (int i = 0; i < NumDynamicSlots; ++i) {
        if (dynamicTrackers[i].active) {
            int configIdx = NumFixedSlots + i;
            
            if (dynamicTrackers[i].holdTimerMs > 0.0f) {
                dynamicTrackers[i].holdTimerMs -= deltaMs;
            } else {
                // Releasing phase
                if (dynamicTrackers[i].currentDepthDb < 0.0f) {
                    // Ramp gain towards 0 dB
                    float releaseRate = (configs[configIdx].gainDb.load() / (releaseTimeMs / deltaMs)); // approximate ramp step
                    float step = -notchDepthDb.load(std::memory_order_relaxed) / (releaseTimeMs.load(std::memory_order_relaxed) / deltaMs);
                    dynamicTrackers[i].currentDepthDb += step;
                    
                    if (dynamicTrackers[i].currentDepthDb >= 0.0f) {
                        dynamicTrackers[i].currentDepthDb = 0.0f;
                        dynamicTrackers[i].active = false;
                        configs[configIdx].enabled.store(false, std::memory_order_relaxed);
                    }
                    configs[configIdx].gainDb.store(dynamicTrackers[i].currentDepthDb, std::memory_order_relaxed);
                }
            }
        }
    }
}

bool FeedbackSuppressor::setFixedNotch(int slot, float frequencyHz, float q, float gainDb) {
    if (slot < 0 || slot >= NumFixedSlots) return false;

    /**
     * Clamped to what the filter can actually deliver, not to what this class would like.
     *
     * Every bound here is `FilterPrimitives`' own, so a caller that reads the config back
     * sees the notch it is actually going to get. This is where the Q mismatch was found:
     * two clamps with different opinions, and the narrower one winning in silence.
     */
    configs[slot].frequency.store(juce::jlimit(FilterPrimitives::MinFreqHz, FilterPrimitives::MaxFreqHz, frequencyHz), std::memory_order_relaxed);
    configs[slot].q.store(juce::jlimit(FilterPrimitives::MinQ, FilterPrimitives::MaxQ, q), std::memory_order_relaxed);
    configs[slot].gainDb.store(juce::jlimit(FilterPrimitives::MinGainDb, 0.0f, gainDb), std::memory_order_relaxed);
    // Last, so the audio thread never sees a slot switched on with a frequency of zero.
    configs[slot].enabled.store(true, std::memory_order_release);
    return true;
}

bool FeedbackSuppressor::clearFixedNotch(int slot) {
    if (slot < 0 || slot >= NumFixedSlots) return false;
    configs[slot].enabled.store(false, std::memory_order_relaxed);
    configs[slot].gainDb.store(0.0f, std::memory_order_relaxed);
    return true;
}

void FeedbackSuppressor::setSensitivity(float s) {
    sensitivity.store(juce::jlimit(0.0f, 1.0f, s), std::memory_order_relaxed);
}

void FeedbackSuppressor::setMaxDynamicNotches(int n) {
    maxDynamicNotches.store(juce::jlimit(1, NumDynamicSlots, n), std::memory_order_relaxed);
}

std::vector<FeedbackSuppressor::NotchReadout> FeedbackSuppressor::getNotchReadout() const {
    std::vector<NotchReadout> list;
    list.reserve(TotalSlots);
    for (int i = 0; i < TotalSlots; ++i) {
        NotchReadout r;
        r.frequencyHz = configs[i].frequency.load(std::memory_order_relaxed);
        r.gainDb = configs[i].gainDb.load(std::memory_order_relaxed);
        r.active = configs[i].enabled.load(std::memory_order_relaxed);
        list.push_back(r);
    }
    return list;
}

std::vector<float> FeedbackSuppressor::getActiveNotchFrequencies() const {
    std::vector<float> list;
    for (int i = 0; i < TotalSlots; ++i) {
        if (configs[i].enabled.load(std::memory_order_relaxed)) {
            list.push_back(configs[i].frequency.load(std::memory_order_relaxed));
        }
    }
    return list;
}

} // namespace dsp
