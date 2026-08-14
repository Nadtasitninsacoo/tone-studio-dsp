#pragma once

#include "FilterPrimitives.h"
#include <juce_core/juce_core.h>
#include <juce_dsp/juce_dsp.h>
#include <array>
#include <atomic>
#include <vector>

namespace dsp {

class FeedbackSuppressor : public juce::Thread {
public:
    static constexpr int NumFixedSlots = 8;
    static constexpr int NumDynamicSlots = 8;
    static constexpr int TotalSlots = NumFixedSlots + NumDynamicSlots;
    static constexpr int FftOrder = 11; // 2048 points
    static constexpr int FftSize = 1 << FftOrder;

    struct NotchConfig {
        std::atomic<float> frequency { 0.0f };
        std::atomic<float> q { 15.0f };
        std::atomic<float> gainDb { 0.0f };
        std::atomic<bool> enabled { false };
    };

    FeedbackSuppressor();
    ~FeedbackSuppressor() override;

    void prepare(double sampleRate, int maxBlockSize);
    void reset();

    // Control parameters
    void setEnabled(bool enabled);
    void setDetectionEnabled(bool enabled);
    void setParams(float q, float depthDb, float holdMs, float releaseMs);
    
    void clearDynamicSlots();
    void clearAllSlots();

    /**
     * Park a notch in one of the fixed slots, permanently, until it is cleared.
     *
     * The eight fixed slots had **no way to be set**. `Main.cpp` describes them as
     * "frequencies somebody chose to hold down permanently" and nothing anywhere could
     * choose one, so a third of this class was unreachable — and `clearDynamicSlots()` exists
     * specifically to protect a half that could never be filled.
     *
     * `slot` is 0-based within the fixed range. Returns false for an out-of-range slot rather
     * than silently writing somewhere else. Frequency, Q and depth are clamped here, because
     * this is reachable from the control plane and every other setter in this class clamps.
     */
    bool setFixedNotch(int slot, float frequencyHz, float q, float gainDb);
    /** Release one fixed slot. Same 0-based range as `setFixedNotch`. */
    bool clearFixedNotch(int slot);

    // Thread function for background detection
    void run() override;

    // Audio thread process
    float processSample(float input);

    /** Mono. `numSamples` is a count of floats. */
    void processBlock(float* buffer, int numSamples);

    /**
     * Interleaved stereo. `numFrames` is a count of **frames**, so the buffer holds
     * `2 * numFrames` floats — the same contract as `Limiter::processBlock`.
     *
     * ---------------------------------------------------------------------------
     * WHY THIS EXISTS RATHER THAN REUSING processBlock()
     *
     * `MixingEngine` called the mono overload with an interleaved buffer and a frame
     * count, which was wrong twice over and silent both times:
     *
     *  1. **Half of every block was never suppressed.** The buffer holds `2 * numFrames`
     *     floats; the mono path walks `numFrames` of them, so the back half of each block
     *     went to the crossover untouched.
     *  2. **The half it did touch was filtered as one interleaved stream.** A biquad is
     *     stateful, so feeding it L,R,L,R runs one filter over two signals at once: the
     *     notch does not land on the frequency it reports, and L leaks into R.
     *
     * A stereo notch therefore needs **two filter banks**, which is why `filtersR` exists.
     * `Limiter` had this right — it deinterleaves into `scratchL`/`scratchR` — and the
     * difference between the two neighbours in the master chain is what showed this was a
     * bug rather than a design.
     *
     * Detection is fed the mono sum: a howl is a room resonance and arrives on both sides,
     * and one FFT is half the work of two.
     */
    void processBlockInterleaved(float* buffer, int numFrames);

    // Control from the web UI (see bridge.js for the OSC addresses)
    /** 0..1. Drives the peak-ratio and level thresholds in processDetection(). */
    void setSensitivity(float s);
    /** 1..NumDynamicSlots. Ceiling on automatic notches; fixed slots are unaffected. */
    void setMaxDynamicNotches(int n);

    // Metering / UI info
    std::vector<float> getActiveNotchFrequencies() const;

    /**
     * Per-slot readout for the web UI.
     *
     * getActiveNotchFrequencies() returns frequencies only, which is not enough to draw the
     * page: it shows a depth per notch and an active lamp per slot, and inventing either
     * from a frequency would be a fabricated reading on the one screen whose job is to
     * report what the engine did to the sound.
     */
    struct NotchReadout {
        float frequencyHz { 0.0f };
        float gainDb { 0.0f };
        bool active { false };
    };
    std::vector<NotchReadout> getNotchReadout() const;

private:
    double sampleRate { 48000.0 };
    bool isEnabled { false };
    std::atomic<bool> detectionEnabled { false };

    // Notch configurations
    std::array<NotchConfig, TotalSlots> configs;
    /** Left, and the mono path. */
    std::array<FilterPrimitives, TotalSlots> filters;
    /** Right. A biquad's state belongs to one signal — see `processBlockInterleaved`. */
    std::array<FilterPrimitives, TotalSlots> filtersR;
    std::array<bool, TotalSlots> activeSlots { false };

    /**
     * Pull the atomic configs into both filter banks and refresh `activeSlots`.
     *
     * Once per block, from the audio thread, shared by every process entry point — so the
     * mono and stereo paths cannot drift into disagreeing about which slots are live.
     */
    void updateActiveSlots();
    /** Hand a block to the detection thread. Non-blocking; drops rather than waits. */
    void pushToDetection(const float* mono, int numSamples);
    /**
     * Empty the FIFO and the filter states.
     *
     * Callers **must** have stopped the detection thread: this rewinds indices and refills a
     * buffer that `run()` reads. `prepare()` and `reset()` are the only two, and both do.
     */
    void clearState();

    // Parameters for suppressor
    std::atomic<float> notchQ { 15.0f };
    std::atomic<float> notchDepthDb { -12.0f };
    std::atomic<float> holdTimeMs { 5000.0f };
    std::atomic<float> releaseTimeMs { 2000.0f };
    std::atomic<float> sensitivity { 0.5f };
    std::atomic<int> maxDynamicNotches { NumDynamicSlots };

    // SPSC FIFO for passing audio to detection thread
    juce::AbstractFifo fifo { FftSize * 4 };
    std::vector<float> fifoBuffer;

    // FFT & Windowing
    juce::dsp::FFT fft { FftOrder };
    std::vector<float> fftBuffer;
    std::vector<float> windowBuffer;
    /** The detection thread's read window. Sized in prepare(), never on the audio thread. */
    std::vector<float> detectionWindow;
    /** Mono sum of one stereo block, for the detection FIFO. Sized in prepare(). */
    std::vector<float> monoScratch;

    // Dynamic slot tracking in background thread
    struct DynamicSlotTracker {
        float frequency { 0.0f };
        float currentDepthDb { 0.0f };
        float holdTimerMs { 0.0f };
        float releaseTimerMs { 0.0f };
        bool active { false };
    };
    std::array<DynamicSlotTracker, NumDynamicSlots> dynamicTrackers;

    // Helper functions for detection thread
    void processDetection();
    void updateDynamicSlots(float deltaMs);
    void addNotch(float freq);
};

} // namespace dsp
