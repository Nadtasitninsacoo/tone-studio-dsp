#pragma once

#include "FilterPrimitives.h"
#include <array>
#include <atomic>

namespace dsp {

class GraphicEQ {
public:
    static constexpr int NumBands = 31;
    
    // Standard ISO 1/3 octave frequencies in Hz
    static constexpr std::array<float, NumBands> IsoFrequencies {
        20.0f, 25.0f, 31.5f, 40.0f, 50.0f, 63.0f, 80.0f, 100.0f, 125.0f, 160.0f, 
        200.0f, 250.0f, 315.0f, 400.0f, 500.0f, 630.0f, 800.0f, 1000.0f, 1250.0f, 
        1600.0f, 2000.0f, 2500.0f, 3150.0f, 4000.0f, 5000.0f, 6300.0f, 8000.0f, 
        10000.0f, 12500.0f, 16000.0f, 20000.0f
    };

    GraphicEQ();
    ~GraphicEQ() = default;

    void prepare(double sampleRate, int maxBlockSize);
    void reset();

    void setBandGain(int bandIndex, float gainDb);
    float getBandGain(int bandIndex) const;
    
    void setAllGains(const std::array<float, NumBands>& gainsDb);

    /**
     * Audio thread. `processBlock` is the entry point that picks up pending gain changes;
     * `processSample` on its own does not, so a caller driving this one sample at a time must
     * accept that a gain set from another thread arrives at the next `processBlock`.
     */
    float processSample(float input);
    void processBlock(float* buffer, int numSamples);

private:
    /**
     * **Band gains cross threads as atomics, and the filters are only ever touched by the
     * audio thread.**
     *
     * `setBandGain` used to recompute a filter's coefficients from the message thread, from
     * inside the OSC handler, while `processSample` was running that same filter. It is the
     * one control on this desk somebody drags continuously — 31 sliders, at frame rate — so
     * it was also the most exercised instance of the problem.
     *
     * The pattern is the one `FeedbackSuppressor::updateActiveSlots` already uses: the writer
     * stores, the audio thread picks the change up once at the top of a block. `dirty` keeps
     * that free — an untouched EQ costs one relaxed load per block, not 31 coefficient
     * recalculations.
     */
    std::array<FilterPrimitives, NumBands> filters;
    std::array<std::atomic<float>, NumBands> bandGainsDb;
    std::atomic<bool> gainsDirty { false };

    /** Pull any pending gains into the filters. Audio thread only. */
    void applyPendingGains();
};

} // namespace dsp
