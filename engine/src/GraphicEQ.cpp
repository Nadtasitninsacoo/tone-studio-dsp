#include "GraphicEQ.h"

namespace dsp {

/** Constant-Q for 1/3 octave. One value, not three copies of it. */
static constexpr float BandQ = 4.318f;

GraphicEQ::GraphicEQ() {
    for (int i = 0; i < NumBands; ++i) {
        filters[i].setType(FilterPrimitives::Type::Peaking);
        bandGainsDb[i].store(0.0f, std::memory_order_relaxed);
    }
    reset();
}

void GraphicEQ::prepare(double sampleRate, int maxBlockSize) {
    for (int i = 0; i < NumBands; ++i) {
        filters[i].prepare(sampleRate, maxBlockSize);
        filters[i].setParameters(IsoFrequencies[i], BandQ,
                                 bandGainsDb[i].load(std::memory_order_relaxed));
    }
    // Whatever was pending has just been applied by the loop above.
    gainsDirty.store(false, std::memory_order_relaxed);
}

void GraphicEQ::reset() {
    for (auto& filter : filters) {
        filter.reset();
    }
}

void GraphicEQ::setBandGain(int bandIndex, float gainDb) {
    if (bandIndex < 0 || bandIndex >= NumBands) return;
    /**
     * Stores; it does not touch a filter.
     *
     * This runs on the OSC message thread while the audio thread is inside
     * `processSample` for the very same filter, and it used to recompute that filter's
     * coefficients from here. The audio thread picks the change up at the top of the next
     * block instead — see `applyPendingGains`.
     *
     * `release` paired with the `acquire` on the reader, so the gain is visible before the
     * flag that advertises it. Ordered the other way round, the reader can see `dirty` and
     * then read the *old* gain, and the move is lost until something else moves.
     */
    bandGainsDb[bandIndex].store(gainDb, std::memory_order_relaxed);
    gainsDirty.store(true, std::memory_order_release);
}

float GraphicEQ::getBandGain(int bandIndex) const {
    if (bandIndex < 0 || bandIndex >= NumBands) return 0.0f;
    return bandGainsDb[bandIndex].load(std::memory_order_relaxed);
}

void GraphicEQ::setAllGains(const std::array<float, NumBands>& gainsDb) {
    for (int i = 0; i < NumBands; ++i) {
        bandGainsDb[i].store(gainsDb[i], std::memory_order_relaxed);
    }
    gainsDirty.store(true, std::memory_order_release);
}

void GraphicEQ::applyPendingGains() {
    // One relaxed-ish load per block when nothing has moved, which is the common case: an
    // EQ that is being dragged is the exception, not the rule.
    if (!gainsDirty.exchange(false, std::memory_order_acquire)) return;

    for (int i = 0; i < NumBands; ++i) {
        filters[i].setParameters(IsoFrequencies[i], BandQ,
                                 bandGainsDb[i].load(std::memory_order_relaxed));
    }
}

float GraphicEQ::processSample(float input) {
    float output = input;
    for (int i = 0; i < NumBands; ++i) {
        output = filters[i].processSample(output);
    }
    return output;
}

void GraphicEQ::processBlock(float* buffer, int numSamples) {
    applyPendingGains();
    for (int i = 0; i < numSamples; ++i) {
        buffer[i] = processSample(buffer[i]);
    }
}

} // namespace dsp
