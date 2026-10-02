#pragma once

#include <vector>

namespace dsp {

/**
 * A plain RBJ-cookbook biquad, transposed direct form II.
 *
 * Not `FilterPrimitives`: that filter clamps its corner to 20 Hz, and the envelope smoother
 * here sits at 12 Hz — at 20 Hz a rectified 55 Hz kick leaves ripple a third louder on the
 * bass's gain. It is also the exact filter the web desk's `BiquadFilterNode` is, so the two
 * duck identically.
 */
struct LowEndBiquad {
    enum class Kind { LowPass, HighPass };
    void set(Kind kind, double sampleRate, double hz, double q);
    void reset() { z1 = z2 = 0.0; }
    float process(float x) {
        const double y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return static_cast<float>(y);
    }
    double b0 { 1.0 }, b1 { 0.0 }, b2 { 0.0 }, a1 { 0.0 }, a2 { 0.0 };
    double z1 { 0.0 }, z2 { 0.0 };
};

/**
 * The low end of the master: a kick→bass sidechain duck and a mono-bass stage.
 *
 * The same two moves the web desk has (`lib/lowEnd.ts` in tone-studio-web), with the same
 * numbers, so a show run through this engine gets the low end the web desk was voiced with:
 *
 * - **Sidechain.** The target channel dips while the key channel hits: the key's post-fader
 *   signal → 150 Hz low-pass → |x| → 12 Hz envelope → a reduction clamped to 0…1 → the target's
 *   gain `1 − depth·reduction`. Clamped so a kick far louder than expected ducks by exactly
 *   `depthDb` and never drives the gain negative (a polarity flip per hit).
 * - **Mono bass.** Below the corner the side signal (L−R) is removed through a Linkwitz-Riley
 *   high-pass; the mid (L+R) is untouched, so mono material is bit-identical through it.
 *
 * Both are off by default, as on the web desk.
 */
class SidechainDuck {
public:
    static constexpr float KeyLowpassHz = 150.0f;
    static constexpr float EnvelopeHz = 12.0f;
    /** The envelope level that counts as a full hit — a kick around −10 dBFS on its strip. */
    static constexpr float FullHitEnvelope = 0.2f;
    static constexpr float MaxDepthDb = 18.0f;

    void prepare(double sampleRate, int maxBlockSize);
    void reset();

    void setDepthDb(float db);
    float getDepthDb() const { return depthDb; }

    /** Run the detector over one block of the key channel's post-fader signal. */
    void processKey(const float* key, int numSamples);
    /** Mark the block as having no key (the key channel produced nothing this block). */
    void processSilence(int numSamples);
    /** The target's gain for sample `i` of the block just processed. */
    float gainAt(int i) const { return gains[static_cast<size_t>(i)]; }

    /** 1 at rest, `1 − fraction` at a full hit — the arithmetic, for the tests. */
    static float duckGain(float envelope, float depthDb);

private:
    LowEndBiquad keyFilter;
    LowEndBiquad envelope;
    double sampleRate { 48000.0 };
    float depthDb { 6.0f };
    std::vector<float> gains;
};

class MonoBass {
public:
    static constexpr float MinHz = 40.0f;
    static constexpr float MaxHz = 250.0f;

    void prepare(double sampleRate, int maxBlockSize);
    void reset();
    void setEnabled(bool on) { enabled = on; }
    bool isEnabled() const { return enabled; }
    void setFrequency(float hz);
    float getFrequency() const { return frequencyHz; }

    /** In place, on the two halves of the master. Does nothing when off. */
    void process(float* left, float* right, int numSamples);

private:
    LowEndBiquad sideA;
    LowEndBiquad sideB;
    double sampleRate { 48000.0 };
    bool enabled { false };
    float frequencyHz { 120.0f };
};

} // namespace dsp
