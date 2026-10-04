#pragma once

#include "ParametricEQ.h"
#include <array>
#include <string_view>

namespace dsp {

/**
 * One physical output's speaker processor — what a system processor does between the desk and an
 * amplifier: gain, a six-band parametric EQ, a high-pass and a low-pass (Butterworth or
 * Linkwitz-Riley, 6…48 dB/oct), polarity, and a protection limiter. The alignment delay is the
 * engine's existing `outputDelays`, applied after this, so a delay set in metres or milliseconds
 * on the web page lands in the one place an output is already delayed.
 *
 * Three decisions worth not undoing:
 *
 * - **Untouched is bit-exact pass-through.** Every section ships off, and `process` returns before
 *   touching a sample while nothing is on. An output nobody has opened this page for sounds
 *   exactly as it did in 1.0.19.
 * - **The limiter has no look-ahead and no oversampling, so no latency, whether on or off.** The
 *   master `Limiter` adds both, and on a speaker processor that would mean a sub with its limiter
 *   on arriving later than tops with theirs off — misaligning the very crossover the delay is set
 *   to align. Instant attack with a hard sample ceiling; protection, not loudness.
 * - **Filter coefficients are computed on the control path and the sections are TPT**, so a moved
 *   corner cannot make a section unstable mid-block, and a Q of 0.5 (Linkwitz-Riley second order)
 *   is as safe as 1.3 (Butterworth eighth order).
 */
class SpeakerProcessor {
public:
    enum class FilterKind : int { Off = 0, Butterworth = 1, LinkwitzRiley = 2 };
    enum class EqShape : int { Peaking = 0, LowShelf = 1, HighShelf = 2 };

    /** The slopes offered, dB/oct. Linkwitz-Riley is two Butterworths, so its slopes are even orders. */
    static constexpr int MaxOrder = 8; // 48 dB/oct
    static constexpr float MinHz = 10.0f;
    static constexpr float MaxHz = 22000.0f;
    static constexpr float MinGainDb = -40.0f;
    static constexpr float MaxGainDb = 12.0f;
    static constexpr float MinThresholdDb = -40.0f;
    static constexpr float MaxThresholdDb = 0.0f;

    struct Pass {
        FilterKind kind { FilterKind::Off };
        float hz { 100.0f };
        int slopeDbPerOct { 24 };
    };

    struct EqBand {
        EqShape shape { EqShape::Peaking };
        float hz { 1000.0f };
        float q { 1.0f };
        float gainDb { 0.0f };
        bool enabled { false };
    };

    struct Settings {
        Pass hpf {};
        Pass lpf { FilterKind::Off, 18000.0f, 24 };
        std::array<EqBand, ParametricEQ::NumBands> eq {};
        bool inverted { false };
        bool muted { false };
        float gainDb { 0.0f };
        bool limiterEnabled { false };
        float thresholdDb { -3.0f };
        float releaseMs { 120.0f };
    };

    void prepare(double sampleRate, int maxBlockSize);
    void reset();

    /** `hpf/type`, `lpf/freq`, `eq/3/gain`, `limiter/threshold`, `polarity`, … Control path. */
    bool setParam(std::string_view param, float value);

    const Settings& settings() const { return s; }
    /** Whether anything at all is switched on — false means `process` leaves the buffer alone. */
    bool isActive() const { return active; }
    /** dB of gain reduction in the last block, ≥ 0. */
    float gainReductionDb() const { return lastReductionDb; }

    /** Audio thread. In place. */
    void process(float* buffer, int numSamples);

    /** Section Qs for a Butterworth of `order`, plus whether it needs a first-order section. Public for the tests. */
    static int butterworthQs(int order, float* qs);

private:
    /** A second-order TPT state-variable section, low- or high-pass. */
    struct Svf {
        float g { 0.0f }, k { 1.414f };
        float s1 { 0.0f }, s2 { 0.0f };
        bool high { false };
        void set(double fs, float hz, float q, bool isHigh);
        float run(float x);
    };
    /** A first-order TPT section. */
    struct OnePole {
        float g { 0.0f }, s { 0.0f };
        bool high { false };
        void set(double fs, float hz, bool isHigh);
        float run(float x);
    };
    struct Chain {
        std::array<Svf, MaxOrder / 2> svf {};
        std::array<OnePole, 2> pole {};
        int svfCount { 0 }, poleCount { 0 };
        void build(double fs, const Pass& p, bool high);
        float run(float x);
        void clear();
    };

    void rebuildFilters();
    void rebuildEq(int band);
    void updateActive();

    double fs { 48000.0 };
    Settings s {};
    Chain hp {}, lp {};
    ParametricEQ eq {};
    bool eqActive { false };
    bool active { false };
    float gain { 1.0f };
    float threshold { 1.0f };
    float release { 0.999f };
    float envGain { 1.0f };
    float lastReductionDb { 0.0f };
};

} // namespace dsp
