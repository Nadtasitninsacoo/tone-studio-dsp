#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>
#include <vector>

namespace dsp::rack {

/**
 * The pieces every web rack is built from, written once for the engine's ports of them.
 *
 * `GuitarAmp` predates this file and keeps its own copies; they are the same arithmetic and a
 * test holds `cabinetImpulse` here against `GuitarAmp::cabinetImpulse` so the two cannot drift.
 * The racks after the guitar (bass first) are built from this.
 */

constexpr double Pi = 3.14159265358979323846;

/** Web Audio's lowpass/highpass `Q` is in dB: the linear Q a value means. */
inline double webPassQ(double qDb) { return std::pow(10.0, qDb / 20.0); }

inline float fromDb(float db) { return db <= -120.0f ? 0.0f : std::pow(10.0f, db / 20.0f); }

/** `coefficient(seconds, sampleRate)` in amp-dsp-processor.js. */
inline float coefficient(double seconds, double sampleRate) {
    if (seconds <= 0.0) return 0.0f;
    return static_cast<float>(std::exp(-1.0 / std::max(1.0, seconds * sampleRate)));
}

/**
 * A `BiquadFilterNode`: the RBJ cookbook, in double, transposed direct form II.
 *
 * Not `FilterPrimitives`, because that one clamps to 20 Hz and the bass rack's DC blocker sits
 * at 18; and not the SVF, because the point here is to be the web node's own formula. Shelves
 * use slope 1 (cookbook Q = √½) and ignore `q`, exactly as Web Audio's do; lowpass and highpass
 * take their Q in dB, as Web Audio's do.
 */
class WebBiquad {
public:
    enum class Kind { LowPass, HighPass, Peaking, LowShelf, HighShelf, BandPass };

    void set(Kind kind, double hz, double q, double gainDb, double fs) {
        const double nyquist = fs * 0.5;
        const double f0 = std::clamp(hz, 1.0, nyquist * 0.999);
        const double w0 = 2.0 * Pi * f0 / fs;
        const double c = std::cos(w0), s = std::sin(w0);
        const double A = std::pow(10.0, gainDb / 40.0);
        double alpha = 0.0;
        if (kind == Kind::LowPass || kind == Kind::HighPass) alpha = s / (2.0 * std::max(webPassQ(q), 1e-4));
        else if (kind == Kind::Peaking || kind == Kind::BandPass) alpha = s / (2.0 * std::max(q, 1e-4)); // linear Q
        else alpha = s / 2.0 * std::sqrt(2.0); // shelf, S = 1
        double b0 = 1, b1 = 0, b2 = 0, a0 = 1, a1 = 0, a2 = 0;
        switch (kind) {
            case Kind::LowPass:
                b0 = (1 - c) / 2; b1 = 1 - c; b2 = (1 - c) / 2;
                a0 = 1 + alpha; a1 = -2 * c; a2 = 1 - alpha;
                break;
            case Kind::HighPass:
                b0 = (1 + c) / 2; b1 = -(1 + c); b2 = (1 + c) / 2;
                a0 = 1 + alpha; a1 = -2 * c; a2 = 1 - alpha;
                break;
            case Kind::BandPass: // constant 0 dB peak, as Web Audio's `bandpass`; Q linear
                b0 = alpha; b1 = 0; b2 = -alpha;
                a0 = 1 + alpha; a1 = -2 * c; a2 = 1 - alpha;
                break;
            case Kind::Peaking:
                b0 = 1 + alpha * A; b1 = -2 * c; b2 = 1 - alpha * A;
                a0 = 1 + alpha / A; a1 = -2 * c; a2 = 1 - alpha / A;
                break;
            case Kind::LowShelf: {
                const double k = 2 * std::sqrt(A) * alpha;
                b0 = A * (A + 1 - (A - 1) * c + k); b1 = 2 * A * (A - 1 - (A + 1) * c);
                b2 = A * (A + 1 - (A - 1) * c - k);
                a0 = A + 1 + (A - 1) * c + k; a1 = -2 * (A - 1 + (A + 1) * c);
                a2 = A + 1 + (A - 1) * c - k;
                break;
            }
            case Kind::HighShelf: {
                const double k = 2 * std::sqrt(A) * alpha;
                b0 = A * (A + 1 + (A - 1) * c + k); b1 = -2 * A * (A - 1 + (A + 1) * c);
                b2 = A * (A + 1 + (A - 1) * c - k);
                a0 = A + 1 - (A - 1) * c + k; a1 = 2 * (A - 1 - (A + 1) * c);
                a2 = A + 1 - (A - 1) * c - k;
                break;
            }
        }
        nb0 = b0 / a0; nb1 = b1 / a0; nb2 = b2 / a0; na1 = a1 / a0; na2 = a2 / a0;
    }

    void reset() { z1 = z2 = 0.0; }

    float process(float x) {
        const double in = x;
        const double y = nb0 * in + z1;
        z1 = nb1 * in - na1 * y + z2;
        z2 = nb2 * in - na2 * y;
        return static_cast<float>(y);
    }

    /** |H(e^jw)| in dB, for the tests. */
    double magnitudeDb(double hz, double fs) const {
        const double w = 2.0 * Pi * hz / fs;
        const double cr = std::cos(w), ci = -std::sin(w), c2r = std::cos(2 * w), c2i = -std::sin(2 * w);
        const double nr = nb0 + nb1 * cr + nb2 * c2r, ni = nb1 * ci + nb2 * c2i;
        const double dr = 1 + na1 * cr + na2 * c2r, di = na1 * ci + na2 * c2i;
        return 10.0 * std::log10((nr * nr + ni * ni) / std::max(dr * dr + di * di, 1e-300));
    }

private:
    double nb0 { 1 }, nb1 { 0 }, nb2 { 0 }, na1 { 0 }, na2 { 0 };
    double z1 { 0 }, z2 { 0 };
};

/** `setTargetAtTime(v, now, 0.01)`, the way every web rack writes a gain: one pole, 10 ms. */
class Smoothed {
public:
    void prepare(double fs) { coef = coefficient(0.01, fs); }
    void set(float v) { target = v; }
    void jump(float v) { target = current = v; }
    float next() { current = target + (current - target) * coef; return current; }
    float value() const { return current; }
    float goal() const { return target; }
private:
    float target { 0.0f }, current { 0.0f }, coef { 0.0f };
};

/** `GateProcessor` in amp-dsp-processor.js, mono. */
class Gate {
public:
    void prepare(double fs) {
        attack = coefficient(0.001, fs);
        release = coefficient(0.12, fs);
        envCoef = coefficient(0.004, fs);
        holdFrames = static_cast<int>(std::floor(0.15 * fs));
    }
    void reset() { env = 0.0f; gain = 0.0f; hold = 0; }
    float process(float v, bool enabled, float thresholdDb) {
        const float openAt = fromDb(thresholdDb);
        const float closeAt = openAt * 0.501f;
        const float peak = std::abs(v);
        env = peak > env ? peak : env * envCoef + peak * (1.0f - envCoef);
        if (env >= openAt) hold = holdFrames;
        else if (hold > 0) --hold;
        const bool open = hold > 0 || env >= closeAt;
        if (!enabled) {
            gain = 1.0f;
        } else {
            const float target = open ? 1.0f : 0.0f;
            const float c = target > gain ? attack : release;
            gain = target + (gain - target) * c;
        }
        return v * gain;
    }
private:
    float env { 0.0f }, gain { 0.0f };
    int hold { 0 }, holdFrames { 0 };
    float attack { 0.0f }, release { 0.0f }, envCoef { 0.0f };
};

/** `LimiterProcessor` in amp-dsp-processor.js, mono: 3 ms look-ahead, two-speed release. */
class Limiter {
public:
    void prepare(double fs) {
        size = std::max(8, static_cast<int>(std::floor(0.003 * fs)));
        delay.assign(static_cast<size_t>(size), 0.0f);
        window.assign(static_cast<size_t>(size), 0.0f);
        deque.assign(static_cast<size_t>(size + 2), 0);
        fastRel = coefficient(0.05, fs);
        slowRel = coefficient(0.35, fs);
        reset();
    }
    void reset() {
        std::fill(delay.begin(), delay.end(), 0.0f);
        std::fill(window.begin(), window.end(), 0.0f);
        cursor = head = tail = 0;
        position = 0;
        gain = slowGain = 1.0f;
    }
    float process(float x, bool enabled, float ceiling) {
        const float peak = std::abs(x);
        const int at = cursor;
        const float delayed = delay[static_cast<size_t>(at)];
        delay[static_cast<size_t>(at)] = x;
        window[static_cast<size_t>(at)] = peak;
        cursor = (at + 1) % size;

        const int capacity = size + 2;
        while (tail != head) {
            const long long back = deque[static_cast<size_t>((tail - 1 + capacity) % capacity)];
            if (window[static_cast<size_t>(back % size)] > peak) break;
            tail = (tail - 1 + capacity) % capacity;
        }
        deque[static_cast<size_t>(tail)] = position;
        tail = (tail + 1) % capacity;
        const long long oldest = position - size + 1;
        while (tail != head && deque[static_cast<size_t>(head)] < oldest) head = (head + 1) % capacity;
        ++position;

        const float windowPeak = window[static_cast<size_t>(deque[static_cast<size_t>(head)] % size)];
        const float target = windowPeak > ceiling ? ceiling / windowPeak : 1.0f;
        if (target < gain) {
            gain -= std::min(gain - target, (1.0f - target) / static_cast<float>(size) + 1e-6f);
            slowGain = std::min(slowGain, gain);
        } else {
            gain = target + (gain - target) * fastRel;
            slowGain = target + (slowGain - target) * slowRel;
            if (slowGain < gain) gain = slowGain;
        }
        float y = delayed * (enabled ? gain : 1.0f);
        if (enabled) y = std::clamp(y, -ceiling, ceiling);
        return y;
    }
private:
    std::vector<float> delay, window;
    std::vector<long long> deque;
    int cursor { 0 }, size { 0 }, head { 0 }, tail { 0 };
    long long position { 0 };
    float gain { 1.0f }, slowGain { 1.0f }, fastRel { 0.0f }, slowRel { 0.0f };
};

//==============================================================================================
// The waveshaper.

constexpr int CurvePoints = 2048;
constexpr double BiasScale = 2.5;   // `BIAS_SCALE` in audioGraph.ts
constexpr double DriveProbePeak = 0.25; // `DRIVE_PROBE_PEAK` in audioGraph.ts
constexpr int DriveProbePoints = 1024;
using Curve = std::array<float, CurvePoints>;

/** `tubeCurve(amount, hardness, bias)` in audioGraph.ts. */
inline void tubeCurve(double amount, double hardness, double bias, Curve& out) {
    const double k = 1.0 + amount * hardness;
    const double shift = bias * amount * BiasScale;
    const double dc = std::tanh(shift);
    const double high = std::tanh(k + shift) - dc;
    const double low = std::tanh(-k + shift) - dc;
    const double normalise = std::max({ std::abs(high), std::abs(low), 1e-6 });
    for (int i = 0; i < CurvePoints; ++i) {
        const double x = static_cast<double>(i) / (CurvePoints - 1) * 2.0 - 1.0;
        out[static_cast<size_t>(i)] = static_cast<float>((std::tanh(k * x + shift) - dc) / normalise);
    }
}

/** `saturationCurve(amount, hardness)` in audioGraph.ts: symmetric tanh, normalised to ±1. */
inline void saturationCurve(double amount, double hardness, Curve& out) {
    const double k = 1.0 + amount * hardness;
    const double normalise = std::tanh(k);
    for (int i = 0; i < CurvePoints; ++i) {
        const double x = static_cast<double>(i) / (CurvePoints - 1) * 2.0 - 1.0;
        out[static_cast<size_t>(i)] = static_cast<float>(std::tanh(k * x) / normalise);
    }
}

/** `WaveShaperNode`'s reading of a table: clamp to ±1, then interpolate. */
inline float readCurve(const Curve& c, float x) {
    const float pos = (std::clamp(x, -1.0f, 1.0f) + 1.0f) / 2.0f * (CurvePoints - 1);
    const int i = static_cast<int>(std::floor(pos));
    if (i >= CurvePoints - 1) return c[CurvePoints - 1];
    const float frac = pos - static_cast<float>(i);
    return c[static_cast<size_t>(i)] * (1.0f - frac) + c[static_cast<size_t>(i + 1)] * frac;
}

/** `driveMakeup(curve, stages, targetRms)` in audioGraph.ts. */
inline float driveMakeup(const Curve& c, int stages, double targetRms) {
    if (stages < 1) return 1.0f;
    std::array<double, DriveProbePoints> out {};
    double mean = 0.0;
    for (int i = 0; i < DriveProbePoints; ++i) {
        float v = static_cast<float>(DriveProbePeak * std::sin(2.0 * Pi * i / DriveProbePoints));
        for (int s = 0; s < stages; ++s) v = readCurve(c, v);
        out[static_cast<size_t>(i)] = v;
        mean += v;
    }
    mean /= DriveProbePoints;
    double sum = 0.0;
    for (const double v : out) sum += (v - mean) * (v - mean);
    const double rms = std::sqrt(sum / DriveProbePoints);
    if (!(rms > 1e-9)) return 1.0f;
    return static_cast<float>(targetRms / rms);
}

//==============================================================================================
// The cabinet impulse (`cabinetImpulse` in cabinet.ts).

struct CabSpec {
    double highpassHz, resonanceHz, resonanceDb, scoopHz, scoopDb, presenceHz, presenceDb;
    double lowpassHz;
    int lowpassStages;
    std::array<std::pair<double, double>, 3> reflections; // ms, amplitude; 0 ms = unused
};

/** `CABINETS` in cabinet.ts — every model, in that table's order. */
inline const CabSpec& cabinetSpec(int index) {
    static const CabSpec table[6] = {
        { 90, 125, 3.5, 500, -3.5, 2600, 6, 5200, 3, {{ {0.42, -0.14}, {1.1, 0.08}, {2.3, -0.05} }} },  // v30
        { 80, 105, 4.5, 420, -2.0, 1700, 5, 4200, 3, {{ {0.55, -0.12}, {1.4, 0.07}, {2.8, -0.04} }} },  // greenback
        { 75, 95, 3.0, 700, -1.5, 3100, 4, 5800, 3, {{ {0.36, -0.10}, {0.95, 0.06}, {0.0, 0.0} }} },     // american
        { 65, 85, 4.0, 900, -1.0, 1600, 4.5, 3400, 3, {{ {0.70, -0.09}, {1.9, 0.05}, {0.0, 0.0} }} },    // jazz
        { 38, 62, 5.0, 500, -2.5, 1200, 2, 3000, 3, {{ {0.90, -0.10}, {2.4, 0.06}, {0.0, 0.0} }} },      // b15
        { 45, 80, 3.0, 420, -1.5, 2200, 4, 5000, 2, {{ {0.32, -0.12}, {0.85, 0.07}, {1.7, -0.04} }} },  // b410
    };
    return table[std::clamp(index, 0, 5)];
}

/** The deepest low-pass any cabinet uses: `DI_TOP_SECTIONS` in bassFx.ts. */
constexpr int MaxCabinetLowpassStages = 3;

namespace detail {
/** The offline biquad cabinet.ts builds its impulses with: plain RBJ, DF1, **linear** Q. */
inline void applyBiquad(std::vector<double>& data, WebBiquad::Kind kind, double hz, double q, double gainDb, double fs) {
    const double f0 = std::min(std::max(hz, 1.0), fs * 0.49);
    const double w0 = 2.0 * Pi * f0 / fs;
    const double c = std::cos(w0), s = std::sin(w0);
    const double alpha = s / (2.0 * std::max(q, 1e-4));
    const double A = std::pow(10.0, gainDb / 40.0);
    double b0 = 1, b1 = 0, b2 = 0, a0 = 1, a1 = 0, a2 = 0;
    using K = WebBiquad::Kind;
    switch (kind) {
        case K::BandPass: b0 = alpha; b1 = 0; b2 = -alpha; a0 = 1 + alpha; a1 = -2 * c; a2 = 1 - alpha; break;
        case K::LowPass: b0 = (1 - c) / 2; b1 = 1 - c; b2 = (1 - c) / 2; a0 = 1 + alpha; a1 = -2 * c; a2 = 1 - alpha; break;
        case K::HighPass: b0 = (1 + c) / 2; b1 = -(1 + c); b2 = (1 + c) / 2; a0 = 1 + alpha; a1 = -2 * c; a2 = 1 - alpha; break;
        case K::Peaking: b0 = 1 + alpha * A; b1 = -2 * c; b2 = 1 - alpha * A; a0 = 1 + alpha / A; a1 = -2 * c; a2 = 1 - alpha / A; break;
        case K::LowShelf: {
            const double k = 2 * std::sqrt(A) * alpha;
            b0 = A * (A + 1 - (A - 1) * c + k); b1 = 2 * A * (A - 1 - (A + 1) * c); b2 = A * (A + 1 - (A - 1) * c - k);
            a0 = A + 1 + (A - 1) * c + k; a1 = -2 * (A - 1 + (A + 1) * c); a2 = A + 1 + (A - 1) * c - k;
            break;
        }
        case K::HighShelf: {
            const double k = 2 * std::sqrt(A) * alpha;
            b0 = A * (A + 1 + (A - 1) * c + k); b1 = -2 * A * (A - 1 + (A + 1) * c); b2 = A * (A + 1 + (A - 1) * c - k);
            a0 = A + 1 - (A - 1) * c + k; a1 = 2 * (A - 1 - (A + 1) * c); a2 = A + 1 - (A - 1) * c - k;
            break;
        }
    }
    b0 /= a0; b1 /= a0; b2 /= a0; a1 /= a0; a2 /= a0;
    double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
    for (auto& v : data) {
        const double y = b0 * v + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1; x1 = v; y2 = y1; y1 = y;
        v = y;
    }
}
} // namespace detail

/** `responseDbAt`: one bin of the DFT, in dB. */
inline double responseDbAt(const std::vector<float>& ir, double fs, double hz) {
    const double w = 2.0 * Pi * hz / fs;
    double re = 0.0, im = 0.0;
    for (size_t n = 0; n < ir.size(); ++n) {
        re += ir[n] * std::cos(w * static_cast<double>(n));
        im -= ir[n] * std::sin(w * static_cast<double>(n));
    }
    return 20.0 * std::log10(std::max(std::hypot(re, im), 1e-12));
}

/**
 * `cabinetImpulse(sr, id, {presenceDb, resonanceDb})` with the centre mic — the only placement
 * the bass rack uses (it passes no mic). `micShelves` 1 and 2 are the guitar's off-axis and
 * distant placements, kept so the test can hold this against `GuitarAmp::cabinetImpulse`.
 */
inline std::vector<float> cabinetImpulse(double fs, int model, float presenceDb, float resonanceDb,
                                         double presenceScale = 1.0, int micShelves = 0) {
    using K = WebBiquad::Kind;
    const auto& cab = cabinetSpec(model);
    const int length = std::max(64, static_cast<int>(std::floor(fs * 0.024)));
    std::vector<double> ir(static_cast<size_t>(length), 0.0);
    ir[0] = 1.0;
    for (const auto& [ms, amp] : cab.reflections) {
        if (ms <= 0.0) continue;
        const int index = static_cast<int>(std::lround(ms / 1000.0 * fs));
        if (index > 0 && index < length) ir[static_cast<size_t>(index)] += amp;
    }
    detail::applyBiquad(ir, K::HighPass, cab.highpassHz, 0.9, 0.0, fs);
    detail::applyBiquad(ir, K::Peaking, cab.resonanceHz, 1.2, cab.resonanceDb + resonanceDb, fs);
    detail::applyBiquad(ir, K::Peaking, cab.scoopHz, 0.8, cab.scoopDb, fs);
    detail::applyBiquad(ir, K::Peaking, cab.presenceHz, 1.4, cab.presenceDb * presenceScale + presenceDb, fs);
    if (micShelves == 1) {
        detail::applyBiquad(ir, K::HighShelf, 3500, 0.7, -4.5, fs);
    } else if (micShelves == 2) {
        detail::applyBiquad(ir, K::LowShelf, 150, 0.7, -3.0, fs);
        detail::applyBiquad(ir, K::HighShelf, 4000, 0.7, -3.5, fs);
    }
    for (int s = 0; s < cab.lowpassStages; ++s) detail::applyBiquad(ir, K::LowPass, cab.lowpassHz, 0.7, 0.0, fs);

    const int fade = static_cast<int>(std::floor(length * 0.25));
    for (int i = 0; i < fade; ++i) ir[static_cast<size_t>(length - fade + i)] *= 1.0 - static_cast<double>(i) / fade;

    std::vector<float> out(ir.size());
    std::transform(ir.begin(), ir.end(), out.begin(), [](double v) { return static_cast<float>(v); });
    const double ref = responseDbAt(out, fs, 1000.0);
    if (std::isfinite(ref)) {
        const float scale = static_cast<float>(std::pow(10.0, -ref / 20.0));
        for (auto& v : out) v *= scale;
    }
    return out;
}

/** What comes out of a mono rack is scaled by √2: through the channel's equal-power panner at
 * centre each side then carries what each side of the web rack's dual-mono output does. */
constexpr float CentreToMono = 1.41421356f;

} // namespace dsp::rack
