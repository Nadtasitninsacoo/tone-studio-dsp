#include "SpeakerProcessor.h"
#include "MixingEngine.h"
#include <algorithm>
#include <cmath>

namespace dsp {

namespace {
constexpr double kPi = 3.14159265358979323846;

/** The nearest slope this filter kind can make: Butterworth any multiple of 6, Linkwitz-Riley even orders only. */
int validSlope(SpeakerProcessor::FilterKind kind, float requested) {
    const int step = kind == SpeakerProcessor::FilterKind::LinkwitzRiley ? 12 : 6;
    const int n = static_cast<int>(std::lround(requested / static_cast<float>(step)));
    return std::clamp(n, 1, 48 / step) * step;
}

float corner(double fs, float hz) {
    // Below Nyquist with margin: tan() of the prewarp runs to infinity at fs/2.
    return static_cast<float>(std::clamp(static_cast<double>(hz), static_cast<double>(SpeakerProcessor::MinHz),
                                         std::min(static_cast<double>(SpeakerProcessor::MaxHz), 0.45 * fs)));
}
} // namespace

// ------------------------------------------------------------------------------------------
// Sections
// ------------------------------------------------------------------------------------------

void SpeakerProcessor::Svf::set(double fs, float hz, float q, bool isHigh) {
    g = static_cast<float>(std::tan(kPi * corner(fs, hz) / fs));
    k = 1.0f / std::max(0.1f, q);
    high = isHigh;
}

float SpeakerProcessor::Svf::run(float x) {
    // Zavalishin's TPT SVF: unconditionally stable, and exact to the bilinear transform.
    const float hp = (x - (k + g) * s1 - s2) / (1.0f + k * g + g * g);
    const float bp = g * hp + s1;
    s1 = g * hp + bp;
    const float lp = g * bp + s2;
    s2 = g * bp + lp;
    return high ? hp : lp;
}

void SpeakerProcessor::OnePole::set(double fs, float hz, bool isHigh) {
    const float raw = static_cast<float>(std::tan(kPi * corner(fs, hz) / fs));
    g = raw / (1.0f + raw);
    high = isHigh;
}

float SpeakerProcessor::OnePole::run(float x) {
    const float v = (x - s) * g;
    const float lp = v + s;
    s = lp + v;
    return high ? x - lp : lp;
}

int SpeakerProcessor::butterworthQs(int order, float* qs) {
    // Pole pairs at sin((2k+1)π / 2N) from the imaginary axis; an odd order has one real pole
    // left over, which is the first-order section the caller adds.
    const int pairs = order / 2;
    for (int k = 0; k < pairs; ++k) {
        qs[k] = static_cast<float>(1.0 / (2.0 * std::sin(kPi * (2 * k + 1) / (2.0 * order))));
    }
    return pairs;
}

void SpeakerProcessor::Chain::build(double fs, const Pass& p, bool high) {
    svfCount = 0;
    poleCount = 0;
    if (p.kind == FilterKind::Off) return;
    const int total = p.slopeDbPerOct / 6;
    // Linkwitz-Riley is a Butterworth of half the order, twice.
    const int bwOrder = p.kind == FilterKind::LinkwitzRiley ? total / 2 : total;
    const int copies = p.kind == FilterKind::LinkwitzRiley ? 2 : 1;
    float qs[MaxOrder / 2] {};
    const int pairs = butterworthQs(bwOrder, qs);
    for (int c = 0; c < copies; ++c) {
        for (int k = 0; k < pairs && svfCount < static_cast<int>(svf.size()); ++k) svf[static_cast<size_t>(svfCount++)].set(fs, p.hz, qs[k], high);
        if (bwOrder % 2 == 1 && poleCount < static_cast<int>(pole.size())) pole[static_cast<size_t>(poleCount++)].set(fs, p.hz, high);
    }
}

float SpeakerProcessor::Chain::run(float x) {
    for (int i = 0; i < poleCount; ++i) x = pole[static_cast<size_t>(i)].run(x);
    for (int i = 0; i < svfCount; ++i) x = svf[static_cast<size_t>(i)].run(x);
    return x;
}

void SpeakerProcessor::Chain::clear() {
    for (auto& f : svf) { f.s1 = 0.0f; f.s2 = 0.0f; }
    for (auto& f : pole) f.s = 0.0f;
}

// ------------------------------------------------------------------------------------------
// The processor
// ------------------------------------------------------------------------------------------

void SpeakerProcessor::prepare(double sampleRate, int maxBlockSize) {
    fs = sampleRate > 0.0 ? sampleRate : 48000.0;
    eq.prepare(fs, maxBlockSize);
    rebuildFilters();
    for (int b = 0; b < ParametricEQ::NumBands; ++b) rebuildEq(b);
    release = static_cast<float>(std::exp(-1.0 / (0.001 * s.releaseMs * fs)));
    reset();
}

void SpeakerProcessor::reset() {
    hp.clear();
    lp.clear();
    eq.reset();
    envGain = 1.0f;
    lastReductionDb = 0.0f;
}

void SpeakerProcessor::rebuildFilters() {
    hp.build(fs, s.hpf, true);
    lp.build(fs, s.lpf, false);
}

void SpeakerProcessor::rebuildEq(int band) {
    const auto& b = s.eq[static_cast<size_t>(band)];
    MixingEngine::EqBandSetting cook;
    cook.shape = static_cast<MixingEngine::EqShape>(static_cast<int>(b.shape));
    cook.frequencyHz = b.hz;
    cook.q = b.q;
    cook.gainDb = b.gainDb;
    cook.enabled = b.enabled;
    // The desk's own cookbook → SVF mapping, so a band here means what the same numbers mean on a strip.
    eq.setBandParameters(band, MixingEngine::cookbookToSvf(cook, fs));
    eqActive = std::any_of(s.eq.begin(), s.eq.end(), [](const EqBand& e) { return e.enabled && e.gainDb != 0.0f; });
}

void SpeakerProcessor::updateActive() {
    gain = std::pow(10.0f, s.gainDb / 20.0f);
    threshold = std::pow(10.0f, s.thresholdDb / 20.0f);
    active = s.muted || s.inverted || s.gainDb != 0.0f || eqActive || s.limiterEnabled ||
             s.hpf.kind != FilterKind::Off || s.lpf.kind != FilterKind::Off;
}

bool SpeakerProcessor::setParam(std::string_view p, float value) {
    if (!std::isfinite(value)) return false;
    const auto on = [](float v) { return v >= 0.5f; };

    const auto pass = [&](Pass& target, std::string_view rest) -> bool {
        if (rest == "type") {
            const int k = static_cast<int>(value);
            if (static_cast<float>(k) != value || k < 0 || k > 2) return false;
            target.kind = static_cast<FilterKind>(k);
            target.slopeDbPerOct = validSlope(target.kind == FilterKind::Off ? FilterKind::Butterworth : target.kind,
                                              static_cast<float>(target.slopeDbPerOct));
        } else if (rest == "freq") {
            target.hz = std::clamp(value, MinHz, MaxHz);
        } else if (rest == "slope") {
            target.slopeDbPerOct = validSlope(target.kind == FilterKind::Off ? FilterKind::Butterworth : target.kind, value);
        } else {
            return false;
        }
        rebuildFilters();
        return true;
    };

    bool handled = true;
    if (p.substr(0, 4) == "hpf/") handled = pass(s.hpf, p.substr(4));
    else if (p.substr(0, 4) == "lpf/") handled = pass(s.lpf, p.substr(4));
    else if (p.size() > 5 && p.substr(0, 3) == "eq/" && p[3] >= '1' && p[3] <= '6' && p[4] == '/') {
        const int band = p[3] - '1';
        auto& b = s.eq[static_cast<size_t>(band)];
        const auto rest = p.substr(5);
        if (rest == "shape") {
            const int k = static_cast<int>(value);
            if (static_cast<float>(k) != value || k < 0 || k > 2) return false;
            b.shape = static_cast<EqShape>(k);
        } else if (rest == "freq") b.hz = std::clamp(value, 20.0f, 20000.0f);
        else if (rest == "q") b.q = std::clamp(value, 0.1f, 16.0f);
        else if (rest == "gain") b.gainDb = std::clamp(value, -18.0f, 18.0f);
        else if (rest == "enabled") b.enabled = on(value);
        else return false;
        rebuildEq(band);
    } else if (p == "polarity") s.inverted = on(value);
    else if (p == "mute") s.muted = on(value);
    else if (p == "gain") s.gainDb = std::clamp(value, MinGainDb, MaxGainDb);
    else if (p == "limiter/enabled") s.limiterEnabled = on(value);
    else if (p == "limiter/threshold") s.thresholdDb = std::clamp(value, MinThresholdDb, MaxThresholdDb);
    else if (p == "limiter/release") {
        s.releaseMs = std::clamp(value, 10.0f, 2000.0f);
        release = static_cast<float>(std::exp(-1.0 / (0.001 * s.releaseMs * fs)));
    } else handled = false;

    if (handled) updateActive();
    return handled;
}

void SpeakerProcessor::process(float* buffer, int numSamples) {
    if (!active) {
        lastReductionDb = 0.0f;
        return;
    }
    if (s.muted) {
        std::fill(buffer, buffer + numSamples, 0.0f);
        lastReductionDb = 0.0f;
        return;
    }
    const bool doHp = s.hpf.kind != FilterKind::Off;
    const bool doLp = s.lpf.kind != FilterKind::Off;
    const float sign = s.inverted ? -1.0f : 1.0f;
    float minGain = 1.0f;

    if (eqActive) eq.processBlock(buffer, numSamples);
    for (int i = 0; i < numSamples; ++i) {
        float x = buffer[i] * gain;
        if (doHp) x = hp.run(x);
        if (doLp) x = lp.run(x);
        if (s.limiterEnabled) {
            // Instant attack, exponential release: the gain is never above what this sample
            // needs, so |out| ≤ threshold holds sample by sample with no look-ahead.
            const float mag = std::abs(x);
            const float need = mag > threshold ? threshold / mag : 1.0f;
            envGain = need < envGain ? need : release * envGain + (1.0f - release) * need;
            if (envGain > need) envGain = need;
            x *= envGain;
            minGain = std::min(minGain, envGain);
        }
        buffer[i] = sign * x;
    }
    lastReductionDb = minGain < 1.0f ? -20.0f * std::log10(std::max(minGain, 1.0e-6f)) : 0.0f;
}

} // namespace dsp
