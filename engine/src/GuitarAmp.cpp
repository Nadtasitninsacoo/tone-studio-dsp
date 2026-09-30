#include "GuitarAmp.h"
#include "MixingEngine.h"

#include <algorithm>
#include <cmath>

namespace dsp {

namespace {

constexpr double Pi = 3.14159265358979323846;

/**
 * Web Audio's lowpass/highpass `Q` is in **dB**, not linear. The web chain writes 0.7 on every
 * one of them, which is a linear Q of 10^(0.7/20) ≈ 1.084 — the same conversion
 * `lib/dspStrip.ts` makes for the strip's HPF.
 */
constexpr float WebPassQ = 1.0839269f;

/** Shelves in Web Audio ignore Q and use slope 1, which is cookbook Q = √½. */
constexpr float ShelfQ = 0.70710678f;

float fromDb(float db) { return db <= -120.0f ? 0.0f : std::pow(10.0f, db / 20.0f); }

/** `coefficient(seconds, sampleRate)` in amp-dsp-processor.js. */
float coefficient(double seconds, double sampleRate) {
    if (seconds <= 0.0) return 0.0f;
    return static_cast<float>(std::exp(-1.0 / std::max(1.0, seconds * sampleRate)));
}

// ---- The offline biquad `cabinet.ts` builds its impulses with: plain RBJ, DF1, linear Q. ----
enum class Kind { LowPass, HighPass, Peaking, LowShelf, HighShelf };

void applyBiquad(std::vector<double>& data, Kind kind, double hz, double q, double gainDb,
                 double fs) {
    const double f0 = std::min(std::max(hz, 1.0), fs * 0.49);
    const double w0 = 2.0 * Pi * f0 / fs;
    const double c = std::cos(w0), s = std::sin(w0);
    const double alpha = s / (2.0 * std::max(q, 1e-4));
    const double A = std::pow(10.0, gainDb / 40.0);
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
    b0 /= a0; b1 /= a0; b2 /= a0; a1 /= a0; a2 /= a0;
    double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
    for (auto& v : data) {
        const double y = b0 * v + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1; x1 = v; y2 = y1; y1 = y;
        v = y;
    }
}

struct CabSpec {
    double highpassHz, resonanceHz, resonanceDb, scoopHz, scoopDb, presenceHz, presenceDb;
    double lowpassHz;
    int lowpassStages;
    std::array<std::pair<double, double>, 3> reflections; // ms, amplitude; 0 ms = unused
};

/** `CABINETS` in cabinet.ts — the four guitar speakers, in `CABINET_IDS` order. */
const CabSpec Cabinets[4] = {
    { 90, 125, 3.5, 500, -3.5, 2600, 6, 5200, 3, {{ {0.42, -0.14}, {1.1, 0.08}, {2.3, -0.05} }} },
    { 80, 105, 4.5, 420, -2.0, 1700, 5, 4200, 3, {{ {0.55, -0.12}, {1.4, 0.07}, {2.8, -0.04} }} },
    { 75,  95, 3.0, 700, -1.5, 3100, 4, 5800, 3, {{ {0.36, -0.10}, {0.95, 0.06}, {0.0, 0.0} }} },
    { 65,  85, 4.0, 900, -1.0, 1600, 4.5, 3400, 3, {{ {0.70, -0.09}, {1.9, 0.05}, {0.0, 0.0} }} },
};

struct MicSpec {
    double presenceScale;
    int shelves; // 0, 1 or 2 of the fixed shelves below
};

// `MIC_PLACEMENTS` in cabinet.ts.
const MicSpec Mics[3] = { { 1.0, 0 }, { 0.6, 1 }, { 0.7, 2 } };

constexpr double IrSeconds = 0.024;
constexpr double ReferenceHz = 1000.0;

void setPass(FilterPrimitives& f, FilterPrimitives::Type type, float hz, double fs, int block) {
    f.prepare(fs, block);
    f.setType(type);
    f.setParameters(hz, WebPassQ, 0.0f);
}

} // namespace

GuitarAmp::GuitarAmp(juce::dsp::ConvolutionMessageQueue& queue)
    : cabinet(juce::dsp::Convolution::Latency { 0 }, queue),
      reverb(juce::dsp::Convolution::NonUniform { 256 }, queue) {
    for (auto& os : oversampler) {
        // 4× (two 2× stages), FIR half-band — `WaveShaperNode.oversample = '4x'`.
        // Not `isMaxQuality`: the shorter half-band filters take the three stages from about
        // 4.3% to 3.3% of a core per rack at a 128-sample block (measured in GuitarAmpTests),
        // and the 13 kHz alias the test reads stays 61 dB under the fundamental. The web's
        // '4x' is an unspecified resampler anyway.
        os = std::make_unique<juce::dsp::Oversampling<float>>(
            1, 2, juce::dsp::Oversampling<float>::filterHalfBandFIREquiripple, false, false);
    }
}

void GuitarAmp::prepare(double newSampleRate, int maxBlockSize) {
    sampleRate = newSampleRate;
    preparedRate.store(sampleRate);

    setPass(dcBlock, FilterPrimitives::Type::HighPass, 22.0f, sampleRate, maxBlockSize);
    for (int i = 0; i < 2; ++i) {
        setPass(interLow[i], FilterPrimitives::Type::LowPass, 6200.0f, sampleRate, maxBlockSize);
        setPass(interDc[i], FilterPrimitives::Type::HighPass, 30.0f, sampleRate, maxBlockSize);
    }
    setPass(delayTone, FilterPrimitives::Type::LowPass, 3200.0f, sampleRate, maxBlockSize);

    tone.prepare(sampleRate, maxBlockSize);
    post.prepare(sampleRate, maxBlockSize);
    comp.prepare(sampleRate, maxBlockSize);
    comp.setKnee(10.0f);
    comp.setAttack(4.0f);
    comp.setRelease(200.0f);
    comp.setDetectionMode(Compressor::DetectionMode::Peak);

    for (auto& os : oversampler) os->initProcessing(static_cast<size_t>(maxBlockSize));

    const juce::dsp::ProcessSpec spec { sampleRate, static_cast<juce::uint32>(maxBlockSize), 1 };
    cabinet.prepare(spec);
    reverb.prepare(spec);
    wetBuffer.assign(static_cast<size_t>(maxBlockSize), 0.0f);

    delayLine.assign(static_cast<size_t>(sampleRate * 2.0) + 4, 0.0f);

    gateAttack = coefficient(0.001, sampleRate);
    gateRelease = coefficient(0.12, sampleRate);
    gateEnvCoef = coefficient(0.004, sampleRate);
    gateHoldFrames = static_cast<int>(std::floor(0.15 * sampleRate));

    limSize = std::max(8, static_cast<int>(std::floor(0.003 * sampleRate)));
    limDelay.assign(static_cast<size_t>(limSize), 0.0f);
    limWindow.assign(static_cast<size_t>(limSize), 0.0f);
    limDeque.assign(static_cast<size_t>(limSize + 2), 0);
    limFastRel = coefficient(0.05, sampleRate);
    limSlowRel = coefficient(0.35, sampleRate);

    applyTone();
    applyComp();
    driveStale = true;
    // A new rate makes every impulse response the wrong length.
    cabDirty.store(true);
    reverbDirty.store(true);
    reset();
}

void GuitarAmp::reset() {
    // The engine resets every channel from its constructor, before any device has prepared
    // it; the oversamplers and convolutions have nothing to reset until then.
    if (preparedRate.load() <= 0.0) return;
    dcBlock.reset();
    for (int i = 0; i < 2; ++i) { interLow[i].reset(); interDc[i].reset(); }
    delayTone.reset();
    tone.reset();
    post.reset();
    comp.reset();
    for (auto& os : oversampler) os->reset();
    cabinet.reset();
    reverb.reset();
    std::fill(delayLine.begin(), delayLine.end(), 0.0f);
    delayWrite = 0;
    gateEnv = 0.0f;
    gateGain = 0.0f;
    gateHold = 0;
    std::fill(limDelay.begin(), limDelay.end(), 0.0f);
    std::fill(limWindow.begin(), limWindow.end(), 0.0f);
    limCursor = limHead = limTail = 0;
    limPosition = 0;
    limGain = limSlowGain = 1.0f;
}

void GuitarAmp::applyTone() {
    auto band = [this](int index, MixingEngine::EqShape shape, float hz, float q, float gainDb) {
        MixingEngine::EqBandSetting s;
        s.shape = shape;
        s.frequencyHz = hz;
        s.q = q;
        s.gainDb = gainDb;
        s.enabled = true;
        tone.setBandParameters(index, MixingEngine::cookbookToSvf(s, sampleRate));
    };
    band(0, MixingEngine::EqShape::LowShelf, 180.0f, ShelfQ, bassDb);
    band(1, MixingEngine::EqShape::Peaking, midHz, 0.9f, midDb);
    band(2, MixingEngine::EqShape::HighShelf, 2400.0f, ShelfQ, trebleDb);

    // Depth and presence sit after the cabinet, at half the cabinet knobs' values.
    auto postBand = [this](int index, MixingEngine::EqShape shape, float hz, float gainDb) {
        MixingEngine::EqBandSetting s;
        s.shape = shape;
        s.frequencyHz = hz;
        s.q = ShelfQ;
        s.gainDb = gainDb;
        s.enabled = true;
        post.setBandParameters(index, MixingEngine::cookbookToSvf(s, sampleRate));
    };
    postBand(0, MixingEngine::EqShape::LowShelf, 120.0f, cabResonanceDb.load() * 0.5f);
    postBand(1, MixingEngine::EqShape::HighShelf, 3200.0f, cabPresenceDb.load() * 0.5f);
}

void GuitarAmp::applyComp() {
    comp.setEnabled(compEnabled);
    comp.setThreshold(compThresholdDb);
    comp.setRatio(compRatio);
    // `DynamicsCompressorNode` adds its own makeup: (1 / fullRangeGain)^0.6. The same formula
    // `lib/dspStrip.ts` sends for the strip's compressor.
    const float makeupDb = compEnabled ? 0.6f * (1.0f - 1.0f / std::max(1.0f, compRatio)) * -compThresholdDb
                                       : 0.0f;
    comp.setMakeup(makeupDb);
}

//==============================================================================================
// Pure halves

void GuitarAmp::tubeCurve(float amount, float bias, std::array<float, CurvePoints>& out) {
    const double k = 1.0 + amount * StageHardness;
    const double shift = static_cast<double>(bias) * amount * BiasScale;
    const double dc = std::tanh(shift);
    const double high = std::tanh(k + shift) - dc;
    const double low = std::tanh(-k + shift) - dc;
    const double normalise = std::max({ std::abs(high), std::abs(low), 1e-6 });
    for (int i = 0; i < CurvePoints; ++i) {
        const double x = static_cast<double>(i) / (CurvePoints - 1) * 2.0 - 1.0;
        out[static_cast<size_t>(i)] = static_cast<float>((std::tanh(k * x + shift) - dc) / normalise);
    }
}

float GuitarAmp::readCurve(const std::array<float, CurvePoints>& c, float x) {
    const float pos = (std::clamp(x, -1.0f, 1.0f) + 1.0f) / 2.0f * (CurvePoints - 1);
    const int i = static_cast<int>(std::floor(pos));
    if (i >= CurvePoints - 1) return c[CurvePoints - 1];
    const float frac = pos - static_cast<float>(i);
    return c[static_cast<size_t>(i)] * (1.0f - frac) + c[static_cast<size_t>(i + 1)] * frac;
}

float GuitarAmp::driveMakeup(const std::array<float, CurvePoints>& c, int stages) {
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
    return static_cast<float>(DriveTargetRms / rms);
}

double GuitarAmp::responseDbAt(const std::vector<float>& ir, double fs, double hz) {
    const double w = 2.0 * Pi * hz / fs;
    double re = 0.0, im = 0.0;
    for (size_t n = 0; n < ir.size(); ++n) {
        re += ir[n] * std::cos(w * static_cast<double>(n));
        im -= ir[n] * std::sin(w * static_cast<double>(n));
    }
    return 20.0 * std::log10(std::max(std::hypot(re, im), 1e-12));
}

std::vector<float> GuitarAmp::cabinetImpulse(double fs, Cabinet cabId, Mic micId, float presenceDb,
                                             float resonanceDb) {
    const auto& cab = Cabinets[std::clamp(static_cast<int>(cabId), 0, 3)];
    const auto& mic = Mics[std::clamp(static_cast<int>(micId), 0, 2)];
    const int length = std::max(64, static_cast<int>(std::floor(fs * IrSeconds)));
    std::vector<double> ir(static_cast<size_t>(length), 0.0);

    ir[0] = 1.0;
    for (const auto& [ms, amp] : cab.reflections) {
        if (ms <= 0.0) continue;
        const int index = static_cast<int>(std::lround(ms / 1000.0 * fs));
        if (index > 0 && index < length) ir[static_cast<size_t>(index)] += amp;
    }

    applyBiquad(ir, Kind::HighPass, cab.highpassHz, 0.9, 0.0, fs);
    applyBiquad(ir, Kind::Peaking, cab.resonanceHz, 1.2, cab.resonanceDb + resonanceDb, fs);
    applyBiquad(ir, Kind::Peaking, cab.scoopHz, 0.8, cab.scoopDb, fs);
    applyBiquad(ir, Kind::Peaking, cab.presenceHz, 1.4, cab.presenceDb * mic.presenceScale + presenceDb, fs);
    if (mic.shelves == 1) {
        applyBiquad(ir, Kind::HighShelf, 3500, 0.7, -4.5, fs);
    } else if (mic.shelves == 2) {
        applyBiquad(ir, Kind::LowShelf, 150, 0.7, -3.0, fs);
        applyBiquad(ir, Kind::HighShelf, 4000, 0.7, -3.5, fs);
    }
    for (int s = 0; s < cab.lowpassStages; ++s) applyBiquad(ir, Kind::LowPass, cab.lowpassHz, 0.7, 0.0, fs);

    const int fade = static_cast<int>(std::floor(length * 0.25));
    for (int i = 0; i < fade; ++i) ir[static_cast<size_t>(length - fade + i)] *= 1.0 - static_cast<double>(i) / fade;

    std::vector<float> out(ir.begin(), ir.end());
    const double ref = responseDbAt(out, fs, ReferenceHz);
    if (std::isfinite(ref)) {
        const float scale = static_cast<float>(std::pow(10.0, -ref / 20.0));
        for (auto& v : out) v *= scale;
    }
    return out;
}

std::vector<float> GuitarAmp::roomImpulse(double fs, float seconds, unsigned seed) {
    const int length = std::max(1, static_cast<int>(std::floor(fs * std::max(0.05f, seconds))));
    std::vector<double> data(static_cast<size_t>(length));
    juce::Random random(static_cast<juce::int64>(seed));
    for (int i = 0; i < length; ++i) {
        const double r = random.nextDouble();
        data[static_cast<size_t>(i)] = (r * 2.0 - 1.0) * std::pow(1.0 - static_cast<double>(i) / length, 2.6);
    }
    applyBiquad(data, Kind::LowPass, 7000, 0.7, 0.0, fs);
    applyBiquad(data, Kind::HighPass, 180, 0.7, 0.0, fs);
    return std::vector<float>(data.begin(), data.end());
}

float GuitarAmp::convolverNormalisation(const std::vector<float>& ir, double fs) {
    // Chromium's `Reverb::CalculateNormalizationScale`, which is what `normalize = true` runs.
    constexpr double GainCalibration = 0.00125;       // -58 dB
    constexpr double GainCalibrationSampleRate = 44100.0;
    constexpr double MinPower = 0.000125;
    double power = 0.0;
    for (const float v : ir) power += static_cast<double>(v) * v;
    power = std::sqrt(power / std::max<size_t>(1, ir.size()));
    if (!std::isfinite(power) || power < MinPower) power = MinPower;
    double scale = 1.0 / power;
    scale *= GainCalibration;
    if (fs > 0.0) scale *= GainCalibrationSampleRate / fs;
    return static_cast<float>(scale);
}

//==============================================================================================
// Control

bool GuitarAmp::setParam(std::string_view name, float v) {
    const bool on = v >= 0.5f;
    auto markWanted = [this] {
        wantCab.store(enabled && cabEnabled);
        wantReverb.store(enabled && reverbEnabled);
    };

    if (name == "enabled") { enabled = on; markWanted(); return true; }
    if (name == "input") { inputDb = std::clamp(v, -12.0f, 12.0f); return true; }
    if (name == "gate/enabled") { gateEnabled = on; return true; }
    if (name == "gate/threshold") { gateThresholdDb = std::clamp(v, -80.0f, -20.0f); return true; }
    if (name == "comp/enabled") { compEnabled = on; applyComp(); return true; }
    if (name == "comp/threshold") { compThresholdDb = std::clamp(v, -48.0f, 0.0f); applyComp(); return true; }
    if (name == "comp/ratio") { compRatio = std::clamp(v, 1.0f, 12.0f); applyComp(); return true; }
    if (name == "tone/bass") { bassDb = std::clamp(v, -12.0f, 12.0f); applyTone(); return true; }
    if (name == "tone/mid") { midDb = std::clamp(v, -12.0f, 12.0f); applyTone(); return true; }
    if (name == "tone/midHz") { midHz = std::clamp(v, 200.0f, 2000.0f); applyTone(); return true; }
    if (name == "tone/treble") { trebleDb = std::clamp(v, -12.0f, 12.0f); applyTone(); return true; }
    if (name == "drive/enabled") { driveEnabled = on; driveStale = true; return true; }
    if (name == "drive/amount") { driveAmount = std::clamp(v, 0.0f, 1.0f); driveStale = true; return true; }
    if (name == "drive/stages") {
        driveStages = std::clamp(static_cast<int>(std::lround(v)), 1, 3);
        driveStale = true;
        return true;
    }
    if (name == "drive/bias") { driveBias = std::clamp(v, 0.0f, 0.4f); driveStale = true; return true; }
    if (name == "cab/enabled") { cabEnabled = on; markWanted(); return true; }
    if (name == "cab/model") {
        cabModel.store(std::clamp(static_cast<int>(std::lround(v)), 0, 3));
        cabDirty.store(true);
        return true;
    }
    if (name == "cab/mic") {
        cabMic.store(std::clamp(static_cast<int>(std::lround(v)), 0, 2));
        cabDirty.store(true);
        return true;
    }
    if (name == "cab/presence") { cabPresenceDb.store(std::clamp(v, -9.0f, 9.0f)); cabDirty.store(true); applyTone(); return true; }
    if (name == "cab/resonance") { cabResonanceDb.store(std::clamp(v, -6.0f, 6.0f)); cabDirty.store(true); applyTone(); return true; }
    if (name == "cab/width") { cabWidth = std::clamp(v, 0.0f, 1.0f); return true; } // mono: accepted, unused
    if (name == "delay/enabled") { delayEnabled = on; return true; }
    if (name == "delay/time") { delayTimeSec = std::clamp(v, 0.02f, 1.5f); return true; }
    if (name == "delay/feedback") { delayFeedback = std::clamp(v, 0.0f, 0.9f); return true; }
    if (name == "delay/mix") { delayMix = std::clamp(v, 0.0f, 1.0f); return true; }
    if (name == "reverb/enabled") { reverbEnabled = on; markWanted(); return true; }
    if (name == "reverb/size") {
        const float next = std::clamp(v, 0.3f, 5.0f);
        // The web chain rebuilds only on a change larger than 50 ms, and so does this.
        if (std::abs(next - reverbSizeSec.load()) > 0.05f) { reverbSizeSec.store(next); reverbDirty.store(true); }
        return true;
    }
    if (name == "reverb/mix") { reverbMix = std::clamp(v, 0.0f, 1.0f); return true; }
    if (name == "output") { outputDb = std::clamp(v, -24.0f, 24.0f); return true; }
    if (name == "limiter/enabled") { limiterEnabled = on; return true; }
    if (name == "limiter/ceiling") { ceilingDb = std::clamp(v, -12.0f, 0.0f); return true; }
    return false;
}

void GuitarAmp::serviceBackground() {
    const double fs = preparedRate.load();
    if (fs <= 0.0) return;

    if (wantCab.load() && cabDirty.exchange(false)) {
        const auto model = static_cast<Cabinet>(cabModel.load());
        const auto mic = static_cast<Mic>(cabMic.load());
        const float pres = cabPresenceDb.load();
        const float reso = cabResonanceDb.load();
        // The web rack's two speakers: the right is 2.5 dB less present and 0.5 dB more
        // resonant. Mono here, so both go into one impulse.
        auto left = cabinetImpulse(fs, model, mic, pres, reso);
        auto right = cabinetImpulse(fs, model, mic, pres - 2.5f, reso + 0.5f);
        juce::AudioBuffer<float> buffer(1, static_cast<int>(left.size()));
        for (size_t i = 0; i < left.size(); ++i) {
            // At the web's centre each speaker goes through a panner at ~0.707 and the pair is
            // summed at 0.5, so the centre carries 0.3536 × (L + R).
            buffer.setSample(0, static_cast<int>(i), 0.35355339f * (left[i] + right[i]));
        }
        cabinet.loadImpulseResponse(std::move(buffer), fs, juce::dsp::Convolution::Stereo::no,
                                    juce::dsp::Convolution::Trim::no, juce::dsp::Convolution::Normalise::no);
    }

    if (wantReverb.load() && reverbDirty.exchange(false)) {
        auto tail = roomImpulse(fs, reverbSizeSec.load(), 0x7057u);
        const float scale = convolverNormalisation(tail, fs);
        juce::AudioBuffer<float> buffer(1, static_cast<int>(tail.size()));
        for (size_t i = 0; i < tail.size(); ++i) buffer.setSample(0, static_cast<int>(i), tail[i] * scale);
        reverb.loadImpulseResponse(std::move(buffer), fs, juce::dsp::Convolution::Stereo::no,
                                   juce::dsp::Convolution::Trim::no, juce::dsp::Convolution::Normalise::no);
    }
}

//==============================================================================================
// Processing

void GuitarAmp::rebuildCurve() {
    driveStale = false;
    const float amount = driveEnabled ? driveAmount : 0.0f;
    curveStages = driveEnabled ? driveStages : 0;
    curveActive = amount > 0.0f && curveStages > 0;
    if (curveActive) {
        tubeCurve(amount, driveBias, curve);
        makeup = driveMakeup(curve, curveStages);
    } else {
        makeup = 1.0f;
    }
}

float GuitarAmp::limitSample(float x, float ceiling) {
    const float peak = std::abs(x);
    const int cursor = limCursor;
    const float delayed = limDelay[static_cast<size_t>(cursor)];
    limDelay[static_cast<size_t>(cursor)] = x;
    limWindow[static_cast<size_t>(cursor)] = peak;
    limCursor = (cursor + 1) % limSize;

    const int capacity = limSize + 2;
    while (limTail != limHead) {
        const long long back = limDeque[static_cast<size_t>((limTail - 1 + capacity) % capacity)];
        if (limWindow[static_cast<size_t>(back % limSize)] > peak) break;
        limTail = (limTail - 1 + capacity) % capacity;
    }
    limDeque[static_cast<size_t>(limTail)] = limPosition;
    limTail = (limTail + 1) % capacity;
    const long long oldest = limPosition - limSize + 1;
    while (limTail != limHead && limDeque[static_cast<size_t>(limHead)] < oldest) limHead = (limHead + 1) % capacity;
    ++limPosition;

    const float windowPeak = limWindow[static_cast<size_t>(limDeque[static_cast<size_t>(limHead)] % limSize)];
    const float target = windowPeak > ceiling ? ceiling / windowPeak : 1.0f;
    if (target < limGain) {
        limGain -= std::min(limGain - target, (1.0f - target) / static_cast<float>(limSize) + 1e-6f);
        limSlowGain = std::min(limSlowGain, limGain);
    } else {
        limGain = target + (limGain - target) * limFastRel;
        limSlowGain = target + (limSlowGain - target) * limSlowRel;
        if (limSlowGain < limGain) limGain = limSlowGain;
    }

    const float applied = limiterEnabled ? limGain : 1.0f;
    float y = delayed * applied;
    if (limiterEnabled) y = std::clamp(y, -ceiling, ceiling);
    return y;
}

void GuitarAmp::process(float* x, int n) {
    if (!enabled || n <= 0) return;
    if (driveStale) rebuildCurve();

    const float trim = fromDb(inputDb);
    const float openAt = fromDb(gateThresholdDb);
    const float closeAt = openAt * 0.501f;

    // 1. DC block, trim, gate, compressor, tone stack.
    for (int i = 0; i < n; ++i) {
        float v = dcBlock.processSample(x[i]) * trim;

        const float peak = std::abs(v);
        gateEnv = peak > gateEnv ? peak : gateEnv * gateEnvCoef + peak * (1.0f - gateEnvCoef);
        if (gateEnv >= openAt) gateHold = gateHoldFrames;
        else if (gateHold > 0) --gateHold;
        const bool open = gateHold > 0 || gateEnv >= closeAt;
        if (!gateEnabled) {
            gateGain = 1.0f;
        } else {
            const float target = open ? 1.0f : 0.0f;
            const float c = target > gateGain ? gateAttack : gateRelease;
            gateGain = target + (gateGain - target) * c;
        }
        v *= gateGain;

        v = comp.processSample(v);
        x[i] = tone.processSample(v);
    }

    // 2. Drive: each stage 4× oversampled, then the interstage low-pass and DC blocker.
    for (int s = 0; s < 3; ++s) {
        if (curveActive && s < curveStages) {
            float* channels[] = { x };
            juce::dsp::AudioBlock<float> block(channels, 1, static_cast<size_t>(n));
            auto up = oversampler[s]->processSamplesUp(block);
            float* u = up.getChannelPointer(0);
            for (size_t i = 0; i < up.getNumSamples(); ++i) u[i] = readCurve(curve, u[i]);
            oversampler[s]->processSamplesDown(block);
        }
        if (s < 2) {
            for (int i = 0; i < n; ++i) x[i] = interDc[s].processSample(interLow[s].processSample(x[i]));
        }
    }

    // 3. Makeup, then the cabinet.
    for (int i = 0; i < n; ++i) x[i] *= makeup;
    // Run whenever the cabinet is switched on, loaded or not: `juce::dsp::Convolution` swaps a
    // freshly built impulse in *inside* `process`, so a stage that waited to be ready before
    // processing would never become ready. Until then it holds a one-sample unit impulse,
    // which passes the signal through — the same as the cabinet being off.
    if (cabEnabled) {
        float* channels[] = { x };
        juce::dsp::AudioBlock<float> block(channels, 1, static_cast<size_t>(n));
        cabinet.process(juce::dsp::ProcessContextReplacing<float>(block));
    }
    // Bypassed, the web rack's centre carries the signal itself (`cabBypass` at 1, `cabSum` at 1).

    // 4. Depth and presence.
    for (int i = 0; i < n; ++i) x[i] = post.processSample(x[i]);

    // 5. Dry + delay + reverb, output trim, limiter.
    // Processed whenever it is on (so a new tail can swap in), mixed only once a tail is in:
    // the placeholder unit impulse would otherwise add a second copy of the dry signal.
    bool reverbOn = false;
    if (reverbEnabled) {
        std::copy(x, x + n, wetBuffer.begin());
        float* channels[] = { wetBuffer.data() };
        juce::dsp::AudioBlock<float> block(channels, 1, static_cast<size_t>(n));
        reverb.process(juce::dsp::ProcessContextReplacing<float>(block));
        reverbOn = reverbReady();
    }

    const int ringSize = static_cast<int>(delayLine.size());
    const float delaySamples = static_cast<float>(std::clamp(static_cast<double>(delayTimeSec), 0.001, 2.0) * sampleRate);
    const float feedback = delayEnabled ? delayFeedback : 0.0f;
    const float delayWet = delayEnabled ? delayMix : 0.0f;
    const float outGain = fromDb(outputDb);
    const float ceiling = fromDb(ceilingDb);
    // The limiter holds the *centre* at the ceiling, as the web rack's does; √2 afterwards
    // turns the centre back into the mono signal the channel's panner expects.
    constexpr float CentreToMono = 1.41421356f;

    for (int i = 0; i < n; ++i) {
        const float dry = x[i];

        float readPos = static_cast<float>(delayWrite) - delaySamples;
        while (readPos < 0.0f) readPos += static_cast<float>(ringSize);
        const int r0 = static_cast<int>(readPos) % ringSize;
        const int r1 = (r0 + 1) % ringSize;
        const float frac = readPos - std::floor(readPos);
        const float tapped = delayLine[static_cast<size_t>(r0)] * (1.0f - frac) + delayLine[static_cast<size_t>(r1)] * frac;
        const float toned = delayTone.processSample(tapped);
        delayLine[static_cast<size_t>(delayWrite)] = dry + toned * feedback;
        delayWrite = (delayWrite + 1) % ringSize;

        float mix = dry + toned * delayWet;
        if (reverbOn) mix += wetBuffer[static_cast<size_t>(i)] * reverbMix;

        x[i] = limitSample(mix * outGain, ceiling) * CentreToMono;
    }
}

} // namespace dsp
