#include "BassRig.h"

#include <algorithm>
#include <cmath>

namespace dsp {

using rack::WebBiquad;
using K = rack::WebBiquad::Kind;

namespace {
/** Web Audio writes 0.7 on these, which it reads in dB. */
constexpr double PassQDb = 0.7;
} // namespace

float BassRig::powerGain(float p) {
    const float clamped = std::isfinite(p) ? std::clamp(p, 0.0f, 1.0f) : 0.0f;
    if (clamped <= 0.0f) return 0.0f;
    return 0.7f * std::pow(10.0f, (clamped - 1.0f) * 15.0f / 20.0f);
}

BassRig::BassRig(juce::dsp::ConvolutionMessageQueue& queue)
    : cabinet(juce::dsp::Convolution::Latency { 0 }, queue) {
    // `oversample = '2x'`: one 2× stage, the same half-band family `GuitarAmp` uses.
    powerOs = std::make_unique<juce::dsp::Oversampling<float>>(
        1, 1, juce::dsp::Oversampling<float>::filterHalfBandFIREquiripple, false, false);
    driveOs = std::make_unique<juce::dsp::Oversampling<float>>(
        1, 1, juce::dsp::Oversampling<float>::filterHalfBandFIREquiripple, false, false);
    rack::tubeCurve(PowerAmount, PowerHardness, PowerBias, powerCurve);
}

void BassRig::prepare(double newSampleRate, int maxBlockSize) {
    sampleRate = newSampleRate;
    preparedRate.store(sampleRate);
    const size_t block = static_cast<size_t>(std::max(1, maxBlockSize));

    dcBlock.set(K::HighPass, 18.0, PassQDb, 0.0, sampleRate);
    driveDc.set(K::HighPass, 30.0, PassQDb, 0.0, sampleRate);
    applyFilters();

    gate.prepare(sampleRate);
    limiter.prepare(sampleRate);
    comp.prepare(sampleRate, maxBlockSize);
    comp.setKnee(12.0f);
    comp.setDetectionMode(Compressor::DetectionMode::Peak);
    applyComp();

    for (auto* s : { &trim, &lowGain, &powerGainS, &driveTrim, &ampOut, &diGain, &outputTrim }) s->prepare(sampleRate);

    powerOs->initProcessing(block);
    driveOs->initProcessing(block);

    const juce::dsp::ProcessSpec spec { sampleRate, static_cast<juce::uint32>(block), 1 };
    cabinet.prepare(spec);

    for (auto* b : { &lowBuf, &highBuf, &powerBuf, &driveBuf, &ampBuf }) b->assign(block, 0.0f);

    driveStale = true;
    cabDirty.store(true);
    reset();
}

void BassRig::reset() {
    if (preparedRate.load() <= 0.0) return;
    for (auto* f : { &dcBlock, &driveDc, &lowBand[0], &lowBand[1], &highBand[0], &highBand[1], &powerHp[0],
                     &powerHp[1], &eqSub, &eqLowMid, &eqMid, &eqHigh })
        f->reset();
    for (auto& f : diTop) f.reset();
    gate.reset();
    limiter.reset();
    comp.reset();
    powerOs->reset();
    driveOs->reset();
    cabinet.reset();
    if (driveStale) rebuildDrive();
    applyGains();
    // Start at the targets rather than gliding up from silence after a reset.
    for (auto* s : { &trim, &lowGain, &powerGainS, &driveTrim, &ampOut, &diGain, &outputTrim }) s->jump(s->goal());
}

void BassRig::applyComp() {
    // The web writes threshold 0 and ratio 1 when the compressor is off; its node then adds no
    // makeup either, because the makeup is (1 / fullRangeGain)^0.6 and that gain is 1.
    comp.setEnabled(compEnabled);
    comp.setThreshold(compThresholdDb);
    comp.setRatio(compRatio);
    comp.setAttack(compAttackMs);
    comp.setRelease(compReleaseMs);
    const float makeupDb = compEnabled ? 0.6f * (1.0f - 1.0f / std::max(1.0f, compRatio)) * -compThresholdDb : 0.0f;
    comp.setMakeup(makeupDb);
}

void BassRig::applyFilters() {
    for (auto& f : lowBand) f.set(K::LowPass, crossoverHz, CrossoverQDb, 0.0, sampleRate);
    for (auto& f : highBand) f.set(K::HighPass, crossoverHz, CrossoverQDb, 0.0, sampleRate);
    for (auto& f : powerHp) f.set(K::HighPass, crossoverHz, CrossoverQDb, 0.0, sampleRate);
    eqSub.set(K::LowShelf, SubHz, 0.0, subDb, sampleRate);
    eqLowMid.set(K::Peaking, LowMidHz, 0.9, lowMidDb, sampleRate);
    eqMid.set(K::Peaking, MidHz, 0.8, midDb, sampleRate);
    eqHigh.set(K::HighShelf, HighHz, 0.0, highDb, sampleRate);
    // The DI's top follows the chosen cabinet's own corner, and opens when the cabinet is off.
    const double diHz = cabEnabled ? rack::cabinetSpec(cabinetTableIndex(cabModel.load())).lowpassHz
                                   : std::min(20000.0, sampleRate / 2.0 - 1.0);
    for (auto& f : diTop) f.set(K::LowPass, diHz, CrossoverQDb, 0.0, sampleRate);
}

void BassRig::applyGains() {
    trim.set(rack::fromDb(inputDb));
    lowGain.set(std::pow(10.0f, lowDb / 20.0f));
    powerGainS.set(powerGain(power));
    driveTrim.set(makeup);
    const float di = std::clamp(diMix, 0.0f, 1.0f);
    diGain.set(static_cast<float>(std::sin(di * rack::Pi / 2.0)));
    ampOut.set(static_cast<float>(std::cos(di * rack::Pi / 2.0)));
    outputTrim.set(std::pow(10.0f, outputDb / 20.0f));
}

void BassRig::rebuildDrive() {
    driveStale = false;
    const float amount = driveEnabled ? driveAmount : 0.0f;
    driveActive = amount > 0.0f;
    if (driveActive) {
        rack::tubeCurve(amount, StageHardness, driveBias, driveCurve);
        makeup = rack::driveMakeup(driveCurve, 1, DriveTargetRms);
    } else {
        makeup = 1.0f; // a null curve is a pass-through: nothing to make up
    }
    driveTrim.set(makeup);
}

bool BassRig::setParam(std::string_view name, float v) {
    const bool on = v >= 0.5f;
    if (name == "enabled") { enabled = on; wantCab.store(enabled && cabEnabled); return true; }
    if (name == "input") { inputDb = std::clamp(v, -12.0f, 12.0f); applyGains(); return true; }
    if (name == "gate/enabled") { gateEnabled = on; return true; }
    if (name == "gate/threshold") { gateThresholdDb = std::clamp(v, -80.0f, -20.0f); return true; }
    if (name == "comp/enabled") { compEnabled = on; applyComp(); return true; }
    if (name == "comp/threshold") { compThresholdDb = std::clamp(v, -48.0f, 0.0f); applyComp(); return true; }
    if (name == "comp/ratio") { compRatio = std::clamp(v, 1.0f, 12.0f); applyComp(); return true; }
    if (name == "comp/attack") { compAttackMs = std::clamp(v, 1.0f, 60.0f); applyComp(); return true; }
    if (name == "comp/release") { compReleaseMs = std::clamp(v, 40.0f, 800.0f); applyComp(); return true; }
    if (name == "crossover") { crossoverHz = std::clamp(v, 80.0f, 400.0f); applyFilters(); return true; }
    if (name == "low") { lowDb = std::clamp(v, -12.0f, 12.0f); applyGains(); return true; }
    if (name == "power") { power = std::clamp(v, 0.0f, 1.0f); applyGains(); return true; }
    if (name == "drive/enabled") { driveEnabled = on; driveStale = true; return true; }
    if (name == "drive/amount") { driveAmount = std::clamp(v, 0.0f, 1.0f); driveStale = true; return true; }
    if (name == "drive/bias") { driveBias = std::clamp(v, 0.0f, 0.4f); driveStale = true; return true; }
    if (name == "eq/sub") { subDb = std::clamp(v, -12.0f, 12.0f); applyFilters(); return true; }
    if (name == "eq/lowMid") { lowMidDb = std::clamp(v, -12.0f, 12.0f); applyFilters(); return true; }
    if (name == "eq/mid") { midDb = std::clamp(v, -12.0f, 12.0f); applyFilters(); return true; }
    if (name == "eq/high") { highDb = std::clamp(v, -12.0f, 12.0f); applyFilters(); return true; }
    if (name == "cab/enabled") { cabEnabled = on; wantCab.store(enabled && cabEnabled); applyFilters(); return true; }
    if (name == "cab/model") {
        cabModel.store(std::clamp(static_cast<int>(std::lround(v)), 0, CabinetCount - 1));
        cabDirty.store(true);
        applyFilters();
        return true;
    }
    if (name == "cab/presence") { cabPresenceDb.store(std::clamp(v, -9.0f, 9.0f)); cabDirty.store(true); return true; }
    if (name == "cab/resonance") { cabResonanceDb.store(std::clamp(v, -9.0f, 9.0f)); cabDirty.store(true); return true; }
    if (name == "di") { diMix = std::clamp(v, 0.0f, 1.0f); applyGains(); return true; }
    if (name == "output") { outputDb = std::clamp(v, -24.0f, 24.0f); applyGains(); return true; }
    if (name == "limiter/enabled") { limiterEnabled = on; return true; }
    if (name == "limiter/ceiling") { ceilingDb = std::clamp(v, -12.0f, 0.0f); return true; }
    return false;
}

void BassRig::serviceBackground() {
    const double fs = preparedRate.load();
    if (fs <= 0.0) return;
    if (!(wantCab.load() && cabDirty.exchange(false))) return;
    // One speaker, centre mic, `normalize = false` — what the web's single convolver holds.
    auto ir = rack::cabinetImpulse(fs, cabinetTableIndex(cabModel.load()), cabPresenceDb.load(), cabResonanceDb.load());
    juce::AudioBuffer<float> buffer(1, static_cast<int>(ir.size()));
    for (size_t i = 0; i < ir.size(); ++i) buffer.setSample(0, static_cast<int>(i), ir[i]);
    cabinet.loadImpulseResponse(std::move(buffer), fs, juce::dsp::Convolution::Stereo::no,
                                juce::dsp::Convolution::Trim::no, juce::dsp::Convolution::Normalise::no);
}

void BassRig::process(float* x, int n) {
    if (!enabled || n <= 0 || lowBuf.empty()) return;
    // In pieces no longer than the buffers `prepare` sized — never a skipped tail.
    const int chunk = static_cast<int>(lowBuf.size());
    for (int at = 0; at < n; at += chunk) processChunk(x + at, std::min(chunk, n - at));
}

void BassRig::processChunk(float* x, int n) {
    if (driveStale) rebuildDrive();

    // 1. DC block, trim, gate, compressor, then the two crossover bands.
    const bool powerOn = power > 0.0f || powerGainS.value() > 1e-6f;
    for (int i = 0; i < n; ++i) {
        float v = dcBlock.process(x[i]) * trim.next();
        v = gate.process(v, gateEnabled, gateThresholdDb);
        v = comp.processSample(v);
        const float low = lowBand[1].process(lowBand[0].process(v));
        lowBuf[static_cast<size_t>(i)] = low;
        highBuf[static_cast<size_t>(i)] = highBand[1].process(highBand[0].process(v));
        if (powerOn) powerBuf[static_cast<size_t>(i)] = low;
    }

    // 2. Power: the low band, shaped at 2×, high-passed at the crossover so its own fundamental
    // never rejoins. The web disconnects this branch when power is 0, so it is skipped then.
    if (powerOn) {
        float* channels[] = { powerBuf.data() };
        juce::dsp::AudioBlock<float> block(channels, 1, static_cast<size_t>(n));
        auto up = powerOs->processSamplesUp(block);
        float* u = up.getChannelPointer(0);
        for (size_t i = 0; i < up.getNumSamples(); ++i) u[i] = rack::readCurve(powerCurve, u[i]);
        powerOs->processSamplesDown(block);
        for (int i = 0; i < n; ++i) {
            const float y = powerHp[1].process(powerHp[0].process(powerBuf[static_cast<size_t>(i)]));
            powerBuf[static_cast<size_t>(i)] = y * powerGainS.next();
        }
    }

    // 3. Drive on the high band only. A null curve is a pass-through in Web Audio.
    std::copy(highBuf.begin(), highBuf.begin() + n, driveBuf.begin());
    if (driveActive) {
        float* channels[] = { driveBuf.data() };
        juce::dsp::AudioBlock<float> block(channels, 1, static_cast<size_t>(n));
        auto up = driveOs->processSamplesUp(block);
        float* u = up.getChannelPointer(0);
        for (size_t i = 0; i < up.getNumSamples(); ++i) u[i] = rack::readCurve(driveCurve, u[i]);
        driveOs->processSamplesDown(block);
    }

    // 4. ampSum = LOW + power + driven high band, then the graphic EQ.
    for (int i = 0; i < n; ++i) {
        const size_t k = static_cast<size_t>(i);
        const float lowG = lowBuf[k] * lowGain.next();
        lowBuf[k] = lowG; // the DI taps it after LOW
        const float driven = driveDc.process(driveBuf[k] * driveTrim.next());
        float sum = lowG + driven + (powerOn ? powerBuf[k] : 0.0f);
        sum = eqHigh.process(eqMid.process(eqLowMid.process(eqSub.process(sum))));
        ampBuf[k] = sum;
    }

    // 5. The cabinet. Run whenever it is on, loaded or not: `juce::dsp::Convolution` swaps a new
    // impulse in inside `process`, and until then holds a unit impulse — the web's bypass.
    if (cabEnabled) {
        float* channels[] = { ampBuf.data() };
        juce::dsp::AudioBlock<float> block(channels, 1, static_cast<size_t>(n));
        cabinet.process(juce::dsp::ProcessContextReplacing<float>(block));
    }

    // 6. The amp and the DI, equal power; output trim; limiter; √2 for the channel's panner.
    const float ceiling = rack::fromDb(ceilingDb);
    for (int i = 0; i < n; ++i) {
        const size_t k = static_cast<size_t>(i);
        float di = lowBuf[k] + highBuf[k];
        for (auto& f : diTop) di = f.process(di);
        const float mix = ampBuf[k] * ampOut.next() + di * diGain.next();
        x[i] = limiter.process(mix * outputTrim.next(), limiterEnabled, ceiling) * rack::CentreToMono;
    }
}

} // namespace dsp
