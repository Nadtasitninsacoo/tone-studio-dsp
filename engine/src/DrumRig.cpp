#include "DrumRig.h"
#include "GuitarAmp.h"

#include <algorithm>
#include <cmath>

namespace dsp {

namespace {
using K = rack::WebBiquad::Kind;
constexpr double PassQDb = 0.7; // the DC blocker's Q, read in dB

/** `DynamicsCompressorNode`'s own makeup: (1 / fullRangeGain)^0.6, in dB. 0 when it is off. */
float webMakeupDb(bool on, float thresholdDb, float ratio) {
    return on ? 0.6f * (1.0f - 1.0f / std::max(1.0f, ratio)) * -thresholdDb : 0.0f;
}
} // namespace

double DrumRig::kickQ(float tight) {
    const double t = std::isfinite(tight) ? std::clamp(static_cast<double>(tight), 0.0, 1.0) : 0.0;
    return 0.7 + t * (4.5 - 0.7);
}

float DrumRig::depthGain(float d) {
    return (std::isfinite(d) ? std::clamp(d, 0.0f, 1.0f) : 0.0f) * 0.8f;
}

DrumRig::DrumRig(juce::dsp::ConvolutionMessageQueue& queue)
    : room(juce::dsp::Convolution::NonUniform { 256 }, queue) {
    driveOs = std::make_unique<juce::dsp::Oversampling<float>>(
        1, 1, juce::dsp::Oversampling<float>::filterHalfBandFIREquiripple, false, false);
}

void DrumRig::prepare(double newSampleRate, int maxBlockSize) {
    sampleRate = newSampleRate;
    preparedRate.store(sampleRate);
    const size_t block = static_cast<size_t>(std::max(1, maxBlockSize));

    dcBlock.set(K::HighPass, 20.0, PassQDb, 0.0, sampleRate);
    applyFilters();

    gate.prepare(sampleRate);
    limiter.prepare(sampleRate);
    crush.prepare(sampleRate, maxBlockSize);
    crush.setKnee(3.0f);
    crush.setAttack(2.0f);
    crush.setRelease(120.0f);
    crush.setDetectionMode(Compressor::DetectionMode::Peak);
    glue.prepare(sampleRate, maxBlockSize);
    glue.setKnee(8.0f);
    glue.setAttack(20.0f);
    glue.setRelease(220.0f);
    glue.setDetectionMode(Compressor::DetectionMode::Peak);
    applyCompressors();

    for (auto* s : { &trim, &depthBlend, &driveTrim, &crushGain, &roomGain, &outputTrim }) s->prepare(sampleRate);
    driveOs->initProcessing(block);
    room.prepare({ sampleRate, static_cast<juce::uint32>(block), 1 });
    for (auto* b : { &mainBuf, &depthBuf, &roomBuf }) b->assign(block, 0.0f);

    driveStale = true;
    roomDirty.store(true);
    reset();
}

void DrumRig::reset() {
    if (preparedRate.load() <= 0.0) return;
    for (auto* f : { &dcBlock, &subCut[0], &subCut[1], &kick, &depth, &box, &snap }) f->reset();
    gate.reset();
    limiter.reset();
    crush.reset();
    glue.reset();
    driveOs->reset();
    room.reset();
    if (driveStale) rebuildDrive();
    applyGains();
    for (auto* s : { &trim, &depthBlend, &driveTrim, &crushGain, &roomGain, &outputTrim }) s->jump(s->goal());
}

void DrumRig::applyFilters() {
    // A cut above the lift would cancel it: the web holds the cut at or below the kick.
    const double cutHz = std::min(subCutHz, kickHz);
    for (auto& f : subCut) f.set(K::HighPass, cutHz, SubCutQDb, 0.0, sampleRate);
    kick.set(K::Peaking, kickHz, kickQ(kickTight), kickDb, sampleRate);
    depth.set(K::BandPass, kickHz, DepthQ, 0.0, sampleRate);
    box.set(K::Peaking, BoxHz, 1.1, boxDb, sampleRate);
    snap.set(K::HighShelf, SnapHz, 0.0, snapDb, sampleRate);
}

void DrumRig::applyCompressors() {
    // Off is threshold 0 and ratio 1 on the web node, which compresses nothing and adds no makeup.
    crush.setEnabled(crushEnabled);
    crush.setThreshold(crushThresholdDb);
    crush.setRatio(crushRatio);
    crush.setMakeup(webMakeupDb(crushEnabled, crushThresholdDb, crushRatio));
    glue.setEnabled(glueEnabled);
    glue.setThreshold(glueThresholdDb);
    glue.setRatio(glueRatio);
    glue.setMakeup(webMakeupDb(glueEnabled, glueThresholdDb, glueRatio));
}

void DrumRig::applyGains() {
    trim.set(rack::fromDb(inputDb));
    depthBlend.set(depthGain(kickDepth));
    driveTrim.set(makeup);
    // PUNCH is the crushed copy's level, and only while the crush is on.
    crushGain.set(crushEnabled ? std::clamp(punch, 0.0f, 1.0f) : 0.0f);
    // The room is heard once a tail is in; until then the web's `room.buffer` is null.
    roomGain.set(roomEnabled && roomReady() ? roomMix : 0.0f);
    outputTrim.set(std::pow(10.0f, outputDb / 20.0f));
}

void DrumRig::rebuildDrive() {
    driveStale = false;
    const float amount = driveEnabled ? driveAmount : 0.0f;
    driveActive = amount > 0.0f;
    if (driveActive) {
        rack::saturationCurve(amount, DriveHardness, driveCurve);
        makeup = rack::driveMakeup(driveCurve, 1, DriveTargetRms);
    } else {
        makeup = 1.0f;
    }
    driveTrim.set(makeup);
}

bool DrumRig::setParam(std::string_view name, float v) {
    const bool on = v >= 0.5f;
    if (name == "enabled") { enabled = on; wantRoom.store(enabled && roomEnabled); return true; }
    if (name == "input") { inputDb = std::clamp(v, -12.0f, 12.0f); applyGains(); return true; }
    if (name == "gate/enabled") { gateEnabled = on; return true; }
    if (name == "gate/threshold") { gateThresholdDb = std::clamp(v, -80.0f, -20.0f); return true; }
    if (name == "kick/hz") { kickHz = std::clamp(v, 40.0f, 100.0f); applyFilters(); return true; }
    if (name == "kick/db") { kickDb = std::clamp(v, -9.0f, 12.0f); applyFilters(); return true; }
    if (name == "kick/tight") { kickTight = std::clamp(v, 0.0f, 1.0f); applyFilters(); return true; }
    if (name == "kick/subCut") { subCutHz = std::clamp(v, 20.0f, 80.0f); applyFilters(); return true; }
    if (name == "kick/depth") { kickDepth = std::clamp(v, 0.0f, 1.0f); applyGains(); return true; }
    if (name == "eq/box") { boxDb = std::clamp(v, -12.0f, 12.0f); applyFilters(); return true; }
    if (name == "eq/snap") { snapDb = std::clamp(v, -12.0f, 12.0f); applyFilters(); return true; }
    if (name == "drive/enabled") { driveEnabled = on; driveStale = true; return true; }
    if (name == "drive/amount") { driveAmount = std::clamp(v, 0.0f, 1.0f); driveStale = true; return true; }
    if (name == "crush/enabled") { crushEnabled = on; applyCompressors(); applyGains(); return true; }
    if (name == "crush/threshold") { crushThresholdDb = std::clamp(v, -48.0f, 0.0f); applyCompressors(); return true; }
    if (name == "crush/ratio") { crushRatio = std::clamp(v, 1.0f, 20.0f); applyCompressors(); return true; }
    if (name == "punch") { punch = std::clamp(v, 0.0f, 1.0f); applyGains(); return true; }
    if (name == "room/enabled") { roomEnabled = on; wantRoom.store(enabled && roomEnabled); applyGains(); return true; }
    if (name == "room/size") {
        const float next = std::clamp(v, 0.3f, 5.0f);
        // The web rebuilds only on a change larger than 50 ms, and so does this.
        if (std::abs(next - roomSizeSec.load()) > 0.05f) { roomSizeSec.store(next); roomDirty.store(true); }
        return true;
    }
    if (name == "room/mix") { roomMix = std::clamp(v, 0.0f, 1.0f); applyGains(); return true; }
    if (name == "glue/enabled") { glueEnabled = on; applyCompressors(); return true; }
    if (name == "glue/threshold") { glueThresholdDb = std::clamp(v, -48.0f, 0.0f); applyCompressors(); return true; }
    if (name == "glue/ratio") { glueRatio = std::clamp(v, 1.0f, 12.0f); applyCompressors(); return true; }
    if (name == "output") { outputDb = std::clamp(v, -24.0f, 24.0f); applyGains(); return true; }
    if (name == "limiter/enabled") { limiterEnabled = on; return true; }
    if (name == "limiter/ceiling") { ceilingDb = std::clamp(v, -12.0f, 0.0f); return true; }
    return false;
}

void DrumRig::serviceBackground() {
    const double fs = preparedRate.load();
    if (fs <= 0.0) return;
    if (!(wantRoom.load() && roomDirty.exchange(false))) return;
    // `roomImpulse(sr, size)` from cabinet.ts, scaled the way `normalize = true` scales it.
    auto tail = GuitarAmp::roomImpulse(fs, roomSizeSec.load(), 0xD2u);
    const float scale = GuitarAmp::convolverNormalisation(tail, fs);
    juce::AudioBuffer<float> buffer(1, static_cast<int>(tail.size()));
    for (size_t i = 0; i < tail.size(); ++i) buffer.setSample(0, static_cast<int>(i), tail[i] * scale);
    room.loadImpulseResponse(std::move(buffer), fs, juce::dsp::Convolution::Stereo::no,
                             juce::dsp::Convolution::Trim::no, juce::dsp::Convolution::Normalise::no);
}

void DrumRig::process(float* x, int n) {
    if (!enabled || n <= 0 || mainBuf.empty()) return;
    const int chunk = static_cast<int>(mainBuf.size());
    for (int at = 0; at < n; at += chunk) processChunk(x + at, std::min(chunk, n - at));
}

void DrumRig::processChunk(float* x, int n) {
    if (driveStale) rebuildDrive();
    // The room's gain waits for its tail, as the web's does; `roomReady` changes inside `process`.
    if (roomEnabled && roomReady() && roomGain.goal() != roomMix) roomGain.set(roomMix);

    // 1. DC block, trim, gate, sub cut; then the kick lift and the depth resonator side by side.
    const bool depthOn = kickDepth > 0.0f || depthBlend.value() > 1e-6f;
    for (int i = 0; i < n; ++i) {
        float v = dcBlock.process(x[i]) * trim.next();
        v = gate.process(v, gateEnabled, gateThresholdDb);
        v = subCut[1].process(subCut[0].process(v));
        depthBuf[static_cast<size_t>(i)] = depthOn ? depth.process(v) * depthBlend.next() : 0.0f;
        mainBuf[static_cast<size_t>(i)] = snap.process(box.process(kick.process(v)));
    }

    // 2. The drive, at 2×. A null curve is a pass-through in Web Audio.
    if (driveActive) {
        float* channels[] = { mainBuf.data() };
        juce::dsp::AudioBlock<float> block(channels, 1, static_cast<size_t>(n));
        auto up = driveOs->processSamplesUp(block);
        float* u = up.getChannelPointer(0);
        for (size_t i = 0; i < up.getNumSamples(); ++i) u[i] = rack::readCurve(driveCurve, u[i]);
        driveOs->processSamplesDown(block);
    }
    for (int i = 0; i < n; ++i) mainBuf[static_cast<size_t>(i)] *= driveTrim.next();

    // 3. The room, from the split. Processed while it is on so a new tail can swap in.
    const bool roomOn = roomEnabled;
    if (roomOn) {
        std::copy(mainBuf.begin(), mainBuf.begin() + n, roomBuf.begin());
        float* channels[] = { roomBuf.data() };
        juce::dsp::AudioBlock<float> block(channels, 1, static_cast<size_t>(n));
        room.process(juce::dsp::ProcessContextReplacing<float>(block));
    }

    // 4. dry + crush × PUNCH + room × MIX + depth → glue → output trim → limiter → √2.
    const float ceiling = rack::fromDb(ceilingDb);
    for (int i = 0; i < n; ++i) {
        const size_t k = static_cast<size_t>(i);
        const float split = mainBuf[k];
        const float crushed = crush.processSample(split) * crushGain.next();
        const float wet = roomOn ? roomBuf[k] * roomGain.next() : 0.0f;
        const float bus = glue.processSample(split + crushed + wet + depthBuf[k]);
        x[i] = limiter.process(bus * outputTrim.next(), limiterEnabled, ceiling) * rack::CentreToMono;
    }
}

} // namespace dsp
