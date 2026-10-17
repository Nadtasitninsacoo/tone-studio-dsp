#include "VocalRig.h"
#include "GuitarAmp.h"

#include <algorithm>
#include <cmath>

namespace dsp {

namespace {
using K = rack::WebBiquad::Kind;
constexpr double DcQDb = 0.7;
constexpr double MaxDelaySec = 2.0; // `createDelay(2.0)`

float webMakeupDb(bool on, float thresholdDb, float ratio) {
    return on ? 0.6f * (1.0f - 1.0f / std::max(1.0f, ratio)) * -thresholdDb : 0.0f;
}
} // namespace

VocalRig::VocalRig(juce::dsp::ConvolutionMessageQueue& queue)
    : reverb(juce::dsp::Convolution::NonUniform { 256 }, queue) {}

void VocalRig::prepare(double newSampleRate, int maxBlockSize) {
    sampleRate = newSampleRate;
    preparedRate.store(sampleRate);
    const size_t block = static_cast<size_t>(std::max(1, maxBlockSize));

    dcBlock.set(K::HighPass, 30.0, DcQDb, 0.0, sampleRate);
    for (auto& f : dsLow) f.set(K::LowPass, DeEssSplitHz, DeEssQDb, 0.0, sampleRate);
    for (auto& f : dsHigh) f.set(K::HighPass, DeEssSplitHz, DeEssQDb, 0.0, sampleRate);
    delayDamp.set(K::LowPass, DelayDampHz, DefaultQDb, 0.0, sampleRate);
    applyFilters();

    gate.prepare(sampleRate);
    limiter.prepare(sampleRate);
    deEss.prepare(sampleRate, maxBlockSize);
    deEss.setKnee(4.0f);
    deEss.setAttack(5.0f);
    deEss.setRelease(50.0f);
    deEss.setDetectionMode(Compressor::DetectionMode::Peak);
    comp.prepare(sampleRate, maxBlockSize);
    comp.setKnee(8.0f);
    comp.setDetectionMode(Compressor::DetectionMode::Peak);
    applyCompressors();

    for (auto* s : { &trim, &delaySamples, &delayFb, &delayWet, &reverbWet, &outputTrim }) s->prepare(sampleRate);
    reverb.prepare({ sampleRate, static_cast<juce::uint32>(block), 1 });
    dryBuf.assign(block, 0.0f);
    wetBuf.assign(block, 0.0f);
    delayLine.assign(static_cast<size_t>(sampleRate * MaxDelaySec) + 4, 0.0f);

    reverbDirty.store(true);
    reset();
}

void VocalRig::reset() {
    if (preparedRate.load() <= 0.0) return;
    for (auto* f : { &dcBlock, &lowCut, &dsLow[0], &dsLow[1], &dsHigh[0], &dsHigh[1], &body, &presence, &air, &delayDamp }) f->reset();
    gate.reset();
    limiter.reset();
    deEss.reset();
    comp.reset();
    reverb.reset();
    std::fill(delayLine.begin(), delayLine.end(), 0.0f);
    delayWrite = 0;
    applyGains();
    for (auto* s : { &trim, &delaySamples, &delayFb, &delayWet, &reverbWet, &outputTrim }) s->jump(s->goal());
}

void VocalRig::applyFilters() {
    // The web writes 20 Hz rather than switching the low cut out of the path.
    lowCut.set(K::HighPass, lowCutEnabled ? LowCutHz : 20.0f, LowCutQDb, 0.0, sampleRate);
    body.set(K::LowShelf, BodyHz, 0.0, bodyDb, sampleRate);
    presence.set(K::Peaking, PresenceHz, 0.8, presenceDb, sampleRate);
    air.set(K::HighShelf, AirHz, 0.0, airDb, sampleRate);
}

void VocalRig::applyCompressors() {
    deEss.setEnabled(deEssEnabled);
    deEss.setThreshold(deEssThresholdDb);
    deEss.setRatio(deEssRatio);
    deEss.setMakeup(webMakeupDb(deEssEnabled, deEssThresholdDb, deEssRatio));
    comp.setEnabled(compEnabled);
    comp.setThreshold(compThresholdDb);
    comp.setRatio(compRatio);
    // The web writes 10 ms / 100 ms while the compressor is off; it compresses nothing either way.
    comp.setAttack((compEnabled ? compAttackSec : 0.01f) * 1000.0f);
    comp.setRelease((compEnabled ? compReleaseSec : 0.1f) * 1000.0f);
    comp.setMakeup(webMakeupDb(compEnabled, compThresholdDb, compRatio));
}

void VocalRig::applyGains() {
    trim.set(rack::fromDb(inputDb));
    delaySamples.set(static_cast<float>((delayEnabled ? delayTimeMs / 1000.0 : 0.0) * sampleRate));
    delayFb.set(delayEnabled ? delayFeedback : 0.0f);
    delayWet.set(delayEnabled ? delayMix : 0.0f);
    reverbWet.set(reverbEnabled && reverbReady() ? reverbMix : 0.0f);
    outputTrim.set(std::pow(10.0f, outputDb / 20.0f));
}

bool VocalRig::setParam(std::string_view name, float v) {
    const bool on = v >= 0.5f;
    if (name == "enabled") { enabled = on; wantReverb.store(enabled && reverbEnabled); return true; }
    if (name == "input") { inputDb = std::clamp(v, -12.0f, 12.0f); applyGains(); return true; }
    if (name == "gate/enabled") { gateEnabled = on; return true; }
    if (name == "gate/threshold") { gateThresholdDb = std::clamp(v, -80.0f, -20.0f); return true; }
    if (name == "deesser/enabled") { deEssEnabled = on; applyCompressors(); return true; }
    if (name == "deesser/threshold") { deEssThresholdDb = std::clamp(v, -60.0f, 0.0f); applyCompressors(); return true; }
    if (name == "deesser/ratio") { deEssRatio = std::clamp(v, 1.0f, 10.0f); applyCompressors(); return true; }
    if (name == "comp/enabled") { compEnabled = on; applyCompressors(); return true; }
    if (name == "comp/threshold") { compThresholdDb = std::clamp(v, -60.0f, 0.0f); applyCompressors(); return true; }
    if (name == "comp/ratio") { compRatio = std::clamp(v, 1.0f, 10.0f); applyCompressors(); return true; }
    // Seconds on the wire, as the web rack stores them.
    if (name == "comp/attack") { compAttackSec = std::clamp(v, 0.001f, 0.1f); applyCompressors(); return true; }
    if (name == "comp/release") { compReleaseSec = std::clamp(v, 0.01f, 1.0f); applyCompressors(); return true; }
    if (name == "eq/lowCut") { lowCutEnabled = on; applyFilters(); return true; }
    if (name == "eq/body") { bodyDb = std::clamp(v, -12.0f, 12.0f); applyFilters(); return true; }
    if (name == "eq/presence") { presenceDb = std::clamp(v, -12.0f, 12.0f); applyFilters(); return true; }
    if (name == "eq/air") { airDb = std::clamp(v, -12.0f, 12.0f); applyFilters(); return true; }
    if (name == "delay/enabled") { delayEnabled = on; applyGains(); return true; }
    if (name == "delay/time") { delayTimeMs = std::clamp(v, 50.0f, 2000.0f); applyGains(); return true; }
    if (name == "delay/feedback") { delayFeedback = std::clamp(v, 0.0f, 0.95f); applyGains(); return true; }
    if (name == "delay/mix") { delayMix = std::clamp(v, 0.0f, 1.0f); applyGains(); return true; }
    if (name == "reverb/enabled") { reverbEnabled = on; wantReverb.store(enabled && reverbEnabled); applyGains(); return true; }
    if (name == "reverb/size") {
        const float next = std::clamp(v, 0.3f, 5.0f);
        if (std::abs(next - reverbSizeSec.load()) > 0.05f) { reverbSizeSec.store(next); reverbDirty.store(true); }
        return true;
    }
    if (name == "reverb/mix") { reverbMix = std::clamp(v, 0.0f, 1.0f); applyGains(); return true; }
    if (name == "output") { outputDb = std::clamp(v, -24.0f, 24.0f); applyGains(); return true; }
    if (name == "limiter/enabled") { limiterEnabled = on; return true; }
    if (name == "limiter/ceiling") { ceilingDb = std::clamp(v, -12.0f, 0.0f); return true; }
    return false;
}

void VocalRig::serviceBackground() {
    const double fs = preparedRate.load();
    if (fs <= 0.0) return;
    if (!(wantReverb.load() && reverbDirty.exchange(false))) return;
    auto tail = GuitarAmp::roomImpulse(fs, reverbSizeSec.load(), 0x70C1u);
    const float scale = GuitarAmp::convolverNormalisation(tail, fs);
    juce::AudioBuffer<float> buffer(1, static_cast<int>(tail.size()));
    for (size_t i = 0; i < tail.size(); ++i) buffer.setSample(0, static_cast<int>(i), tail[i] * scale);
    reverb.loadImpulseResponse(std::move(buffer), fs, juce::dsp::Convolution::Stereo::no,
                               juce::dsp::Convolution::Trim::no, juce::dsp::Convolution::Normalise::no);
}

void VocalRig::process(float* x, int n) {
    if (!enabled || n <= 0 || dryBuf.empty()) return;
    const int chunk = static_cast<int>(dryBuf.size());
    for (int at = 0; at < n; at += chunk) processChunk(x + at, std::min(chunk, n - at));
}

void VocalRig::processChunk(float* x, int n) {
    if (reverbEnabled && reverbReady() && reverbWet.goal() != reverbMix) reverbWet.set(reverbMix);

    // 1. DC, trim, gate, low cut, de-esser, compressor, EQ — the dry signal.
    for (int i = 0; i < n; ++i) {
        float v = dcBlock.process(x[i]) * trim.next();
        v = gate.process(v, gateEnabled, gateThresholdDb);
        v = lowCut.process(v);
        v = dsLow[1].process(dsLow[0].process(v)) + deEss.processSample(dsHigh[1].process(dsHigh[0].process(v)));
        v = comp.processSample(v);
        dryBuf[static_cast<size_t>(i)] = air.process(presence.process(body.process(v)));
    }

    // 2. The reverb, while it is on, so a new tail can swap in.
    const bool reverbOn = reverbEnabled;
    if (reverbOn) {
        std::copy(dryBuf.begin(), dryBuf.begin() + n, wetBuf.begin());
        float* channels[] = { wetBuf.data() };
        juce::dsp::AudioBlock<float> block(channels, 1, static_cast<size_t>(n));
        reverb.process(juce::dsp::ProcessContextReplacing<float>(block));
    }

    // 3. Delay (input = dry + damped × feedback, out = damped × mix), output trim, limiter, √2.
    const int ring = static_cast<int>(delayLine.size());
    const float ceiling = rack::fromDb(ceilingDb);
    for (int i = 0; i < n; ++i) {
        const size_t k = static_cast<size_t>(i);
        const float dry = dryBuf[k];
        // A DelayNode in a cycle cannot be shorter than one render quantum (128 frames) in Web Audio.
        const float d = std::clamp(delaySamples.next(), 128.0f, static_cast<float>(ring - 2));
        float readPos = static_cast<float>(delayWrite) - d;
        while (readPos < 0.0f) readPos += static_cast<float>(ring);
        const int r0 = static_cast<int>(readPos) % ring;
        const int r1 = (r0 + 1) % ring;
        const float frac = readPos - std::floor(readPos);
        const float tapped = delayLine[static_cast<size_t>(r0)] * (1.0f - frac) + delayLine[static_cast<size_t>(r1)] * frac;
        const float damped = delayDamp.process(tapped);
        delayLine[static_cast<size_t>(delayWrite)] = dry + damped * delayFb.next();
        delayWrite = (delayWrite + 1) % ring;

        float mix = dry + damped * delayWet.next();
        if (reverbOn) mix += wetBuf[k] * reverbWet.next();
        x[i] = limiter.process(mix * outputTrim.next(), limiterEnabled, ceiling) * rack::CentreToMono;
    }
}

} // namespace dsp
