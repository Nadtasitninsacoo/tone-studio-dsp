#include "LightRig.h"
#include "GuitarAmp.h"

#include <algorithm>
#include <cmath>

namespace dsp {

namespace {
using K = rack::WebBiquad::Kind;
constexpr double MaxDelaySec = 1.5; // `createDelay(1.5)`

float webMakeupDb(bool on, float thresholdDb, float ratio) {
    return on ? 0.6f * (1.0f - 1.0f / std::max(1.0f, ratio)) * -thresholdDb : 0.0f;
}
} // namespace

LightRig::LightRig(juce::dsp::ConvolutionMessageQueue& queue, Voice voice)
    : v(voice), reverb(juce::dsp::Convolution::NonUniform { 256 }, queue) {}

void LightRig::prepare(double newSampleRate, int maxBlockSize) {
    sampleRate = newSampleRate;
    preparedRate.store(sampleRate);
    const size_t block = static_cast<size_t>(std::max(1, maxBlockSize));
    dcBlock.set(K::HighPass, v.dcHz, 0.7, 0.0, sampleRate);
    applyFilters();
    gate.prepare(sampleRate);
    limiter.prepare(sampleRate);
    comp.prepare(sampleRate, maxBlockSize);
    comp.setKnee(8.0f);
    comp.setDetectionMode(Compressor::DetectionMode::Peak);
    applyComp();
    for (auto* s : { &trim, &delaySamples, &delayFb, &delayWet, &reverbWet, &outputTrim }) s->prepare(sampleRate);
    reverb.prepare({ sampleRate, static_cast<juce::uint32>(block), 1 });
    dryBuf.assign(block, 0.0f);
    wetBuf.assign(block, 0.0f);
    delayLine.assign(static_cast<size_t>(sampleRate * MaxDelaySec) + 4, 0.0f);
    reverbDirty.store(true);
    reset();
}

void LightRig::reset() {
    if (preparedRate.load() <= 0.0) return;
    for (auto* f : { &dcBlock, &eqLow, &eqMid, &eqHigh }) f->reset();
    gate.reset();
    limiter.reset();
    comp.reset();
    reverb.reset();
    std::fill(delayLine.begin(), delayLine.end(), 0.0f);
    delayWrite = 0;
    applyGains();
    for (auto* s : { &trim, &delaySamples, &delayFb, &delayWet, &reverbWet, &outputTrim }) s->jump(s->goal());
}

void LightRig::applyFilters() {
    eqLow.set(K::LowShelf, v.lowHz, 0.0, lowDb, sampleRate);
    eqMid.set(K::Peaking, v.midHz, v.midQ, midDb, sampleRate);
    eqHigh.set(K::HighShelf, v.highHz, 0.0, highDb, sampleRate);
}

void LightRig::applyComp() {
    comp.setEnabled(compEnabled);
    comp.setThreshold(compThresholdDb);
    comp.setRatio(compRatio);
    // The web writes 10 ms / 150 ms while the compressor is off.
    comp.setAttack((compEnabled ? compAttackSec : 0.01f) * 1000.0f);
    comp.setRelease((compEnabled ? compReleaseSec : 0.15f) * 1000.0f);
    comp.setMakeup(webMakeupDb(compEnabled, compThresholdDb, compRatio));
}

void LightRig::applyGains() {
    trim.set(rack::fromDb(inputDb));
    delaySamples.set(static_cast<float>((delayEnabled ? delayTimeMs / 1000.0 : 0.0) * sampleRate));
    delayFb.set(delayEnabled ? delayFeedback : 0.0f);
    delayWet.set(delayEnabled ? delayMix : 0.0f);
    reverbWet.set(reverbEnabled && reverbReady() ? reverbMix : 0.0f);
    outputTrim.set(std::pow(10.0f, outputDb / 20.0f));
}

bool LightRig::setParam(std::string_view name, float x) {
    const bool on = x >= 0.5f;
    if (name == "enabled") { enabled = on; wantReverb.store(enabled && reverbEnabled); return true; }
    if (name == "input") { inputDb = std::clamp(x, -12.0f, 12.0f); applyGains(); return true; }
    if (name == "gate/enabled") { gateEnabled = on; return true; }
    if (name == "gate/threshold") { gateThresholdDb = std::clamp(x, -80.0f, -20.0f); return true; }
    if (name == "comp/enabled") { compEnabled = on; applyComp(); return true; }
    if (name == "comp/threshold") { compThresholdDb = std::clamp(x, -48.0f, 0.0f); applyComp(); return true; }
    if (name == "comp/ratio") { compRatio = std::clamp(x, 1.0f, 12.0f); applyComp(); return true; }
    if (name == "comp/attack") { compAttackSec = std::clamp(x, 0.001f, 0.1f); applyComp(); return true; }   // seconds
    if (name == "comp/release") { compReleaseSec = std::clamp(x, 0.01f, 1.0f); applyComp(); return true; }   // seconds
    if (name == "eq/low") { lowDb = std::clamp(x, -12.0f, 12.0f); applyFilters(); return true; }
    if (name == "eq/mid") { midDb = std::clamp(x, -12.0f, 12.0f); applyFilters(); return true; }
    if (name == "eq/high") { highDb = std::clamp(x, -12.0f, 12.0f); applyFilters(); return true; }
    if (name == "delay/enabled") { delayEnabled = on; applyGains(); return true; }
    if (name == "delay/time") { delayTimeMs = std::clamp(x, 50.0f, 1500.0f); applyGains(); return true; }
    if (name == "delay/feedback") { delayFeedback = std::clamp(x, 0.0f, 0.95f); applyGains(); return true; }
    if (name == "delay/mix") { delayMix = std::clamp(x, 0.0f, 1.0f); applyGains(); return true; }
    if (name == "reverb/enabled") { reverbEnabled = on; wantReverb.store(enabled && reverbEnabled); applyGains(); return true; }
    if (name == "reverb/size") {
        const float next = std::clamp(x, 0.3f, 5.0f);
        if (std::abs(next - reverbSizeSec.load()) > 0.05f) { reverbSizeSec.store(next); reverbDirty.store(true); }
        return true;
    }
    if (name == "reverb/mix") { reverbMix = std::clamp(x, 0.0f, 1.0f); applyGains(); return true; }
    if (name == "output") { outputDb = std::clamp(x, -24.0f, 24.0f); applyGains(); return true; }
    if (name == "limiter/enabled") { limiterEnabled = on; return true; }
    if (name == "limiter/ceiling") { ceilingDb = std::clamp(x, -12.0f, 0.0f); return true; }
    return false;
}

void LightRig::serviceBackground() {
    const double fs = preparedRate.load();
    if (fs <= 0.0) return;
    if (!(wantReverb.load() && reverbDirty.exchange(false))) return;
    auto tail = GuitarAmp::roomImpulse(fs, reverbSizeSec.load(), 0x1167u);
    const float scale = GuitarAmp::convolverNormalisation(tail, fs);
    juce::AudioBuffer<float> buffer(1, static_cast<int>(tail.size()));
    for (size_t i = 0; i < tail.size(); ++i) buffer.setSample(0, static_cast<int>(i), tail[i] * scale);
    reverb.loadImpulseResponse(std::move(buffer), fs, juce::dsp::Convolution::Stereo::no,
                               juce::dsp::Convolution::Trim::no, juce::dsp::Convolution::Normalise::no);
}

void LightRig::process(float* x, int n) {
    if (!enabled || n <= 0 || dryBuf.empty()) return;
    const int chunk = static_cast<int>(dryBuf.size());
    for (int at = 0; at < n; at += chunk) processChunk(x + at, std::min(chunk, n - at));
}

void LightRig::processChunk(float* x, int n) {
    if (reverbEnabled && reverbReady() && reverbWet.goal() != reverbMix) reverbWet.set(reverbMix);
    for (int i = 0; i < n; ++i) {
        float s = dcBlock.process(x[i]) * trim.next();
        s = gate.process(s, gateEnabled, gateThresholdDb);
        s = comp.processSample(s);
        dryBuf[static_cast<size_t>(i)] = eqHigh.process(eqMid.process(eqLow.process(s)));
    }
    const bool reverbOn = reverbEnabled;
    if (reverbOn) {
        std::copy(dryBuf.begin(), dryBuf.begin() + n, wetBuf.begin());
        float* channels[] = { wetBuf.data() };
        juce::dsp::AudioBlock<float> block(channels, 1, static_cast<size_t>(n));
        reverb.process(juce::dsp::ProcessContextReplacing<float>(block));
    }
    const int ring = static_cast<int>(delayLine.size());
    const float ceiling = rack::fromDb(ceilingDb);
    for (int i = 0; i < n; ++i) {
        const size_t k = static_cast<size_t>(i);
        const float dry = dryBuf[k];
        // A DelayNode in a cycle cannot be shorter than one render quantum in Web Audio.
        const float d = std::clamp(delaySamples.next(), 128.0f, static_cast<float>(ring - 2));
        float readPos = static_cast<float>(delayWrite) - d;
        while (readPos < 0.0f) readPos += static_cast<float>(ring);
        const int r0 = static_cast<int>(readPos) % ring;
        const int r1 = (r0 + 1) % ring;
        const float frac = readPos - std::floor(readPos);
        const float tapped = delayLine[static_cast<size_t>(r0)] * (1.0f - frac) + delayLine[static_cast<size_t>(r1)] * frac;
        delayLine[static_cast<size_t>(delayWrite)] = dry + tapped * delayFb.next();
        delayWrite = (delayWrite + 1) % ring;
        float mix = dry + tapped * delayWet.next();
        if (reverbOn) mix += wetBuf[k] * reverbWet.next();
        x[i] = limiter.process(mix * outputTrim.next(), limiterEnabled, ceiling) * rack::CentreToMono;
    }
}

} // namespace dsp
