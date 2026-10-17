#include "KeysRig.h"
#include "GuitarAmp.h"

#include <algorithm>
#include <cmath>

namespace dsp {

namespace {
using K = rack::WebBiquad::Kind;
constexpr double ChorusMaxSec = 0.1; // `createDelay(0.1)`
float webMakeupDb(bool on, float thresholdDb, float ratio) {
    return on ? 0.6f * (1.0f - 1.0f / std::max(1.0f, ratio)) * -thresholdDb : 0.0f;
}
} // namespace

KeysRig::KeysRig(juce::dsp::ConvolutionMessageQueue& queue)
    : reverb(juce::dsp::Convolution::NonUniform { 256 }, queue) {}

void KeysRig::prepare(double newSampleRate, int maxBlockSize) {
    sampleRate = newSampleRate;
    preparedRate.store(sampleRate);
    const size_t block = static_cast<size_t>(std::max(1, maxBlockSize));
    dcBlock.set(K::HighPass, 20.0, 0.7, 0.0, sampleRate);
    applyFilters();
    gate.prepare(sampleRate);
    limiter.prepare(sampleRate);
    comp.prepare(sampleRate, maxBlockSize);
    comp.setKnee(10.0f);
    comp.setAttack(15.0f);
    comp.setRelease(200.0f);
    comp.setDetectionMode(Compressor::DetectionMode::Peak);
    applyComp();
    for (auto* s : { &trim, &depth, &dryGain, &wetGain, &reverbWet, &outputTrim }) s->prepare(sampleRate);
    reverb.prepare({ sampleRate, static_cast<juce::uint32>(block), 1 });
    dryBuf.assign(block, 0.0f);
    wetBuf.assign(block, 0.0f);
    chorusLine.assign(static_cast<size_t>(sampleRate * ChorusMaxSec) + 4, 0.0f);
    reverbDirty.store(true);
    reset();
}

void KeysRig::reset() {
    if (preparedRate.load() <= 0.0) return;
    for (auto* f : { &dcBlock, &eqLow, &eqMid, &eqHigh }) f->reset();
    gate.reset();
    limiter.reset();
    comp.reset();
    reverb.reset();
    std::fill(chorusLine.begin(), chorusLine.end(), 0.0f);
    chorusWrite = 0;
    lfoPhase = 0.0;
    applyGains();
    for (auto* s : { &trim, &depth, &dryGain, &wetGain, &reverbWet, &outputTrim }) s->jump(s->goal());
}

void KeysRig::applyFilters() {
    eqLow.set(K::LowShelf, LowHz, 0.0, lowDb, sampleRate);
    eqMid.set(K::Peaking, MidHz, 0.8, midDb, sampleRate);
    eqHigh.set(K::HighShelf, HighHz, 0.0, highDb, sampleRate);
}

void KeysRig::applyComp() {
    comp.setEnabled(compEnabled);
    comp.setThreshold(compThresholdDb);
    comp.setRatio(compRatio);
    comp.setMakeup(webMakeupDb(compEnabled, compThresholdDb, compRatio));
}

void KeysRig::applyGains() {
    trim.set(rack::fromDb(inputDb));
    depth.set(chorusEnabled ? static_cast<float>(chorusDepthMs / 1000.0 * sampleRate) : 0.0f); // in samples
    dryGain.set(chorusEnabled ? 1.0f - chorusMix * 0.5f : 1.0f);
    wetGain.set(chorusEnabled ? chorusMix : 0.0f);
    reverbWet.set(reverbEnabled && reverbReady() ? reverbMix : 0.0f);
    outputTrim.set(std::pow(10.0f, outputDb / 20.0f));
}

bool KeysRig::setParam(std::string_view name, float x) {
    const bool on = x >= 0.5f;
    if (name == "enabled") { enabled = on; wantReverb.store(enabled && reverbEnabled); return true; }
    if (name == "input") { inputDb = std::clamp(x, -12.0f, 12.0f); applyGains(); return true; }
    if (name == "gate/enabled") { gateEnabled = on; return true; }
    if (name == "gate/threshold") { gateThresholdDb = std::clamp(x, -80.0f, -20.0f); return true; }
    if (name == "chorus/enabled") { chorusEnabled = on; applyGains(); return true; }
    if (name == "chorus/rate") { chorusRateHz = std::clamp(x, 0.1f, 5.0f); return true; }
    if (name == "chorus/depth") { chorusDepthMs = std::clamp(x, 0.1f, 8.0f); applyGains(); return true; }
    if (name == "chorus/mix") { chorusMix = std::clamp(x, 0.0f, 1.0f); applyGains(); return true; }
    if (name == "comp/enabled") { compEnabled = on; applyComp(); return true; }
    if (name == "comp/threshold") { compThresholdDb = std::clamp(x, -48.0f, 0.0f); applyComp(); return true; }
    if (name == "comp/ratio") { compRatio = std::clamp(x, 1.0f, 12.0f); applyComp(); return true; }
    if (name == "eq/low") { lowDb = std::clamp(x, -12.0f, 12.0f); applyFilters(); return true; }
    if (name == "eq/mid") { midDb = std::clamp(x, -12.0f, 12.0f); applyFilters(); return true; }
    if (name == "eq/high") { highDb = std::clamp(x, -12.0f, 12.0f); applyFilters(); return true; }
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

void KeysRig::serviceBackground() {
    const double fs = preparedRate.load();
    if (fs <= 0.0) return;
    if (!(wantReverb.load() && reverbDirty.exchange(false))) return;
    auto tail = GuitarAmp::roomImpulse(fs, reverbSizeSec.load(), 0x4E15u);
    const float scale = GuitarAmp::convolverNormalisation(tail, fs);
    juce::AudioBuffer<float> buffer(1, static_cast<int>(tail.size()));
    for (size_t i = 0; i < tail.size(); ++i) buffer.setSample(0, static_cast<int>(i), tail[i] * scale);
    reverb.loadImpulseResponse(std::move(buffer), fs, juce::dsp::Convolution::Stereo::no,
                               juce::dsp::Convolution::Trim::no, juce::dsp::Convolution::Normalise::no);
}

float KeysRig::tap(float d) const {
    const int ring = static_cast<int>(chorusLine.size());
    float readPos = static_cast<float>(chorusWrite) - d;
    while (readPos < 0.0f) readPos += static_cast<float>(ring);
    const int r0 = static_cast<int>(readPos) % ring;
    const int r1 = (r0 + 1) % ring;
    const float frac = readPos - std::floor(readPos);
    return chorusLine[static_cast<size_t>(r0)] * (1.0f - frac) + chorusLine[static_cast<size_t>(r1)] * frac;
}

void KeysRig::process(float* x, int n) {
    if (!enabled || n <= 0 || dryBuf.empty()) return;
    const int chunk = static_cast<int>(dryBuf.size());
    for (int at = 0; at < n; at += chunk) processChunk(x + at, std::min(chunk, n - at));
}

void KeysRig::processChunk(float* x, int n) {
    if (reverbEnabled && reverbReady() && reverbWet.goal() != reverbMix) reverbWet.set(reverbMix);
    const int ring = static_cast<int>(chorusLine.size());
    const float base = static_cast<float>(ChorusBaseSec * sampleRate);
    const double step = 2.0 * rack::Pi * chorusRateHz / sampleRate;
    for (int i = 0; i < n; ++i) {
        float s = dcBlock.process(x[i]) * trim.next();
        s = gate.process(s, gateEnabled, gateThresholdDb);

        // The chorus: write, then read two taps swept in opposite directions around 20 ms.
        chorusLine[static_cast<size_t>(chorusWrite)] = s;
        const float sweep = static_cast<float>(std::sin(lfoPhase)) * depth.next();
        lfoPhase += step;
        if (lfoPhase > 2.0 * rack::Pi) lfoPhase -= 2.0 * rack::Pi;
        const float maxD = static_cast<float>(ring - 2);
        const float wet = tap(std::clamp(base + sweep, 1.0f, maxD)) + tap(std::clamp(base - sweep, 1.0f, maxD));
        chorusWrite = (chorusWrite + 1) % ring;
        s = s * dryGain.next() + wet * wetGain.next();

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
    const float ceiling = rack::fromDb(ceilingDb);
    for (int i = 0; i < n; ++i) {
        const size_t k = static_cast<size_t>(i);
        float mix = dryBuf[k];
        if (reverbOn) mix += wetBuf[k] * reverbWet.next();
        x[i] = limiter.process(mix * outputTrim.next(), limiterEnabled, ceiling) * rack::CentreToMono;
    }
}

} // namespace dsp
