#include "LowEnd.h"

#include <algorithm>
#include <cmath>

namespace dsp {

void LowEndBiquad::set(Kind kind, double sampleRate, double hz, double q) {
    const double w = 2.0 * 3.14159265358979323846 * hz / sampleRate;
    const double cs = std::cos(w);
    const double alpha = std::sin(w) / (2.0 * q);
    const double a0 = 1.0 + alpha;
    if (kind == Kind::LowPass) {
        b0 = (1.0 - cs) / 2.0 / a0;
        b1 = (1.0 - cs) / a0;
        b2 = b0;
    } else {
        b0 = (1.0 + cs) / 2.0 / a0;
        b1 = -(1.0 + cs) / a0;
        b2 = b0;
    }
    a1 = -2.0 * cs / a0;
    a2 = (1.0 - alpha) / a0;
}

// ---------------------------------------------------------------- sidechain ---

void SidechainDuck::prepare(double newSampleRate, int maxBlockSize) {
    sampleRate = newSampleRate;
    keyFilter.set(LowEndBiquad::Kind::LowPass, sampleRate, KeyLowpassHz, 0.70710678);
    envelope.set(LowEndBiquad::Kind::LowPass, sampleRate, EnvelopeHz, 0.70710678);
    gains.assign(static_cast<size_t>(std::max(1, maxBlockSize)), 1.0f);
    reset();
}

void SidechainDuck::reset() {
    keyFilter.reset();
    envelope.reset();
    std::fill(gains.begin(), gains.end(), 1.0f);
}

void SidechainDuck::setDepthDb(float db) {
    depthDb = std::isfinite(db) ? std::clamp(db, 0.0f, MaxDepthDb) : 0.0f;
}

float SidechainDuck::duckGain(float env, float db) {
    const float depth = std::isfinite(db) ? std::clamp(db, 0.0f, MaxDepthDb) : 0.0f;
    const float fraction = 1.0f - std::pow(10.0f, -depth / 20.0f);
    const float reduction = std::clamp(env / FullHitEnvelope, 0.0f, 1.0f);
    return 1.0f - fraction * reduction;
}

void SidechainDuck::processKey(const float* key, int numSamples) {
    const int n = std::min(numSamples, static_cast<int>(gains.size()));
    const float fraction = 1.0f - std::pow(10.0f, -depthDb / 20.0f);
    for (int i = 0; i < n; ++i) {
        const float rectified = std::abs(keyFilter.process(key[i]));
        const float env = envelope.process(rectified);
        const float reduction = std::clamp(env / FullHitEnvelope, 0.0f, 1.0f);
        gains[static_cast<size_t>(i)] = 1.0f - fraction * reduction;
    }
}

void SidechainDuck::processSilence(int numSamples) {
    const int n = std::min(numSamples, static_cast<int>(gains.size()));
    // The envelope still has to decay, or a key channel that stops mid-hit leaves the bass
    // ducked until it plays again.
    const float fraction = 1.0f - std::pow(10.0f, -depthDb / 20.0f);
    for (int i = 0; i < n; ++i) {
        const float env = envelope.process(std::abs(keyFilter.process(0.0f)));
        const float reduction = std::clamp(env / FullHitEnvelope, 0.0f, 1.0f);
        gains[static_cast<size_t>(i)] = 1.0f - fraction * reduction;
    }
}

// ---------------------------------------------------------------- mono bass ---

void MonoBass::prepare(double newSampleRate, int) {
    sampleRate = newSampleRate;
    setFrequency(frequencyHz);
    reset();
}

void MonoBass::reset() {
    sideA.reset();
    sideB.reset();
}

void MonoBass::setFrequency(float hz) {
    frequencyHz = std::isfinite(hz) ? std::clamp(hz, MinHz, MaxHz) : 120.0f;
    // Two Butterworth sections: a Linkwitz-Riley high-pass, −6 dB at the corner.
    sideA.set(LowEndBiquad::Kind::HighPass, sampleRate, frequencyHz, 0.70710678);
    sideB.set(LowEndBiquad::Kind::HighPass, sampleRate, frequencyHz, 0.70710678);
}

void MonoBass::process(float* left, float* right, int numSamples) {
    if (!enabled) return;
    for (int i = 0; i < numSamples; ++i) {
        const float mid = 0.5f * (left[i] + right[i]);
        const float side = sideB.process(sideA.process(0.5f * (left[i] - right[i])));
        left[i] = mid + side;
        right[i] = mid - side;
    }
}

} // namespace dsp
