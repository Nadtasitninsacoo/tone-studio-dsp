#pragma once

#include <juce_dsp/juce_dsp.h>

#include <atomic>
#include <string_view>
#include <vector>

#include "Compressor.h"
#include "RackKit.h"

namespace dsp {

/**
 * Four of the web app's racks — brass, phin, khaen, saw (`createBrassChain`, `createPhinChain`,
 * `createKhaenChain`, `createSawChain`) — are one chain with different numbers, so they are one class
 * here, built from a `Voice`:
 *
 *   DC block → input trim → gate → compressor (knee 8) → low shelf · mid peak · high shelf
 *     → dry + delay (no damping, up to 1.5 s) + reverb (normalised) → output trim → limiter
 *
 * The numbers that differ are the DC blocker's corner and the three EQ frequencies and the mid's Q; every
 * range is the same in all four tables (`BRASS_RANGES` … `SAW_RANGES`). One tail where the web has two,
 * × √2 for the channel's panner, as the other racks.
 */
class LightRig {
public:
    struct Voice {
        double dcHz, lowHz, midHz, midQ, highHz;
    };
    /** `BRASS_EQ_HZ`, `PHIN_EQ_HZ`, `KHAEN_EQ_HZ`, `SAW_EQ_HZ` and each chain's DC corner and mid Q. */
    static constexpr Voice Brass { 25.0, 350.0, 3200.0, 0.85, 6500.0 };
    static constexpr Voice Phin { 40.0, 220.0, 1250.0, 0.9, 7000.0 };
    static constexpr Voice Khaen { 40.0, 260.0, 2000.0, 0.9, 6000.0 };
    static constexpr Voice Saw { 40.0, 300.0, 1000.0, 0.9, 5000.0 };

    LightRig(juce::dsp::ConvolutionMessageQueue& queue, Voice voice);

    void prepare(double sampleRate, int maxBlockSize);
    void reset();
    /** Audio thread. `name` is the part after the rack's prefix. False for an unknown name. */
    bool setParam(std::string_view name, float value);
    void process(float* samples, int numSamples);
    void serviceBackground();

    bool isEnabled() const { return enabled; }
    bool reverbReady() const { return reverb.getCurrentIRSize() > 1; }
    const Voice& voice() const { return v; }

private:
    Voice v;
    bool enabled { false };
    float inputDb { 0.0f };
    bool gateEnabled { true };
    float gateThresholdDb { -60.0f };
    bool compEnabled { true };
    float compThresholdDb { -18.0f }, compRatio { 3.0f }, compAttackSec { 0.01f }, compReleaseSec { 0.18f };
    float lowDb { 0.0f }, midDb { 0.0f }, highDb { 0.0f };
    bool delayEnabled { false };
    float delayTimeMs { 250.0f }, delayFeedback { 0.25f }, delayMix { 0.12f };
    bool reverbEnabled { true };
    float reverbMix { 0.15f };
    float outputDb { 0.0f };
    bool limiterEnabled { true };
    float ceilingDb { -0.3f };

    std::atomic<float> reverbSizeSec { 1.8f };
    std::atomic<double> preparedRate { 0.0 };
    std::atomic<bool> reverbDirty { true };
    std::atomic<bool> wantReverb { false };

    double sampleRate { 48000.0 };
    rack::WebBiquad dcBlock, eqLow, eqMid, eqHigh;
    rack::Gate gate;
    rack::Limiter limiter;
    Compressor comp;
    rack::Smoothed trim, delaySamples, delayFb, delayWet, reverbWet, outputTrim;
    juce::dsp::Convolution reverb;
    std::vector<float> dryBuf, wetBuf, delayLine;
    int delayWrite { 0 };

    void applyFilters();
    void applyGains();
    void applyComp();
    void processChunk(float* samples, int numSamples);
};

} // namespace dsp
