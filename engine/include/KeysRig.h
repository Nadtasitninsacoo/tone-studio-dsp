#pragma once

#include <juce_dsp/juce_dsp.h>

#include <atomic>
#include <string_view>
#include <vector>

#include "Compressor.h"
#include "RackKit.h"

namespace dsp {

/**
 * The web app's keys rack (`createKeysChain` in guitar-recorder-web `src/lib/keysFx.ts`), as a
 * per-channel insert:
 *
 *   DC block 20 Hz → input trim → gate → chorus → compressor (knee 10, 15 / 200 ms)
 *     → low 180 Hz shelf · mid 1.2 kHz peak · high 10 kHz shelf → dry + reverb → output trim → limiter
 *
 * **The chorus is the web's, and it is mono there too.** Two delay lines at 20 ms swept by one sine in
 * opposite directions (±DEPTH), summed into one gain node — so on the web's dual-mono signal both sides
 * carry the same sum, and here one channel carries it with nothing lost. Dry is 1 − MIX/2, wet is MIX.
 * The sweep reads with linear interpolation, which is what a modulated `DelayNode` does in Chromium.
 */
class KeysRig {
public:
    explicit KeysRig(juce::dsp::ConvolutionMessageQueue& queue);

    void prepare(double sampleRate, int maxBlockSize);
    void reset();
    /** Audio thread. `name` is the part after `keys/`. */
    bool setParam(std::string_view name, float value);
    void process(float* samples, int numSamples);
    void serviceBackground();

    bool isEnabled() const { return enabled; }
    bool reverbReady() const { return reverb.getCurrentIRSize() > 1; }

    static constexpr float LowHz = 180.0f, MidHz = 1200.0f, HighHz = 10000.0f; // `KEYS_EQ_HZ`
    static constexpr double ChorusBaseSec = 0.02;

private:
    bool enabled { false };
    float inputDb { 0.0f };
    bool gateEnabled { false };
    float gateThresholdDb { -60.0f };
    bool chorusEnabled { true };
    float chorusRateHz { 0.8f }, chorusDepthMs { 2.5f }, chorusMix { 0.25f };
    bool compEnabled { true };
    float compThresholdDb { -16.0f }, compRatio { 2.5f };
    float lowDb { 0.0f }, midDb { 0.0f }, highDb { 1.5f };
    bool reverbEnabled { true };
    float reverbMix { 0.15f };
    float outputDb { 0.0f };
    bool limiterEnabled { true };
    float ceilingDb { -0.3f };

    std::atomic<float> reverbSizeSec { 2.0f };
    std::atomic<double> preparedRate { 0.0 };
    std::atomic<bool> reverbDirty { true };
    std::atomic<bool> wantReverb { false };

    double sampleRate { 48000.0 };
    rack::WebBiquad dcBlock, eqLow, eqMid, eqHigh;
    rack::Gate gate;
    rack::Limiter limiter;
    Compressor comp;
    rack::Smoothed trim, depth, dryGain, wetGain, reverbWet, outputTrim;
    double lfoPhase { 0.0 };
    juce::dsp::Convolution reverb;
    std::vector<float> dryBuf, wetBuf, chorusLine;
    int chorusWrite { 0 };

    void applyFilters();
    void applyGains();
    void applyComp();
    void processChunk(float* samples, int numSamples);
    float tap(float delaySamples) const;
};

} // namespace dsp
