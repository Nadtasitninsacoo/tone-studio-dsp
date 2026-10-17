#pragma once

#include <juce_dsp/juce_dsp.h>

#include <atomic>
#include <memory>
#include <string_view>
#include <vector>

#include "Compressor.h"
#include "RackKit.h"

namespace dsp {

/**
 * The web app's vocal rack (`createVocalChain` in guitar-recorder-web `src/lib/vocalFx.ts`), ported
 * into the engine as a per-channel insert. Same order and the same numbers as the web chain:
 *
 *   DC block 30 Hz → input trim → gate → low cut (100 Hz, or 20 Hz when off)
 *     → de-esser: low-pass 5 kHz ─────────────────────────┐
 *                 high-pass 5 kHz → compressor (knee 4) ──┴→ compressor (knee 8)
 *     → body 200 Hz shelf · presence 2.5 kHz peak · air 12 kHz shelf
 *     → dry + delay (2.5 kHz low-pass in the loop) + reverb (normalised) → output trim → limiter
 *
 * The de-esser's split is Linkwitz-Riley — two Butterworth sections per band, summing flat. It was one
 * low-pass and one high-pass at Web Audio's default Q, which at one corner sum to a null: the web rack had a
 * hole at 5 kHz, fixed on both sides together. Each `DynamicsCompressorNode` adds its own makeup — on the
 * de-esser's band too — and that is copied, because it is what the web rack does. Mono with one reverb
 * tail where the web has two, × √2 for the channel's panner; see `DrumRig` for why.
 */
class VocalRig {
public:
    explicit VocalRig(juce::dsp::ConvolutionMessageQueue& queue);

    void prepare(double sampleRate, int maxBlockSize);
    void reset();

    /** Audio thread. `name` is the part after `vocals/`. False for an unknown name. */
    bool setParam(std::string_view name, float value);

    /** Audio thread. In place, mono. A no-op while the rack is switched off. */
    void process(float* samples, int numSamples);

    /** Message thread. Builds and loads the reverb tail when its length has changed. */
    void serviceBackground();

    bool isEnabled() const { return enabled; }
    bool reverbReady() const { return reverb.getCurrentIRSize() > 1; }

    static constexpr float LowCutHz = 100.0f, BodyHz = 200.0f, PresenceHz = 2500.0f, AirHz = 12000.0f,
                           DeEssSplitHz = 5000.0f; // `VOCAL_EQ_HZ`
    static constexpr double DefaultQDb = 1.0;  // a `BiquadFilterNode` whose Q nobody set (the delay's damping)
    /** `DE_ESS_Q_DB`: a Butterworth section (1/√2) in dB — the split is Linkwitz-Riley, two per band. */
    static constexpr double DeEssQDb = -3.0102999566398120;
    static constexpr double LowCutQDb = 0.707; // `lowCut.Q`
    static constexpr float DelayDampHz = 2500.0f;

private:
    // ---- settings, in wire units (`DEFAULT_VOCALS`) ------------------------------------------
    bool enabled { false };
    float inputDb { 0.0f };
    bool gateEnabled { true };
    float gateThresholdDb { -60.0f };
    bool deEssEnabled { true };
    float deEssThresholdDb { -32.0f }, deEssRatio { 4.5f };
    bool compEnabled { true };
    float compThresholdDb { -18.0f }, compRatio { 3.5f }, compAttackSec { 0.015f }, compReleaseSec { 0.2f };
    bool lowCutEnabled { true };
    float bodyDb { 1.0f }, presenceDb { 2.0f }, airDb { 3.0f };
    bool delayEnabled { false };
    float delayTimeMs { 400.0f }, delayFeedback { 0.35f }, delayMix { 0.2f };
    bool reverbEnabled { true };
    float reverbMix { 0.18f };
    float outputDb { 0.0f };
    bool limiterEnabled { true };
    float ceilingDb { -0.3f };

    std::atomic<float> reverbSizeSec { 1.8f };
    std::atomic<double> preparedRate { 0.0 };
    std::atomic<bool> reverbDirty { true };
    std::atomic<bool> wantReverb { false };

    // ---- DSP state ---------------------------------------------------------------------------
    double sampleRate { 48000.0 };
    rack::WebBiquad dcBlock, lowCut, dsLow[2], dsHigh[2], body, presence, air, delayDamp;
    rack::Gate gate;
    rack::Limiter limiter;
    Compressor deEss, comp;
    rack::Smoothed trim, delaySamples, delayFb, delayWet, reverbWet, outputTrim;

    juce::dsp::Convolution reverb;
    std::vector<float> dryBuf, wetBuf, delayLine;
    int delayWrite { 0 };

    void applyFilters();
    void applyGains();
    void applyCompressors();
    void processChunk(float* samples, int numSamples);
};

} // namespace dsp
