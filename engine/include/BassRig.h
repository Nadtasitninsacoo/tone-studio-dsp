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
 * The web app's bass rig (`createBassChain` in guitar-recorder-web `src/lib/bassFx.ts`), ported
 * into the engine as a per-channel insert. Same order and the same numbers as the web chain:
 *
 *   DC block 18 Hz → input trim → gate → compressor (knee 12) → split
 *     ├─ LR4 low-pass at the crossover → LOW gain ─┬──────────────────────────┐
 *     │    └─ power: tube curve (0.85, 24, 0.35) 2× → LR4 high-pass → POWER ──┤
 *     └─ LR4 high-pass at the crossover ─┬─ drive (hardness 9) 2× → makeup → 30 Hz ─┤→ ampSum
 *                                        │                                            │
 *   ampSum → sub 60 · low-mid 250 · mid 800 · high 2500 → cabinet → cos(di·π/2) ─┐   │
 *   LOW gain + high band → three low-passes at the cabinet's corner → sin(di·π/2) ┴→ output
 *   trim → limiter
 *
 * **Mono, where the web rack is dual mono.** The web rack's gate worklet turns the mono input
 * into two identical channels and everything after it is the same on both sides, so one channel
 * here is exactly one side there; the output is scaled by √2 for the channel's panner
 * (`rack::CentreToMono`).
 *
 * **What is not sample-identical, said rather than hidden.** The two waveshapers run at 2× in
 * both, but through different resamplers: Chromium's is a fixed windowed sinc and this is JUCE's
 * equiripple half-band, so the drive and power paths are delayed by a slightly different amount
 * relative to the clean low band. Neither side compensates. Chromium's `DynamicsCompressorNode`
 * also delays its whole output by 6 ms of look-ahead and the engine's compressor does not; that
 * is the same delay on every path, so it changes latency and not tone. The compressor itself is
 * the engine's, with the web node's knee, timing and makeup — the same trade `GuitarAmp` makes.
 *
 * **Threads.** As `GuitarAmp`: `setParam` and `process` on the audio thread, allocation-free;
 * the cabinet impulse is built in `serviceBackground` on the message thread.
 */
class BassRig {
public:
    explicit BassRig(juce::dsp::ConvolutionMessageQueue& queue);

    void prepare(double sampleRate, int maxBlockSize);
    void reset();

    /** Audio thread. `name` is the part after `bass/`. False for an unknown name. */
    bool setParam(std::string_view name, float value);

    /** Audio thread. In place, mono. A no-op while the rack is switched off. */
    void process(float* samples, int numSamples);

    /** Message thread. Builds and loads the cabinet impulse when a setting has made it stale. */
    void serviceBackground();

    bool isEnabled() const { return enabled; }
    bool cabinetReady() const { return cabinet.getCurrentIRSize() > 1; }

    //==========================================================================================
    // The fixed numbers, public so the tests can hold them against bassFx.ts.

    static constexpr double StageHardness = 9.0;     // `STAGE_HARDNESS`
    static constexpr double DriveTargetRms = 0.3256; // `DRIVE_TARGET_RMS`
    static constexpr double PowerAmount = 0.85;      // `POWER_AMOUNT`
    static constexpr double PowerHardness = 24.0;    // `POWER_HARDNESS`
    static constexpr double PowerBias = 0.35;        // `POWER_BIAS`
    static constexpr double CrossoverQDb = 0.70710678118654752; // `Math.SQRT1_2`, in dB as Web Audio reads it
    static constexpr float SubHz = 60.0f, LowMidHz = 250.0f, MidHz = 800.0f, HighHz = 2500.0f; // `BASS_EQ_HZ`

    /** The bass cabinets, in `cabinetsFor('bass')` order — the index the web sends. */
    static constexpr int CabinetCount = 2;
    static int cabinetTableIndex(int bassCabinet) { return bassCabinet <= 0 ? 4 : 5; } // b15, b410

    /** `bassPowerGain`: dB-linear from −15 dB at the bottom of the travel to 0.7 at the top, exactly 0 at 0. */
    static float powerGain(float power);

private:
    // ---- settings, in wire units (`DEFAULT_BASS`) --------------------------------------------
    bool enabled { false };
    float inputDb { 0.0f };
    bool gateEnabled { true };
    float gateThresholdDb { -62.0f };
    bool compEnabled { true };
    float compThresholdDb { -18.0f };
    float compRatio { 3.0f };
    float compAttackMs { 12.0f }, compReleaseMs { 280.0f };
    float crossoverHz { 160.0f };
    float lowDb { 4.0f };
    float power { 0.2f };
    bool driveEnabled { false };
    float driveAmount { 0.25f }, driveBias { 0.2f };
    float subDb { 4.5f }, lowMidDb { -1.5f }, midDb { 1.0f }, highDb { 1.0f };
    bool cabEnabled { true };
    float diMix { 0.25f };
    float outputDb { 0.0f };
    bool limiterEnabled { true };
    float ceilingDb { -0.3f };

    // Read by the message thread when it builds the impulse.
    std::atomic<int> cabModel { 1 }; // b410, `DEFAULT_BASS_CABINET`
    std::atomic<float> cabPresenceDb { 0.0f };
    std::atomic<float> cabResonanceDb { 0.0f };
    std::atomic<double> preparedRate { 0.0 };
    std::atomic<bool> cabDirty { true };
    std::atomic<bool> wantCab { false };

    // ---- DSP state ---------------------------------------------------------------------------
    double sampleRate { 48000.0 };
    rack::WebBiquad dcBlock, driveDc;
    rack::WebBiquad lowBand[2], highBand[2], powerHp[2];
    rack::WebBiquad eqSub, eqLowMid, eqMid, eqHigh;
    rack::WebBiquad diTop[rack::MaxCabinetLowpassStages];
    rack::Gate gate;
    rack::Limiter limiter;
    Compressor comp;
    rack::Smoothed trim, lowGain, powerGainS, driveTrim, ampOut, diGain, outputTrim;

    rack::Curve powerCurve {};
    rack::Curve driveCurve {};
    bool driveActive { false };
    float makeup { 1.0f };
    bool driveStale { true };
    std::unique_ptr<juce::dsp::Oversampling<float>> powerOs, driveOs;

    juce::dsp::Convolution cabinet;
    std::vector<float> lowBuf, highBuf, powerBuf, driveBuf, ampBuf;

    void applyComp();
    void applyFilters();
    void applyGains();
    void rebuildDrive();
    void processChunk(float* samples, int numSamples);
};

} // namespace dsp
