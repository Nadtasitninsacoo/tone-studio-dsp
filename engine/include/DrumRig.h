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
 * The web app's drum bus (`createDrumChain` in guitar-recorder-web `src/lib/drumFx.ts`), ported
 * into the engine as a per-channel insert. Same order and the same numbers as the web chain:
 *
 *   DC block 20 Hz → input trim → gate → sub cut (2 × high-pass at min(subCut, kick))
 *     ├─ kick peak (Q from TIGHT) → box 400 Hz → snap 3.8 kHz shelf → drive (tanh, 2×) → makeup → split
 *     │      split → dry ─────────────────────────────────────────────────┐
 *     │      split → crush (knee 3, 2 / 120 ms) → PUNCH ───────────────────┤
 *     │      split → room (convolution, normalised) → MIX ─────────────────┤→ glue (knee 8,
 *     └─ depth: band-pass Q 7 at the kick → 0.8 × DEPTH ──────────────────┘  20 / 220 ms) → output → limiter
 *
 * **Mono, where the web rack is dual mono** — except the room, whose two web impulses are two
 * different random tails. Here there is one tail at the same normalised level, so the room has the
 * web's level and decay and not its width. Output × √2 for the channel's panner, as `BassRig`.
 *
 * The two compressors are the engine's `Compressor` with each web node's knee, timing and the
 * `DynamicsCompressorNode` makeup — which on the crush is large (−32 dB at 10:1 is +17 dB), and is
 * what makes PUNCH a punch rather than a quiet copy. The drive's 2× resampler is JUCE's and not
 * Chromium's, the same caveat as the bass.
 */
class DrumRig {
public:
    explicit DrumRig(juce::dsp::ConvolutionMessageQueue& queue);

    void prepare(double sampleRate, int maxBlockSize);
    void reset();

    /** Audio thread. `name` is the part after `drums/`. False for an unknown name. */
    bool setParam(std::string_view name, float value);

    /** Audio thread. In place, mono. A no-op while the rack is switched off. */
    void process(float* samples, int numSamples);

    /** Message thread. Builds and loads the room's tail when its length has changed. */
    void serviceBackground();

    bool isEnabled() const { return enabled; }
    bool roomReady() const { return room.getCurrentIRSize() > 1; }

    static constexpr double DriveHardness = 18.0;     // `DRIVE_HARDNESS`
    static constexpr double DriveTargetRms = 0.4691;  // `DRIVE_TARGET_RMS`
    static constexpr double SubCutQDb = 0.70710678118654752; // `Math.SQRT1_2`, read in dB
    static constexpr float BoxHz = 400.0f, SnapHz = 3800.0f; // `DRUM_EQ_HZ`
    static constexpr double DepthQ = 7.0;             // `DEPTH_Q`

    /** `kickQ`: 0.7 wide to 4.5 tight. */
    static double kickQ(float tight);
    /** `depthGain`: 0…0.8. */
    static float depthGain(float depth);

private:
    // ---- settings, in wire units (`DEFAULT_DRUMS`) -------------------------------------------
    bool enabled { false };
    float inputDb { 0.0f };
    bool gateEnabled { false };
    float gateThresholdDb { -54.0f };
    float kickHz { 55.0f }, kickDb { 6.0f }, kickTight { 0.45f }, subCutHz { 30.0f }, kickDepth { 0.4f };
    float boxDb { -2.0f }, snapDb { 2.0f };
    bool driveEnabled { true };
    float driveAmount { 0.15f };
    bool crushEnabled { true };
    float crushThresholdDb { -32.0f }, crushRatio { 10.0f };
    float punch { 0.3f };
    bool roomEnabled { true };
    float roomMix { 0.14f };
    bool glueEnabled { true };
    float glueThresholdDb { -14.0f }, glueRatio { 2.0f };
    float outputDb { 0.0f };
    bool limiterEnabled { true };
    float ceilingDb { -0.3f };

    std::atomic<float> roomSizeSec { 0.8f };
    std::atomic<double> preparedRate { 0.0 };
    std::atomic<bool> roomDirty { true };
    std::atomic<bool> wantRoom { false };

    // ---- DSP state ---------------------------------------------------------------------------
    double sampleRate { 48000.0 };
    rack::WebBiquad dcBlock, subCut[2], kick, depth, box, snap;
    rack::Gate gate;
    rack::Limiter limiter;
    Compressor crush, glue;
    rack::Smoothed trim, depthBlend, driveTrim, crushGain, roomGain, outputTrim;

    rack::Curve driveCurve {};
    bool driveActive { false };
    float makeup { 1.0f };
    bool driveStale { true };
    std::unique_ptr<juce::dsp::Oversampling<float>> driveOs;

    juce::dsp::Convolution room;
    std::vector<float> mainBuf, depthBuf, roomBuf;

    void applyFilters();
    void applyGains();
    void applyCompressors();
    void rebuildDrive();
    void processChunk(float* samples, int numSamples);
};

} // namespace dsp
