#pragma once

#include <juce_dsp/juce_dsp.h>

#include <array>
#include <atomic>
#include <string_view>
#include <vector>

#include "Compressor.h"
#include "FilterPrimitives.h"
#include "ParametricEQ.h"

namespace dsp {

/**
 * The web app's guitar rack (`createAmpChain` in guitar-recorder-web `src/lib/ampFx.ts`),
 * ported into the engine as a per-channel insert.
 *
 * Same order as the web chain, and the same numbers wherever the web chain fixes one:
 *
 *   DC block 22 Hz → input trim → gate → compressor → bass 180 Hz shelf · mid peak ·
 *   treble 2.4 kHz shelf → up to three tube stages (4× oversampled, 6.2 kHz / 30 Hz between
 *   them) → makeup → cabinet (convolution) → depth 120 Hz · presence 3.2 kHz shelves →
 *   dry + delay (3.2 kHz in the loop) + reverb (convolution) → output trim → limiter
 *
 * **Mono, where the web rack is stereo.** An engine channel is mono until its panner, so the
 * rack's two cabinet IRs are averaged and the reverb is one channel. `cab/width` is accepted
 * and does nothing here. What comes out is scaled by √2 so that, through the channel's
 * equal-power panner at centre, each side carries what the web rack's centre does.
 *
 * **Threads.** `setParam` and `process` run on the audio thread and never allocate. Building
 * an impulse response does allocate, so a change that needs one only raises a flag, and
 * `serviceBackground` — called from the message thread — builds it and hands it to a
 * `juce::dsp::Convolution`, which swaps it in without allocating on the audio thread.
 */
class GuitarAmp {
public:
    explicit GuitarAmp(juce::dsp::ConvolutionMessageQueue& queue);

    void prepare(double sampleRate, int maxBlockSize);
    void reset();

    /** Audio thread. `name` is the part after `amp/`. False for an unknown name. */
    bool setParam(std::string_view name, float value);

    /** Audio thread. In place, mono. A no-op while the rack is switched off. */
    void process(float* samples, int numSamples);

    /** Message thread. Builds and loads any impulse response a setting has made stale. */
    void serviceBackground();

    bool isEnabled() const { return enabled; }
    /** Whether a built impulse has been swapped in yet. Until then that stage is bypassed. */
    bool cabinetReady() const { return cabinet.getCurrentIRSize() > 1; }
    bool reverbReady() const { return reverb.getCurrentIRSize() > 1; }

    //==========================================================================================
    // The pure halves, public so the tests can hold them against the web app's own formulas.

    static constexpr int CurvePoints = 2048;
    static constexpr float StageHardness = 14.0f;   // `STAGE_HARDNESS` in ampFx.ts
    static constexpr float BiasScale = 2.5f;        // `BIAS_SCALE` in audioGraph.ts
    static constexpr float DriveTargetRms = 0.389f; // `DRIVE_TARGET_RMS` in ampFx.ts
    static constexpr float DriveProbePeak = 0.25f;  // `DRIVE_PROBE_PEAK` in audioGraph.ts
    static constexpr int DriveProbePoints = 1024;

    /** `tubeCurve(amount, 14, bias)`. */
    static void tubeCurve(float amount, float bias, std::array<float, CurvePoints>& out);
    /** `WaveShaperNode`'s reading of a table: clamp to ±1, then interpolate. */
    static float readCurve(const std::array<float, CurvePoints>& curve, float x);
    /** `driveMakeup`: the gain that lands a −12 dBFS sine at `DriveTargetRms` after `stages`. */
    static float driveMakeup(const std::array<float, CurvePoints>& curve, int stages);

    enum class Cabinet { V30 = 0, Greenback = 1, American = 2, Jazz = 3 };
    enum class Mic { Center = 0, OffAxis = 1, Distant = 2 };

    /** `cabinetImpulse(sr, id, {presenceDb, resonanceDb, mic})` from cabinet.ts. */
    static std::vector<float> cabinetImpulse(double sampleRate, Cabinet cab, Mic mic,
                                             float presenceDb, float resonanceDb);
    /** `responseDbAt`: one bin of the DFT, in dB. */
    static double responseDbAt(const std::vector<float>& ir, double sampleRate, double hz);
    /** `roomImpulse(sr, seconds, 1)`, from a seeded generator so a rebuild is repeatable. */
    static std::vector<float> roomImpulse(double sampleRate, float seconds, unsigned seed);
    /** The scale Web Audio's `ConvolverNode` applies with `normalize = true`. */
    static float convolverNormalisation(const std::vector<float>& ir, double sampleRate);

private:
    // ---- settings, in wire units -----------------------------------------------------------
    bool enabled { false };
    float inputDb { 0.0f };
    bool gateEnabled { true };
    float gateThresholdDb { -58.0f };
    bool compEnabled { false };
    float compThresholdDb { -20.0f };
    float compRatio { 3.0f };
    float bassDb { 2.0f }, midDb { 0.0f }, midHz { 700.0f }, trebleDb { 2.0f };
    bool driveEnabled { true };
    float driveAmount { 0.35f };
    int driveStages { 2 };
    float driveBias { 0.18f };
    bool cabEnabled { true };
    float cabWidth { 0.35f };
    bool delayEnabled { false };
    float delayTimeSec { 0.34f }, delayFeedback { 0.28f }, delayMix { 0.22f };
    bool reverbEnabled { true };
    float reverbMix { 0.18f };
    float outputDb { 0.0f };
    bool limiterEnabled { true };
    float ceilingDb { -0.3f };

    // Read by the message thread when it builds an impulse response.
    std::atomic<int> cabModel { 0 };
    std::atomic<int> cabMic { 0 };
    std::atomic<float> cabPresenceDb { 0.0f };
    std::atomic<float> cabResonanceDb { 0.0f };
    std::atomic<float> reverbSizeSec { 1.6f };
    std::atomic<double> preparedRate { 0.0 };
    std::atomic<bool> cabDirty { true };
    std::atomic<bool> reverbDirty { true };
    std::atomic<bool> wantCab { false };
    std::atomic<bool> wantReverb { false };

    // ---- DSP state ---------------------------------------------------------------------------
    double sampleRate { 48000.0 };
    FilterPrimitives dcBlock;
    FilterPrimitives interLow[2];
    FilterPrimitives interDc[2];
    FilterPrimitives delayTone;
    ParametricEQ tone;   // before the drive: bass, mid, treble
    ParametricEQ post;   // after the cabinet: depth 120 Hz, presence 3.2 kHz
    Compressor comp;

    // Gate — `GateProcessor` in amp-dsp-processor.js.
    float gateEnv { 0.0f }, gateGain { 0.0f };
    int gateHold { 0 }, gateHoldFrames { 0 };
    float gateAttack { 0.0f }, gateRelease { 0.0f }, gateEnvCoef { 0.0f };

    // Drive.
    std::array<float, CurvePoints> curve {};
    bool curveActive { false };
    int curveStages { 0 };
    float makeup { 1.0f };
    bool driveStale { true };
    std::unique_ptr<juce::dsp::Oversampling<float>> oversampler[3];

    // Cabinet and reverb.
    juce::dsp::Convolution cabinet;
    juce::dsp::Convolution reverb;
    std::vector<float> wetBuffer;

    // Delay — a ring of two seconds.
    std::vector<float> delayLine;
    int delayWrite { 0 };

    // Limiter — `LimiterProcessor` in amp-dsp-processor.js, mono.
    std::vector<float> limDelay, limWindow;
    std::vector<long long> limDeque;
    int limCursor { 0 }, limSize { 0 }, limHead { 0 }, limTail { 0 };
    long long limPosition { 0 };
    float limGain { 1.0f }, limSlowGain { 1.0f }, limFastRel { 0.0f }, limSlowRel { 0.0f };

    void applyTone();
    void applyComp();
    void rebuildCurve();
    float limitSample(float x, float ceiling);
};

} // namespace dsp
