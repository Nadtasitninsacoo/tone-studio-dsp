#pragma once

#include "FilterPrimitives.h"
#include "ParametricEQ.h"
#include "GateExpander.h"
#include "Compressor.h"
#include "DeEsser.h"
#include "GraphicEQ.h"
#include "FeedbackSuppressor.h"
#include "Limiter.h"
#include "Crossover.h"
#include "DelayLine.h"
#include "SpeakerProcessor.h"
#include "ReverbDelay.h"
#include "Panner.h"
#include "Metering.h"
#include "GuitarAmp.h"
#include "BassRig.h"
#include "DrumRig.h"
#include "VocalRig.h"
#include "KeysRig.h"
#include "LightRig.h"
#include "LowEnd.h"
#include <algorithm>
#include <memory>
#include <array>
#include <atomic>
#include <cmath>
#include <vector>
#include <string>
#include <string_view>

namespace dsp {

class MixingEngine {
public:
    static constexpr int MaxChannels = 32;
    static constexpr int MaxOutputs = 32; // the X32 USB card is 32 out; 16 left outputs 17-32 unopened
    static constexpr int MaxAuxBuses = 10; // 8 monitors + 2 FX sends (reverb, delay)
    /**
     * Channels whose meters are reported to the web desk. All of them: the desk has 32 strips,
     * and a strip whose meter is never sent reads as a strip with no signal. Main.cpp sends these.
     */
    static constexpr int MeteredChannels = MaxChannels;

    /**
     * ---------------------------------------------------------------------------
     * THE ONLY TWO GAINS IN THIS ENGINE THAT WERE UNBOUNDED
     *
     * `faderDb` and `trimDb` are plain fields written straight from the OSC handler, and
     * every other setter in this codebase clamps — `setPan`, `setCeiling`, `setDelayFeedback`,
     * `setSensitivity`, all of them. These two did not, and `decibelsToGain` is happy to
     * return 1e10.
     *
     * That matters more than an ordinary range check because of what sits beside it: the
     * master limiter can be switched **off** over the same control plane
     * (`/master/limiter/enabled 0`). Unbounded gain plus a defeatable limiter, reachable
     * from a socket that until now accepted connections from anywhere, is a path from the
     * network to a blown driver or somebody's hearing.
     *
     * The values are ordinary console travel. A fader that reaches −80 is off in practice,
     * and `MinFaderDb` is deliberately finite rather than −inf so the clamp is total the way
     * `clampAmp` is in the web app: it never throws and never returns a partial answer.
     * ------------------------------------------------------------------------- */
    static constexpr float MinFaderDb = -80.0f;
    static constexpr float MaxFaderDb = 12.0f;
    static constexpr float MinTrimDb = -24.0f;
    static constexpr float MaxTrimDb = 24.0f;

    /**
     * Total, in the `clampAmp` sense: a non-finite input is **rejected to a safe value**
     * rather than clamped, because `std::clamp(NaN, lo, hi)` is NaN and NaN reaching a gain
     * multiply silences the bus with every reading on screen still looking correct.
     */
    static float clampFaderDb(float db) {
        if (!std::isfinite(db)) return 0.0f;
        return std::clamp(db, MinFaderDb, MaxFaderDb);
    }
    static float clampTrimDb(float db) {
        if (!std::isfinite(db)) return 0.0f;
        return std::clamp(db, MinTrimDb, MaxTrimDb);
    }

    /**
     * What one physical output carries — the output patch (1.0.17). `Legacy` is the routing
     * this engine has always had (master on 0/1, or mains/sub on 0..3 with the crossover on),
     * kept for a desk that never sends a patch. Codes are the wire's: `/output/<n>/source`.
     */
    enum class OutputSource : int {
        None = 0, MasterL = 1, MasterR = 2, MainL = 3, MainR = 4, SubL = 5, SubR = 6,
        Aux1 = 7, Aux2 = 8, Aux3 = 9, Aux4 = 10, Aux5 = 11, Aux6 = 12, Aux7 = 13, Aux8 = 14,
    };
    static constexpr int OutputSourceCount = 15;
    /** Aux 1..8 are the monitor sends (six before 1.0.22); the next two buses are the reverb and delay sends. */
    static constexpr int MonitorAuxBuses = 8;
    static constexpr int ReverbAux = MonitorAuxBuses;
    static constexpr int DelayAux = MonitorAuxBuses + 1;
    static_assert(DelayAux < MaxAuxBuses, "the FX sends need their own buses");

    /**
     * DCA groups (1.0.20). A DCA carries no audio: it is a fader and a mute that scale every
     * channel assigned to it, **without moving those channels' own faders**. A channel in several
     * DCAs takes the sum of their dB, as on an X32. A muted DCA silences its channels, sends
     * included where they are post-fader, because the effective fader is what reaches zero.
     */
    static constexpr int DcaCount = 8;
    struct Dca {
        float gainDb { 0.0f };
        bool muted { false };
    };
    const Dca& dca(int index) const { return dcas[static_cast<size_t>(std::clamp(index, 0, DcaCount - 1))]; }
    /** The linear gain the DCAs apply to channel `index` — 1 when it is in none. */
    float dcaGainFor(int index) const;

    /**
     * Subgroups (1.0.22). Unlike a DCA a subgroup is *after* the channel: its fader and mute
     * scale only what the channel puts on the main bus, never its sends, because on the web
     * desk a monitor send leaves the strip before the subgroup bus. A channel is in at most
     * one subgroup (`channel/<n>/group`, 0 = none, 1..8). The group's pan is web-only.
     */
    static constexpr int GroupCount = 8;
    struct Group {
        float gainDb { 0.0f };
        bool muted { false };
    };
    const Group& group(int index) const { return groups[static_cast<size_t>(std::clamp(index, 0, GroupCount - 1))]; }
    /** The linear gain channel `index`'s subgroup applies to its main-bus contribution — 1 when in none. */
    float groupGainFor(int index) const;

    /**
     * One monitor mix (aux 1..8): the sum of the channels' sends, then its own 3-band EQ,
     * its master fader and mute, and a peak limiter, so a wedge or an in-ear mix can be
     * shaped and protected on its own. Mono, like the aux bus it processes.
     *
     * The limiter is a sample-peak brickwall with instant attack and no lookahead, not the
     * master's oversampled true-peak `Limiter`: six of those would be six oversamplers for a
     * job whose point is "never louder than this", and instant attack never lets a peak
     * through. Its cost is some distortion on a hit that lands on it, which on a monitor is
     * the right trade against a blown wedge or somebody's ears.
     */
    struct MonitorMix {
        float gainDb { 0.0f };
        bool muted { false };
        float lowDb { 0.0f };
        float midDb { 0.0f };
        float highDb { 0.0f };
        bool limiterEnabled { true };
        float ceilingDb { -1.0f };

        FilterPrimitives low, mid, high;
        juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> gain;
        float limGain { 1.0f };
        float releaseCoef { 0.0f };

        void prepare(double sampleRate, int maxBlockSize) {
            low.prepare(sampleRate, maxBlockSize);
            mid.prepare(sampleRate, maxBlockSize);
            high.prepare(sampleRate, maxBlockSize);
            low.setType(FilterPrimitives::Type::LowShelf);
            mid.setType(FilterPrimitives::Type::Peaking);
            high.setType(FilterPrimitives::Type::HighShelf);
            applyEq();
            gain.reset(sampleRate, 0.02);
            gain.setCurrentAndTargetValue(targetGain());
            // About 150 ms back to unity once a peak has passed.
            releaseCoef = 1.0f - std::exp(-1.0f / (0.15f * static_cast<float>(sampleRate)));
            limGain = 1.0f;
        }
        void applyEq() {
            low.setParameters(120.0f, 0.7071f, lowDb);
            mid.setParameters(1000.0f, 0.9f, midDb);
            high.setParameters(6000.0f, 0.7071f, highDb);
        }
        float targetGain() const {
            return muted ? 0.0f : juce::Decibels::decibelsToGain(clampFaderDb(gainDb));
        }
        void process(float* buffer, int numSamples) {
            gain.setTargetValue(targetGain());
            const bool eqOn = lowDb != 0.0f || midDb != 0.0f || highDb != 0.0f;
            const float ceiling = juce::Decibels::decibelsToGain(ceilingDb);
            for (int i = 0; i < numSamples; ++i) {
                float x = buffer[i];
                if (eqOn) x = high.processSample(mid.processSample(low.processSample(x)));
                x *= gain.getNextValue();
                if (limiterEnabled) {
                    const float a = std::abs(x);
                    const float want = a > ceiling ? ceiling / a : 1.0f;
                    limGain = want < limGain ? want : limGain + (1.0f - limGain) * releaseCoef;
                    x *= limGain;
                    // The clamp is what makes it a ceiling: the release step can leave the
                    // first sample of a rise a hair above it.
                    x = std::clamp(x, -ceiling, ceiling);
                }
                buffer[i] = x;
            }
        }
    };

    /**
     * ---------------------------------------------------------------------------
     * THE TEST TONES, WHICH A STAGE ENGINE DID NOT HAVE
     *
     * Finding which cable goes to which speaker is the first job at every soundcheck, and the only
     * per-jack test this system had played out of the **browser's** sound device — not this
     * engine's card. On the engine path there was no way at all to make one output beep.
     *
     * `output/<n>/test` beeps output n: n short beeps for 1..8, one long tone above that, so the
     * person at a wedge can tell which jack it is without a screen. It goes **through that output's
     * speaker processor** — its crossover, polarity, mute and protection limiter — because a jack
     * driving a compression driver must never be handed an unfiltered tone, and at a pitch inside
     * the processor's own pass band, so a sub behind an 80 Hz low-pass still beeps (at ~50 Hz)
     * rather than being silent and read as a dead cable. Not through the patch, the master or any
     * mix: the question is the cable.
     *
     * `aux/<n>/test` puts a 1.5 s 1 kHz tone into monitor mix n's bus, *before* its EQ, fader,
     * limiter and patch — which separates "the sends are wrong" from "the mix goes nowhere".
     *
     * Both are actions: applied, never journalled, never restored after a restart. A new test
     * replaces one that is running. Levels: −24 dBFS out of a jack, −30 dBFS into a mix.
     * ------------------------------------------------------------------------- */
    struct TestTone {
        double fs { 48000.0 };
        float level { 0.0f };
        double phase { 0.0 };
        double inc { 0.0 };
        int onSamples { 0 }, periodSamples { 1 }, count { 0 };
        int pos { 0 };
        int target { -1 };

        bool active() const { return target >= 0; }
        void start(double sampleRate, int which, float hz, float lvl, int beeps, double onS, double periodS) {
            fs = sampleRate;
            level = lvl;
            phase = 0.0;
            inc = 2.0 * 3.14159265358979323846 * static_cast<double>(hz) / fs;
            onSamples = std::max(1, static_cast<int>(onS * fs));
            periodSamples = std::max(1, static_cast<int>(periodS * fs));
            count = std::max(1, beeps);
            pos = 0;
            target = which;
        }
        void stop() { target = -1; }
        /** Writes the next `n` samples (zeros between beeps). Returns false, writing nothing, when idle. */
        bool fill(float* out, int n) {
            if (!active()) return false;
            const float ramp = static_cast<float>(std::max(1, static_cast<int>(0.01 * fs)));
            const int total = count * periodSamples;
            for (int i = 0; i < n; ++i, ++pos) {
                float v = 0.0f;
                if (pos < total) {
                    const int inBeep = pos % periodSamples;
                    if (inBeep < onSamples) {
                        const float env = std::min({ 1.0f, static_cast<float>(inBeep) / ramp,
                                                     static_cast<float>(onSamples - inBeep) / ramp });
                        v = level * env * static_cast<float>(std::sin(phase));
                    }
                }
                phase += inc;
                if (phase > 6.283185307179586) phase -= 6.283185307179586;
                out[i] = v;
            }
            if (pos >= total) target = -1;
            return true;
        }
    };
    /** How many beeps output `n` (0-based) gets, and their timing — the web page's `outputBeeps`. */
    static void outputBeeps(int output, int& count, double& onS, double& periodS) {
        const int n = output + 1;
        if (n > 8) { count = 1; onS = 1.5; periodS = 1.5; return; }
        count = n; onS = 0.16; periodS = 0.32;
    }
    /** The pitch an output's test beeps at: 1 kHz, moved inside its speaker processor's pass band. */
    static float testPitchFor(const SpeakerProcessor::Settings& s);
    bool isOutputTesting() const { return outputTone.active(); }
    bool isMonitorTesting() const { return monitorTone.active(); }

    /** `aux/<n>/<param>`: gain, mute, eq/low|mid|high, limiter/enabled, limiter/ceiling, test. */
    bool setMonitorParam(int mix, std::string_view param, float value);
    const MonitorMix& getMonitor(int mix) const {
        return monitors[static_cast<size_t>(std::clamp(mix, 0, MonitorAuxBuses - 1))];
    }

    /**
     * The finished master of the last block — after the GEQ, the suppressor and the limiter,
     * before the output patch and the crossover. What the master recorder writes. Valid after
     * `processAudio` returns, on the audio thread, for that block's sample count.
     */
    const float* masterLeft() const { return mainBusL.data(); }
    const float* masterRight() const { return mainBusR.data(); }

    /** The output patch. `patched` false = legacy routing, untouched. */
    void setOutputSource(int output, OutputSource source);
    /** One output's speaker processor, for the tests and the meters. */
    const SpeakerProcessor& speaker(int output) const { return speakers[static_cast<size_t>(std::clamp(output, 0, MaxOutputs - 1))]; }
    /**
     * What has left one physical output since the last call — the sample peak (linear, after the
     * speaker processor and the alignment delay) and the most its protection limiter took (dB, >= 0) —
     * and resets both (1.0.24). Called from the meter timer, never from the audio thread. An output
     * nothing was routed to in that time reads 0 and 0: silence, which is true.
     */
    float takeOutputPeak(int output);
    float takeOutputReductionDb(int output);
    /** The alignment delay on one output, ms. */
    float outputDelay(int output) const { return outputDelayMs[static_cast<size_t>(std::clamp(output, 0, MaxOutputs - 1))]; }
    /** The longest alignment delay an output takes, ms (~170 m). */
    static constexpr float MaxOutputDelayMs = 500.0f;
    OutputSource getOutputSource(int output) const;
    bool isOutputPatched() const { return outputPatched; }

    struct SendConfig {
        float levelDb { -120.0f };
        bool preFader { false };
        bool enabled { false };
    };

    /**
     * ---------------------------------------------------------------------------
     * THE WIRE SPEAKS THE AUDIO EQ COOKBOOK, AND THIS FILTER DOES NOT
     *
     * A channel EQ band arrives from the web desk as a shape, a frequency, a Q and a gain,
     * defined the way every console and Web Audio's `BiquadFilterNode` define them (Robert
     * Bristow-Johnson's cookbook). `FilterPrimitives` is a state-variable filter whose
     * parameters mean something slightly different, and handing the numbers straight across
     * compiles, runs and sounds *almost* right — which is the worst way to be wrong:
     *
     *  - **A peak's Q.** The SVF bell's bandwidth is set in the boosted direction only, so
     *    the same Q is narrower than the cookbook's by √K on a boost and wider by √K on a
     *    cut. At the web desk's ±15 dB that is a factor of 2.4 either way.
     *  - **A shelf's frequency.** The cookbook's shelf frequency is the midpoint of the
     *    transition; the SVF's is its pole, so the midpoint lands at f·K^¼ — a 120 Hz low
     *    shelf boosted 15 dB turns over at 185 Hz, and cut 15 dB at 78 Hz. The frequency
     *    moved with the gain knob.
     *
     * Both are exact algebra on the analog prototypes, and the frequency is converted in the
     * prewarped domain so it holds right up to Nyquist. It needs the sample rate, which is
     * why it lives here and not in the web app — the only place that knows both what the
     * wire means and what this filter means. `ChannelParamTests` compares the result against
     * the cookbook's own formulas, not against the numbers in this function.
     * ------------------------------------------------------------------------- */
    enum class EqShape { Peaking = 0, LowShelf = 1, HighShelf = 2 };

    struct EqBandSetting {
        EqShape shape { EqShape::Peaking };
        float frequencyHz { 1000.0f };
        float q { 0.7071f };
        float gainDb { 0.0f };
        bool enabled { false };
    };

    static ParametricEQ::BandConfig cookbookToSvf(const EqBandSetting& band, double sampleRate);

    /** `Channel::inputIndex` for a channel that listens to nothing. */
    static constexpr int NoInput = -1;

    struct Channel {
        std::string name { "Ch" };

        /**
         * Which physical input feeds this channel — the input patch.
         *
         * It was implicitly the channel's own index, so a two-input interface could only ever
         * fill two channels, and the web desk's eight strips reached eight channels of which
         * six had no hardware behind them: they moved, sent their fader and EQ, and made no
         * sound. Now any number of channels can take the same input, each with its own EQ,
         * dynamics and fader, and a channel the desk has given no source takes `NoInput`.
         *
         * Defaults to the channel's own index (set by the engine's constructor), so a desk
         * that never sends a patch behaves exactly as before.
         */
        int inputIndex { NoInput };

        // Signal flow primitives
        float trimDb { 0.0f };
        bool phaseInvert { false };
        
        FilterPrimitives hpf;
        FilterPrimitives lpf;
        bool hpfEnabled { false };
        bool lpfEnabled { false };
        
        GateExpander gate;
        ParametricEQ eq;
        /**
         * The guitar rack, as an insert after the EQ and before the dynamics — where the web
         * desk puts a strip's rack. Off until the desk sends `amp/enabled 1`, and off costs a
         * branch. Behind a pointer only because `juce::dsp::Convolution` needs the engine's
         * shared loader queue to be built; the engine's constructor makes one per channel.
         */
        std::unique_ptr<GuitarAmp> amp;
        /** The bass rig (1.0.23), the same kind of insert in the same place. `bass/enabled 1` switches it on. */
        std::unique_ptr<BassRig> bass;
        /** The drum bus (1.0.23), the same kind of insert. `drums/enabled 1` switches it on. */
        std::unique_ptr<DrumRig> drums;
        /** The vocal rack (1.0.23). `vocals/enabled 1` switches it on. */
        std::unique_ptr<VocalRig> vocals;
        /** Keys, and the four racks that share one chain (1.0.23): brass, phin, khaen, saw. */
        std::unique_ptr<KeysRig> keys;
        std::unique_ptr<LightRig> brass, phin, khaen, saw;
        DeEsser deesser;
        Compressor comp;

        /**
         * **The alignment delay, which the web desk had and this engine did not.**
         *
         * The desk's strip has carried an align delay (0–50 ms) since it had a strip, and on the
         * engine path it was understood, logged and heard by nobody — the web refused it by name
         * (`ENGINE_DEAD_DESK_KINDS`) rather than pretend. It sits after the dynamics and before the
         * fader, so the sends and the main bus both take the aligned signal: it aligns the
         * channel, not one stage of it — the web desk's own reasoning, one stage earlier because
         * here the sends leave from the fader.
         *
         * A delay of 0 is an exact pass-through (`DelayLine`'s interpolator has weight 1 on the
         * written sample), so every channel runs it and nothing changes until it is set.
         */
        DelayLine alignDelay;
        float delayMs { 0.0f };
        static constexpr float MaxDelayMs = 50.0f;

        float faderDb { 0.0f };
        /** Bit n set = this channel is in DCA n+1. */
        uint32_t dcaMask { 0 };
        /** 0 = in no subgroup, n = subgroup n (1-based, as on the wire). */
        int group { 0 };
        /** The subgroup's gain, ramped like the fader: a subgroup move must not click either. */
        juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> groupGain;
        Panner panner;

        std::array<SendConfig, MaxAuxBuses> sends;
        ChannelMetering metering;

        bool routedToMain { true };

        /**
         * **Trim and fader are ramped, not stepped.**
         *
         * `decibelsToGain` was called once per block and applied as a constant, so every
         * fader move was a step discontinuity at a block boundary — a click on each one, and
         * a burst of them while somebody rides a fader. `Panner` was already doing this
         * correctly with its own `SmoothedValue`; these two were the odd ones out, which is
         * what made it a bug rather than a design.
         *
         * 20 ms is the same order as the ramps the web app uses when it connects and
         * disconnects a rack channel, and for the same reason: long enough to be inaudible,
         * short enough that the control still feels immediate.
         */
        juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> trimGain;
        juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> faderGain;
        static constexpr double GainRampSeconds = 0.02;

        /**
         * Where the ramp has actually got to. Read by the tests.
         *
         * It is exposed because the claim cannot be checked from the audio: a step in level
         * is smeared by the master GEQ and the Linkwitz-Riley crossover over a few hundred
         * samples, which is the same order as the ramp itself — a first attempt at an
         * output-level assertion measured the crossover's decay and passed happily with the
         * ramp removed. This reads the mechanism instead.
         */
        float currentFaderGain() const { return faderGain.getCurrentValue(); }

        /**
         * The strip's filter settings, kept in wire units.
         *
         * `FilterPrimitives` takes frequency and Q in one call and has no getters, so a
         * message carrying only a new Q needs the frequency from somewhere. And the EQ has to
         * be re-derived when the sample rate changes (see `cookbookToSvf`), which is only
         * possible if the cookbook values survive — a device change from 48 to 44.1 kHz would
         * otherwise leave every band at the old conversion.
         */
        std::array<EqBandSetting, ParametricEQ::NumBands> eqSettings {};
        float hpfHz { 80.0f };
        float hpfQ { 0.7071f };
        float lpfHz { 18000.0f };
        float lpfQ { 0.7071f };
        double sampleRate { 48000.0 };

        void applyEqBand(int band) {
            eq.setBandParameters(band, cookbookToSvf(eqSettings[static_cast<size_t>(band)], sampleRate));
        }
        void applyFilters() {
            hpf.setParameters(hpfHz, hpfQ, 0.0f);
            lpf.setParameters(lpfHz, lpfQ, 0.0f);
        }

        void prepare(double newSampleRate, int maxBlockSize) {
            sampleRate = newSampleRate;
            trimGain.reset(sampleRate, GainRampSeconds);
            faderGain.reset(sampleRate, GainRampSeconds);
            groupGain.reset(sampleRate, GainRampSeconds);
            groupGain.setCurrentAndTargetValue(1.0f);
            // Start *at* the current setting rather than ramping up to it from zero: a
            // device change must not fade the desk in from silence.
            trimGain.setCurrentAndTargetValue(juce::Decibels::decibelsToGain(clampTrimDb(trimDb)));
            faderGain.setCurrentAndTargetValue(juce::Decibels::decibelsToGain(clampFaderDb(faderDb)));
            hpf.prepare(sampleRate, maxBlockSize);
            hpf.setType(FilterPrimitives::Type::HighPass);
            
            lpf.prepare(sampleRate, maxBlockSize);
            lpf.setType(FilterPrimitives::Type::LowPass);
            
            gate.prepare(sampleRate, maxBlockSize);
            eq.prepare(sampleRate, maxBlockSize);
            if (amp) amp->prepare(sampleRate, maxBlockSize);
            if (bass) bass->prepare(sampleRate, maxBlockSize);
            if (drums) drums->prepare(sampleRate, maxBlockSize);
            if (vocals) vocals->prepare(sampleRate, maxBlockSize);
            if (keys) keys->prepare(sampleRate, maxBlockSize);
            for (auto* light : { brass.get(), phin.get(), khaen.get(), saw.get() }) if (light) light->prepare(sampleRate, maxBlockSize);
            deesser.prepare(sampleRate, maxBlockSize);
            comp.prepare(sampleRate, maxBlockSize);
            alignDelay.prepare(sampleRate, maxBlockSize);
            alignDelay.setDelayMs(delayMs);
            panner.prepare(sampleRate);
            metering.prepare(sampleRate);
            applyFilters();
            for (int b = 0; b < ParametricEQ::NumBands; ++b) applyEqBand(b);
        }
        
        /**
         * Clear every filter and envelope, and nothing else: the gains, the pan and the meters
         * keep their place. What the audio thread calls when this channel produced a NaN or an
         * infinity — an IIR filter that has taken one keeps it in its state for ever, so the
         * channel would otherwise stay silent (or poison the bus) until the device restarted.
         * Allocation-free.
         */
        void clearDspState() {
            hpf.reset();
            lpf.reset();
            gate.reset();
            eq.reset();
            if (amp) amp->reset();
            if (bass) bass->reset();
            if (drums) drums->reset();
            if (vocals) vocals->reset();
            if (keys) keys->reset();
            for (auto* light : { brass.get(), phin.get(), khaen.get(), saw.get() }) if (light) light->reset();
            deesser.reset();
            comp.reset();
            alignDelay.reset();
            alignDelay.setDelayMs(delayMs);
        }

        void reset() {
            clearDspState();
            // The channel's own rate, not 48 kHz: the pan ramp is 20 ms at whatever the device runs.
            panner.prepare(sampleRate);
            metering.reset();
            // Land on the current setting rather than ramping to it — reset() runs when a
            // device stops, and the desk must come back at the level it was left at.
            trimGain.setCurrentAndTargetValue(juce::Decibels::decibelsToGain(clampTrimDb(trimDb)));
            faderGain.setCurrentAndTargetValue(juce::Decibels::decibelsToGain(clampFaderDb(faderDb)));
        }
    };

    struct MasterBus {
        GraphicEQ geqL;
        GraphicEQ geqR;
        FeedbackSuppressor suppressor;
        Limiter limiter;
        Crossover crossover;
        MasterMetering metering;

        /**
         * **The master fader, which did not exist.**
         *
         * `bridge.js` has always translated `masterGain` and `masterMute` into
         * `/master/gain` and `/master/mute`, and `Main.cpp` dropped both on the floor with a
         * comment explaining that the engine had no such field. So the web app's master
         * fader moved, reported success, and changed nothing — a control that does nothing,
         * spread across two repositories, which is the one product rule this project treats
         * as non-negotiable.
         *
         * Applied **after the GEQ and before the suppressor and the limiter**: it is a mix
         * decision, so everything that protects the output has to see its result. Putting it
         * after the limiter would let the master fader push the desk past a ceiling the
         * limiter had already enforced.
         *
         * Same range as a channel fader, and the same ramp, for the same reason.
         */
        float gainDb { 0.0f };
        bool muted { false };

        /**
         * **Whether the master is split into sub and mains on four outputs. Off by default.**
         *
         * It used to be always on, and on any device with four or more outputs the split
         * decided the channel map: outputs 0/1 carried the *sub* band and the programme went
         * to 2/3. Windows opens plenty of ordinary stereo hardware as a multichannel endpoint
         * — the Tank-G reports "2 in / 8 out", a laptop with a Dolby APO the same — so the
         * first real listen was the full programme on the meters and near silence in the
         * headphones, the music on two outputs nothing was connected to. Reported as "เสียงเข้า
         * แต่ไม่ออกลำโพง".
         *
         * Off, outputs 0/1 are the full-range master on every device. On (a sub rig is wired),
         * 0/1 are the mains above the crossover and 2/3 the sub below it — mains first, so a
         * speaker on the first pair never becomes a subwoofer by accident.
         */
        bool crossoverEnabled { false };
        juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> gain;

        /**
         * The web desk's low end, here too — see `LowEnd.h`. The key and target are 0-based
         * channel indices, −1 for none; the wire sends them 1-based with 0 meaning none.
         */
        SidechainDuck duck;
        bool sidechainEnabled { false };
        int sidechainKey { -1 };
        int sidechainTarget { -1 };
        MonoBass monoBass;

        bool sidechainActive() const {
            return sidechainEnabled && sidechainKey >= 0 && sidechainKey < MaxChannels &&
                   sidechainTarget >= 0 && sidechainTarget < MaxChannels &&
                   sidechainKey != sidechainTarget;
        }

        /** The gain the fader and the mute switch multiply to. One writer, one value. */
        float targetGain() const {
            return muted ? 0.0f : juce::Decibels::decibelsToGain(clampFaderDb(gainDb));
        }

        void prepare(double sampleRate, int maxBlockSize) {
            gain.reset(sampleRate, Channel::GainRampSeconds);
            gain.setCurrentAndTargetValue(targetGain());
            geqL.prepare(sampleRate, maxBlockSize);
            geqR.prepare(sampleRate, maxBlockSize);
            suppressor.prepare(sampleRate, maxBlockSize);
            limiter.prepare(sampleRate, maxBlockSize);
            crossover.prepare(sampleRate, maxBlockSize);
            metering.prepare(sampleRate, maxBlockSize);
            duck.prepare(sampleRate, maxBlockSize);
            monoBass.prepare(sampleRate, maxBlockSize);
        }
        
        void reset() {
            geqL.reset();
            geqR.reset();
            suppressor.reset();
            limiter.reset();
            crossover.reset();
            metering.reset();
            duck.reset();
            monoBass.reset();
            gain.setCurrentAndTargetValue(targetGain());
        }
    };

    MixingEngine();
    ~MixingEngine() = default;

    void prepare(double sampleRate, int maxBlockSize);
    void reset();

    /**
     * Message thread, a few times a second. Builds any rack impulse response a setting has
     * made stale — the one part of the racks that allocates, so the audio thread only flags it.
     */
    void serviceBackgroundWork();

    /**
     * Process multi-channel audio blocks.
     * inputs: array of input pointers. channelsCount specifies active inputs (up to 32).
     * outputs: array of output pointers. outputsCount specifies active outputs (up to 16).
     */
    void processAudio(const float** inputs, int channelsCount, float** outputs, int outputsCount, int numSamples);

    /**
     * One channel control, by its wire name — `fader`, `hpf/freq`, `eq/2/gain`, `comp/attack`.
     *
     * This is the whole `/channel/<n>/...` routing table, and it lives here rather than in
     * `Main.cpp` because that file is not reached by the test suite: the last routing bug in
     * it (`removeEmptyStrings`) meant *no command had ever reached the DSP*, and nothing
     * could see it. Here every path is asserted.
     *
     * Returns false — and changes nothing — for an unknown name, a channel out of range, or a
     * non-finite value. A `NaN` threshold is not a request to compress; the rest of this
     * engine rejects non-finite the same way (`clampFaderDb`).
     *
     * Units are the wire's: dB, Hz, linear Q, **milliseconds** for times, 0|1 for switches.
     */
    bool setChannelParam(int index, std::string_view param, float value);

    /**
     * One control by its OSC address without the leading slash — `channel/3/eq/2/gain`,
     * `master/limiter/ceiling`, `master/fx/delay/time`. The whole control plane except the
     * feedback suppressor, which runs its own detection thread and keeps its own setters.
     * Same contract as `setChannelParam`: false and no change for anything it cannot use.
     *
     * **Call it only where the audio thread is not running** — tests, or `serviceControls`.
     * From the OSC thread use `postControl`.
     */
    bool setControl(std::string_view address, float value);

    /**
     * The first word of every address `setControl` routes — `channel`, `master`, `suppressor`,
     * `dca`, `group`, `aux`, `output`. The OSC receiver asks this rather than keeping its own list:
     * its own list lacked `dca` and `group`, so both features were unreachable from the web.
     */
    static bool isControlRoot(std::string_view word) {
        return word == "channel" || word == "master" || word == "suppressor" || word == "dca"
            || word == "group" || word == "aux" || word == "output";
    }

    /**
     * ---------------------------------------------------------------------------
     * THE CONTROL PLANE WROTE INTO THE DSP WHILE THE AUDIO THREAD WAS READING IT
     *
     * Every OSC message was applied on JUCE's message thread, straight into filters and
     * compressors the audio callback was running at that moment: a `SmoothedValue` half
     * re-targeted, a biquad switching type mid-sample, three coefficients from two different
     * settings. With four controls per channel that was rare and survivable. With a channel
     * strip it is dozens per knob turn, and moving the instrument racks in makes it hundreds.
     *
     * So the OSC thread only **queues** — a lock-free single-producer FIFO, no allocation —
     * and the audio thread applies everything pending at the top of the next block, before a
     * single sample is processed. `prepare()` drains too, so a change made while the device
     * was stopped is waiting in the DSP when it starts.
     *
     * A full queue refuses rather than blocks (the OSC thread must never wait on the audio
     * thread), and the refusal is counted so the caller can say so.
     * ------------------------------------------------------------------------- */
    bool postControl(std::string_view address, float value);
    /** Apply everything queued. Audio thread (top of each block) and `prepare`. */
    void serviceControls();
    /** Controls refused because the queue was full, since start. */
    int droppedControls() const { return dropped.load(); }
    /** Queued controls the table refused (unknown, out of range, not a number), since start. */
    int rejectedControls() const { return rejected.load(); }
    static constexpr int ControlQueueSize = 4096;
    static constexpr size_t MaxControlAddress = 47;

    /** One control as it travels the queues: its wire address and its value. */
    struct ControlRecord {
        char address[MaxControlAddress + 1] {};
        float value { 0.0f };
    };

    /**
     * ---------------------------------------------------------------------------
     * THE JOURNAL: what the DSP actually accepted, handed to the thing that saves it
     *
     * `serviceControls` writes every control it applied *successfully* into a second lock-free
     * FIFO, and `drainApplied` reads it from another thread. That is the whole bridge between
     * the audio thread and the disk: no allocation, no lock and no I/O on the audio side, and
     * nothing is recorded that `setControl` refused — so a typo from a client, a channel out of
     * range or a NaN can never become saved state. See `ControlStore`.
     *
     * One consumer. A full journal drops the *newest* record and counts it (`lostApplied`); it
     * holds as many as the control queue does, so it can only fill if the consumer has stopped.
     * ------------------------------------------------------------------------- */
    int drainApplied(ControlRecord* out, int maxCount);

    /**
     * How many times a channel produced a NaN or an infinity and was cleared, since start. The
     * audio thread cannot print, so it counts; `Main.cpp` turns a change into a log line and the
     * device state reports the total. A non-zero count is a bug or a broken input, never normal.
     */
    int nonFiniteResets() const { return nonFinite.load(std::memory_order_relaxed); }
    int lostApplied() const { return appliedLost.load(); }
    static constexpr int AppliedQueueSize = 4096;

    // Getters for UI/WebSocket metering
    Channel& getChannel(int index) { return channels[index]; }
    MasterBus& getMaster() { return master; }
    ReverbDelay& getFx() { return fxBus; }

private:
    bool setMasterParam(std::string_view param, float value);
    bool setSuppressorParam(std::string_view param, float value);
    void applyQueued(const ControlRecord& record) noexcept;

    using PendingControl = ControlRecord;
    void journalApplied(const ControlRecord& record) noexcept;
    // On the heap, not inline: ~200 KB inside an object `Main.cpp` keeps on the stack.
    std::vector<PendingControl> controlQueue;
    juce::AbstractFifo controlFifo { ControlQueueSize };
    std::vector<ControlRecord> appliedQueue;
    juce::AbstractFifo appliedFifo { AppliedQueueSize };
    std::atomic<int> appliedLost { 0 };
    juce::SpinLock consumerLock;
    std::atomic<int> dropped { 0 };
    std::atomic<int> rejected { 0 };
    std::atomic<int> nonFinite { 0 };

    double sampleRate { 48000.0 };
    int maxBlockSize { 512 };

    // Declared before the channels so it outlives the convolutions that post to it. One
    // background thread for all 32 racks, not one each. On the heap, like the control queue:
    // an engine is often a stack object (the tests hold eleven), and the queue runs a thread.
    std::unique_ptr<juce::dsp::ConvolutionMessageQueue> convolutionQueue;

    std::array<Channel, MaxChannels> channels;
    
    // Aux monitor sum buses (pre-allocated)
    std::array<std::vector<float>, MaxAuxBuses> auxBuses;
    
    // FX return bus (Reverb/Delay)
    ReverbDelay fxBus;
    std::vector<float> fxReturnL;
    std::vector<float> fxReturnR;

    // Main stereo sum bus L/R
    std::vector<float> mainBusL;
    std::vector<float> mainBusR;

    // Master processing
    MasterBus master;

    std::array<MonitorMix, MonitorAuxBuses> monitors;
    std::array<Dca, DcaCount> dcas {};
    bool setDcaParam(int dca, std::string_view param, float value);
    std::array<Group, GroupCount> groups {};
    bool setGroupParam(int group, std::string_view param, float value);
    std::array<OutputSource, MaxOutputs> outputSources {};
    bool outputPatched { false };

    // Output delay lines (per physical output)
    std::array<DelayLine, MaxOutputs> outputDelays;
    // The speaker processor on each physical output (1.0.20): crossover, EQ, polarity, limiter.
    // Runs before `outputDelays`, which is where its alignment delay lives.
    std::array<SpeakerProcessor, MaxOutputs> speakers;
    std::array<float, MaxOutputs> outputDelayMs {};
    // Held between meter reads: the audio thread raises them, the meter timer takes and zeroes them.
    std::array<std::atomic<float>, MaxOutputs> outputPeak {};
    std::array<std::atomic<float>, MaxOutputs> outputReduction {};
    /** Speaker processor, then the alignment delay — the last thing every output goes through. */
    void finishOutput(int output, float* buffer, int numSamples);
    bool setOutputParam(int output, std::string_view param, float value);

    // Sub / Main crossover outputs
    std::vector<float> subL;
    std::vector<float> subR;
    std::vector<float> mainL;
    std::vector<float> mainR;

    // Temporary real-time buffers
    std::vector<float> postDspBuffer;
    std::vector<float> postFaderBuffer;
    std::vector<float> masterInterleaved;
    std::vector<float> subInterleaved;
    std::vector<float> mainInterleaved;

    TestTone outputTone;
    TestTone monitorTone;
    std::vector<float> testBuffer;
};

} // namespace dsp
