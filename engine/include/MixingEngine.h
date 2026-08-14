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
#include "ReverbDelay.h"
#include "Panner.h"
#include "Metering.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>
#include <string>

namespace dsp {

class MixingEngine {
public:
    static constexpr int MaxChannels = 32;
    static constexpr int MaxOutputs = 16;
    static constexpr int MaxAuxBuses = 8; // e.g. 6 monitors + 2 FX sends

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

    struct SendConfig {
        float levelDb { -120.0f };
        bool preFader { false };
        bool enabled { false };
    };

    struct Channel {
        std::string name { "Ch" };
        
        // Signal flow primitives
        float trimDb { 0.0f };
        bool phaseInvert { false };
        
        FilterPrimitives hpf;
        FilterPrimitives lpf;
        bool hpfEnabled { false };
        bool lpfEnabled { false };
        
        GateExpander gate;
        ParametricEQ eq;
        DeEsser deesser;
        Compressor comp;
        
        float faderDb { 0.0f };
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

        void prepare(double sampleRate, int maxBlockSize) {
            trimGain.reset(sampleRate, GainRampSeconds);
            faderGain.reset(sampleRate, GainRampSeconds);
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
            deesser.prepare(sampleRate, maxBlockSize);
            comp.prepare(sampleRate, maxBlockSize);
            panner.prepare(sampleRate);
            metering.prepare(sampleRate);
        }
        
        void reset() {
            hpf.reset();
            lpf.reset();
            gate.reset();
            eq.reset();
            deesser.reset();
            comp.reset();
            panner.prepare(48000.0); // reset panner
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
        juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> gain;

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
        }
        
        void reset() {
            geqL.reset();
            geqR.reset();
            suppressor.reset();
            limiter.reset();
            crossover.reset();
            metering.reset();
            gain.setCurrentAndTargetValue(targetGain());
        }
    };

    MixingEngine();
    ~MixingEngine() = default;

    void prepare(double sampleRate, int maxBlockSize);
    void reset();

    /**
     * Process multi-channel audio blocks.
     * inputs: array of input pointers. channelsCount specifies active inputs (up to 32).
     * outputs: array of output pointers. outputsCount specifies active outputs (up to 16).
     */
    void processAudio(const float** inputs, int channelsCount, float** outputs, int outputsCount, int numSamples);

    // Getters for UI/WebSocket metering
    Channel& getChannel(int index) { return channels[index]; }
    MasterBus& getMaster() { return master; }
    ReverbDelay& getFx() { return fxBus; }

private:
    double sampleRate { 48000.0 };
    int maxBlockSize { 512 };

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

    // Output delay lines (per physical output)
    std::array<DelayLine, MaxOutputs> outputDelays;

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
};

} // namespace dsp
