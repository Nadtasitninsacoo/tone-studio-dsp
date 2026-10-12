#pragma once

#include <cmath>
#include <optional>

namespace dsp {

/**
 * What the first argument of an incoming control message was. `Main.cpp` maps a
 * `juce::OSCArgument` onto this so the rule below can be tested without the OSC module.
 */
enum class ArgKind { Missing, Float32, Int32, Other };

/**
 * The value of a control message, or nothing.
 *
 * The receiver used to read "no argument" and "a string" as **0**, and 0 is a meaningful value on
 * this wire: `/master/limiter/enabled` with nothing after it switched the master limiter off,
 * `/master/mute "x"` unmuted the master, `/channel/1/fader` with no number set the fader to
 * 0 dB. A malformed message is refused, not read as the most dangerous number it could mean.
 * Non-finite is refused here too, before it reaches anything.
 */
inline std::optional<float> controlValue(ArgKind kind, float f, int i) {
    switch (kind) {
        case ArgKind::Float32: if (std::isfinite(f)) return f; return std::nullopt;
        case ArgKind::Int32:   return static_cast<float>(i);
        case ArgKind::Missing:
        case ArgKind::Other:   return std::nullopt;
    }
    return std::nullopt;
}

} // namespace dsp
