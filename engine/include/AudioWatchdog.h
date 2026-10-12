#pragma once

#include <cstdint>

namespace dsp {

/**
 * ---------------------------------------------------------------------------
 * WHAT THE WATCHDOG MAY DO, AND THE ONE THING IT MUST NOT
 *
 * It decided from two readings, and one of them was wrong to act on. "The callback stopped" is a
 * stream that has died: the output is already silent, so closing and reopening the device costs
 * nothing and is the only cure. "Every input has delivered exact zeros for 3 s" is **what a stage
 * sounds like between songs** — the Tank-G sends digital silence when nobody plays, and so does an
 * input with nothing plugged in or a muted channel. Restarting on that cut an output that was
 * working, and a reopen that Windows refused ("Can't open the audio device") turned it into
 * seconds of silence in the middle of a show. A cap of two restarts did not help: any sound reset
 * the budget, so every pause longer than 3 s could cut the PA again.
 *
 * So silence is **reported, never acted on**. The watchdog restarts only a stalled stream, and
 * when a device is down it tries to reopen it quickly at first (a dropout is usually over in a
 * second or two) and then every two seconds, so an unplugged card does not spin.
 *
 * Pure: the readings come in, a decision goes out. `Main.cpp` owns the device; the tests own this.
 * ------------------------------------------------------------------------- */
class AudioWatchdog {
public:
    /** No callback for this long while the device says it is playing = the stream died. */
    static constexpr uint32_t StallMs = 3000;
    /** Exact zeros on every input for this long = worth telling somebody (and nothing more). */
    static constexpr double SilentInputSec = 3.0;
    /** While a device is down: try this often for the first `FastReopenForMs`, then every `SlowReopenMs`. */
    static constexpr uint32_t FastReopenMs = 250;
    static constexpr uint32_t FastReopenForMs = 4000;
    static constexpr uint32_t SlowReopenMs = 2000;

    enum class Action {
        None,
        /** Close and reopen the same setup: the stream has stopped calling back. */
        RestartStalled,
        /** The device is not running: try to open the chosen setup again. */
        Reopen,
    };

    struct Reading {
        uint32_t nowMs { 0 };
        /** The audio callback's block counter. */
        uint64_t blocks { 0 };
        /** The device exists and says it is playing. */
        bool playing { false };
        /** How long every input has delivered exact zeros, in seconds. */
        double silentInputSec { 0.0 };
        /** The open device has at least one input. */
        bool hasInputs { false };
    };

    /** Start watching from now — after an open, a restart, or a change of device. */
    void rearm(uint32_t nowMs, uint64_t blocks) {
        lastBlocks = blocks;
        lastAdvanceMs = nowMs;
        stalled = false;
        inputSilent = false;
    }

    Action tick(const Reading& r) {
        if (r.blocks != lastBlocks) {
            lastBlocks = r.blocks;
            lastAdvanceMs = r.nowMs;
            stalled = false;
        }

        if (r.playing) {
            if (down) { down = false; reopenAttempts = 0; }
            inputSilent = r.hasInputs && r.silentInputSec >= SilentInputSec;
            if (!stalled && r.nowMs - lastAdvanceMs > StallMs) {
                stalled = true;
                ++stallRestarts;
                return Action::RestartStalled;
            }
            return Action::None;
        }

        inputSilent = false;
        if (!down) {
            down = true;
            downSinceMs = r.nowMs;
            lastReopenMs = r.nowMs;
            reopenAttempts = 0;
            // The first try at once: a dropout is often over by the time anybody notices it.
            ++reopenAttempts;
            return Action::Reopen;
        }
        const uint32_t gap = (r.nowMs - downSinceMs) < FastReopenForMs ? FastReopenMs : SlowReopenMs;
        if (r.nowMs - lastReopenMs >= gap) {
            lastReopenMs = r.nowMs;
            ++reopenAttempts;
            return Action::Reopen;
        }
        return Action::None;
    }

    bool isStalled() const { return stalled; }
    bool isDown() const { return down; }
    /** Every input has been exactly zero for `SilentInputSec`: reported, never restarted. */
    bool isInputSilent() const { return inputSilent; }
    int attempts() const { return reopenAttempts; }
    int stalls() const { return stallRestarts; }

private:
    uint64_t lastBlocks { 0 };
    uint32_t lastAdvanceMs { 0 };
    uint32_t downSinceMs { 0 };
    uint32_t lastReopenMs { 0 };
    bool stalled { false };
    bool down { false };
    bool inputSilent { false };
    int reopenAttempts { 0 };
    int stallRestarts { 0 };
};

} // namespace dsp
