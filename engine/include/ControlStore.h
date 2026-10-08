#pragma once

#include <juce_core/juce_core.h>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

namespace dsp {

/**
 * The control plane's state, kept on disk — so the engine can play on its own.
 *
 * ---------------------------------------------------------------------------
 * WHY THIS EXISTS
 *
 * The whole control plane is `address -> float`: `channel/3/eq/2/gain -3.5`, `master/fx/delay/time
 * 380`, `output/2/speaker/hpf/freq 80`. Every one of those arrives over OSC from a web page, and
 * the engine kept none of them anywhere but in its DSP objects. So the page was the only place
 * the mix existed: close it and the engine plays on (it never sees the socket — it sees UDP),
 * but **restart the engine and every fader, EQ and patch is gone** until a page that still has
 * them sends them again. A show whose settings live in a browser tab is a show that one crashed
 * tab can end.
 *
 * Recording the *last value per address* is enough to rebuild all of it, because replaying the
 * addresses through the same routing table (`MixingEngine::setControl`) is exactly what the page
 * did to get there. No parameter is named in this file, and a parameter added to the table later
 * is persisted with no change here.
 *
 * WHAT IS KEPT, AND WHAT IS NOT
 *
 *  - Only controls the DSP **accepted**. `MixingEngine` journals a control from the audio thread
 *    only after `setControl` returned true, so a typo from a buggy client, an out-of-range
 *    channel or a NaN never becomes saved state.
 *  - Last value wins. A fader dragged through two hundred positions is one entry.
 *  - Not the audio device (`devices.json` already holds that), not recordings, not meters.
 *
 * HOW IT SURVIVES A BAD MOMENT
 *
 *  - **Atomic.** Each version is written to a temporary file, flushed to the disk, and then
 *    swapped in (`ReplaceFile` on Windows). A power cut or a `kill -9` leaves the previous version
 *    whole, never half of the new one.
 *  - **Versioned.** The newest N files are kept (`state-0000000012.json`), each carrying its own
 *    checksum. A file that does not parse, fails its checksum, has the wrong schema or holds a
 *    value that is not a number is *skipped*, and the loader falls back to the one before it.
 *  - **Monotonic revision.** Every save has a higher revision than anything on disk, a damaged
 *    file included, so "which is newer" never needs a clock.
 *  - **Never on the audio thread.** This class allocates and does file I/O. The audio thread only
 *    writes into a lock-free journal; `StateKeeper` in the app drains it on a thread of its own.
 *
 * Values are stored as text with nine significant digits, which round-trips every `float`
 * exactly. A JSON number would go through `double` formatting this library does not promise to
 * make exact, and the checksum would then disagree with the data it covers.
 * ---------------------------------------------------------------------------
 */
class ControlStore {
public:
    static constexpr int SchemaVersion = 1;
    static constexpr int DefaultKeep = 5;
    /** Same ceiling as `MixingEngine::MaxControlAddress`; nothing longer is ever accepted by the DSP. */
    static constexpr size_t MaxAddress = 47;

    struct Entry {
        std::string address;
        float value { 0.0f };
    };

    struct Loaded {
        std::vector<Entry> controls;
        uint64_t revision { 0 };
        juce::String file;
        /** Newer versions that were present but unusable, newest first. */
        juce::StringArray skipped;
    };

    explicit ControlStore(juce::File directory, int versionsToKeep = DefaultKeep);

    /**
     * Read the newest version that is whole, and adopt it as the current state. Returns false
     * when there is none (a first run, or every file damaged) — then the engine starts from its
     * defaults, and `out.skipped` still says what was refused.
     */
    bool load(Loaded& out);

    /**
     * Any thread except the audio thread. Last value per address wins. Returns false, and
     * records nothing, for an address that is empty, too long, or has a character the wire never
     * carries — and for a value that is not finite.
     */
    bool record(std::string_view address, float value);

    /** Whether something changed since the last successful `save`. */
    bool dirty() const;
    /** `juce::Time::getMillisecondCounter()` at the last `record` that changed something. */
    juce::uint32 lastChangeMs() const;
    /** `juce::Time::getMillisecondCounter()` when the first unsaved change was recorded; 0 if none. */
    juce::uint32 dirtySinceMs() const;

    /**
     * Write a new version if anything changed. Returns true if one was written. A failure (a full
     * disk, a locked file) is kept in `lastError()` and leaves the previous version untouched —
     * the state stays dirty, so the next call tries again.
     */
    bool save();

    uint64_t revision() const;
    size_t size() const;
    std::vector<Entry> snapshot() const;
    juce::File directory() const { return dir; }
    juce::String lastError() const;

    /** `state-0000000012.json` — the one name a version may have. Anything else is ignored. */
    static juce::String fileNameFor(uint64_t revision);
    /** The revision a file name carries, or 0 if it is not a version's name. */
    static uint64_t revisionOf(const juce::String& fileName);
    /** The whole-file text for a given state; exposed so tests can build files, good and bad. */
    static std::string serialise(const std::vector<Entry>& sorted, uint64_t revision, const juce::String& engineVersion);
    /** Parse and verify one file's text. Empty string on success, the reason otherwise. */
    static juce::String parse(const juce::String& text, std::vector<Entry>& out, uint64_t& revision);

    void setEngineVersion(juce::String v) { engineVersion = std::move(v); }

    /**
     * Split a state into `address=value\n` chunks of at most `maxBytes` each, for sending over
     * OSC/UDP — a mix does not fit one datagram (a full desk is ~100 KB, a datagram 64 KB). Whole
     * lines only, in the order given, values in the same nine-digit form the files use. A line is
     * never split, so every chunk parses on its own.
     */
    static std::vector<std::string> chunk(const std::vector<Entry>& entries, size_t maxBytes);

private:
    juce::Array<juce::File> versions() const;
    static bool addressOk(std::string_view address);

    juce::File dir;
    int keep;
    juce::String engineVersion { "unknown" };

    /** Held for the length of a `save`, so two saves never interleave their file operations. */
    std::mutex saveLock;
    mutable std::mutex lock;
    std::map<std::string, float> values;
    uint64_t rev { 0 };
    /** Bumped on every real change; a save marks the generation it covered. */
    uint64_t generation { 0 };
    uint64_t savedGeneration { 0 };
    juce::uint32 changedAt { 0 };
    juce::uint32 firstChangedAt { 0 };
    juce::String error;
};

} // namespace dsp
