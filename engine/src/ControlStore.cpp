#include "ControlStore.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace dsp {

namespace {

constexpr const char* Prefix = "state-";
constexpr const char* Suffix = ".json";
constexpr int RevisionDigits = 10;

/** Nine significant digits round-trips every IEEE-754 binary32 exactly. */
std::string formatValue(float v) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.9g", static_cast<double>(v));
    return buf;
}

bool parseValue(const std::string& text, float& out) {
    if (text.empty()) return false;
    char* end = nullptr;
    const float v = std::strtof(text.c_str(), &end);
    if (end == nullptr || *end != '\0') return false;
    if (!std::isfinite(v)) return false;
    out = v;
    return true;
}

/** What the checksum covers: every entry in address order, value as its stored text. */
juce::String checksumOf(const std::vector<ControlStore::Entry>& sorted) {
    std::string canonical;
    canonical.reserve(sorted.size() * 40);
    for (const auto& e : sorted) {
        canonical += e.address;
        canonical += '=';
        canonical += formatValue(e.value);
        canonical += '\n';
    }
    // FNV-1a, 64-bit. This detects a damaged file — a truncation, a hand edit, a bad sector —
    // and is not a defence against anybody: the files are the user's own, in their own profile.
    // (JUCE's MD5 lives in a module this engine does not otherwise link.)
    juce::uint64 h = 14695981039346656037ull;
    for (const unsigned char c : canonical) {
        h ^= c;
        h *= 1099511628211ull;
    }
    return juce::String::toHexString(static_cast<juce::int64>(h)).paddedLeft('0', 16);
}

juce::String safeVersion(const juce::String& v) {
    return v.retainCharacters("0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ.-+_");
}

} // namespace

ControlStore::ControlStore(juce::File directory, int versionsToKeep)
    : dir(std::move(directory)), keep(std::max(2, versionsToKeep)) {}

bool ControlStore::addressOk(std::string_view a) {
    if (a.empty() || a.size() > MaxAddress) return false;
    for (const char c : a) {
        const bool ok = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')
                        || c == '/' || c == '_' || c == '.' || c == '-';
        if (!ok) return false;
    }
    return true;
}

juce::String ControlStore::fileNameFor(uint64_t revision) {
    return juce::String(Prefix) + juce::String(static_cast<juce::int64>(revision)).paddedLeft('0', RevisionDigits) + Suffix;
}

uint64_t ControlStore::revisionOf(const juce::String& name) {
    const juce::String prefix(Prefix), suffix(Suffix);
    if (!name.startsWith(prefix) || !name.endsWith(suffix)) return 0;
    const auto digits = name.substring(prefix.length(), name.length() - suffix.length());
    if (digits.length() != RevisionDigits || !digits.containsOnly("0123456789")) return 0;
    return static_cast<uint64_t>(digits.getLargeIntValue());
}

juce::Array<juce::File> ControlStore::versions() const {
    juce::Array<juce::File> found;
    if (!dir.isDirectory()) return found;
    for (const auto& f : dir.findChildFiles(juce::File::findFiles, false, "state-*.json")) {
        if (revisionOf(f.getFileName()) > 0) found.add(f);
    }
    // Newest first. The name carries the revision, so this needs no clock.
    std::sort(found.begin(), found.end(), [](const juce::File& a, const juce::File& b) {
        return revisionOf(a.getFileName()) > revisionOf(b.getFileName());
    });
    return found;
}

std::vector<std::string> ControlStore::chunk(const std::vector<Entry>& entries, size_t maxBytes) {
    std::vector<std::string> parts;
    std::string current;
    for (const auto& e : entries) {
        std::string line = e.address;
        line += '=';
        line += formatValue(e.value);
        line += '\n';
        if (!current.empty() && current.size() + line.size() > maxBytes) {
            parts.push_back(std::move(current));
            current.clear();
        }
        current += line;
    }
    if (!current.empty()) parts.push_back(std::move(current));
    return parts;
}

std::string ControlStore::serialise(const std::vector<Entry>& sorted, uint64_t revision, const juce::String& engineVersion) {
    std::string out;
    out.reserve(sorted.size() * 48 + 256);
    out += "{\"schema\":" + std::to_string(SchemaVersion);
    out += ",\"revision\":" + std::to_string(revision);
    out += ",\"savedAt\":\"" + juce::Time::getCurrentTime().toISO8601(true).toStdString() + "\"";
    out += ",\"engineVersion\":\"" + safeVersion(engineVersion).toStdString() + "\"";
    out += ",\"count\":" + std::to_string(sorted.size());
    out += ",\"checksum\":\"" + checksumOf(sorted).toStdString() + "\"";
    out += ",\"controls\":{";
    bool first = true;
    for (const auto& e : sorted) {
        if (!first) out += ',';
        first = false;
        // Addresses are restricted to characters that need no escaping; see `addressOk`.
        out += '"' + e.address + "\":\"" + formatValue(e.value) + '"';
    }
    out += "}}\n";
    return out;
}

juce::String ControlStore::parse(const juce::String& text, std::vector<Entry>& out, uint64_t& revision) {
    out.clear();
    if (text.trim().isEmpty()) return "empty file";

    juce::var root;
    const auto parsed = juce::JSON::parse(text, root);
    if (parsed.failed()) return "not JSON (" + parsed.getErrorMessage() + ")";
    auto* object = root.getDynamicObject();
    if (object == nullptr) return "not a JSON object";

    if (static_cast<int>(object->getProperty("schema")) != SchemaVersion) return "unknown schema";
    const auto rev = static_cast<juce::int64>(object->getProperty("revision"));
    if (rev <= 0) return "no revision";
    const auto checksum = object->getProperty("checksum").toString();
    const auto count = static_cast<juce::int64>(object->getProperty("count"));

    auto* controls = object->getProperty("controls").getDynamicObject();
    if (controls == nullptr) return "no controls";

    for (const auto& property : controls->getProperties()) {
        const std::string address = property.name.toString().toStdString();
        if (!addressOk(address)) return "bad address \"" + juce::String(address).substring(0, 60) + "\"";
        if (!property.value.isString()) return "value of " + juce::String(address) + " is not text";
        float v = 0.0f;
        if (!parseValue(property.value.toString().toStdString(), v)) return "value of " + juce::String(address) + " is not a number";
        out.push_back({ address, v });
    }
    std::sort(out.begin(), out.end(), [](const Entry& a, const Entry& b) { return a.address < b.address; });
    for (size_t i = 1; i < out.size(); ++i)
        if (out[i].address == out[i - 1].address) return "duplicate address";

    if (static_cast<juce::int64>(out.size()) != count) return "count does not match";
    if (checksumOf(out) != checksum) return "checksum does not match";
    revision = static_cast<uint64_t>(rev);
    return {};
}

bool ControlStore::load(Loaded& out) {
    out = {};
    uint64_t highest = 0;
    bool ok = false;
    for (const auto& file : versions()) {
        const uint64_t named = revisionOf(file.getFileName());
        highest = std::max(highest, named);
        if (ok) continue; // only counting names now, so the next save is above everything on disk
        std::vector<Entry> entries;
        uint64_t revision = 0;
        const auto text = file.loadFileAsString();
        auto why = parse(text, entries, revision);
        if (why.isEmpty() && revision != named) why = "file name and revision disagree";
        if (why.isNotEmpty()) {
            out.skipped.add(file.getFileName() + ": " + why);
            continue;
        }
        out.controls = std::move(entries);
        out.revision = revision;
        out.file = file.getFileName();
        ok = true;
    }

    const std::lock_guard<std::mutex> guard(lock);
    values.clear();
    for (const auto& e : out.controls) values[e.address] = e.value;
    rev = std::max(highest, out.revision);
    generation = 0;
    savedGeneration = 0;
    firstChangedAt = 0;
    return ok;
}

bool ControlStore::record(std::string_view address, float value) {
    if (!address.empty() && address.front() == '/') address.remove_prefix(1);
    if (!addressOk(address) || !std::isfinite(value)) return false;

    const std::lock_guard<std::mutex> guard(lock);
    auto [it, inserted] = values.try_emplace(std::string(address), value);
    if (!inserted) {
        if (it->second == value) return true; // the same thing said twice is not a change
        it->second = value;
    }
    const bool wasClean = generation == savedGeneration;
    ++generation;
    changedAt = juce::Time::getMillisecondCounter();
    if (wasClean) firstChangedAt = changedAt;
    return true;
}

bool ControlStore::dirty() const {
    const std::lock_guard<std::mutex> guard(lock);
    return generation != savedGeneration;
}

juce::uint32 ControlStore::lastChangeMs() const {
    const std::lock_guard<std::mutex> guard(lock);
    return changedAt;
}

juce::uint32 ControlStore::dirtySinceMs() const {
    const std::lock_guard<std::mutex> guard(lock);
    return generation != savedGeneration ? firstChangedAt : 0;
}

uint64_t ControlStore::revision() const {
    const std::lock_guard<std::mutex> guard(lock);
    return rev;
}

size_t ControlStore::size() const {
    const std::lock_guard<std::mutex> guard(lock);
    return values.size();
}

std::vector<ControlStore::Entry> ControlStore::snapshot() const {
    const std::lock_guard<std::mutex> guard(lock);
    std::vector<Entry> out;
    out.reserve(values.size());
    for (const auto& [address, value] : values) out.push_back({ address, value }); // std::map: already sorted
    return out;
}

juce::String ControlStore::lastError() const {
    const std::lock_guard<std::mutex> guard(lock);
    return error;
}

bool ControlStore::save() {
    const std::lock_guard<std::mutex> serial(saveLock);

    std::vector<Entry> entries;
    uint64_t next = 0;
    uint64_t gen = 0;
    {
        const std::lock_guard<std::mutex> guard(lock);
        if (generation == savedGeneration) return false;
        entries.reserve(values.size());
        for (const auto& [address, value] : values) entries.push_back({ address, value });
        next = rev + 1;
        gen = generation;
    }

    auto fail = [this](const juce::String& why) {
        const std::lock_guard<std::mutex> guard(lock);
        error = why;
        return false;
    };

    if (!dir.createDirectory()) return fail("cannot create " + dir.getFullPathName());
    const auto target = dir.getChildFile(fileNameFor(next));
    const std::string text = serialise(entries, next, engineVersion);

    {
        // Written beside the target, flushed to the disk, then swapped in whole. A reader — or a
        // restart after a crash — sees the old version or the new one, never the middle.
        juce::TemporaryFile temp(target);
        {
            std::unique_ptr<juce::FileOutputStream> stream(temp.getFile().createOutputStream());
            if (stream == nullptr || stream->failedToOpen()) return fail("cannot write " + temp.getFile().getFullPathName());
            if (!stream->write(text.data(), text.size())) return fail("write failed: " + stream->getStatus().getErrorMessage());
            stream->flush();
        }
        if (!temp.overwriteTargetFileWithTemporary()) return fail("cannot replace " + target.getFullPathName());
    }

    {
        const std::lock_guard<std::mutex> guard(lock);
        rev = next;
        savedGeneration = gen;
        if (generation == savedGeneration) firstChangedAt = 0;
        error.clear();
    }

    // Old versions beyond the ones worth keeping. A failure to delete is not a failure to save.
    const auto all = versions();
    for (int i = keep; i < all.size(); ++i) all.getReference(i).deleteFile();
    return true;
}

} // namespace dsp
