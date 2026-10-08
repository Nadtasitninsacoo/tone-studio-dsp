#include "ControlStore.h"
#include "MixingEngine.h"
#include <juce_core/juce_core.h>
#include <algorithm>
#include <atomic>
#include <functional>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <thread>
#include <vector>
#include "TestHelpers.h"

extern thread_local bool allowAllocations;
extern thread_local const char* rtSection;

/**
 * The engine's saved state: that what the DSP accepted is what gets written, that a restart
 * brings back exactly it, and that a bad file — truncated, edited, empty, half-written — never
 * takes the show down with it.
 *
 * The one that matters most is the equivalence test: an engine rebuilt from disk must produce
 * the same samples as the engine that was running, bit for bit. A "close enough" there would be
 * a mix that comes back slightly different after every restart.
 */
class ControlStoreTests : public juce::UnitTest {
public:
    ControlStoreTests() : juce::UnitTest("ControlStore") {}

    struct TempDir {
        juce::File dir;
        TempDir() {
            dir = juce::File::getSpecialLocation(juce::File::tempDirectory)
                      .getChildFile("tsd-store-test-" + juce::String(juce::Random::getSystemRandom().nextInt64()));
            dir.createDirectory();
        }
        ~TempDir() { dir.deleteRecursively(); }
    };

    static uint32_t bitsOf(float v) {
        uint32_t b = 0;
        std::memcpy(&b, &v, sizeof b);
        return b;
    }

    /** Write a file the way a version is written, then let the test damage it. */
    static juce::File writeVersion(const juce::File& dir, uint64_t revision, const std::vector<dsp::ControlStore::Entry>& entries) {
        dir.createDirectory();
        auto sorted = entries;
        std::sort(sorted.begin(), sorted.end(), [](auto& a, auto& b) { return a.address < b.address; });
        const auto file = dir.getChildFile(dsp::ControlStore::fileNameFor(revision));
        file.replaceWithText(juce::String(dsp::ControlStore::serialise(sorted, revision, "test")));
        return file;
    }

    void runTest() override {
        // ---- the names ---------------------------------------------------------------------------
        beginTest("a version has one name, and nothing else is a version");
        {
            expectEquals(dsp::ControlStore::fileNameFor(12), juce::String("state-0000000012.json"));
            expect(dsp::ControlStore::revisionOf("state-0000000012.json") == 12);
            expect(dsp::ControlStore::revisionOf("state-0000000012_temp345.json") == 0, "a temporary file is not a version");
            expect(dsp::ControlStore::revisionOf("state-12.json") == 0);
            expect(dsp::ControlStore::revisionOf("state-0000000012.json.tmp") == 0);
            expect(dsp::ControlStore::revisionOf("devices.json") == 0);
            expect(dsp::ControlStore::revisionOf("state-00000000x2.json") == 0);
        }

        // ---- what is accepted ------------------------------------------------------------------
        beginTest("only an address the wire can carry, with a finite value, is recorded");
        {
            TempDir t;
            dsp::ControlStore s(t.dir);
            expect(s.record("channel/1/fader", -3.0f));
            expect(s.record("/channel/2/fader", 1.0f), "a leading slash is normalised away");
            expect(!s.record("", 1.0f));
            expect(!s.record(std::string(48, 'a'), 1.0f), "one past the DSP's own address limit");
            expect(s.record(std::string(47, 'a'), 1.0f), "exactly at it");
            expect(!s.record("has space", 1.0f));
            expect(!s.record("bad\"quote", 1.0f));
            expect(!s.record("bad\\slash", 1.0f));
            expect(!s.record("channel/1/fader", std::numeric_limits<float>::quiet_NaN()));
            expect(!s.record("channel/1/fader", std::numeric_limits<float>::infinity()));
            expectEquals((int) s.size(), 3);
            const auto snap = s.snapshot();
            expect(snap.size() == 3 && snap[0].address < snap[1].address, "snapshot is in address order");
            expect(std::any_of(snap.begin(), snap.end(), [](auto& e) { return e.address == "channel/2/fader"; }), "stored without the slash");
        }

        beginTest("last value wins, and saying the same thing twice is not a change");
        {
            TempDir t;
            dsp::ControlStore s(t.dir);
            s.record("master/gain", -1.0f);
            expect(s.dirty());
            expect(s.save());
            expect(!s.dirty(), "clean after a save");
            expect(!s.save(), "nothing to write when nothing changed");
            expect(s.record("master/gain", -1.0f));
            expect(!s.dirty(), "the same value again does not dirty it");
            s.record("master/gain", -2.0f);
            expect(s.dirty());
            expectEquals((int) s.size(), 1);
        }

        // ---- the round trip ------------------------------------------------------------------------
        beginTest("every float comes back with the same bits");
        {
            TempDir t;
            std::vector<float> samples = {
                0.0f, -0.0f, 1.0f, -1.0f, 0.1f, 0.3f, 1.0f / 3.0f, 3.14159265f, 1e-30f, -1e-30f, 123456.789f,
                std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest(),
                std::numeric_limits<float>::min(), std::numeric_limits<float>::denorm_min(),
                std::numeric_limits<float>::epsilon(), -60.0f, 12.0f, 20000.0f, 0.707107f,
            };
            juce::Random rng(12345);
            for (int i = 0; i < 2000; ++i) {
                uint32_t b = static_cast<uint32_t>(rng.nextInt64());
                float v;
                std::memcpy(&v, &b, sizeof v);
                if (std::isfinite(v)) samples.push_back(v);
            }
            {
                dsp::ControlStore s(t.dir);
                for (size_t i = 0; i < samples.size(); ++i) s.record("test/" + std::to_string(i), samples[i]);
                expect(s.save());
            }
            dsp::ControlStore again(t.dir);
            dsp::ControlStore::Loaded loaded;
            expect(again.load(loaded));
            expectEquals((int) loaded.controls.size(), (int) samples.size());
            int mismatches = 0;
            for (const auto& e : loaded.controls) {
                const auto index = static_cast<size_t>(juce::String(e.address).fromLastOccurrenceOf("/", false, false).getIntValue());
                if (bitsOf(e.value) != bitsOf(samples[index])) ++mismatches;
            }
            expectEquals(mismatches, 0, "every value is bit-exact");
        }

        beginTest("a save is a new, higher revision, and only the newest few are kept");
        {
            TempDir t;
            dsp::ControlStore s(t.dir, 5);
            for (int i = 1; i <= 8; ++i) {
                s.record("master/gain", static_cast<float>(i));
                expect(s.save());
                expectEquals((int) s.revision(), i);
            }
            juce::StringArray names;
            for (const auto& f : t.dir.findChildFiles(juce::File::findFiles, false, "*")) names.add(f.getFileName());
            names.sort(false);
            expectEquals(names.size(), 5, "five versions kept: " + names.joinIntoString(" "));
            expectEquals(names[0], juce::String("state-0000000004.json"));
            expectEquals(names[4], juce::String("state-0000000008.json"));
        }

        beginTest("an empty directory is a first run, not an error");
        {
            TempDir t;
            dsp::ControlStore s(t.dir);
            dsp::ControlStore::Loaded loaded;
            expect(!s.load(loaded));
            expect(loaded.controls.empty() && loaded.skipped.isEmpty());
            s.record("master/gain", 1.0f);
            expect(s.save());
            expectEquals((int) s.revision(), 1);
        }

        // ---- bad files ------------------------------------------------------------------------------
        beginTest("a damaged newest version is skipped and the one before it is used");
        {
            struct Damage { const char* what; std::function<void(const juce::File&)> apply; };
            const std::vector<Damage> damages = {
                { "truncated", [](const juce::File& f) { auto text = f.loadFileAsString(); f.replaceWithText(text.substring(0, text.length() / 2)); } },
                { "garbage", [](const juce::File& f) { f.replaceWithText("this is not json at all \x01\x02"); } },
                { "empty", [](const juce::File& f) { f.replaceWithText(""); } },
                { "a value edited by hand", [](const juce::File& f) { f.replaceWithText(f.loadFileAsString().replace("\"-3\"", "\"-9\"")); } },
                { "unknown schema", [](const juce::File& f) { f.replaceWithText(f.loadFileAsString().replace("\"schema\":1", "\"schema\":99")); } },
                { "a value that is not a number", [](const juce::File& f) { f.replaceWithText(f.loadFileAsString().replace("\"-3\"", "\"nan\"")); } },
                { "a value that is not a string", [](const juce::File& f) { f.replaceWithText(f.loadFileAsString().replace("\"-3\"", "-3")); } },
                { "an address the DSP could never have accepted", [](const juce::File& f) { f.replaceWithText(f.loadFileAsString().replace("channel/1/fader", "channel/1/fa der")); } },
                { "a count that does not match", [](const juce::File& f) { f.replaceWithText(f.loadFileAsString().replace("\"count\":2", "\"count\":3")); } },
                { "a file name that disagrees with its revision", [](const juce::File& f) { f.replaceWithText(f.loadFileAsString().replace("\"revision\":3", "\"revision\":2")); } },
            };
            for (const auto& damage : damages) {
                TempDir t;
                {
                    dsp::ControlStore s(t.dir);
                    s.record("channel/1/fader", -1.0f); s.record("master/gain", 0.5f); s.save();
                    s.record("channel/1/fader", -2.0f); s.save();
                    s.record("channel/1/fader", -3.0f); s.save();
                }
                damage.apply(t.dir.getChildFile("state-0000000003.json"));
                dsp::ControlStore s(t.dir);
                dsp::ControlStore::Loaded loaded;
                expect(s.load(loaded), juce::String(damage.what) + ": fell back instead of giving up");
                expectEquals((int) loaded.revision, 2, juce::String(damage.what) + ": used the version before");
                expectEquals(loaded.skipped.size(), 1, juce::String(damage.what) + ": said what it refused");
                float fader = 0.0f;
                for (auto& e : loaded.controls) if (e.address == "channel/1/fader") fader = e.value;
                expectEquals(fader, -2.0f, juce::String(damage.what) + ": and the value is the older one");
                s.record("channel/1/fader", -4.0f);
                expect(s.save());
                expectEquals((int) s.revision(), 4, juce::String(damage.what) + ": the next save is above the damaged file");
            }
        }

        beginTest("every version damaged: nothing loads, the engine starts from defaults, and says so");
        {
            TempDir t;
            for (int i = 1; i <= 3; ++i) t.dir.getChildFile(dsp::ControlStore::fileNameFor((uint64_t) i)).replaceWithText("{broken");
            dsp::ControlStore s(t.dir);
            dsp::ControlStore::Loaded loaded;
            expect(!s.load(loaded));
            expectEquals(loaded.skipped.size(), 3);
            s.record("master/gain", 1.0f);
            expect(s.save());
            expectEquals((int) s.revision(), 4, "above every file on disk, damaged ones included");
        }

        beginTest("a half-written temporary file from a crash is ignored");
        {
            TempDir t;
            {
                dsp::ControlStore s(t.dir);
                s.record("master/gain", 0.25f); s.save();
            }
            t.dir.getChildFile("state-0000000002_temp9876.json").replaceWithText("{\"schema\":1,\"revi");
            t.dir.getChildFile("state-0000000002.json.tmp").replaceWithText("{");
            dsp::ControlStore s(t.dir);
            dsp::ControlStore::Loaded loaded;
            expect(s.load(loaded));
            expectEquals((int) loaded.revision, 1);
            expect(loaded.skipped.isEmpty(), "a temporary file is not even a candidate");
        }

        beginTest("a save that cannot be written fails quietly, keeps the old version, and stays dirty");
        {
            TempDir t;
            dsp::ControlStore good(t.dir);
            good.record("master/gain", 1.0f);
            expect(good.save());
            // A *file* where the directory should be.
            const auto blocker = t.dir.getChildFile("blocked");
            blocker.replaceWithText("not a directory");
            dsp::ControlStore bad(blocker);
            bad.record("master/gain", 2.0f);
            expect(!bad.save());
            expect(bad.dirty(), "still dirty, so the next try writes it");
            expect(bad.lastError().isNotEmpty(), "and says why");
        }

        // ---- threads ------------------------------------------------------------------------------
        beginTest("recording from another thread while saving loses nothing and corrupts nothing");
        {
            TempDir t;
            dsp::ControlStore s(t.dir);
            std::atomic<bool> stop { false };
            std::thread writer([&] {
                for (int round = 1; round <= 200 && !stop.load(); ++round)
                    for (int a = 0; a < 40; ++a) s.record("channel/" + std::to_string(a + 1) + "/fader", static_cast<float>(round));
            });
            int saves = 0;
            for (int i = 0; i < 40; ++i) { if (s.save()) ++saves; juce::Thread::sleep(1); }
            writer.join();
            expect(s.save() || !s.dirty());
            dsp::ControlStore again(t.dir);
            dsp::ControlStore::Loaded loaded;
            expect(again.load(loaded), "the newest file after the race is whole");
            expect(loaded.skipped.isEmpty(), "no version was torn");
            expectEquals((int) loaded.controls.size(), 40);
            for (auto& e : loaded.controls) expectEquals(e.value, 200.0f, "the last value won on " + juce::String(e.address));
            expect(saves >= 1);
        }

        beginTest("a state is cut into whole-line chunks that put the same state back together");
        {
            std::vector<dsp::ControlStore::Entry> entries;
            for (int i = 0; i < 3200; ++i)
                entries.push_back({ "channel/" + std::to_string(1 + i % 32) + "/eq/" + std::to_string(1 + i % 4) + "/gain/" + std::to_string(i),
                                    static_cast<float>(i % 241 - 120) / 10.0f });
            for (const size_t limit : { size_t(200), size_t(4000), size_t(16000), size_t(100000) }) {
                const auto parts = dsp::ControlStore::chunk(entries, limit);
                std::vector<dsp::ControlStore::Entry> back;
                bool tooBig = false, notWhole = false;
                for (const auto& p : parts) {
                    if (p.size() > limit) tooBig = true;
                    if (p.empty() || p.back() != '\n') notWhole = true;
                    juce::StringArray lines;
                    lines.addLines(juce::String(p));
                    for (const auto& line : lines) {
                        if (line.isEmpty()) continue;
                        const auto at = line.lastIndexOfChar('=');
                        back.push_back({ line.substring(0, at).toStdString(), line.substring(at + 1).getFloatValue() });
                    }
                }
                expect(!tooBig, "no chunk over " + juce::String((int) limit));
                expect(!notWhole, "every chunk is whole lines");
                expectEquals((int) back.size(), (int) entries.size(), "nothing lost at limit " + juce::String((int) limit));
                bool same = true;
                for (size_t i = 0; i < back.size() && i < entries.size(); ++i)
                    same = same && back[i].address == entries[i].address && bitsOf(back[i].value) == bitsOf(entries[i].value);
                expect(same, "same entries, same order, same bits at limit " + juce::String((int) limit));
            }
            expect(dsp::ControlStore::chunk({}, 1000).empty(), "an empty state is no chunks");
        }

        // ---- the journal between the audio thread and the disk ----------------------------------
        beginTest("the engine journals what it applied, in order, and nothing it refused");
        {
            auto e = std::make_unique<dsp::MixingEngine>();
            e->prepare(48000.0, 256);
            expect(e->postControl("channel/1/fader", -6.0f));
            expect(e->postControl("channel/99/fader", -6.0f), "queued: the queue does not know");
            expect(e->postControl("nonsense/path", 1.0f));
            expect(e->postControl("channel/1/fader", std::numeric_limits<float>::quiet_NaN()));
            expect(e->postControl("master/gain", -2.0f));
            expect(e->postControl("channel/1/fader", -4.0f));
            e->serviceControls();
            dsp::MixingEngine::ControlRecord out[16];
            const int n = e->drainApplied(out, 16);
            expectEquals(n, 3, "three were accepted");
            expectEquals(juce::String(out[0].address), juce::String("channel/1/fader"));
            expectEquals(out[0].value, -6.0f);
            expectEquals(juce::String(out[1].address), juce::String("master/gain"));
            expectEquals(juce::String(out[2].address), juce::String("channel/1/fader"));
            expectEquals(out[2].value, -4.0f, "in the order applied, so the last value wins downstream");
            expectEquals(e->rejectedControls(), 3);
            expectEquals(e->drainApplied(out, 16), 0, "drained once");
        }

        beginTest("a full journal drops the newest and counts it; the audio thread never waits");
        {
            auto e = std::make_unique<dsp::MixingEngine>();
            e->prepare(48000.0, 256);
            int posted = 0;
            for (int i = 0; i < 4000; ++i) posted += e->postControl("channel/1/fader", static_cast<float>(-(i % 50))) ? 1 : 0;
            expectEquals(posted, 4000);
            e->serviceControls();
            expectEquals(e->lostApplied(), 0);
            for (int i = 0; i < 300; ++i) e->postControl("channel/2/fader", -1.0f);
            e->serviceControls();
            // A JUCE AbstractFifo holds one fewer than its size.
            const int capacity = dsp::MixingEngine::AppliedQueueSize - 1;
            expectEquals(e->lostApplied(), 4000 + 300 - capacity);
            std::vector<dsp::MixingEngine::ControlRecord> out(5000);
            expectEquals(e->drainApplied(out.data(), 5000), capacity);
        }

        beginTest("journaling allocates nothing on the audio thread");
        {
            auto e = std::make_unique<dsp::MixingEngine>();
            e->prepare(48000.0, 256);
            for (int i = 0; i < 64; ++i) e->postControl("channel/" + std::to_string(1 + i % 8) + "/fader", -1.0f * (float) (i % 7));
            std::vector<float> in(256, 0.0f);
            std::vector<std::vector<float>> out(2, std::vector<float>(256));
            const float* ip[1] = { in.data() };
            float* op[2] = { out[0].data(), out[1].data() };
            allowAllocations = false;
            rtSection = "journalApplied";
            e->processAudio(ip, 1, op, 2, 256);
            allowAllocations = true;
            dsp::MixingEngine::ControlRecord rec[128];
            expect(e->drainApplied(rec, 128) >= 8);
        }

        // ---- the whole point -------------------------------------------------------------------------
        beginTest("an engine rebuilt from disk makes the same samples as the one that was running");
        {
            TempDir t;
            const std::vector<std::pair<std::string, float>> mix = {
                { "channel/1/input", 0.0f }, { "channel/2/input", 1.0f },
                { "channel/1/fader", -10.0f }, { "channel/1/fader", -4.5f }, // overwritten: the last one counts
                { "channel/1/trim", 2.0f }, { "channel/1/pan", -0.3f },
                { "channel/1/eq/1/gain", 3.0f }, { "channel/1/eq/2/gain", -4.0f }, { "channel/1/eq/2/freq", 800.0f },
                { "channel/1/comp/enabled", 1.0f }, { "channel/1/comp/threshold", -22.0f }, { "channel/1/comp/ratio", 4.0f },
                { "channel/1/hpf/enabled", 1.0f }, { "channel/1/hpf/freq", 120.0f },
                { "channel/1/gate/enabled", 1.0f }, { "channel/1/gate/threshold", -60.0f },
                { "channel/2/fader", -7.0f }, { "channel/2/pan", 0.6f }, { "channel/2/invert", 1.0f },
                { "channel/2/dca", 1.0f }, { "dca/1/gain", -3.0f },
                { "channel/2/send/aux/1", -6.0f }, { "aux/1/gain", -2.0f }, { "aux/1/eq/high", 3.0f },
                { "master/gain", -1.5f }, { "master/limiter/enabled", 1.0f }, { "master/limiter/ceiling", -1.0f },
                { "master/geq/5", 2.5f }, { "master/geq/20", -3.0f },
                { "master/fx/reverb/enabled", 1.0f }, { "master/fx/reverb/wet", 0.25f }, { "master/fx/reverb/room", 0.6f },
                { "master/fx/delay/enabled", 1.0f }, { "master/fx/delay/time", 380.0f }, { "master/fx/delay/feedback", 0.35f },
                { "output/1/delay", 4.0f }, { "output/1/speaker/hpf/type", 1.0f }, { "output/1/speaker/hpf/freq", 60.0f },
                { "output/2/source", 1.0f },
            };

            auto running = std::make_unique<dsp::MixingEngine>();
            running->prepare(48000.0, 256);
            for (const auto& [address, value] : mix) running->postControl(address, value);
            running->serviceControls();

            int accepted = 0;
            {
                dsp::ControlStore store(t.dir);
                dsp::MixingEngine::ControlRecord rec[256];
                const int n = running->drainApplied(rec, 256);
                for (int i = 0; i < n; ++i) { store.record(rec[i].address, rec[i].value); ++accepted; }
                expect(store.save());
            }
            expect(accepted >= 35, "the table accepted what was posted: " + juce::String(accepted) + " of " + juce::String((int) mix.size()));
            {
                // Name the refused ones, so a typo in this table is a sentence and not a number.
                auto probe = std::make_unique<dsp::MixingEngine>();
                probe->prepare(48000.0, 256);
                juce::StringArray refusedNames;
                for (const auto& [address, value] : mix) if (!probe->setControl(address, value)) refusedNames.add(juce::String(address));
                expect(refusedNames.isEmpty(), "addresses the engine refuses: " + refusedNames.joinIntoString(", "));
            }
            expectEquals(running->rejectedControls(), 0, "every address in this test is real");

            // A new process: nothing but the files.
            dsp::ControlStore restored(t.dir);
            dsp::ControlStore::Loaded loaded;
            expect(restored.load(loaded));
            auto rebuilt = std::make_unique<dsp::MixingEngine>();
            rebuilt->prepare(48000.0, 256);
            int refused = 0;
            for (const auto& entry : loaded.controls) if (!rebuilt->setControl(entry.address, entry.value)) ++refused;
            expectEquals(refused, 0, "the engine accepts everything it saved");

            // Same audio through both.
            constexpr int block = 256;
            std::vector<float> in0(block), in1(block);
            std::vector<std::vector<float>> a(2, std::vector<float>(block)), b(2, std::vector<float>(block));
            double worst = 0.0;
            int t0 = 0;
            for (int blockIndex = 0; blockIndex < 60; ++blockIndex) {
                for (int i = 0; i < block; ++i, ++t0) {
                    in0[(size_t) i] = 0.2f * std::sin(2.0f * 3.14159265f * 440.0f * (float) t0 / 48000.0f);
                    in1[(size_t) i] = 0.1f * std::sin(2.0f * 3.14159265f * 1500.0f * (float) t0 / 48000.0f);
                }
                const float* ip[2] = { in0.data(), in1.data() };
                float* oa[2] = { a[0].data(), a[1].data() };
                float* ob[2] = { b[0].data(), b[1].data() };
                running->processAudio(ip, 2, oa, 2, block);
                rebuilt->processAudio(ip, 2, ob, 2, block);
                for (int c = 0; c < 2; ++c)
                    for (int i = 0; i < block; ++i)
                        worst = std::max(worst, (double) std::abs(a[(size_t) c][(size_t) i] - b[(size_t) c][(size_t) i]));
            }
            expectEquals(worst, 0.0, "the rebuilt engine is bit-identical to the one that was running (worst difference " + juce::String(worst) + ")");
        }
    }
};

static ControlStoreTests controlStoreTests;
