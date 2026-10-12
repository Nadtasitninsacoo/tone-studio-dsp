#include "MixingEngine.h"
#include "ControlValue.h"
#include "TestHelpers.h"
#include <juce_core/juce_core.h>
#include <cmath>
#include <limits>
#include <random>
#include <string>
#include <vector>

/**
 * The engine on a stage: what happens when something it is handed is wrong.
 *
 * Every test here answers "does a bad input cost a block or the show?" — a NaN from a driver, a
 * control message with no number in it, a number too big for an int, a stream of garbage from a
 * socket. Each one was a real path to silence, a stuck setting or undefined behaviour before this
 * file existed; the comment on each test names which.
 */
class HardeningTests : public juce::UnitTest
{
public:
    HardeningTests() : juce::UnitTest ("Hardening") {}

    static constexpr double kRate = 48000.0;
    static constexpr int kBlock = 256;

    struct Outputs
    {
        Outputs (int count, int frames) : storage ((size_t) count, std::vector<float> ((size_t) frames, 0.0f))
        {
            for (auto& v : storage) ptrs.push_back (v.data());
        }
        std::vector<std::vector<float>> storage;
        std::vector<float*> ptrs;
        float** data() { return ptrs.data(); }
    };

    static bool allFinite (const std::vector<float>& v)
    {
        for (float x : v) if (! std::isfinite (x)) return false;
        return true;
    }
    static float rms (const std::vector<float>& v)
    {
        double s = 0.0;
        for (float x : v) s += (double) x * x;
        return v.empty() ? 0.0f : (float) std::sqrt (s / (double) v.size());
    }
    static void sine (std::vector<float>& v, int block, float hz = 440.0f, float amp = 0.3f)
    {
        for (int i = 0; i < (int) v.size(); ++i)
            v[(size_t) i] = amp * (float) std::sin (2.0 * juce::MathConstants<double>::pi * hz
                                                    * (double) (block * (int) v.size() + i) / kRate);
    }

    void runTest() override
    {
        beginTest ("A NaN on an input costs one block of that channel, not the show");
        {
            // Before: the NaN went through the channel into the master GEQ and the limiter, whose
            // filter and gain states then held it — every output NaN (or silence, depending on
            // the driver) until the device was restarted. One bad sample from a USB driver
            // ended the show.
            auto engine = std::make_unique<dsp::MixingEngine>();
            engine->prepare (kRate, kBlock);
            expect (engine->setControl ("channel/1/hpf/enabled", 1.0f));
            expect (engine->setControl ("channel/1/eq/1/enabled", 1.0f));
            expect (engine->setControl ("channel/1/eq/1/gain", 6.0f));

            std::vector<float> in ((size_t) kBlock);
            const float* inPtrs[1] = { in.data() };
            Outputs out (2, kBlock);

            sine (in, 0);
            in[10] = std::numeric_limits<float>::quiet_NaN();
            engine->processAudio (inPtrs, 1, out.data(), 2, kBlock);
            expect (allFinite (out.storage[0]) && allFinite (out.storage[1]),
                    "even the block that carried the NaN reaches the outputs finite");

            for (int b = 1; b < 40; ++b) { sine (in, b); engine->processAudio (inPtrs, 1, out.data(), 2, kBlock); }
            expect (allFinite (out.storage[0]) && allFinite (out.storage[1]), "outputs stay finite");
            expect (rms (out.storage[0]) > 0.02f, "and the channel is playing again, "
                    "rms " + juce::String (rms (out.storage[0])));
            expect (engine->nonFiniteResets() >= 1, "the reset is counted, so it can be reported");

            // An infinity is the same fault.
            sine (in, 40);
            in[3] = std::numeric_limits<float>::infinity();
            engine->processAudio (inPtrs, 1, out.data(), 2, kBlock);
            for (int b = 41; b < 80; ++b) { sine (in, b); engine->processAudio (inPtrs, 1, out.data(), 2, kBlock); }
            expect (allFinite (out.storage[0]) && rms (out.storage[0]) > 0.02f, "an infinity is survived the same way");
        }

        beginTest ("A silent desk produces no subnormal floats");
        {
            // Every IIR filter decays through subnormals after a signal stops, and x86 handles
            // those up to a hundred times slower: the CPU of a silent desk climbs until a block
            // misses its deadline. The callback flushes them to zero for its whole duration.
            auto engine = std::make_unique<dsp::MixingEngine>();
            engine->prepare (kRate, kBlock);
            for (int c = 1; c <= 4; ++c)
            {
                const auto ch = "channel/" + std::to_string (c) + "/";
                engine->setControl (ch + "hpf/enabled", 1.0f);
                engine->setControl (ch + "eq/2/enabled", 1.0f);
                engine->setControl (ch + "eq/2/gain", 9.0f);
                engine->setControl (ch + "eq/2/freq", 60.0f);
                engine->setControl (ch + "eq/2/q", 8.0f);
            }
            std::vector<float> in ((size_t) kBlock, 0.0f);
            const float* inPtrs[4] = { in.data(), in.data(), in.data(), in.data() };
            Outputs out (2, kBlock);

            in[0] = 0.8f; // one impulse, then silence for ~10 s
            int subnormal = 0;
            for (int b = 0; b < 2000; ++b)
            {
                engine->processAudio (inPtrs, 4, out.data(), 2, kBlock);
                if (b == 0) std::fill (in.begin(), in.end(), 0.0f);
                for (int o = 0; o < 2; ++o)
                    for (float x : out.storage[(size_t) o])
                        if (std::fpclassify (x) == FP_SUBNORMAL) ++subnormal;
                const float* masterL = engine->masterLeft();
                for (int i = 0; i < kBlock; ++i)
                    if (std::fpclassify (masterL[i]) == FP_SUBNORMAL) ++subnormal;
            }
            expectEquals (subnormal, 0, "subnormal samples reached the master or the outputs");
        }

        beginTest ("The suppressor's controls go through the queue, and the action is not kept");
        {
            auto engine = std::make_unique<dsp::MixingEngine>();
            engine->prepare (kRate, kBlock);
            auto& s = engine->getMaster().suppressor;

            expect (engine->postControl ("suppressor/bypass", 0.0f));
            expect (engine->postControl ("suppressor/sensitivity", 0.3f));
            expect (engine->postControl ("suppressor/max-dynamic-notches", 3.0f));
            expect (engine->postControl ("suppressor/clear-dynamic", 1.0f));
            engine->serviceControls();
            expect (s.getEnabled(), "bypass 0 switches the suppressor on");
            expectWithinAbsoluteTolerance (s.getSensitivity(), 0.3f, 1e-6);
            expectEquals (s.getMaxDynamicNotches(), 3);

            dsp::MixingEngine::ControlRecord journal[16];
            const int n = engine->drainApplied (journal, 16);
            expectEquals (n, 3, "three settings are journalled; clear-dynamic is an action and is not");
            for (int i = 0; i < n; ++i)
                expect (std::string (journal[i].address) != "suppressor/clear-dynamic");

            // A count is a whole number in 1..8.
            expect (! engine->setControl ("suppressor/max-dynamic-notches", 3.5f));
            expect (! engine->setControl ("suppressor/max-dynamic-notches", 0.0f));
            expect (! engine->setControl ("suppressor/max-dynamic-notches", 9.0f));
            expect (! engine->setControl ("suppressor/max-dynamic-notches", 1e30f));
            expectEquals (s.getMaxDynamicNotches(), 3, "a refused count leaves the setting alone");
            expect (engine->setControl ("suppressor/bypass", 1.0f));
            expect (! s.getEnabled(), "bypass 1 switches it off");
            expect (! engine->setControl ("suppressor/nonsense", 1.0f));
        }

        beginTest ("A NaN sensitivity is refused, not stored");
        {
            // `jlimit(0, 1, NaN)` is NaN, and a NaN threshold makes every comparison false: the
            // suppressor would stop finding howls while reporting that it was running.
            dsp::FeedbackSuppressor s;
            s.setSensitivity (0.7f);
            s.setSensitivity (std::numeric_limits<float>::quiet_NaN());
            expectWithinAbsoluteTolerance (s.getSensitivity(), 0.7f, 1e-6);
            s.setSensitivity (std::numeric_limits<float>::infinity());
            expectWithinAbsoluteTolerance (s.getSensitivity(), 0.7f, 1e-6);
        }

        beginTest ("A number too big for an int is refused before it is cast");
        {
            // `static_cast<int>(1e30f)` is undefined behaviour; every integer field on the wire
            // cast first and checked after.
            auto engine = std::make_unique<dsp::MixingEngine>();
            const float huge[] = { 1e30f, -1e30f, 3.0e9f, -3.0e9f, 2147483648.0f };
            for (float v : huge)
            {
                const auto sv = juce::String (v);
                expect (! engine->setControl ("channel/1/input", v), "input " + sv);
                expect (! engine->setControl ("channel/1/dca", v), "dca " + sv);
                expect (! engine->setControl ("channel/1/group", v), "group " + sv);
                expect (! engine->setControl ("channel/1/eq/1/shape", v), "shape " + sv);
                expect (! engine->setControl ("output/1/source", v), "source " + sv);
                expect (! engine->setControl ("suppressor/max-dynamic-notches", v), "notches " + sv);
            }
            expectEquals (engine->getChannel (0).inputIndex, 0, "the patch is untouched");
            // In range still works, and the sidechain still rounds as it always did.
            expect (engine->setControl ("channel/1/input", 5.0f));
            expectEquals (engine->getChannel (0).inputIndex, 5);
            expect (engine->setControl ("master/sidechain/key", 2.6f));
            expectEquals (engine->getMaster().sidechainKey, 2);
            expect (engine->setControl ("master/sidechain/key", 1e30f));
            expectEquals (engine->getMaster().sidechainKey, -1, "out of range is none, not undefined");
        }

        beginTest ("Every address the engine routes has a first word the OSC receiver forwards");
        {
            // The receiver kept its own list of first words, and `dca` and `group` were not on it:
            // every DCA and subgroup message from the web was dropped before the queue while
            // these tests, which call `setControl` directly, passed.
            auto engine = std::make_unique<dsp::MixingEngine>();
            const char* routed[] = {
                "channel/1/fader", "master/gain", "suppressor/sensitivity", "dca/1/gain",
                "group/1/gain", "aux/1/gain", "output/1/delay",
            };
            for (auto* a : routed)
            {
                const std::string addr (a);
                expect (engine->setControl (addr, 0.0f), "the engine routes " + addr);
                const auto root = addr.substr (0, addr.find ('/'));
                expect (dsp::MixingEngine::isControlRoot (root), "the receiver forwards " + root);
            }
            expect (! dsp::MixingEngine::isControlRoot ("device"));
            expect (! dsp::MixingEngine::isControlRoot ("record"));
            expect (! dsp::MixingEngine::isControlRoot (""));
        }

        beginTest ("A control message with no number is refused, never read as 0");
        {
            // 0 is a meaningful value here: `/master/limiter/enabled` with no argument switched
            // the master limiter off.
            expect (! dsp::controlValue (dsp::ArgKind::Missing, 0.0f, 0).has_value());
            expect (! dsp::controlValue (dsp::ArgKind::Other, 0.0f, 0).has_value());
            expect (! dsp::controlValue (dsp::ArgKind::Float32, std::numeric_limits<float>::quiet_NaN(), 0).has_value());
            expect (! dsp::controlValue (dsp::ArgKind::Float32, -std::numeric_limits<float>::infinity(), 0).has_value());
            expectEquals (*dsp::controlValue (dsp::ArgKind::Float32, 0.25f, 0), 0.25f);
            expectEquals (*dsp::controlValue (dsp::ArgKind::Int32, 0.0f, 7), 7.0f);
            expectEquals (*dsp::controlValue (dsp::ArgKind::Int32, 0.0f, 0), 0.0f);
        }

        beginTest ("200,000 garbage controls: no crash, finite audio, and the desk still answers");
        {
            auto engine = std::make_unique<dsp::MixingEngine>();
            engine->prepare (kRate, kBlock);
            std::mt19937 rng (20261012u);
            const char* words[] = {
                "channel", "master", "suppressor", "dca", "group", "aux", "output", "eq", "comp",
                "gate", "send", "amp", "fx", "reverb", "delay", "limiter", "speaker", "1", "2", "8",
                "9", "32", "33", "0", "-1", "999", "gain", "mute", "freq", "q", "enabled", "pre",
                "source", "input", "", "//", "x",
            };
            const float values[] = {
                0.0f, 1.0f, -1.0f, 0.5f, 1e30f, -1e30f, 1e-40f, 3.5f, 2147483648.0f,
                std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(),
                -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::max(),
            };
            std::uniform_int_distribution<int> word (0, (int) std::size (words) - 1);
            std::uniform_int_distribution<int> parts (1, 6);
            std::uniform_int_distribution<int> val (0, (int) std::size (values) - 1);
            std::uniform_int_distribution<int> byte (0, 255);
            std::uniform_real_distribution<float> any (-200.0f, 200.0f);

            std::vector<float> in ((size_t) kBlock);
            const float* inPtrs[2] = { in.data(), in.data() };
            Outputs out (8, kBlock);
            bool finite = true;

            for (int m = 0; m < 200000; ++m)
            {
                std::string addr;
                if (m % 7 == 0)
                {
                    // Raw bytes, including ones no address can contain.
                    const int len = parts (rng) * 12;
                    for (int i = 0; i < len; ++i) addr.push_back ((char) byte (rng));
                }
                else
                {
                    const int n = parts (rng);
                    for (int i = 0; i < n; ++i) { if (i) addr += '/'; addr += words[word (rng)]; }
                }
                const float v = (m % 3 == 0) ? any (rng) : values[val (rng)];
                if (m % 2 == 0) engine->postControl (addr, v);
                else engine->setControl (addr, v);

                if (m % 500 == 0)
                {
                    sine (in, m / 500);
                    engine->processAudio (inPtrs, 2, out.data(), 8, kBlock);
                    for (auto& o : out.storage) finite = finite && allFinite (o);
                }
            }
            expect (finite, "every output stayed finite through the garbage");

            // And the desk is still a desk.
            expect (engine->setControl ("channel/1/fader", -6.0f));
            expectWithinAbsoluteTolerance (engine->getChannel (0).faderDb, -6.0f, 1e-6);
            expect (! engine->postControl (std::string (500, 'a'), 1.0f), "a too-long address is refused");
        }
    }
};

static HardeningTests hardeningTests;
