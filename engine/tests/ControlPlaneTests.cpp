#include "MixingEngine.h"
#include <juce_core/juce_core.h>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include "TestHelpers.h"

extern thread_local bool allowAllocations;
extern thread_local const char* rtSection;

/**
 * The input patch and the control queue — stage 0 of moving the racks into the engine.
 *
 * Engines are heap-allocated here on purpose: one is ~80 KB, and MixingEngineTests already
 * showed what several on one stack frame do.
 */
class ControlPlaneTests : public juce::UnitTest {
public:
    ControlPlaneTests() : juce::UnitTest("ControlPlane") {}

    static constexpr double kRate = 48000.0;
    static constexpr int kBlock = 256;

    struct Rig {
        std::unique_ptr<dsp::MixingEngine> engine = std::make_unique<dsp::MixingEngine>();
        std::vector<float> in0 = std::vector<float>(kBlock);
        std::vector<float> in1 = std::vector<float>(kBlock, 0.0f);
        std::vector<std::vector<float>> out = std::vector<std::vector<float>>(2, std::vector<float>(kBlock));
        int t = 0;

        Rig() {
            engine->prepare(kRate, kBlock);
            engine->getMaster().limiter.setEnabled(false);
        }

        /** Run n blocks of a 1 kHz sine on input 0 and silence on input 1; RMS of the last block, output 0. */
        float run(int blocks = 20) {
            const float* ins[2] = { in0.data(), in1.data() };
            float* outs[2] = { out[0].data(), out[1].data() };
            for (int b = 0; b < blocks; ++b) {
                for (int i = 0; i < kBlock; ++i, ++t)
                    in0[(size_t) i] = 0.1f * std::sin(2.0f * 3.14159265f * 1000.0f * (float) t / (float) kRate);
                engine->processAudio(ins, 2, outs, 2, kBlock);
            }
            double s = 0;
            for (float v : out[0]) s += (double) v * v;
            return (float) std::sqrt(s / kBlock);
        }

        void patchOnly(int channel, int input) {
            for (int c = 0; c < dsp::MixingEngine::MaxChannels; ++c)
                engine->setChannelParam(c, "input", c == channel ? (float) input : -1.0f);
        }
    };

    /** RMS of one output buffer of a Rig. */
    static float rms(const std::vector<float>& v) {
        double s = 0;
        for (float x : v) s += (double) x * x;
        return (float) std::sqrt(s / (double) v.size());
    }

    void runTest() override {
        beginTest("output patch: unpatched keeps the legacy master on 0/1");
        {
            Rig r;
            r.patchOnly(0, 0);
            r.run();
            expectGreaterThan(rms(r.out[0]), 0.01f, "master L on output 1");
            expect(! r.engine->isOutputPatched());
        }

        beginTest("output patch: master can be moved, and an unpatched output is silent");
        {
            Rig r;
            r.patchOnly(0, 0);
            expect(r.engine->setControl("/output/1/source", 0.0f), "none");
            expect(r.engine->setControl("/output/2/source", 1.0f), "master L to output 2");
            r.run();
            expectEquals(rms(r.out[0]), 0.0f, "output 1 carries nothing");
            expectGreaterThan(rms(r.out[1]), 0.01f, "output 2 carries master L");
            expect(! r.engine->setControl("/output/2/source", 13.0f), "an unknown source is refused");
            expect(r.engine->setControl("/output/32/source", 0.0f), "output 32 exists (MaxOutputs is 32)");
            expect(! r.engine->setControl("/output/33/source", 1.0f), "an output beyond 32 is refused");
            expect(! r.engine->setControl("/output/2/source", 1.5f), "a fraction is refused");
        }

        beginTest("output patch: an aux carries only what is sent to it");
        {
            Rig r;
            r.patchOnly(0, 0);
            r.engine->setControl("/output/1/source", 7.0f); // aux 1
            r.engine->setControl("/output/2/source", 0.0f);
            r.run();
            expectEquals(rms(r.out[0]), 0.0f, "no send, no aux");
            expect(r.engine->setChannelParam(0, "send/aux/1", 0.0f), "send at 0 dB");
            r.run();
            expectGreaterThan(rms(r.out[0]), 0.01f, "the aux carries the channel");
            expect(! r.engine->setChannelParam(0, "send/aux/7", 0.0f), "aux 7 is the reverb, not a monitor send");
        }

        beginTest("monitor mix: its own fader, mute and limiter act on the aux only");
        {
            Rig r;
            r.patchOnly(0, 0);
            r.engine->setControl("/output/1/source", 7.0f); // aux 1
            r.engine->setControl("/output/2/source", 1.0f); // master L
            r.engine->setChannelParam(0, "send/aux/1", 0.0f);
            r.run();
            const float aux = rms(r.out[0]);
            const float main = rms(r.out[1]);
            expectGreaterThan(aux, 0.01f, "the send reaches the mix");
            expect(r.engine->setControl("/aux/1/gain", -12.0f), "mix fader");
            r.run();
            expectWithinAbsoluteError(rms(r.out[0]) / aux, 0.2512f, 0.02f, "the mix fader moves the mix by 12 dB");
            expectWithinAbsoluteError(rms(r.out[1]), main, 0.001f, "and leaves the master alone");
            expect(r.engine->setControl("/aux/1/mute", 1.0f), "mix mute");
            r.run();
            expectEquals(rms(r.out[0]), 0.0f, "a muted mix is silent");
            r.engine->setControl("/aux/1/mute", 0.0f);
            r.engine->setControl("/aux/1/gain", 12.0f);
            r.engine->setControl("/aux/1/limiter/ceiling", -20.0f);
            r.run();
            float peak = 0;
            for (float v : r.out[0]) peak = std::max(peak, std::abs(v));
            expectLessOrEqual(peak, juce::Decibels::decibelsToGain(-20.0f) + 1.0e-6f, "the limiter holds its ceiling");
            expect(! r.engine->setControl("/aux/7/gain", 0.0f), "only six monitor mixes");
            expect(r.engine->setChannelParam(0, "send/aux/1/pre", 1.0f), "pre-fader tap");
            r.engine->setChannelParam(0, "fader", -80.0f);
            r.run();
            expectGreaterThan(rms(r.out[0]), 0.01f, "a pre-fader send ignores the channel fader");
        }

        beginTest("every channel's meter is reported: the web desk has 32 strips");
        {
            expectEquals(dsp::MixingEngine::MeteredChannels, dsp::MixingEngine::MaxChannels);
            // A channel above the old eight, patched to the live input, meters its signal.
            Rig r;
            r.patchOnly(19, 0);
            r.run();
            expect(r.engine->getChannel(19).metering.getPeakDb() > -40.0f, "channel 20 did not meter");
        }

        beginTest("channels default to the input with their own number");
        {
            Rig r;
            for (int c : { 0, 1, 7, 31 }) expectEquals(r.engine->getChannel(c).inputIndex, c);
        }

        beginTest("channel 3 can hear input 1 on a two-input device");
        {
            Rig r;
            r.patchOnly(2, 0);
            expect(r.run() > 0.01f, "patched channel is silent");
            r.patchOnly(2, 1); // input 2 carries silence
            expect(r.run() < 1e-6f, "patched to the silent input but still sounding");
        }

        beginTest("a channel patched to nothing is silent, and its meter says so");
        {
            Rig r;
            r.patchOnly(0, 0);
            r.run();
            expect(r.engine->getChannel(0).metering.getPeakDb() > -40.0f, "meter did not read the signal");
            const float before = r.engine->getChannel(0).metering.getPeakDb();
            r.patchOnly(0, dsp::MixingEngine::NoInput);
            const float rms = r.run(250);
            expect(rms < 1e-6f, "unpatched channel still sounding");
            // Falling, not frozen. The release rate is the meter's business; what matters here
            // is that a channel with no input keeps being told it has none.
            const float mid = r.engine->getChannel(0).metering.getPeakDb();
            r.run(250);
            const float after = r.engine->getChannel(0).metering.getPeakDb();
            expect(mid < before - 10.0f && after < mid - 10.0f,
                   "meter froze: " + juce::String(before) + " -> " + juce::String(mid) + " -> " + juce::String(after));
        }

        beginTest("two channels on one input are two channels: the level doubles");
        {
            Rig one;
            one.patchOnly(0, 0);
            const float a = one.run();
            Rig two;
            two.patchOnly(0, 0);
            two.engine->setChannelParam(1, "input", 0.0f);
            const float b = two.run();
            expectWithinAbsoluteTolerance(20.0 * std::log10(b / a), 6.02, 0.1);
        }

        beginTest("an input patch outside the device or malformed is refused");
        {
            Rig r;
            expect(!r.engine->setChannelParam(0, "input", 32.0f));
            expect(!r.engine->setChannelParam(0, "input", -2.0f));
            expect(!r.engine->setChannelParam(0, "input", 0.5f));
            expectEquals(r.engine->getChannel(0).inputIndex, 0);
            // A patch beyond the device's inputs is legal (the device may change) and silent.
            r.patchOnly(0, 5);
            expect(r.run() < 1e-6f);
        }

        beginTest("a posted control waits for the audio thread, then lands");
        {
            Rig r;
            expect(r.engine->postControl("channel/1/fader", -20.0f));
            expectEquals(r.engine->getChannel(0).faderDb, 0.0f); // not applied from this thread
            r.run(1);
            expectEquals(r.engine->getChannel(0).faderDb, -20.0f);
        }

        beginTest("prepare applies what was posted while the device was stopped");
        {
            Rig r;
            r.engine->postControl("master/gain", -6.0f);
            r.engine->prepare(kRate, kBlock);
            expectEquals(r.engine->getMaster().gainDb, -6.0f);
        }

        beginTest("the master routes that used to live in Main.cpp");
        {
            Rig r;
            auto& e = *r.engine;
            expect(e.setControl("master/gain", 400.0f));
            expectEquals(e.getMaster().gainDb, dsp::MixingEngine::MaxFaderDb);
            expect(e.setControl("/master/mute", 1.0f), "a leading slash is accepted");
            expect(e.getMaster().muted);
            expect(e.setControl("master/crossover/enabled", 1.0f));
            expect(e.getMaster().crossoverEnabled);
            for (const char* a : { "master/limiter/enabled", "master/limiter/ceiling", "master/crossover/frequency",
                                   "master/geq/1", "master/geq/31", "master/fx/reverb/enabled", "master/fx/reverb/room",
                                   "master/fx/reverb/damping", "master/fx/reverb/width", "master/fx/reverb/wet",
                                   "master/fx/delay/enabled", "master/fx/delay/time", "master/fx/delay/feedback",
                                   "master/fx/delay/wet", "master/fx/delay/pingpong", "master/fx/delay/hpf",
                                   "master/fx/delay/lpf", "channel/32/fader", "channel/1/eq/2/gain" })
                expect(e.setControl(a, 0.5f), juce::String("refused ") + a);
            for (const char* a : { "master/geq/0", "master/geq/32", "master/geq/1x", "master/geq/",
                                   "master/volume", "channel/33/fader", "channel/0/fader", "channel/x/fader",
                                   "channel/1", "channel//fader", "suppressor/bypass", "", "/" })
                expect(!e.setControl(a, 0.5f), juce::String("accepted ") + a);
            expect(!e.setControl("master/gain", std::nanf("")), "NaN");
        }

        beginTest("a refused queued control is counted, so Main.cpp can report it");
        {
            Rig r;
            r.engine->postControl("channel/1/colour", 1.0f);
            r.engine->postControl("channel/1/fader", -3.0f);
            r.run(1);
            expectEquals(r.engine->rejectedControls(), 1);
            expectEquals(r.engine->getChannel(0).faderDb, -3.0f);
        }

        beginTest("a full queue refuses and counts, it never blocks or overwrites");
        {
            Rig r;
            int accepted = 0;
            for (int i = 0; i < dsp::MixingEngine::ControlQueueSize + 10; ++i)
                if (r.engine->postControl("channel/1/fader", (float) -(i % 60))) ++accepted;
            expect(accepted >= dsp::MixingEngine::ControlQueueSize - 1);
            expect(r.engine->droppedControls() > 0);
            r.run(1);
            expect(r.engine->postControl("channel/1/fader", -1.0f), "queue did not recover after draining");
        }

        beginTest("an address longer than the slot is refused, not truncated");
        {
            Rig r;
            const std::string longAddress = "channel/1/" + std::string(60, 'x');
            expect(!r.engine->postControl(longAddress, 1.0f));
        }

        beginTest("draining every kind of control allocates nothing on the audio thread");
        {
            Rig r;
            for (int c = 1; c <= 8; ++c) {
                const juce::String ch = "channel/" + juce::String(c) + "/";
                for (const char* p : { "fader", "trim", "pan", "mute", "invert", "input", "hpf/enabled", "hpf/freq",
                                       "hpf/q", "lpf/freq", "eq/1/shape", "eq/1/freq", "eq/2/gain", "eq/3/q",
                                       "eq/4/enabled", "comp/enabled", "comp/threshold", "comp/attack",
                                       "comp/makeup", "comp/detection", "gate/enabled", "gate/threshold",
                                       "deesser/freq" })
                    r.engine->postControl((ch + p).toStdString(), std::string(p) == "input" ? 0.0f : 1.0f);
            }
            for (const char* a : { "master/gain", "master/mute", "master/limiter/ceiling", "master/geq/12",
                                   "master/crossover/frequency", "master/fx/reverb/room", "master/fx/delay/time" })
                r.engine->postControl(a, 0.3f);

            const float* ins[2] = { r.in0.data(), r.in1.data() };
            float* outs[2] = { r.out[0].data(), r.out[1].data() };
            rtSection = "MixingEngine::serviceControls";
            allowAllocations = false;
            r.engine->processAudio(ins, 2, outs, 2, kBlock);
            allowAllocations = true;
            rtSection = "(unknown)";
            expectEquals(r.engine->rejectedControls(), 0);
            expectEquals(r.engine->getMaster().gainDb, 0.3f);
        }
    }
};

static ControlPlaneTests controlPlaneTests;
