#include "DrumRig.h"
#include "MixingEngine.h"
#include <juce_core/juce_core.h>
#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>
#include "TestHelpers.h"

/**
 * The drum bus ported from the web app (`createDrumChain` in `src/lib/drumFx.ts`). As with the bass,
 * the web numbers are written out again here so the port cannot agree with itself.
 */
namespace {

constexpr double kPi = 3.14159265358979323846;
using Cx = std::complex<double>;

Cx zAt(double f, double fs) { return std::polar(1.0, -2.0 * kPi * f / fs); }

/** Web Audio high-pass, Q in dB. */
Cx webHp(double fc, double qDb, double fs, double f) {
    const double w0 = 2.0 * kPi * fc / fs, c = std::cos(w0), alpha = std::sin(w0) / 2.0 * std::pow(10.0, -qDb / 20.0);
    const Cx z1 = zAt(f, fs), z2 = z1 * z1;
    return ((1 + c) / 2 - (1 + c) * z1 + (1 + c) / 2 * z2) / ((1 + alpha) - 2 * c * z1 + (1 - alpha) * z2);
}
/** Web Audio peaking, Q linear. */
Cx webPeak(double fc, double q, double db, double fs, double f) {
    const double w0 = 2.0 * kPi * fc / fs, c = std::cos(w0), alpha = std::sin(w0) / (2 * q), A = std::pow(10.0, db / 40.0);
    const Cx z1 = zAt(f, fs), z2 = z1 * z1;
    return ((1 + alpha * A) - 2 * c * z1 + (1 - alpha * A) * z2) / ((1 + alpha / A) - 2 * c * z1 + (1 - alpha / A) * z2);
}
/** Web Audio band-pass, Q linear, 0 dB at its centre. */
Cx webBand(double fc, double q, double fs, double f) {
    const double w0 = 2.0 * kPi * fc / fs, c = std::cos(w0), alpha = std::sin(w0) / (2 * q);
    const Cx z1 = zAt(f, fs), z2 = z1 * z1;
    return (alpha - alpha * z2) / ((1 + alpha) - 2 * c * z1 + (1 - alpha) * z2);
}

double toDb(double v) { return 20.0 * std::log10(std::max(v, 1e-12)); }

double sineRms(dsp::DrumRig& rig, double fs, double hz, double amp, int block, int blocks, int skip) {
    std::vector<float> x(static_cast<size_t>(block));
    double sum = 0.0;
    long count = 0;
    for (int b = 0; b < blocks; ++b) {
        for (int i = 0; i < block; ++i)
            x[static_cast<size_t>(i)] = static_cast<float>(amp * std::sin(2.0 * kPi * hz * (b * block + i) / fs));
        rig.process(x.data(), block);
        if (b >= skip)
            for (const float v : x) { sum += static_cast<double>(v) * v; ++count; }
    }
    return std::sqrt(sum / std::max(1L, count));
}

void plain(dsp::DrumRig& rig) {
    for (const auto* name : { "gate/enabled", "drive/enabled", "crush/enabled", "room/enabled", "glue/enabled", "limiter/enabled" })
        rig.setParam(name, 0.0f);
    rig.setParam("enabled", 1);
    rig.setParam("kick/db", 0);
    rig.setParam("kick/depth", 0);
    rig.setParam("eq/box", 0);
    rig.setParam("eq/snap", 0);
}

bool waitForRoom(dsp::DrumRig& rig, int block) {
    std::vector<float> silence(static_cast<size_t>(block), 0.0f);
    for (int tries = 0; tries < 400; ++tries) {
        rig.serviceBackground();
        rig.process(silence.data(), block);
        if (rig.roomReady()) return true;
        juce::Thread::sleep(5);
    }
    return false;
}

} // namespace

class DrumRigTests : public juce::UnitTest {
public:
    DrumRigTests() : juce::UnitTest("DrumRig", "DSP") {}

    void runTest() override {
        using Rig = dsp::DrumRig;
        const double fs = 48000.0;
        const int block = 256;

        beginTest("kickQ and depthGain are the web's");
        {
            expectWithinAbsoluteError(Rig::kickQ(0.0f), 0.7, 1e-9);
            expectWithinAbsoluteError(Rig::kickQ(1.0f), 4.5, 1e-9);
            expectWithinAbsoluteError(Rig::kickQ(0.45f), 0.7 + 0.45 * 3.8, 1e-6);
            expectWithinAbsoluteError(static_cast<double>(Rig::depthGain(0.4f)), 0.32, 1e-6);
            expect(Rig::depthGain(std::nanf("")) == 0.0f);
        }

        beginTest("the saturation curve and its makeup are the web's (hardness 18, target 0.4691)");
        {
            dsp::rack::Curve c {};
            dsp::rack::saturationCurve(0.15, Rig::DriveHardness, c);
            const double k = 1 + 0.15 * 18;
            for (const int i : { 0, 700, 1024, 2047 }) {
                const double x = static_cast<double>(i) / 2047 * 2 - 1;
                expectWithinAbsoluteError(static_cast<double>(c[static_cast<size_t>(i)]), std::tanh(k * x) / std::tanh(k), 1e-6);
            }
            const float m = dsp::rack::driveMakeup(c, 1, Rig::DriveTargetRms);
            double sum = 0;
            for (int i = 0; i < 1024; ++i) {
                const double y = dsp::rack::readCurve(c, static_cast<float>(0.25 * std::sin(2 * kPi * i / 1024))) * m;
                sum += y * y; // symmetric curve: no DC to remove
            }
            expectWithinAbsoluteError(std::sqrt(sum / 1024), Rig::DriveTargetRms, 1e-4);
        }

        beginTest("off is untouched; plain is the DC blocker and the sub cut, × √2");
        {
            juce::dsp::ConvolutionMessageQueue queue;
            Rig rig(queue);
            rig.prepare(fs, block);
            std::vector<float> x(static_cast<size_t>(block), 0.2f);
            const auto untouched = x;
            rig.process(x.data(), block);
            expect(x == untouched, "a rack that is off is not in the path");
            plain(rig);
            for (const double f : { 25.0, 40.0, 60.0, 200.0, 2000.0 }) {
                rig.reset();
                // 75 blocks = 19200 samples: whole cycles of every frequency here (10 at 25 Hz).
                const double got = sineRms(rig, fs, f, 0.1, block, 155, 80);
                const Cx sub = webHp(30, Rig::SubCutQDb, fs, f);
                const double expected = 0.1 * std::abs(webHp(20, 0.7, fs, f) * sub * sub); // 0.1/√2 × √2
                expectWithinAbsoluteError(toDb(got), toDb(expected), 0.05, juce::String(f) + " Hz");
            }
            // The cut never sits above the kick: at 45 Hz with the cut asked for 80, it is held at 45.
            rig.setParam("kick/hz", 45.0f);
            rig.setParam("kick/subCut", 80.0f);
            rig.reset();
            const double got = sineRms(rig, fs, 100.0, 0.1, block, 160, 80);
            const Cx sub = webHp(45, Rig::SubCutQDb, fs, 100.0);
            expectWithinAbsoluteError(toDb(got), toDb(0.1 * std::abs(webHp(20, 0.7, fs, 100.0) * sub * sub)), 0.05);
        }

        beginTest("the kick lift, the box, the snap and the depth resonator, against the web's own filters");
        {
            juce::dsp::ConvolutionMessageQueue queue;
            Rig rig(queue);
            rig.prepare(fs, block);
            plain(rig);
            rig.setParam("kick/hz", 55.0f);
            rig.setParam("kick/db", 6.0f);
            rig.setParam("kick/tight", 0.45f);
            rig.setParam("eq/box", -2.0f);
            rig.setParam("eq/snap", 2.0f);
            rig.setParam("kick/depth", 1.0f);
            for (const double f : { 45.0, 55.0, 70.0, 400.0, 5000.0 }) {
                rig.reset();
                const double got = sineRms(rig, fs, f, 0.05, block, 200, 100);
                const Cx sub = webHp(30, Rig::SubCutQDb, fs, f);
                // Snap is a high shelf: the kit's shelf is Web Audio's, already held in BassRigTests;
                // here its gain at f is taken from the kit itself.
                dsp::rack::WebBiquad shelf;
                shelf.set(dsp::rack::WebBiquad::Kind::HighShelf, 3800, 0, 2.0, fs);
                const double shelfMag = std::pow(10.0, shelf.magnitudeDb(f, fs) / 20.0);
                const Cx main = webPeak(55, Rig::kickQ(0.45f), 6.0, fs, f) * webPeak(400, 1.1, -2.0, fs, f);
                // The shelf's phase matters only far from the kick; compare magnitudes per path where one dominates.
                const Cx withDepth = main * shelfMag + 0.8 * webBand(55, Rig::DepthQ, fs, f);
                const double expected = 0.05 * std::abs(webHp(20, 0.7, fs, f) * sub * sub * withDepth);
                expectWithinAbsoluteError(toDb(got), toDb(expected), f < 100 ? 0.1 : 0.15, juce::String(f) + " Hz");
            }
        }

        beginTest("PUNCH adds the crushed copy, and only while the crush is on");
        {
            juce::dsp::ConvolutionMessageQueue queue;
            Rig rig(queue);
            rig.prepare(fs, block);
            plain(rig);
            rig.setParam("crush/enabled", 1.0f);
            rig.setParam("punch", 0.0f);
            rig.reset();
            const double none = sineRms(rig, fs, 200.0, 0.1, block, 120, 60);
            rig.setParam("punch", 1.0f);
            rig.reset();
            const double full = sineRms(rig, fs, 200.0, 0.1, block, 120, 60);
            expect(toDb(full) - toDb(none) > 3.0, "the crushed copy, with its makeup, is underneath: +" + juce::String(toDb(full) - toDb(none), 2) + " dB");
            rig.setParam("crush/enabled", 0.0f);
            rig.reset();
            const double off = sineRms(rig, fs, 200.0, 0.1, block, 120, 60);
            expectWithinAbsoluteError(toDb(off), toDb(none), 0.01, "PUNCH with the crush off is nothing");
        }

        beginTest("the room builds off the audio thread and rings after the hit stops");
        {
            juce::dsp::ConvolutionMessageQueue queue;
            Rig rig(queue);
            rig.prepare(fs, block);
            plain(rig);
            rig.setParam("room/enabled", 1.0f);
            rig.setParam("room/mix", 0.5f);
            rig.setParam("room/size", 1.0f);
            expect(!rig.roomReady(), "nothing is built until the message thread asks");
            expect(waitForRoom(rig, block), "the room's tail swapped in");
            std::vector<float> x(static_cast<size_t>(block), 0.0f);
            x[0] = 1.0f; // one hit
            rig.process(x.data(), block);
            double tail = 0;
            for (int b = 0; b < 40; ++b) {
                std::fill(x.begin(), x.end(), 0.0f);
                rig.process(x.data(), block);
                for (const float v : x) { tail += v * v; expect(std::isfinite(v)); }
            }
            expect(tail > 1e-6, "the room is still ringing 200 ms later");
        }

        beginTest("the limiter holds the output at its ceiling (× √2)");
        {
            juce::dsp::ConvolutionMessageQueue queue;
            Rig rig(queue);
            rig.prepare(fs, block);
            plain(rig);
            rig.setParam("limiter/enabled", 1.0f);
            rig.setParam("limiter/ceiling", -6.0f);
            rig.setParam("output", 12.0f);
            rig.reset();
            std::vector<float> x(static_cast<size_t>(block));
            float peak = 0;
            for (int b = 0; b < 100; ++b) {
                for (int i = 0; i < block; ++i) x[static_cast<size_t>(i)] = static_cast<float>(0.8 * std::sin(2.0 * kPi * 80.0 * (b * block + i) / fs));
                rig.process(x.data(), block);
                if (b > 20) for (const float v : x) peak = std::max(peak, std::abs(v));
            }
            expect(peak <= std::pow(10.0f, -6.0f / 20.0f) * 1.41421356f + 1e-4f, "peak " + juce::String(peak));
        }

        beginTest("the engine routes drums/ to the channel's bus, and the desk's defaults sound");
        {
            // Exactly what `drumDspParams(channel, DEFAULT_DRUMS)` sends, in that order.
            const std::pair<const char*, float> web[] = {
                { "drums/enabled", 1 }, { "drums/input", 0 }, { "drums/gate/enabled", 0 }, { "drums/gate/threshold", -54 },
                { "drums/kick/hz", 55 }, { "drums/kick/db", 6 }, { "drums/kick/tight", 0.45f }, { "drums/kick/subCut", 30 },
                { "drums/kick/depth", 0.4f }, { "drums/eq/box", -2 }, { "drums/eq/snap", 2 }, { "drums/drive/enabled", 1 },
                { "drums/drive/amount", 0.15f }, { "drums/crush/enabled", 1 }, { "drums/crush/threshold", -32 },
                { "drums/crush/ratio", 10 }, { "drums/punch", 0.3f }, { "drums/room/enabled", 1 }, { "drums/room/size", 0.8f },
                { "drums/room/mix", 0.14f }, { "drums/glue/enabled", 1 }, { "drums/glue/threshold", -14 }, { "drums/glue/ratio", 2 },
                { "drums/output", 0 }, { "drums/limiter/enabled", 1 }, { "drums/limiter/ceiling", -0.3f },
            };
            dsp::MixingEngine engine;
            engine.prepare(fs, block);
            for (const auto& [name, value] : web) expect(engine.setChannelParam(2, name, value), name);
            expect(!engine.setChannelParam(2, "drums/nonsense", 1.0f));
            expect(engine.getChannel(2).drums->isEnabled());
            expect(!engine.getChannel(2).bass->isEnabled() && !engine.getChannel(2).amp->isEnabled(), "only the drum bus");

            std::vector<float> in0(static_cast<size_t>(block)), in1(static_cast<size_t>(block)), in2(static_cast<size_t>(block));
            const float* inputs[] = { in0.data(), in1.data(), in2.data() };
            std::vector<float> outL(static_cast<size_t>(block)), outR(static_cast<size_t>(block));
            float* outputs[] = { outL.data(), outR.data() };
            double energy = 0.0;
            bool finite = true;
            for (int b = 0; b < 400; ++b) {
                engine.serviceBackgroundWork();
                std::fill(in2.begin(), in2.end(), 0.0f);
                if (b % 50 == 0) for (int i = 0; i < 64; ++i) in2[static_cast<size_t>(i)] = static_cast<float>(0.5 * std::sin(2 * kPi * 55.0 * i / fs) * (1.0 - i / 64.0));
                engine.processAudio(inputs, 3, outputs, 2, block);
                if (b > 300)
                    for (int i = 0; i < block; ++i) {
                        finite = finite && std::isfinite(outL[static_cast<size_t>(i)]);
                        energy += outL[static_cast<size_t>(i)] * outL[static_cast<size_t>(i)];
                    }
                juce::Thread::sleep(1);
            }
            expect(finite, "no NaN reaches the output");
            expect(energy > 1e-7, "a kick through the bus makes a sound");
            expect(engine.getChannel(2).drums->roomReady(), "the room loaded through the engine's own queue");
            expect(engine.postControl("channel/32/drums/glue/threshold", -10.0f), "the longest name fits the queue");
        }

        beginTest("cost: one full drum bus, measured against real time (printed, not asserted)");
        {
            juce::dsp::ConvolutionMessageQueue queue;
            Rig rig(queue);
            const int b = 128;
            rig.prepare(fs, b);
            rig.setParam("enabled", 1.0f);
            expect(waitForRoom(rig, b));
            std::vector<float> x(static_cast<size_t>(b));
            const int blocks = 2000;
            const auto start = juce::Time::getHighResolutionTicks();
            for (int k = 0; k < blocks; ++k) {
                for (int i = 0; i < b; ++i) x[static_cast<size_t>(i)] = static_cast<float>(0.2 * std::sin(0.02 * (k * b + i)));
                rig.process(x.data(), b);
            }
            const double sec = juce::Time::highResolutionTicksToSeconds(juce::Time::getHighResolutionTicks() - start);
            logMessage("DrumRig: " + juce::String(100.0 * sec / (blocks * b / fs), 2) + "% of one core");
        }
    }
};

static DrumRigTests drumRigTests;
