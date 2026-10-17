#include "BassRig.h"
#include "GuitarAmp.h"
#include "MixingEngine.h"
#include <juce_core/juce_core.h>
#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>
#include "TestHelpers.h"

/**
 * The bass rig ported from the web app (`createBassChain` in `src/lib/bassFx.ts`).
 *
 * The web chain's numbers are **written out again here** — the Web Audio biquad, the power taper,
 * the makeup probe's target, the crossover and DI arithmetic — so a port that got one wrong
 * cannot agree with itself. The cabinet kit is held sample for sample against `GuitarAmp`'s own
 * cabinet, which its suite already holds against cabinet.ts.
 */
namespace {

constexpr double kPi = 3.14159265358979323846;
using Cx = std::complex<double>;

/** H(f) of Web Audio's lowpass/highpass with Q in dB — the formula Chromium's `Biquad` runs. */
Cx webPass(bool high, double fc, double qDb, double fs, double f) {
    const double w0 = 2.0 * kPi * fc / fs, c = std::cos(w0);
    const double alpha = std::sin(w0) / 2.0 * std::pow(10.0, -qDb / 20.0);
    const double b0 = high ? (1 + c) / 2 : (1 - c) / 2, b1 = high ? -(1 + c) : 1 - c, b2 = b0;
    const double a0 = 1 + alpha, a1 = -2 * c, a2 = 1 - alpha;
    const Cx z1 = std::polar(1.0, -2.0 * kPi * f / fs), z2 = z1 * z1;
    return (b0 + b1 * z1 + b2 * z2) / (a0 + a1 * z1 + a2 * z2);
}

double toDb(double v) { return 20.0 * std::log10(std::max(v, 1e-12)); }

/** Steady-state RMS of a sine through the rack, after a settling time. */
double sineRms(dsp::BassRig& rig, double fs, double hz, double amp, int block, int blocks, int skip) {
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

/** Every stage off but the crossover, at unity: what is left is the split summed back together. */
void plain(dsp::BassRig& rig) {
    rig.setParam("enabled", 1);
    rig.setParam("gate/enabled", 0);
    rig.setParam("comp/enabled", 0);
    rig.setParam("low", 0);
    rig.setParam("power", 0);
    rig.setParam("drive/enabled", 0);
    rig.setParam("eq/sub", 0);
    rig.setParam("eq/lowMid", 0);
    rig.setParam("eq/mid", 0);
    rig.setParam("eq/high", 0);
    rig.setParam("cab/enabled", 0);
    rig.setParam("di", 0);
    rig.setParam("limiter/enabled", 0);
}

bool waitForCabinet(dsp::BassRig& rig, int block) {
    std::vector<float> silence(static_cast<size_t>(block), 0.0f);
    for (int tries = 0; tries < 400; ++tries) {
        rig.serviceBackground();
        rig.process(silence.data(), block);
        if (rig.cabinetReady()) return true;
        juce::Thread::sleep(5);
    }
    return false;
}

} // namespace

class BassRigTests : public juce::UnitTest {
public:
    BassRigTests() : juce::UnitTest("BassRig", "DSP") {}

    void runTest() override {
        using Rig = dsp::BassRig;
        const double fs = 48000.0;
        const int block = 256;

        beginTest("WebBiquad is Web Audio's biquad: Q in dB on the passes, slope 1 on the shelves");
        {
            dsp::rack::WebBiquad lp, hp, shelf;
            lp.set(dsp::rack::WebBiquad::Kind::LowPass, 160.0, Rig::CrossoverQDb, 0.0, fs);
            hp.set(dsp::rack::WebBiquad::Kind::HighPass, 160.0, Rig::CrossoverQDb, 0.0, fs);
            for (const double f : { 30.0, 80.0, 160.0, 320.0, 1000.0, 8000.0 }) {
                expectWithinAbsoluteError(lp.magnitudeDb(f, fs), toDb(std::abs(webPass(false, 160, Rig::CrossoverQDb, fs, f))), 1e-6);
                expectWithinAbsoluteError(hp.magnitudeDb(f, fs), toDb(std::abs(webPass(true, 160, Rig::CrossoverQDb, fs, f))), 1e-6);
            }
            // A Web Audio low shelf at +6 dB reaches +6 at DC and 0 at Nyquist, +3 at its corner.
            shelf.set(dsp::rack::WebBiquad::Kind::LowShelf, 60.0, 0.0, 6.0, fs);
            expectWithinAbsoluteError(shelf.magnitudeDb(2.0, fs), 6.0, 0.05);
            expectWithinAbsoluteError(shelf.magnitudeDb(60.0, fs), 3.0, 0.05);
            expectWithinAbsoluteError(shelf.magnitudeDb(20000.0, fs), 0.0, 0.05);
            // Not clamped at 20 Hz the way FilterPrimitives is: the DC blocker is at 18.
            dsp::rack::WebBiquad dc;
            dc.set(dsp::rack::WebBiquad::Kind::HighPass, 18.0, 0.7, 0.0, fs);
            expectWithinAbsoluteError(dc.magnitudeDb(18.0, fs), toDb(std::abs(webPass(true, 18, 0.7, fs, 18))), 1e-6);
        }

        beginTest("bassPowerGain: 0 is silence, 1 is 0.7, dB-linear over 15 dB between");
        {
            expect(Rig::powerGain(0.0f) == 0.0f, "exactly zero at zero");
            expectWithinAbsoluteError(static_cast<double>(Rig::powerGain(1.0f)), 0.7, 1e-6);
            expectWithinAbsoluteError(static_cast<double>(Rig::powerGain(0.2f)), 0.7 * std::pow(10.0, -0.8 * 15.0 / 20.0), 1e-6);
            expectWithinAbsoluteError(toDb(Rig::powerGain(0.6f)) - toDb(Rig::powerGain(0.4f)), 3.0, 1e-4);
            expect(Rig::powerGain(std::nanf("")) == 0.0f, "a NaN is no power, not a NaN gain");
        }

        beginTest("the drive curve and its makeup are the web's (hardness 9, target 0.3256)");
        {
            dsp::rack::Curve c {};
            dsp::rack::tubeCurve(0.55, Rig::StageHardness, 0.32, c);
            const double k = 1 + 0.55 * 9, shift = 0.32 * 0.55 * 2.5, dc = std::tanh(shift);
            const double norm = std::max(std::abs(std::tanh(k + shift) - dc), std::abs(std::tanh(-k + shift) - dc));
            for (const int i : { 0, 512, 1023, 1024, 1500, 2047 }) {
                const double x = static_cast<double>(i) / 2047 * 2 - 1;
                expectWithinAbsoluteError(static_cast<double>(c[static_cast<size_t>(i)]), (std::tanh(k * x + shift) - dc) / norm, 1e-6);
            }
            // The probe: a -12 dBFS sine through one stage, DC removed, lands at 0.3256 RMS.
            const float m = dsp::rack::driveMakeup(c, 1, Rig::DriveTargetRms);
            double mean = 0, sum = 0;
            std::vector<double> y(1024);
            for (int i = 0; i < 1024; ++i) { y[static_cast<size_t>(i)] = dsp::rack::readCurve(c, static_cast<float>(0.25 * std::sin(2 * kPi * i / 1024))) * m; mean += y[static_cast<size_t>(i)]; }
            mean /= 1024;
            for (const double v : y) sum += (v - mean) * (v - mean);
            expectWithinAbsoluteError(std::sqrt(sum / 1024), Rig::DriveTargetRms, 1e-4);
        }

        beginTest("the kit's cabinet is GuitarAmp's cabinet, sample for sample, and the bass models are unity at 1 kHz");
        {
            using Amp = dsp::GuitarAmp;
            const double scales[3] = { 1.0, 0.6, 0.7 };
            for (int model = 0; model < 4; ++model)
                for (int mic = 0; mic < 3; ++mic) {
                    const auto a = Amp::cabinetImpulse(fs, static_cast<Amp::Cabinet>(model), static_cast<Amp::Mic>(mic), 1.5f, -2.0f);
                    const auto b = dsp::rack::cabinetImpulse(fs, model, 1.5f, -2.0f, scales[mic], mic);
                    expect(a.size() == b.size());
                    double worst = 0;
                    for (size_t i = 0; i < a.size(); ++i) worst = std::max(worst, std::abs(static_cast<double>(a[i]) - b[i]));
                    expect(worst < 1e-6, "model " + juce::String(model) + " mic " + juce::String(mic) + " differs by " + juce::String(worst));
                }
            for (int bass = 0; bass < Rig::CabinetCount; ++bass) {
                const auto ir = dsp::rack::cabinetImpulse(fs, Rig::cabinetTableIndex(bass), 0.0f, 0.0f);
                expectWithinAbsoluteError(dsp::rack::responseDbAt(ir, fs, 1000.0), 0.0, 1e-3);
                expect(dsp::rack::responseDbAt(ir, fs, 20.0) < -6.0, "the speaker's own high-pass");
                // A bass cabinet keeps a low E (41 Hz) that a guitar 4x12 throws away.
                const auto v30 = dsp::rack::cabinetImpulse(fs, 0, 0.0f, 0.0f);
                expect(dsp::rack::responseDbAt(ir, fs, 41.2) > dsp::rack::responseDbAt(v30, fs, 41.2) + 6.0);
            }
            expect(dsp::rack::cabinetSpec(Rig::cabinetTableIndex(0)).lowpassHz == 3000.0, "b15");
            expect(dsp::rack::cabinetSpec(Rig::cabinetTableIndex(1)).lowpassHz == 5000.0, "b410");
        }

        beginTest("off is untouched; on and plain is the crossover summed back, × √2 for the panner");
        {
            juce::dsp::ConvolutionMessageQueue queue;
            Rig rig(queue);
            rig.prepare(fs, block);
            std::vector<float> x(static_cast<size_t>(block));
            for (size_t i = 0; i < x.size(); ++i) x[i] = static_cast<float>(0.3 * std::sin(0.01 * static_cast<double>(i)));
            const auto untouched = x;
            rig.process(x.data(), block);
            expect(x == untouched, "a rack that is off is not in the path");

            plain(rig);
            rig.reset();
            for (const double f : { 50.0, 120.0, 160.0, 220.0, 1000.0 }) {
                rig.reset();
                const double got = sineRms(rig, fs, f, 0.1, block, 120, 60);
                const Cx lp = webPass(false, 160, Rig::CrossoverQDb, fs, f), hp = webPass(true, 160, Rig::CrossoverQDb, fs, f);
                // DC block, then LOW (lp²) + the high band through the drive's 30 Hz blocker (hp²·H30).
                const double expected = 0.1 / std::sqrt(2.0) * std::abs(webPass(true, 18, 0.7, fs, f))
                                        * std::abs(lp * lp + hp * hp * webPass(true, 30, 0.7, fs, f)) * std::sqrt(2.0);
                expectWithinAbsoluteError(toDb(got), toDb(expected), 0.05,
                                          juce::String(f) + " Hz");
            }
        }

        beginTest("the DI is the low and high bands through three low-passes at the cabinet's corner, equal power");
        {
            juce::dsp::ConvolutionMessageQueue queue;
            Rig rig(queue);
            rig.prepare(fs, block);
            plain(rig);
            rig.setParam("di", 1.0f);          // all DI: cos(π/2) = 0 on the amp
            rig.setParam("cab/model", 1.0f);   // b410, corner 5 kHz
            rig.setParam("cab/enabled", 1.0f); // the DI's top follows the cabinet
            rig.reset();
            for (const double f : { 200.0, 2000.0, 5000.0, 10000.0 }) {
                rig.reset();
                const double got = sineRms(rig, fs, f, 0.1, block, 80, 40);
                const Cx lp = webPass(false, 160, Rig::CrossoverQDb, fs, f), hp = webPass(true, 160, Rig::CrossoverQDb, fs, f);
                const Cx top = webPass(false, 5000, Rig::CrossoverQDb, fs, f);
                const double expected = 0.1 / std::sqrt(2.0) * std::abs((lp * lp + hp * hp) * top * top * top)
                                        * std::abs(webPass(true, 18, 0.7, fs, f)) * std::sqrt(2.0);
                expectWithinAbsoluteError(toDb(got), toDb(expected), 0.05, juce::String(f) + " Hz");
            }
            // Halfway: each path at sin/cos(π/4) — the amp path is the crossover, the DI that same sum low-passed.
            rig.setParam("di", 0.5f);
            rig.setParam("cab/enabled", 0.0f); // the DI's top opens, so both halves are the same signal
            rig.reset();
            const double half = sineRms(rig, fs, 1000.0, 0.1, block, 80, 40);
            rig.setParam("di", 0.0f);
            rig.reset();
            const double none = sineRms(rig, fs, 1000.0, 0.1, block, 80, 40);
            expectWithinAbsoluteError(toDb(half) - toDb(none), toDb(std::sqrt(2.0)), 0.15, "two coherent halves at -3 dB each sum to +3");
        }

        beginTest("power adds harmonics above the crossover and nothing at zero");
        {
            juce::dsp::ConvolutionMessageQueue queue;
            Rig rig(queue);
            rig.prepare(fs, block);
            auto harmonicEnergy = [&](float power) {
                plain(rig);
                rig.setParam("power", power);
                rig.reset();
                std::vector<float> x(static_cast<size_t>(block));
                std::vector<float> all;
                for (int b = 0; b < 200; ++b) {
                    for (int i = 0; i < block; ++i)
                        x[static_cast<size_t>(i)] = static_cast<float>(0.3 * std::sin(2.0 * kPi * 50.0 * (b * block + i) / fs));
                    rig.process(x.data(), block);
                    // 150 blocks × 256 = 38400 samples: exactly 40 cycles of 50 Hz, so the
                    // fundamental leaks into no harmonic bin and "clean" can mean clean.
                    if (b >= 50) all.insert(all.end(), x.begin(), x.end());
                }
                // Energy at 150 Hz and 250 Hz (3rd, 5th) by correlation.
                double e = 0;
                for (const double h : { 150.0, 250.0, 350.0 }) {
                    double re = 0, im = 0;
                    for (size_t i = 0; i < all.size(); ++i) { re += all[i] * std::cos(2 * kPi * h * static_cast<double>(i) / fs); im += all[i] * std::sin(2 * kPi * h * static_cast<double>(i) / fs); }
                    e += (re * re + im * im) / (static_cast<double>(all.size()) * all.size());
                }
                return e;
            };
            const double none = harmonicEnergy(0.0f);
            const double some = harmonicEnergy(0.5f);
            const double full = harmonicEnergy(1.0f);
            expect(none < 1e-9, "power 0: a clean sine stays clean (" + juce::String(none) + ")");
            expect(some > none * 100.0 + 1e-8, "power adds harmonics");
            expect(full > some, "and more of it adds more");
        }

        beginTest("the limiter holds the output at its ceiling (× √2 for the panner)");
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
                for (int i = 0; i < block; ++i) x[static_cast<size_t>(i)] = static_cast<float>(0.8 * std::sin(2.0 * kPi * 110.0 * (b * block + i) / fs));
                rig.process(x.data(), block);
                if (b > 20) for (const float v : x) peak = std::max(peak, std::abs(v));
            }
            expect(peak <= std::pow(10.0f, -6.0f / 20.0f) * 1.41421356f + 1e-4f, "peak " + juce::String(peak));
        }

        beginTest("the web defaults, every stage on, the cabinet built off the audio thread: a sound, never a NaN");
        {
            juce::dsp::ConvolutionMessageQueue queue;
            Rig rig(queue);
            rig.prepare(fs, block);
            rig.setParam("enabled", 1.0f);
            rig.setParam("drive/enabled", 1.0f);
            rig.setParam("drive/amount", 0.55f);
            expect(!rig.cabinetReady(), "nothing is built until the message thread asks");
            expect(waitForCabinet(rig, block), "the cabinet impulse swapped in");
            const double out = sineRms(rig, fs, 55.0, 0.2, block, 200, 100);
            expect(std::isfinite(out) && out > 1e-3, "a bass through the rig makes a sound, RMS " + juce::String(out, 5));
            // An odd block length longer than the prepared size is processed in pieces, not cut short.
            std::vector<float> big(static_cast<size_t>(block * 3 + 17), 0.25f);
            rig.process(big.data(), static_cast<int>(big.size()));
            for (const float v : big) expect(std::isfinite(v));
            expect(big.back() != 0.25f, "the tail past the prepared block was processed too");
        }

        beginTest("the engine routes bass/ to the channel's rig, refuses unknown names, and the desk's defaults sound");
        {
            // Exactly what `bassDspParams(channel, DEFAULT_BASS)` sends, in that order.
            const std::pair<const char*, float> web[] = {
                { "bass/enabled", 1 }, { "bass/input", 0 }, { "bass/gate/enabled", 1 }, { "bass/gate/threshold", -62 },
                { "bass/comp/enabled", 1 }, { "bass/comp/threshold", -18 }, { "bass/comp/ratio", 3 },
                { "bass/comp/attack", 12 }, { "bass/comp/release", 280 }, { "bass/crossover", 160 }, { "bass/low", 4 },
                { "bass/power", 0.2f }, { "bass/drive/enabled", 0 }, { "bass/drive/amount", 0.25f }, { "bass/drive/bias", 0.2f },
                { "bass/eq/sub", 4.5f }, { "bass/eq/lowMid", -1.5f }, { "bass/eq/mid", 1 }, { "bass/eq/high", 1 },
                { "bass/cab/enabled", 1 }, { "bass/cab/model", 1 }, { "bass/cab/presence", 0 }, { "bass/cab/resonance", 0 },
                { "bass/di", 0.25f }, { "bass/output", 0 }, { "bass/limiter/enabled", 1 }, { "bass/limiter/ceiling", -0.3f },
            };
            dsp::MixingEngine engine;
            engine.prepare(fs, block);
            for (const auto& [name, value] : web) expect(engine.setChannelParam(1, name, value), name);
            expect(!engine.setChannelParam(1, "bass/nonsense", 1.0f));
            expect(!engine.setChannelParam(1, "bass/", 1.0f));
            expect(engine.getChannel(1).bass->isEnabled());
            expect(!engine.getChannel(0).bass->isEnabled(), "one channel's rig, not every channel's");
            expect(!engine.getChannel(1).amp->isEnabled(), "the bass rig is not the guitar rack");

            std::vector<float> in0(static_cast<size_t>(block)), in1(static_cast<size_t>(block));
            const float* inputs[] = { in0.data(), in1.data() };
            std::vector<float> outL(static_cast<size_t>(block)), outR(static_cast<size_t>(block));
            float* outputs[] = { outL.data(), outR.data() };
            double energy = 0.0;
            bool finite = true;
            for (int b = 0; b < 400; ++b) {
                engine.serviceBackgroundWork();
                for (int i = 0; i < block; ++i)
                    in1[static_cast<size_t>(i)] = static_cast<float>(0.1 * std::sin(2.0 * kPi * 41.2 * (b * block + i) / fs));
                engine.processAudio(inputs, 2, outputs, 2, block);
                if (b > 300)
                    for (int i = 0; i < block; ++i) {
                        finite = finite && std::isfinite(outL[static_cast<size_t>(i)]);
                        energy += outL[static_cast<size_t>(i)] * outL[static_cast<size_t>(i)];
                    }
                juce::Thread::sleep(1);
            }
            expect(finite, "no NaN reaches the output");
            logMessage("BassRig: engine channel 2 with the web defaults, output RMS " + juce::String(std::sqrt(energy / (99.0 * block)), 5));
            expect(energy > 1e-6, "a low E through the rig makes a sound");
            expect(engine.getChannel(1).bass->cabinetReady(), "the cabinet loaded through the engine's own queue");
            expect(engine.postControl("channel/32/bass/comp/release", 100.0f), "the longest name fits the queue");
        }

        beginTest("cost: one full bass rig, measured against real time (printed, not asserted)");
        {
            juce::dsp::ConvolutionMessageQueue queue;
            Rig rig(queue);
            const int b = 128;
            rig.prepare(fs, b);
            rig.setParam("enabled", 1.0f);
            rig.setParam("drive/enabled", 1.0f);
            rig.setParam("drive/amount", 0.5f);
            rig.setParam("power", 0.6f);
            expect(waitForCabinet(rig, b));
            std::vector<float> x(static_cast<size_t>(b));
            const int blocks = 2000;
            const auto start = juce::Time::getHighResolutionTicks();
            for (int k = 0; k < blocks; ++k) {
                for (int i = 0; i < b; ++i) x[static_cast<size_t>(i)] = static_cast<float>(0.2 * std::sin(0.02 * (k * b + i)));
                rig.process(x.data(), b);
            }
            const double sec = juce::Time::highResolutionTicksToSeconds(juce::Time::getHighResolutionTicks() - start);
            logMessage("BassRig: " + juce::String(100.0 * sec / (blocks * b / fs), 2) + "% of one core");
        }
    }
};

static BassRigTests bassRigTests;
