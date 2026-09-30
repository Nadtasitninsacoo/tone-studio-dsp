#include "GuitarAmp.h"
#include "MixingEngine.h"
#include <juce_core/juce_core.h>
#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>
#include "TestHelpers.h"

// MainTests.cpp's allocation guard: `operator new` aborts while this is false.
extern thread_local bool allowAllocations;
extern thread_local const char* rtSection;

/**
 * The guitar rack ported from the web app (`createAmpChain` in `src/lib/ampFx.ts`).
 *
 * The pure halves are held against the web app's formulas **written out again here** —
 * `tubeCurve`, the makeup probe, the cabinet's 1 kHz normalisation, Chromium's convolver
 * scale — so a port that got one of them wrong cannot agree with itself.
 */
namespace {

constexpr double kPi = 3.14159265358979323846;

/** |H| in dB of the RBJ low- or high-pass at f — the fixed filters the web chain always runs. */
double passDb(bool high, double fc, double q, double fs, double f) {
    const double w0 = 2.0 * kPi * fc / fs, c = std::cos(w0), alpha = std::sin(w0) / (2.0 * q);
    const double b0 = high ? (1 + c) / 2 : (1 - c) / 2, b1 = high ? -(1 + c) : 1 - c, b2 = b0;
    const double a0 = 1 + alpha, a1 = -2 * c, a2 = 1 - alpha;
    const std::complex<double> z1 = std::polar(1.0, -2.0 * kPi * f / fs), z2 = z1 * z1;
    return 20.0 * std::log10(std::abs((b0 + b1 * z1 + b2 * z2) / (a0 + a1 * z1 + a2 * z2)));
}

double rmsOf(const std::vector<float>& x, size_t from) {
    double sum = 0.0;
    for (size_t i = from; i < x.size(); ++i) sum += static_cast<double>(x[i]) * x[i];
    return std::sqrt(sum / static_cast<double>(x.size() - from));
}

/** Run blocks until the amp's background-built impulses are in, or give up. */
bool waitForImpulses(dsp::GuitarAmp& amp, bool cab, bool rev, int block) {
    std::vector<float> silence(static_cast<size_t>(block), 0.0f);
    for (int tries = 0; tries < 400; ++tries) {
        amp.serviceBackground();
        amp.process(silence.data(), block);
        if ((!cab || amp.cabinetReady()) && (!rev || amp.reverbReady())) return true;
        juce::Thread::sleep(5);
    }
    return false;
}

/** Everything off but the path under test, at unity. */
void plain(dsp::GuitarAmp& amp) {
    amp.setParam("enabled", 1);
    amp.setParam("gate/enabled", 0);
    amp.setParam("comp/enabled", 0);
    amp.setParam("tone/bass", 0);
    amp.setParam("tone/mid", 0);
    amp.setParam("tone/treble", 0);
    amp.setParam("drive/enabled", 0);
    amp.setParam("cab/enabled", 0);
    amp.setParam("delay/enabled", 0);
    amp.setParam("reverb/enabled", 0);
    amp.setParam("limiter/enabled", 0);
}

} // namespace

class GuitarAmpTests : public juce::UnitTest {
public:
    GuitarAmpTests() : juce::UnitTest("GuitarAmp") {}

    void runTest() override {
        using Amp = dsp::GuitarAmp;
        const double fs = 48000.0;
        const int block = 512;

        beginTest("tubeCurve is the web app's formula, rails at ±1");
        {
            std::array<float, Amp::CurvePoints> curve {};
            const float amount = 0.35f, bias = 0.18f;
            Amp::tubeCurve(amount, bias, curve);
            const double k = 1.0 + amount * 14.0;
            const double shift = bias * amount * 2.5;
            const double dc = std::tanh(shift);
            const double norm = std::max(std::abs(std::tanh(k + shift) - dc), std::abs(std::tanh(-k + shift) - dc));
            for (int i : { 0, 300, 1023, 1500, 2047 }) {
                const double x = static_cast<double>(i) / (Amp::CurvePoints - 1) * 2.0 - 1.0;
                expectWithinAbsoluteTolerance(curve[static_cast<size_t>(i)], (std::tanh(k * x + shift) - dc) / norm, 1e-6);
            }
            const float top = std::max(std::abs(curve.front()), std::abs(curve.back()));
            expectWithinAbsoluteTolerance(top, 1.0, 1e-6);
            // Zero in, zero out: the DC offset of the bias is removed.
            expectWithinAbsoluteTolerance(Amp::readCurve(curve, 0.0f), 0.0, 1e-3);
            // The WaveShaper clamps its input to ±1 before reading.
            expectEquals(Amp::readCurve(curve, 5.0f), curve.back());
            expectEquals(Amp::readCurve(curve, -5.0f), curve.front());
        }

        beginTest("driveMakeup lands a -12 dBFS sine at the target RMS through every stage");
        {
            std::array<float, Amp::CurvePoints> curve {};
            for (const float amount : { 0.1f, 0.35f, 1.0f }) {
                Amp::tubeCurve(amount, 0.18f, curve);
                for (int stages = 1; stages <= 3; ++stages) {
                    const float makeup = Amp::driveMakeup(curve, stages);
                    std::vector<double> out(4096);
                    double mean = 0.0;
                    for (size_t i = 0; i < out.size(); ++i) {
                        float v = static_cast<float>(0.25 * std::sin(2.0 * kPi * static_cast<double>(i) / out.size()));
                        for (int s = 0; s < stages; ++s) v = Amp::readCurve(curve, v);
                        out[i] = v * makeup;
                        mean += out[i];
                    }
                    mean /= static_cast<double>(out.size());
                    double sum = 0.0;
                    for (const double v : out) sum += (v - mean) * (v - mean);
                    expectWithinAbsoluteTolerance(std::sqrt(sum / static_cast<double>(out.size())), 0.389, 0.002);
                }
            }
            expectEquals(Amp::driveMakeup(curve, 0), 1.0f);
        }

        beginTest("the cabinet impulse is 24 ms, unity at 1 kHz, and band-limited like a speaker");
        {
            for (int model = 0; model < 4; ++model) {
                const auto ir = Amp::cabinetImpulse(fs, static_cast<Amp::Cabinet>(model), Amp::Mic::Center, 0.0f, 0.0f);
                expectEquals(static_cast<int>(ir.size()), static_cast<int>(std::floor(fs * 0.024)));
                expectWithinAbsoluteTolerance(Amp::responseDbAt(ir, fs, 1000.0), 0.0, 1e-3);
                expect(Amp::responseDbAt(ir, fs, 40.0) < -8.0, "the speaker's own high-pass");
                expect(Amp::responseDbAt(ir, fs, 12000.0) < -20.0, "and its roll-off");
            }
            // Off-axis is darker than on the cap: the whole reason the control exists.
            const auto centre = Amp::cabinetImpulse(fs, Amp::Cabinet::V30, Amp::Mic::Center, 0, 0);
            const auto off = Amp::cabinetImpulse(fs, Amp::Cabinet::V30, Amp::Mic::OffAxis, 0, 0);
            expect(Amp::responseDbAt(off, fs, 3000.0) < Amp::responseDbAt(centre, fs, 3000.0) - 2.0);
            // The presence knob moves the presence peak.
            const auto bright = Amp::cabinetImpulse(fs, Amp::Cabinet::V30, Amp::Mic::Center, 6.0f, 0.0f);
            expect(Amp::responseDbAt(bright, fs, 2600.0) > Amp::responseDbAt(centre, fs, 2600.0) + 3.0);
        }

        beginTest("the reverb impulse is repeatable and scaled the way Chromium's convolver scales it");
        {
            const auto a = Amp::roomImpulse(fs, 1.6f, 7u);
            const auto b = Amp::roomImpulse(fs, 1.6f, 7u);
            expectEquals(static_cast<int>(a.size()), static_cast<int>(std::floor(fs * 1.6)));
            expect(a == b, "same seed, same tail");
            // A constant impulse has power 1, so the scale is the calibration alone.
            const std::vector<float> ones(1000, 1.0f);
            expectWithinAbsoluteTolerance(Amp::convolverNormalisation(ones, fs), 0.00125 * 44100.0 / fs, 1e-9);
        }

        beginTest("off is untouched; on at unity is the centre carried to mono (+3 dB)");
        {
            juce::dsp::ConvolutionMessageQueue queue;
            Amp amp(queue);
            amp.prepare(fs, block);

            std::vector<float> x(static_cast<size_t>(block * 40));
            for (size_t i = 0; i < x.size(); ++i) x[i] = static_cast<float>(0.1 * std::sin(2.0 * kPi * 1000.0 * i / fs));
            auto untouched = x;
            for (size_t o = 0; o < x.size(); o += block) amp.process(x.data() + o, block);
            expect(x == untouched, "a rack that is off is not in the path");

            plain(amp);
            for (size_t o = 0; o < x.size(); o += block) amp.process(x.data() + o, block);
            // The filters the web chain never switches off: the 22 Hz DC block and, twice, the
            // interstage 6.2 kHz low-pass and 30 Hz high-pass — Web Audio Q 0.7 dB, linear 1.084.
            const double q = std::pow(10.0, 0.7 / 20.0);
            const double fixedDb = passDb(true, 22.0, q, fs, 1000.0)
                                 + 2.0 * (passDb(false, 6200.0, q, fs, 1000.0) + passDb(true, 30.0, q, fs, 1000.0));
            expectWithinAbsoluteTolerance(20.0 * std::log10(rmsOf(x, x.size() / 2) / rmsOf(untouched, x.size() / 2)),
                                          20.0 * std::log10(std::sqrt(2.0)) + fixedDb, 0.02);
        }

        beginTest("the limiter holds the centre at its ceiling");
        {
            juce::dsp::ConvolutionMessageQueue queue;
            Amp amp(queue);
            amp.prepare(fs, block);
            plain(amp);
            amp.setParam("limiter/enabled", 1);
            amp.setParam("limiter/ceiling", -6.0f);
            amp.setParam("output", 12.0f);
            std::vector<float> x(static_cast<size_t>(block * 20));
            for (size_t i = 0; i < x.size(); ++i) x[i] = static_cast<float>(0.9 * std::sin(2.0 * kPi * 220.0 * i / fs));
            for (size_t o = 0; o < x.size(); o += block) amp.process(x.data() + o, block);
            float peak = 0.0f;
            for (const float v : x) peak = std::max(peak, std::abs(v));
            expect(peak <= std::pow(10.0f, -6.0f / 20.0f) * 1.41421356f + 1e-4f, "peak " + juce::String(peak));
        }

        beginTest("drive adds harmonics; the cabinet and reverb load off the audio thread");
        {
            juce::dsp::ConvolutionMessageQueue queue;
            Amp amp(queue);
            amp.prepare(fs, block);
            plain(amp);
            amp.setParam("drive/enabled", 1);
            amp.setParam("drive/amount", 0.8f);
            amp.setParam("drive/stages", 3);
            amp.setParam("cab/enabled", 1);
            amp.setParam("reverb/enabled", 1);
            amp.setParam("limiter/enabled", 1);
            expect(!amp.cabinetReady(), "nothing is built until the message thread asks");
            expect(waitForImpulses(amp, true, true, block), "both impulses swapped in");

            // A 220 Hz sine through three stages: the 3rd harmonic must appear.
            std::vector<float> x(static_cast<size_t>(block * 60));
            for (size_t i = 0; i < x.size(); ++i) x[i] = static_cast<float>(0.2 * std::sin(2.0 * kPi * 220.0 * i / fs));
            for (size_t o = 0; o < x.size(); o += block) amp.process(x.data() + o, block);
            auto bin = [&](double hz) {
                double re = 0, im = 0;
                for (size_t i = x.size() / 2; i < x.size(); ++i) {
                    re += x[i] * std::cos(2.0 * kPi * hz * i / fs);
                    im += x[i] * std::sin(2.0 * kPi * hz * i / fs);
                }
                return std::hypot(re, im);
            };
            expect(bin(660.0) > bin(220.0) * 0.02, "third harmonic from the tube stages");
            for (const float v : x) expect(std::isfinite(v));
        }

        beginTest("the oversampled drive holds its aliases down");
        {
            // A 5 kHz sine through three hard stages. Its 7th harmonic (35 kHz) is above the
            // 24 kHz Nyquist; without oversampling it folds to 13 kHz, a frequency no harmonic
            // of 5 kHz lands on — so energy there is aliasing and nothing else.
            juce::dsp::ConvolutionMessageQueue queue;
            Amp amp(queue);
            amp.prepare(fs, block);
            plain(amp);
            amp.setParam("drive/enabled", 1);
            amp.setParam("drive/amount", 1.0f);
            amp.setParam("drive/stages", 3);
            std::vector<float> x(static_cast<size_t>(block * 200));
            for (size_t i = 0; i < x.size(); ++i) x[i] = static_cast<float>(0.5 * std::sin(2.0 * kPi * 5000.0 * i / fs));
            for (size_t o = 0; o < x.size(); o += block) amp.process(x.data() + o, block);
            auto binDb = [&](double hz) {
                double re = 0, im = 0;
                const size_t from = x.size() / 2;
                for (size_t i = from; i < x.size(); ++i) {
                    const double w = 0.5 - 0.5 * std::cos(2.0 * kPi * (i - from) / (x.size() - from)); // Hann
                    re += w * x[i] * std::cos(2.0 * kPi * hz * i / fs);
                    im += w * x[i] * std::sin(2.0 * kPi * hz * i / fs);
                }
                return 20.0 * std::log10(std::hypot(re, im) + 1e-12);
            };
            const double alias = binDb(13000.0) - binDb(5000.0);
            logMessage("GuitarAmp: 7th-harmonic alias at 13 kHz " + juce::String(alias, 1) + " dB below the fundamental");
            expect(alias < -50.0, "alias " + juce::String(alias, 1) + " dB");
        }

        beginTest("the web desk's own settings, in its own order, through the engine: sound, not silence or NaN");
        {
            // Exactly what `ampDspParams(channel, DEFAULT_AMP)` sends, in that order.
            const std::pair<const char*, float> web[] = {
                { "amp/enabled", 1 }, { "amp/input", 0 }, { "amp/gate/enabled", 1 }, { "amp/gate/threshold", -58 },
                { "amp/comp/enabled", 0 }, { "amp/comp/threshold", -20 }, { "amp/comp/ratio", 3 },
                { "amp/tone/bass", 2 }, { "amp/tone/mid", 0 }, { "amp/tone/midHz", 700 }, { "amp/tone/treble", 2 },
                { "amp/drive/enabled", 1 }, { "amp/drive/amount", 0.35f }, { "amp/drive/stages", 2 }, { "amp/drive/bias", 0.18f },
                { "amp/cab/enabled", 1 }, { "amp/cab/model", 0 }, { "amp/cab/mic", 0 }, { "amp/cab/presence", 0 },
                { "amp/cab/resonance", 0 }, { "amp/cab/width", 0.35f }, { "amp/delay/enabled", 0 }, { "amp/delay/time", 0.34f },
                { "amp/delay/feedback", 0.28f }, { "amp/delay/mix", 0.22f }, { "amp/reverb/enabled", 1 }, { "amp/reverb/size", 1.6f },
                { "amp/reverb/mix", 0.18f }, { "amp/output", 0 }, { "amp/limiter/enabled", 1 }, { "amp/limiter/ceiling", -0.3f },
            };
            dsp::MixingEngine engine;
            engine.prepare(fs, block);
            for (const auto& [name, value] : web) expect(engine.setChannelParam(1, name, value), name);

            std::vector<float> in0(static_cast<size_t>(block)), in1(static_cast<size_t>(block));
            const float* inputs[] = { in0.data(), in1.data() };
            std::vector<float> outL(static_cast<size_t>(block)), outR(static_cast<size_t>(block));
            float* outputs[] = { outL.data(), outR.data() };
            double energy = 0.0;
            bool finite = true;
            for (int b = 0; b < 400; ++b) {
                engine.serviceBackgroundWork();
                for (int i = 0; i < block; ++i) {
                    in1[static_cast<size_t>(i)] = static_cast<float>(0.1 * std::sin(2.0 * kPi * 196.0 * (b * block + i) / fs));
                }
                engine.processAudio(inputs, 2, outputs, 2, block);
                if (b > 300)
                    for (int i = 0; i < block; ++i) {
                        finite = finite && std::isfinite(outL[static_cast<size_t>(i)]);
                        energy += outL[static_cast<size_t>(i)] * outL[static_cast<size_t>(i)];
                    }
                juce::Thread::sleep(1);
            }
            expect(finite, "no NaN reaches the output");
            logMessage("GuitarAmp: engine channel 2 with the web defaults, output RMS " + juce::String(std::sqrt(energy / (99.0 * block)), 5));
            expect(energy > 1e-6, "a guitar through the rack makes a sound");
            expect(engine.getChannel(1).amp->cabinetReady(), "the cabinet loaded through the engine's own queue");
        }

        beginTest("the engine routes amp/ to the channel's rack, and refuses unknown names");
        {
            dsp::MixingEngine engine;
            engine.prepare(fs, block);
            expect(engine.setChannelParam(0, "amp/enabled", 1.0f));
            expect(engine.setChannelParam(0, "amp/drive/amount", 0.5f));
            expect(!engine.setChannelParam(0, "amp/nonsense", 1.0f));
            expect(!engine.setChannelParam(0, "amp/", 1.0f));
            expect(engine.getChannel(0).amp->isEnabled());
            expect(!engine.getChannel(1).amp->isEnabled(), "one channel's rack, not every channel's");

            std::vector<float> in(static_cast<size_t>(block));
            for (size_t i = 0; i < in.size(); ++i) in[i] = static_cast<float>(0.2 * std::sin(2.0 * kPi * 440.0 * i / fs));
            const float* inputs[] = { in.data() };
            std::vector<float> outL(static_cast<size_t>(block)), outR(static_cast<size_t>(block));
            float* outputs[] = { outL.data(), outR.data() };
            for (int b = 0; b < 20; ++b) engine.processAudio(inputs, 1, outputs, 2, block);
            for (const float v : outL) expect(std::isfinite(v));

            // The path the web desk actually takes: OSC → the control queue → the audio thread.
            // The longest name has to fit the queue's address, on the highest channel.
            expect(engine.postControl("channel/32/amp/reverb/enabled", 1.0f));
            expect(engine.postControl("channel/2/amp/enabled", 1.0f));
            engine.processAudio(inputs, 1, outputs, 2, block);
            expect(engine.getChannel(1).amp->isEnabled(), "a queued control reaches the rack");
        }

        beginTest("cost: one full rack, measured against real time (printed, not asserted)");
        {
            juce::dsp::ConvolutionMessageQueue queue;
            Amp amp(queue);
            const int b = 128;
            amp.prepare(fs, b);
            amp.setParam("enabled", 1);
            amp.setParam("drive/stages", 3);
            amp.setParam("delay/enabled", 1);
            amp.setParam("comp/enabled", 1);
            expect(waitForImpulses(amp, true, true, b));
            std::vector<float> x(static_cast<size_t>(b));
            const int blocks = static_cast<int>(10.0 * fs / b); // ten seconds of audio
            const auto start = juce::Time::getHighResolutionTicks();
            for (int k = 0; k < blocks; ++k) {
                for (int i = 0; i < b; ++i) x[static_cast<size_t>(i)] = 0.2f * std::sin(0.05f * static_cast<float>(k * b + i));
                amp.process(x.data(), b);
            }
            const double seconds = juce::Time::highResolutionTicksToSeconds(juce::Time::getHighResolutionTicks() - start);
            logMessage("GuitarAmp: 10 s of audio in " + juce::String(seconds * 1000.0, 1) + " ms = "
                       + juce::String(seconds / 10.0 * 100.0, 2) + "% of one core, 3 stages + cabinet + 1.6 s reverb + delay");
            expect(seconds < 10.0, "faster than real time");

            // Where it goes: switch one part off at a time.
            for (const char* part : { "reverb/enabled", "cab/enabled", "drive/enabled" }) {
                amp.setParam(part, 0);
                const auto t0 = juce::Time::getHighResolutionTicks();
                for (int k = 0; k < blocks; ++k) amp.process(x.data(), b);
                const double s = juce::Time::highResolutionTicksToSeconds(juce::Time::getHighResolutionTicks() - t0);
                logMessage(juce::String("GuitarAmp: without ") + part + " (and what is already off) "
                           + juce::String(s / 10.0 * 100.0, 2) + "%");
            }
        }

        beginTest("processing and every setting allocate nothing on the audio thread");
        {
            juce::dsp::ConvolutionMessageQueue queue;
            Amp amp(queue);
            amp.prepare(fs, block);
            amp.setParam("enabled", 1);
            amp.setParam("delay/enabled", 1);
            amp.setParam("comp/enabled", 1);
            expect(waitForImpulses(amp, true, true, block));

            std::vector<float> x(static_cast<size_t>(block), 0.1f);
            allowAllocations = false;
            rtSection = "GuitarAmp";
            for (const char* name : { "input", "gate/threshold", "comp/ratio", "tone/bass", "tone/midHz",
                                      "drive/amount", "drive/stages", "drive/bias", "cab/model", "cab/mic",
                                      "cab/presence", "delay/time", "reverb/size", "output", "limiter/ceiling" }) {
                amp.setParam(name, 1.0f);
                amp.process(x.data(), block);
            }
            allowAllocations = true;
            expect(true);
        }
    }
};

static GuitarAmpTests guitarAmpTests;
