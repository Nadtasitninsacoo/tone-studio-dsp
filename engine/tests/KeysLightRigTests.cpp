#include "KeysRig.h"
#include "LightRig.h"
#include "MixingEngine.h"
#include <juce_core/juce_core.h>
#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>
#include "TestHelpers.h"

/**
 * The keys rack (`keysFx.ts`) and the four racks that share one chain — brass, phin, khaen, saw
 * (`brassFx.ts`, `phinFx.ts`, `khaenFx.ts`, `sawFx.ts`). The web numbers are written out again here.
 */
namespace {

constexpr double kPi = 3.14159265358979323846;
using Cx = std::complex<double>;
Cx zAt(double f, double fs) { return std::polar(1.0, -2.0 * kPi * f / fs); }

Cx webHp(double fc, double qDb, double fs, double f) {
    const double w0 = 2.0 * kPi * fc / fs, c = std::cos(w0), alpha = std::sin(w0) / 2.0 * std::pow(10.0, -qDb / 20.0);
    const Cx z1 = zAt(f, fs), z2 = z1 * z1;
    return ((1 + c) / 2 - (1 + c) * z1 + (1 + c) / 2 * z2) / ((1 + alpha) - 2 * c * z1 + (1 - alpha) * z2);
}
Cx webPeak(double fc, double q, double db, double fs, double f) {
    const double w0 = 2.0 * kPi * fc / fs, c = std::cos(w0), alpha = std::sin(w0) / (2 * q), A = std::pow(10.0, db / 40.0);
    const Cx z1 = zAt(f, fs), z2 = z1 * z1;
    return ((1 + alpha * A) - 2 * c * z1 + (1 - alpha * A) * z2) / ((1 + alpha / A) - 2 * c * z1 + (1 - alpha / A) * z2);
}
/** Web Audio's shelves (slope 1), written out: Chromium's `SetLowShelfParams` / `SetHighShelfParams`. */
Cx webShelf(bool high, double fc, double db, double fs, double f) {
    const double w0 = 2.0 * kPi * fc / fs, c = std::cos(w0), A = std::pow(10.0, db / 40.0);
    const double alpha = std::sin(w0) / 2.0 * std::sqrt(2.0), k = 2 * std::sqrt(A) * alpha;
    double b0, b1, b2, a0, a1, a2;
    if (!high) {
        b0 = A * (A + 1 - (A - 1) * c + k); b1 = 2 * A * (A - 1 - (A + 1) * c); b2 = A * (A + 1 - (A - 1) * c - k);
        a0 = A + 1 + (A - 1) * c + k; a1 = -2 * (A - 1 + (A + 1) * c); a2 = A + 1 + (A - 1) * c - k;
    } else {
        b0 = A * (A + 1 + (A - 1) * c + k); b1 = -2 * A * (A - 1 + (A + 1) * c); b2 = A * (A + 1 + (A - 1) * c - k);
        a0 = A + 1 - (A - 1) * c + k; a1 = 2 * (A - 1 - (A + 1) * c); a2 = A + 1 - (A - 1) * c - k;
    }
    const Cx z1 = zAt(f, fs), z2 = z1 * z1;
    return (b0 + b1 * z1 + b2 * z2) / (a0 + a1 * z1 + a2 * z2);
}
double toDb(double v) { return 20.0 * std::log10(std::max(v, 1e-12)); }

template <typename Rig>
double sineRms(Rig& rig, double fs, double hz, double amp, int block, int blocks, int skip) {
    std::vector<float> x(static_cast<size_t>(block));
    double sum = 0.0;
    long count = 0;
    for (int b = 0; b < blocks; ++b) {
        for (int i = 0; i < block; ++i)
            x[static_cast<size_t>(i)] = static_cast<float>(amp * std::sin(2.0 * kPi * hz * (b * block + i) / fs));
        rig.process(x.data(), block);
        if (b >= skip) for (const float v : x) { sum += static_cast<double>(v) * v; ++count; }
    }
    return std::sqrt(sum / std::max(1L, count));
}

template <typename Rig>
void plain(Rig& rig) {
    for (const auto* name : { "gate/enabled", "comp/enabled", "delay/enabled", "reverb/enabled", "limiter/enabled", "chorus/enabled" })
        rig.setParam(name, 0.0f);
    rig.setParam("enabled", 1);
    for (const auto* name : { "eq/low", "eq/mid", "eq/high" }) rig.setParam(name, 0.0f);
}

} // namespace

class KeysLightRigTests : public juce::UnitTest {
public:
    KeysLightRigTests() : juce::UnitTest("KeysLightRig", "DSP") {}

    void runTest() override {
        const double fs = 48000.0;
        const int block = 240;

        beginTest("brass, phin, khaen, saw: the DC block and the three bands are each web chain's own");
        {
            const std::pair<const char*, dsp::LightRig::Voice> voices[] = {
                { "brass", dsp::LightRig::Brass }, { "phin", dsp::LightRig::Phin },
                { "khaen", dsp::LightRig::Khaen }, { "saw", dsp::LightRig::Saw },
            };
            for (const auto& [name, voice] : voices) {
                juce::dsp::ConvolutionMessageQueue queue;
                dsp::LightRig rig(queue, voice);
                rig.prepare(fs, block);
                std::vector<float> x(static_cast<size_t>(block), 0.2f);
                const auto untouched = x;
                rig.process(x.data(), block);
                expect(x == untouched, juce::String(name) + ": off is not in the path");
                plain(rig);
                rig.setParam("eq/low", 3.0f);
                rig.setParam("eq/mid", -4.0f);
                rig.setParam("eq/high", 5.0f);
                for (const double f : { 100.0, voice.lowHz, voice.midHz, voice.highHz, 12000.0 }) {
                    rig.reset();
                    const double got = sineRms(rig, fs, f, 0.1, block, 200, 100);
                    const double expected = 0.1 * std::abs(webHp(voice.dcHz, 0.7, fs, f) * webShelf(false, voice.lowHz, 3.0, fs, f)
                                                           * webPeak(voice.midHz, voice.midQ, -4.0, fs, f) * webShelf(true, voice.highHz, 5.0, fs, f));
                    expectWithinAbsoluteError(toDb(got), toDb(expected), 0.05, juce::String(name) + " " + juce::String(f) + " Hz");
                }
            }
        }

        beginTest("the shared chain's compressor carries the node's makeup; its delay is one tap at its time");
        {
            juce::dsp::ConvolutionMessageQueue queue;
            dsp::LightRig rig(queue, dsp::LightRig::Phin);
            rig.prepare(fs, block);
            plain(rig);
            rig.setParam("comp/enabled", 1);
            rig.setParam("comp/threshold", -20);
            rig.setParam("comp/ratio", 4);
            rig.reset();
            const double quiet = sineRms(rig, fs, 1000.0, 0.0005, block, 200, 100);
            const double expected = 0.0005 * std::abs(webHp(40, 0.7, fs, 1000)) * std::pow(10.0, 0.6 * 0.75 * 20.0 / 20.0);
            expectWithinAbsoluteError(toDb(quiet), toDb(expected), 0.1, "far under the threshold the band is at its makeup");

            plain(rig);
            rig.setParam("delay/enabled", 1);
            rig.setParam("delay/time", 100);
            rig.setParam("delay/feedback", 0);
            rig.setParam("delay/mix", 1);
            for (const double f : { 200.0, 1000.0 }) {
                rig.reset();
                const double got = sineRms(rig, fs, f, 0.1, block, 220, 120);
                const double e = 0.1 * std::abs(webHp(40, 0.7, fs, f) * (1.0 + std::pow(zAt(f, fs), 4800.0)));
                expectWithinAbsoluteError(toDb(got), toDb(e), 0.1, juce::String(f) + " Hz");
            }
        }

        beginTest("keys: plain is the DC block and the bands; the chorus is two taps around 20 ms, dry 1 − MIX/2");
        {
            juce::dsp::ConvolutionMessageQueue queue;
            dsp::KeysRig rig(queue);
            rig.prepare(fs, block);
            plain(rig);
            rig.setParam("eq/mid", 4.0f);
            for (const double f : { 100.0, 1200.0, 8000.0 }) {
                rig.reset();
                const double got = sineRms(rig, fs, f, 0.1, block, 200, 100);
                const double e = 0.1 * std::abs(webHp(20, 0.7, fs, f) * webPeak(1200, 0.8, 4.0, fs, f));
                expectWithinAbsoluteError(toDb(got), toDb(e), 0.05, juce::String(f) + " Hz");
            }
            plain(rig);
            rig.setParam("chorus/enabled", 1);
            rig.setParam("chorus/mix", 1);
            rig.setParam("chorus/depth", 0.1f);  // a sweep of a few samples...
            rig.setParam("chorus/rate", 0.1f);   // ...barely started: the taps sit at 20 ms
            rig.reset();
            const double got = sineRms(rig, fs, 200.0, 0.1, block, 30, 20);
            const double e = 0.1 * std::abs(webHp(20, 0.7, fs, 200) * (0.5 + 2.0 * std::pow(zAt(200, fs), 960.0)));
            expectWithinAbsoluteError(toDb(got), toDb(e), 0.15, "dry 0.5 + two taps at 20 ms");
            // Swept for real, it moves and stays finite.
            rig.setParam("chorus/depth", 4.0f);
            rig.setParam("chorus/rate", 2.0f);
            std::vector<float> x(static_cast<size_t>(block));
            for (int b = 0; b < 200; ++b) {
                for (int i = 0; i < block; ++i) x[static_cast<size_t>(i)] = static_cast<float>(0.3 * std::sin(0.03 * (b * block + i)));
                rig.process(x.data(), block);
                for (const float v : x) expect(std::isfinite(v));
            }
        }

        beginTest("the limiter holds the ceiling on both classes (× √2)");
        {
            juce::dsp::ConvolutionMessageQueue queue;
            dsp::KeysRig keys(queue);
            dsp::LightRig saw(queue, dsp::LightRig::Saw);
            keys.prepare(fs, block);
            saw.prepare(fs, block);
            for (auto* rig : { static_cast<void*>(&keys), static_cast<void*>(&saw) }) {
                float peak = 0;
                std::vector<float> x(static_cast<size_t>(block));
                auto run = [&](auto& r) {
                    plain(r);
                    r.setParam("limiter/enabled", 1);
                    r.setParam("limiter/ceiling", -6);
                    r.setParam("output", 12);
                    r.reset();
                    for (int b = 0; b < 100; ++b) {
                        for (int i = 0; i < block; ++i) x[static_cast<size_t>(i)] = static_cast<float>(0.8 * std::sin(2.0 * kPi * 300.0 * (b * block + i) / fs));
                        r.process(x.data(), block);
                        if (b > 20) for (const float v : x) peak = std::max(peak, std::abs(v));
                    }
                };
                if (rig == &keys) run(keys); else run(saw);
                expect(peak <= std::pow(10.0f, -6.0f / 20.0f) * 1.41421356f + 1e-4f, "peak " + juce::String(peak));
            }
        }

        beginTest("the engine routes keys/ brass/ phin/ khaen/ saw/ to their racks, and each sounds");
        {
            dsp::MixingEngine engine;
            engine.prepare(fs, 256);
            const char* prefixes[] = { "keys/", "brass/", "phin/", "khaen/", "saw/" };
            for (int i = 0; i < 5; ++i) {
                const juce::String p(prefixes[i]);
                for (const auto* name : { "enabled", "eq/mid", "reverb/enabled", "limiter/enabled" })
                    expect(engine.setChannelParam(i, (p + name).toStdString(), 1.0f), p + name);
                expect(!engine.setChannelParam(i, (p + "nonsense").toStdString(), 1.0f));
            }
            expect(engine.setChannelParam(0, "keys/chorus/rate", 1.0f) && !engine.setChannelParam(0, "keys/delay/time", 100.0f), "keys has a chorus and no delay");
            expect(engine.setChannelParam(1, "brass/delay/time", 300.0f) && !engine.setChannelParam(1, "brass/chorus/rate", 1.0f), "brass has a delay and no chorus");
            const auto& c = engine.getChannel(0);
            expect(c.keys->isEnabled() && !c.vocals->isEnabled() && !c.amp->isEnabled(), "only the keys on strip 1");
            expect(engine.getChannel(1).brass->isEnabled() && engine.getChannel(2).phin->isEnabled()
                   && engine.getChannel(3).khaen->isEnabled() && engine.getChannel(4).saw->isEnabled(), "each on its own strip");
            expect(engine.getChannel(2).phin->voice().midHz == 1250.0, "the phin strip holds the phin's voice");

            std::vector<std::vector<float>> in(5, std::vector<float>(256));
            const float* inputs[] = { in[0].data(), in[1].data(), in[2].data(), in[3].data(), in[4].data() };
            std::vector<float> outL(256), outR(256);
            float* outputs[] = { outL.data(), outR.data() };
            double energy = 0;
            bool finite = true;
            for (int b = 0; b < 200; ++b) {
                engine.serviceBackgroundWork();
                for (int k = 0; k < 5; ++k)
                    for (int i = 0; i < 256; ++i) in[static_cast<size_t>(k)][static_cast<size_t>(i)] = static_cast<float>(0.05 * std::sin(2.0 * kPi * (220.0 + 50 * k) * (b * 256 + i) / fs));
                engine.processAudio(inputs, 5, outputs, 2, 256);
                if (b > 120) for (const float v : outL) { finite = finite && std::isfinite(v); energy += v * v; }
                juce::Thread::sleep(1);
            }
            expect(finite, "no NaN reaches the output");
            expect(energy > 1e-6, "five racks through the engine make a sound");
            expect(engine.postControl("channel/32/khaen/reverb/enabled", 1.0f), "the longest name fits the queue");
        }
    }
};

static KeysLightRigTests keysLightRigTests;
