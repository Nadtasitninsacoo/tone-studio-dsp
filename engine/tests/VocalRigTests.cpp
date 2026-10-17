#include "VocalRig.h"
#include "MixingEngine.h"
#include <juce_core/juce_core.h>
#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>
#include "TestHelpers.h"

/**
 * The vocal rack ported from the web app (`createVocalChain` in `src/lib/vocalFx.ts`). The web
 * numbers are written out again here so the port cannot agree with itself.
 */
namespace {

constexpr double kPi = 3.14159265358979323846;
using Cx = std::complex<double>;

Cx zAt(double f, double fs) { return std::polar(1.0, -2.0 * kPi * f / fs); }

/** Web Audio low/high-pass, Q in dB. */
Cx webPass(bool high, double fc, double qDb, double fs, double f) {
    const double w0 = 2.0 * kPi * fc / fs, c = std::cos(w0), alpha = std::sin(w0) / 2.0 * std::pow(10.0, -qDb / 20.0);
    const double b0 = high ? (1 + c) / 2 : (1 - c) / 2, b1 = high ? -(1 + c) : 1 - c;
    const Cx z1 = zAt(f, fs), z2 = z1 * z1;
    return (b0 + b1 * z1 + b0 * z2) / ((1 + alpha) - 2 * c * z1 + (1 - alpha) * z2);
}

double toDb(double v) { return 20.0 * std::log10(std::max(v, 1e-12)); }

double sineRms(dsp::VocalRig& rig, double fs, double hz, double amp, int block, int blocks, int skip) {
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

void plain(dsp::VocalRig& rig) {
    for (const auto* name : { "gate/enabled", "deesser/enabled", "comp/enabled", "delay/enabled", "reverb/enabled", "limiter/enabled" })
        rig.setParam(name, 0.0f);
    rig.setParam("enabled", 1);
    rig.setParam("eq/body", 0);
    rig.setParam("eq/presence", 0);
    rig.setParam("eq/air", 0);
    rig.setParam("eq/lowCut", 1);
}

/** DC 30 Hz, the low cut, and the de-esser's two filters summed back — the plain path. */
Cx plainPath(double fs, double f, double lowCutHz = 100.0) {
    using R = dsp::VocalRig;
    return webPass(true, 30, 0.7, fs, f) * webPass(true, lowCutHz, R::LowCutQDb, fs, f)
           * (std::pow(webPass(false, R::DeEssSplitHz, R::DeEssQDb, fs, f), 2) + std::pow(webPass(true, R::DeEssSplitHz, R::DeEssQDb, fs, f), 2));
}

} // namespace

class VocalRigTests : public juce::UnitTest {
public:
    VocalRigTests() : juce::UnitTest("VocalRig", "DSP") {}

    void runTest() override {
        using Rig = dsp::VocalRig;
        const double fs = 48000.0;
        const int block = 240; // 240 samples: whole cycles of 200 Hz, 1 kHz, 5 kHz, 8 kHz per block

        beginTest("off is untouched; plain is the DC block, the low cut and the de-esser's split summed, × √2");
        {
            juce::dsp::ConvolutionMessageQueue queue;
            Rig rig(queue);
            rig.prepare(fs, block);
            std::vector<float> x(static_cast<size_t>(block), 0.2f);
            const auto untouched = x;
            rig.process(x.data(), block);
            expect(x == untouched, "a rack that is off is not in the path");
            plain(rig);
            for (const double f : { 80.0, 200.0, 1000.0, 4000.0, 8000.0 }) {
                rig.reset();
                const double got = sineRms(rig, fs, f, 0.1, block, 200, 100);
                expectWithinAbsoluteError(toDb(got), toDb(0.1 * std::abs(plainPath(fs, f))), 0.05, juce::String(f) + " Hz");
            }
            // At 5 kHz the split used to be a null (one LP + one HP at one corner). Linkwitz-Riley sums flat.
            rig.reset();
            expectWithinAbsoluteError(toDb(sineRms(rig, fs, 5000.0, 0.1, block, 200, 100)), toDb(0.1 * std::abs(plainPath(fs, 5000.0))), 0.05, "5 kHz, the split itself");
            expect(std::abs(std::abs(std::pow(webPass(false, 5000, Rig::DeEssQDb, fs, 5000), 2) + std::pow(webPass(true, 5000, Rig::DeEssQDb, fs, 5000), 2)) - 1.0) < 1e-6, "LR4 sums to unity at its corner");
            // Low cut off: the web writes 20 Hz rather than taking the filter out.
            rig.setParam("eq/lowCut", 0);
            rig.reset();
            const double got = sineRms(rig, fs, 80.0, 0.1, block, 200, 100);
            expectWithinAbsoluteError(toDb(got), toDb(0.1 * std::abs(plainPath(fs, 80.0, 20.0))), 0.05, "80 Hz, low cut off");
        }

        beginTest("the de-esser's band carries the node's own makeup, as the web's does");
        {
            juce::dsp::ConvolutionMessageQueue queue;
            Rig rig(queue);
            rig.prepare(fs, block);
            plain(rig);
            rig.setParam("deesser/enabled", 1);
            rig.setParam("deesser/threshold", -32);
            rig.setParam("deesser/ratio", 4.5f);
            rig.reset();
            // −66 dBFS, far under the threshold: the band is at exactly its makeup, 0.6 × (1 − 1/4.5) × 32 dB.
            const double f = 8000.0, amp = 0.0005;
            const double got = sineRms(rig, fs, f, amp, block, 200, 100);
            const double makeup = std::pow(10.0, 0.6 * (1.0 - 1.0 / 4.5) * 32.0 / 20.0);
            const Cx expected = webPass(true, 30, 0.7, fs, f) * webPass(true, 100, Rig::LowCutQDb, fs, f)
                                * (std::pow(webPass(false, 5000, Rig::DeEssQDb, fs, f), 2) + makeup * std::pow(webPass(true, 5000, Rig::DeEssQDb, fs, f), 2));
            expectWithinAbsoluteError(toDb(got), toDb(amp * std::abs(expected)), 0.1);
        }

        beginTest("the delay: one tap at its time through the 2.5 kHz damping, added to the dry");
        {
            juce::dsp::ConvolutionMessageQueue queue;
            Rig rig(queue);
            rig.prepare(fs, block);
            plain(rig);
            rig.setParam("delay/enabled", 1);
            rig.setParam("delay/time", 100);   // 4800 samples
            rig.setParam("delay/feedback", 0);
            rig.setParam("delay/mix", 1);
            rig.reset();
            for (const double f : { 200.0, 1000.0, 3000.0 }) {
                rig.reset();
                const double got = sineRms(rig, fs, f, 0.1, block, 220, 120);
                const Cx tap = webPass(false, Rig::DelayDampHz, 1.0, fs, f) * std::pow(zAt(f, fs), 4800.0);
                expectWithinAbsoluteError(toDb(got), toDb(0.1 * std::abs(plainPath(fs, f) * (1.0 + tap))), 0.1, juce::String(f) + " Hz");
            }
        }

        beginTest("the reverb builds off the audio thread and rings; the limiter holds its ceiling");
        {
            juce::dsp::ConvolutionMessageQueue queue;
            Rig rig(queue);
            rig.prepare(fs, block);
            plain(rig);
            rig.setParam("reverb/enabled", 1);
            rig.setParam("reverb/mix", 0.5f);
            expect(!rig.reverbReady());
            std::vector<float> silence(static_cast<size_t>(block), 0.0f);
            bool ready = false;
            for (int t = 0; t < 400 && !ready; ++t) {
                rig.serviceBackground();
                rig.process(silence.data(), block);
                ready = rig.reverbReady();
                juce::Thread::sleep(5);
            }
            expect(ready, "the tail swapped in");
            std::vector<float> x(static_cast<size_t>(block), 0.0f);
            x[0] = 1.0f;
            rig.process(x.data(), block);
            double tail = 0;
            for (int b = 0; b < 40; ++b) {
                std::fill(x.begin(), x.end(), 0.0f);
                rig.process(x.data(), block);
                for (const float v : x) tail += v * v;
            }
            expect(tail > 1e-6, "still ringing 200 ms later");

            plain(rig);
            rig.setParam("limiter/enabled", 1);
            rig.setParam("limiter/ceiling", -6.0f);
            rig.setParam("output", 12.0f);
            rig.reset();
            float peak = 0;
            for (int b = 0; b < 100; ++b) {
                for (int i = 0; i < block; ++i) x[static_cast<size_t>(i)] = static_cast<float>(0.8 * std::sin(2.0 * kPi * 400.0 * (b * block + i) / fs));
                rig.process(x.data(), block);
                if (b > 20) for (const float v : x) peak = std::max(peak, std::abs(v));
            }
            expect(peak <= std::pow(10.0f, -6.0f / 20.0f) * 1.41421356f + 1e-4f, "peak " + juce::String(peak));
        }

        beginTest("the engine routes vocals/ to the channel's rack, and the desk's defaults sound");
        {
            // Exactly what `vocalDspParams(channel, DEFAULT_VOCALS)` sends, in that order.
            const std::pair<const char*, float> web[] = {
                { "vocals/enabled", 1 }, { "vocals/input", 0 }, { "vocals/gate/enabled", 1 }, { "vocals/gate/threshold", -60 },
                { "vocals/deesser/enabled", 1 }, { "vocals/deesser/threshold", -32 }, { "vocals/deesser/ratio", 4.5f },
                { "vocals/comp/enabled", 1 }, { "vocals/comp/threshold", -18 }, { "vocals/comp/ratio", 3.5f },
                { "vocals/comp/attack", 0.015f }, { "vocals/comp/release", 0.2f }, { "vocals/eq/lowCut", 1 },
                { "vocals/eq/body", 1 }, { "vocals/eq/presence", 2 }, { "vocals/eq/air", 3 }, { "vocals/delay/enabled", 0 },
                { "vocals/delay/time", 400 }, { "vocals/delay/feedback", 0.35f }, { "vocals/delay/mix", 0.2f },
                { "vocals/reverb/enabled", 1 }, { "vocals/reverb/size", 1.8f }, { "vocals/reverb/mix", 0.18f },
                { "vocals/output", 0 }, { "vocals/limiter/enabled", 1 }, { "vocals/limiter/ceiling", -0.3f },
            };
            dsp::MixingEngine engine;
            engine.prepare(fs, 256);
            for (const auto& [name, value] : web) expect(engine.setChannelParam(0, name, value), name);
            expect(!engine.setChannelParam(0, "vocals/nonsense", 1.0f));
            expect(engine.getChannel(0).vocals->isEnabled());
            std::vector<float> in(256);
            const float* inputs[] = { in.data() };
            std::vector<float> outL(256), outR(256);
            float* outputs[] = { outL.data(), outR.data() };
            double energy = 0.0;
            bool finite = true;
            for (int b = 0; b < 300; ++b) {
                engine.serviceBackgroundWork();
                for (int i = 0; i < 256; ++i) in[static_cast<size_t>(i)] = static_cast<float>(0.1 * std::sin(2.0 * kPi * 220.0 * (b * 256 + i) / fs));
                engine.processAudio(inputs, 1, outputs, 2, 256);
                if (b > 200) for (const float v : outL) { finite = finite && std::isfinite(v); energy += v * v; }
                juce::Thread::sleep(1);
            }
            expect(finite, "no NaN reaches the output");
            expect(energy > 1e-6, "a voice through the rack makes a sound");
            expect(engine.postControl("channel/32/vocals/deesser/threshold", -30.0f), "the longest name fits the queue");
        }
    }
};

static VocalRigTests vocalRigTests;
