#include "MixingEngine.h"
#include "LowEnd.h"
#include <juce_core/juce_core.h>
#include <cmath>
#include <memory>
#include <vector>
#include "TestHelpers.h"

/**
 * The sidechain duck and mono bass — the web desk's `check-lowend.js`, against the engine.
 *
 * Measured, not reasoned: a synthesised kick through the real detector, the real mono stage,
 * and a whole engine with the key on a *later* channel than the target, which is the case a
 * naive one-pass loop gets a block late.
 */
class LowEndTests : public juce::UnitTest {
public:
    LowEndTests() : juce::UnitTest("LowEnd") {}

    static constexpr double kRate = 48000.0;
    static constexpr float kPi = 3.14159265f;

    static float db(float g) { return 20.0f * std::log10(std::max(g, 1e-9f)); }

    /** A 55 Hz kick peaking at 0.5, decaying over ~150 ms, one hit per 500 ms. */
    static float kick(int t) {
        const float s = (float) (t % (int) (kRate / 2)) / (float) kRate;
        return 0.5f * std::exp(-s / 0.08f) * std::sin(2.0f * kPi * 55.0f * s);
    }

    void runTest() override {
        beginTest("the arithmetic: unity at rest, exactly the depth at a full hit, never negative");
        {
            expectEquals(dsp::SidechainDuck::duckGain(0.0f, 12.0f), 1.0f);
            for (float depth : { 0.0f, 3.0f, 6.0f, 12.0f, 18.0f }) {
                expectWithinAbsoluteTolerance(db(dsp::SidechainDuck::duckGain(50.0f, depth)), -depth, 1e-4f);
                for (float env : { 0.0f, 0.05f, 0.2f, 1.0f, 1000.0f }) {
                    const float g = dsp::SidechainDuck::duckGain(env, depth);
                    expect(g > 0.0f && g <= 1.0f, "the duck gain stays in (0, 1]");
                }
            }
            dsp::SidechainDuck d;
            d.setDepthDb(400.0f);
            expectEquals(d.getDepthDb(), dsp::SidechainDuck::MaxDepthDb);
            d.setDepthDb(std::nanf(""));
            expectEquals(d.getDepthDb(), 0.0f);
        }

        beginTest("a −6 dBFS kick ducks the target by the full depth and lets it back before the next beat");
        {
            dsp::SidechainDuck d;
            d.prepare(kRate, 256);
            d.setDepthDb(6.0f);
            std::vector<float> block(256);
            float minG = 1.0f, last = 0.0f;
            const int total = (int) kRate * 2;
            for (int t = 0; t < total; t += 256) {
                for (int i = 0; i < 256; ++i) block[(size_t) i] = kick(t + i);
                d.processKey(block.data(), 256);
                for (int i = 0; i < 256; ++i) {
                    if (t + i > (int) (kRate * 1.5)) minG = std::min(minG, d.gainAt(i));
                    last = d.gainAt(i);
                }
            }
            expectWithinAbsoluteTolerance(db(minG), -6.0f, 0.6f);
            expect(last > 0.95f, "recovered before the next beat");
        }

        beginTest("a hi-hat as loud as the kick barely moves it — the key is band-limited");
        {
            dsp::SidechainDuck d;
            d.prepare(kRate, 256);
            d.setDepthDb(6.0f);
            std::vector<float> block(256);
            float minG = 1.0f;
            for (int t = 0; t < (int) kRate / 2; t += 256) {
                for (int i = 0; i < 256; ++i) block[(size_t) i] = 0.5f * std::sin(2.0f * kPi * 8000.0f * (float) (t + i) / (float) kRate);
                d.processKey(block.data(), 256);
                for (int i = 0; i < 256; ++i) minG = std::min(minG, d.gainAt(i));
            }
            expect(db(minG) > -0.5f, "a hat does not pump the bass");
        }

        beginTest("mono bass: a mono signal is untouched, low side is removed, high side passes");
        {
            dsp::MonoBass m;
            m.prepare(kRate, 512);
            m.setFrequency(120.0f);
            m.setEnabled(true);
            std::vector<float> l(4800), r(4800);
            for (int i = 0; i < 4800; ++i) l[(size_t) i] = r[(size_t) i] = 0.3f * std::sin(2.0f * kPi * 50.0f * (float) i / (float) kRate);
            const auto lc = l, rc = r;
            m.process(l.data(), r.data(), 4800);
            float worst = 0.0f;
            for (int i = 0; i < 4800; ++i) worst = std::max(worst, std::abs(l[(size_t) i] - lc[(size_t) i]) + std::abs(r[(size_t) i] - rc[(size_t) i]));
            expect(worst < 1e-6f, "mono material is bit-identical");

            auto sideRms = [&](float hz) {
                dsp::MonoBass mb;
                mb.prepare(kRate, 512);
                mb.setFrequency(120.0f);
                mb.setEnabled(true);
                std::vector<float> a(9600), b(9600);
                for (int i = 0; i < 9600; ++i) {
                    const float s = 0.3f * std::sin(2.0f * kPi * hz * (float) i / (float) kRate);
                    a[(size_t) i] = s;
                    b[(size_t) i] = -s;
                }
                mb.process(a.data(), b.data(), 9600);
                double acc = 0;
                for (int i = 4800; i < 9600; ++i) acc += 0.25 * (double) (a[(size_t) i] - b[(size_t) i]) * (a[(size_t) i] - b[(size_t) i]);
                return (float) std::sqrt(acc / 4800.0) / (0.3f / std::sqrt(2.0f));
            };
            expect(db(sideRms(40.0f)) < -15.0f, "a 40 Hz stereo difference is mostly gone");
            expectWithinAbsoluteTolerance(db(sideRms(2000.0f)), 0.0f, 0.3f);

            m.setFrequency(5.0f);
            expectEquals(m.getFrequency(), dsp::MonoBass::MinHz);
        }

        beginTest("in the engine: the key on a later channel still ducks the target in the same block");
        {
            auto run = [&](bool sidechainOn) {
                auto e = std::make_unique<dsp::MixingEngine>();
                e->prepare(kRate, 256);
                e->getMaster().limiter.setEnabled(false);
                for (int c = 0; c < dsp::MixingEngine::MaxChannels; ++c) e->setChannelParam(c, "input", -1.0f);
                // Target is channel 1 (index 0), panned right; key is channel 2 (index 1), panned left.
                e->setChannelParam(0, "input", 1.0f);
                e->setChannelParam(0, "pan", 1.0f);
                e->setChannelParam(1, "input", 0.0f);
                e->setChannelParam(1, "pan", -1.0f);
                expect(e->setControl("master/sidechain/key", 2.0f));
                expect(e->setControl("master/sidechain/target", 1.0f));
                expect(e->setControl("master/sidechain/depth", 9.0f));
                expect(e->setControl("master/sidechain/enabled", sidechainOn ? 1.0f : 0.0f));
                std::vector<float> in0(256), in1(256), o0(256), o1(256);
                const float* ins[2] = { in0.data(), in1.data() };
                float* outs[2] = { o0.data(), o1.data() };
                float peakAtHit = 0.0f;
                for (int t = 0; t < (int) kRate * 2; t += 256) {
                    for (int i = 0; i < 256; ++i) {
                        in0[(size_t) i] = kick(t + i);
                        in1[(size_t) i] = 0.2f * std::sin(2.0f * kPi * 80.0f * (float) (t + i) / (float) kRate);
                    }
                    e->processAudio(ins, 2, outs, 2, 256);
                    const int phase = t % (int) (kRate / 2);
                    if (t > (int) kRate && phase >= 1024 && phase < 2048)
                        for (float v : o1) peakAtHit = std::max(peakAtHit, std::abs(v));
                }
                return peakAtHit;
            };
            const float off = run(false);
            const float on = run(true);
            expect(off > 0.0f, "the bass reaches the right output");
            expect(db(on) - db(off) < -6.0f, "on a hit the bass is ducked by most of the 9 dB");
        }

        beginTest("the control plane rejects nonsense rather than wiring it");
        {
            auto e = std::make_unique<dsp::MixingEngine>();
            e->prepare(kRate, 256);
            expect(e->setControl("master/sidechain/key", 0.0f));
            expectEquals(e->getMaster().sidechainKey, -1);
            expect(e->setControl("master/sidechain/key", 99.0f));
            expectEquals(e->getMaster().sidechainKey, -1);
            e->setControl("master/sidechain/key", 3.0f);
            e->setControl("master/sidechain/target", 3.0f);
            e->setControl("master/sidechain/enabled", 1.0f);
            expect(!e->getMaster().sidechainActive(), "a channel cannot duck itself");
            expect(e->setControl("master/monobass/enabled", 1.0f));
            expect(e->getMaster().monoBass.isEnabled());
        }
    }
};

static LowEndTests lowEndTests;
