#include "SpeakerProcessor.h"
#include "MixingEngine.h"
#include <juce_core/juce_core.h>
#include <cmath>
#include <memory>
#include <vector>
#include "TestHelpers.h"

namespace {
constexpr double kFs = 48000.0;
constexpr double kPi = 3.14159265358979323846;

/** Steady-state gain in dB of a sine at `hz` through `sp`, measured after the filters settle. */
float gainDbAt(dsp::SpeakerProcessor& sp, float hz, float amplitude = 0.25f) {
    sp.reset();
    const int n = 48000;
    std::vector<float> x(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) x[static_cast<size_t>(i)] = amplitude * static_cast<float>(std::sin(2.0 * kPi * hz * i / kFs));
    std::vector<float> y = x;
    sp.process(y.data(), n);
    double in = 0.0, out = 0.0;
    for (int i = n / 2; i < n; ++i) {
        in += static_cast<double>(x[static_cast<size_t>(i)]) * x[static_cast<size_t>(i)];
        out += static_cast<double>(y[static_cast<size_t>(i)]) * y[static_cast<size_t>(i)];
    }
    return static_cast<float>(10.0 * std::log10(out / in));
}

std::unique_ptr<dsp::SpeakerProcessor> fresh() {
    auto sp = std::make_unique<dsp::SpeakerProcessor>();
    sp->prepare(kFs, 512);
    return sp;
}
} // namespace

class SpeakerProcessorTests : public juce::UnitTest {
public:
    SpeakerProcessorTests() : juce::UnitTest("SpeakerProcessor") {}

    void runTest() override {
        beginTest("untouched is bit-exact pass-through");
        {
            auto sp = fresh();
            expect(!sp->isActive());
            std::vector<float> x { 0.1f, -0.7f, 0.33f, 1.2f, -1.5f };
            auto y = x;
            sp->process(y.data(), static_cast<int>(y.size()));
            for (size_t i = 0; i < x.size(); ++i) expect(y[i] == x[i]);
        }

        beginTest("Butterworth section Qs");
        {
            float qs[4] {};
            expectEquals(dsp::SpeakerProcessor::butterworthQs(2, qs), 1);
            expectWithinAbsoluteTolerance(qs[0], 0.70711, 1e-4);
            expectEquals(dsp::SpeakerProcessor::butterworthQs(4, qs), 2);
            expectWithinAbsoluteTolerance(qs[0], 1.30656, 1e-4);
            expectWithinAbsoluteTolerance(qs[1], 0.54120, 1e-4);
            expectEquals(dsp::SpeakerProcessor::butterworthQs(3, qs), 1);
            expectWithinAbsoluteTolerance(qs[0], 1.0, 1e-4);
        }

        beginTest("Butterworth 24 dB/oct high-pass: -3 dB at the corner, ~-24 dB an octave below");
        {
            auto sp = fresh();
            expect(sp->setParam("hpf/type", 1.0f));
            expect(sp->setParam("hpf/slope", 24.0f));
            expect(sp->setParam("hpf/freq", 100.0f));
            expectWithinAbsoluteTolerance(gainDbAt(*sp, 100.0f), -3.01, 0.15);
            expectWithinAbsoluteTolerance(gainDbAt(*sp, 50.0f), -24.1, 0.4);
            expectWithinAbsoluteTolerance(gainDbAt(*sp, 1000.0f), 0.0, 0.05);
        }

        beginTest("every Butterworth slope is -3 dB at its corner, every Linkwitz-Riley -6 dB");
        {
            for (int slope : { 6, 12, 18, 24, 36, 48 }) {
                auto sp = fresh();
                sp->setParam("lpf/type", 1.0f);
                sp->setParam("lpf/slope", static_cast<float>(slope));
                sp->setParam("lpf/freq", 1000.0f);
                expectEquals(sp->settings().lpf.slopeDbPerOct, slope);
                expectWithinAbsoluteTolerance(gainDbAt(*sp, 1000.0f), -3.01, 0.2);
            }
            for (int slope : { 12, 24, 36, 48 }) {
                auto sp = fresh();
                sp->setParam("lpf/type", 2.0f);
                sp->setParam("lpf/slope", static_cast<float>(slope));
                sp->setParam("lpf/freq", 1000.0f);
                expectEquals(sp->settings().lpf.slopeDbPerOct, slope);
                expectWithinAbsoluteTolerance(gainDbAt(*sp, 1000.0f), -6.02, 0.2);
            }
        }

        beginTest("Linkwitz-Riley takes only even orders");
        {
            auto sp = fresh();
            sp->setParam("hpf/type", 2.0f);
            sp->setParam("hpf/slope", 18.0f);
            expectEquals(sp->settings().hpf.slopeDbPerOct, 24);
            sp->setParam("hpf/slope", 6.0f);
            expectEquals(sp->settings().hpf.slopeDbPerOct, 12);
        }

        beginTest("an LR24 sub and top at one corner sum flat (an all-pass)");
        {
            auto sub = fresh();
            auto top = fresh();
            sub->setParam("lpf/type", 2.0f); sub->setParam("lpf/slope", 24.0f); sub->setParam("lpf/freq", 100.0f);
            top->setParam("hpf/type", 2.0f); top->setParam("hpf/slope", 24.0f); top->setParam("hpf/freq", 100.0f);
            for (float hz : { 40.0f, 70.0f, 100.0f, 140.0f, 300.0f }) {
                const int n = 48000;
                std::vector<float> x(n), a(n), b(n);
                for (int i = 0; i < n; ++i) x[static_cast<size_t>(i)] = 0.25f * static_cast<float>(std::sin(2.0 * kPi * hz * i / kFs));
                a = x; b = x;
                sub->reset(); top->reset();
                sub->process(a.data(), n);
                top->process(b.data(), n);
                double in = 0.0, out = 0.0;
                for (int i = n / 2; i < n; ++i) {
                    const double s = static_cast<double>(a[static_cast<size_t>(i)]) + b[static_cast<size_t>(i)];
                    in += static_cast<double>(x[static_cast<size_t>(i)]) * x[static_cast<size_t>(i)];
                    out += s * s;
                }
                expectWithinAbsoluteTolerance(10.0 * std::log10(out / in), 0.0, 0.1);
            }
        }

        beginTest("a parametric band moves its frequency by its gain");
        {
            auto sp = fresh();
            expect(sp->setParam("eq/2/freq", 1000.0f));
            expect(sp->setParam("eq/2/q", 1.0f));
            expect(sp->setParam("eq/2/gain", 6.0f));
            expect(sp->setParam("eq/2/enabled", 1.0f));
            expect(sp->isActive());
            expectWithinAbsoluteTolerance(gainDbAt(*sp, 1000.0f), 6.0, 0.3);
            expectWithinAbsoluteTolerance(gainDbAt(*sp, 100.0f), 0.0, 0.3);
        }

        beginTest("polarity flips the sign exactly, mute writes silence");
        {
            auto sp = fresh();
            sp->setParam("polarity", 1.0f);
            std::vector<float> y { 0.5f, -0.25f, 0.125f };
            sp->process(y.data(), 3);
            expect(y[0] == -0.5f && y[1] == 0.25f && y[2] == -0.125f);
            sp->setParam("mute", 1.0f);
            sp->process(y.data(), 3);
            expect(y[0] == 0.0f && y[1] == 0.0f && y[2] == 0.0f);
        }

        beginTest("the limiter holds every sample under its threshold, with no latency");
        {
            auto sp = fresh();
            sp->setParam("limiter/enabled", 1.0f);
            sp->setParam("limiter/threshold", -6.0f);
            const int n = 4800;
            std::vector<float> y(n);
            for (int i = 0; i < n; ++i) y[static_cast<size_t>(i)] = static_cast<float>(std::sin(2.0 * kPi * 100.0 * i / kFs));
            sp->process(y.data(), n);
            float peak = 0.0f;
            for (float v : y) peak = std::max(peak, std::abs(v));
            expect(peak <= std::pow(10.0f, -6.0f / 20.0f) + 1.0e-6f, "peak " + juce::String(peak));
            expectWithinAbsoluteTolerance(sp->gainReductionDb(), 6.0, 0.1);

            // An impulse under the threshold comes out on the same sample it went in on.
            auto quiet = fresh();
            quiet->setParam("limiter/enabled", 1.0f);
            quiet->setParam("limiter/threshold", -6.0f);
            std::vector<float> imp(16, 0.0f);
            imp[3] = 0.1f;
            quiet->process(imp.data(), 16);
            expect(imp[3] == 0.1f && imp[2] == 0.0f && imp[4] == 0.0f);
        }

        beginTest("refuses what is not a control");
        {
            auto sp = fresh();
            expect(!sp->setParam("hpf/type", 3.0f));
            expect(!sp->setParam("hpf/type", 1.5f));
            expect(!sp->setParam("eq/7/gain", 3.0f));
            expect(!sp->setParam("eq/0/gain", 3.0f));
            expect(!sp->setParam("bogus", 1.0f));
            expect(!sp->setParam("gain", std::nanf("")));
            expect(!sp->isActive());
        }

        beginTest("the engine routes output/<n>/speaker and output/<n>/delay");
        {
            auto engine = std::make_unique<dsp::MixingEngine>();
            engine->prepare(kFs, 512);
            expect(engine->setControl("/output/3/delay", 10.0f));
            expectWithinAbsoluteTolerance(engine->outputDelay(2), 10.0, 1e-6);
            expect(engine->setControl("/output/3/delay", 900.0f));
            expectWithinAbsoluteTolerance(engine->outputDelay(2), dsp::MixingEngine::MaxOutputDelayMs, 1e-6);
            expect(engine->setControl("/output/1/speaker/hpf/type", 2.0f));
            expect(engine->speaker(0).settings().hpf.kind == dsp::SpeakerProcessor::FilterKind::LinkwitzRiley);
            expect(engine->setControl("/output/32/speaker/limiter/enabled", 1.0f));
            expect(!engine->setControl("/output/33/speaker/gain", -3.0f));
            expect(!engine->setControl("/output/0/delay", 1.0f));
            expect(!engine->setControl("/output/2/speaker/nothing", 1.0f));
            expect(engine->setControl("/output/2/source", 1.0f));
        }
    }
};

static SpeakerProcessorTests speakerProcessorTests;
