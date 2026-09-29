#include "MixingEngine.h"
#include <juce_core/juce_core.h>
#include <cmath>
#include <complex>
#include <string>
#include <vector>
#include "TestHelpers.h"

/**
 * The per-channel control plane and the cookbook conversion.
 *
 * The EQ half is checked against **Robert Bristow-Johnson's cookbook formulas written out
 * below**, which is the definition the web desk's `BiquadFilterNode` uses â€” an independent
 * statement of what the numbers mean, not a restatement of `cookbookToSvf`. A test that
 * compared the engine against its own conversion would pass with the conversion wrong.
 */
namespace {

constexpr double kPi = 3.14159265358979323846;

enum class Rbj { Peak, LowShelf, HighShelf, HighPass };

/** |H(e^jw)| in dB of the cookbook biquad. */
double rbjDb(Rbj type, double f0, double q, double gainDb, double fs, double f) {
    const double A = std::pow(10.0, gainDb / 40.0);
    const double w0 = 2.0 * kPi * f0 / fs;
    const double c = std::cos(w0);
    const double alpha = std::sin(w0) / (2.0 * q);
    const double sa = 2.0 * std::sqrt(A) * alpha;
    double b0, b1, b2, a0, a1, a2;
    switch (type) {
        case Rbj::Peak:
            b0 = 1 + alpha * A; b1 = -2 * c; b2 = 1 - alpha * A;
            a0 = 1 + alpha / A; a1 = -2 * c; a2 = 1 - alpha / A;
            break;
        case Rbj::LowShelf:
            b0 = A * ((A + 1) - (A - 1) * c + sa); b1 = 2 * A * ((A - 1) - (A + 1) * c);
            b2 = A * ((A + 1) - (A - 1) * c - sa);
            a0 = (A + 1) + (A - 1) * c + sa; a1 = -2 * ((A - 1) + (A + 1) * c);
            a2 = (A + 1) + (A - 1) * c - sa;
            break;
        case Rbj::HighShelf:
            b0 = A * ((A + 1) + (A - 1) * c + sa); b1 = -2 * A * ((A - 1) + (A + 1) * c);
            b2 = A * ((A + 1) + (A - 1) * c - sa);
            a0 = (A + 1) - (A - 1) * c + sa; a1 = 2 * ((A - 1) - (A + 1) * c);
            a2 = (A + 1) - (A - 1) * c - sa;
            break;
        case Rbj::HighPass:
        default:
            b0 = (1 + c) / 2; b1 = -(1 + c); b2 = (1 + c) / 2;
            a0 = 1 + alpha; a1 = -2 * c; a2 = 1 - alpha;
            break;
    }
    const std::complex<double> z1 = std::polar(1.0, -2.0 * kPi * f / fs);
    const std::complex<double> z2 = z1 * z1;
    const auto h = (b0 + b1 * z1 + b2 * z2) / (a0 + a1 * z1 + a2 * z2);
    return 20.0 * std::log10(std::abs(h));
}

/** Steady-state gain in dB of `process` at frequency f, by quadrature over whole seconds. */
template <typename Fn>
double measureDb(Fn&& process, double fs, double f) {
    const int settle = static_cast<int>(fs * 0.5);
    const int window = static_cast<int>(fs * 0.5);
    double si = 0, co = 0;
    for (int n = 0; n < settle + window; ++n) {
        const double ph = 2.0 * kPi * f * n / fs;
        const float y = process(static_cast<float>(0.25 * std::sin(ph)));
        if (n >= settle) { si += y * std::sin(ph); co += y * std::cos(ph); }
    }
    const double amp = 2.0 / window * std::sqrt(si * si + co * co);
    return 20.0 * std::log10(amp / 0.25);
}

const double kProbeHz[] = { 40, 80, 120, 185, 250, 500, 1000, 2000, 4000, 8000, 12000, 16000 };

} // namespace

class ChannelParamTests : public juce::UnitTest {
public:
    ChannelParamTests() : juce::UnitTest("ChannelParams") {}

    void runTest() override {
        testRouting();
        testEqAgainstCookbook(48000.0);
        testEqAgainstCookbook(44100.0);
        testConversionIsNeeded();
        testFieldUpdatesKeepTheOthers();
        testSampleRateChangeRederives();
        testHighPass();
        testCompressorRouting();
    }

private:
    void testRouting() {
        beginTest("every wire path the web desk and the other modules use is accepted");
        dsp::MixingEngine engine;
        engine.prepare(48000.0, 256);
        const char* paths[] = {
            "fader", "trim", "pan", "mute", "invert",
            "hpf/enabled", "hpf/freq", "hpf/q", "lpf/enabled", "lpf/freq", "lpf/q",
            "eq/1/shape", "eq/1/freq", "eq/1/q", "eq/1/gain", "eq/1/enabled", "eq/6/gain",
            "comp/enabled", "comp/threshold", "comp/ratio", "comp/attack", "comp/release",
            "comp/knee", "comp/makeup", "comp/detection",
            "gate/enabled", "gate/threshold", "gate/ratio", "gate/attack", "gate/hold",
            "gate/release", "gate/range", "gate/hysteresis",
            "deesser/enabled", "deesser/threshold", "deesser/ratio", "deesser/freq",
        };
        for (auto* p : paths) {
            const float v = std::string(p) == "eq/1/shape" ? 1.0f : 0.0f;
            expect(engine.setChannelParam(0, p, v), juce::String("refused ") + p);
        }

        beginTest("unknown, out-of-range and non-finite are refused and change nothing");
        expect(!engine.setChannelParam(0, "eq/7/gain", 3.0f), "band 7 does not exist");
        expect(!engine.setChannelParam(0, "eq/0/gain", 3.0f), "bands are 1-based");
        expect(!engine.setChannelParam(0, "eq/1/colour", 3.0f), "unknown field");
        expect(!engine.setChannelParam(0, "eq/1/shape", 3.0f), "shape code 3 is not defined");
        expect(!engine.setChannelParam(0, "eq/1/shape", 1.5f), "a shape code is an integer");
        expect(!engine.setChannelParam(0, "reverb", 1.0f), "unknown parameter");
        expect(!engine.setChannelParam(32, "fader", 0.0f), "channel 33 does not exist");
        expect(!engine.setChannelParam(-1, "fader", 0.0f), "negative channel");
        engine.setChannelParam(0, "fader", -6.0f);
        expect(!engine.setChannelParam(0, "fader", std::nanf("")), "NaN refused");
        expect(!engine.setChannelParam(0, "fader", INFINITY), "inf refused");
        expectEquals(engine.getChannel(0).faderDb, -6.0f);

        beginTest("switches and existing controls land where they did before");
        engine.setChannelParam(2, "mute", 1.0f);
        expect(!engine.getChannel(2).routedToMain);
        engine.setChannelParam(2, "mute", 0.0f);
        expect(engine.getChannel(2).routedToMain);
        engine.setChannelParam(2, "invert", 1.0f);
        expect(engine.getChannel(2).phaseInvert);
        engine.setChannelParam(2, "hpf/enabled", 1.0f);
        expect(engine.getChannel(2).hpfEnabled);
        engine.setChannelParam(2, "fader", 400.0f);
        expectEquals(engine.getChannel(2).faderDb, dsp::MixingEngine::MaxFaderDb);
    }

    void checkBand(dsp::MixingEngine::EqShape shape, Rbj rbj, double f0, double q, double gainDb, double fs) {
        dsp::MixingEngine::EqBandSetting s;
        s.shape = shape; s.frequencyHz = static_cast<float>(f0); s.q = static_cast<float>(q);
        s.gainDb = static_cast<float>(gainDb); s.enabled = true;
        double worst = 0, worstAt = 0;
        for (double f : kProbeHz) {
            if (f > 0.4 * fs) continue;
            dsp::ParametricEQ eq;
            eq.prepare(fs, 256);
            eq.setBandParameters(0, dsp::MixingEngine::cookbookToSvf(s, fs));
            const double got = measureDb([&](float x) { return eq.processSample(x); }, fs, f);
            const double want = rbjDb(rbj, f0, q, gainDb, fs, f);
            if (std::abs(got - want) > worst) { worst = std::abs(got - want); worstAt = f; }
        }
        expect(worst <= 0.1, juce::String("worst ") + juce::String(worst, 3) + " dB at "
                                 + juce::String(worstAt) + " Hz");
    }

    void testEqAgainstCookbook(double fs) {
        using S = dsp::MixingEngine::EqShape;
        beginTest("peaking band matches the cookbook within 0.1 dB @ " + juce::String(fs));
        for (double g : { -15.0, -6.0, 3.0, 15.0 }) checkBand(S::Peaking, Rbj::Peak, 1000, 1.1, g, fs);
        checkBand(S::Peaking, Rbj::Peak, 250, 1.1, 9.0, fs);
        checkBand(S::Peaking, Rbj::Peak, 5000, 2.0, -12.0, fs);

        beginTest("low shelf matches the cookbook within 0.1 dB @ " + juce::String(fs));
        for (double g : { -15.0, -6.0, 6.0, 15.0 }) checkBand(S::LowShelf, Rbj::LowShelf, 120, 0.7071, g, fs);

        beginTest("high shelf matches the cookbook within 0.1 dB @ " + juce::String(fs));
        for (double g : { -15.0, -6.0, 6.0, 15.0 }) checkBand(S::HighShelf, Rbj::HighShelf, 8000, 0.7071, g, fs);
    }

    /**
     * The control: handing the cookbook numbers to the SVF unconverted is measurably wrong.
     * Without this, a conversion that did nothing and a correct one would both be "green".
     */
    void testConversionIsNeeded() {
        beginTest("without the conversion a 15 dB low shelf is off by more than 3 dB");
        const double fs = 48000.0;
        dsp::ParametricEQ eq;
        eq.prepare(fs, 256);
        dsp::ParametricEQ::BandConfig raw;
        raw.type = dsp::FilterPrimitives::Type::LowShelf;
        raw.frequencyHz = 120.0f; raw.q = 0.7071f; raw.gainDb = 15.0f; raw.enabled = true;
        eq.setBandParameters(0, raw);
        const double got = measureDb([&](float x) { return eq.processSample(x); }, fs, 185.0);
        const double want = rbjDb(Rbj::LowShelf, 120, 0.7071, 15, fs, 185.0);
        expect(std::abs(got - want) > 3.0, juce::String("unconverted differs by only ") + juce::String(std::abs(got - want)));

        beginTest("without the conversion a -15 dB peak is 2.4x too wide");
        dsp::ParametricEQ eq2;
        eq2.prepare(fs, 256);
        raw.type = dsp::FilterPrimitives::Type::Peaking;
        raw.frequencyHz = 1000.0f; raw.q = 1.1f; raw.gainDb = -15.0f;
        eq2.setBandParameters(0, raw);
        const double got2 = measureDb([&](float x) { return eq2.processSample(x); }, fs, 2000.0);
        const double want2 = rbjDb(Rbj::Peak, 1000, 1.1, -15, fs, 2000.0);
        expect(std::abs(got2 - want2) > 3.0, juce::String("unconverted differs by only ") + juce::String(std::abs(got2 - want2)));
    }

    /** A message carrying only a gain must not reset the frequency or Q set earlier. */
    void testFieldUpdatesKeepTheOthers() {
        beginTest("eq/<n>/gain alone keeps the band's frequency, Q and shape");
        const double fs = 48000.0;
        dsp::MixingEngine engine;
        engine.prepare(fs, 256);
        engine.setChannelParam(0, "eq/2/shape", 0.0f);
        engine.setChannelParam(0, "eq/2/freq", 2500.0f);
        engine.setChannelParam(0, "eq/2/q", 1.1f);
        engine.setChannelParam(0, "eq/2/enabled", 1.0f);
        engine.setChannelParam(0, "eq/2/gain", 9.0f);
        auto& eq = engine.getChannel(0).eq;
        const double got = measureDb([&](float x) { return eq.processSample(x); }, fs, 2500.0);
        expectWithinAbsoluteTolerance(got, 9.0, 0.1);
        const double off = measureDb([&](float x) { return eq.processSample(x); }, fs, 1000.0);
        expectWithinAbsoluteTolerance(off, rbjDb(Rbj::Peak, 2500, 1.1, 9, fs, 1000.0), 0.1);
    }

    void testSampleRateChangeRederives() {
        beginTest("a device change to 44.1 kHz re-derives every band from the stored settings");
        dsp::MixingEngine engine;
        engine.prepare(48000.0, 256);
        engine.setChannelParam(0, "eq/1/shape", 2.0f);
        engine.setChannelParam(0, "eq/1/freq", 8000.0f);
        engine.setChannelParam(0, "eq/1/q", 0.7071f);
        engine.setChannelParam(0, "eq/1/gain", 12.0f);
        engine.setChannelParam(0, "eq/1/enabled", 1.0f);
        engine.prepare(44100.0, 256);
        auto& eq = engine.getChannel(0).eq;
        for (double f : { 4000.0, 8000.0, 12000.0 }) {
            const double got = measureDb([&](float x) { return eq.processSample(x); }, 44100.0, f);
            expectWithinAbsoluteTolerance(got, rbjDb(Rbj::HighShelf, 8000, 0.7071, 12, 44100.0, f), 0.1);
        }
    }

    void testHighPass() {
        beginTest("hpf/freq and hpf/q are a cookbook high-pass with linear Q");
        const double fs = 48000.0;
        dsp::MixingEngine engine;
        engine.prepare(fs, 256);
        engine.setChannelParam(0, "hpf/freq", 100.0f);
        engine.setChannelParam(0, "hpf/q", 1.084f);
        engine.setChannelParam(0, "hpf/enabled", 1.0f);
        auto& hpf = engine.getChannel(0).hpf;
        for (double f : { 40.0, 100.0, 250.0 }) {
            const double got = measureDb([&](float x) { return hpf.processSample(x); }, fs, f);
            expectWithinAbsoluteTolerance(got, rbjDb(Rbj::HighPass, 100, 1.084, 0, fs, f), 0.1);
        }
    }

    void testCompressorRouting() {
        beginTest("comp/* reaches the compressor in wire units (ms, dB), knee 0, peak detection");
        dsp::MixingEngine engine;
        engine.prepare(48000.0, 256);
        engine.setChannelParam(0, "comp/threshold", -18.0f);
        engine.setChannelParam(0, "comp/ratio", 3.0f);
        engine.setChannelParam(0, "comp/attack", 10.0f);   // milliseconds on the wire
        engine.setChannelParam(0, "comp/release", 150.0f);
        engine.setChannelParam(0, "comp/knee", 0.0f);
        engine.setChannelParam(0, "comp/detection", 0.0f); // peak
        engine.setChannelParam(0, "comp/makeup", 4.0f);
        engine.setChannelParam(0, "comp/enabled", 1.0f);
        auto& comp = engine.getChannel(0).comp;
        float y = 0;
        // 10 ms attack is a time constant: after 200 ms the gain has settled completely.
        for (int i = 0; i < 9600; ++i) y = comp.processSample(0.5f);
        const double inDb = 20.0 * std::log10(0.5);
        const double wantDb = inDb + (1.0 / 3.0 - 1.0) * (inDb + 18.0) + 4.0;
        expectWithinAbsoluteTolerance(20.0 * std::log10(y), wantDb, 0.05);

        beginTest("comp/enabled 0 is a bypass");
        engine.setChannelParam(0, "comp/enabled", 0.0f);
        expectEquals(comp.processSample(0.5f), 0.5f);
    }
};

static ChannelParamTests channelParamTests;

