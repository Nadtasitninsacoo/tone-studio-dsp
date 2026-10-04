#include "MixingEngine.h"
#include <juce_core/juce_core.h>
#include <cmath>
#include <memory>
#include <vector>
#include "TestHelpers.h"

class DcaTests : public juce::UnitTest {
public:
    DcaTests() : juce::UnitTest("DCA") {}

    static double masterRmsDb(dsp::MixingEngine& e) {
        constexpr int block = 256;
        std::vector<float> in(block);
        const float* ip[1] = { in.data() };
        std::vector<std::vector<float>> st(2, std::vector<float>(block, 0.0f));
        float* op[2] = { st[0].data(), st[1].data() };
        double sum = 0.0;
        int count = 0;
        for (int b = 0; b < 60; ++b) {
            for (int i = 0; i < block; ++i) in[static_cast<size_t>(i)] = 0.1f * static_cast<float>(std::sin(2.0 * 3.14159265358979 * 440.0 * (b * block + i) / 48000.0));
            e.processAudio(ip, 1, op, 2, block);
            if (b < 30) continue; // let every ramp settle
            for (int i = 0; i < block; ++i) { sum += static_cast<double>(st[0][static_cast<size_t>(i)]) * st[0][static_cast<size_t>(i)]; ++count; }
        }
        return 10.0 * std::log10(sum / count + 1e-30);
    }

    void runTest() override {
        beginTest("a channel in no DCA is untouched; one in a DCA follows it, its own fader does not move");
        {
            auto e = std::make_unique<dsp::MixingEngine>();
            e->prepare(48000.0, 256);
            expectWithinAbsoluteTolerance(e->dcaGainFor(0), 1.0, 1e-9);
            expect(e->setControl("/channel/1/dca", 1.0f));
            expect(e->setControl("/dca/1/gain", -6.0f));
            expectWithinAbsoluteTolerance(e->dcaGainFor(0), std::pow(10.0, -6.0 / 20.0), 1e-5);
            expectWithinAbsoluteTolerance(e->dcaGainFor(1), 1.0, 1e-9);
            expect(e->setControl("/channel/1/dca", 3.0f)); // DCA 1 and 2
            expect(e->setControl("/dca/2/gain", -6.0f));
            expectWithinAbsoluteTolerance(e->dcaGainFor(0), std::pow(10.0, -12.0 / 20.0), 1e-5);
            expect(e->setControl("/dca/2/mute", 1.0f));
            expectWithinAbsoluteTolerance(e->dcaGainFor(0), 0.0, 1e-9);
            expect(e->dca(1).muted && !e->dca(0).muted);
        }

        beginTest("the control plane refuses what is not a DCA");
        {
            auto e = std::make_unique<dsp::MixingEngine>();
            e->prepare(48000.0, 256);
            expect(!e->setControl("/channel/1/dca", 256.0f));
            expect(!e->setControl("/channel/1/dca", 1.5f));
            expect(!e->setControl("/channel/1/dca", -1.0f));
            expect(!e->setControl("/dca/9/gain", -3.0f));
            expect(!e->setControl("/dca/0/gain", -3.0f));
            expect(!e->setControl("/dca/1/pan", 0.0f));
            expect(e->setControl("/dca/8/gain", -3.0f));
        }

        beginTest("the audio really moves: a −6 dB DCA is 6 dB down at the master, a muted one is silence");
        {
            auto run = [this](float dcaDb, bool mute) {
                auto e = std::make_unique<dsp::MixingEngine>();
                e->prepare(48000.0, 256);
                expect(e->setControl("/channel/1/input", 0.0f));
                expect(e->setControl("/channel/1/dca", 1.0f));
                expect(e->setControl("/dca/1/gain", dcaDb));
                if (mute) expect(e->setControl("/dca/1/mute", 1.0f));
                for (int i = 0; i < 4; ++i) e->serviceControls();
                return masterRmsDb(*e);
            };
            const double flat = run(0.0f, false);
            const double down = run(-6.0f, false);
            expect(flat > -60.0, "the channel reaches the master at all: " + juce::String(flat));
            expectWithinAbsoluteTolerance(flat - down, 6.0, 0.2);
            expect(run(0.0f, true) < -100.0, "a muted DCA is silence");
        }
    }
};

static DcaTests dcaTests;
