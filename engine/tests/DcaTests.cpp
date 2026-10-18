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

        beginTest("the strip EQ is after the rack insert: a 12 dB cut is 12 dB at the master, rack or not (1.0.27)");
        {
            auto run = [this](float eqDb, bool rack) {
                auto e = std::make_unique<dsp::MixingEngine>();
                e->prepare(48000.0, 256);
                expect(e->setControl("/channel/1/input", 0.0f));
                expect(e->setControl("/channel/1/eq/2/shape", 0.0f));
                expect(e->setControl("/channel/1/eq/2/freq", 440.0f));
                expect(e->setControl("/channel/1/eq/2/q", 1.0f));
                expect(e->setControl("/channel/1/eq/2/gain", eqDb));
                expect(e->setControl("/channel/1/eq/2/enabled", 1.0f));
                if (rack) expect(e->setControl("/channel/1/drums/enabled", 1.0f));
                for (int i = 0; i < 4; ++i) e->serviceControls();
                return masterRmsDb(*e);
            };
            for (const bool rack : { false, true }) {
                const double flat = run(0.0f, rack);
                const double cut = run(-12.0f, rack);
                expect(flat > -60.0, "signal reaches the master");
                expectWithinAbsoluteTolerance(flat - cut, 12.0, 0.6);
            }
        }

        beginTest("subgroups: gain and mute scale the main bus only, never a post-fader send");
        {
            auto e = std::make_unique<dsp::MixingEngine>();
            e->prepare(48000.0, 256);
            expectWithinAbsoluteTolerance(e->groupGainFor(0), 1.0, 1e-9);
            expect(e->setControl("/channel/1/group", 2.0f));
            expect(e->setControl("/group/2/gain", -6.0f));
            expectWithinAbsoluteTolerance(e->groupGainFor(0), std::pow(10.0, -6.0 / 20.0), 1e-5);
            expectWithinAbsoluteTolerance(e->groupGainFor(1), 1.0, 1e-9);
            expect(e->setControl("/group/2/mute", 1.0f));
            expectWithinAbsoluteTolerance(e->groupGainFor(0), 0.0, 1e-9);
            expect(e->setControl("/channel/1/group", 0.0f));
            expectWithinAbsoluteTolerance(e->groupGainFor(0), 1.0, 1e-9);

            expect(!e->setControl("/channel/1/group", 9.0f));
            expect(!e->setControl("/channel/1/group", 1.5f));
            expect(!e->setControl("/channel/1/group", -1.0f));
            expect(!e->setControl("/group/9/gain", -3.0f));
            expect(!e->setControl("/group/0/gain", -3.0f));
            expect(!e->setControl("/group/1/pan", 0.0f));
            expect(e->setControl("/group/8/gain", -3.0f));
        }

        beginTest("the audio really moves: a −6 dB subgroup is 6 dB down at the master, the send is not");
        {
            auto run = [this](float groupDb, bool mute, bool viaAux) {
                auto e = std::make_unique<dsp::MixingEngine>();
                e->prepare(48000.0, 256);
                expect(e->setControl("/channel/1/input", 0.0f));
                expect(e->setControl("/channel/1/group", 1.0f));
                expect(e->setControl("/group/1/gain", groupDb));
                if (mute) expect(e->setControl("/group/1/mute", 1.0f));
                if (viaAux) {
                    expect(e->setControl("/channel/1/send/aux/1", 0.0f));
                    expect(e->setControl("/channel/1/send/aux/1/pre", 0.0f));
                    expect(e->setControl("/output/1/source", 7.0f));
                }
                for (int i = 0; i < 4; ++i) e->serviceControls();
                return masterRmsDb(*e);
            };
            const double flat = run(0.0f, false, false);
            const double down = run(-6.0f, false, false);
            expect(flat > -60.0, "the channel reaches the master at all: " + juce::String(flat));
            expectWithinAbsoluteTolerance(flat - down, 6.0, 0.2);
            expect(run(0.0f, true, false) < -100.0, "a muted subgroup is silence at the master");
            const double auxFlat = run(0.0f, false, true);
            const double auxMuted = run(0.0f, true, true);
            expect(auxFlat > -60.0, "the post-fader send reaches aux 1: " + juce::String(auxFlat));
            expectWithinAbsoluteTolerance(auxFlat - auxMuted, 0.0, 0.2);
        }
    }
};

static DcaTests dcaTests;
