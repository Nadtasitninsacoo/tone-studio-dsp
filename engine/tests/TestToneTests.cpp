#include "MixingEngine.h"
#include "TestHelpers.h"
#include <juce_core/juce_core.h>
#include <cmath>
#include <memory>
#include <vector>

/**
 * The engine's own test tones — what a soundcheck needs to find which cable goes where.
 *
 * Before these, the only per-jack test played out of the browser's sound device, so on the engine
 * path (the stage path) no output could be made to beep at all, and the page sent people to a
 * monitor test that also played out of the browser.
 */
class TestToneTests : public juce::UnitTest
{
public:
    TestToneTests() : juce::UnitTest ("TestTones") {}

    static constexpr double kRate = 48000.0;
    static constexpr int kBlock = 256;

    struct Run
    {
        std::vector<std::vector<float>> out;   // whole recording per output
    };

    /** Run `seconds` of silence in, `outs` outputs out, recording every output. */
    static Run run (dsp::MixingEngine& e, int outs, double seconds)
    {
        Run r;
        r.out.assign ((size_t) outs, {});
        std::vector<float> in ((size_t) kBlock, 0.0f);
        const float* inPtrs[1] = { in.data() };
        std::vector<std::vector<float>> buf ((size_t) outs, std::vector<float> ((size_t) kBlock));
        std::vector<float*> ptrs;
        for (auto& b : buf) ptrs.push_back (b.data());
        const int blocks = (int) (seconds * kRate / kBlock);
        for (int b = 0; b < blocks; ++b)
        {
            e.processAudio (inPtrs, 1, ptrs.data(), outs, kBlock);
            for (int o = 0; o < outs; ++o)
                r.out[(size_t) o].insert (r.out[(size_t) o].end(), buf[(size_t) o].begin(), buf[(size_t) o].end());
        }
        return r;
    }
    static float peak (const std::vector<float>& v)
    {
        float p = 0.0f;
        for (float x : v) p = std::max (p, std::abs (x));
        return p;
    }
    /** Separate bursts: runs of audible samples split by at least 50 ms of quiet. */
    static int bursts (const std::vector<float>& v)
    {
        int count = 0, quiet = 1 << 30;
        for (float x : v)
        {
            if (std::abs (x) > 0.005f) { if (quiet > (int) (0.05 * kRate)) ++count; quiet = 0; }
            else ++quiet;
        }
        return count;
    }
    /** Pitch from zero crossings over the loudest stretch. */
    static double pitch (const std::vector<float>& v)
    {
        // A crossing sample is near zero by definition, so loudness is judged by the last loud
        // sample nearby, not by the crossing itself.
        int crossings = 0, first = -1, last = -1, lastLoud = -100000;
        for (size_t i = 1; i < v.size(); ++i)
        {
            if (std::abs (v[i]) > 0.005f) lastLoud = (int) i;
            if ((int) i - lastLoud < 2000 && (v[i - 1] < 0.0f) != (v[i] < 0.0f))
            {
                if (first < 0) first = (int) i;
                last = (int) i;
                ++crossings;
            }
        }
        if (crossings < 4) return 0.0;
        return (crossings - 1) / 2.0 / ((last - first) / kRate);
    }

    void runTest() override
    {
        beginTest ("An output that nothing is routed to beeps its own number of times, and only it does");
        {
            auto e = std::make_unique<dsp::MixingEngine>();
            e->prepare (kRate, kBlock);
            expect (e->setControl ("output/5/test", 1.0f));
            const auto r = run (*e, 8, 2.5);
            expectEquals (bursts (r.out[4]), 5, "output 5 beeps five times");
            expect (peak (r.out[4]) > 0.04f && peak (r.out[4]) < 0.07f, "at about -24 dBFS: " + juce::String (peak (r.out[4])));
            for (int o = 0; o < 8; ++o)
                if (o != 4) expectEquals (peak (r.out[(size_t) o]), 0.0f, "output " + juce::String (o + 1) + " stays silent");
            expect (! e->isOutputTesting(), "and the test ends by itself");
        }

        beginTest ("A routed output carries its programme and the beeps; output 9 and up gets one long tone");
        {
            auto e = std::make_unique<dsp::MixingEngine>();
            e->prepare (kRate, kBlock);
            expect (e->setControl ("output/12/test", 1.0f));
            const auto r = run (*e, 16, 2.0);
            expectEquals (bursts (r.out[11]), 1);
            expectEquals (peak (r.out[10]), 0.0f, "the jack beside it is silent");
        }

        beginTest ("The tone goes through the jack's speaker processor, at a pitch inside its pass band");
        {
            // A sub behind an 80 Hz low-pass must still beep — at a low pitch — not be silent and
            // read as a dead cable; and a jack's protection is never bypassed.
            auto e = std::make_unique<dsp::MixingEngine>();
            e->prepare (kRate, kBlock);
            expect (e->setControl ("output/3/speaker/lpf/type", 2.0f));
            expect (e->setControl ("output/3/speaker/lpf/freq", 80.0f));
            expect (e->setControl ("output/3/speaker/hpf/type", 1.0f));
            expect (e->setControl ("output/3/speaker/hpf/freq", 30.0f));
            expect (e->setControl ("output/3/test", 1.0f));
            const auto r = run (*e, 4, 1.5);
            const double hz = pitch (r.out[2]);
            expect (hz > 30.0 && hz < 80.0, "a sub's beep sits between its corners: " + juce::String (hz) + " Hz, peak " + juce::String (peak (r.out[2])) + ", pitch set " + juce::String (dsp::MixingEngine::testPitchFor (e->speaker (2).settings())));
            expect (peak (r.out[2]) > 0.01f, "and it is heard");

            // A muted output stays muted: the processor's mute is the person's decision.
            expect (e->setControl ("output/3/speaker/mute", 1.0f));
            expect (e->setControl ("output/3/test", 1.0f));
            expectEquals (peak (run (*e, 4, 1.5).out[2]), 0.0f, "a muted jack does not beep");

            // A plain output beeps at 1 kHz.
            expectWithinAbsoluteTolerance (dsp::MixingEngine::testPitchFor (dsp::SpeakerProcessor::Settings {}), 1000.0, 0.01);
            dsp::SpeakerProcessor::Settings hf;
            hf.hpf.kind = dsp::SpeakerProcessor::FilterKind::LinkwitzRiley;
            hf.hpf.hz = 1600.0f;
            expect (dsp::MixingEngine::testPitchFor (hf) >= 2400.0f, "a horn behind a 1.6 kHz high-pass beeps above it");
        }

        beginTest ("A monitor mix's test tone goes through its own fader and out of its patched jack");
        {
            auto e = std::make_unique<dsp::MixingEngine>();
            e->prepare (kRate, kBlock);
            expect (e->setControl ("output/3/source", 8.0f)); // Aux 2 on jack 3
            expect (e->setControl ("aux/2/test", 1.0f));
            const auto r = run (*e, 4, 2.0);
            expect (peak (r.out[2]) > 0.02f, "jack 3 carries mix 2's tone: " + juce::String (peak (r.out[2])));
            expectEquals (bursts (r.out[2]), 1);
            expectEquals (peak (r.out[0]), 0.0f, "the master jacks do not");

            expect (e->setControl ("aux/2/mute", 1.0f));
            run (*e, 4, 0.1); // the mix's own 20 ms fade to silence, before the tone starts
            expect (e->setControl ("aux/2/test", 1.0f));
            expectEquals (peak (run (*e, 4, 2.0).out[2]), 0.0f, "a muted mix stays silent: the test goes through the fader");
        }

        beginTest ("A test tone is an action: never journalled, and a device stop ends it");
        {
            auto e = std::make_unique<dsp::MixingEngine>();
            e->prepare (kRate, kBlock);
            expect (e->postControl ("/output/2/test", 1.0f));
            expect (e->postControl ("/aux/1/test", 1.0f));
            expect (e->postControl ("/output/2/delay", 3.0f));
            e->serviceControls();
            dsp::MixingEngine::ControlRecord journal[8];
            const int n = e->drainApplied (journal, 8);
            expectEquals (n, 1, "only the delay is kept");
            expect (e->isOutputTesting() && e->isMonitorTesting());
            e->reset();
            expect (! e->isOutputTesting() && ! e->isMonitorTesting(), "a device that stops does not resume beeping");
            expect (! e->setControl ("output/0/test", 1.0f), "output 0 does not exist");
            expect (! e->setControl ("output/33/test", 1.0f));
            expect (! e->setControl ("aux/9/test", 1.0f));
        }
    }
};

static TestToneTests testToneTests;
