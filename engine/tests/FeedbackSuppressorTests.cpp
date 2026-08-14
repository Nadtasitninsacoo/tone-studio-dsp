#include "FeedbackSuppressor.h"
#include <juce_core/juce_core.h>
#include <cmath>
#include <vector>

/**
 * The claim these tests exist for is **not** "it notches howls".
 *
 * It is that the stereo path notches the *whole* block, on *each* channel separately. That
 * was broken and silent: `MixingEngine` handed the mono overload an interleaved buffer with a
 * frame count, so the back half of every block reached the crossover unsuppressed and the
 * front half was filtered by one biquad bank fed L,R,L,R. Neither shows up in a spectrum
 * anybody was looking at, and the page reported the notch as deployed either way.
 */
class FeedbackSuppressorTests : public juce::UnitTest
{
public:
    FeedbackSuppressorTests() : juce::UnitTest ("FeedbackSuppressor") {}

    /** RMS of one channel of an interleaved stereo buffer, over a frame range. */
    static float channelRms (const std::vector<float>& buf, int channel, int fromFrame, int toFrame)
    {
        double sum = 0.0;
        for (int i = fromFrame; i < toFrame; ++i)
        {
            const float s = buf[(size_t) (2 * i + channel)];
            sum += (double) s * s;
        }
        const int n = toFrame - fromFrame;
        return n > 0 ? (float) std::sqrt (sum / n) : 0.0f;
    }

    void runTest() override
    {
        constexpr double sampleRate = 48000.0;
        constexpr int blockSize = 512;
        constexpr float toneHz = 1000.0f;

        beginTest ("Fixed Slots Clear");
        {
            dsp::FeedbackSuppressor suppressor;
            suppressor.prepare (sampleRate, blockSize);
            suppressor.setEnabled (true);

            expect (suppressor.getActiveNotchFrequencies().empty());
            suppressor.clearDynamicSlots();
            suppressor.clearAllSlots();
            expect (suppressor.getActiveNotchFrequencies().empty());
        }

        beginTest ("Fixed slots are reachable at all");
        {
            // They were not. Eight slots, no setter, and a clearDynamicSlots() written to
            // protect them from a page that could not have filled them either.
            dsp::FeedbackSuppressor suppressor;
            suppressor.prepare (sampleRate, blockSize);

            expect (suppressor.setFixedNotch (0, toneHz, 15.0f, -18.0f));
            expect (suppressor.getActiveNotchFrequencies().size() == 1);

            expect (! suppressor.setFixedNotch (-1, toneHz, 15.0f, -12.0f),
                    "a negative slot is refused, not written somewhere else");
            expect (! suppressor.setFixedNotch (dsp::FeedbackSuppressor::NumFixedSlots,
                                                toneHz, 15.0f, -12.0f),
                    "and nor is the first dynamic slot reachable through this door");
            expect (suppressor.getActiveNotchFrequencies().size() == 1,
                    "so a refused call placed nothing");

            expect (suppressor.clearFixedNotch (0));
            expect (suppressor.getActiveNotchFrequencies().empty());
        }

        beginTest ("Interleaved: the whole block is filtered, not the front half");
        {
            /**
             * The buffer holds `2 * numFrames` floats. The mono overload walks `numSamples`
             * of them, so calling it with a frame count left frames `numFrames/2 .. numFrames`
             * completely untouched. Compare the two halves of the same steady tone: with the
             * bug the back half is at full amplitude.
             */
            dsp::FeedbackSuppressor suppressor;
            suppressor.prepare (sampleRate, blockSize);
            suppressor.setEnabled (true);
            suppressor.setFixedNotch (0, toneHz, 15.0f, -18.0f);

            std::vector<float> buf ((size_t) blockSize * 2, 0.0f);
            double phase = 0.0;
            const double step = 2.0 * juce::MathConstants<double>::pi * toneHz / sampleRate;

            // Several blocks first, so the filter's smoothed coefficients have arrived and
            // its state has settled — otherwise this measures the ramp, not the notch.
            for (int block = 0; block < 40; ++block)
            {
                for (int i = 0; i < blockSize; ++i)
                {
                    const float s = (float) std::sin (phase);
                    phase += step;
                    buf[(size_t) (2 * i)]     = s;
                    buf[(size_t) (2 * i + 1)] = s;
                }
                suppressor.processBlockInterleaved (buf.data(), blockSize);
            }

            const float front = channelRms (buf, 0, blockSize / 8, blockSize / 2);
            const float back  = channelRms (buf, 0, blockSize / 2, blockSize);

            expect (front < 0.4f, "the notch attenuates the tone it is placed on");
            expect (back < 0.4f, "and it attenuates the back half of the block too");
            // The discriminating assertion. Under the bug `back` is the untouched input at
            // ~0.707 RMS while `front` is attenuated, so the ratio blows up.
            expect (back < front * 2.0f,
                    "both halves of one block are attenuated by the same filter");
        }

        beginTest ("Interleaved: one channel does not leak into the other");
        {
            /**
             * A biquad is stateful, so one bank fed L,R,L,R runs a single filter over two
             * signals. Silence on the right is the cleanest way to see it: anything but
             * silence coming out means the right channel was carrying the left one's filter
             * state.
             */
            dsp::FeedbackSuppressor suppressor;
            suppressor.prepare (sampleRate, blockSize);
            suppressor.setEnabled (true);
            suppressor.setFixedNotch (0, toneHz, 15.0f, -18.0f);

            std::vector<float> buf ((size_t) blockSize * 2, 0.0f);
            double phase = 0.0;
            const double step = 2.0 * juce::MathConstants<double>::pi * toneHz / sampleRate;

            for (int block = 0; block < 40; ++block)
            {
                for (int i = 0; i < blockSize; ++i)
                {
                    buf[(size_t) (2 * i)]     = (float) std::sin (phase);
                    buf[(size_t) (2 * i + 1)] = 0.0f; // right is silent
                    phase += step;
                }
                suppressor.processBlockInterleaved (buf.data(), blockSize);
            }

            const float leftOut  = channelRms (buf, 0, 0, blockSize);
            const float rightOut = channelRms (buf, 1, 0, blockSize);

            expect (leftOut > 0.0f, "the left channel is still carrying the tone");
            expect (rightOut < 1.0e-6f,
                    "silence in, silence out — the right channel has its own filter bank");
        }

        beginTest ("Bypassed leaves the buffer alone");
        {
            // Same rule as the web app's suppressor: bypass forbids placing anything, and it
            // must not be confused with having found nothing.
            dsp::FeedbackSuppressor suppressor;
            suppressor.prepare (sampleRate, blockSize);
            suppressor.setEnabled (false);
            suppressor.setFixedNotch (0, toneHz, 15.0f, -18.0f);

            std::vector<float> buf ((size_t) blockSize * 2, 0.0f);
            for (int i = 0; i < blockSize; ++i)
            {
                buf[(size_t) (2 * i)]     = 0.5f;
                buf[(size_t) (2 * i + 1)] = -0.25f;
            }
            suppressor.processBlockInterleaved (buf.data(), blockSize);

            bool untouched = true;
            for (int i = 0; i < blockSize; ++i)
            {
                if (buf[(size_t) (2 * i)] != 0.5f || buf[(size_t) (2 * i + 1)] != -0.25f)
                    untouched = false;
            }
            expect (untouched, "a bypassed suppressor is not in the signal path at all");
        }

        beginTest ("A zero-length or negative block is a no-op, not a crash");
        {
            dsp::FeedbackSuppressor suppressor;
            suppressor.prepare (sampleRate, blockSize);
            suppressor.setEnabled (true);
            std::vector<float> buf (4, 0.25f);
            suppressor.processBlockInterleaved (buf.data(), 0);
            suppressor.processBlockInterleaved (buf.data(), -8);
            expect (buf[0] == 0.25f);
        }

        beginTest ("Re-preparing does not reallocate under the detection thread");
        {
            /**
             * The thread was started in the constructor and `prepare()` resizes three
             * vectors it reads, so every device start and every USB re-enumeration was a
             * use-after-free. The thread is started at the end of `prepare()` now and stopped
             * at the top of it.
             *
             * **What this test is and is not.** A data race cannot be made to fail
             * deterministically from a unit test — it is a smoke test that the stop/start
             * cycle survives being driven hard, plus the two assertions below, which *are*
             * deterministic: nothing is running before the first prepare, and something is
             * after it. Proving the race is gone needs a sanitiser build, which this project
             * does not have.
             */
            dsp::FeedbackSuppressor suppressor;
            expect (! suppressor.isThreadRunning(),
                    "the constructor does not start a thread against unsized buffers");

            suppressor.prepare (sampleRate, blockSize);
            expect (suppressor.isThreadRunning(), "prepare() starts it once there is data");

            suppressor.setEnabled (true);
            suppressor.setDetectionEnabled (true);

            std::vector<float> buf ((size_t) blockSize * 2, 0.05f);
            for (int round = 0; round < 12; ++round)
            {
                for (int block = 0; block < 8; ++block)
                    suppressor.processBlockInterleaved (buf.data(), blockSize);

                // Alternate the block size so the resize really reallocates.
                suppressor.prepare (sampleRate, (round % 2 == 0) ? 1024 : blockSize);
                suppressor.reset();
                expect (suppressor.isThreadRunning(),
                        "reset() gives the thread back rather than leaving detection dead");
            }
        }

        beginTest ("reset() before prepare() does not start a thread");
        {
            // MixingEngine's constructor calls reset() on the whole master bus, long before
            // any device has said what the block size is.
            dsp::FeedbackSuppressor suppressor;
            suppressor.reset();
            expect (! suppressor.isThreadRunning());
        }

        beginTest ("Processing before prepare() does not touch unsized buffers");
        {
            /**
             * The detection thread is started in the constructor, and every buffer it uses is
             * sized in prepare(). So this window is real on every startup, and the audio
             * thread can reach it too if a host calls process before prepare.
             */
            dsp::FeedbackSuppressor suppressor;
            suppressor.setEnabled (true);
            suppressor.setDetectionEnabled (true);
            std::vector<float> buf ((size_t) 64, 0.1f);
            suppressor.processBlockInterleaved (buf.data(), 32);
            suppressor.processBlock (buf.data(), 64);
            expect (true, "no out-of-bounds write on an empty fifoBuffer");
        }
    }
};

static FeedbackSuppressorTests feedbackSuppressorTests;
