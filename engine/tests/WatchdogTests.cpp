#include "AudioWatchdog.h"
#include <juce_core/juce_core.h>
#include <vector>

/**
 * The watchdog's whole contract, as a sequence of readings.
 *
 * The reported failure: a pause between songs (every input exactly zero for 3 s) made the engine
 * close and reopen the device, Windows refused the reopen, and the PA went silent mid-show. The
 * claim these tests make is the opposite of what used to happen: **silence is reported and never
 * acted on**, and only a stream that has stopped calling back is restarted.
 */
class WatchdogTests : public juce::UnitTest
{
public:
    WatchdogTests() : juce::UnitTest ("Watchdog") {}

    using W = dsp::AudioWatchdog;

    /** A healthy, playing device whose callback advances every tick. */
    static W::Reading playing (uint32_t now, uint64_t blocks, double silentSec = 0.0)
    {
        W::Reading r;
        r.nowMs = now;
        r.blocks = blocks;
        r.playing = true;
        r.hasInputs = true;
        r.silentInputSec = silentSec;
        return r;
    }

    void runTest() override
    {
        beginTest ("Silent inputs for a whole set never restart the audio");
        {
            W w;
            w.rearm (0, 0);
            int restarts = 0;
            uint64_t blocks = 0;
            // Ten minutes of exact zeros, the callback running all the while.
            for (uint32_t t = 250; t <= 600000; t += 250)
            {
                blocks += 10;
                const auto a = w.tick (playing (t, blocks, t / 1000.0));
                if (a != W::Action::None) ++restarts;
            }
            expectEquals (restarts, 0, "silence is not a reason to touch the device");
            expect (w.isInputSilent(), "but it is reported");
        }

        beginTest ("Silence that ends and comes back, again and again, is still never a restart");
        {
            // The old cap of two reset on any sound, so every pause could cut the PA again.
            W w;
            w.rearm (0, 0);
            int restarts = 0;
            uint64_t blocks = 0;
            double silent = 0.0;
            for (uint32_t t = 250; t <= 600000; t += 250)
            {
                blocks += 10;
                const bool songPlaying = (t / 20000) % 2 == 0; // 20 s on, 20 s off
                silent = songPlaying ? 0.0 : silent + 0.25;
                if (w.tick (playing (t, blocks, silent)) != W::Action::None) ++restarts;
            }
            expectEquals (restarts, 0);
        }

        beginTest ("A device with no inputs is never reported as silent");
        {
            W w;
            w.rearm (0, 0);
            auto r = playing (5000, 50, 10.0);
            r.hasInputs = false;
            w.tick (r);
            expect (! w.isInputSilent());
        }

        beginTest ("A stream that stops calling back is restarted once, after StallMs");
        {
            W w;
            w.rearm (0, 100);
            // The counter frozen at 100 while the device still says it is playing.
            int restarts = 0;
            uint32_t firstAt = 0;
            for (uint32_t t = 250; t <= 10000; t += 250)
            {
                if (w.tick (playing (t, 100)) == W::Action::RestartStalled)
                {
                    if (restarts == 0) firstAt = t;
                    ++restarts;
                }
            }
            expectEquals (restarts, 1, "once, not every tick");
            expect (firstAt > W::StallMs && firstAt <= W::StallMs + 250, "just after the stall threshold: " + juce::String (firstAt));
            expect (w.isStalled());
            expectEquals (w.stalls(), 1);
        }

        beginTest ("After a restart the stall clock starts again, and a stream that moves is fine");
        {
            W w;
            w.rearm (0, 100);
            for (uint32_t t = 250; t <= 3500; t += 250) w.tick (playing (t, 100));
            expect (w.isStalled());
            w.rearm (3500, 100);
            expect (! w.isStalled());
            expect (w.tick (playing (3750, 140)) == W::Action::None);
            expect (! w.isStalled());
        }

        beginTest ("A device that is down is reopened at once, then fast, then every two seconds");
        {
            W w;
            w.rearm (0, 0);
            W::Reading down;
            down.playing = false;
            std::vector<uint32_t> reopenAt;
            for (uint32_t t = 0; t <= 12000; t += 250)
            {
                down.nowMs = t;
                if (w.tick (down) == W::Action::Reopen) reopenAt.push_back (t);
            }
            expect (! reopenAt.empty() && reopenAt.front() == 0, "the first try is immediate");
            int fast = 0;
            for (auto t : reopenAt) if (t < W::FastReopenForMs) ++fast;
            expectEquals (fast, (int) (W::FastReopenForMs / W::FastReopenMs), "every 250 ms for the first four seconds");
            bool slowAfter = true;
            for (size_t i = 1; i < reopenAt.size(); ++i)
                if (reopenAt[i - 1] >= W::FastReopenForMs && reopenAt[i] - reopenAt[i - 1] != W::SlowReopenMs) slowAfter = false;
            expect (slowAfter, "then every two seconds, so an unplugged card does not spin");
            expect (w.isDown());
            expect (! w.isInputSilent(), "a device that is down is not 'silent', it is down");

            // It comes back.
            expect (w.tick (playing (12250, 1)) == W::Action::None);
            expect (! w.isDown());
            expectEquals (w.attempts(), 0);
        }
    }
};

static WatchdogTests watchdogTests;
