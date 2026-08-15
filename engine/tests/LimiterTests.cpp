#include "Limiter.h"
#include "Metering.h"
#include <juce_core/juce_core.h>
#include <cmath>
#include "TestHelpers.h"

class LimiterTests : public juce::UnitTest
{
public:
    LimiterTests() : juce::UnitTest ("Limiter") {}

    void runTest() override
    {
        beginTest ("Brickwall Ceiling Test");
        
        dsp::Limiter limiter;
        limiter.prepare(48000.0, 512);
        limiter.setEnabled(true);
        limiter.setThreshold(0.0f);
        limiter.setCeiling(-1.0f); // -1.0 dBFS ceiling
        limiter.setLookahead(2.0f);
        limiter.setRelease(50.0f);

        // Input stereo samples at +6 dBFS (amplitude = 2.0f)
        int numSamples = 1000;
        std::vector<float> buffer(2 * numSamples, 2.0f); // Stereo interleaved

        // Process block
        limiter.processBlock(buffer.data(), numSamples);

        // Verify output peak does not exceed the ceiling (-1.0 dBFS approx 0.891f)
        float maxPeak = 0.0f;
        for (float val : buffer) {
            maxPeak = std::max(maxPeak, std::abs(val));
        }

        /**
         * **The contract is one-sided, and it used to be written as if it were not.**
         *
         * This asserted `maxPeak == ceiling ± 0.05` — that the output arrives *at* the
         * ceiling. Those are two claims of very different weight: over the ceiling is a
         * broken limiter feeding a PA, under it is a quiet one. A single tolerance made the
         * safe direction fail as loudly as the dangerous one, and it did exactly that —
         * adding `kDownsampleHeadroomDb` to fix a measured overshoot turned this red at
         * 0.794 against 0.891 with nothing actually wrong.
         *
         * Two assertions now: the ceiling one exact, the floor one describing the design
         * rather than restating the constant.
         */
        float expectedCeilingGain = juce::Decibels::decibelsToGain(-1.0f);

        expect(maxPeak <= expectedCeilingGain + 1.0e-4f,
               juce::String("the output never exceeds the ceiling — got ")
                   + juce::String(juce::Decibels::gainToDecibels(maxPeak))
                   + " dBFS against -1.0");

        /**
         * **And it is not crushing — bounded by an absolute number, not by the constant.**
         *
         * The first version of this read `maxPeak >= gain(-1 - kDownsampleHeadroomDb)`,
         * which is the headroom constant checking itself: raise the constant and the bound
         * moves with it, so the assertion cannot fail however much output is thrown away. It
         * passed at 1.0 dB, which was ten times the measured need and a straight 1 dB of
         * lost level on steady material.
         *
         * DC is the strictest case in this direction: nothing between its samples for the
         * decimation filter to overshoot on, so the output lands at exactly the working
         * ceiling and the whole headroom shows up as loss. 0.3 dB is the most worth paying
         * for a mechanism measured at 0.03.
         */
        expect(juce::Decibels::gainToDecibels(maxPeak) >= -1.0f - 0.3f,
               juce::String("and the headroom below the ceiling stays small — got ")
                   + juce::String(juce::Decibels::gainToDecibels(maxPeak))
                   + " dBFS against a -1.0 ceiling");


        beginTest("A sudden burst does not get through, and does not get clipped either");
        /**
         * **DC cannot catch what this class actually got wrong, and neither could the first
         * three signals tried here.**
         *
         * The overshoot lives in `processSamplesDown`: the gain is applied at 4× and the
         * decimation filter puts a little of it back. A constant has no impulse for a filter
         * to overshoot on, so `Brickwall Ceiling Test` above is blind to it by construction.
         *
         * Three plausible-looking replacements were blind too, and one of them was blind for
         * a reason worth writing down. **Alternating ±full-scale samples** — reached for as
         * "the densest transient content available at this rate" — is a square wave at
         * exactly Nyquist, and the oversampler's own filter removes it: the output measured
         * −22 dBFS against a −6 ceiling, 16 dB *below* the thing being tested, and the test
         * passed while exercising nothing. A hostile-looking signal the system under test
         * simply deletes is the harness bug, not a hard case.
         *
         * A silence-to-full-scale burst is what reproduces it, because the mechanism is the
         * **gain snapping**, not the input's spectrum: an instantaneous gain change at 4× is
         * a discontinuity, and a discontinuity is what rings on the way back down. Measured
         * overshoot ≈ 0.03 dB, and it is the only one of the four that reaches the clamp at
         * all when the headroom is set to zero.
         */
        {
            dsp::Limiter tr;
            tr.prepare(48000.0, 512);
            tr.setEnabled(true);
            tr.setThreshold(0.0f);
            tr.setCeiling(-6.0f);
            tr.setLookahead(2.0f);
            tr.setRelease(50.0f);

            const int n = 4000;
            std::vector<float> buf(2 * (size_t) n, 0.0f);
            for (int i = 1000; i < n; ++i) {
                // +6 dBFS from a standing start, a quarter of the way in.
                const float v = 2.0f * std::sin(juce::MathConstants<float>::twoPi * 1000.0f
                                                * (float) (i - 1000) / 48000.0f);
                buf[2 * (size_t) i] = v;
                buf[2 * (size_t) i + 1] = v;
            }

            tr.processBlock(buf.data(), n);

            // Past the burst's onset and the lookahead's fill, so this measures the limiter
            // in its steady state rather than the delay line emptying.
            const size_t from = 2400;
            float peak = 0.0f;
            int clamped = 0;
            const float ceilingGain = juce::Decibels::decibelsToGain(-6.0f);
            for (size_t i = from; i < buf.size(); ++i) {
                peak = std::max(peak, std::abs(buf[i]));
                if (std::abs(buf[i]) >= ceilingGain - 1.0e-6f) ++clamped;
            }

            expect(peak <= ceilingGain + 1.0e-4f,
                   juce::String("a burst does not get through the ceiling — got ")
                       + juce::String(juce::Decibels::gainToDecibels(peak))
                       + " dBFS against -6.0");

            /**
             * **And the clamp stays a backstop rather than becoming the mechanism.**
             *
             * This is the assertion that holds `kDownsampleHeadroomDb` in place. The one
             * above passes at zero headroom too — the clamp alone guarantees the ceiling —
             * so without this the constant could be deleted and every test would stay green
             * while the limiter quietly turned into a clipper. At zero it clips 3.36% of
             * this burst; at 0.1 dB it clips none.
             *
             * The reverse is not covered and saying so is the point: with the headroom
             * sized correctly the clamp never fires on any signal that can be synthesised
             * here, so **removing the clamp does not fail any test in this file.** It is a
             * backstop for the real-signal case the bench does not reproduce, and it is
             * kept on that basis rather than on a green light.
             */
            const double clampedFraction = (double) clamped / (double) (buf.size() - from);
            expect(clampedFraction < 0.001,
                   juce::String("the gain stage does the limiting, not the clamp — ")
                       + juce::String(clampedFraction * 100.0, 2) + "% of samples were clipped");

            /**
             * **And the ceiling holds as a TRUE peak, which is the claim this class makes.**
             *
             * This is the assertion the whole investigation was for, and it could not have
             * been written until `MasterMetering` measured true peak — the bench had no
             * instrument for it, which is a large part of why the defect survived.
             *
             * The sample assertions above were green while a real guitar came out 0.7 dB
             * over a −6 ceiling, and both facts were true at once: every *sample* was under
             * the ceiling because the clamp puts them there, and the waveform *between* the
             * samples was not. A sample-domain clamp cannot bound the curve between two
             * samples, and flattening the tops adds the harmonics that push it further up.
             *
             * The fix was making the limiter reconstruct the signal the way the meter does —
             * `filterHalfBandFIREquiripple` on both. The polyphase IIR it used before is a
             * poor interpolator, so the peaks the loop saw at 4× were not the ones that were
             * there; it limited what it could see, correctly, and the rest went out. **A
             * limiter judged by a better interpolator than it uses will always appear to
             * overshoot.**
             *
             * Measured through the meter's own path rather than a second implementation, so
             * this asserts the two stages agree rather than that both match a third opinion
             * written for the test.
             */
            {
                dsp::MasterMetering meter;
                meter.prepare(48000.0, 512);

                std::vector<float> l, r;
                for (int i = (int) from / 2; i < n; ++i) {
                    l.push_back(buf[2 * (size_t) i]);
                    r.push_back(buf[2 * (size_t) i + 1]);
                }
                for (int off = 0; off + 512 <= (int) l.size(); off += 512) {
                    const float* q[2] = { l.data() + off, r.data() + off };
                    meter.processBlock(q, 512, 0.0f);
                }

                expect(meter.getTruePeakDbL() <= -6.0f + 0.05f,
                       juce::String("the TRUE peak is under the ceiling too, not just the "
                                    "samples — got ")
                           + juce::String(meter.getTruePeakDbL(), 3) + " dBFS against -6.0");
            }

            /**
             * The latency this stage adds, printed rather than asserted.
             *
             * Nothing in the engine compensates for it, and the FIR interpolator costs more
             * of it than the IIR it replaced. There is no correct figure to assert against —
             * the point is that a change to the oversampler surfaces here as a number in the
             * log rather than as delay somebody meets on a stage.
             */
            logMessage(juce::String("limiter latency: ")
                       + juce::String(tr.getLatencySamples(), 1) + " samples ("
                       + juce::String(tr.getLatencySamples() / 48.0, 2) + " ms at 48 kHz)");
        }
    }
};

static LimiterTests limiterTests;
