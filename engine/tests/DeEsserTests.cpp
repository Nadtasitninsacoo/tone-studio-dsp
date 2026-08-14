#include "DeEsser.h"
#include <juce_core/juce_core.h>
#include <cmath>
#include "TestHelpers.h"

class DeEsserTests : public juce::UnitTest
{
public:
    DeEsserTests() : juce::UnitTest ("DeEsser") {}

    void runTest() override
    {
        beginTest ("DeEsser Threshold Test");
        
        dsp::DeEsser deesser;
        deesser.prepare(48000.0, 512);
        deesser.setEnabled(true);
        // Crossover two octaves below the probe tone, so the splitter's own roll-off is
        // not part of the measurement. At one octave it still cost 1.9 dB, which the test
        // would have read as the ratio being wrong.
        deesser.setFrequency(3000.0f);
        deesser.setThreshold(-20.0f);
        deesser.setRatio(4.0f);

        /**
         * **The signal has to actually be at 6 kHz.**
         *
         * Both cases fed `processSample(constant)` — a DC level — while the comments said
         * 6000 Hz. A de-esser detects through a band-pass on its sidechain, and a band-pass
         * at 6 kHz sees essentially nothing of DC, so the detector never triggered and the
         * gain reduction sat at -0.009 dB. The test could only ever have passed against a
         * de-esser with no sidechain filter at all — i.e. against a broken one.
         */
        /**
         * The probe tone sits at **12 kHz, not at the 6 kHz crossover.**
         *
         * A Butterworth split is -3.01 dB on *both* sides at its own corner frequency — that
         * is what "crossover" means — so a tone placed exactly there arrives in the sibilance
         * band 3 dB below its nominal level. The de-esser then correctly reduced by
         * 7.0 x 0.75 = 5.24 dB while the test expected 10 x 0.75 = 7.5, and the 2.26 dB gap
         * was the filter behaving exactly as designed.
         *
         * Two octaves up the high-pass is within a few tenths of a dB of unity, so the level in the band is
         * the level fed in and the ratio law is what is actually under test.
         */
        const double twoPiFOverFs = 2.0 * 3.14159265358979 * 12000.0 / 48000.0;

        // Test with low sibilance (below threshold): 6000 Hz at -40 dBFS
        float lowSignal = juce::Decibels::decibelsToGain(-40.0f);
        float output = 0.0f;
        for (int i = 0; i < 1000; ++i) {
            output = deesser.processSample(lowSignal * (float) std::sin(twoPiFOverFs * i));
        }
        float grDb = deesser.getGainReductionDb();
        // Expect no gain reduction (0 dB)
        expectWithinAbsoluteTolerance(grDb, 0.0f, 0.5f);

        // Test with high sibilance (above threshold): 6000 Hz at -10 dBFS
        float highSignal = juce::Decibels::decibelsToGain(-10.0f);
        for (int i = 0; i < 4000; ++i) {
            output = deesser.processSample(highSignal * (float) std::sin(twoPiFOverFs * i));
        }
        grDb = deesser.getGainReductionDb();
        // Expected compression: (10dB excess / 4 ratio) = -7.5 dB gain reduction
        expectWithinAbsoluteTolerance(grDb, -7.5f, 1.0f);
    }
};

static DeEsserTests deEsserTests;
