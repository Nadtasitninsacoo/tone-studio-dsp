#pragma once

#include <juce_core/juce_core.h>
#include <cmath>

/**
 * `expectWithinAbsoluteTolerance` does not exist on `juce::UnitTest` in the JUCE version
 * this project pins — that class has `expectEquals` and `expect`, and nothing between them.
 * Every DSP test here was written against it, so all fourteen failed to compile.
 *
 * Supplied as a macro rather than a free function on purpose: it keeps `expect(...)`
 * resolving to the UnitTest member, so a failure is still reported against the test that
 * raised it, and no call site had to change. Bumping JUCE to get the real thing would be a
 * much larger change for a test helper.
 */
#define expectWithinAbsoluteTolerance(actualValue, expectedValue, toleranceValue)          \
    expect (std::abs ((double) (actualValue) - (double) (expectedValue))                   \
                <= (double) (toleranceValue),                                              \
            juce::String ("expected ") + juce::String ((double) (expectedValue))           \
                + " +/- " + juce::String ((double) (toleranceValue))                       \
                + ", got " + juce::String ((double) (actualValue)))
