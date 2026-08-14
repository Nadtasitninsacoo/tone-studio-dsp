#include <juce_core/juce_core.h>
#include <new>
#include <cstdlib>
#include <cstdio>
#include <vector>

// Thread-local flag to enable/disable allocation tracking
thread_local bool allowAllocations = true;

/**
 * Which module is being exercised right now.
 *
 * The check aborted with "Memory allocation detected in real-time block!" and nothing else,
 * so the one thing it could not say was *which of the fourteen modules did it* — and an
 * abort means no test name reaches the output either. A diagnostic that proves a bug exists
 * but not where is barely better than none. One `const char*` turns a bisection into a line.
 */
thread_local const char* rtSection = "(unknown)";

void* operator new(std::size_t size) {
    if (!allowAllocations) {
        std::fprintf(stderr, "ERROR: Memory allocation (%zu bytes) in real-time block: %s\n",
                     size, rtSection);
        std::fflush(stderr);
        std::abort();
    }
    void* p = std::malloc(size);
    if (!p) throw std::bad_alloc();
    return p;
}

void operator delete(void* p) noexcept {
    std::free(p);
}

void operator delete(void* p, std::size_t) noexcept {
    std::free(p);
}

/**
 * JUCE's UnitTestRunner reports through `Logger::writeToLog`, which on a console build goes
 * nowhere visible — so a failing suite exited 1 and printed absolutely nothing. Exit codes
 * are not a test report: you cannot fix what has no name.
 */
class ConsoleLogger : public juce::Logger {
    void logMessage(const juce::String& message) override {
        std::printf("%s\n", message.toRawUTF8());
        std::fflush(stdout);
    }
};

int main()
{
    ConsoleLogger logger;
    juce::Logger::setCurrentLogger(&logger);

    juce::UnitTestRunner runner;
    runner.runAllTests();

    int numFailures = 0;
    int numPasses = 0;
    for (int i = 0; i < runner.getNumResults(); ++i)
    {
        auto* result = runner.getResult(i);
        numFailures += result->failures;
        numPasses += result->passes;
        if (result->failures > 0) {
            std::printf("FAILED  %s / %s  (%d failed, %d passed)\n",
                        result->unitTestName.toRawUTF8(),
                        result->subcategoryName.toRawUTF8(),
                        result->failures, result->passes);
        }
    }

    std::printf("\n%d passed, %d failed\n", numPasses, numFailures);
    std::fflush(stdout);

    juce::Logger::setCurrentLogger(nullptr);
    return numFailures > 0 ? 1 : 0;
}


/* -- No-Allocation Test Case -- */
#include "FilterPrimitives.h"
#include "ParametricEQ.h"
#include "GateExpander.h"
#include "Compressor.h"
#include "DeEsser.h"
#include "GraphicEQ.h"
#include "FeedbackSuppressor.h"
#include "Limiter.h"
#include "Crossover.h"
#include "DelayLine.h"
#include "ReverbDelay.h"
#include "Panner.h"
#include "Metering.h"

class NoAllocationTests : public juce::UnitTest {
public:
    NoAllocationTests() : juce::UnitTest("NoAllocationInProcess") {}
    
    void runTest() override {
        beginTest("Zero Allocation in Audio Loop");
        
        // Instantiate all modules
        dsp::FilterPrimitives filter;
        dsp::ParametricEQ eq;
        dsp::GateExpander gate;
        dsp::Compressor comp;
        dsp::DeEsser deesser;
        dsp::GraphicEQ geq;
        dsp::FeedbackSuppressor suppressor;
        dsp::Limiter limiter;
        dsp::Crossover crossover;
        dsp::DelayLine delay;
        dsp::ReverbDelay rv;
        dsp::Panner panner;
        dsp::ChannelMetering chMeter;
        dsp::MasterMetering masterMeter;
        
        // Prepare them all
        double sr = 48000.0;
        int blockSize = 512;
        
        filter.prepare(sr, blockSize);
        eq.prepare(sr, blockSize);
        gate.prepare(sr, blockSize);
        comp.prepare(sr, blockSize);
        deesser.prepare(sr, blockSize);
        geq.prepare(sr, blockSize);
        suppressor.prepare(sr, blockSize);
        limiter.prepare(sr, blockSize);
        crossover.prepare(sr, blockSize);
        delay.prepare(sr, blockSize);
        rv.prepare(sr, blockSize);
        panner.prepare(sr);
        chMeter.prepare(sr);
        masterMeter.prepare(sr, blockSize);
        
        // Configure them for active DSP
        filter.setParameters(1000.0f, 0.707f, 6.0f);
        gate.setEnabled(true);
        comp.setEnabled(true);
        deesser.setEnabled(true);
        suppressor.setEnabled(true);
        limiter.setEnabled(true);
        crossover.setFrequency(100.0f);
        delay.setDelayMs(10.0f);
        rv.setDelayEnabled(true);
        rv.setReverbEnabled(true);
        panner.setPan(0.5f);
        
        // Prepare test data
        std::vector<float> inputL(blockSize, 1.0f);
        std::vector<float> inputR(blockSize, 1.0f);
        std::vector<float> outputL(blockSize, 0.0f);
        std::vector<float> outputR(blockSize, 0.0f);
        
        float* stereoOutput[2] = { outputL.data(), outputR.data() };
        const float* stereoInput[2] = { inputL.data(), inputR.data() };
        std::vector<float> interleavedStereo(2 * blockSize, 0.5f);
        std::vector<float> subBuffer(2 * blockSize, 0.0f);
        std::vector<float> mainBuffer(2 * blockSize, 0.0f);
        
        // Enable tracking!
        allowAllocations = false;
        
        rtSection = "FilterPrimitives";
        // Exercise filter
        filter.processSample(1.0f);
        filter.processBlock(inputL.data(), blockSize);
        
        rtSection = "ParametricEQ";
        // Exercise EQ
        eq.processSample(1.0f);
        eq.processBlock(inputL.data(), blockSize);
        
        rtSection = "GateExpander";
        // Exercise Gate
        gate.processSample(1.0f);
        gate.processBlock(inputL.data(), blockSize);
        
        rtSection = "Compressor";
        // Exercise Compressor
        comp.processSample(1.0f);
        comp.processBlock(inputL.data(), blockSize);
        
        rtSection = "DeEsser";
        // Exercise DeEsser
        deesser.processSample(1.0f);
        deesser.processBlock(inputL.data(), blockSize);
        
        rtSection = "GraphicEQ";
        // Exercise GEQ
        geq.processSample(1.0f);
        geq.processBlock(inputL.data(), blockSize);
        
        rtSection = "FeedbackSuppressor";
        // Exercise FeedbackSuppressor
        suppressor.processSample(1.0f);
        suppressor.processBlock(inputL.data(), blockSize);
        
        rtSection = "Limiter";
        // Exercise Limiter (processes stereo block)
        limiter.processBlock(interleavedStereo.data(), blockSize);
        
        rtSection = "Crossover";
        // Exercise Crossover
        crossover.processBlockInterleaved(interleavedStereo.data(), subBuffer.data(), mainBuffer.data(), blockSize);
        
        rtSection = "DelayLine";
        // Exercise Delay
        delay.processSample(1.0f);
        delay.processBlock(inputL.data(), blockSize);
        
        rtSection = "ReverbDelay";
        // Exercise Reverb/Delay send return
        rv.processBlock(stereoInput, stereoOutput, blockSize);
        
        rtSection = "Panner";
        // Exercise Panner
        float outL = 0.0f, outR = 0.0f;
        panner.processSample(1.0f, outL, outR);
        panner.processBlock(inputL.data(), interleavedStereo.data(), blockSize);
        
        rtSection = "Metering";
        // Exercise Metering
        chMeter.processBlock(inputL.data(), blockSize, 0.0f);
        masterMeter.processBlock(stereoInput, blockSize, 0.0f);
        
        // Disable tracking
        allowAllocations = true;
        
        expect(true); // If we reached here, no allocations happened!
    }
};

static NoAllocationTests noAllocationTests;
