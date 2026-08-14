#include "DelayLine.h"
#include <algorithm>
#include <cmath>

namespace dsp {

DelayLine::DelayLine() {
    buffer.resize(BufferSize, 0.0f);
    reset();
}

void DelayLine::prepare(double newSampleRate, int /*maxBlockSize*/) {
    sampleRate = newSampleRate;
    reset();
}

void DelayLine::reset() {
    std::fill(buffer.begin(), buffer.end(), 0.0f);
    writeIndex = 0;
    currentDelaySamples = 0.0f;
    delayPrimed = false;
}

void DelayLine::setDelayMs(float val) {
    targetDelayMs = std::clamp(val, 0.0f, 1000.0f);
}

float DelayLine::readSample(int index) const {
    return buffer[index & IndexMask];
}

float DelayLine::processSample(float input) {
    // 1. Write input to circular buffer
    buffer[writeIndex] = input;
    
    // 2. Smooth delay time in samples
    float delaySamples = (targetDelayMs / 1000.0f) * static_cast<float>(sampleRate);
    if (!delayPrimed) {
        // Start *at* the delay, then glide from there. See `delayPrimed`.
        currentDelaySamples = delaySamples;
        delayPrimed = true;
    } else {
        // Smooth slowly to avoid pitch-shifting artifacts during alignment adjustments
        currentDelaySamples += 0.005f * (delaySamples - currentDelaySamples);
    }

    // 3. Lagrange 3rd-order interpolation
    int M = static_cast<int>(currentDelaySamples);
    float d = currentDelaySamples - M;

    // We need 4 points around the fractional delay
    // p0 is at n-M, p1 is at n-M-1. We interpolate between them.
    // p-1 is at n-M+1, p2 is at n-M-2.
    float p_minus1 = readSample(writeIndex - M + 1);
    float p0        = readSample(writeIndex - M);
    float p1        = readSample(writeIndex - M - 1);
    float p2        = readSample(writeIndex - M - 2);

    // Lagrange interpolation coefficients
    float h_minus1 = -d * (d - 1.0f) * (d - 2.0f) / 6.0f;
    float h0        = (d + 1.0f) * (d - 1.0f) * (d - 2.0f) / 2.0f;
    float h1        = -(d + 1.0f) * d * (d - 2.0f) / 2.0f;
    float h2        = (d + 1.0f) * d * (d - 1.0f) / 6.0f;

    float output = h_minus1 * p_minus1 + h0 * p0 + h1 * p1 + h2 * p2;

    // 4. Update write index
    writeIndex = (writeIndex + 1) & IndexMask;

    return output;
}

void DelayLine::processBlock(float* bufferPtr, int numSamples) {
    for (int i = 0; i < numSamples; ++i) {
        bufferPtr[i] = processSample(bufferPtr[i]);
    }
}

} // namespace dsp
