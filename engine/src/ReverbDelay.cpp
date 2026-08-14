#include "ReverbDelay.h"
#include <algorithm>
#include <cmath>

namespace dsp {

ReverbDelay::ReverbDelay() {
    delayBufferL.resize(BufferSize, 0.0f);
    delayBufferR.resize(BufferSize, 0.0f);

    feedbackHpfL.setType(FilterPrimitives::Type::HighPass);
    feedbackHpfR.setType(FilterPrimitives::Type::HighPass);
    feedbackLpfL.setType(FilterPrimitives::Type::LowPass);
    feedbackLpfR.setType(FilterPrimitives::Type::LowPass);

    reset();
}

void ReverbDelay::prepare(double newSampleRate, int maxBlockSize) {
    sampleRate = newSampleRate;
    
    // Prepare delay feedback filters
    feedbackHpfL.prepare(sampleRate, maxBlockSize);
    feedbackHpfR.prepare(sampleRate, maxBlockSize);
    feedbackLpfL.prepare(sampleRate, maxBlockSize);
    feedbackLpfR.prepare(sampleRate, maxBlockSize);

    wetL.assign(static_cast<size_t>(maxBlockSize), 0.0f);
    wetR.assign(static_cast<size_t>(maxBlockSize), 0.0f);
    revL.assign(static_cast<size_t>(maxBlockSize), 0.0f);
    revR.assign(static_cast<size_t>(maxBlockSize), 0.0f);

    reset();
}

void ReverbDelay::reset() {
    std::fill(delayBufferL.begin(), delayBufferL.end(), 0.0f);
    std::fill(delayBufferR.begin(), delayBufferR.end(), 0.0f);
    writeIndex = 0;

    feedbackHpfL.reset();
    feedbackHpfR.reset();
    feedbackLpfL.reset();
    feedbackLpfR.reset();

    currentDelayMs = targetDelayMs;
    
    // Set default filter parameters
    feedbackHpfL.setParameters(feedbackHpfHz, 0.707f, 0.0f);
    feedbackHpfR.setParameters(feedbackHpfHz, 0.707f, 0.0f);
    feedbackLpfL.setParameters(feedbackLpfHz, 0.707f, 0.0f);
    feedbackLpfR.setParameters(feedbackLpfHz, 0.707f, 0.0f);
}

void ReverbDelay::setReverbEnabled(bool enabled) {
    reverbEnabled = enabled;
}

void ReverbDelay::setReverbParams(float roomSize, float damping, float width, float wetLevel) {
    reverbParams.roomSize = roomSize;
    reverbParams.damping = damping;
    reverbParams.width = width;
    reverbParams.wetLevel = wetLevel;
    reverbParams.dryLevel = 0.0f; // Return effect is 100% wet
    reverb.setParameters(reverbParams);
}

void ReverbDelay::setReverbRoomSize(float roomSize) {
    reverbParams.roomSize = std::clamp(roomSize, 0.0f, 1.0f);
    reverb.setParameters(reverbParams);
}

void ReverbDelay::setReverbDamping(float damping) {
    reverbParams.damping = std::clamp(damping, 0.0f, 1.0f);
    reverb.setParameters(reverbParams);
}

void ReverbDelay::setReverbWidth(float width) {
    reverbParams.width = std::clamp(width, 0.0f, 1.0f);
    reverb.setParameters(reverbParams);
}

void ReverbDelay::setReverbWetLevel(float wetLevel) {
    reverbParams.wetLevel = std::clamp(wetLevel, 0.0f, 1.0f);
    reverb.setParameters(reverbParams);
}

void ReverbDelay::setDelayEnabled(bool enabled) {
    delayEnabled = enabled;
}

void ReverbDelay::setDelayParams(float delayMs, float fb, float wetLevel, bool pingPong) {
    targetDelayMs = std::clamp(delayMs, 1.0f, 2500.0f);
    feedback = std::clamp(fb, 0.0f, 0.95f);
    delayWetLevel = std::clamp(wetLevel, 0.0f, 1.0f);
    pingPongEnabled = pingPong;
}

void ReverbDelay::setDelayMs(float delayMs) {
    targetDelayMs = std::clamp(delayMs, 1.0f, 2500.0f);
    if (!delayPrimed) {
        currentDelayMs = targetDelayMs;
        delayPrimed = true;
    }
}

void ReverbDelay::setDelayFeedback(float fb) {
    feedback = std::clamp(fb, 0.0f, 0.95f);
}

void ReverbDelay::setDelayWetLevel(float wetLevel) {
    delayWetLevel = std::clamp(wetLevel, 0.0f, 1.0f);
}

void ReverbDelay::setDelayPingPong(bool pingPong) {
    pingPongEnabled = pingPong;
}

void ReverbDelay::setDelayFilters(float hpfHz, float lpfHz) {
    feedbackHpfHz = hpfHz;
    feedbackLpfHz = lpfHz;
    feedbackHpfL.setParameters(feedbackHpfHz, 0.707f, 0.0f);
    feedbackHpfR.setParameters(feedbackHpfHz, 0.707f, 0.0f);
    feedbackLpfL.setParameters(feedbackLpfHz, 0.707f, 0.0f);
    feedbackLpfR.setParameters(feedbackLpfHz, 0.707f, 0.0f);
}

void ReverbDelay::setDelayHpf(float hpfHz) {
    feedbackHpfHz = hpfHz;
    feedbackHpfL.setParameters(feedbackHpfHz, 0.707f, 0.0f);
    feedbackHpfR.setParameters(feedbackHpfHz, 0.707f, 0.0f);
}

void ReverbDelay::setDelayLpf(float lpfHz) {
    feedbackLpfHz = lpfHz;
    feedbackLpfL.setParameters(feedbackLpfHz, 0.707f, 0.0f);
    feedbackLpfR.setParameters(feedbackLpfHz, 0.707f, 0.0f);
}

float ReverbDelay::readSample(const std::vector<float>& buffer, float delayInSamples) const {
    int idx = static_cast<int>(delayInSamples);
    float d = delayInSamples - idx;
    
    float s0 = buffer[(writeIndex - idx) & IndexMask];
    float s1 = buffer[(writeIndex - idx - 1) & IndexMask];
    
    return s0 + d * (s1 - s0);
}

void ReverbDelay::processBlock(const float** inputChannels, float** outputChannels, int numSamples) {
    const int maxChunk = static_cast<int>(wetL.size());
    if (maxChunk <= 0) return;
    for (int offset = 0; offset < numSamples; ) {
        const int n = std::min(maxChunk, numSamples - offset);
        const float* in[2]  = { inputChannels[0] + offset, inputChannels[1] + offset };
        float*       out[2] = { outputChannels[0] + offset, outputChannels[1] + offset };
        processChunk(in, out, n);
        offset += n;
    }
}

void ReverbDelay::processChunk(const float** inputChannels, float** outputChannels, int numSamples) {
    // Reused buffers, cleared rather than reallocated. The caller chunks, so numSamples is
    // always within size.
    std::fill_n(wetL.begin(), numSamples, 0.0f);
    std::fill_n(wetR.begin(), numSamples, 0.0f);

    const float* inL = inputChannels[0];
    const float* inR = inputChannels[1];

    // 1. Process Delay if enabled
    if (delayEnabled) {
        if (!delayPrimed) {
            // Start at the requested delay, then glide from there on later changes.
            currentDelayMs = targetDelayMs;
            delayPrimed = true;
        }
        float delaySamples = (currentDelayMs / 1000.0f) * static_cast<float>(sampleRate);
        float targetSamples = (targetDelayMs / 1000.0f) * static_cast<float>(sampleRate);
        
        // Linear smooth delay time to avoid clicks
        float delaySmoothStep = (targetSamples - delaySamples) / numSamples;

        for (int i = 0; i < numSamples; ++i) {
            delaySamples += delaySmoothStep * 0.05f; // limit speed
            
            float xl = inL[i];
            float xr = inR[i];
            
            // Read delay tap
            float delL = readSample(delayBufferL, delaySamples);
            float delR = readSample(delayBufferR, delaySamples);
            
            // Apply feedback high-pass / low-pass filters
            float filteredL = feedbackHpfL.processSample(feedbackLpfL.processSample(delL));
            float filteredR = feedbackHpfR.processSample(feedbackLpfR.processSample(delR));
            
            // Feedback mix
            float feedL = filteredL * feedback;
            float feedR = filteredR * feedback;
            
            // Write input + feedback to buffer
            if (pingPongEnabled) {
                delayBufferL[writeIndex] = xl + feedR;
                delayBufferR[writeIndex] = xr + feedL;
            } else {
                delayBufferL[writeIndex] = xl + feedL;
                delayBufferR[writeIndex] = xr + feedR;
            }
            
            writeIndex = (writeIndex + 1) & IndexMask;
            
            // Add to wet output
            wetL[i] += delL * delayWetLevel;
            wetR[i] += delR * delayWetLevel;
        }
        
        // Carry the ramp across blocks. Without this the smoothing restarted from
        // `currentDelayMs` every block and the delay never reached its target.
        currentDelayMs = (delaySamples / static_cast<float>(sampleRate)) * 1000.0f;
    }

    // 2. Process Reverb if enabled
    if (reverbEnabled) {
        // Reverb expects input to contain dry input, which it processes in-place
        // We copy the input channels to temp arrays, run reverb, and add it to wet output
        std::copy_n(inL, numSamples, revL.begin());
        std::copy_n(inR, numSamples, revR.begin());
        
        reverb.processStereo(revL.data(), revR.data(), numSamples);
        
        for (int i = 0; i < numSamples; ++i) {
            wetL[i] += revL[i];
            wetR[i] += revR[i];
        }
    }

    // Write back wet-only outputs (add/write to send return buffers)
    for (int i = 0; i < numSamples; ++i) {
        outputChannels[0][i] = wetL[i];
        outputChannels[1][i] = wetR[i];
    }
}

} // namespace dsp
