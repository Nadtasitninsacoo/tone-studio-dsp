#include "Panner.h"
#include <algorithm>
#include <cmath>

namespace dsp {

Panner::Panner() {
    smoothedPan.setCurrentAndTargetValue(0.0f);
}

void Panner::prepare(double sampleRate) {
    // 20ms ramp time for panning changes
    smoothedPan.reset(sampleRate, 0.020);
    smoothedPan.setCurrentAndTargetValue(targetPan);
}

void Panner::setLaw(Law panningLaw) {
    law = panningLaw;
}

void Panner::setPan(float pan) {
    targetPan = std::clamp(pan, -1.0f, 1.0f);
    smoothedPan.setTargetValue(targetPan);
}

void Panner::getGainFactors(float panVal, float& gainL, float& gainR) const {
    // Map pan -1..1 to angle 0..pi/2
    float angle = (panVal + 1.0f) * 3.1415926535f / 4.0f;
    
    if (law == Law::Db3) {
        gainL = std::cos(angle);
        gainR = std::sin(angle);
    } else { // Law::Db45
        gainL = std::pow(std::cos(angle), 1.5f);
        gainR = std::pow(std::sin(angle), 1.5f);
    }
}

void Panner::processSample(float input, float& outputL, float& outputR) {
    float currentPanVal = smoothedPan.getNextValue();
    float gainL = 1.0f;
    float gainR = 1.0f;
    getGainFactors(currentPanVal, gainL, gainR);
    
    outputL = input * gainL;
    outputR = input * gainR;
}

void Panner::processBlock(const float* monoInput, float* stereoOutput, int numSamples) {
    for (int i = 0; i < numSamples; ++i) {
        float outL = 0.0f;
        float outR = 0.0f;
        processSample(monoInput[i], outL, outR);
        
        stereoOutput[2 * i] = outL;
        stereoOutput[2 * i + 1] = outR;
    }
}

} // namespace dsp
