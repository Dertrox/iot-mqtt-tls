#include "COMP_SPO2.h"
#include <Arduino.h>

// Lookup table for SpO2 values based on ratio index
const uint8_t SpO2Processor::spO2LUT[43] = {
    100,100,100,100,99,99,99,99,99,99,
    98,98,98,98,98,97,97,97,97,97,97,
    96,96,96,96,96,96,95,95,95,95,95,95,
    94,94,94,94,94,93,93,93,93,93
};

SpO2Processor::SpO2Processor()
    : irACSumSq(0), redACSumSq(0), sampleCount(0), beatsDetected(0), spO2(0) {
}

void SpO2Processor::reset() {
    irACSumSq = 0;
    redACSumSq = 0;
    sampleCount = 0;
    beatsDetected = 0;
    spO2 = 0;
}

void SpO2Processor::update(float irAC, float redAC, bool beatDetected) {
    // Accumulate squared AC components
    irACSumSq += irAC * irAC;
    redACSumSq += redAC * redAC;
    sampleCount++;

    if (beatDetected) {
        beatsDetected++;
        if (beatsDetected >= SPO2_CALC_EVERY_N_BEATS) {
            computeSpO2();
        }
    }
}

uint8_t SpO2Processor::getSpO2() const {
    return spO2;
}

void SpO2Processor::computeSpO2() {
    if (sampleCount == 0 || irACSumSq <= 0 || redACSumSq <= 0) {
        spO2 = 0;
    } else {
        // Calculate RMS values
        float rmsIR = sqrt(irACSumSq / sampleCount);
        float rmsRed = sqrt(redACSumSq / sampleCount);

        // Calculate ratio of log(Red) / log(IR) using RMS
        float ratio = 100.0f * log(rmsRed) / log(rmsIR);

        // Map ratio to index in LUT
        int index = 0;
        if (ratio > 66.0f) {
            index = (int)ratio - 66;
        } else if (ratio > 50.0f) {
            index = (int)ratio - 50;
        }
        if (index < 0) index = 0;
        if (index >= 43) index = 42;

        spO2 = spO2LUT[index];
    }

    // Reset accumulators but keep last computed value
    irACSumSq = 0;
    redACSumSq = 0;
    sampleCount = 0;
    beatsDetected = 0;
}