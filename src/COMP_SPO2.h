#ifndef COMP_SPO2_H
#define COMP_SPO2_H

#include <stdint.h>
#include <math.h>

// Number of beats over which to calculate SpO2
#ifndef SPO2_CALC_EVERY_N_BEATS
#define SPO2_CALC_EVERY_N_BEATS 3
#endif

class SpO2Processor {
public:
    // Constructor
    SpO2Processor();

    // Reset internal accumulators
    void reset();

    /**
     *  Update the processor with a new sample.
     *  @param irAC          AC component of infrared LED signal
     *  @param redAC         AC component of red LED signal
     *  @param beatDetected  True if a heartbeat was detected at this sample
     */
    void update(float irAC, float redAC, bool beatDetected);

    /**
     *  Retrieve the last calculated SpO2 percentage.
     *  @return SpO2 value (0-100). Returns 0 if invalid.
     */
    uint8_t getSpO2() const;

private:
    // Sum of squares of AC values for IR and Red
    float irACSumSq;
    float redACSumSq;

    // Number of samples accumulated
    uint32_t sampleCount;

    // Number of beats detected since last calculation
    uint8_t beatsDetected;

    // Last computed SpO2 value
    uint8_t spO2;

    // Lookup table for SpO2 values based on ratio index
    static const uint8_t spO2LUT[43];

    // Helper: compute ratio using RMS method and lookup SpO2
    void computeSpO2();
};

#endif // COMP_SPO2_H