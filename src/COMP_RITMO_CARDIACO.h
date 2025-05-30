#ifndef COMP_RITMO_CARDIACO_H
#define COMP_RITMO_CARDIACO_H

#include <stdint.h>

// Heart rate processor: detect beats and calculate BPM
class HeartRateProcessor {
public:
    // Constructor
    HeartRateProcessor();

    // Reset internal state and counters
    void reset();

    /**
     *  Feed a new AC component of the IR signal.
     *  @param irACValue  Filtered AC value from IR LED channel
     *  @param timestampMs Time (ms) of this sample (real or simulated)
     *  @return True if a heartbeat was detected on this sample
     */
    bool update(float irACValue, uint32_t timestampMs);

    /**
     *  Get the current heart rate in beats per minute.
     *  @return BPM (0.0 if invalid)
     */
    float getBPM() const;

    /**
     *  Check whether the last call to update() detected a beat.
     *  @return True if a beat was detected on last sample
     */
    bool isBeatDetected() const;

private:
    // State machine states for beat detection
    enum State {
        INIT,
        WAITING,
        FOLLOWING_SLOPE,
        MAYBE_DETECTED,
        MASKING
    } state;

    // Threshold and beat timing
    float threshold;
    float beatPeriod;         // filtered beat period in ms
    float lastMaxValue;
    uint32_t tsLastBeat;      // timestamp of last beat (ms)
    bool beatDetectedFlag;

    // Configuration constants
    static constexpr uint32_t INIT_HOLDOFF      = 2000;  // ms
    static constexpr uint32_t MASKING_HOLDOFF   = 300;   // ms
    static constexpr float    ALPHA             = 0.95f;  // EMA factor for period
    static constexpr float    MIN_THRESHOLD     = 50.0f;
    static constexpr float    MAX_THRESHOLD     = 800.0f;
    static constexpr float    STEP_RESILIENCY   = 50.0f; // max negative jump
    static constexpr float    THRESH_FALLOFF    = 0.3f;  // ratio after beat
    static constexpr float    THRESH_DECAY      = 0.99f; // continuous decay
    static constexpr uint32_t INVALID_DELAY     = 2000;  // ms without beat resets
    static constexpr uint32_t SAMPLE_PERIOD     = 10;    // ms between samples

    // Internal detection methods
    bool checkForBeat(float sample, uint32_t now);
    void decreaseThreshold();
};

#endif // COMP_RITMO_CARDIACO_H