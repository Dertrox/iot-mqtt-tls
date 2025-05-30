#include "COMP_RITMO_CARDIACO.h"

HeartRateProcessor::HeartRateProcessor()
    : state(INIT),
      threshold(MIN_THRESHOLD),
      beatPeriod(0.0f),
      lastMaxValue(0.0f),
      tsLastBeat(0),
      beatDetectedFlag(false) {
}

void HeartRateProcessor::reset() {
    state = INIT;
    threshold = MIN_THRESHOLD;
    beatPeriod = 0.0f;
    lastMaxValue = 0.0f;
    tsLastBeat = 0;
    beatDetectedFlag = false;
}

bool HeartRateProcessor::update(float irACValue, uint32_t timestampMs) {
    beatDetectedFlag = checkForBeat(irACValue, timestampMs);
    return beatDetectedFlag;
}

float HeartRateProcessor::getBPM() const {
    if (beatPeriod > 0.0f) {
        return (60000.0f / beatPeriod);
    } else {
        return 0.0f;
    }
}

bool HeartRateProcessor::isBeatDetected() const {
    return beatDetectedFlag;
}

bool HeartRateProcessor::checkForBeat(float sample, uint32_t now) {
    bool beatDetected = false;

    switch (state) {
        case INIT:
            if (now > INIT_HOLDOFF) {
                state = WAITING;
            }
            break;

        case WAITING:
            if (sample > threshold) {
                threshold = (sample < MAX_THRESHOLD ? sample : MAX_THRESHOLD);
                state = FOLLOWING_SLOPE;
            }
            // Reset if no beat for long time
            if ((now - tsLastBeat) > INVALID_DELAY) {
                beatPeriod = 0.0f;
                lastMaxValue = 0.0f;
            }
            decreaseThreshold();
            break;

        case FOLLOWING_SLOPE:
            if (sample < threshold) {
                state = MAYBE_DETECTED;
            } else {
                threshold = (sample < MAX_THRESHOLD ? sample : MAX_THRESHOLD);
            }
            break;

        case MAYBE_DETECTED:
            if ((sample + STEP_RESILIENCY) < threshold) {
                // Beat detected
                beatDetected = true;
                lastMaxValue = sample;
                state = MASKING;
                if (tsLastBeat != 0) {
                    float delta = (float)(now - tsLastBeat);
                    beatPeriod = ALPHA * delta + (1 - ALPHA) * beatPeriod;
                }
                tsLastBeat = now;
            } else {
                state = FOLLOWING_SLOPE;
            }
            break;

        case MASKING:
            if ((now - tsLastBeat) > MASKING_HOLDOFF) {
                state = WAITING;
            }
            decreaseThreshold();
            break;
    }

    return beatDetected;
}

void HeartRateProcessor::decreaseThreshold() {
    if (lastMaxValue > 0.0f && beatPeriod > 0.0f) {
        threshold -= lastMaxValue * (1.0f - THRESH_FALLOFF) / (beatPeriod / SAMPLE_PERIOD);
    } else {
        threshold *= THRESH_DECAY;
    }
    if (threshold < MIN_THRESHOLD) {
        threshold = MIN_THRESHOLD;
    }
}