/*
* MAX30100_BeatDetector.c
*
* Created on: Sep 24, 2025
* Author: Hemavathi
*/

#include "MAX30100_BeatDetector.h"
#include "stm32f1xx_hal.h" // For HAL_GetTick()

// A standard C min macro
#define MIN(a, b) ((a) < (b) ? (a) : (b))

// Private (static) function prototypes
static bool checkForBeat(BeatDetector_t *beatDetector, float sample);
static void decreaseThreshold(BeatDetector_t *beatDetector);

// Replaces the C++ constructor
void BeatDetector_init(BeatDetector_t *beatDetector) {
beatDetector->state = BEATDETECTOR_STATE_INIT;
beatDetector->threshold = BEATDETECTOR_MIN_THRESHOLD;
beatDetector->beatPeriod = 0;
beatDetector->lastMaxValue = 0;
// **IMPROVEMENT 1: Initialize timestamp for robust hold-off**
beatDetector->tsLastBeat = HAL_GetTick();
}

bool BeatDetector_addSample(BeatDetector_t *beatDetector, float sample) {
return checkForBeat(beatDetector, sample);
}

float BeatDetector_getRate(BeatDetector_t *beatDetector) {
if (beatDetector->beatPeriod != 0) {
return 1.0f / beatDetector->beatPeriod * 1000.0f * 60.0f;
} else {
return 0;
}
}

float BeatDetector_getCurrentThreshold(BeatDetector_t *beatDetector) {
return beatDetector->threshold;
}

// --- Private (static) functions ---
void BeatDetector_setOnBeatDetectedCallback(BeatDetector_t *beatDetector, void (*cb)())
{
beatDetector->onBeatDetected = cb;
}

static bool checkForBeat(BeatDetector_t *beatDetector, float sample) {
bool beatDetected = false;
uint32_t now = HAL_GetTick();

switch (beatDetector->state) {
case BEATDETECTOR_STATE_INIT:
// **IMPROVEMENT 1: Use robust delta-time for initialization hold-off**
if (now - beatDetector->tsLastBeat > BEATDETECTOR_INIT_HOLDOFF) {
beatDetector->state = BEATDETECTOR_STATE_WAITING;
}
break;

case BEATDETECTOR_STATE_WAITING:
// ... (no changes in this state) ...
if (sample > beatDetector->threshold) {
beatDetector->threshold = MIN(sample, BEATDETECTOR_MAX_THRESHOLD);
beatDetector->state = BEATDETECTOR_STATE_FOLLOWING_SLOPE;
}
if (now - beatDetector->tsLastBeat > BEATDETECTOR_INVALID_READOUT_DELAY) {
beatDetector->beatPeriod = 0;
beatDetector->lastMaxValue = 0;
}
decreaseThreshold(beatDetector);
break;

case BEATDETECTOR_STATE_FOLLOWING_SLOPE:
// ... (no changes in this state) ...
if (sample < beatDetector->threshold) {
beatDetector->state = BEATDETECTOR_STATE_MAYBE_DETECTED;
} else {
beatDetector->threshold = MIN(sample, BEATDETECTOR_MAX_THRESHOLD);
}
break;

case BEATDETECTOR_STATE_MAYBE_DETECTED:
if (sample + BEATDETECTOR_STEP_RESILIENCY < beatDetector->threshold) {
beatDetected = true;
if (beatDetector->onBeatDetected) {
beatDetector->onBeatDetected();
}
beatDetector->lastMaxValue = sample;
beatDetector->state = BEATDETECTOR_STATE_MASKING;

// **IMPROVEMENT 2: Add safeguard for HAL_GetTick() overflow**
if ((int32_t)(now - beatDetector->tsLastBeat) > 0) {
float delta = (float)(now - beatDetector->tsLastBeat);
beatDetector->beatPeriod = BEATDETECTOR_BPFILTER_ALPHA * delta +
(1.0f - BEATDETECTOR_BPFILTER_ALPHA) * beatDetector->beatPeriod;
}

beatDetector->tsLastBeat = now;
} else {
beatDetector->state = BEATDETECTOR_STATE_FOLLOWING_SLOPE;
}
break;

case BEATDETECTOR_STATE_MASKING:
// ... (no changes in this state) ...
if (now - beatDetector->tsLastBeat > BEATDETECTOR_MASKING_HOLDOFF) {
beatDetector->state = BEATDETECTOR_STATE_WAITING;
}
decreaseThreshold(beatDetector);
break;
}

return beatDetected;
}

static void decreaseThreshold(BeatDetector_t *beatDetector) {
// ... (no changes in this function) ...
if (beatDetector->lastMaxValue > 0 && beatDetector->beatPeriod > 0) {
beatDetector->threshold -= beatDetector->lastMaxValue * (1.0f - BEATDETECTOR_THRESHOLD_FALLOFF_TARGET) /
(beatDetector->beatPeriod / BEATDETECTOR_SAMPLES_PERIOD);
} else {
beatDetector->threshold *= BEATDETECTOR_THRESHOLD_DECAY_FACTOR;
}

if (beatDetector->threshold < BEATDETECTOR_MIN_THRESHOLD) {
beatDetector->threshold = BEATDETECTOR_MIN_THRESHOLD;
}
}
