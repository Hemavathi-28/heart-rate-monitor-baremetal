/*
* MAX30100_BeatDetector.h
*
* Created on: Sep 24, 2025
* Author: Hemavathi
*/

#ifndef INC_MAX30100_BEATDETECTOR_H_
#define INC_MAX30100_BEATDETECTOR_H_

#include <stdint.h>
#include <stdbool.h>

#define BEATDETECTOR_INIT_HOLDOFF 2000 // in ms
#define BEATDETECTOR_MASKING_HOLDOFF 200 // in ms
#define BEATDETECTOR_BPFILTER_ALPHA 0.6f // EMA factor
#define BEATDETECTOR_MIN_THRESHOLD 20.0f
#define BEATDETECTOR_MAX_THRESHOLD 800.0f
#define BEATDETECTOR_STEP_RESILIENCY 30.0f
#define BEATDETECTOR_THRESHOLD_FALLOFF_TARGET 0.3f
#define BEATDETECTOR_THRESHOLD_DECAY_FACTOR 0.99f
#define BEATDETECTOR_INVALID_READOUT_DELAY 2000 // in ms
#define BEATDETECTOR_SAMPLES_PERIOD 10 // in ms, 1/Fs

typedef enum {
BEATDETECTOR_STATE_INIT,
BEATDETECTOR_STATE_WAITING,
BEATDETECTOR_STATE_FOLLOWING_SLOPE,
BEATDETECTOR_STATE_MAYBE_DETECTED,
BEATDETECTOR_STATE_MASKING
} BeatDetectorState;

// The C struct that replaces the C++ class
typedef struct {
BeatDetectorState state;
float threshold;
float beatPeriod;
float lastMaxValue;
uint32_t tsLastBeat;
void (*onBeatDetected)(); // Function pointer for the callback
} BeatDetector_t;

// C function prototypes
void BeatDetector_init(BeatDetector_t *beatDetector);
bool BeatDetector_addSample(BeatDetector_t *beatDetector, float sample);
float BeatDetector_getRate(BeatDetector_t *beatDetector);
float BeatDetector_getCurrentThreshold(BeatDetector_t *beatDetector);
void BeatDetector_setOnBeatDetectedCallback(BeatDetector_t *beatDetector, void (*cb)());

#endif /* INC_MAX30100_BEATDETECTOR_H_ */
