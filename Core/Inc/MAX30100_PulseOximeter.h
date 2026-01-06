/*
 * MAX30100_PulseOximeter.h
 *
 *  Created on: Sep 24, 2025
 *      Author: Hemavathi
 */

#ifndef INC_MAX30100_PULSEOXIMETER_H_
#define INC_MAX30100_PULSEOXIMETER_H_

#include "MAX30100.h"
#include "MAX30100_BeatDetector.h"
#include "MAX30100_Filters.h"
#include "MAX30100_SpO2Calculator.h" // Now includes the real file

#define DC_REMOVER_ALPHA 0.95f

typedef enum {
    PULSEOXIMETER_STATE_INIT,
    PULSEOXIMETER_STATE_IDLE,
    PULSEOXIMETER_STATE_DETECTING
} PulseOximeterState;

// The main struct that holds all sub-modules
typedef struct {
    PulseOximeterState state;
    MAX30100_t hrm;
    BeatDetector_t beatDetector;
    DCRemover_t irDCRemover;
    DCRemover_t redDCRemover;
    FilterBuLp1_t lpf;
    SpO2Calculator_t spO2calculator;
    uint32_t tsLastBiasCheck;
    LEDCurrent irLedCurrent;
    uint8_t redLedCurrentIndex;
    void (*onBeatDetected)();
    uint16_t latestIRValue; // To store the latest raw sample
} PulseOximeter_t;

// C function prototypes
void PulseOximeter_init(PulseOximeter_t *pox);
bool PulseOximeter_begin(PulseOximeter_t *pox, I2C_HandleTypeDef *i2c_handle);
void PulseOximeter_update(PulseOximeter_t *pox);
float PulseOximeter_getHeartRate(PulseOximeter_t *pox);
uint8_t PulseOximeter_getSpO2(PulseOximeter_t *pox);
void PulseOximeter_setOnBeatDetectedCallback(PulseOximeter_t *pox, void (*cb)());
void PulseOximeter_setIRLedCurrent(PulseOximeter_t *pox, LEDCurrent irLedCurrent);

#endif /* INC_MAX30100_PULSEOXIMETER_H_ */
