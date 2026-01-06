/*
 * MAX30100_PulseOximeter.c
 *
 *  Created on: Sep 24, 2025
 *      Author: Hemavathi
 */

#include "MAX30100_PulseOximeter.h"
#include "stm32f1xx_hal.h"

#define CURRENT_ADJUSTMENT_PERIOD_MS 500
#define DEFAULT_IR_LED_CURRENT MAX30100_LED_CURR_50MA
#define RED_LED_CURRENT_START MAX30100_LED_CURR_27_1MA

// Private (static) function prototypes
static void checkSample(PulseOximeter_t *pox);
static void checkCurrentBias(PulseOximeter_t *pox);

void PulseOximeter_init(PulseOximeter_t *pox) {
    pox->state = PULSEOXIMETER_STATE_INIT;
    pox->tsLastBiasCheck = 0;
    pox->irLedCurrent = DEFAULT_IR_LED_CURRENT;
    pox->redLedCurrentIndex = RED_LED_CURRENT_START;
    pox->onBeatDetected = NULL;

    BeatDetector_init(&pox->beatDetector);
    SpO2Calculator_init(&pox->spO2calculator);
}

bool PulseOximeter_begin(PulseOximeter_t *pox, I2C_HandleTypeDef *i2c_handle) {
    if (!MAX30100_begin(&pox->hrm, i2c_handle)) {
        return false;
    }

    MAX30100_setMode(&pox->hrm, MAX30100_MODE_SPO2_HR);
    MAX30100_setLedsCurrent(&pox->hrm, pox->irLedCurrent, (LEDCurrent)pox->redLedCurrentIndex);

    DCRemover_init(&pox->irDCRemover, DC_REMOVER_ALPHA);
    DCRemover_init(&pox->redDCRemover, DC_REMOVER_ALPHA);
    FilterBuLp1_init(&pox->lpf);

    pox->state = PULSEOXIMETER_STATE_IDLE;
    return true;
}

void PulseOximeter_update(PulseOximeter_t *pox) {
    MAX30100_update(&pox->hrm);
    checkSample(pox);
    checkCurrentBias(pox);
}

float PulseOximeter_getHeartRate(PulseOximeter_t *pox) {
    return BeatDetector_getRate(&pox->beatDetector);
}

uint8_t PulseOximeter_getSpO2(PulseOximeter_t *pox) {
    return SpO2Calculator_getSpO2(&pox->spO2calculator);
}

void PulseOximeter_setOnBeatDetectedCallback(PulseOximeter_t *pox, void (*cb)()) {
    pox->onBeatDetected = cb;
}

void PulseOximeter_setIRLedCurrent(PulseOximeter_t *pox, LEDCurrent irLedNewCurrent) {
    pox->irLedCurrent = irLedNewCurrent;
    MAX30100_setLedsCurrent(&pox->hrm, pox->irLedCurrent, (LEDCurrent)pox->redLedCurrentIndex);
}

// --- Private (static) functions ---

static void checkSample(PulseOximeter_t *pox) {
    uint16_t rawIRValue, rawRedValue;

    while (MAX30100_getRawValues(&pox->hrm, &rawIRValue, &rawRedValue)) {
        pox->latestIRValue = rawIRValue; // Save the raw value for the main loop to use
        float irACValue = DCRemover_step(&pox->irDCRemover, (float)rawIRValue);
        float redACValue = DCRemover_step(&pox->redDCRemover, (float)rawRedValue);

        float filteredPulseValue = FilterBuLp1_step(&pox->lpf, -irACValue);
        bool beatDetected = BeatDetector_addSample(&pox->beatDetector, filteredPulseValue);

        if (BeatDetector_getRate(&pox->beatDetector) > 0) {
            pox->state = PULSEOXIMETER_STATE_DETECTING;
            SpO2Calculator_update(&pox->spO2calculator, irACValue, redACValue, beatDetected);
        } else if (pox->state == PULSEOXIMETER_STATE_DETECTING) {
            pox->state = PULSEOXIMETER_STATE_IDLE;
            SpO2Calculator_reset(&pox->spO2calculator);
        }

        if (beatDetected && pox->onBeatDetected) {
            pox->onBeatDetected();
        }
    }
}

static void checkCurrentBias(PulseOximeter_t *pox) {
    uint32_t now = HAL_GetTick();
    if (now - pox->tsLastBiasCheck > CURRENT_ADJUSTMENT_PERIOD_MS) {
        bool changed = false;
        float irDCW = DCRemover_getDCW(&pox->irDCRemover);
        float redDCW = DCRemover_getDCW(&pox->redDCRemover);

        if (irDCW - redDCW > 70000 && pox->redLedCurrentIndex < MAX30100_LED_CURR_50MA) {
            ++(pox->redLedCurrentIndex);
            changed = true;
        } else if (redDCW - irDCW > 70000 && pox->redLedCurrentIndex > 0) {
            --(pox->redLedCurrentIndex);
            changed = true;
        }

        if (changed) {
            MAX30100_setLedsCurrent(&pox->hrm, pox->irLedCurrent, (LEDCurrent)pox->redLedCurrentIndex);
        }
        pox->tsLastBiasCheck = now;
    }
}
