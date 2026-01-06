/*
 * MAX30100_SpO2Calculator.c
 *
 *  Created on: Sep 24, 2025
 *      Author: Hemavathi
 */

#include "MAX30100_SpO2Calculator.h"
#include <math.h> // For the logf() function

// SaO2 Look-up Table
static const uint8_t spO2LUT[43] = {100,100,100,100,99,99,99,99,99,99,98,98,98,98,
                                  98,97,97,97,97,97,97,96,96,96,96,96,96,95,95,
                                  95,95,95,95,94,94,94,94,94,93,93,93,93,93};

void SpO2Calculator_init(SpO2Calculator_t *spo2) {
    spo2->irACValueSqSum = 0;
    spo2->redACValueSqSum = 0;
    spo2->beatsDetectedNum = 0;
    spo2->samplesRecorded = 0;
    spo2->spO2 = 0;
}

void SpO2Calculator_update(SpO2Calculator_t *spo2, float irACValue, float redACValue, bool beatDetected) {
    spo2->irACValueSqSum += irACValue * irACValue;
    spo2->redACValueSqSum += redACValue * redACValue;
    ++(spo2->samplesRecorded);

    if (beatDetected) {
        ++(spo2->beatsDetectedNum);
        if (spo2->beatsDetectedNum == CALCULATE_EVERY_N_BEATS) {

            // **IMPROVEMENT: Add division-by-zero protection**
            if (spo2->samplesRecorded == 0) {
                // This should not happen, but as a safeguard, reset and exit
                SpO2Calculator_reset(spo2);
                return;
            }

            float acSqRatio = 100.0f * logf(spo2->redACValueSqSum / spo2->samplesRecorded) / logf(spo2->irACValueSqSum / spo2->samplesRecorded);
            uint8_t index = 0;

            if (acSqRatio > 66) {
                index = (uint8_t)acSqRatio - 66;
            } else if (acSqRatio > 50) {
                index = (uint8_t)acSqRatio - 50;
            }

            if (index >= 43) {
                index = 42; // Clamp index to the size of the LUT
            }

            SpO2Calculator_reset(spo2);
            spo2->spO2 = spO2LUT[index];
        }
    }
}

void SpO2Calculator_reset(SpO2Calculator_t *spo2) {
    spo2->samplesRecorded = 0;
    spo2->redACValueSqSum = 0;
    spo2->irACValueSqSum = 0;
    spo2->beatsDetectedNum = 0;
    spo2->spO2 = 0;
}

uint8_t SpO2Calculator_getSpO2(SpO2Calculator_t *spo2) {
    return spo2->spO2;
}
