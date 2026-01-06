/*
 * MAX30100_SpO2Calculator.h
 *
 *  Created on: Sep 24, 2025
 *      Author: Hemavathi
 */

#ifndef INC_MAX30100_SPO2CALCULATOR_H_
#define INC_MAX30100_SPO2CALCULATOR_H_

#include <stdint.h>
#include <stdbool.h>

#define CALCULATE_EVERY_N_BEATS 3

// The C struct that replaces the C++ class
typedef struct {
    float irACValueSqSum;
    float redACValueSqSum;
    uint8_t beatsDetectedNum;
    uint32_t samplesRecorded;
    uint8_t spO2;
} SpO2Calculator_t;

// C function prototypes
void SpO2Calculator_init(SpO2Calculator_t *spo2);
void SpO2Calculator_update(SpO2Calculator_t *spo2, float irACValue, float redACValue, bool beatDetected);
void SpO2Calculator_reset(SpO2Calculator_t *spo2);
uint8_t SpO2Calculator_getSpO2(SpO2Calculator_t *spo2);


#endif /* INC_MAX30100_SPO2CALCULATOR_H_ */
