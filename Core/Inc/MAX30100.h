/*
 * MAX30100.h
 *
 *  Created on: Sep 24, 2025
 *      Author: Hemavathi
 */

#ifndef INC_MAX30100_H_
#define INC_MAX30100_H_

#include "stm32f1xx_hal.h"
#include "CircularBuffer.h"
#include "MAX30100_Registers.h"
#include <stdbool.h>

#define DEFAULT_MODE                MAX30100_MODE_SPO2_HR
#define DEFAULT_SAMPLING_RATE       MAX30100_SAMPRATE_100HZ
#define DEFAULT_PULSE_WIDTH         MAX30100_SPC_PW_1600US_16BITS
#define DEFAULT_RED_LED_CURRENT     MAX30100_LED_CURR_50MA
#define DEFAULT_IR_LED_CURRENT      MAX30100_LED_CURR_50MA

// The main struct to hold our sensor's data and configuration
typedef struct {
    I2C_HandleTypeDef *i2c_handle;
    CircularBuffer readoutsBuffer;
} MAX30100_t;

// C Function Prototypes (converted from C++ methods)
bool MAX30100_begin(MAX30100_t *sensor, I2C_HandleTypeDef *i2c_handle);
void MAX30100_setMode(MAX30100_t *sensor, Mode mode);
void MAX30100_setLedsPulseWidth(MAX30100_t *sensor, LEDPulseWidth ledPulseWidth);
void MAX30100_setSamplingRate(MAX30100_t *sensor, SamplingRate samplingRate);
void MAX30100_setLedsCurrent(MAX30100_t *sensor, LEDCurrent irLedCurrent, LEDCurrent redLedCurrent);
void MAX30100_setHighresModeEnabled(MAX30100_t *sensor, bool enabled);
void MAX30100_update(MAX30100_t *sensor);
bool MAX30100_getRawValues(MAX30100_t *sensor, uint16_t *ir, uint16_t *red);
void MAX30100_resetFifo(MAX30100_t *sensor);
void MAX30100_shutdown(MAX30100_t *sensor);
void MAX30100_resume(MAX30100_t *sensor);
uint8_t MAX30100_getPartId(MAX30100_t *sensor);


#endif /* INC_MAX30100_H_ */
