/*
 * MAX30100.c
 *
 *  Created on: Sep 24, 2025
 *      Author: Hemavathi
 */

#include "MAX30100.h"

// Private (static) function prototypes
static uint8_t readRegister(MAX30100_t *sensor, uint8_t address);
static void writeRegister(MAX30100_t *sensor, uint8_t address, uint8_t data);
static void burstRead(MAX30100_t *sensor, uint8_t baseAddress, uint8_t *buffer, uint8_t length);
static void readFifoData(MAX30100_t *sensor);

// Main begin function
bool MAX30100_begin(MAX30100_t *sensor, I2C_HandleTypeDef *i2c_handle)
{
    sensor->i2c_handle = i2c_handle;
    CircularBuffer_init(&sensor->readoutsBuffer);

    if (MAX30100_getPartId(sensor) != EXPECTED_PART_ID) {
        return false;
    }

    MAX30100_setMode(sensor, DEFAULT_MODE);
    MAX30100_setLedsPulseWidth(sensor, DEFAULT_PULSE_WIDTH);
    MAX30100_setSamplingRate(sensor, DEFAULT_SAMPLING_RATE);
    MAX30100_setLedsCurrent(sensor, DEFAULT_IR_LED_CURRENT, DEFAULT_RED_LED_CURRENT);
    MAX30100_setHighresModeEnabled(sensor, true);

    return true;
}

void MAX30100_setMode(MAX30100_t *sensor, Mode mode)
{
    writeRegister(sensor, MAX30100_REG_MODE_CONFIGURATION, mode);
}

void MAX30100_setLedsPulseWidth(MAX30100_t *sensor, LEDPulseWidth ledPulseWidth)
{
    uint8_t previous = readRegister(sensor, MAX30100_REG_SPO2_CONFIGURATION);
    writeRegister(sensor, MAX30100_REG_SPO2_CONFIGURATION, (previous & 0xfc) | ledPulseWidth);
}

void MAX30100_setSamplingRate(MAX30100_t *sensor, SamplingRate samplingRate)
{
    uint8_t previous = readRegister(sensor, MAX30100_REG_SPO2_CONFIGURATION);
    writeRegister(sensor, MAX30100_REG_SPO2_CONFIGURATION, (previous & 0xe3) | (samplingRate << 2));
}

void MAX30100_setLedsCurrent(MAX30100_t *sensor, LEDCurrent irLedCurrent, LEDCurrent redLedCurrent)
{
    writeRegister(sensor, MAX30100_REG_LED_CONFIGURATION, (redLedCurrent << 4) | irLedCurrent);
}

void MAX30100_setHighresModeEnabled(MAX30100_t *sensor, bool enabled)
{
    uint8_t previous = readRegister(sensor, MAX30100_REG_SPO2_CONFIGURATION);
    if (enabled) {
        writeRegister(sensor, MAX30100_REG_SPO2_CONFIGURATION, previous | MAX30100_SPC_SPO2_HI_RES_EN);
    } else {
        writeRegister(sensor, MAX30100_REG_SPO2_CONFIGURATION, previous & ~MAX30100_SPC_SPO2_HI_RES_EN);
    }
}

void MAX30100_update(MAX30100_t *sensor)
{
    readFifoData(sensor);
}

bool MAX30100_getRawValues(MAX30100_t *sensor, uint16_t *ir, uint16_t *red)
{
    if (!CircularBuffer_isEmpty(&sensor->readoutsBuffer)) {
        // Use shift() for correct FIFO (First-In, First-Out) behavior
        SensorReadout readout = CircularBuffer_shift(&sensor->readoutsBuffer);
        *ir = readout.ir;
        *red = readout.red;
        return true;
    }
    return false;
}

void MAX30100_resetFifo(MAX30100_t *sensor)
{
    writeRegister(sensor, MAX30100_REG_FIFO_WRITE_POINTER, 0);
    writeRegister(sensor, MAX30100_REG_FIFO_READ_POINTER, 0);
    writeRegister(sensor, MAX30100_REG_FIFO_OVERFLOW_COUNTER, 0);
}

void MAX30100_shutdown(MAX30100_t *sensor)
{
    uint8_t modeConfig = readRegister(sensor, MAX30100_REG_MODE_CONFIGURATION);
    writeRegister(sensor, MAX30100_REG_MODE_CONFIGURATION, modeConfig | MAX30100_MC_SHDN);
}

void MAX30100_resume(MAX30100_t *sensor)
{
    uint8_t modeConfig = readRegister(sensor, MAX30100_REG_MODE_CONFIGURATION);
    writeRegister(sensor, MAX30100_REG_MODE_CONFIGURATION, modeConfig & ~MAX30100_MC_SHDN);
}

uint8_t MAX30100_getPartId(MAX30100_t *sensor)
{
    return readRegister(sensor, 0xFF);
}

// --- Private (static) functions ---

static uint8_t readRegister(MAX30100_t *sensor, uint8_t address)
{
    uint8_t value;
    HAL_I2C_Master_Transmit(sensor->i2c_handle, MAX30100_I2C_ADDRESS, &address, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(sensor->i2c_handle, MAX30100_I2C_ADDRESS, &value, 1, HAL_MAX_DELAY);
    return value;
}

static void writeRegister(MAX30100_t *sensor, uint8_t address, uint8_t data)
{
    uint8_t buffer[2] = {address, data};
    HAL_I2C_Master_Transmit(sensor->i2c_handle, MAX30100_I2C_ADDRESS, buffer, 2, HAL_MAX_DELAY);
}

static void burstRead(MAX30100_t *sensor, uint8_t baseAddress, uint8_t *buffer, uint8_t length)
{
    HAL_I2C_Master_Transmit(sensor->i2c_handle, MAX30100_I2C_ADDRESS, &baseAddress, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(sensor->i2c_handle, MAX30100_I2C_ADDRESS, buffer, length, HAL_MAX_DELAY);
}

static void readFifoData(MAX30100_t *sensor)
{
    // Using "static" here moves the buffer out of the stack, helping prevent overflows.
    static uint8_t buffer[CBUFFER_SIZE * 4];
    uint8_t toRead;

    uint8_t writePointer = readRegister(sensor, MAX30100_REG_FIFO_WRITE_POINTER);
    uint8_t readPointer = readRegister(sensor, MAX30100_REG_FIFO_READ_POINTER);

    if (writePointer != readPointer) {
        toRead = (writePointer - readPointer) & (CBUFFER_SIZE - 1);
        if (toRead > 0) {
            burstRead(sensor, MAX30100_REG_FIFO_DATA, buffer, 4 * toRead);
            for (uint8_t i = 0; i < toRead; ++i) {
                SensorReadout readout;
                readout.ir = (uint16_t)((buffer[i * 4] << 8) | buffer[i * 4 + 1]);
                // This line is now corrected to match the original library's logic
                readout.red = (uint16_t)((buffer[i * 4 + 2] << 8) | buffer[i * 4 + 3]);
                CircularBuffer_push(&sensor->readoutsBuffer, readout);
            }
        }
    }
}
