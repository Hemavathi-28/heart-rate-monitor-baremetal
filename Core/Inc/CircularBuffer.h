/*
 * CircularBuffer.h
 *
 *  Created on: Sep 24, 2025
 *      Author: Hemavathi
 */

#ifndef INC_CIRCULARBUFFER_H_
#define INC_CIRCULARBUFFER_H_

#include <stdint.h>
#include <stdbool.h>

// The specific data type our buffer will hold
typedef struct {
    uint16_t ir;
    uint16_t red;
} SensorReadout;

// The size of the buffer, matching the original library
#define CBUFFER_SIZE 16

// The C struct that replaces the C++ class
typedef struct {
    SensorReadout buffer[CBUFFER_SIZE];
    uint8_t head;  // Index of the first item
    uint8_t tail;  // Index of the last item
    uint8_t count; // Number of items in the buffer
} CircularBuffer;

// C function prototypes
void CircularBuffer_init(CircularBuffer* cb);
bool CircularBuffer_isEmpty(CircularBuffer* cb);
bool CircularBuffer_isFull(CircularBuffer* cb);
bool CircularBuffer_push(CircularBuffer* cb, SensorReadout readout);
SensorReadout CircularBuffer_pop(CircularBuffer* cb);
SensorReadout CircularBuffer_shift(CircularBuffer* cb);

#endif /* INC_CIRCULARBUFFER_H_ */
