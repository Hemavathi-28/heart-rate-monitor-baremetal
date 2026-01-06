/*
 * CircularBuffer.c
 *
 *  Created on: Sep 24, 2025
 *      Author: Hemavathi
 */

#include "CircularBuffer.h"

void CircularBuffer_init(CircularBuffer* cb) {
    cb->head = 0;
    cb->tail = 0;
    cb->count = 0;
}

bool CircularBuffer_isEmpty(CircularBuffer* cb) {
    return cb->count == 0;
}

bool CircularBuffer_isFull(CircularBuffer* cb) {
    return cb->count == CBUFFER_SIZE;
}

// Adds an element to the end of the buffer (the tail)
bool CircularBuffer_push(CircularBuffer* cb, SensorReadout readout) {
    bool overwrite = CircularBuffer_isFull(cb);

    if (cb->count == 0) {
        // If buffer is empty, head and tail start at the same spot
        cb->head = 0;
        cb->tail = 0;
    } else {
        // Advance the tail, wrapping around if necessary
        cb->tail = (cb->tail + 1) % CBUFFER_SIZE;
    }

    cb->buffer[cb->tail] = readout;

    if (overwrite) {
        // If we overwrote, the head also moves forward
        cb->head = (cb->head + 1) % CBUFFER_SIZE;
        return false; // Return false to indicate overwrite
    } else {
        cb->count++;
        return true; // Return true for a clean push
    }
}

// Removes an element from the end of the buffer (LIFO with push)
SensorReadout CircularBuffer_pop(CircularBuffer* cb) {
    // Assumes buffer is not empty
    SensorReadout readout = cb->buffer[cb->tail];

    if (cb->tail == 0) {
        cb->tail = CBUFFER_SIZE - 1;
    } else {
        cb->tail--;
    }

    cb->count--;
    return readout;
}

// Removes an element from the beginning of the buffer (FIFO with push)
SensorReadout CircularBuffer_shift(CircularBuffer* cb) {
    // Assumes buffer is not empty
    SensorReadout readout = cb->buffer[cb->head];
    cb->head = (cb->head + 1) % CBUFFER_SIZE;
    cb->count--;
    return readout;
}
