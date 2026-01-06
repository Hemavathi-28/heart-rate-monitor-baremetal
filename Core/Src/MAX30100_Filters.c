/*
 * MAX30100_Filters.c
 *
 *  Created on: Sep 24, 2025
 *      Author: Hemavathi
 */

#include "MAX30100_Filters.h"
#include <string.h> // For memset

// --- DC Remover Filter Implementation ---

void DCRemover_init(DCRemover_t *filter, float alpha) {
    filter->alpha = alpha;
    filter->dcw = 0;
}

// Processes one sample, returns the DC-free value
float DCRemover_step(DCRemover_t *filter, float x) {
    float olddcw = filter->dcw;
    filter->dcw = x + filter->alpha * filter->dcw;
    return filter->dcw - olddcw;
}

float DCRemover_getDCW(DCRemover_t *filter) {
    return filter->dcw;
}

// --- Butterworth Low-Pass Filter Implementation ---

void FilterBuLp1_init(FilterBuLp1_t *filter) {
    // Zero out the state buffer
    memset(filter->v, 0, sizeof(filter->v));
}

// Processes one sample, returns the filtered value
float FilterBuLp1_step(FilterBuLp1_t *filter, float x) {
    filter->v[0] = filter->v[1];
    filter->v[1] = (2.452372752527856026e-1f * x) + (0.50952544949442879485f * filter->v[0]);
    return (filter->v[0] + filter->v[1]);
}
