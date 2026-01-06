/*
 * MAX30100_Filters.h
 *
 *  Created on: Sep 24, 2025
 *      Author: Hemavathi
 */

#ifndef INC_MAX30100_FILTERS_H_
#define INC_MAX30100_FILTERS_H_

#include <stdint.h>

// --- DC Remover Filter ---

typedef struct {
    float alpha;
    float dcw;
} DCRemover_t;

void DCRemover_init(DCRemover_t *filter, float alpha);
float DCRemover_step(DCRemover_t *filter, float x);
float DCRemover_getDCW(DCRemover_t *filter);

// --- Butterworth Low-Pass Filter ---

typedef struct {
    float v[2];
} FilterBuLp1_t;

void FilterBuLp1_init(FilterBuLp1_t *filter);
float FilterBuLp1_step(FilterBuLp1_t *filter, float x);


#endif /* INC_MAX30100_FILTERS_H_ */
