/*
 * datprocessing.h
 *
 *  Created on: 2023. 8. 2.
 *      Author: JH
 */

#ifndef SYSTEM_CONTROL_DATPROCESSING_H_
#define SYSTEM_CONTROL_DATPROCESSING_H_

#include "driverlib.h"


typedef struct{
    uint16_t cutoff_freq;
    float32_t in[2];
    float32_t out[2];
    float32_t coef_a;
    float32_t coef_b;
}LOW_PASS_FILTER;

typedef struct {
    int16_t cnt;
    int16_t order;
    float32_t buf[256];
    float32_t Average,temp;
    float32_t Namuge;
    float32_t TotalExtPosRef;
    float64_t Total;
}MOVING_AVERAGE_FILTER;


typedef struct{
    float64_t freq_z, zeta_z, freq_p, zeta_p, Ts;
    float64_t K, n1, n2, d1, d2;  // filter coefficients
    float64_t x_old_1, x_old_2, y_old_1, y_old_2; // filter internal variables
    float64_t Remainder;
    float64_t Notch_out;
    int32_t step;
} SNOTCH;


void lpf_initialize(float32_t fc, LOW_PASS_FILTER *Flt, float32_t smaplingtime);
float32_t filter_lpf(LOW_PASS_FILTER *Flt, float32_t nFlt_In);
void maf_initialize(MOVING_AVERAGE_FILTER *Flt, int16_t order);
float32_t filter_maf(MOVING_AVERAGE_FILTER *Flt, float32_t CmdIn);
void notch_initialize(float64_t freq_z, float64_t zeta_z,float64_t freq_p, float64_t zeta_p, float64_t Ts, SNOTCH *N);
float32_t filter_notch(SNOTCH *N, float64_t x_new);


#endif /* SYSTEM_CONTROL_DATPROCESSING_H_ */
