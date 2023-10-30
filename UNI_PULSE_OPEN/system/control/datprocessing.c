/*
 * datprocessing.c
 *
 *  Created on: 2023. 8. 2.
 *      Author: JH
 */

#include "stdlib.h"
#include "math.h"
#include "common.h"
#include "datprocessing.h"

void lpf_initialize(float32_t fc, LOW_PASS_FILTER *Flt, float32_t smaplingtime)
{
    float32_t wc, wcT;
    float32_t temp_a, temp_b;

    Flt->cutoff_freq = fc;
    Flt->in[0] = Flt->in[1] = 0;
    Flt->out[0] = Flt->out[1] = 0;

    wc = 2.0 * PI * Flt->cutoff_freq;
    wcT = wc * smaplingtime;

    temp_a  = wcT/(2.0+wcT);
    temp_b  = 1 - (temp_a*2);

    Flt->coef_a = temp_a;
    Flt->coef_b = temp_b;
}


float32_t filter_lpf(LOW_PASS_FILTER *Flt, float32_t nFlt_In)
{
    static float32_t Flt_out;

    Flt->in[0] = nFlt_In;

    Flt->out[0] = Flt->coef_a*(Flt->in[0]+Flt->in[1]) + Flt->coef_b * Flt->out[1];
    Flt->in[1] = Flt->in[0];
    Flt->out[1] = Flt->out[0];
    Flt_out = Flt->out[0];

    return Flt_out;
}

void maf_initialize(MOVING_AVERAGE_FILTER *Flt, int16_t order)
{
    uint16_t i;

    if( order >= 0 && order < 256) { Flt->order = order+1; }
    else if( order < 0)            { Flt->order = 1; }
    else if( order > (256-1) )        { Flt->order = 256; }

    for( i = 0; i < Flt->order; i++) { Flt->buf[i] = 0; }

    Flt->cnt = 0;
    Flt->Namuge = 0;
    Flt->Average = 0;
    Flt->Total = 0;
    Flt->TotalExtPosRef = 0;
}

float32_t filter_maf(MOVING_AVERAGE_FILTER *Flt, float32_t CmdIn)
{
    if( Flt->order == 1)
    {
        Flt->TotalExtPosRef += CmdIn;
        return CmdIn;
    }

    Flt->Total += (CmdIn - Flt->buf[Flt->cnt]);
    Flt->temp = (Flt->Total/ Flt->order);
    Flt->Namuge += (Flt->Total)-(Flt->temp * Flt->order);

    if( Flt->Namuge >= Flt->order)
    {
        Flt->Namuge -= Flt->order;
        ++Flt->temp;
    }
    else if( Flt->Namuge <= -Flt->order)
    {
        Flt->Namuge += Flt->order;
        --Flt->temp;
    }

    Flt->buf[Flt->cnt++] = CmdIn;
    if( Flt->cnt >= Flt->order) { Flt->cnt = 0; }
    Flt->TotalExtPosRef += Flt->Average;

    return (Flt->Average = Flt->temp);
}

void notch_initialize(float64_t freq_z, float64_t zeta_z,float64_t freq_p, float64_t zeta_p, float64_t Ts, SNOTCH *N)
{
/*
    freq_z : cutoff frequency [Hz]
    zeta_z : damping constant at cutoff frequency [pu]
    freq_p : pass frequency [Hz]
    zeta_p : damping constant at pass frequency [pu]
    Ts       : sampling time [sec]
    N            : pointer to structure sNotch
*/
    float64_t alpha_z, alpha_p;

    N->Remainder = 0.0;
    N->x_old_1 = 0.0;
    N->x_old_2 = 0.0;
    N->y_old_1 = 0.0;
    N->y_old_2 = 0.0;

    if(freq_z >= freq_p || zeta_z > 1.0 || zeta_p > 1.0){
    N->n1 = 0.0;        N->n2 = 0.0;
    N->d1 = 0.0;        N->d2 = 0.0;
    N->K = 0.0;
    return;
    }

    alpha_z = 1.0 + 2.0 * zeta_z * 2.0 * PI * freq_z * Ts + pow((2.0 * PI * freq_z * Ts),2.0);
    alpha_p = 1.0 + 2.0 * zeta_p * 2.0 * PI * freq_p * Ts + pow((2.0 * PI * freq_p * Ts),2.0);

    N->n1 = -2.0 * (1.0 + zeta_z * 2.0 * PI * freq_z * Ts) / alpha_z;
    N->n2 = 1.0 / alpha_z;

    N->d1 = -2.0 * (1.0 + zeta_p * 2.0 * PI * freq_p * Ts) / alpha_p;
    N->d2 = 1.0 / alpha_p;

    N->K = pow((freq_p / freq_z),2) * alpha_z / alpha_p;
}

float32_t filter_notch(SNOTCH *N, float64_t x_new)
{
    float64_t y;
    float64_t alpha_z, alpha_p;

    switch(N->step)
    {
        case 0:
        N->Remainder = 0.0;
        N->x_old_1 = 0.0;
        N->x_old_2 = 0.0;
        N->y_old_1 = 0.0;
        N->y_old_2 = 0.0;

        if(N->freq_z >= N->freq_p || N->zeta_z > 1.0 || N->zeta_p > 1.0)
        {
            N->n1 = 0.0;        N->n2 = 0.0;
            N->d1 = 0.0;        N->d2 = 0.0;
            N->K = 0.0;
            return 0;
        }

        alpha_z = 1.0 + 2.0 * N->zeta_z * 2.0 * PI * N->freq_z * N->Ts + pow((2.0 * PI * N->freq_z * N->Ts),2.0);
        alpha_p = 1.0 + 2.0 * N->zeta_p * 2.0 * PI * N->freq_p * N->Ts + pow((2.0 * PI * N->freq_p * N->Ts),2.0);

        N->n1 = -2.0 * (1.0 + N->zeta_z * 2.0 * PI * N->freq_z * N->Ts) / alpha_z;
        N->n2 = 1.0 / alpha_z;

        N->d1 = -2.0 * (1.0 + N->zeta_p * 2.0 * PI * N->freq_p * N->Ts) / alpha_p;
        N->d2 = 1.0 / alpha_p;

        N->K = pow((N->freq_p / N->freq_z),2) * alpha_z / alpha_p;
        N->step = 1;


        case 1:
        y = N->K * (x_new + N->n1 * N->x_old_1 + N->n2 * N->x_old_2) - N->d1 * N->y_old_1 - N->d2 * N->y_old_2;
        N->x_old_2 = N->x_old_1;
        N->x_old_1 = x_new;
        N->y_old_2 = N->y_old_1;
        N->y_old_1 = y;

        N->Notch_out = y;
    }
    return (float32_t)N->Notch_out;
}


