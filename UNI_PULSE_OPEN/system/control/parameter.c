/*
 * parameter.c
 *
 *  Created on: 2023. 10. 26.
 *      Author: JH
 */

#include "parameter.h"

SYSTEM_CONFIGURATION_VARIABLE   system_configration_variable;
PULSE_INPUT_MODE                pulse_input_mode;
INITIAL_PARAMETER               initial_parameter;

typedef struct {
    uint16_t   p_gain;
    uint16_t   d_gain;
}PARAM_GAIN;

float32_t param_run_current [] =
{
                             0.5,
                             0.8,
                             1.0,
                             1.13,
                             1.2,
                             1.3,
                             1.5,
                             1.8,
                             2.0,
                             2.2,
                             2.8,
                             3.0,
                             3.5,
                             4.0,
                             0,
                             0
};

uint16_t param_stop_currnet [] =
{
                             30,
                             50,
                             70,
                             90
};

uint32_t param_resoltion [] =
{
                             500,
                             1000,
                             1600,
                             2000,
                             3200,
                             3600,
                             4000,
                             5000,
                             6400,
                             8000,
                             10000,
                             20000,
                             25000,
                             36000,
                             40000,
                             50000
};


PARAM_GAIN param_gain[16][4] =
{
  /*    size    current           // S          // M          // L          // XL */
  /*    all       0.5     */     {{5,  25},    {15, 25},     {20, 40},     {25, 80}},
  /*    20        0.8     */     {{5,  25},    {15, 25},     {20, 40},     {25, 80}},
  /*    all       1.0     */     {{5,  25},    {15, 25},     {20, 40},     {25, 80}},
  /*    42        1.13    */     {{15, 55},    {15, 20},     {20, 55},     {25, 120}},
  /*    all       1.2     */     {{15, 25},    {15, 20},     {20, 40},     {25, 80}},
  /*    28        1.3     */     {{17, 26},    {5,  25},     {15, 20},     {20, 40}},
  /*    all       1.5     */     {{15, 25},    {15, 20},     {20, 40},     {25, 80}},
  /*    all       1.8     */     {{15, 25},    {15, 20},     {20, 40},     {25, 80}},
  /*    35        2.0     */     {{5,  25},    {15, 20},     {20, 55},     {30, 90}},
  /*    42        2.0     */     {{20, 40},    {15, 20},     {30, 90},     {25, 200}},
  /*    all       2.2     */     {{5, 40},     {15, 20},     {20, 40},     {25, 80}},
  /*    56        2.8     */     {{25,120},    {48,120},     {30,132},     {35, 80}},
  /*    56        3.0     */     {{30,100},    {35,120},     {30,132},     {35, 40}},
  /*    60        3.0     */     {{15, 55},    {19, 74},     {30,120},     {30, 40}},
  /*    all       3.5     */     {{5,  60},    {25, 60},     {30,120},     {30, 60}},
  /*    60        4.0     */     {{5,  60},    {25, 60},     {30,120},     {30, 60}}

};

PARAM_GAIN param_gain_other[4] =
{
 /* unknown motor gain step      // S          // M          // L          // XL */
                                 {5,  25},    {15, 25},     {20, 40},     {25, 80}
};


void get_system_parameter(void)
{
    initial_parameter.run_current       = param_run_current [(uint16_t) system_configration_variable.data.run_current] ;
    initial_parameter.stop_current      = param_stop_currnet [(uint16_t) system_configration_variable.data.stop_current] ;
    initial_parameter.resolution        = param_resoltion [(uint16_t) system_configration_variable.data.resoulution] ;
    initial_parameter.input_mode        = (uint16_t)system_configration_variable.data.input_mode;
    initial_parameter.motor_direction   = (uint16_t)system_configration_variable.data.motor_direction;
    initial_parameter.selftest          = (uint16_t)system_configration_variable.data.selftest;
    initial_parameter.gain_sel          = (uint16_t)system_configration_variable.data.gain_sel;
    if(initial_parameter.gain_sel == true)
    {
        initial_parameter.p_gain            = (float32_t)param_gain [(uint16_t) system_configration_variable.data.run_current]
                                                         [(uint16_t) system_configration_variable.data.gain].p_gain;
        initial_parameter.d_gain            = (float32_t)param_gain [(uint16_t) system_configration_variable.data.run_current]
                                                         [(uint16_t) system_configration_variable.data.gain].d_gain;
    }
    else
    {
        initial_parameter.p_gain            = (float32_t)param_gain_other [(uint16_t) system_configration_variable.data.gain].p_gain;
        initial_parameter.d_gain            = (float32_t)param_gain_other [(uint16_t) system_configration_variable.data.gain].d_gain;
    }
}


