/*
 * parameter.h
 *
 *  Created on: 2023. 10. 26.
 *      Author: JH
 */

#ifndef SYSTEM_CONTROL_PARAMETER_H_
#define SYSTEM_CONTROL_PARAMETER_H_

#include "driverlib.h"


typedef enum{
    one_pulse,
    tow_pulse,
}PULSE_INPUT_MODE;
extern PULSE_INPUT_MODE pulse_input_mode;


typedef union {
    struct  {
        uint16_t    run_current3            :1;
        uint16_t    run_current2            :1;
        uint16_t    run_current1            :1;
        uint16_t    run_current0            :1;
        uint16_t    stop_current1           :1;
        uint16_t    stop_current0           :1;
        uint16_t    gain1                   :1;
        uint16_t    gain0                   :1;
        uint16_t    resoulgion3             :1;
        uint16_t    resoulgion2             :1;
        uint16_t    resoulgion1             :1;
        uint16_t    resoulgion0             :1;
        uint16_t    selftest                :1;
        uint16_t    input_mode              :1;
        uint16_t    motor_direction         :1;
        uint16_t    gain_sel                :1;
    }bit;

    struct  {
        uint16_t    run_current            :4;
        uint16_t    stop_current           :2;
        uint16_t    gain                   :2;
        uint16_t    resoulution            :4;
        uint16_t    selftest               :1;
        uint16_t    input_mode             :1;
        uint16_t    motor_direction        :1;
        uint16_t    gain_sel               :1;
    }data;

    uint16_t    full;
}SYSTEM_CONFIGURATION_VARIABLE;
extern SYSTEM_CONFIGURATION_VARIABLE   system_configration_variable;


typedef struct {
    float32_t   run_current;
    float32_t   p_gain;
    float32_t   d_gain;
    uint32_t    resolution;
    uint16_t    stop_current;
    uint16_t    selftest;
    uint16_t    input_mode;
    uint16_t    motor_direction;
    uint16_t    gain_sel;
}INITIAL_PARAMETER;
extern INITIAL_PARAMETER initial_parameter;


void get_system_parameter(void);


#endif /* SYSTEM_CONTROL_PARAMETER_H_ */
