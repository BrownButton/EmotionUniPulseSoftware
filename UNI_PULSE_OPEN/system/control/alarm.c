/*
 * alarm.c
 *
 *  Created on: 2023. 8. 2.
 *      Author: JH
 */
#include "math.h"
#include "string.h"
#include "parameter.h"
#include "common.h"
#include "control.h"
#include "board.h"
#include "alarm.h"


ERR_STATUS      err_status;
ALARM_CNT       alarm_cnt;
float32_t       MOTOR_OVERLOAD_LIMIT;
float32_t       motor_load_current;

void check_system_alarm(void)
{

    // check over voltage
    if(system_condition.dc_link > DC_LINK_LIMIT_UPPER) { alarm_cnt.over_voltage++; }
    else { alarm_cnt.over_voltage = 0; }
    if(alarm_cnt.over_voltage >= DC_LINK_LIMIT_CNT)
    {
        err_status.Bit.over_voltage = true;
        alarm_cnt.over_voltage = DC_LINK_LIMIT_CNT ;
    }
    // check under voltage
    if(system_condition.dc_link < DC_LIMK_LIMIT_LOWER)  { alarm_cnt.under_voltage++; }
    else { alarm_cnt.under_voltage = 0; }
    if(alarm_cnt.under_voltage >= DC_LINK_LIMIT_CNT)
    {
        err_status.Bit.under_voltage = true;
        alarm_cnt.under_voltage = DC_LINK_LIMIT_CNT;
    }


    // check device over temperature
    if(system_condition.device_temp > DEVIECE_TEMP_UPPER)  { alarm_cnt.device_over_temp++; }
    else { alarm_cnt.device_over_temp = 0; }
    if(alarm_cnt.device_over_temp >= TEMP_LIMIT_CNT)
    {
        err_status.Bit.device_over_temp = true;
        alarm_cnt.device_over_temp = TEMP_LIMIT_CNT;
    }
    // check device under temperature
    if(system_condition.device_temp < DEVIECE_TEMP_LOWER)  { alarm_cnt.device_under_temp++; }
    else { alarm_cnt.device_under_temp = 0; }
    if(alarm_cnt.device_under_temp >= TEMP_LIMIT_CNT)
    {
        err_status.Bit.device_under_temp = true;
        alarm_cnt.device_under_temp = TEMP_LIMIT_CNT;
    }


    // check dsp over temperature
    if(system_condition.dsp_temp > DSP_TEMP_UPPER)  { alarm_cnt.dsp_over_temp++; }
    else { alarm_cnt.dsp_over_temp = 0; }
    if(alarm_cnt.dsp_over_temp >= TEMP_LIMIT_CNT)
    {
        err_status.Bit.dsp_over_temp = true;
        alarm_cnt.dsp_over_temp = TEMP_LIMIT_CNT;
    }
    // check dsp under temperature
    if(system_condition.dsp_temp < DSP_TEMP_LOWER)  { alarm_cnt.dsp_under_temp++; }
    else { alarm_cnt.dsp_under_temp = 0; }
    if(alarm_cnt.dsp_under_temp >= TEMP_LIMIT_CNT)
    {
        err_status.Bit.dsp_under_temp = true;
        alarm_cnt.dsp_under_temp = TEMP_LIMIT_CNT;
    }


    // check command over rpm
    if( abs(cstep.cmd_speed) > CMD_RPM_LIMIT ) { alarm_cnt.cmd_over_rpm++; }
    else { alarm_cnt.cmd_over_rpm = 0; }
    if(alarm_cnt.cmd_over_rpm >= CMD_RPM_LIMIT_CNT)
    {
        err_status.Bit.cmd_over_rpm = true;
        alarm_cnt.cmd_over_rpm = CMD_RPM_LIMIT_CNT;
    }

    // check motor over load
    motor_load_current += (pow(cur_pid.a_real, 2.0) + pow(cur_pid.b_real, 2.0)
                            - pow(initial_parameter.run_current * 1.15, 2.0)) * SPEED_CONTROL_SAMPLE_TIME;
    if(motor_load_current < 0) { motor_load_current = 0; }
    if(motor_load_current > MOTOR_OVERLOAD_LIMIT)
    {
        err_status.Bit.motor_over_load = true;
        motor_load_current = 0;
    }

    if(err_status.Full != 0)
    {
        if(phase_find_mode != DONE_PHASE_FIND)
        {
            driver_status.Bit.servo_state = sevo_control(DISABLE);
            err_status.Bit.fail_phase_find = true;
        }
    }

}


void reset_alarm(void)
{
    err_status.Full &= RESETABLE_ALARM;
}






