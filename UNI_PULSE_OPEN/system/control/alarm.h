/*
 * alarm.h
 *
 *  Created on: 2023. 8. 2.
 *      Author: JH
 */

#ifndef SYSTEM_CONTROL_ALARM_H_
#define SYSTEM_CONTROL_ALARM_H_

#include "driverlib.h"

#define DC_LINK_LIMIT_UPPER     ( VDC + 15 )
#define DC_LIMK_LIMIT_LOWER     ( VDC - 3 )
#define DC_LINK_LIMIT_CNT       50 // 50ms

#define DEVIECE_TEMP_UPPER      70.0
#define DEVIECE_TEMP_LOWER      -40.0
#define DSP_TEMP_UPPER          105.0
#define DSP_TEMP_LOWER          -40.0
#define TEMP_LIMIT_CNT          1000

#define CMD_RPM_LIMIT           3500
#define CMD_RPM_LIMIT_CNT       10

#define RESETABLE_ALARM         0x1F

extern float32_t        MOTOR_OVERLOAD_LIMIT;



typedef union {
    struct  {
        unsigned    over_current_A            :1;
        unsigned    over_current_Abar         :1;
        unsigned    over_current_B            :1;
        unsigned    over_current_Bbar         :1;
        unsigned    fail_phase_find           :1;
        unsigned    over_voltage              :1;
        unsigned    under_voltage             :1;
        unsigned    device_over_temp          :1;
        unsigned    device_under_temp         :1;
        unsigned    dsp_over_temp             :1;
        unsigned    dsp_under_temp            :1;
        unsigned    cmd_over_rpm              :1;
        unsigned    motor_over_load           :1;
        unsigned    reserved1                 :4;
        unsigned    reserved2                 :15;
    }Bit;
    int32_t    Full;
}ERR_STATUS;
extern ERR_STATUS   err_status;

typedef struct {
    uint16_t    over_voltage;
    uint16_t    under_voltage;
    uint16_t    device_over_temp;
    uint16_t    device_under_temp;
    uint16_t    dsp_over_temp;
    uint16_t    dsp_under_temp;
    uint16_t    cmd_over_rpm;
}ALARM_CNT;
extern ALARM_CNT    alarm_cnt;



void check_system_alarm(void);
void reset_alarm(void);


#endif /* SYSTEM_CONTROL_ALARM_H_ */
