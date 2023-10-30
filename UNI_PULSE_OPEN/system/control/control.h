/*
 * control.h
 *
 *  Created on: 2023. 10. 18.
 *      Author: JH
 */

#ifndef SYSTEM_CONTROL_H_
#define SYSTEM_CONTROL_H_

#include "driverlib.h"


#define MAF_1ST_ORDER       39
#define MAF_2ND_ORDER       9
#define MAF_3RD_ORDER       255

#define CUTOFF_FREQ_GAIN         3000
#define CUTOFF_FREQ_CURRENT      2500

#define MID_VOLT_POINT           (EPWM_TBPRD / 2)

#define RPM_TO_DELTA_ELECTRIC_THETA     ( 50.0 / 60.0 / 1000.0 / 40.0  ) //  pole pair / min to sec / sec to msec / msec to 25usec
#define SELP_TEST_RPM       (0.5)

#define D_GAIN_TH_SPEED         (100.0)
#define PWMVOUT_LPF_TH1_SPEED   (600.0)
#define PWMVOUT_LPF_TH2_SPEED   (700.0)

#define MONITORING_RPM_CNT      60

#define ENABLE      1
#define DISABLE     0

#define MOTOR_STOP_TIME         1000 // [ms]
#define MOTOR_RUN   1
#define MOTOR_STOP  0

typedef enum {
    CONTROL_MOTOR_DETECTION,
    CONTROL_PHASE_FINDING,
    CONTROL_OPEN_LOOP,
    CONTROL_SELF_TEST,
}PWM_CONTROL_MODE;
extern     PWM_CONTROL_MODE   control_mode;

typedef struct{
    float32_t     a_ref,a_real;
    float32_t     b_ref,b_real;
    float32_t     ab_real;
    float32_t     Kp;
    float32_t     Ki;
    float32_t     Kd;
    float32_t     Kb;
    float32_t     apwm,bpwm;
    float32_t     a_err,a_err_old;
    float32_t     b_err,b_err_old;
    float32_t     a_errd, b_errd;
    float32_t     a_kp_temp, a_kd_temp, b_kp_temp, b_kd_temp;
    float32_t     a_pwmout,b_pwmout;
    float32_t     a_lpf_pwmout,b_lpf_pwmout;
}CUR_PID;
extern CUR_PID  cur_pid;

typedef struct{
    int32_t       curcnt,precnt;
    int32_t       deltacnt;
    int32_t       cmdcnt;
    int32_t       total;
    int32_t       v_cmdcnt;
    int32_t       v_curcnt,v_precnt;
    int32_t       v_deltacnt;
    int32_t       v_total;
    int32_t       cal_pos;
    int32_t       m_curcnt,m_precnt;
    int32_t       m_deltacnt;
}CMD_POS;

typedef struct{
    float32_t     cmd_speed;
    float32_t     wrpm_real;
    float32_t     m_cmd_speed;
    float32_t     m_wrpm_real;
    float64_t     cmd_theta;
    float64_t     real_theta;
    float32_t     isq;
    float32_t     run_cur_ref;
    float32_t     stop_cur_ref;
    float32_t     actual_cur_ref;
    float32_t     phasefinding_cur_ref;
    float32_t     THETA_PULSE_TO_DEG_CONST;
    float32_t     Semi_opne_Degree;
    float32_t     CONTROL_RPM_CONST;
    float32_t     MONITORING_RPM_CONSTANT;
    float32_t     CURRENT_LIMIT;
}STEP_CONTROL;
extern STEP_CONTROL cstep;

typedef struct{
    int32_t       curcnt;
    int32_t       precnt;
    int32_t       deltacnt;
    int32_t       v_curcnt;
    int32_t       v_precnt;
    int32_t       v_deltacnt;
    int32_t       m_curcnt;
    int32_t       m_precnt;
    int32_t       m_deltacnt;
}PULSE;

typedef struct{
    uint16_t      cnt;
}PHASE_FIND;

typedef enum {
    INIT_PHASE_FIND,
    SET_RATED_CURRENT,
    SET_10P_OVER_CURRENT,
    SET_CONTROL_MODE,
    DONE_PHASE_FIND,
}PHASE_FIND_MODE;
extern PHASE_FIND_MODE         phase_find_mode;

typedef struct{
    uint16_t      cnt;
    float32_t     a_pwm_voltage;
    float32_t     theta;
    float32_t     vlotage_average;
    float32_t     current_average;
    float32_t     im,vm,z;
    float32_t     motor_ls;
    float32_t     motor_dcr;
}MOTOR_DETECT;

typedef enum {
    INIT_MOTOR_DETECT,
    SET_TEST_CURRENT,
    CALCULATE_DCR,
    GENERATE_1KHZ,
    CALCULATE_RL,
}MOTOR_DETECT_MODE;

typedef struct{
    float32_t   dc_link;
    float32_t   device_temp;
    float32_t   dsp_temp;
}SYSTEM_CONDITION;
extern SYSTEM_CONDITION     system_condition;

typedef union {
    struct  {
        unsigned    ready           :1;
        unsigned    servo_state     :1;
        unsigned    motor_state     :1;
        unsigned    io_enable_state :1;
        unsigned    reserved        :13;
    }Bit;
    int16_t    Full;
}DRIVER_STATUS;
extern DRIVER_STATUS driver_status;




void control_pwm(void);
void check_system_condtion(void);
void initialze_variable(void);


#endif /* SYSTEM_CONTROL_H_ */

