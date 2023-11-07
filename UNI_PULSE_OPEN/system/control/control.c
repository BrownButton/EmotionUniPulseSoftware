/*
 * control.c
 *
 *  Created on: 2023. 10. 18.
 *      Author: JH
 */
#include "math.h"
#include "string.h"
#include "parameter.h"
#include "common.h"
#include "datprocessing.h"
#include "board.h"
#include "alarm.h"
#include "io_process.h"
#include "control.h"

#pragma CODE_SECTION(read_current, ".TI.ramfunc");
#pragma CODE_SECTION(ADC_readPPBResult, ".TI.ramfunc");
#pragma CODE_SECTION(update_command_position, ".TI.ramfunc");
#pragma CODE_SECTION(read_cw_command, ".TI.ramfunc");
#pragma CODE_SECTION(read_ccw_command, ".TI.ramfunc");
#pragma CODE_SECTION(EQEP_getPosition, ".TI.ramfunc");
#pragma CODE_SECTION(filter_maf, ".TI.ramfunc");
#pragma CODE_SECTION(run_control_mode, ".TI.ramfunc");
#pragma CODE_SECTION(control_current_pid, ".TI.ramfunc");
#pragma CODE_SECTION(filter_lpf, ".TI.ramfunc");
#pragma CODE_SECTION(lpf_initialize, ".TI.ramfunc");
#pragma CODE_SECTION(set_voltage_duty, ".TI.ramfunc");
#pragma CODE_SECTION(EPWM_setCounterCompareValue, ".TI.ramfunc");




PULSE                   cw,ccw;
PHASE_FIND              phase_find;
PHASE_FIND_MODE         phase_find_mode;
MOTOR_DETECT            motor_detect;
MOTOR_DETECT_MODE       motor_detect_mode;
CMD_POS                 cmd_pos;
CUR_PID                 cur_pid;
STEP_CONTROL            cstep;
PWM_CONTROL_MODE        control_mode;
SYSTEM_CONDITION        system_condition;
DRIVER_STATUS           driver_status;


MOVING_AVERAGE_FILTER   cmd_maf_1st, cmd_maf_2nd, cmd_maf_3rd;
MOVING_AVERAGE_FILTER   cmd_velocity_maf, actual_velocity_maf;
LOW_PASS_FILTER         a_dgain_lpf, b_dgain_lpf, a_current_lpf, b_current_lpf;

void set_system_parameter(void);
void update_command_position(void);
void detect_motor_characteristic(void);
void find_motor_aphase(void);
float32_t rpm_to_electric_theta(float32_t rpm);
void run_control_mode(void);
void control_current_pid(void);
void calculate_monitoring_variable(void);
void control_stop_current(void);
void control_servo(void);



void check_system_condtion(void)
{
    // calculate command rpm
    calculate_monitoring_variable();

    // check system alarlm
    check_system_alarm();

    // control stop current
    if(driver_status.Bit.ready == true)
    { control_stop_current(); }

    io_process();

    //control servo on/off
    control_servo();

}



void control_pwm(void)
{
    int16_t apwm, bpwm;

    // update actual current
    read_current(&cur_pid.a_real, &cur_pid.b_real);

    // update command position every 1[ms]
    update_command_position();

    // open loop or self test mode
    run_control_mode();

    if(driver_status.Bit.servo_state == ENABLE)
    {
        //current pid control
        control_current_pid();

        apwm = (EPWM_TBPRD >> 1) - (int16_t)((float32_t)(EPWM_TBPRD >> 1) * cur_pid.apwm) ;
        bpwm = (EPWM_TBPRD >> 1) - (int16_t)((float32_t)(EPWM_TBPRD >> 1) * cur_pid.bpwm) ;

        if(apwm < 0 ){ apwm = 0; }
        else if(apwm >= EPWM_TBPRD) { apwm = EPWM_TBPRD - 1 ; }
        if(bpwm < 0 ){ bpwm = 0; }
        else if(bpwm >= EPWM_TBPRD) { bpwm = EPWM_TBPRD - 1 ; }

        set_voltage_duty((uint16_t)apwm, (uint16_t)bpwm);
    }
    else
    {
        lpf_initialize(CUTOFF_FREQ_GAIN, &a_dgain_lpf, CURRENT_CONTROL_SAMPLE_TIME);
        lpf_initialize(CUTOFF_FREQ_GAIN, &b_dgain_lpf, CURRENT_CONTROL_SAMPLE_TIME);
        lpf_initialize(CUTOFF_FREQ_CURRENT, &a_current_lpf, CURRENT_CONTROL_SAMPLE_TIME);
        lpf_initialize(CUTOFF_FREQ_CURRENT, &b_current_lpf, CURRENT_CONTROL_SAMPLE_TIME);

        cur_pid.a_err = cur_pid.a_err_old = cur_pid.a_errd
                = cur_pid.b_err = cur_pid.b_err_old = cur_pid.b_errd = 0;
    }
}

void run_control_mode(void)
{
    switch(control_mode)
    {
        case CONTROL_MOTOR_DETECTION:
            detect_motor_characteristic();
            cur_pid.a_ref = cstep.isq * __cospuf32(cstep.cmd_theta);
            cur_pid.b_ref = cstep.isq * __sinpuf32(cstep.cmd_theta);
            break;

        case CONTROL_PHASE_FINDING:
            find_motor_aphase();
            cur_pid.a_ref = cstep.isq * __cospuf32(cstep.cmd_theta);
            cur_pid.b_ref = cstep.isq * __sinpuf32(cstep.cmd_theta);
            break;

        case CONTROL_OPEN_LOOP:
            cstep.isq = cstep.actual_cur_ref;

            if( cstep.isq > cstep.CURRENT_LIMIT)        cstep.isq = cstep.CURRENT_LIMIT;
            else if( cstep.isq < -cstep.CURRENT_LIMIT)  cstep.isq = -cstep.CURRENT_LIMIT;

            cstep.cmd_theta = (float64_t)cmd_pos.cal_pos * (float64_t)cstep.THETA_PULSE_TO_DEG_CONST;
            cstep.cmd_theta -= (float64_t)((int64_t)cstep.cmd_theta);

            cur_pid.a_ref = cstep.isq * __cospuf32(cstep.cmd_theta);
            cur_pid.b_ref = cstep.isq * __sinpuf32(cstep.cmd_theta);
            break;

        case CONTROL_SELF_TEST:
            cstep.isq = cstep.actual_cur_ref;

            if( cstep.isq > cstep.CURRENT_LIMIT)        cstep.isq = cstep.CURRENT_LIMIT;
            else if( cstep.isq < -cstep.CURRENT_LIMIT)  cstep.isq = -cstep.CURRENT_LIMIT;

            cstep.cmd_theta += rpm_to_electric_theta(SELP_TEST_RPM);
            cstep.cmd_theta -= (float64_t)((int64_t)cstep.cmd_theta);

            cur_pid.a_ref = cstep.isq * __cospuf32(cstep.cmd_theta);
            cur_pid.b_ref = cstep.isq * __sinpuf32(cstep.cmd_theta);
            break;

        default :
            break;
    }
}

void detect_motor_characteristic(void)
{
    switch(motor_detect_mode)
    {
        case INIT_MOTOR_DETECT :
            epwm_aq_enable_aphase();
            driver_status.Bit.servo_state = sevo_control(ENABLE);

            motor_detect.cnt = 0;
            cstep.cmd_theta = 0;
            motor_detect_mode = SET_TEST_CURRENT;
            break;

        case SET_TEST_CURRENT :
            motor_detect.cnt++;

            if(motor_detect.cnt < 20000) //500msec //Max 5[A]
            {
                if(cstep.isq > (cstep.run_cur_ref / 2)) { cstep.isq = cstep.run_cur_ref / 2; }
                else { cstep.isq += 0.00025; }
            }
            else
            {
                motor_detect_mode = CALCULATE_DCR;
                motor_detect.cnt = 0;
             }
            break;

        case CALCULATE_DCR :
            motor_detect.a_pwm_voltage = ((float32_t)(MID_VOLT_POINT - get_voltage_duty() ))
                                            * (float32_t)VDC / (float32_t)MID_VOLT_POINT;
            motor_detect.vlotage_average += abs(motor_detect.a_pwm_voltage);
            motor_detect.current_average += abs(cur_pid.a_real);

            if(motor_detect.cnt > 10000)
            {
                motor_detect.motor_dcr = motor_detect.vlotage_average / motor_detect.current_average;
                motor_detect_mode = GENERATE_1KHZ;
                motor_detect.cnt = 0;
            }
            break;

        case GENERATE_1KHZ :
            motor_detect.cnt++;

            //theta frequency 1khz
            motor_detect.theta += 0.025;
            motor_detect.theta -= (float32_t)((int32_t) motor_detect.theta);

            cstep.isq =  0.75 * cstep.run_cur_ref
                                +  0.25*cstep.run_cur_ref * __sinpuf32(motor_detect.theta);
            motor_detect.a_pwm_voltage = ((float32_t)(MID_VOLT_POINT - get_voltage_duty() ))
                                            * (float32_t)VDC / (float32_t)MID_VOLT_POINT;

            if(motor_detect.cnt > 200 && motor_detect.cnt < 800)
            {
                if(cur_pid.a_real > motor_detect.im) { motor_detect.im = cur_pid.a_real; }
                if(motor_detect.a_pwm_voltage > motor_detect.vm) { motor_detect.vm = motor_detect.a_pwm_voltage; }

            }

            if(motor_detect.cnt > 800)
            {
                motor_detect.im = motor_detect.im * 0.75 * cstep.run_cur_ref ;
                motor_detect.vm = motor_detect.vm
                                    - (0.75 * cstep.run_cur_ref * motor_detect.motor_dcr);

                motor_detect_mode = CALCULATE_RL;
                motor_detect.cnt = 0;
            }
            break;

        case CALCULATE_RL :
            motor_detect.cnt++;

            motor_detect.z = motor_detect.vm / motor_detect.im;
            motor_detect.motor_ls = (motor_detect.z * motor_detect.z)
                                        - (motor_detect.motor_dcr * motor_detect.motor_dcr);
            motor_detect.motor_ls = sqrt(motor_detect.motor_ls);
            motor_detect.motor_ls = motor_detect.motor_ls
                                        / (2*PI*1000) ; // frequency = 1000[Hz]
            motor_detect.motor_ls = motor_detect.motor_ls * 1000; // unit uH to mH

            control_mode = CONTROL_PHASE_FINDING ;

            break;

         default :
             break;
    }
}

void find_motor_aphase(void)
{
    switch(phase_find_mode)
    {
        case INIT_PHASE_FIND :
            epwm_aq_enable_all();
            driver_status.Bit.servo_state = sevo_control(ENABLE);
            phase_find.cnt = 0;
            cstep.cmd_theta = 0;
            phase_find_mode = SET_RATED_CURRENT;
            BRAKE_UNLOCK;
            break;

        case SET_RATED_CURRENT:
            phase_find.cnt++;

            if(phase_find.cnt < 20000)
            {
                if(cstep.isq > cstep.run_cur_ref) { cstep.isq = cstep.run_cur_ref; }
                else { cstep.isq += 0.00025; }
            }
            else
            {
                phase_find_mode = SET_10P_OVER_CURRENT;
                phase_find.cnt = 0;
            }
            break;

        case SET_10P_OVER_CURRENT:
            phase_find.cnt++;

            if(phase_find.cnt > 20000)
            {
                phase_find.cnt = 0;
                memset(&cmd_pos, 0, sizeof(cmd_pos));
                clear_cw_command(); //clear dsp register
                clear_ccw_command();
                phase_find_mode = SET_CONTROL_MODE;
            }
            else
            {
                if(cstep.isq > cstep.phasefinding_cur_ref) { cstep.isq = cstep.phasefinding_cur_ref; }
                else { cstep.isq  += 0.00025; }
            }
            break;

        case SET_CONTROL_MODE:
            phase_find.cnt++;

            if(phase_find.cnt > 10000)
            {
                phase_find_mode = DONE_PHASE_FIND;
                cstep.actual_cur_ref = cstep.run_cur_ref;
                driver_status.Bit.ready = true;
                if(initial_parameter.selftest == true) { control_mode = CONTROL_SELF_TEST; }
                else{ control_mode = CONTROL_OPEN_LOOP; }
            }
            else
            {
                cstep.isq   -= 0.0025;
                if(cstep.isq < cstep.run_cur_ref) { cstep.isq = cstep.run_cur_ref; }
            }
            break;

        default :
            break;

    }
}

void control_current_pid(void)
{
    // calculate  phase current error
    cur_pid.a_err = cur_pid.a_ref - cur_pid.a_real;
    cur_pid.a_errd = cur_pid.a_err - cur_pid.a_err_old;
    cur_pid.a_err_old = cur_pid.a_err;

    cur_pid.b_err = cur_pid.b_ref - cur_pid.b_real;
    cur_pid.b_errd = cur_pid.b_err - cur_pid.b_err_old;
    cur_pid.b_err_old = cur_pid.b_err;

    // calculate a control p-gain and d-gain
    cur_pid.a_kp_temp = cur_pid.Kp * cur_pid.a_err;
    cur_pid.a_kd_temp = cur_pid.Kd * cur_pid.a_errd;
    cur_pid.a_kd_temp = filter_lpf(&a_dgain_lpf, cur_pid.a_kd_temp);

    cur_pid.b_kp_temp = cur_pid.Kp * cur_pid.b_err;
    cur_pid.b_kd_temp = cur_pid.Kd * cur_pid.b_errd;
    cur_pid.b_kd_temp = filter_lpf(&b_dgain_lpf, cur_pid.b_kd_temp);

    // scale voltage to bit
    if(abs(cstep.cmd_speed) > D_GAIN_TH_SPEED)
    {
        cur_pid.a_pwmout = cur_pid.a_kp_temp + cur_pid.a_kd_temp;
        cur_pid.b_pwmout = cur_pid.b_kp_temp + cur_pid.b_kd_temp;
    }
    else
    {
        cur_pid.a_pwmout = cur_pid.a_kp_temp;
        cur_pid.b_pwmout = cur_pid.b_kp_temp;
    }
    if( cur_pid.a_pwmout > VDC)  { cur_pid.a_pwmout  = VDC; }
    else if( cur_pid.a_pwmout < -VDC) { cur_pid.a_pwmout  = -VDC; }

    if( cur_pid.b_pwmout > VDC)  { cur_pid.b_pwmout  = VDC; }
    else if( cur_pid.b_pwmout < -VDC) { cur_pid.b_pwmout  = -VDC; }

    cur_pid.a_pwmout = cur_pid.a_pwmout * VDC_RECIPROCAL;
    cur_pid.b_pwmout = cur_pid.b_pwmout * VDC_RECIPROCAL;

   // changes in pwmout(delta value) based on command speed
    if(abs(cstep.cmd_speed) < PWMVOUT_LPF_TH1_SPEED)
    {
        cur_pid.a_pwmout = filter_lpf(&a_current_lpf, cur_pid.a_pwmout);
        cur_pid.b_pwmout = filter_lpf(&b_current_lpf, cur_pid.b_pwmout);
    }
    else
    {
        if(abs(cstep.cmd_speed) >= PWMVOUT_LPF_TH1_SPEED && abs(cstep.cmd_speed) >= PWMVOUT_LPF_TH2_SPEED)
        {
            cur_pid.a_lpf_pwmout = filter_lpf(&a_current_lpf, cur_pid.a_pwmout);
            cur_pid.b_lpf_pwmout = filter_lpf(&b_current_lpf, cur_pid.b_pwmout);

            cur_pid.a_pwmout = ((( PWMVOUT_LPF_TH2_SPEED - abs(cstep.cmd_speed)) *cur_pid.a_lpf_pwmout )
                                    + ((abs(cstep.cmd_speed) - PWMVOUT_LPF_TH2_SPEED)* cur_pid.a_pwmout))
                                    / 100.0 ;
            cur_pid.b_pwmout = ((( PWMVOUT_LPF_TH2_SPEED - abs(cstep.cmd_speed)) *cur_pid.b_lpf_pwmout )
                                    + ((abs(cstep.cmd_speed) - PWMVOUT_LPF_TH2_SPEED)* cur_pid.b_pwmout))
                                    / 100.0 ;
        }
        else
        {
            filter_lpf(&a_current_lpf, cur_pid.a_pwmout);
            filter_lpf(&b_current_lpf, cur_pid.b_pwmout);
        }
    }

    cur_pid.apwm = cur_pid.a_pwmout;
    cur_pid.bpwm = cur_pid.b_pwmout;
}

void update_command_position(void)
{
    static uint16_t cmd_update_cnt = 0;
    int32_t updated_cmd_pulse;

    // update command position every 1[ms]
    if(cmd_update_cnt++ >= 39)
    {
        if( initial_parameter.input_mode == one_pulse) // pulse and direction mode
        {
            cmd_pos.curcnt = read_cw_command();
            cmd_pos.deltacnt = cmd_pos.curcnt - cmd_pos.precnt;
            cmd_pos.precnt = cmd_pos.curcnt;
        }
        else // cw and ccw pulse mode
        {
            cw.curcnt = read_cw_command();
            cw.deltacnt = cw.curcnt - cw.precnt;
            cw.precnt = cw.curcnt;

            ccw.curcnt = read_ccw_command();
            ccw.deltacnt = ccw.curcnt - ccw.precnt;
            ccw.precnt = ccw.curcnt;

            cmd_pos.deltacnt = cw.deltacnt - ccw.deltacnt;
        }

        if(initial_parameter.motor_direction == true) { cmd_pos.deltacnt = -cmd_pos.deltacnt; }
        updated_cmd_pulse =  filter_maf(&cmd_maf_1st , cmd_pos.deltacnt);
        cmd_update_cnt = 0;
    }
    else
    { updated_cmd_pulse = 0;  }

    cmd_pos.deltacnt = filter_maf(&cmd_maf_2nd , updated_cmd_pulse);
    cmd_pos.deltacnt = filter_maf(&cmd_maf_3rd , cmd_pos.deltacnt);

    cmd_pos.total += cmd_pos.deltacnt;
    cmd_pos.cal_pos = cmd_pos.total % initial_parameter.resolution;
}

void initialze_variable(void)
{

    memset(&cw, 0, sizeof(cw));
    memset(&ccw, 0, sizeof(ccw));
    memset(&cmd_pos, 0, sizeof(cmd_pos));
    memset(&cur_pid, 0, sizeof(cur_pid));
    memset(&cstep, 0, sizeof(cstep));

    get_system_parameter();
    set_system_parameter();

    control_mode = CONTROL_MOTOR_DETECTION ;
    motor_detect_mode = INIT_MOTOR_DETECT;
    phase_find_mode = INIT_PHASE_FIND;
}


void set_system_parameter(void)
{
    cstep.run_cur_ref   =   initial_parameter.run_current ;
    cstep.stop_cur_ref  =   cstep.run_cur_ref * ((float32_t)initial_parameter.stop_current * 0.01 ) ;
    cstep.CURRENT_LIMIT =   cstep.run_cur_ref * 1.1 ;

    if(cstep.CURRENT_LIMIT > CONTROL_CURRENT_LIMIT)
    { cstep.phasefinding_cur_ref =  cstep.run_cur_ref ; }
    else
    { cstep.phasefinding_cur_ref  = cstep.CURRENT_LIMIT ; } // 10% over current initialize @ phase finding

    cstep.THETA_PULSE_TO_DEG_CONST =  ((float32_t)POLE_NUMBER / 2 )
                                            / (float32_t)initial_parameter.resolution ;
    cstep.CONTROL_RPM_CONST = (SEC_MIN_CONST * SPEED_CONTROL_FREQ)
                                            / (float32_t)initial_parameter.resolution ;
    cstep.MONITORING_RPM_CONSTANT = SEC_MIN_CONST * SPEED_CONTROL_FREQ
                                            / ((float32_t)initial_parameter.resolution);

    cur_pid.Kp = initial_parameter.p_gain;
    cur_pid.Kd = initial_parameter.d_gain;

    MOTOR_OVERLOAD_LIMIT = pow(initial_parameter.run_current * 3.0 , 2.0);

    // input command filter
    maf_initialize(&cmd_maf_1st, MAF_1ST_ORDER);
    maf_initialize(&cmd_maf_2nd, MAF_2ND_ORDER);
    maf_initialize(&cmd_maf_3rd, MAF_3RD_ORDER);

    //  command and actual velocity filter
    maf_initialize(&cmd_velocity_maf, (uint16_t)SEC_MIN_CONST - 1);
    maf_initialize(&actual_velocity_maf, (uint16_t)SEC_MIN_CONST - 1);

    // low pass filter for current pid loop
    lpf_initialize(CUTOFF_FREQ_GAIN, &a_dgain_lpf, CURRENT_CONTROL_SAMPLE_TIME);
    lpf_initialize(CUTOFF_FREQ_GAIN, &b_dgain_lpf, CURRENT_CONTROL_SAMPLE_TIME);
    lpf_initialize(CUTOFF_FREQ_CURRENT, &a_current_lpf, CURRENT_CONTROL_SAMPLE_TIME);
    lpf_initialize(CUTOFF_FREQ_CURRENT, &b_current_lpf, CURRENT_CONTROL_SAMPLE_TIME);
}

float32_t rpm_to_electric_theta(float32_t rpm)
{
    return RPM_TO_DELTA_ELECTRIC_THETA * rpm ;
}

void calculate_monitoring_variable(void)
{
    static uint16_t monitoring_cnt = 0;

    // command pulse count for control loop
    if( initial_parameter.input_mode == one_pulse) // pulse and direction mode
    {
        cmd_pos.v_curcnt = read_cw_command();
        cmd_pos.v_deltacnt = cmd_pos.v_curcnt - cmd_pos.v_precnt;
        cmd_pos.v_precnt = cmd_pos.v_curcnt;
    }
    else // cw and ccw pulse mode
    {
        cw.v_curcnt = read_cw_command();
        cw.v_deltacnt = cw.v_curcnt - cw.v_precnt;
        cw.v_precnt = cw.v_curcnt;

        ccw.v_curcnt = read_ccw_command();
        ccw.v_deltacnt = ccw.v_curcnt - ccw.v_precnt;
        ccw.v_precnt = ccw.v_curcnt;

        cmd_pos.v_deltacnt = cw.v_deltacnt - ccw.v_deltacnt;
    }
    if(initial_parameter.motor_direction == true) { cmd_pos.v_deltacnt = -cmd_pos.v_deltacnt; }
    cmd_pos.v_total += cmd_pos.v_deltacnt;
    cstep.cmd_speed = cstep.MONITORING_RPM_CONSTANT * cmd_pos.v_deltacnt ;


    // command pulse count for monitoring
    if(++monitoring_cnt >= MONITORING_RPM_CNT)
    {
        if( initial_parameter.input_mode == one_pulse) // pulse and direction mode
        {
            cmd_pos.m_curcnt = read_cw_command();
            cmd_pos.m_deltacnt = cmd_pos.m_curcnt - cmd_pos.m_precnt;
            cmd_pos.m_precnt = cmd_pos.m_curcnt;
        }
        else // cw and ccw pulse mode
        {
            cw.m_curcnt = read_cw_command();
            cw.m_deltacnt = cw.m_curcnt - cw.m_precnt;
            cw.m_precnt = cw.m_curcnt;

            ccw.m_curcnt = read_ccw_command();
            ccw.m_deltacnt = ccw.m_curcnt - ccw.m_precnt;
            ccw.m_precnt = ccw.m_curcnt;

            cmd_pos.m_deltacnt = cw.m_deltacnt - ccw.m_deltacnt;
        }
        if(initial_parameter.motor_direction == true) { cmd_pos.m_deltacnt = -cmd_pos.m_deltacnt; }
        cstep.m_cmd_speed = cstep.MONITORING_RPM_CONSTANT * (cmd_pos.v_deltacnt / MONITORING_RPM_CNT );

        monitoring_cnt = 0;
    }

    // monitor current
    cur_pid.ab_real = sqrt( (cur_pid.a_real * cur_pid.a_real) + (cur_pid.b_real * cur_pid.b_real) );
}



void control_stop_current(void)
{
    static uint16_t stop_cnt = 0;

    if(cmd_pos.v_deltacnt != 0)
    {
        driver_status.Bit.motor_state = MOTOR_RUN;
        cstep.actual_cur_ref = cstep.run_cur_ref;
    }

    if(cmd_pos.v_deltacnt == 0) { stop_cnt++; }
    else { stop_cnt = 0; }

    if(stop_cnt >= MOTOR_STOP_TIME)
    {
        driver_status.Bit.motor_state = MOTOR_STOP;
        stop_cnt = MOTOR_STOP_TIME;

        if(cstep.actual_cur_ref > cstep.stop_cur_ref) { cstep.actual_cur_ref -= 0.01; }
    }
}



void control_servo(void)
{
    static uint16_t servo_on_timer = 0;
    static uint16_t servo_off_timer = 0;

    // over current and fail to find motor phase alarm condition
    if(err_status.Full & RESETABLE_ALARM){ BRAKE_LOCK; }
    else //normal alarm condition
    {
        if(driver_status.Bit.servo_state == ENABLE  )
        {
            if(driver_status.Bit.io_enable_state == ENABLE || err_status.Full != 0 )
            {
                BRAKE_LOCK;
                servo_off_timer++;
                if(servo_off_timer >= BRAKE_LOCK_TIME)
                {
                    driver_status.Bit.servo_state = sevo_control(DISABLE);
                    servo_off_timer = BRAKE_LOCK_TIME;
                }
            }
        }
        else if(driver_status.Bit.servo_state == DISABLE)
        {
            servo_off_timer = 0 ;
            if(driver_status.Bit.io_enable_state == DISABLE && err_status.Full == 0 )
            {
                servo_on_timer++;
                BRAKE_UNLOCK;
                if(servo_on_timer >= BRAKE_UNLOCK_TIME)
                {
                    driver_status.Bit.servo_state = sevo_control(ENABLE);
                    servo_on_timer = BRAKE_UNLOCK_TIME;
                }
            }
        }
    }
}

















