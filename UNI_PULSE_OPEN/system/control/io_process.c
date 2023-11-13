/*
 * io_process.c
 *
 *  Created on: 2023. 10. 31.
 *      Author: JH
 */

#include "board.h"
#include "control.h"
#include "alarm.h"
#include "io_process.h"


uint16_t    io_enable_cnt;
uint16_t    run_led_timer;
INPUT_STATUS input_status;
OUTPUT_STATUS output_status;




void display_system_status(void);



void io_process(void)
{
    input_status.Full = get_input_io_status();
    output_status.Full = get_output_io_status();
    if(input_status.Bit.enable == true) { io_enable_cnt++; }
    else
    {
        io_enable_cnt = 0;
        driver_status.Bit.io_enable_state = DISABLE;
    }

    if(io_enable_cnt >= IO_ENABLE_CHANGE_TIME)
    {
        driver_status.Bit.io_enable_state = ENABLE;
        reset_alarm();
        io_enable_cnt = IO_ENABLE_CHANGE_TIME;
    }

    display_system_status();
}

void display_system_status(void)
{
    // diplay run led
    if(driver_status.Bit.motor_state == MOTOR_RUN)
    {   run_led_timer++;
        if(run_led_timer >= LED_TOGGLE_TIME)
        {
            RUN_LED_TOGGLE;
            run_led_timer = 0;
        }
    }
    else
    {
        RUN_LED_OFF;
        run_led_timer = 0;
    }

    //display error led , alarm output
    if(err_status.Full != 0)
    {
        ERR_LED_ON;
        ALARM_ON;
    }
    else
    {
        ERR_LED_OFF;
        ALARM_OFF;
    }
}




















