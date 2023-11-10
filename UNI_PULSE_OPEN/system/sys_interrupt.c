/*
 * sys_interrupt.c
 *
 *  Created on: 2023. 10. 20.
 *      Author: JH
 */

#include "pulse_device.h"
#include "board.h"
#include "alarm.h"
#include "sys_interrupt.h"
#include "control.h"


__interrupt void control_system_loop(void)
{
    EINT;

    convert_adc_system_variable();

    update_adc_system_variable(&system_condition.dc_link, &system_condition.device_temp, &system_condition.dsp_temp);

    check_system_condtion();

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);

}


__interrupt void control_current_loop(void)
{
    // current control
    control_pwm();

    // Clear the interrupt flag
    ADC_clearInterruptStatus(CURRENT_A_ADC, ADC_INT_NUMBER1);

    // Check if overflow has occurred
    if(true == ADC_getInterruptOverflowStatus(CURRENT_A_ADC, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(CURRENT_A_ADC, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(CURRENT_A_ADC, ADC_INT_NUMBER1);
    }

    // Acknowledge the interrupt
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}


__interrupt void calibration_adc_ref_loop(void)
{

    if( initialize_adc_reference() == true)
    {
        // Starts CPU-Timer 0
        Interrupt_register(INT_ADCA1, &control_current_loop);
        CPUTimer_startTimer(CPUTIMER0_BASE);
    }

    // Clear the interrupt flag
    ADC_clearInterruptStatus(CURRENT_A_ADC, ADC_INT_NUMBER1);

    // Check if overflow has occurred
    if(true == ADC_getInterruptOverflowStatus(CURRENT_A_ADC, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(CURRENT_A_ADC, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(CURRENT_A_ADC, ADC_INT_NUMBER1);
    }

    // Acknowledge the interrupt
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}



__interrupt void check_over_current(void)
{
    uint32_t intStatus;

    driver_status.Bit.servo_state = sevo_control(DISABLE);

    intStatus = ADC_getPPBEventStatus(CURRENT_A_ADC, ADC_PPB_NUMBER1);

    if((intStatus & ADC_EVT_TRIPHI) != 0U)
    {
        ADC_clearPPBEventStatus(CURRENT_A_ADC, ADC_PPB_NUMBER1, ADC_EVT_TRIPHI);
        err_status.Bit.over_current_A = true;
    }
    if((intStatus & ADC_EVT_TRIPLO) != 0U)
    {
        ADC_clearPPBEventStatus(CURRENT_A_ADC, ADC_PPB_NUMBER1, ADC_EVT_TRIPLO);
        err_status.Bit.over_current_Abar = true;
    }

    intStatus = 0;

    intStatus = ADC_getPPBEventStatus(CURRENT_B_ADC, ADC_PPB_NUMBER1);

    if((intStatus & ADC_EVT_TRIPHI) != 0U)
    {
        ADC_clearPPBEventStatus(CURRENT_B_ADC, ADC_PPB_NUMBER1, ADC_EVT_TRIPHI);
        err_status.Bit.over_current_B = true;
    }
    if((intStatus & ADC_EVT_TRIPLO) != 0U)
    {
        ADC_clearPPBEventStatus(CURRENT_B_ADC, ADC_PPB_NUMBER1, ADC_EVT_TRIPLO);
        err_status.Bit.over_current_Bbar = true;
    }

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);

}































