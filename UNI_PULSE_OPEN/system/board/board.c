/*
 * board.c
 *
 *  Created on: 2023. 10. 18.
 *      Author: JH
 */


#include "common.h"
#include "pulse_device.h"
#include "board.h"

/********************************************************************/
// epwm
/********************************************************************/
void epwm_aq_enable_all(void)
{

    EPWM_setActionQualifierAction(A_PWM_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(A_PWM_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(A_PWM_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(A_PWM_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    EPWM_setActionQualifierAction(B_PWM_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(B_PWM_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(B_PWM_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(B_PWM_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
}

void epwm_aq_enable_aphase(void)
{
    EPWM_setActionQualifierAction(A_PWM_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(A_PWM_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(A_PWM_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(A_PWM_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
}

void set_voltage_duty(uint16_t a_comp_count, uint16_t b_comp_count)
{
    EPWM_setCounterCompareValue(A_PWM_BASE, EPWM_COUNTER_COMPARE_A, a_comp_count);
    EPWM_setCounterCompareValue(B_PWM_BASE, EPWM_COUNTER_COMPARE_A, b_comp_count);
}

uint16_t get_voltage_duty(void)
{
    return   EPWM_getCounterCompareValue(A_PWM_BASE, EPWM_COUNTER_COMPARE_A);
}

/********************************************************************/
// eqep
/********************************************************************/
int32_t read_cw_command(void)
{
    return EQEP_getPosition(ONE_PULSE_EQEP_BASE);
}

int32_t read_ccw_command(void)
{
    return EQEP_getPosition(TWO_PULSE_EQEP_BASE);
}

void clear_cw_command(void)
{
    EQEP_setPosition(ONE_PULSE_EQEP_BASE, 0);
}

void clear_ccw_command(void)
{
    EQEP_setPosition(TWO_PULSE_EQEP_BASE, 0);
}


/********************************************************************/
// adc
/********************************************************************/
int16_t initialize_adc_reference(void)
{
    static uint32_t adc_init_cnt = 0;
    static uint32_t current_a_offset = 0;
    static uint32_t current_b_offset = 0;

    int16_t     temp;
    float32_t   adc_offset;

    if(adc_init_cnt < ADC_REF_CAL_CNT)
    {
        current_a_offset += ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
        current_b_offset += ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0);

        adc_init_cnt++;
        temp = false;
    }
    else if(adc_init_cnt == ADC_REF_CAL_CNT)
    {
        current_a_offset /= ADC_REF_CAL_CNT;
        current_b_offset /= ADC_REF_CAL_CNT;

        ADC_setPPBReferenceOffset(CURRENT_A_ADC, ADC_PPB_NUMBER1, current_a_offset);
        ADC_setPPBReferenceOffset(CURRENT_B_ADC, ADC_PPB_NUMBER1, current_b_offset);

        adc_init_cnt++;
        temp = false;
    }
    else
    {
        adc_offset = ALARM_CURRENT_LIMIT / CURRENT_PER_BITS;

        ADC_setupPPB(CURRENT_A_ADC, ADC_PPB_NUMBER1, ADC_SOC_NUMBER0);
        ADC_setupPPB(CURRENT_B_ADC, ADC_PPB_NUMBER1, ADC_SOC_NUMBER0);

        ADC_disablePPBTwosComplement(CURRENT_A_ADC, ADC_PPB_NUMBER1);
        ADC_disablePPBTwosComplement(CURRENT_B_ADC, ADC_PPB_NUMBER1);

        ADC_setPPBTripLimits(CURRENT_A_ADC, ADC_PPB_NUMBER1, (uint16_t)adc_offset, (uint16_t)(-(int16_t)adc_offset));
        ADC_setPPBTripLimits(CURRENT_B_ADC, ADC_PPB_NUMBER1, (uint16_t)adc_offset, (uint16_t)(-(int16_t)adc_offset));

        ADC_disablePPBEvent(CURRENT_A_ADC, ADC_PPB_NUMBER1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
        ADC_disablePPBEvent(CURRENT_B_ADC, ADC_PPB_NUMBER1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));


        ADC_enablePPBEventInterrupt(CURRENT_A_ADC, ADC_PPB_NUMBER1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO));
        ADC_enablePPBEventInterrupt(CURRENT_B_ADC, ADC_PPB_NUMBER1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO));

        ADC_disablePPBEventInterrupt(CURRENT_A_ADC, ADC_PPB_NUMBER1, (ADC_EVT_ZERO));
        ADC_disablePPBEventInterrupt(CURRENT_B_ADC, ADC_PPB_NUMBER1, (ADC_EVT_ZERO));


        ADC_enablePPBEvent(CURRENT_A_ADC, ADC_PPB_NUMBER1,  (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO));
        ADC_enablePPBEvent(CURRENT_B_ADC, ADC_PPB_NUMBER1,  (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO));

        ADC_clearPPBEventStatus(CURRENT_A_ADC, ADC_PPB_NUMBER1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
        ADC_clearPPBEventStatus(CURRENT_A_ADC, ADC_PPB_NUMBER1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));

        temp = true;
    }

    return temp;
}

void read_current(float32_t *a_phase, float32_t *b_phase)
{
    int16_t result_a, result_b;

    result_a = ADC_readPPBResult(CURRENT_A_ADCARESULT, ADC_PPB_NUMBER1);
    result_b = ADC_readPPBResult(CURRENT_B_ADCARESULT, ADC_PPB_NUMBER1);

    *a_phase = (float32_t)result_a * (float32_t)CURRENT_PER_BITS ;
    *b_phase = (float32_t)result_b * (float32_t)CURRENT_PER_BITS ;
}

void convert_adc_system_variable(void)
{
    ADC_forceMultipleSOC(CURRENT_A_ADC, (ADC_FORCE_SOC1 | ADC_FORCE_SOC2 | ADC_FORCE_SOC3 ));
}
void update_adc_system_variable(float32_t *dc_link, float32_t *device_temp, float32_t *dsp_temp)
{
    //DC LINK
    if(ADC_getInterruptStatus(CURRENT_A_ADC, ADC_INT_NUMBER2) == true)
    {
        ADC_clearInterruptStatus(CURRENT_A_ADC, ADC_INT_NUMBER2);
        *dc_link = (float32_t)ADC_readResult(CURRENT_A_ADCARESULT, ADC_SOC_NUMBER1) * DC_LINK_CONST;
    }

    //Device TEMP
    if(ADC_getInterruptStatus(CURRENT_A_ADC, ADC_INT_NUMBER3) == true)
    {
        ADC_clearInterruptStatus(CURRENT_A_ADC, ADC_INT_NUMBER3);
        *device_temp = (float32_t)ADC_readResult(CURRENT_A_ADCARESULT, ADC_SOC_NUMBER2) * (float32_t)DEVICE_TEMP_CONST - 50.0;

    }

    //DSP TEMP
    if(ADC_getInterruptStatus(CURRENT_A_ADC, ADC_INT_NUMBER4) == true)
    {
        //ADC_clearInterruptStatus(CURRENT_A_ADC, ADC_INT_NUMBER4);
        //*dsp_temp = ((float32_t)ADC_readResult(CURRENT_A_ADCARESULT, ADC_SOC_NUMBER3) * 3.3/2.5 - ADC_EXT_REF_TSOFFSET ) * 4096.0 / ADC_EXT_REF_TSSLOPE ;
        *dsp_temp = 25;
    }
}


/********************************************************************/
// gpios
/********************************************************************/
uint16_t sevo_control(uint16_t cmd)
{
    if(cmd == true) { PWM_ENABLE; }
    else { PWM_DIABLE; }

    return cmd;
}



uint16_t get_input_io_status(void)
{
    uint16_t temp;

    temp =  (uint16_t)GPIO_readPin(DRIVER_ENABLE_GPIO) ;
    return temp = ~temp & 0x1 ;
}




/********************************************************************/
// SCI
/********************************************************************/
uint16_t check_rx_buf(void)
{
    return SCI_getRxFIFOStatus(RS232_SCI_BASE);
}

void read_rx_data(uint16_t *rx_data)
{
    *rx_data = SCI_readCharNonBlocking(RS232_SCI_BASE);
}

void write_tx_data(uint16_t tx_data)
{
    SCI_writeCharBlockingFIFO(RS232_SCI_BASE, tx_data);
}











