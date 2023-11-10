/*
 * system.c
 *
 *  Created on: 2023. 10. 11.
 *      Author: JH
 */

#include "pulse_device.h"
#include "sys_interrupt.h"
#include "parameter.h"
#include "board.h"
#include "system.h"

void configure_gpios(void);
void confiure_peripherals(void);
void init_epwm(void);
void init_eqep(void);
void init_sci(void);
void init_adc(void);
void get_system_configration_info(SYSTEM_CONFIGURATION_VARIABLE *sys_info);

void init_cpu_timers(void);
void configure_cpu_timer(uint32_t cpuTimer, float freq, float period);


void initialize_system(void)
{
    // Initialize device clock and peripherals
    Device_init();

    // Disable pin locks and enable internal pullups.
    Device_initGPIO();

    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    Interrupt_initModule();

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    Interrupt_initVectorTable();


    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    Interrupt_register(INT_TIMER0, &control_system_loop);
    Interrupt_register(INT_ADCA1, &calibration_adc_ref_loop);
    Interrupt_register(INT_ADCA_EVT, &check_over_current);
    Interrupt_register(INT_ADCC_EVT, &check_over_current);


    // GPIOs pin configuration
    configure_gpios();

    // get information to initialize system configuration
    get_system_configration_info(&system_configration_variable);

    // Initialize peripherals
    confiure_peripherals();

    // Initialize cpu0 timer
    init_cpu_timers();

    // Enable Interrupt
    Interrupt_enable(INT_TIMER0);
    Interrupt_enable(INT_ADCA1);
    Interrupt_enable(INT_ADCA_EVT);
    Interrupt_enable(INT_ADCC_EVT);

    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    EINT;
    ERTM;

}

void  get_system_configration_info(SYSTEM_CONFIGURATION_VARIABLE *sys_info)
{
    sys_info->bit.run_current0 = GPIO_readPin(RUN_CURRNET0_GPIO) ;
    sys_info->bit.run_current1 = GPIO_readPin(RUN_CURRNET1_GPIO) ;
    sys_info->bit.run_current2 = GPIO_readPin(RUN_CURRNET2_GPIO) ;
    sys_info->bit.run_current3 = GPIO_readPin(RUN_CURRNET3_GPIO) ;

    sys_info->bit.stop_current0 = GPIO_readPin(STOP_CURRNET0_GPIO) ;
    sys_info->bit.stop_current1 = GPIO_readPin(STOP_CURRNET1_GPIO) ;

    sys_info->bit.gain0 = GPIO_readPin(GAIN0_GPIO) ;
    sys_info->bit.gain1 = GPIO_readPin(GAIN1_GPIO) ;

    sys_info->bit.resoulgion0 = GPIO_readPin(RESOLUTION0_GPIO) ;
    sys_info->bit.resoulgion1 = GPIO_readPin(RESOLUTION1_GPIO) ;
    sys_info->bit.resoulgion2 = GPIO_readPin(RESOLUTION2_GPIO) ;
    sys_info->bit.resoulgion3 = GPIO_readPin(RESOLUTION3_GPIO) ;

    sys_info->bit.selftest = GPIO_readPin(SELP_TEST_GPIO) ;
    sys_info->bit.gain_sel = GPIO_readPin(GAIN_SEL_GPIO) ;
    sys_info->bit.input_mode = GPIO_readPin(INPUT_MODE_GPIO) ;
    sys_info->bit.motor_direction = GPIO_readPin(MOTOR_DIRECTION_GPIO) ;

    sys_info->full = ~sys_info->full;
}


void configure_gpios(void)
{

    /********************************************************************/
    // Configure the GPIOs for ePWM
    /********************************************************************/
    //Current A
    GPIO_setPinConfig(A_PWM_PIN_CFG);
    GPIO_setPadConfig(A_PWM_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(A_PWM_GPIO, GPIO_QUAL_SYNC);
    //Current /A
    GPIO_setPinConfig(Abar_PWM_PIN_CFG);
    GPIO_setPadConfig(Abar_PWM_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(Abar_PWM_GPIO, GPIO_QUAL_SYNC);
    //Current B
    GPIO_setPinConfig(B_PWM_PIN_CFG);
    GPIO_setPadConfig(B_PWM_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(B_PWM_GPIO, GPIO_QUAL_SYNC);
    //Current /B
    GPIO_setPinConfig(Bbar_PWM_PIN_CFG);
    GPIO_setPadConfig(Bbar_PWM_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(Bbar_PWM_GPIO, GPIO_QUAL_SYNC);




    /********************************************************************/
    // Configure the GPIOs for RS232
    /********************************************************************/
    // DEVICE_GPIO_PIN_SCIRXDA is the SCI Rx pin.
    GPIO_setPinConfig(RS232_RXD_PIN_CFG);
    GPIO_setDirectionMode(RS232_RXD_GPIO, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(RS232_RXD_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(RS232_RXD_GPIO, GPIO_QUAL_ASYNC);

    // DEVICE_GPIO_PIN_SCITXDA is the SCI Tx pin.
    GPIO_setPinConfig(RS232_TXD_PIN_CFG);
    GPIO_setDirectionMode(RS232_TXD_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(RS232_TXD_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(RS232_TXD_GPIO, GPIO_QUAL_ASYNC);



    /********************************************************************/
    // Configure the OUTPUT GPIOs
    /********************************************************************/

    GPIO_setPinConfig(PWM_ENABLE_PIN_CFG);
    GPIO_setPadConfig(PWM_ENABLE_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(PWM_ENABLE_GPIO, GPIO_DIR_MODE_OUT);
    PWM_DIABLE;

    GPIO_setPinConfig(RUN_LED_GPIO_PIN_CFG);
    GPIO_setPadConfig(RUN_LED_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(RUN_LED_GPIO, GPIO_DIR_MODE_OUT);
    RUN_LED_OFF;

    GPIO_setPinConfig(ERR_LED_GPIO_PIN_CFG);
    GPIO_setPadConfig(ERR_LED_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(ERR_LED_GPIO, GPIO_DIR_MODE_OUT);
    ERR_LED_OFF;

    GPIO_setPinConfig(BRAKE_GPIO_PIN_CFG);
    GPIO_setPadConfig(BRAKE_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(BRAKE_GPIO, GPIO_DIR_MODE_OUT);
    BRAKE_LOCK;

    GPIO_setPinConfig(ALARM_GPIO_PIN_CFG);
    GPIO_setPadConfig(ALARM_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(ALARM_GPIO, GPIO_DIR_MODE_OUT);
    ALARM_OFF;


    /********************************************************************/
    // Configure the INPUT GPIOs
    /********************************************************************/
    GPIO_setPadConfig(DRIVER_ENABLE_GPIO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(DRIVER_ENABLE_PIN_CFG);
    GPIO_setDirectionMode(DRIVER_ENABLE_GPIO, GPIO_DIR_MODE_IN);

    GPIO_setPadConfig(RUN_CURRNET0_GPIO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(RUN_CURRNET0_PIN_CFG);
    GPIO_setDirectionMode(RUN_CURRNET0_GPIO, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(RUN_CURRNET1_GPIO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(RUN_CURRNET1_PIN_CFG);
    GPIO_setDirectionMode(RUN_CURRNET1_GPIO, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(RUN_CURRNET2_GPIO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(RUN_CURRNET2_PIN_CFG);
    GPIO_setDirectionMode(RUN_CURRNET2_GPIO, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(RUN_CURRNET3_GPIO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(RUN_CURRNET3_PIN_CFG);
    GPIO_setDirectionMode(RUN_CURRNET3_GPIO, GPIO_DIR_MODE_IN);

    GPIO_setPadConfig(STOP_CURRNET0_GPIO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(STOP_CURRNET0_PIN_CFG);
    GPIO_setDirectionMode(STOP_CURRNET0_GPIO, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(STOP_CURRNET1_GPIO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(STOP_CURRNET1_PIN_CFG);
    GPIO_setDirectionMode(STOP_CURRNET1_GPIO, GPIO_DIR_MODE_IN);

    GPIO_setPadConfig(GAIN0_GPIO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GAIN0_PIN_CFG);
    GPIO_setDirectionMode(GAIN0_GPIO, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(GAIN1_GPIO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GAIN1_PIN_CFG);
    GPIO_setDirectionMode(GAIN1_GPIO, GPIO_DIR_MODE_IN);

    GPIO_setPadConfig(RESOLUTION0_GPIO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(RESOLUTION0_PIN_CFG);
    GPIO_setDirectionMode(RESOLUTION0_GPIO, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(RESOLUTION1_GPIO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(RESOLUTION1_PIN_CFG);
    GPIO_setDirectionMode(RESOLUTION1_GPIO, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(RESOLUTION2_GPIO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(RESOLUTION2_PIN_CFG);
    GPIO_setDirectionMode(RESOLUTION2_GPIO, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(RESOLUTION3_GPIO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(RESOLUTION3_PIN_CFG);
    GPIO_setDirectionMode(RESOLUTION3_GPIO, GPIO_DIR_MODE_IN);

    GPIO_setPadConfig(SELP_TEST_GPIO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(SELP_TEST_PIN_CFG);
    GPIO_setDirectionMode(SELP_TEST_GPIO, GPIO_DIR_MODE_IN);

    GPIO_setPadConfig(GAIN_SEL_GPIO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GAIN_SEL_PIN_CFG);
    GPIO_setDirectionMode(GAIN_SEL_GPIO, GPIO_DIR_MODE_IN);

    GPIO_setPadConfig(INPUT_MODE_GPIO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(INPUT_MODE_PIN_CFG);
    GPIO_setDirectionMode(INPUT_MODE_GPIO, GPIO_DIR_MODE_IN);

    GPIO_setPadConfig(MOTOR_DIRECTION_GPIO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(MOTOR_DIRECTION_PIN_CFG);
    GPIO_setDirectionMode(MOTOR_DIRECTION_GPIO, GPIO_DIR_MODE_IN);


    // Qualification Sampling Period input Filter
    // Do not configure GPIO16 to GPIO23 and GPIO8 to GPIO15 because conflict pulse input eQEPs.
    GPIO_setQualificationPeriod(DRIVER_ENABLE_GPIO, QUALPRD_DIVIDE);

//    GPIO_setQualificationPeriod(RUN_CURRNET0_GPIO, QUALPRD_DIVIDE);
    GPIO_setQualificationPeriod(RUN_CURRNET1_GPIO, QUALPRD_DIVIDE);
//    GPIO_setQualificationPeriod(RUN_CURRNET2_GPIO, QUALPRD_DIVIDE);
//    GPIO_setQualificationPeriod(RUN_CURRNET3_GPIO, QUALPRD_DIVIDE);

    GPIO_setQualificationPeriod(STOP_CURRNET0_GPIO, QUALPRD_DIVIDE);
    GPIO_setQualificationPeriod(STOP_CURRNET1_GPIO, QUALPRD_DIVIDE);

    GPIO_setQualificationPeriod(GAIN0_GPIO, QUALPRD_DIVIDE);
    GPIO_setQualificationPeriod(GAIN1_GPIO, QUALPRD_DIVIDE);

//    GPIO_setQualificationPeriod(RESOLUTION0_GPIO, QUALPRD_DIVIDE);
    GPIO_setQualificationPeriod(RESOLUTION1_GPIO, QUALPRD_DIVIDE);
    GPIO_setQualificationPeriod(RESOLUTION2_GPIO, QUALPRD_DIVIDE);
    GPIO_setQualificationPeriod(RESOLUTION3_GPIO, QUALPRD_DIVIDE);

    GPIO_setQualificationPeriod(SELP_TEST_GPIO, QUALPRD_DIVIDE);
    GPIO_setQualificationPeriod(GAIN_SEL_GPIO, QUALPRD_DIVIDE);
//    GPIO_setQualificationPeriod(INPUT_MODE_GPIO, QUALPRD_DIVIDE);
//    GPIO_setQualificationPeriod(MOTOR_DIRECTION_GPIO, QUALPRD_DIVIDE);


    // Input qualification type
    GPIO_setQualificationMode(DRIVER_ENABLE_GPIO, GPIO_QUAL_6SAMPLE);

    GPIO_setQualificationMode(RUN_CURRNET0_GPIO, GPIO_QUAL_6SAMPLE);
    GPIO_setQualificationMode(RUN_CURRNET1_GPIO, GPIO_QUAL_6SAMPLE);
    GPIO_setQualificationMode(RUN_CURRNET2_GPIO, GPIO_QUAL_6SAMPLE);
    GPIO_setQualificationMode(RUN_CURRNET3_GPIO, GPIO_QUAL_6SAMPLE);

    GPIO_setQualificationMode(STOP_CURRNET0_GPIO, GPIO_QUAL_6SAMPLE);
    GPIO_setQualificationMode(STOP_CURRNET1_GPIO, GPIO_QUAL_6SAMPLE);

    GPIO_setQualificationMode(GAIN0_GPIO, GPIO_QUAL_6SAMPLE);
    GPIO_setQualificationMode(GAIN1_GPIO, GPIO_QUAL_6SAMPLE);

    GPIO_setQualificationMode(RESOLUTION0_GPIO, GPIO_QUAL_6SAMPLE);
    GPIO_setQualificationMode(RESOLUTION1_GPIO, GPIO_QUAL_6SAMPLE);
    GPIO_setQualificationMode(RESOLUTION2_GPIO, GPIO_QUAL_6SAMPLE);
    GPIO_setQualificationMode(RESOLUTION3_GPIO, GPIO_QUAL_6SAMPLE);

    GPIO_setQualificationMode(SELP_TEST_GPIO, GPIO_QUAL_6SAMPLE);
    GPIO_setQualificationMode(GAIN_SEL_GPIO, GPIO_QUAL_6SAMPLE);
    GPIO_setQualificationMode(INPUT_MODE_GPIO, GPIO_QUAL_6SAMPLE);
    GPIO_setQualificationMode(MOTOR_DIRECTION_GPIO, GPIO_QUAL_6SAMPLE);


}


void confiure_peripherals(void)
{
    init_epwm();
    init_eqep();
    init_sci();
    init_adc();
}





void init_epwm(void)
{

    // Disable the ePWM time base clock before configuring the module
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);


    /********************************************************************/
    // Current A phase ePWM configure
    /********************************************************************/

    // Set phase shift to 0 and clear the time base counter
    EPWM_setPhaseShift(A_PWM_BASE, 0);
    EPWM_setTimeBaseCounter(A_PWM_BASE, 0);


    EPWM_setClockPrescaler(A_PWM_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(A_PWM_BASE, EPWM_TBPRD);
    EPWM_setTimeBaseCounterMode(A_PWM_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(A_PWM_BASE);
    EPWM_setCounterCompareShadowLoadMode(A_PWM_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareShadowLoadMode(A_PWM_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);

    EPWM_setSyncInPulseSource(A_PWM_BASE, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1);
    EPWM_enableSyncOutPulseSource(A_PWM_BASE, EPWM_SYNCOUTEN_ZEROEN);

    EPWM_setActionQualifierAction(A_PWM_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(A_PWM_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(A_PWM_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(A_PWM_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    EPWM_setCounterCompareValue(A_PWM_BASE, EPWM_COUNTER_COMPARE_C, 0);

    EPWM_enableADCTrigger(A_PWM_BASE, EPWM_SOC_A);
    EPWM_setADCTriggerSource(A_PWM_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_U_CMPC);

    // ePWM ADC Start-of-Conversion A Event (EPWMxSOCA) Period Select
    // Generate the EPWMxSOCA pulse on the first event
    HWREGH(A_PWM_BASE + EPWM_O_ETPS) |= (1 << EPWM_ETPS_SOCAPRD_S);


    /********************************************************************/
    // Current B phase ePWM configure
    /********************************************************************/

    // Set phase shift to 0 and clear the time base counter
    EPWM_setPhaseShift(B_PWM_BASE, 0);
    EPWM_setTimeBaseCounter(B_PWM_BASE, 0);

    EPWM_setClockPrescaler(B_PWM_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(B_PWM_BASE, EPWM_TBPRD);
    EPWM_setTimeBaseCounterMode(B_PWM_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_enablePhaseShiftLoad(B_PWM_BASE);
    EPWM_setCounterCompareShadowLoadMode(B_PWM_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareShadowLoadMode(B_PWM_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);

    EPWM_setSyncInPulseSource(B_PWM_BASE, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1);
    EPWM_setCounterCompareValue(B_PWM_BASE, EPWM_COUNTER_COMPARE_C, EPWM_TBPRD / 2);

    EPWM_setActionQualifierAction(A_PWM_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(A_PWM_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(A_PWM_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(A_PWM_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    // Sync the ePWM time base clock
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

}


void init_eqep(void)
{

    switch(system_configration_variable.data.input_mode)
    {
        case one_pulse :   // pulse, direction mode

            // Configure the GPIOs for eQEP 1pulse cw counter / direction
            GPIO_setPinConfig(CW_COUNT_PIN_CFG);
            GPIO_setPadConfig(CW_COUNT_GPIO, GPIO_PIN_TYPE_STD);
            GPIO_setQualificationMode(CW_COUNT_GPIO, GPIO_QUAL_SYNC);

            GPIO_setPinConfig(DIRECTION_PIN_CFG);
            GPIO_setPadConfig(DIRECTION_GPIO, GPIO_PIN_TYPE_STD);
            GPIO_setQualificationMode(DIRECTION_GPIO, GPIO_QUAL_SYNC);


            // Sets the polarity of the eQEP module's input signals.
            EQEP_setInputPolarity(ONE_PULSE_EQEP_BASE,false,false,false,false); // invertQEPA, invertQEPB, invertIndex, invertStrobe

            // Configures eQEP module's clock and direction mode.
            EQEP_setDecoderConfig(ONE_PULSE_EQEP_BASE, (EQEP_CONFIG_CLOCK_DIR | EQEP_CONFIG_1X_RESOLUTION | EQEP_CONFIG_NO_SWAP | EQEP_CONFIG_IGATE_DISABLE));

            // Set the emulation mode of the eQEP module.
            EQEP_setEmulationMode(ONE_PULSE_EQEP_BASE,EQEP_EMULATIONMODE_RUNFREE);

            // Configures eQEP module position counter unit.
            EQEP_setPositionCounterConfig(ONE_PULSE_EQEP_BASE,EQEP_POSITION_RESET_MAX_POS,0xFFFFFFFF);

            // Enables the eQEP module.
            EQEP_enableModule(ONE_PULSE_EQEP_BASE);
            break;

        case tow_pulse : // cw , ccw counting mode

            // Configure the GPIOs for eQEP 2pulse ccw counter
            GPIO_setPinConfig(CW_COUNT_PIN_CFG);
            GPIO_setPadConfig(CW_COUNT_GPIO, GPIO_PIN_TYPE_STD);
            GPIO_setQualificationMode(CW_COUNT_GPIO, GPIO_QUAL_SYNC);

            GPIO_setPinConfig(CCW_COUNT_PIN_CFG);
            GPIO_setPadConfig(CCW_COUNT_GPIO, GPIO_PIN_TYPE_STD);
            GPIO_setQualificationMode(CCW_COUNT_GPIO, GPIO_QUAL_SYNC);

            // Sets the polarity of the eQEP module's input signals.
            EQEP_setInputPolarity(ONE_PULSE_EQEP_BASE,false,false,false,false); // invertQEPA, invertQEPB, invertIndex, invertStrobe
            EQEP_setInputPolarity(TWO_PULSE_EQEP_BASE,false,false,false,false); // invertQEPA, invertQEPB, invertIndex, invertStrobe

            // Configures eQEP module's UP count mode.
            EQEP_setDecoderConfig(ONE_PULSE_EQEP_BASE, (EQEP_CONFIG_UP_COUNT | EQEP_CONFIG_1X_RESOLUTION | EQEP_CONFIG_NO_SWAP | EQEP_CONFIG_IGATE_DISABLE));
            EQEP_setDecoderConfig(TWO_PULSE_EQEP_BASE, (EQEP_CONFIG_UP_COUNT | EQEP_CONFIG_1X_RESOLUTION | EQEP_CONFIG_NO_SWAP | EQEP_CONFIG_IGATE_DISABLE));

            // Set the emulation mode of the eQEP module.
            EQEP_setEmulationMode(ONE_PULSE_EQEP_BASE,EQEP_EMULATIONMODE_RUNFREE);
            EQEP_setEmulationMode(TWO_PULSE_EQEP_BASE,EQEP_EMULATIONMODE_RUNFREE);

            // Configures eQEP module position counter unit.
            EQEP_setPositionCounterConfig(ONE_PULSE_EQEP_BASE,EQEP_POSITION_RESET_MAX_POS,0xFFFFFFFF);
            EQEP_setPositionCounterConfig(TWO_PULSE_EQEP_BASE,EQEP_POSITION_RESET_MAX_POS,0xFFFFFFFF);

            // Enables the eQEP module.
            EQEP_enableModule(ONE_PULSE_EQEP_BASE);
            EQEP_enableModule(TWO_PULSE_EQEP_BASE);
            break;
    }
}


void init_sci(void)
{
    // Initialize SCI and its FIFO.
    SCI_performSoftwareReset(RS232_SCI_BASE);

    // Configure SCI with FIFO
    // real baud rate = 57692
    SCI_setConfig(RS232_SCI_BASE, DEVICE_LSPCLK_FREQ, 57600, (SCI_CONFIG_WLEN_8 |
                                                        SCI_CONFIG_STOP_ONE |
                                                        SCI_CONFIG_PAR_NONE));
    SCI_enableModule(RS232_SCI_BASE);
    SCI_enableFIFO(RS232_SCI_BASE);
    SCI_resetChannels(RS232_SCI_BASE);
    SCI_resetRxFIFO(RS232_SCI_BASE);
    SCI_resetTxFIFO(RS232_SCI_BASE);
    SCI_performSoftwareReset(RS232_SCI_BASE);

}

void init_adc(void)
{

    // Enables the temperature sensor output to the ADC.
    ASysCtl_enableTemperatureSensor();
    DEVICE_DELAY_US(500);
    // Set the analog voltage reference selection to external.
    ASysCtl_setAnalogReferenceExternal( ASYSCTL_VREFHI );
    // Set the internal analog voltage reference selection to 1.65V.
    ASysCtl_setAnalogReference1P65( ASYSCTL_VREFHI );


    // Configures the ADC module's offset trim
    ADC_setOffsetTrimAll(ADC_REFERENCE_EXTERNAL,ADC_REFERENCE_3_3V);

    // Configures the analog-to-digital converter module prescaler.
    ADC_setPrescaler(CURRENT_A_ADC, ADC_CLK_DIV_2_0);
    ADC_setPrescaler(CURRENT_B_ADC, ADC_CLK_DIV_2_0);

    // Sets the timing of the end-of-conversion pulse
    ADC_setInterruptPulseMode(CURRENT_A_ADC, ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(CURRENT_B_ADC, ADC_PULSE_END_OF_CONV);

    // Powers up the analog-to-digital converter core.
    ADC_enableConverter(CURRENT_A_ADC);
    ADC_enableConverter(CURRENT_B_ADC);

    DEVICE_DELAY_US(5000);

    // Disables SOC burst mode.
    ADC_disableBurstMode(CURRENT_A_ADC);
    ADC_disableBurstMode(CURRENT_B_ADC);

    // Sets the priority mode of the SOCs.
    ADC_setSOCPriority(CURRENT_A_ADC, ADC_PRI_ALL_ROUND_ROBIN);
    ADC_setSOCPriority(CURRENT_B_ADC, ADC_PRI_ALL_ROUND_ROBIN);

    // Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
    ADC_setupSOC(CURRENT_A_ADC, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN0, CURRENT_ACQPS);
    ADC_setupSOC(CURRENT_B_ADC, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN0, CURRENT_ACQPS);

    ADC_setupSOC(CURRENT_A_ADC, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN1, DC_LINK_ACQPS);
    ADC_setupSOC(CURRENT_A_ADC, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN5, DEVICE_TEMP_SENSOR_ACQPS);
    ADC_setupSOC(CURRENT_A_ADC, ADC_SOC_NUMBER3, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN12, DSP_TEMP_SENSOR_ACQPS);


    ADC_setInterruptSOCTrigger(CURRENT_A_ADC, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_NONE);
    ADC_setInterruptSOCTrigger(CURRENT_B_ADC, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_NONE);

    ADC_setInterruptSource(CURRENT_A_ADC, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
    ADC_clearInterruptStatus(CURRENT_A_ADC, ADC_INT_NUMBER1);
    ADC_disableContinuousMode(CURRENT_A_ADC, ADC_INT_NUMBER1);
    ADC_enableInterrupt(CURRENT_A_ADC, ADC_INT_NUMBER1);

    ADC_setInterruptSource(CURRENT_B_ADC, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
    ADC_clearInterruptStatus(CURRENT_B_ADC, ADC_INT_NUMBER1);
    ADC_disableContinuousMode(CURRENT_B_ADC, ADC_INT_NUMBER1);
    ADC_enableInterrupt(CURRENT_B_ADC, ADC_INT_NUMBER1);


    // DC LINK interrupt no mapping
    ADC_setInterruptSource(CURRENT_A_ADC, ADC_INT_NUMBER2, ADC_SOC_NUMBER1);
    ADC_clearInterruptStatus(CURRENT_A_ADC, ADC_INT_NUMBER2);
    ADC_disableContinuousMode(CURRENT_A_ADC, ADC_INT_NUMBER2);
    ADC_enableInterrupt(CURRENT_A_ADC, ADC_INT_NUMBER2);

    // Device temp interrupt no mapping
    ADC_setInterruptSource(CURRENT_A_ADC, ADC_INT_NUMBER3, ADC_SOC_NUMBER2);
    ADC_clearInterruptStatus(CURRENT_A_ADC, ADC_INT_NUMBER3);
    ADC_disableContinuousMode(CURRENT_A_ADC, ADC_INT_NUMBER3);
    ADC_enableInterrupt(CURRENT_A_ADC, ADC_INT_NUMBER3);

    // DSP temp interrupt no mapping
    ADC_setInterruptSource(CURRENT_A_ADC, ADC_INT_NUMBER4, ADC_SOC_NUMBER3);
    ADC_clearInterruptStatus(CURRENT_A_ADC, ADC_INT_NUMBER4);
    ADC_disableContinuousMode(CURRENT_A_ADC, ADC_INT_NUMBER4);
    ADC_enableInterrupt(CURRENT_A_ADC, ADC_INT_NUMBER4);


    ADC_setupPPB(CURRENT_A_ADC, ADC_PPB_NUMBER1, ADC_SOC_NUMBER0);
    ADC_setPPBReferenceOffset(CURRENT_A_ADC, ADC_PPB_NUMBER1, 2048);

    ADC_setupPPB(CURRENT_B_ADC, ADC_PPB_NUMBER1, ADC_SOC_NUMBER0);
    ADC_setPPBReferenceOffset(CURRENT_B_ADC, ADC_PPB_NUMBER1, 2048);
}


void init_cpu_timers(void)
{
    // Initialize timer period to maximum
    CPUTimer_setPeriod(CPUTIMER0_BASE, 0xFFFFFFFF);

    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);

    // Make sure timer is stopped
    CPUTimer_stopTimer(CPUTIMER0_BASE);

    // Reload all counter register with period value
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);

    // Period respectively (in uSeconds)
    configure_cpu_timer(CPUTIMER0_BASE, DEVICE_SYSCLK_FREQ, 1000);

}


void configure_cpu_timer(uint32_t cpuTimer, float freq, float period)
{
    uint32_t temp;

    //
    // Initialize timer period:
    //
    temp = (uint32_t)((freq / 1000000) * period);
    CPUTimer_setPeriod(cpuTimer, temp);

    //
    // Set pre-scale counter to divide by 1 (SYSCLKOUT):
    //
    CPUTimer_setPreScaler(cpuTimer, 0);

    //
    // Initializes timer control register. The timer is stopped, reloaded,
    // free run disabled, and interrupt enabled.
    // Additionally, the free and soft bits are set
    //
    CPUTimer_stopTimer(cpuTimer);
    CPUTimer_reloadTimerCounter(cpuTimer);
    CPUTimer_setEmulationMode(cpuTimer,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);

    CPUTimer_enableInterrupt(cpuTimer);
}




