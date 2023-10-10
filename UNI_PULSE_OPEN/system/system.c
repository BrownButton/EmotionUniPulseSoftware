/*
 * system.c
 *
 *  Created on: 2023. 10. 11.
 *      Author: JH
 */

#include "pulse_device.h"
#include "system.h"



void configure_gpios(void);


void initialization_system(void)
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
//    Interrupt_register(INT_EPWM1, &epwm1ISR);


    // Disable sync(Freeze clock to PWM as well)
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);


    // GPIOs pin configuration
    configure_gpios();

    // Initialize peripherals
//    confiure_peripherals();



    // Enable sync and clock to PWM
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // Enable EPWM1 Interrupt
//    Interrupt_enable(INT_EPWM1);



    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    EINT;
    ERTM;

}

void configure_gpios(void)
{

    /********************************************************************/
    // Configure the GPIOs for ePWM
    /********************************************************************/
    //Current A
    GPIO_setPinConfig(CURRENT_A_PWM_PIN_CFG);
    GPIO_setPadConfig(CURRENT_A_PWM_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(CURRENT_A_PWM_GPIO, GPIO_QUAL_SYNC);
    //Current /A
    GPIO_setPinConfig(CURRENT_Abar_PWM_PIN_CFG);
    GPIO_setPadConfig(CURRENT_Abar_PWM_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(CURRENT_Abar_PWM_GPIO, GPIO_QUAL_SYNC);
    //Current B
    GPIO_setPinConfig(CURRENT_B_PWM_PIN_CFG);
    GPIO_setPadConfig(CURRENT_B_PWM_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(CURRENT_B_PWM_GPIO, GPIO_QUAL_SYNC);
    //Current /B
    GPIO_setPinConfig(CURRENT_Bbar_PWM_PIN_CFG);
    GPIO_setPadConfig(CURRENT_Bbar_PWM_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(CURRENT_Bbar_PWM_GPIO, GPIO_QUAL_SYNC);

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




}




















































