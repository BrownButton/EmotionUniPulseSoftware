/*
 * pulse_device.c
 *
 *  Created on: 2023. 10. 11.
 *      Author: JH
 */


//
// Included Files
//
#include "pulse_device.h"
#include "driverlib.h"
#include "inc/hw_lcm.h"

#ifdef __cplusplus
using std::memcpy;
#endif
#ifdef CMDTOOL
#include "device_cmd.h"
#endif

//*****************************************************************************
//
// Function to initialize the device. Primarily initializes system control to a
// known state by disabling the watchdog, setting up the SYSCLKOUT frequency,
// and enabling the clocks to the peripherals.
// The function also configures the AGPIO pins 12, 13, 20, 21, 28 in digital mode.
// To configure these pins as analog pins, use the function GPIO_setAnalogMode
//
//*****************************************************************************
void Device_init(void)
{
    //
    // Disable the watchdog
    //
    SysCtl_disableWatchdog();
#ifdef CMDTOOL
    CMD_init();
#endif

#ifdef _FLASH
#ifndef CMDTOOL
    //
    // Copy time critical code and flash setup code to RAM. This includes the
    // following functions: InitFlash();
    //
    // The RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart symbols
    // are created by the linker. Refer to the device .cmd file.
    //
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
#endif
    //
    // Call Flash Initialization to setup flash waitstates. This function must
    // reside in RAM.
    //
    Flash_initModule(FLASH0CTRL_BASE, FLASH0ECC_BASE, DEVICE_FLASH_WAITSTATES);
#endif

    //
    // Set up PLL control and clock dividers
    //
    SysCtl_setClock(DEVICE_SETCLOCK_CFG);

    //
    // Make sure the LSPCLK divider is set to the default (divide by 4)
    //
    SysCtl_setLowSpeedClock(SYSCTL_LSPCLK_PRESCALE_4);

    //
    // These asserts will check that the #defines for the clock rates in
    // device.h match the actual rates that have been configured. If they do
    // not match, check that the calculations of DEVICE_SYSCLK_FREQ and
    // DEVICE_LSPCLK_FREQ are accurate. Some examples will not perform as
    // expected if these are not correct.
    //
    ASSERT(SysCtl_getClock(DEVICE_OSCSRC_FREQ) == DEVICE_SYSCLK_FREQ);
    ASSERT(SysCtl_getLowSpeedClock(DEVICE_OSCSRC_FREQ) == DEVICE_LSPCLK_FREQ);

#ifndef _FLASH
    //
    // Call Device_cal function when run using debugger
    // This function is called as part of the Boot code. The function is called
    // in the Device_init function since during debug time resets, the boot code
    // will not be executed and the gel script will reinitialize all the
    // registers and the calibrated values will be lost.
    // Sysctl_deviceCal is a wrapper function for Device_Cal
    //
    SysCtl_deviceCal();
#endif

    //
    // Turn on all peripherals
    //
    Device_enableAllPeripherals();

    //
    // Lock VREGCTL Register
    // The register VREGCTL is not supported in this device. It is locked to
    // prevent any writes to this register
    //
    ASysCtl_lockVREG();

    //
    // Configure the portA AGPIOs as digital pins
    //
    GPIO_setAnalogMode(12U, GPIO_ANALOG_DISABLED);
    GPIO_setAnalogMode(13U, GPIO_ANALOG_DISABLED);
    GPIO_setAnalogMode(20U, GPIO_ANALOG_DISABLED);
    GPIO_setAnalogMode(21U, GPIO_ANALOG_DISABLED);
    GPIO_setAnalogMode(28U, GPIO_ANALOG_DISABLED);

    //
    // Enable lockstep compare module
    //
    HWREG_BP(LCM_CPU1_BASE + LCM_O_CONTROL) |= LCM_CONTROL_CMPEN;

}

//*****************************************************************************
//
// Function to turn on all peripherals, enabling reads and writes to the
// peripherals' registers.
//
// Note that to reduce power, unused peripherals should be disabled.
//
//*****************************************************************************
void Device_enableAllPeripherals(void)
{
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER0);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_HRCAL);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM5);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM6);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM7);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP3);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP2);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCIA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCIB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCIC);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPIA);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_I2CA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_I2CB);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CANA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_MCANA);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCC);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS4);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_LINA);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_PMBUSA);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DCC0);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPG1);
}

//*****************************************************************************
//
// Function to disable pin locks and enable pullups on GPIOs.
//
//*****************************************************************************
void Device_initGPIO(void)
{
    //
    // Disable pin locks.
    //
    GPIO_unlockPortConfig(GPIO_PORT_A, 0xFFFFFFFF);
    GPIO_unlockPortConfig(GPIO_PORT_B, 0xFFFFFFFF);
    GPIO_unlockPortConfig(GPIO_PORT_H, 0xFFFFFFFF);
}

//*****************************************************************************
//
// Error handling function to be called when an ASSERT is violated
//
//*****************************************************************************
void __error__(const char *filename, uint32_t line)
{
    //
    // An ASSERT condition was evaluated as false. You can use the filename and
    // line parameters to determine what went wrong.
    //
    ESTOP0;
}