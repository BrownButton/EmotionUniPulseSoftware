/*
 * pulse_device.h
 *
 *  Created on: 2023. 10. 11.
 *      Author: JH
 */

#ifndef SYSTEM_BOARD_PULSE_DEVICE_H_
#define SYSTEM_BOARD_PULSE_DEVICE_H_


#ifdef __cplusplus
extern "C"
{
#endif

//
// Included Files
//
#include "driverlib.h"


/********************************************************************/
// initial configuration values
/********************************************************************/
#define QUALPRD_DIVIDE          90          // Qualification sampling period =   PLLSYSCLK / QUALPRD_150 / 2
#define EPWM_TBPRD              1500        // TBCLK = EPWMCLK / (HSPCLKDIV x CLKDIV) => 120MHz / (1 x 1) => 120[MHz]
                                            // TPWM = 2 x TBPRD x (1/TBCLK) => 2 x 1500 x (1/120MHz) => 25[us] = 40[KHz]

//ePWMs
#define A_PWM_BASE              EPWM1_BASE
#define B_PWM_BASE              EPWM2_BASE


//eQEPs
#define ONE_PULSE_EQEP_BASE            EQEP1_BASE
#define TWO_PULSE_EQEP_BASE            EQEP2_BASE


//SCI
#define RS232_SCI_BASE          SCIB_BASE

//ADCs
#define CURRENT_A_ADC           ADCA_BASE
#define CURRENT_B_ADC           ADCC_BASE

#define CURRENT_A_ADCARESULT    ADCARESULT_BASE
#define CURRENT_B_ADCARESULT    ADCCRESULT_BASE


#define CURRENT_ACQPS                17
#define DSP_TEMP_SENSOR_ACQPS        84
#define DEVICE_TEMP_SENSOR_ACQPS     511
#define DC_LINK_ACQPS                420



/********************************************************************/
// Configure PIN ePWM for control current
/********************************************************************/
//Current A, /A ePWM
#define A_PWM_GPIO              0
#define A_PWM_PIN_CFG           GPIO_0_EPWM1_A

#define Abar_PWM_GPIO           1
#define Abar_PWM_PIN_CFG        GPIO_1_EPWM1_B

//Current B, /B ePWM
#define B_PWM_GPIO              2
#define B_PWM_PIN_CFG           GPIO_2_EPWM2_A

#define Bbar_PWM_GPIO           3
#define Bbar_PWM_PIN_CFG        GPIO_3_EPWM2_B

/********************************************************************/
// Configure PIN eQEP for count pulse
/********************************************************************/
// 1pulse mode / direction
#define CW_COUNT_GPIO             20
#define CW_COUNT_PIN_CFG          GPIO_20_EQEP1_A

#define DIRECTION_GPIO            21
#define DIRECTION_PIN_CFG         GPIO_21_EQEP1_B

// 2pulse ccw count
#define CCW_COUNT_GPIO            11
#define CCW_COUNT_PIN_CFG         GPIO_11_EQEP2_A

/********************************************************************/
// Configure PIN UART for RS232 communication
/********************************************************************/
#define RS232_RXD_GPIO                  23
#define RS232_RXD_PIN_CFG               GPIO_23_SCIB_RX

#define RS232_TXD_GPIO                  22
#define RS232_TXD_PIN_CFG               GPIO_22_SCIB_TX

/********************************************************************/
// Configure PIN the GPIOs Outputs
/********************************************************************/
#define PWM_ENABLE_GPIO             40
#define PWM_ENABLE_PIN_CFG         GPIO_40_GPIO40

#define RUN_LED_GPIO                24
#define RUN_LED_GPIO_PIN_CFG        GPIO_24_GPIO24

#define ERR_LED_GPIO                32
#define ERR_LED_GPIO_PIN_CFG        GPIO_32_GPIO32

#define BRAKE_GPIO                  227
#define BRAKE_GPIO_PIN_CFG          GPIO_227_GPIO227

#define ALARM_GPIO                  13
#define ALARM_GPIO_PIN_CFG          GPIO_13_GPIO13


/********************************************************************/
// Configure PIN the GPIOs Inputs
/********************************************************************/
#define DRIVER_ENABLE_GPIO              230
#define DRIVER_ENABLE_PIN_CFG           GPIO_230_GPIO230

#define RUN_CURRNET0_GPIO               12
#define RUN_CURRNET0_PIN_CFG            GPIO_12_GPIO12
#define RUN_CURRNET1_GPIO               33
#define RUN_CURRNET1_PIN_CFG            GPIO_33_GPIO33
#define RUN_CURRNET2_GPIO               16
#define RUN_CURRNET2_PIN_CFG            GPIO_16_GPIO16
#define RUN_CURRNET3_GPIO               17
#define RUN_CURRNET3_PIN_CFG            GPIO_17_GPIO17

#define STOP_CURRNET0_GPIO              228
#define STOP_CURRNET0_PIN_CFG           GPIO_228_GPIO228
#define STOP_CURRNET1_GPIO              226
#define STOP_CURRNET1_PIN_CFG           GPIO_226_GPIO226

#define GAIN0_GPIO                      29
#define GAIN0_PIN_CFG                   GPIO_29_GPIO29
#define GAIN1_GPIO                      28
#define GAIN1_PIN_CFG                   GPIO_28_GPIO28

#define RESOLUTION0_GPIO                8
#define RESOLUTION0_PIN_CFG             GPIO_8_GPIO8
#define RESOLUTION1_GPIO                4
#define RESOLUTION1_PIN_CFG             GPIO_4_GPIO4
#define RESOLUTION2_GPIO                41
#define RESOLUTION2_PIN_CFG             GPIO_41_GPIO41
#define RESOLUTION3_GPIO                7
#define RESOLUTION3_PIN_CFG             GPIO_7_GPIO7

#define SELP_TEST_GPIO                  5
#define SELP_TEST_PIN_CFG               GPIO_5_GPIO5

#define GAIN_SEL_GPIO                   6
#define GAIN_SEL_PIN_CFG                GPIO_6_GPIO6

#define INPUT_MODE_GPIO                 9
#define INPUT_MODE_PIN_CFG              GPIO_9_GPIO9

#define MOTOR_DIRECTION_GPIO            10
#define MOTOR_DIRECTION_PIN_CFG         GPIO_10_GPIO10



//*****************************************************************************
//
// Defines related to clock configuration
//
//*****************************************************************************

//
// To use INTOSC as the clock source, comment #define USE_PLL_SRC_XTAL,
// and uncomment the #define USE_PLL_SRC_INTOSC
//
#define USE_PLL_SRC_XTAL
//#define USE_PLL_SRC_INTOSC

#if defined(USE_PLL_SRC_XTAL)
//
// 20MHz XTAL on controlCARD is used as the PLL source.
// For use with SysCtl_getClock().
//
#define DEVICE_OSCSRC_FREQ          20000000U

//
// Define to pass to SysCtl_setClock(). Will configure the clock as follows:
// PLLSYSCLK = 20MHz (XTAL_OSC) * 48 (IMULT) / (2 (REFDIV) * 4 (ODIV) * 1(SYSDIV))
//
#define DEVICE_SETCLOCK_CFG          (SYSCTL_OSCSRC_XTAL | SYSCTL_IMULT(48) | \
                                      SYSCTL_REFDIV(2) | SYSCTL_ODIV(4) | \
                                      SYSCTL_SYSDIV(1) | SYSCTL_PLL_ENABLE | \
                                      SYSCTL_DCC_BASE_0)

//
// 120MHz SYSCLK frequency based on the above DEVICE_SETCLOCK_CFG. Update the
// code below if a different clock configuration is used!
//
#define DEVICE_SYSCLK_FREQ          ((DEVICE_OSCSRC_FREQ * 48) / (2 * 4 * 1))

#elif defined(USE_PLL_SRC_INTOSC)
//
// 10MHz INTOSC on the device is used as the PLL source.
// For use with SysCtl_getClock().
//
#define DEVICE_OSCSRC_FREQ          10000000U

//
// Define to pass to SysCtl_setClock(). Will configure the clock as follows:
// PLLSYSCLK = 10MHz (INT_OSC2) * 48 (IMULT) / (1 (REFDIV) * 4 (ODIV) * 1(SYSDIV))
//
#define DEVICE_SETCLOCK_CFG          (SYSCTL_OSCSRC_OSC2 | SYSCTL_IMULT(48) | \
                                      SYSCTL_REFDIV(1) | SYSCTL_ODIV(4) | \
                                      SYSCTL_SYSDIV(1) | SYSCTL_PLL_ENABLE | \
                                      SYSCTL_DCC_BASE_0)

//
// 120MHz SYSCLK frequency based on the above DEVICE_SETCLOCK_CFG. Update the
// code below if a different clock configuration is used!
//
#define DEVICE_SYSCLK_FREQ          ((DEVICE_OSCSRC_FREQ * 48) / (1 * 4 * 1))
#endif

//
// 30MHz LSPCLK frequency based on the above DEVICE_SYSCLK_FREQ and a default
// low speed peripheral clock divider of 4. Update the code below if a
// different LSPCLK divider is used!
//
//#define DEVICE_LSPCLK_FREQ          (DEVICE_SYSCLK_FREQ / 4)
#define DEVICE_LSPCLK_FREQ          (DEVICE_SYSCLK_FREQ)

//*****************************************************************************
//
// Macro to call SysCtl_delay() to achieve a delay in microseconds. The macro
// will convert the desired delay in microseconds to the count value expected
// by the function. \b x is the number of microseconds to delay.
//
//*****************************************************************************
#define DEVICE_DELAY_US(x) SysCtl_delay(((((long double)(x)) / (1000000.0L /  \
                              (long double)DEVICE_SYSCLK_FREQ)) - 9.0L) / 5.0L)

//*****************************************************************************
//
// Defines, Globals, and Header Includes related to Flash Support
//
//*****************************************************************************
#ifdef _FLASH
#include <stddef.h>

#ifndef CMDTOOL
extern uint16_t RamfuncsLoadStart;
extern uint16_t RamfuncsLoadEnd;
extern uint16_t RamfuncsLoadSize;
extern uint16_t RamfuncsRunStart;
extern uint16_t RamfuncsRunEnd;
extern uint16_t RamfuncsRunSize;
#endif

#endif

#define DEVICE_FLASH_WAITSTATES 2

//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************
//*****************************************************************************
//
//! \addtogroup device_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! @brief Function to initialize the device. Primarily initializes system
//!  control to aknown state by disabling the watchdog, setting up the
//!  SYSCLKOUT frequency, and enabling the clocks to the peripherals.
//!
//! \param None.
//! \return None.
//
//*****************************************************************************
extern void Device_init(void);

//*****************************************************************************
//!
//!
//! @brief Function to turn on all peripherals, enabling reads and writes to the
//! peripherals' registers.
//!
//! Note that to reduce power, unused peripherals should be disabled.
//!
//! @param None
//! @return None
//
//*****************************************************************************
extern void Device_enableAllPeripherals(void);
//*****************************************************************************
//!
//!
//! @brief Function to disable pin locks on GPIOs.
//!
//! @param None
//! @return None
//
//*****************************************************************************
extern void Device_initGPIO(void);

//*****************************************************************************
//!
//! @brief Error handling function to be called when an ASSERT is violated
//!
//! @param *filename File name in which the error has occurred
//! @param line Line number within the file
//! @return None
//
//*****************************************************************************
extern void __error__(const char *filename, uint32_t line);

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

#ifdef __cplusplus
}
#endif


#endif /* SYSTEM_BOARD_PULSE_DEVICE_H_ */
