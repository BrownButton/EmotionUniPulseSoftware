/*
 * board.h
 *
 *  Created on: 2023. 10. 23.
 *      Author: JH
 */

#ifndef SYSTEM_BOARD_BOARD_H_
#define SYSTEM_BOARD_BOARD_H_

#include "pulse_device.h"


/********************************************************************/
// Define function I/O On/Off
/********************************************************************/
#define PWM_ENABLE                  GPIO_writePin(PWM_ENABLE_GPIO, 1)
#define PWM_DIABLE                  GPIO_writePin(PWM_ENABLE_GPIO, 0)

#define RUN_LED_ON                  GPIO_writePin(RUN_LED_GPIO, 1)
#define RUN_LED_OFF                 GPIO_writePin(RUN_LED_GPIO, 0)
#define RUN_LED_TOGGLE              GPIO_togglePin(RUN_LED_GPIO);

#define ERR_LED_ON                  GPIO_writePin(ERR_LED_GPIO, 1)
#define ERR_LED_OFF                 GPIO_writePin(ERR_LED_GPIO, 0)

#define BRAKE_LOCK                  GPIO_writePin(BRAKE_GPIO, 1)
#define BRAKE_UNLOCK                GPIO_writePin(BRAKE_GPIO, 0)

#define ALARM_OFF                   GPIO_writePin(ALARM_GPIO, 1)
#define ALARM_ON                    GPIO_writePin(ALARM_GPIO, 0)






void epwm_aq_enable_all(void);
void epwm_aq_enable_aphase(void);
uint16_t get_voltage_duty(void);
void set_voltage_duty(uint16_t a_comp_count, uint16_t b_comp_count);


int16_t initialize_adc_reference(void);
void read_current(float32_t *a_phase, float32_t *b_phase);
void convert_adc_system_variable(void);
void update_adc_system_variable(float32_t *dc_link, float32_t *device_temp, float32_t *dsp_temp);

int32_t read_cw_command(void);
int32_t read_ccw_command(void);

void clear_cw_command(void);
void clear_ccw_command(void);

uint16_t sevo_control(uint16_t cmd);
uint16_t get_input_io_status(void);
uint16_t get_output_io_status(void);

uint16_t check_rx_buf(void);
void read_rx_data(uint16_t *rx_data);
void write_tx_data(uint16_t tx_data);


#endif /* SYSTEM_BOARD_BOARD_H_ */
