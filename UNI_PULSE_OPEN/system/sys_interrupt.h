/*
 * sys_interrupt.h
 *
 *  Created on: 2023. 10. 20.
 *      Author: JH
 */

#ifndef SYSTEM_SYS_INTERRUPT_H_
#define SYSTEM_SYS_INTERRUPT_H_



__interrupt void control_system_loop(void);
__interrupt void control_current_loop(void);
__interrupt void calibration_adc_ref_loop(void);
__interrupt void check_over_current(void);


#endif /* SYSTEM_SYS_INTERRUPT_H_ */
