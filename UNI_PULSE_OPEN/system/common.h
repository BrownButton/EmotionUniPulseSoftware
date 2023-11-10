/*
 * common.h
 *
 *  Created on: 2023. 10. 24.
 *      Author: JH
 */

#ifndef SYSTEM_COMMON_H_
#define SYSTEM_COMMON_H_

#define     PI                      3.14159265358979

#define     ADC_REF_CAL_CNT                 20000
#define     ADC_VREF_HIGH                   3.3
#define     ADC_RESOLUTION                  4096.0
#define     SHUNT_RES                       0.05
#define     OPAMP_GAIN                      6.0
#define     CURRENT_PER_BITS                (float32_t)(ADC_VREF_HIGH/(ADC_RESOLUTION * SHUNT_RES * OPAMP_GAIN))
#define     ALARM_CURRENT_LIMIT             5.4
#define     CONTROL_CURRENT_LIMIT           4.5

#define     CURRENT_CONTROL_FREQ            40000    /* 40.0 kHz [25usec] */
#define     CURRENT_CONTROL_SAMPLE_TIME     (float32_t)(1.0/CURRENT_CONTROL_FREQ)
#define     SPEED_CONTROL_FREQ              1000.0 /* 1 kHz [1msec]*/
#define     SPEED_LOOP_COUNT                ( CURRENT_CONTROL_FREQ/SPEED_CONTROL_FREQ )
#define     SPEED_CONTROL_SAMPLE_TIME       (float32_t)(1.0/SPEED_CONTROL_FREQ)
#define     SEC_MIN_CONST                   60.0

#define     VDC                             24.0
#define     VDC_RECIPROCAL                  (float32_t)(1.0 / VDC)

#define DC_LINK_CONST       0.012890625 // (3.3/4096)*((27000+1800)/1800)
#define DEVICE_TEMP_CONST   3.3*100.0 / 4096.0

#define POLE_NUMBER         100

#endif /* SYSTEM_COMMON_H_ */
