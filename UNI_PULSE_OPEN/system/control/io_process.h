/*
 * io_process.h
 *
 *  Created on: 2023. 10. 31.
 *      Author: JH
 */

#ifndef SYSTEM_CONTROL_IO_PROCESS_H_
#define SYSTEM_CONTROL_IO_PROCESS_H_

#include "driverlib.h"


#define IO_ENABLE_CHANGE_TIME       50
#define LED_TOGGLE_TIME             20
#define BRAKE_LOCK_TIME             10 //ms
#define BRAKE_UNLOCK_TIME           5  //ms

typedef union {
    struct  {
        unsigned    enable          :1;
        unsigned    reserved       :15;
    }Bit;
    int16_t    Full;
}INPUT_STATUS;
extern INPUT_STATUS input_status;

typedef union {
    struct  {
        unsigned    brake          :1;
        unsigned    alarm          :1;
        unsigned    reserved       :14;
    }Bit;
    int16_t    Full;
}OUTPUT_STATUS;
extern OUTPUT_STATUS output_status;




void io_process(void);



#endif /* SYSTEM_CONTROL_IO_PROCESS_H_ */
