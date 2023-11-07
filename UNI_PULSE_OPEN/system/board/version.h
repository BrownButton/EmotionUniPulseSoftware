/*
 * version.h
 *
 *  Created on: 2023. 10. 24.
 *      Author: JH
 */

#ifndef SYSTEM_BOARD_VERSION_H_
#define SYSTEM_BOARD_VERSION_H_

#include "driverlib.h"


//===== SW version info ======//
#define OS_VERSION_MAJOR  1
#define OS_VERSION_MINOR  0
#define OS_VERSION_YEAR   23
#define OS_VERSION_MONTH  10
#define OS_VERSION_DAY    24

#define PRODUCT_NO       0x10004001 //UNI PULSE 60 1X (open loop type)



typedef union {
    struct  {
        unsigned    SW_Update_DD    :5;
        unsigned    SW_Update_MM    :4;
        unsigned    SW_Update_YY    :7;
        unsigned    SW_Minor_Ver    :8;
        unsigned    SW_Major_Ver    :8;
    }Bit;
    uint32_t    Full;
}OS_VERSION;
extern OS_VERSION os_version;

void get_product_infomation(void);



#endif /* SYSTEM_BOARD_VERSION_H_ */
