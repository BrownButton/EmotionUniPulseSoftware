/*
 * version.c
 *
 *  Created on: 2023. 10. 24.
 *      Author: JH
 */






#include "version.h"

OS_VERSION os_version;


void get_product_infomation(void)
{
    os_version.Bit.SW_Major_Ver = OS_VERSION_MAJOR;
    os_version.Bit.SW_Minor_Ver = OS_VERSION_MINOR;
    os_version.Bit.SW_Update_YY = OS_VERSION_YEAR;
    os_version.Bit.SW_Update_MM = OS_VERSION_MONTH;
    os_version.Bit.SW_Update_DD = OS_VERSION_DAY;
}


/*
 *
 * [Version 1.0.]
 * - 2023-10-24
 *  1) initial released
 *
 *
 *
 */
