/*
 * serial.h
 *
 *  Created on: 2023. 11. 3.
 *      Author: JH
 */


#include "driverlib.h"


#define     CMD_VERSION_INFO             0
#define     CMD_PRODUCT_INFO             1
#define     CMD_PARAMETER_INFO           2
#define     CMD_ERROR_STATUS             3
#define     CMD_MONITORING_DATA0         4
#define     CMD_MONITORING_DATA1         5
#define     CMD_MONITORING_DATA2         6
#define     CMD_MONITORING_DATA3         7

#define     ERROR_INVALID_CMD           100

#define     INDEX_CMD                   0



void process_serial_communication(void);

