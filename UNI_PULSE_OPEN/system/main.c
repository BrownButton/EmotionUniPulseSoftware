/*
 * main.c
 *
 *  Created on: 2023. 10. 11.
 *      Author: JH
 */


#include "system.h"
#include "control.h"
#include "version.h"


void main(void)
{
    // initialize device
    initialize_system();

    // initialize control variable
    initialze_variable();
    
    // version information
    get_product_infomation();

    while(1)
    {
        ;
    }


}
