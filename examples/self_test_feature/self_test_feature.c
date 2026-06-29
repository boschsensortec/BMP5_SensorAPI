/**\
 * Copyright (c) 2026 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include "bmp5_selftest.h"
#include "common.h"

/******************************************************************************/
/*!            Functions                                        */
int main()
{
    /* Variable to store the result of API calls */
    int8_t rslt;

    /* Device structure to hold BMP5 sensor configuration and state */
    struct bmp5_dev dev;

    /* Interface reference is given as a parameter
     * For I2C : BMP5_I2C_INTF
     * For SPI : BMP5_SPI_INTF
     */
    rslt = bmp5_interface_init(&dev, BMP5_SPI_INTF);
    bmp5_error_codes_print_result("bmp5_interface_init", rslt);

    if (rslt == BMP5_OK)
    {
        /* Start Self Test */
        printf("Self test starting\n");
        rslt = bmp5_selftest_check(&dev);
        bmp5_error_codes_print_result("bmp5_selftest_check", rslt);
    }

    if (rslt == BMP5_OK)
    {
        printf("\nSelf test completed");
    }
    else
    {
        printf("\nSelf test failed");
    }

    bmp5_coines_deinit();

    return rslt;
}
