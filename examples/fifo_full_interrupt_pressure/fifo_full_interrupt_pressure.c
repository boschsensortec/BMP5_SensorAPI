/**\
 * Copyright (c) 2026 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include "bmp5.h"
#include "common.h"

/******************************************************************************/
/*!                Macro definition                                           */

#define BMP5_FIFO_DATA_BUFFER_SIZE  UINT8_C(96)
#define BMP5_FIFO_DATA_USER_LENGTH  UINT8_C(96)
#define BMP5_FIFO_P_FRAME_COUNT     UINT8_C(32)

/******************************************************************************/
/*!            Functions                                        */

int main(void)
{
    /* Variable to store the result of API calls */
    int8_t rslt;

    /* Index variable for iterating through FIFO frames */
    uint8_t idx = 0;

    /* Loop counter for limiting the number of iterations */
    uint8_t loop = 0;

    /* Variable to store interrupt status */
    uint8_t int_status;

    /* Device structure to hold sensor configuration and interface details */
    struct bmp5_dev dev;

    /* FIFO structure to hold FIFO configuration and data */
    struct bmp5_fifo fifo;

    /* Structure to hold oversampling rate (OSR) and output data rate (ODR) configuration for pressure */
    struct bmp5_osr_odr_press_config osr_odr_press_cfg;

    /* Structure to hold IIR filter configuration */
    struct bmp5_iir_config iir_cfg;

    /* Structure to select interrupt sources */
    struct bmp5_int_source_select int_source_select;

    uint8_t fifo_buffer[BMP5_FIFO_DATA_BUFFER_SIZE];
    struct bmp5_sensor_data sensor_data[BMP5_FIFO_P_FRAME_COUNT] = { { 0 } };

    /* Interface reference is given as a parameter
     * For I2C : BMP5_I2C_INTF
     * For SPI : BMP5_SPI_INTF
     */
    rslt = bmp5_interface_init(&dev, BMP5_SPI_INTF);
    bmp5_error_codes_print_result("bmp5_interface_init", rslt);

    if (rslt == BMP5_OK)
    {
        rslt = bmp5_init(&dev);
        bmp5_error_codes_print_result("bmp5_init", rslt);

        rslt = bmp5_get_osr_odr_press_config(&osr_odr_press_cfg, &dev);
        bmp5_error_codes_print_result("bmp5_get_osr_odr_press_config", rslt);

        osr_odr_press_cfg.odr = BMP5_ODR_50_HZ;

        /* Enable pressure */
        osr_odr_press_cfg.press_en = BMP5_ENABLE;
        osr_odr_press_cfg.osr_t = BMP5_OVERSAMPLING_8X;
        osr_odr_press_cfg.osr_p = BMP5_OVERSAMPLING_64X;

        rslt = bmp5_set_osr_odr_press_config(&osr_odr_press_cfg, &dev);
        bmp5_error_codes_print_result("bmp5_set_osr_odr_press_config", rslt);
        printf("OSR ODR configurations\n\n");
        printf("OSR T : %u\n", osr_odr_press_cfg.press_en);

        rslt = bmp5_get_iir_config(&iir_cfg, &dev);
        bmp5_error_codes_print_result("bmp5_get_iir_config", rslt);

        iir_cfg.set_iir_p = BMP5_IIR_FILTER_COEFF_1;

        rslt = bmp5_set_iir_config(&iir_cfg, &dev);
        bmp5_error_codes_print_result("bmp5_set_iir_config", rslt);
        printf("IIR configurations\n\n");
        printf("IIR T : %u\n\n", iir_cfg.set_iir_t);

        rslt = bmp5_get_fifo_configuration(&fifo, &dev);
        bmp5_error_codes_print_result("bmp5_get_fifo_configuration", rslt);

        fifo.mode = BMP5_FIFO_MODE_STREAMING;
        fifo.frame_sel = BMP5_FIFO_PRESSURE_DATA;
        fifo.dec_sel = BMP5_FIFO_NO_DOWNSAMPLING;
        fifo.set_fifo_iir_p = BMP5_ENABLE;

        rslt = bmp5_set_fifo_configuration(&fifo, &dev);
        bmp5_error_codes_print_result("bmp5_set_fifo_configuration", rslt);
        printf("FIFO configurations\n\n");
        printf("FIFO Mode : %u\n", fifo.mode);
        printf("FIFO Frame Selection : %u\n", fifo.frame_sel);
        printf("FIFO Decimation Selection : %u\n", fifo.dec_sel);
        printf("FIFO IIR P : %u\n\n", fifo.set_fifo_iir_p);

        rslt = bmp5_configure_interrupt(BMP5_PULSED, BMP5_ACTIVE_HIGH, BMP5_INTR_PUSH_PULL, BMP5_INTR_ENABLE, &dev);

        int_source_select.fifo_full_en = BMP5_ENABLE;
        rslt = bmp5_int_source_select(&int_source_select, &dev);
        bmp5_error_codes_print_result("bmp5_int_source_select", rslt);

        /*
         * FIFO example can be executed on,
         * normal mode - BMP5_POWERMODE_NORMAL
         * continuous mode - BMP5_POWERMODE_CONTINUOUS
         * Here, used normal mode (BMP5_POWERMODE_NORMAL)
         */

        rslt = bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, &dev);
        bmp5_error_codes_print_result("bmp5_set_power_mode", rslt);

        while (loop < 5)
        {
            rslt = bmp5_get_interrupt_status(&int_status, &dev);
            bmp5_error_codes_print_result("bmp5_get_interrupt_status", rslt);

            if (int_status & BMP5_INT_ASSERTED_FIFO_FULL)
            {
                fifo.length = BMP5_FIFO_DATA_USER_LENGTH;
                fifo.data = fifo_buffer;

                printf("Iteration: %u\n", loop);
                printf("Each fifo frame contains 3 bytes of data\n");
                printf("Fifo data bytes requested: %u\n\n", fifo.length);

                rslt = bmp5_get_fifo_data(&fifo, &dev);
                bmp5_error_codes_print_result("bmp5_get_fifo_data", rslt);

                printf("Fifo data bytes available: %u\n", fifo.length);
                printf("Fifo frames available: %u\n", fifo.fifo_count);

                if (rslt == BMP5_OK)
                {
                    rslt = bmp5_extract_fifo_data(&fifo, sensor_data);
                    bmp5_error_codes_print_result("bmp5_extract_fifo_data", rslt);

                    if (rslt == BMP5_OK)
                    {
                        printf("\n%-10s %-20s\n", "Index", "Pressure (Pa)");
                        for (idx = 0; idx < fifo.fifo_count; idx++)
                        {
#ifdef BMP5_USE_FIXED_POINT
#if defined(MCU_I2C) || defined(MCU_SPI)
                            printf("%-10u %-20lu\n", idx, (unsigned long)sensor_data[idx].pressure);
#else
                            printf("%-10u %-20I64u\n", idx, sensor_data[idx].pressure);
#endif
#else
                            printf("%-10u %-20f\n", idx, sensor_data[idx].pressure);
#endif
                        }
                    }
                }

                loop++;
            }
        }

        bmp5_coines_deinit();

        return rslt;
    }
}
