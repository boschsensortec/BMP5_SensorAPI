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
/*!         Static Function Declaration                                       */

/*!
 *  @brief This internal API is used to set fifo configurations of the sensor.
 *
 *  @param[in,out] fifo : Structure instance of bmp5_fifo.
 *  @param[in] dev      : Structure instance of bmp5_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_config(struct bmp5_fifo *fifo, struct bmp5_dev *dev);

/*!
 *  @brief This internal API is used to get sensor fifo data.
 *
 *  @param[in] fifo : Structure instance of bmp5_fifo.
 *  @param[in] dev  : Structure instance of bmp5_dev.
 *
 *  @return Status of execution.
 */
static int8_t get_fifo_data(struct bmp5_fifo *fifo, struct bmp5_dev *dev);

/******************************************************************************/
/*!            Macros                                        */

#define LOOP_COUNT                  UINT8_C(20)
#define BMP5_FIFO_DATA_BUFFER_SIZE  UINT8_C(96)
#define BMP5_FIFO_DATA_USER_LENGTH  UINT8_C(96)
#define BMP5_FIFO_P_T_FRAME_COUNT   UINT8_C(16)

/*#define BMP5_USE_FIXED_POINT */

/******************************************************************************/
/*!            Functions                                        */

int main(void)
{
    /* Variable to store the result of API calls */
    int8_t rslt;

    /* Device structure to hold sensor configuration and interface details */
    struct bmp5_dev dev;

    /* FIFO structure to hold FIFO configuration and data */
    struct bmp5_fifo fifo;

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

        if (rslt == BMP5_OK)
        {
            rslt = set_config(&fifo, &dev);
            bmp5_error_codes_print_result("set_config", rslt);
        }

        if (rslt == BMP5_OK)
        {
            rslt = get_fifo_data(&fifo, &dev);
            bmp5_error_codes_print_result("get_fifo_data", rslt);
        }
    }

    bmp5_coines_deinit();

    return rslt;
}

static int8_t set_config(struct bmp5_fifo *fifo, struct bmp5_dev *dev)
{
    /* Variable to store the result of API calls */
    int8_t rslt;

    /* Structure to hold IIR filter configuration */
    struct bmp5_iir_config set_iir_cfg;

    /* Structure to hold oversampling and ODR configuration for pressure */
    struct bmp5_osr_odr_press_config osr_odr_press_cfg;

    /* Structure to hold interrupt source selection configuration */
    struct bmp5_int_source_select int_source_select;

    rslt = bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, dev);
    bmp5_error_codes_print_result("bmp5_set_power_mode1", rslt);

    if (rslt == BMP5_OK)
    {
        /* Get default odr */
        rslt = bmp5_get_osr_odr_press_config(&osr_odr_press_cfg, dev);
        bmp5_error_codes_print_result("bmp5_get_osr_odr_press_config", rslt);

        if (rslt == BMP5_OK)
        {
            /* Set ODR as 50Hz */
            osr_odr_press_cfg.odr = BMP5_ODR_50_HZ;

            /* Enable pressure */

            /* NOTE: If temperature data only is required then pressure enable(press_en) is not required
             * or can be disabled. */
            osr_odr_press_cfg.press_en = BMP5_ENABLE;

            /* Set Over-sampling rate with respect to odr */
            osr_odr_press_cfg.osr_t = BMP5_OVERSAMPLING_8X;
            osr_odr_press_cfg.osr_p = BMP5_OVERSAMPLING_64X;

            rslt = bmp5_set_osr_odr_press_config(&osr_odr_press_cfg, dev);
            bmp5_error_codes_print_result("bmp5_set_osr_odr_press_config", rslt);
            printf("OSR ODR configurations\n");
            printf("OSR T : %u\n", osr_odr_press_cfg.osr_t);
            printf("OSR P : %u\n", osr_odr_press_cfg.osr_p);
            printf("ODR   : %u\n", osr_odr_press_cfg.odr);
            printf("Press Enable : %u\n\n", osr_odr_press_cfg.press_en);

            if (rslt == BMP5_OK)
            {
                set_iir_cfg.set_iir_t = BMP5_IIR_FILTER_COEFF_1;
                set_iir_cfg.set_iir_p = BMP5_IIR_FILTER_COEFF_1;

                rslt = bmp5_set_iir_config(&set_iir_cfg, dev);
                bmp5_error_codes_print_result("bmp5_set_iir_config", rslt);
                printf("IIR configurations\n\n");
                printf("IIR T : %u\n", set_iir_cfg.set_iir_t);
                printf("IIR P : %u\n\n", set_iir_cfg.set_iir_p);

            }
        }

        if (rslt == BMP5_OK)
        {
            rslt = bmp5_get_fifo_configuration(fifo, dev);
            bmp5_error_codes_print_result("bmp5_get_fifo_configuration", rslt);

            if (rslt == BMP5_OK)
            {
                fifo->mode = BMP5_FIFO_MODE_STREAMING;
                fifo->frame_sel = BMP5_FIFO_PRESS_TEMP_DATA;

                fifo->dec_sel = BMP5_FIFO_NO_DOWNSAMPLING;
                fifo->set_fifo_iir_t = BMP5_ENABLE;
                fifo->set_fifo_iir_p = BMP5_ENABLE;

                rslt = bmp5_set_fifo_configuration(fifo, dev);
                bmp5_error_codes_print_result("bmp5_set_fifo_configuration", rslt);
                printf("FIFO configurations\n\n");
                printf("FIFO Mode : %u\n", fifo->mode);
                printf("FIFO Frame Selection : %u\n", fifo->frame_sel);
                printf("FIFO Decimation Selection : %u\n", fifo->dec_sel);
                printf("FIFO IIR T : %u\n", fifo->set_fifo_iir_t);
                printf("FIFO IIR P : %u\n\n", fifo->set_fifo_iir_p);

            }
        }

        if (rslt == BMP5_OK)
        {
            rslt = bmp5_configure_interrupt(BMP5_PULSED, BMP5_ACTIVE_HIGH, BMP5_INTR_PUSH_PULL, BMP5_INTR_ENABLE, dev);
            bmp5_error_codes_print_result("bmp5_configure_interrupt", rslt);

            if (rslt == BMP5_OK)
            {
                /* Note : Select INT_SOURCE after configuring interrupt */
                int_source_select.fifo_full_en = BMP5_ENABLE;
                rslt = bmp5_int_source_select(&int_source_select, dev);
                bmp5_error_codes_print_result("bmp5_int_source_select", rslt);
            }
        }

        /*
         * FIFO example can be executed on,
         * normal mode - BMP5_POWERMODE_NORMAL
         * continuous mode - BMP5_POWERMODE_CONTINUOUS
         * Here, used normal mode (BMP5_POWERMODE_NORMAL)
         */
        rslt = bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, dev);
        bmp5_error_codes_print_result("bmp5_set_power_mode", rslt);
    }

    return rslt;
}

static int8_t get_fifo_data(struct bmp5_fifo *fifo, struct bmp5_dev *dev)
{
    /* Variable to store the result of API calls */
    int8_t rslt = 0;

    /* Index variable for iterating through sensor data */
    uint8_t idx = 0;

    /* Loop counter for the number of iterations */
    uint8_t loop = 0;

    /* Variable to store interrupt status */
    uint8_t int_status;

    /* Buffer to hold FIFO data */
    uint8_t fifo_buffer[BMP5_FIFO_DATA_BUFFER_SIZE];

    /* Array to hold extracted sensor data from FIFO */
    struct bmp5_sensor_data sensor_data[BMP5_FIFO_P_T_FRAME_COUNT] = { { 0 } };

    printf("\nOutput :\n");

    while (loop < LOOP_COUNT)
    {
        rslt = bmp5_get_interrupt_status(&int_status, dev);
        bmp5_error_codes_print_result("bmp5_get_interrupt_status", rslt);

        if (int_status & BMP5_INT_ASSERTED_FIFO_FULL)
        {
            fifo->length = BMP5_FIFO_DATA_USER_LENGTH;
            fifo->data = fifo_buffer;

            printf("\nIteration: %d\n", loop);
            printf("Each fifo frame contains 6 bytes of data\n");
            printf("Fifo data bytes requested: %u\n", fifo->length);

            rslt = bmp5_get_fifo_data(fifo, dev);
            bmp5_error_codes_print_result("bmp5_get_fifo_data", rslt);

            printf("Fifo data bytes available: %u\n", fifo->length);
            printf("Fifo frames available: %u\n", fifo->fifo_count);

            if (rslt == BMP5_OK)
            {
                rslt = bmp5_extract_fifo_data(fifo, sensor_data);
                bmp5_error_codes_print_result("bmp5_extract_fifo_data", rslt);

                if (rslt == BMP5_OK)
                {
                    printf("\n%-10s %-20s %-20s\n", "Index", "Pressure (Pa)", "Temperature (deg C)");
                    printf("-------------------------------------------------------------\n");

                    for (idx = 0; idx < fifo->fifo_count; idx++)
                    {
#ifdef BMP5_USE_FIXED_POINT
                        printf("%-10u %-20lu %-20ld\n",
                               idx,
                               (long unsigned int)sensor_data[idx].pressure,
                               (long int)sensor_data[idx].temperature);
#else
                        printf("%-10u %-20f %-20f\n", idx, sensor_data[idx].pressure, sensor_data[idx].temperature);
#endif
                    }
                }

                loop++;
            }
        }
    }

    return rslt;
}
