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
 *  @brief This internal API is used to set configurations of the sensor.
 *
 *  @param[in,out] osr_odr_press_cfg : Structure instance of bmp5_osr_odr_press_config
 *  @param[in] dev                   : Structure instance of bmp5_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_config(struct bmp5_osr_odr_press_config *osr_odr_press_cfg, struct bmp5_dev *dev);

/*!
 *  @brief This internal API is used to get sensor data.
 *
 *  @param[in] osr_odr_press_cfg : Structure instance of bmp5_osr_odr_press_config
 *  @param[in] dev               : Structure instance of bmp5_dev.
 *
 *  @return Status of execution.
 */
static int8_t get_sensor_data(const struct bmp5_osr_odr_press_config *osr_odr_press_cfg, struct bmp5_dev *dev);

/******************************************************************************/
/*!            Functions                                        */

int main(void)
{
    /* Variable to store the result of API calls */
    int8_t rslt;

    /* Device structure to hold BMP5 sensor configuration and state */
    struct bmp5_dev dev;

    /* Configuration structure for oversampling, output data rate, and pressure settings */
    struct bmp5_osr_odr_press_config osr_odr_press_cfg = { 0 };

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
            rslt = set_config(&osr_odr_press_cfg, &dev);
            bmp5_error_codes_print_result("set_config", rslt);
        }

        if (rslt == BMP5_OK)
        {
            rslt = get_sensor_data(&osr_odr_press_cfg, &dev);
            bmp5_error_codes_print_result("get_sensor_data", rslt);
        }
    }

    bmp5_coines_deinit();

    return rslt;
}

static int8_t set_config(struct bmp5_osr_odr_press_config *osr_odr_press_cfg, struct bmp5_dev *dev)
{
    /* Variable to store the result of API calls */
    int8_t rslt;

    /* Structure to configure IIR filter settings */
    struct bmp5_iir_config set_iir_cfg;

    /* Structure to configure (OOR) pressure settings */
    struct bmp5_oor_press_configuration set_oor_press_config;

    /* Structure to configure interrupt source selection */
    struct bmp5_int_source_select int_source_select;

    rslt = bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, dev);
    bmp5_error_codes_print_result("bmp5_set_power_mode_standby_mode", rslt);

    if (rslt == BMP5_OK)
    {
        rslt = bmp5_get_osr_odr_press_config(osr_odr_press_cfg, dev);
        bmp5_error_codes_print_result("bmp5_get_osr_odr_press_config", rslt);

        if (rslt == BMP5_OK)
        {
            /* Enable pressure */
            osr_odr_press_cfg->press_en = BMP5_ENABLE;

            rslt = bmp5_set_osr_odr_press_config(osr_odr_press_cfg, dev);
            bmp5_error_codes_print_result("bmp5_set_osr_odr_press_config", rslt);
            printf("OSR/ODR configurations:\n");
            printf("osr_odr_press_cfg->press_en : %u\n", osr_odr_press_cfg->press_en);
        }

        if (rslt == BMP5_OK)
        {
            /* Set IIR for pressure */
            set_iir_cfg.set_iir_p = BMP5_IIR_FILTER_COEFF_1;

            rslt = bmp5_set_iir_config(&set_iir_cfg, dev);
            bmp5_error_codes_print_result("bmp5_set_iir_config", rslt);
            printf("IIR configurations:\n");
            printf("set_iir_p : %u\n\n", set_iir_cfg.set_iir_p);
        }

        if (rslt == BMP5_OK)
        {
            /* Setting oor threshold as 100000Pa */
            set_oor_press_config.oor_thr_p = 0x0186A0;

            /* Setting oor range as 25Pa */
            set_oor_press_config.oor_range_p = 0x19;

            set_oor_press_config.cnt_lim = BMP5_OOR_COUNT_LIMIT_3;
            set_oor_press_config.oor_sel_iir_p = BMP5_ENABLE;

            rslt = bmp5_set_oor_configuration(&set_oor_press_config, dev);
            bmp5_error_codes_print_result("bmp5_set_oor_configuration", rslt);
            printf("OOR configurations:\n");
            printf("oor_thr_p : %lu\n", (unsigned long)set_oor_press_config.oor_thr_p);
            printf("oor_range_p : %u\n", set_oor_press_config.oor_range_p);
            printf("cnt_lim : %u\n", set_oor_press_config.cnt_lim);
            printf("oor_sel_iir_p : %u\n\n", set_oor_press_config.oor_sel_iir_p);

        }

        if (rslt == BMP5_OK)
        {
            rslt = bmp5_configure_interrupt(BMP5_PULSED, BMP5_ACTIVE_HIGH, BMP5_INTR_PUSH_PULL, BMP5_INTR_ENABLE, dev);
            bmp5_error_codes_print_result("bmp5_configure_interrupt", rslt);

            if (rslt == BMP5_OK)
            {
                /* Note : Select INT_SOURCE after configuring interrupt */
                int_source_select.oor_press_en = BMP5_ENABLE;
                rslt = bmp5_int_source_select(&int_source_select, dev);
                bmp5_error_codes_print_result("bmp5_int_source_select", rslt);
            }
        }

        /* Set powermode as continuous */
        rslt = bmp5_set_power_mode(BMP5_POWERMODE_CONTINUOUS, dev);
        bmp5_error_codes_print_result("bmp5_set_power_mode_continuous_mode", rslt);
    }

    return rslt;
}

static int8_t get_sensor_data(const struct bmp5_osr_odr_press_config *osr_odr_press_cfg, struct bmp5_dev *dev)
{
    /* Variable to store the result of API calls */
    int8_t rslt = 0;

    /* Index variable for loop iteration */
    uint8_t idx = 0;

    /* Variable to store interrupt status */
    uint8_t int_status;

    /* Structure to hold sensor data */
    struct bmp5_sensor_data sensor_data;

    printf("\nOutput :\n\n");
    printf("%-10s %-15s\n", "Data", "Pressure (Pa)");

    while (idx < 50)
    {
        rslt = bmp5_get_interrupt_status(&int_status, dev);
        bmp5_error_codes_print_result("bmp5_get_interrupt_status", rslt);

        if (int_status & BMP5_INT_ASSERTED_PRESSURE_OOR)
        {
            rslt = bmp5_get_sensor_data(&sensor_data, osr_odr_press_cfg, dev);
            bmp5_error_codes_print_result("bmp5_get_sensor_data", rslt);

            if (rslt == BMP5_OK)
            {
#ifdef BMP5_USE_FIXED_POINT
                printf("%-10u %-15lu\n", idx, (long unsigned int)sensor_data.pressure);
#else
                printf("%-10u %-15f\n", idx, sensor_data.pressure);
#endif
                idx++;
            }
        }
    }

    return rslt;
}
