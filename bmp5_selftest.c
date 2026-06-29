/**
* Copyright (c) 2026 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       bmp5_selftest.c
* @date       2026-05-29
* @version    v1.5.0
*
*/

#include "bmp5_selftest.h"

/******************************************************************************/
/*!         Global Declaration                                                */

#define SAMPLE_COUNT  UINT8_C(100)

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
 *  @brief This internal API is used to analyze the sensor temperature and pressure data.
 *
 *  @param[in] data : Structure holding sensor data.
 *
 *  @return Status of execution.
 */
static int8_t analyze_sensor_data(const struct bmp5_sensor_data *data);

/*!
 *  @brief This internal API is used to get sensor data.
 *
 *  @param[in] data : Structure holding sensor data.
 *  @param[in] osr_odr_press_cfg : Structure instance of bmp5_osr_odr_press_config
 *  @param[in] dev               : Structure instance of bmp5_dev.
 *
 *  @return Status of execution.
 */
static int8_t get_sensor_data(struct bmp5_sensor_data *data,
                              const struct bmp5_osr_odr_press_config *osr_odr_press_cfg,
                              struct bmp5_dev *dev);

/*!
 * @brief This API is used to do self test.
 */
int8_t bmp5_selftest_check(struct bmp5_dev *dev)
{
    int8_t rslt;
    struct bmp5_sensor_data sensor_data[SAMPLE_COUNT] = { { 0 } };
    struct bmp5_osr_odr_press_config osr_odr_press_cfg = { 0 };

    /* Initialize the BMP5 dev instance */
    rslt = bmp5_init(dev);

    if (rslt == BMP5_E_COM_FAIL || rslt == BMP5_E_DEV_NOT_FOUND)
    {
        rslt = BMP5_COMMUNICATION_ERROR_OR_WRONG_DEVICE;
    }

    if (rslt == BMP5_OK)
    {
        /* sensor configuration*/
        rslt = set_config(&osr_odr_press_cfg, dev);
    }

    if (rslt == BMP5_OK)
    {
        /* getting data from sensor and store it to struct bmp5_sensor_data */
        rslt = get_sensor_data(sensor_data, &osr_odr_press_cfg, dev);
    }

    if (rslt == BMP5_OK)
    {
        /* bmp5 temperature and pressure validation*/
        rslt = analyze_sensor_data(sensor_data);
    }

    if (rslt == BMP5_OK)
    {
        /* Bring device to stand-by mode */
        rslt = bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, dev);
    }

    return rslt;

}

/*!
 * @brief This API is used to analyze the data read during the Seft Test procedure.
 */
static int8_t analyze_sensor_data(const struct bmp5_sensor_data *data)
{

#ifdef BMP5_USE_FIXED_POINT
    int64_t avg_p = 0, avg_t = 0;
#else
    float avg_p = 0.0, avg_t = 0.0;
#endif

    int8_t rslt = BMP5_OK;
    uint8_t i;

    for (i = 0; i < SAMPLE_COUNT; i++)
    {
        avg_p = avg_p + data[i].pressure;
        avg_t = avg_t + data[i].temperature;
    }

#ifdef BMP5_USE_FIXED_POINT
    avg_t = div_s64(avg_t, SAMPLE_COUNT);
    if ((avg_t < BMP5_MIN_TEMPERATURE) || (avg_t > BMP5_MAX_TEMPERATURE))
    {
        rslt = BMP5_IMPLAUSIBLE_TEMPERATURE;
    }

    avg_p = div_u64(div_u64(avg_p, 100), SAMPLE_COUNT);

    if ((avg_p < BMP5_MIN_PRESSURE) || (avg_p > BMP5_MAX_PRESSURE))
    {
        rslt = BMP5_IMPLAUSIBLE_PRESSURE;
    }

#else
    avg_t = avg_t / SAMPLE_COUNT;
    if ((avg_t < BMP5_MIN_TEMPERATURE) || (avg_t > BMP5_MAX_TEMPERATURE))
    {
        rslt = BMP5_IMPLAUSIBLE_TEMPERATURE;
    }

    avg_p = (avg_p / 100.0f) / SAMPLE_COUNT;

    if ((avg_p < BMP5_MIN_PRESSURE) || (avg_p > BMP5_MAX_PRESSURE))
    {
        rslt = BMP5_IMPLAUSIBLE_PRESSURE;
    }

#endif

    return rslt;
}

/*!
 * @brief This API is used to set sensor configuration for Pressue and Temperature values.
 */
static int8_t set_config(struct bmp5_osr_odr_press_config *osr_odr_press_cfg, struct bmp5_dev *dev)
{
    int8_t rslt;
    struct bmp5_iir_config set_iir_cfg;
    struct bmp5_int_source_select int_source_select;

    rslt = bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, dev);

    if (rslt == BMP5_OK)
    {
        /* Get default odr */
        rslt = bmp5_get_osr_odr_press_config(osr_odr_press_cfg, dev);

        if (rslt == BMP5_OK)
        {
            /* Set ODR as 50Hz */
            osr_odr_press_cfg->odr = BMP5_ODR_50_HZ;

            /* Enable pressure */
            osr_odr_press_cfg->press_en = BMP5_ENABLE;

            /* Set Over-sampling rate with respect to odr */
            osr_odr_press_cfg->osr_t = BMP5_OVERSAMPLING_64X;
            osr_odr_press_cfg->osr_p = BMP5_OVERSAMPLING_4X;

            rslt = bmp5_set_osr_odr_press_config(osr_odr_press_cfg, dev);
        }

        if (rslt == BMP5_OK)
        {
            /* Set IIR Configuration */
            set_iir_cfg.set_iir_t = BMP5_IIR_FILTER_COEFF_1;
            set_iir_cfg.set_iir_p = BMP5_IIR_FILTER_COEFF_1;
            set_iir_cfg.shdw_set_iir_t = BMP5_ENABLE;
            set_iir_cfg.shdw_set_iir_p = BMP5_ENABLE;

            rslt = bmp5_set_iir_config(&set_iir_cfg, dev);
        }

        if (rslt == BMP5_OK)
        {
            /* Configure interrupt */
            rslt = bmp5_configure_interrupt(BMP5_PULSED, BMP5_ACTIVE_HIGH, BMP5_INTR_PUSH_PULL, BMP5_INTR_ENABLE, dev);

            if (rslt == BMP5_OK)
            {
                /* Note : Select INT_SOURCE after configuring interrupt */
                int_source_select.drdy_en = BMP5_ENABLE;
                rslt = bmp5_int_source_select(&int_source_select, dev);
            }
        }

        if (rslt == BMP5_OK)
        {
            /* Set powermode as normal */
            rslt = bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This API is used to get sensor data
 */
static int8_t get_sensor_data(struct bmp5_sensor_data *data,
                              const struct bmp5_osr_odr_press_config *osr_odr_press_cfg,
                              struct bmp5_dev *dev)
{
    int8_t rslt = 0;
    uint8_t idx = 0;
    uint8_t int_status;
    uint16_t try_count = 0;

    while (idx < SAMPLE_COUNT)
    {
        rslt = bmp5_get_interrupt_status(&int_status, dev);

        if (rslt == BMP5_OK)
        {
            if (int_status & BMP5_INT_ASSERTED_DRDY)
            {
                rslt = bmp5_get_sensor_data(&data[idx], osr_odr_press_cfg, dev);
                idx++;
            }
            else
            {
                /* Data not ready yet (or status read failed): wait 1 ms and
                * check whether the overall timeout has been exceeded. */
                dev->delay_us(BMP5_SELFTEST_POLL_PERIOD_US, dev->intf_ptr);
                try_count++;
                if (try_count >= BMP5_SELFTEST_TIMEOUT_MS)
                {
                    rslt = BMP5_E_SELFTEST_TIMEOUT;
                    break;
                }
            }
        }
    }

    return rslt;
}
