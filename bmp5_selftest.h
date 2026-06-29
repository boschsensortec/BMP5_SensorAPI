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
* @file       bmp5_selftest.h
* @date       2026-05-29
* @version    v1.5.0
*
*/

#ifndef BMP5_SELFTEST_H_
#define BMP5_SELFTEST_H_

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

#include "bmp5.h"

#ifdef BMP5_USE_FIXED_POINT

/* 0 degree celsius */
#define BMP5_MIN_TEMPERATURE                                (0)

/* 40 degree celsius */
#define BMP5_MAX_TEMPERATURE                                (40)

/* 900 hecto Pascals */
#define BMP5_MIN_PRESSURE                                   (900)

/* 1100 hecto Pascals */
#define BMP5_MAX_PRESSURE                                   (1100)

#else

/* 0 degree celsius */
#define BMP5_MIN_TEMPERATURE                                (0.0f)

/* 40 degree celsius */
#define BMP5_MAX_TEMPERATURE                                (40.0f)

/* 900 hecto Pascals */
#define BMP5_MIN_PRESSURE                                   (900.0f)

/* 1100 hecto Pascals */
#define BMP5_MAX_PRESSURE                                   (1100.0f)
#endif

/* Error codes for self test  */
#define BMP5_COMMUNICATION_ERROR_OR_WRONG_DEVICE            INT8_C(-11)
#define BMP5_TRIMMING_DATA_OUT_OF_BOUND                     INT8_C(-12)
#define BMP5_TEMPERATURE_BOUND_WIRE_FAILURE_OR_MEMS_DEFECT  INT8_C(-13)
#define BMP5_PRESSURE_BOUND_WIRE_FAILURE_OR_MEMS_DEFECT     INT8_C(-14)
#define BMP5_IMPLAUSIBLE_TEMPERATURE                        INT8_C(-15)
#define BMP5_IMPLAUSIBLE_PRESSURE                           INT8_C(-16)
#define BMP5_E_SELFTEST_TIMEOUT                             INT8_C(-17)

/* Polling interval applied when data-ready is not yet asserted (microseconds) */
#define BMP5_SELFTEST_POLL_PERIOD_US                        UINT16_C(1000)

/* Maximum number of 1 ms polling iterations before the loop is aborted (10 s total) */
#define BMP5_SELFTEST_TIMEOUT_MS                            UINT16_C(10000)

/**
 * \ingroup bmp5
 * \defgroup bmp5ApiSelftest Self test
 * @brief Perform self test of sensor
 */

/*!
 * \ingroup bmp5ApiSelftest
 * \page bmp5_api_bmp5_selftest_check bmp5_selftest_check
 * \code
 * int8_t bmp5_selftest_check(struct bmp5_dev *dev);
 * \endcode
 * @details Self-test API for the BMP5
 *
 * @param[in]   dev    : Structure instance of bmp5_dev
 *
 * @return Result of API execution status
 * @retval 0  -> Success
 * @retval <0 -> Error
 */
int8_t bmp5_selftest_check(struct bmp5_dev *dev);

/*! CPP guard */
#ifdef __cplusplus
}
#endif

#endif /* BMP5_SELFTEST_H_ */
