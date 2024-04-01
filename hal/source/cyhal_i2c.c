/*******************************************************************************
* File Name: cyhal_i2c.c
*
* Description:
* Provides a high level interface for interacting with the Cypress I2C. This is
* a wrapper around the lower level PDL API.
*
********************************************************************************
* \copyright
* Copyright 2018-2021 Cypress Semiconductor Corporation
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <stdlib.h>
#include <string.h>
#include "cyhal_i2c.h"
#include "wiced_hal_i2c.h"

#if defined(__cplusplus)
extern "C"
{
#endif

cy_rslt_t cyhal_i2c_master_write(cyhal_i2c_t *obj, uint16_t dev_addr, uint8_t *data, uint16_t size, uint32_t timeout, bool send_stop)
{
    uint8_t status;

    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(timeout);
    CY_UNUSED_PARAMETER(send_stop);

    status = wiced_hal_i2c_write(data, size, dev_addr);
    if (status)
        return CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_BOARD_HARDWARE_SSD1306, 0);

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cyhal_i2c_master_read(cyhal_i2c_t *obj, uint16_t dev_addr, uint8_t *data, uint16_t size, uint32_t timeout, bool send_stop)
{
    uint8_t status;

    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(timeout);
    CY_UNUSED_PARAMETER(send_stop);

    status = wiced_hal_i2c_read(data, size, dev_addr);
    if (status)
        return CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_BOARD_HARDWARE_SSD1306, 0);

    return CY_RSLT_SUCCESS;
}

#if defined(__cplusplus)
}
#endif
