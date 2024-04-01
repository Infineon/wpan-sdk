/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

#include "cycfg_pins.h"
#include "optiga/pal/pal_gpio.h"
#include "optiga/pal/pal_i2c.h"
#include "optiga/ifx_i2c/ifx_i2c_config.h"

#define OPTIGA_SLAVE_ADDRESS 0x30

uint32_t pal_i2c_master_vcc_pin = PLATFORM_OPTIGA_VDD_CTRL;

// !!!OPTIGA_LIB_PORTING_REQUIRED
typedef struct locl_i2c_struct_to_descroibe_master
{
    // you parameters to control the master instance
    // See other implementation to get intuition on how to implement this part
}local_i2c_struct_to_descroibe_master_t;

local_i2c_struct_to_descroibe_master_t i2c_master_0;

/**
 * \brief PAL I2C configuration for OPTIGA.
 */
pal_i2c_t optiga_pal_i2c_context_0 =
{
    /// Pointer to I2C master platform specific context
    (void*)&i2c_master_0,
    /// Upper layer context
    NULL,
    /// Callback event handler
    NULL,
    /// Slave address
    OPTIGA_SLAVE_ADDRESS,
};

/**
* \brief PAL vdd pin configuration for OPTIGA.
 */
pal_gpio_t optiga_vdd_0 =
{
    // !!!OPTIGA_LIB_PORTING_REQUIRED
    // Platform specific GPIO context for the pin used to toggle Vdd.
    // You should have vdd_pin define in your system,
    // alternativly you can put here raw GPIO number, but without the & sign
    (void*)&pal_i2c_master_vcc_pin
};

/**
 * \brief PAL reset pin configuration for OPTIGA.
 */
pal_gpio_t optiga_reset_0 =
{
    // !!!OPTIGA_LIB_PORTING_REQUIRED
    // Platform specific GPIO context for the pin used to toggle Reset.
    // You should have reset_pin define in your system,
    // alternativly you can put here raw GPIO number, but without the & sign
    (void*)NULL
};
