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
#include "optiga/pal/pal_i2c.h"
#include "wiced_hal_i2c.h"
#include "wiced_optiga.h"

#define PAL_I2C_MASTER_MAX_BITRATE  (400U)

static volatile uint32_t g_entry_count = 0;
static pal_i2c_t * gp_pal_i2c_current_ctx;

static pal_status_t pal_i2c_acquire(const void * p_i2c_context)
{
    // To avoid compiler errors/warnings. This context might be used by a target
    // system to implement a proper mutex handling
    (void)p_i2c_context;

    if (0 == g_entry_count)
    {
        g_entry_count++;
        if (1 == g_entry_count)
        {
            return PAL_STATUS_SUCCESS;
        }
    }
    return PAL_STATUS_FAILURE;
}

static void pal_i2c_release(const void * p_i2c_context)
{
    // To avoid compiler errors/warnings. This context might be used by a target
    // system to implement a proper mutex handling
    (void)p_i2c_context;

    g_entry_count = 0;
}

void invoke_upper_layer_callback (const pal_i2c_t * p_pal_i2c_ctx, optiga_lib_status_t event)
{
    upper_layer_callback_t upper_layer_handler;

    upper_layer_handler = (upper_layer_callback_t)p_pal_i2c_ctx->upper_layer_event_handler;

    upper_layer_handler(p_pal_i2c_ctx->p_upper_layer_ctx, event);

    //Release I2C Bus
    pal_i2c_release(p_pal_i2c_ctx->p_upper_layer_ctx);
}

// !!!OPTIGA_LIB_PORTING_REQUIRED
// The next 5 functions are required only in case you have interrupt based i2c implementation
void i2c_master_end_of_transmit_callback(void)
{
    invoke_upper_layer_callback(gp_pal_i2c_current_ctx, PAL_I2C_EVENT_SUCCESS);
}

void i2c_master_end_of_receive_callback(void)
{
    invoke_upper_layer_callback(gp_pal_i2c_current_ctx, PAL_I2C_EVENT_SUCCESS);
}

void i2c_master_error_detected_callback(void)
{
    invoke_upper_layer_callback(gp_pal_i2c_current_ctx, PAL_I2C_EVENT_ERROR);
}

void i2c_master_nack_received_callback(void)
{
    i2c_master_error_detected_callback();
}

void i2c_master_arbitration_lost_callback(void)
{
    i2c_master_error_detected_callback();
}

pal_status_t pal_i2c_init(const pal_i2c_t * p_i2c_context)
{
    UNUSED_VARIABLE(p_i2c_context);
    wiced_hal_i2c_select_pads(PLATFORM_I2C_1_SCL, PLATFORM_I2C_1_SDA);
    wiced_hal_i2c_init();
    return PAL_STATUS_SUCCESS;
}

pal_status_t pal_i2c_deinit(const pal_i2c_t * p_i2c_context)
{
    (void)p_i2c_context;
    return PAL_STATUS_SUCCESS;
}

pal_status_t pal_i2c_write(const pal_i2c_t * p_i2c_context, uint8_t * p_data, uint16_t length)
{
    pal_status_t status = PAL_STATUS_FAILURE;

    LOG_WICED_OPTIGA("pal_i2c_write: %u\n", length);
    LOG_WICED_OPTIGA_HEX(p_data, length);

    //Acquire the I2C bus before read/write
    if (PAL_STATUS_SUCCESS == pal_i2c_acquire(p_i2c_context))
    {
        gp_pal_i2c_current_ctx = (pal_i2c_t *)p_i2c_context;

        //Invoke the low level i2c master driver API to write to the bus
        // !!!OPTIGA_LIB_PORTING_REQUIRED
        if (wiced_hal_i2c_write(p_data, length, p_i2c_context->slave_address) == I2CM_SUCCESS)
        {
            // !!!OPTIGA_LIB_PORTING_REQUIRED
            /**
            * Infineon I2C Protocol is a polling based protocol, if foo_i2c_write will fail it will be reported to the
            * upper layers by calling
            * (p_i2c_context->upper_layer_event_handler))(p_i2c_context->p_upper_layer_ctx , PAL_I2C_EVENT_ERROR);
            * If the function foo_i2c_write() will succedd then two options are possible
            * 1. if foo_i2c_write() is interrupt based, then you need to configure interrupts in the function
            *    pal_i2c_init() so that on a succesfull transmit interrupt the callback i2c_master_end_of_transmit_callback(),
            *    in case of successfull receive i2c_master_end_of_receive_callback() callback
            *    in case of not acknowedged, arbitration lost, generic error i2c_master_nack_received_callback() or
            *    i2c_master_arbitration_lost_callback()
            * 2. If foo_i2c_write() is a blocking function which will return either ok or failure after transmitting data
            *    you can handle this case directly here and call
            *    invoke_upper_layer_callback(gp_pal_i2c_current_ctx, PAL_I2C_EVENT_SUCCESS);
            *
            */
            status = PAL_STATUS_SUCCESS;
            invoke_upper_layer_callback(gp_pal_i2c_current_ctx, PAL_I2C_EVENT_SUCCESS);
        }
        else
        {
            //If I2C Master fails to invoke the write operation, invoke upper layer event handler with error.
            ((upper_layer_callback_t)(p_i2c_context->upper_layer_event_handler))
                                                       (p_i2c_context->p_upper_layer_ctx , PAL_I2C_EVENT_ERROR);

            //Release I2C Bus
            pal_i2c_release((void * )p_i2c_context);
        }
    }
    else
    {
        status = PAL_STATUS_I2C_BUSY;
        ((upper_layer_callback_t)(p_i2c_context->upper_layer_event_handler))
                                                        (p_i2c_context->p_upper_layer_ctx , PAL_I2C_EVENT_BUSY);
    }

    return status;
}

pal_status_t pal_i2c_read(const pal_i2c_t * p_i2c_context, uint8_t * p_data, uint16_t length)
{
    pal_status_t status = PAL_STATUS_FAILURE;

    LOG_WICED_OPTIGA("pal_i2c_read: %u\n", length);

    //Acquire the I2C bus before read/write
    if (PAL_STATUS_SUCCESS == pal_i2c_acquire(p_i2c_context))
    {
        gp_pal_i2c_current_ctx = (pal_i2c_t *)p_i2c_context;

        //Invoke the low level i2c master driver API to read from the bus
        if (wiced_hal_i2c_read(p_data, length, p_i2c_context->slave_address) == I2CM_SUCCESS)
        {
            // !!!OPTIGA_LIB_PORTING_REQUIRED
            /**
            * Similar to the foo_i2c_write() case you can directly call
            * invoke_upper_layer_callback(gp_pal_i2c_current_ctx, PAL_I2C_EVENT_SUCCESS);
            * if you have blocking (non-interrupt) i2c calls
            */
            LOG_WICED_OPTIGA_HEX(p_data, length);
            status = PAL_STATUS_SUCCESS;
            invoke_upper_layer_callback(gp_pal_i2c_current_ctx, PAL_I2C_EVENT_SUCCESS);
        }
        else
        {
            //If I2C Master fails to invoke the read operation, invoke upper layer event handler with error.
            ((upper_layer_callback_t)(p_i2c_context->upper_layer_event_handler))
                                                       (p_i2c_context->p_upper_layer_ctx , PAL_I2C_EVENT_ERROR);

            //Release I2C Bus
            pal_i2c_release((void * )p_i2c_context);
        }
    }
    else
    {
        status = PAL_STATUS_I2C_BUSY;
        ((upper_layer_callback_t)(p_i2c_context->upper_layer_event_handler))
                                                        (p_i2c_context->p_upper_layer_ctx , PAL_I2C_EVENT_BUSY);
    }

    return status;
}

pal_status_t pal_i2c_set_bitrate(const pal_i2c_t * p_i2c_context, uint16_t bitrate)
{
    pal_status_t return_status = PAL_STATUS_FAILURE;
    optiga_lib_status_t event = PAL_I2C_EVENT_ERROR;

    //Acquire the I2C bus before setting the bitrate
    if (PAL_STATUS_SUCCESS == pal_i2c_acquire(p_i2c_context))
    {
        // If the user provided bitrate is greater than the I2C master hardware maximum supported value,
        // set the I2C master to its maximum supported value.
        if (bitrate > PAL_I2C_MASTER_MAX_BITRATE)
        {
            bitrate = PAL_I2C_MASTER_MAX_BITRATE;
        }
        // !!!OPTIGA_LIB_PORTING_REQUIRED
        // This function is NOT absolutely required for the correct working of the system, but it's recommended
        // to implement it, though
        wiced_hal_i2c_set_speed(I2CM_SPEED_100KHZ);
        return_status = PAL_STATUS_SUCCESS;
        event = PAL_I2C_EVENT_SUCCESS;
    }
    else
    {
        return_status = PAL_STATUS_I2C_BUSY;
        event = PAL_I2C_EVENT_BUSY;
    }
    if (0 != p_i2c_context->upper_layer_event_handler)
    {
        ((callback_handler_t)(p_i2c_context->upper_layer_event_handler))(p_i2c_context->p_upper_layer_ctx , event);
    }
    //Release I2C Bus if its acquired
    if (PAL_STATUS_I2C_BUSY != return_status)
    {
        pal_i2c_release((void * )p_i2c_context);
    }
    return return_status;
}
