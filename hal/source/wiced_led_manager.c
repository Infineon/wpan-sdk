/*
 * Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
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
/** @file
 *
 * This file provides implementation for the LED Manager library interface.
 * LED Manager library provides API's to enable/disable, blink and set brightness of a LED.
 */

#include <wiced_led_manager.h>

#include <cycfg_pins.h>
#include <platform_led.h>
#include <stdio.h>
#include <wiced_timer.h>

/******************************************************
 *                      Macros
 ******************************************************/
#define LED_FREQ (60) /*Hz*/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
static wiced_led_config_t * led_config_get(wiced_led_t led);
static void led_timer_function(uint32_t arg);

static wiced_led_config_t * led_config = NULL;
static size_t led_config_count         = 0;

wiced_result_t wiced_led_manager_init(wiced_led_config_t * config, size_t count)
{
    size_t i;

    if (config == NULL)
        return WICED_BADARG;

    if (led_config != NULL)
        return WICED_ERROR;

    for (i = 0; i < count; i++)
    {
        uint16_t bright = config->bright;

        if (bright > 99)
        {
            bright = 99;
        }

        if (WICED_SUCCESS != platform_led_init(&config[i].platform_config, LED_FREQ, bright, config[i].led))
        {
            printf("Err: platform_led_init failed for pin %d\n", config[i].led);
            continue;
        }

        /* initialize timer */
        wiced_init_timer(&config[i].timer, &led_timer_function, (uint32_t) &config[i], WICED_MILLI_SECONDS_PERIODIC_TIMER);
    }

    led_config       = config;
    led_config_count = count;

    return WICED_SUCCESS;
}

/**
 * Function to de-initialize the LED Manager
 *
 * @param  void        : No arguments.
 * @return             : result.
 */
wiced_result_t wiced_led_manager_deinit()
{
    size_t i;

    if (led_config == NULL)
        return WICED_SUCCESS;

    for (i = 0; i < led_config_count; i++)
    {
        platform_led_deinit(&led_config[i].platform_config);
        wiced_deinit_timer(&led_config[i].timer);
    }

    led_config       = NULL;
    led_config_count = 0;

    return WICED_SUCCESS;
}

/**
 * Enables the selected LED
 *
 * @param  led      : LED to be enabled.
 * @return          : result.
 */
wiced_result_t wiced_led_manager_enable_led(wiced_led_t led)
{
    wiced_led_config_t * config = led_config_get(led);
    if (config == NULL)
        return WICED_BADARG;

    return platform_led_start(&config->platform_config);
}

/**
 * Disables the selected LED
 *
 * @param  led      : LED to be disabled.
 * @return          : result.
 */
wiced_result_t wiced_led_manager_disable_led(wiced_led_t led)
{
    wiced_led_config_t * config = led_config_get(led);
    if (config == NULL)
        return WICED_BADARG;

    if (wiced_is_timer_in_use(&config->timer))
    {
        wiced_stop_timer(&config->timer);
    }

    return platform_led_stop(&config->platform_config);
}

wiced_result_t wiced_led_manager_reconfig_led(wiced_led_t led)
{
    uint16_t bright;

    wiced_led_config_t * config = led_config_get(led);
    if (config == NULL)
        return WICED_BADARG;

    bright = config->bright;

    if (bright > 99)
    {
        bright = 99;
    }

    return platform_led_reinit(&config->platform_config, LED_FREQ, bright);
}

wiced_led_config_t * led_config_get(wiced_led_t led)
{
    size_t i;

    for (i = 0; i < led_config_count; i++)
    {
        if (led_config[i].led == led)
        {
            return &led_config[i];
        }
    }

    return NULL;
}

/**
 * LED timer handler
 *
 * @param  arg            : arguments passed to the handler.
 * @return                : no return value expected.
 */
void led_timer_function(uint32_t arg)
{
    wiced_led_config_t * config = (wiced_led_config_t *) arg;

    if (config->led_state)
    {
        platform_led_stop(&config->platform_config);
        config->led_state = WICED_FALSE;
    }
    else
    {
        platform_led_start(&config->platform_config);
        config->led_state = WICED_TRUE;
    }

    wiced_stop_timer(&config->timer);
    wiced_start_timer(&config->timer, config->led_state ? config->on_period : config->off_period);
}

/**
 * Function called to blink a LED
 *
 * @param  led            : LED to be blinked.
 * @param  on_period      : on period (ms)
 * @param  off_period     : off period (ms)
 * @return                : result.
 */
wiced_result_t wiced_led_manager_blink_led(wiced_led_t led, uint32_t on_period, uint32_t off_period)
{
    wiced_led_config_t * config = led_config_get(led);
    wiced_result_t result;

    if (config == NULL)
        return WICED_BADARG;

    config->on_period  = on_period;
    config->off_period = off_period;

    result = platform_led_start(&config->platform_config);
    if (WICED_SUCCESS != result)
    {
        return result;
    }

    config->led_state = WICED_TRUE;
    wiced_start_timer(&config->timer, on_period);

    return WICED_SUCCESS;
}
