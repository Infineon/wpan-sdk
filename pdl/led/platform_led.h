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
#pragma once

#include <stdbool.h>
#include <wiced_device_config.h>
#include <wiced_hal_pwm.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/
typedef struct
{
    const wiced_platform_pwm_config_t *platform_pwm_config;
    wiced_pwm_config_t pwm_config;
    bool is_pwm_enabled;
} platform_led_config_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/**
 * Function to initialize the platform LED
 *
 * @param  config      : Configurations for the platform LED.
 * @param  frequency   : Frequency of PWM to be generated.
 * @param  duty_cycle  : Duty cycle of the PWM in %. (1 to 100)
 * @param  gpio        : GPIO pin number for the LED.
 * @return             : result.
 */
wiced_result_t platform_led_init(platform_led_config_t* config, uint32_t frequency, uint32_t duty_cycle, wiced_bt_gpio_numbers_t gpio);

/**
 * Function to re-initialize the platform LED
 *
 * @param  config      : Configurations for the platform LED.
 * @param  frequency   : Frequency of PWM to be generated.
 * @param  duty_cycle  : Duty cycle of the PWM in %. (1 to 100)
 * @return             : result.
 */
wiced_result_t platform_led_reinit(platform_led_config_t *config, uint32_t frequency, uint32_t duty_cycle);

/**
 * Function to start the platform LED
 *
 * @param  config      : Configurations for the platform LED.
 * @return             : result.
 */
wiced_result_t platform_led_start(platform_led_config_t *config);

/**
 * Function to stop the platform LED
 *
 * @param  config      : Configurations for the platform LED.
 * @return             : result.
 */
wiced_result_t platform_led_stop(platform_led_config_t *config);

/**
 * Function to de-initialize the platform LED
 *
 * @param  config      : Configurations for the platform LED.
 * @return             : result.
 */
wiced_result_t platform_led_deinit(platform_led_config_t * config);

#ifdef __cplusplus
} /* extern "C" */
#endif
