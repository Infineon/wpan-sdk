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

#include <platform_led.h>

#include <cycfg_pins.h>
#include <wiced_hal_aclk.h>
#include <wiced_rtos.h>

/*
 * This will be the max PWM freq we can generate
 * we have set it to 256KHz now, can be configured as supported.
 */
#define PWM_BASE_CLK    ((uint32_t)256000)

static const wiced_platform_pwm_config_t * platform_pwm_config_get(wiced_bt_gpio_numbers_t gpio);

static wiced_mutex_t *pwm_aclk_mutex = NULL;
static uint32_t pwm_aclk_count;
static wiced_bool_t pwm_aclk_state = WICED_FALSE;

wiced_result_t platform_led_clk_init(void)
{
    wiced_result_t res = WICED_ERROR;

    if (pwm_aclk_mutex != NULL)
        return WICED_SUCCESS;

    pwm_aclk_mutex = wiced_rtos_create_mutex();
    if (pwm_aclk_mutex != NULL)
    {
        res = wiced_rtos_init_mutex(pwm_aclk_mutex);
        if (WICED_SUCCESS != res)
        {
            return WICED_NO_MEMORY;
        }
    }

    return res;
}

/*enable the ACLK, keep use count
 * PWM BASE CLK is the clk freq used by the PWM */
void pwm_clk_enable(void)
{
    wiced_rtos_lock_mutex(pwm_aclk_mutex);

    if (pwm_aclk_count == 0)
    {
        wiced_hal_aclk_enable(PWM_BASE_CLK, WICED_ACLK1, WICED_ACLK_FREQ_24_MHZ);
        pwm_aclk_state = WICED_TRUE;
    }
    pwm_aclk_count++;
    wiced_rtos_unlock_mutex(pwm_aclk_mutex);
}

/*Disable the PWM base clock*/
void pwm_clk_disable(void)
{
    wiced_rtos_lock_mutex(pwm_aclk_mutex);

    if (pwm_aclk_state == WICED_FALSE)
    {
        wiced_rtos_unlock_mutex(pwm_aclk_mutex);
        return;
    }
    pwm_aclk_count--;
    if (pwm_aclk_count == 0)
    {
        wiced_hal_aclk_disable(WICED_ACLK1);
        pwm_aclk_state = WICED_FALSE;
    }
    wiced_rtos_unlock_mutex(pwm_aclk_mutex);
}

wiced_result_t platform_led_init(platform_led_config_t* config, uint32_t frequency, uint32_t duty_cycle, wiced_bt_gpio_numbers_t gpio)
{
    config->platform_pwm_config = platform_pwm_config_get(gpio);
    if (config->platform_pwm_config == NULL)
    {
        return WICED_BADARG;
    }

    if (duty_cycle > 100)
        duty_cycle = 100;

    /*convert duty cycle and frequency to toggle and init vals*/
    wiced_hal_pwm_get_params(PWM_BASE_CLK, duty_cycle, frequency, &config->pwm_config);
    /*Init PWM base clock*/
    platform_led_clk_init();
    pwm_clk_enable();
    return WICED_SUCCESS;
}

wiced_result_t platform_led_reinit(platform_led_config_t *config, uint32_t frequency, uint32_t duty_cycle)
{
    if (duty_cycle > 100)
        duty_cycle = 100;

    /*convert duty cycle and frequency to toggle and init vals*/
    wiced_hal_pwm_get_params(PWM_BASE_CLK, duty_cycle, frequency, &config->pwm_config);

    /*if started lets update new set of values*/
    if (config->is_pwm_enabled)
        wiced_hal_pwm_change_values(config->platform_pwm_config->channel, config->pwm_config.toggle_count, config->pwm_config.init_count);

    return WICED_SUCCESS;
}

wiced_result_t platform_led_start(platform_led_config_t *config)
{
    if (config == NULL)
    {
        return WICED_ERROR;
    }

    if (!config->is_pwm_enabled)
    {
        wiced_hal_pwm_start(config->platform_pwm_config->channel, PMU_CLK, config->pwm_config.toggle_count, config->pwm_config.init_count, config->platform_pwm_config->invert);
        config->is_pwm_enabled = true;
    }
    else
    {
        wiced_hal_pwm_change_values(config->platform_pwm_config->channel, config->pwm_config.toggle_count, config->pwm_config.init_count);
    }
    return WICED_SUCCESS;
}

wiced_result_t platform_led_stop(platform_led_config_t *config)
{
    if (config == NULL)
    {
        return WICED_ERROR;
    }

    if (config->is_pwm_enabled)
    {
        wiced_pwm_config_t pwm_config;

        /*convert duty cycle and frequency to toggle and init vals*/
        wiced_hal_pwm_get_params(PWM_BASE_CLK, 0, 0, &pwm_config);
        wiced_hal_pwm_change_values(config->platform_pwm_config->channel, pwm_config.init_count, pwm_config.toggle_count);

        config->is_pwm_enabled = false;
    }
    return WICED_SUCCESS;
}

wiced_result_t platform_led_deinit(platform_led_config_t *config)
{
    if (config == NULL)
    {
        return WICED_ERROR;
    }

    wiced_hal_pwm_disable(config->platform_pwm_config->channel);
    pwm_clk_disable();
    config->is_pwm_enabled = false;

    return WICED_SUCCESS;
}

const wiced_platform_pwm_config_t * platform_pwm_config_get(wiced_bt_gpio_numbers_t gpio)
{
    size_t i;

    for (i = 0; i < pwm_count; i++)
    {
        if (platform_pwm[i].gpio->gpio_pin == gpio)
        {
            return &platform_pwm[i];
        }
    }

    return NULL;
}
