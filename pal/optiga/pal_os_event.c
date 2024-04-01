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

#include "optiga/pal/pal_os_event.h"
#include "wiced.h"
#include "wiced_timer.h"

/* 1000us is recommended from optiga sample code */
#define FIRST_TRIGGERED_EVENT_DELAY 1000

static pal_os_event_t pal_os_event_0 = {0};
static wiced_timer_t timer;
static void trigger_registered_callback(WICED_TIMER_PARAM_TYPE cb_params);

void pal_os_event_start(pal_os_event_t * p_pal_os_event, register_callback callback, void * callback_args)
{
    if (0 == p_pal_os_event->is_event_triggered)
    {
        p_pal_os_event->is_event_triggered = TRUE;
        pal_os_event_register_callback_oneshot(p_pal_os_event, callback, callback_args, FIRST_TRIGGERED_EVENT_DELAY);
    }
}

void pal_os_event_stop(pal_os_event_t * p_pal_os_event)
{
    p_pal_os_event->is_event_triggered = 0;
}

pal_os_event_t * pal_os_event_create(register_callback callback, void * callback_args)
{
    wiced_init_timer(&timer, trigger_registered_callback, 0, WICED_MICRO_SECONDS_TIMER);
    if (( NULL != callback )&&( NULL != callback_args ))
    {
        pal_os_event_start(&pal_os_event_0, callback, callback_args);
    }
    return (&pal_os_event_0);
}

static void trigger_registered_callback(WICED_TIMER_PARAM_TYPE cb_params)
{
    register_callback callback;

    // !!!OPTIGA_LIB_PORTING_REQUIRED
    // User should take care to stop the timer if it sin't stoped automatically
    // IMPORTANT: Make sure you don't call this callback from the ISR.
    // It could work, but not recommended.
    wiced_stop_timer(&timer);
    if (pal_os_event_0.callback_registered)
    {
        callback = pal_os_event_0.callback_registered;
        callback((void * )pal_os_event_0.callback_ctx);
    }
}


void pal_os_event_register_callback_oneshot(pal_os_event_t * p_pal_os_event,
                                             register_callback callback,
                                             void * callback_args,
                                             uint32_t time_us)
{
    p_pal_os_event->callback_registered = callback;
    p_pal_os_event->callback_ctx = callback_args;
    // !!!OPTIGA_LIB_PORTING_REQUIRED
    // User should start the timer here with the
    // pal_os_event_trigger_registered_callback() function as a callback
    wiced_start_timer(&timer, time_us);
}

void pal_os_event_destroy(pal_os_event_t * pal_os_event)
{
    (void)pal_os_event;
    // User should take care to destroy the event if it's not required
}
