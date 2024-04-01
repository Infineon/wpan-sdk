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
#ifndef __WICED_HAL_PLATFORM_H__
#define __WICED_HAL_PLATFORM_H__

#include <wiced_bt_dev.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * \brief prototype for application thread event handler
 */
typedef void (wiced_platform_application_thread_event_handler)(void);

/**
 * \brief prototype for application thread specific handler - used for user application
 */
typedef void (wiced_platform_application_thread_specific_handler)(void);


/**
 *  \brief Initialize all the required pins and configure their functionality
 */
void wiced_platform_init(void);

/**
 * \brief Register the BT stack event handler.
 *
 * @param [in] user specific handler (callback)
 */
void wiced_platform_register_bt_management_callback(wiced_bt_management_cback_t *p_callback);

/**
 * \brief Register the HCI VSE event handler
 *
 * @param [in] HCI VSE event code
 * @param [in] target event code handler (callback)
 *
 * @return WICED_TRUE  : Success
 *         WICED_FALSE : Fail
 */
wiced_bool_t wiced_platform_register_hci_vse_callback(uint8_t evt_code, wiced_bt_dev_vse_callback_t *p_callback);

/**
 * \brief Helper function to check if current utility is executed under application thread.
 *
 * @return WICED_TRUE  : Current utility is executed under application thread.
 *         WICED_FALSE : Current utility is NOT executed under application thread.
 */
wiced_bool_t wiced_platform_application_thread_check(void);

/**
 * \brief Get application thread event code and the register the corresponding event handler if
 *        provided.
 *
 * @param [out]    allocated event code
 * @param [in]     user specified event handler
 *
 * @return WICED_TRUE  : Success
 *         WICED_FALSE : Fail.
 */
wiced_bool_t wiced_platform_application_thread_event_register(uint32_t *p_event_code,
        wiced_platform_application_thread_event_handler *p_event_handler);

/**
 * \brief Set an application thread event
 *
 * @param [in] event code (get by register utility)
 */
void wiced_platform_application_thread_event_set(uint32_t event_code);

/**
 * \brief Wait an application thread event
 *
 * @param [in] event code (get by register utility)
 */
void wiced_platform_application_thread_event_wait(uint32_t event_code);

/**
 * \brief Register a user application periodical handler under application thread.
 *        The register handler will be executed periodically (defined in
 *        WICED_PLATFORM_APPLICATION_THREAD_EVENT_WAIT_TIME) if the application thread
 *        is id idle state.
 *
 * @param p_handler - user application handler
 */
void wiced_platform_application_thread_specific_handler_register(wiced_platform_application_thread_specific_handler *p_handler);

/**
 * \brief This function waits and dispatches application events.
 */
void wiced_platform_application_thread_event_dispatch(void);

/**
 * \brief Initialize PUART
 *
 * @param[in] puart_rx_cbk         Call back function to process rx bytes.
 */
void wiced_platform_puart_init(void (*puart_rx_cbk)(void*));

/**
 * \brief Read and clear the specified GPIO interrupt status.
 *
 * @param[in] pin  GPIO pin id
 *
 * @retval 1  There was a pending interrupt from the specified pin.
 * @retval 0  There was not any pending interrupt from the specified pin.
 */
uint8_t wiced_hal_platform_gpio_int_status_get_and_clear(uint32_t pin);

/**
 * \brief This function retrieves random values from the platform-specific
 *        implementation and stores it in the provided buffer.
 *
 * @param[out] output         A pointer to the buffer where random values is stored.
 *                            Must not be NULL.
 * @param[in]  length         The size of the buffer, in bytes.
 * @param[out] output_length  The actual number of bytes stored.
 *
 * @retval WICED_SUCCESS  Successfully filled the buffer with random values.
 * @retval WICED_ERROR    Failed to fill the buffer with random values.
 * @retval WICED_BADARG   The buffer pointer was set to NULL.
 */
wiced_result_t wiced_hal_platform_random_get(uint8_t *output, size_t length, size_t *output_length);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __WICED_HAL_PLATFORM_H__ */
