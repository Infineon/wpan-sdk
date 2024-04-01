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

/*
 * @file
 * Header for WICED OPTIGA API
 */

#ifndef _WICED_OPTIGA_H_
#define _WICED_OPTIGA_H_

#ifdef __cplusplus
extern "C" {
#endif

//==================================================================================================
// Include
//==================================================================================================
#include <stdint.h>
#include "wiced.h"
#include "wiced_result.h"

//==================================================================================================
// Constants
//==================================================================================================

//==================================================================================================
// Type Definitions
//==================================================================================================
//#define WICED_OPTIGA_LOGGING

#if defined (WICED_OPTIGA_LOGGING)
#define LOG_WICED_OPTIGA(...)  printf(__VA_ARGS__);
#define LOG_WICED_OPTIGA_HEX(ptr, len) \
{ \
    uint16_t i; \
    for(i=0; i<len; i++) \
        printf("%x ", ptr[i]); \
    printf("\n"); \
}
#define LOG_WICED_OPTIGA_PERFORMANCE(time_taken, return_value) \
{ \
    char performance_buffer_string[30]; \
    if(OPTIGA_LIB_SUCCESS == return_value) \
    { \
        sprintf(performance_buffer_string, "takes %d usec, ret %x", (int)time_taken, return_value); \
        printf("%s\n", performance_buffer_string); \
    } \
}
#else
#define LOG_WICED_OPTIGA(...)
#define LOG_WICED_OPTIGA_HEX(ptr, len)
#define LOG_WICED_OPTIGA_PERFORMANCE(time_taken, return_value)
#endif

/* object id */
#define OPTIGA_OBJECT_ID_GLOBAL_LIFE_CYCLE_STATUS 0xE0C0
#define OPTIGA_OBJECT_ID_GLOBAL_SECURITY_STATUS 0xE0C1
#define OPTIGA_OBJECT_ID_COPROCESSOR_UID 0xE0C2
#define OPTIGA_OBJECT_ID_SLEEP_MODE_ACITVATE_DELAY 0xE0C3
#define OPTIGA_OBJECT_ID_CURRENT_LIMIT 0xE0C4
#define OPTIGA_OBJECT_ID_SECURITY_EVENT_COUNTER 0xE0C5
#define OPTIGA_OBJECT_ID_MAX_COM_BUF_SIZE 0xE0C6
#define OPTIGA_OBJECT_ID_SECURITY_MONITOR_CONFIG 0xE0C9
#define OPTIGA_OBJECT_ID_PUBLIC_KEY_CERT1_INF 0xE0E0
#define OPTIGA_OBJECT_ID_PUBLIC_KEY_CERT2 0xE0E1
#define OPTIGA_OBJECT_ID_PUBLIC_KEY_CERT3 0xE0E2
#define OPTIGA_OBJECT_ID_PUBLIC_KEY_CERT4 0xE0E3
#define OPTIGA_OBJECT_ID_ROOT_CA_PUBLIC_KEY_CERT1 0xE0E8
#define OPTIGA_OBJECT_ID_ROOT_CA_PUBLIC_KEY_CERT2 0xE0E9
#define OPTIGA_OBJECT_ID_ROOT_CA_PUBLIC_KEY_CERT8 0xE0EF
#define OPTIGA_OBJECT_ID_MONOTONIC_COUNTER1 0xE120
#define OPTIGA_OBJECT_ID_MONOTONIC_COUNTER2 0xE121
#define OPTIGA_OBJECT_ID_MONOTONIC_COUNTER3 0xE122
#define OPTIGA_OBJECT_ID_MONOTONIC_COUNTER4 0xE123
#define OPTIGA_OBJECT_ID_BINDING_PRESHARE_SECRET 0xE140

#define OPTIGA_OBJECT_ID_DEVICE_PRIVATE_ECC_KEY1 0xE0F0
#define OPTIGA_OBJECT_ID_DEVICE_PRIVATE_ECC_KEY2 0xE0F1
#define OPTIGA_OBJECT_ID_DEVICE_PRIVATE_ECC_KEY3 0xE0F2
#define OPTIGA_OBJECT_ID_DEVICE_PRIVATE_ECC_KEY4 0xE0F3
#define OPTIGA_OBJECT_ID_DEVICE_PRIVATE_RSA_KEY1 0xE0FC
#define OPTIGA_OBJECT_ID_DEVICE_PRIVATE_RSA_KEY2 0xE0FD
#define OPTIGA_OBJECT_ID_SESSION_CONTEXT1 0xE100
#define OPTIGA_OBJECT_ID_SESSION_CONTEXT2 0xE101
#define OPTIGA_OBJECT_ID_SESSION_CONTEXT3 0xE102
#define OPTIGA_OBJECT_ID_SESSION_CONTEXT4 0xE103
#define OPTIGA_OBJECT_ID_DEVICE_SYMMETRIC_KEY1 0xE200

/* Object DATA and METADATA */
#define OPTIGA_OBJECT_DATA 0
#define OPTIGA_OBJECT_METADATA 1

//==================================================================================================
// Functions
//==================================================================================================
/*
 * Function         wiced_optiga_init
 *
 * Initialize Optiga utility and crypt context. Run OpenApplication command.
 */
void wiced_optiga_init(void);

/*
 * Function         wiced_optiga_deinit
 *
 * Run CloseApplication command. Free Optiga utility and crypt context.
 */
void wiced_optiga_deinit(void);

/*
 * Function         wiced_optiga_read_data
 *
 * @param[in]       id: Optiga object id
 *
 * @param[in]       data_type: OPTIGA_OBJECT_DATA for read data; OPTIGA_OBJECT_METADATA for read metadata
 *
 * @param[in]       data_len: read length
 *
 * @param[out]      p_data: data buffer of an output holding read back data

 * @param[out]      p_status: output status of the read. WICED_SUCCESS or WICED_ERROR
 *
 * @return          the number of bytes read
 */
uint16_t wiced_optiga_read_data(uint16_t id, uint8_t data_type, uint16_t data_len, uint8_t *p_data, wiced_result_t *p_status);

/*
 * Function         wiced_optiga_write_data
 *
 * @param[in]       id: Optiga object id
 *
 * @param[in]       data_type: OPTIGA_OBJECT_DATA for read data; OPTIGA_OBJECT_METADATA for read metadata
 *
 * @param[in]       data_len: write length
 *
 * @param[out]      p_data: data buffer of the data to be written

 * @param[out]      p_status: output status of the read. WICED_SUCCESS or WICED_ERROR
 *
 * @return          the number of bytes write
 */
void wiced_optiga_write_data(uint16_t id, uint8_t data_type, uint16_t data_len, uint8_t *p_data, wiced_result_t *p_status);

#if defined(__cplusplus)
}
#endif

#endif //_WICED_OPTIGA_H_
