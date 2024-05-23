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
        wiced_optiga.c
        Wrapper functions for upper layer operating OPTIGA.
*/

#include "optiga/common/optiga_lib_logger.h"
#include "optiga/optiga_util.h"
#include "optiga/optiga_crypt.h"
#include "optiga/pal/pal.h"
#include "optiga/pal/pal_os_datastore.h"
#include "optiga/pal/pal_os_timer.h"
#include "wiced_optiga.h"

// Check return status
#define WAIT_AND_CHECK_STATUS(return_status, optiga_lib_status)\
                            if (OPTIGA_LIB_SUCCESS != return_status)\
                            {\
                            }\
                            else\
                            {\
                                while (OPTIGA_LIB_BUSY == optiga_lib_status)\
                                { } \
                                if (OPTIGA_LIB_SUCCESS != optiga_lib_status)\
                                {\
                                    return_status = optiga_lib_status;\
                                }\
                            }

//Start timer for performance measurement
#define START_PERFORMANCE_MEASUREMENT(time_taken) time_taken = pal_os_timer_get_time_in_milliseconds();

//Stop timer and calculate performance measurement
#define READ_PERFORMANCE_MEASUREMENT(time_taken) time_taken = pal_os_timer_get_time_in_milliseconds() - time_taken;

// Value of Operational state
#define LCSO_STATE_CREATION       (0x01)
// Value of Operational state
#define LCSO_STATE_OPERATIONAL    (0x07)

//Currently set to Creation state(defualt value). At the real time/customer side this needs to be LCSO_STATE_OPERATIONAL (0x07)
#define FINAL_LCSO_STATE          (LCSO_STATE_CREATION)

//#define WICED_OPTIGA_PAIR_HOST    1

/* Platform Binding Shared Secret (0xE140) Metadata to be updated */
const uint8_t platform_binding_shared_secret_metadata_final [] = {
    //Metadata to be updated
    0x20, 0x17,
    // LcsO
    0xC0, 0x01,
    FINAL_LCSO_STATE,       // Refer Macro to see the value or some more notes
    // Change/Write Access tag
    0xD0, 0x07,
    // This allows updating the binding secret during the runtime using shielded connection
    // If not required to update the secret over the runtime, set this to NEV and
    // update Metadata length accordingly
    0xE1, 0xFC, LCSO_STATE_OPERATIONAL,   // LcsO < Operational state
    0xFE,
    0x20, 0xE1, 0x40,
    // Read Access tag
    0xD1, 0x03,
    0xE1, 0xFC, LCSO_STATE_OPERATIONAL,   // LcsO < Operational state
    // Execute Access tag
    0xD3, 0x01,
    0x00,   // Always
    // Data object Type
    0xE8, 0x01,
    0x22,   // Platform binding secret type
};

static volatile optiga_lib_status_t  optiga_lib_status;
optiga_util_t                       *me_util_instance  = NULL;
optiga_crypt_t                      *me_crypt_instance = NULL;
#ifdef WICED_OPTIGA_PAIR_HOST
static uint8_t host_optiga_pairing_completed = FALSE;
#endif

static void optiga_lib_callback(void * context, optiga_lib_status_t return_status)
{
    optiga_lib_status = return_status;
    if (NULL != context)
    {
        // callback to upper layer here
    }
}

#ifdef WICED_OPTIGA_PAIR_HOST
static optiga_lib_status_t pair_host_and_optiga_using_pre_shared_secret(void)
{
    uint16_t bytes_to_read;
    uint8_t platform_binding_secret[64];
    uint8_t platform_binding_secret_metadata[44];
    optiga_lib_status_t return_status = !OPTIGA_LIB_SUCCESS;
    uint32_t time_taken_for_pairing = 0;
    pal_status_t pal_return_status;

    do
    {
        /**
         * 1. Check OPTIGA Util and Crypt Instances created.
         */
        if ((NULL == me_util_instance) || (NULL == me_crypt_instance))
        {
            break;
        }

        /**
         * 2. Initialize the protection level and protocol version for the instances
         */
        LOG_WICED_OPTIGA("2. Initialize the protection level and protocol \n");
        OPTIGA_UTIL_SET_COMMS_PROTECTION_LEVEL(me_util_instance, OPTIGA_COMMS_NO_PROTECTION);
        OPTIGA_UTIL_SET_COMMS_PROTOCOL_VERSION(me_util_instance, OPTIGA_COMMS_PROTOCOL_VERSION_PRE_SHARED_SECRET);

        OPTIGA_CRYPT_SET_COMMS_PROTECTION_LEVEL(me_crypt_instance, OPTIGA_COMMS_NO_PROTECTION);
        OPTIGA_CRYPT_SET_COMMS_PROTOCOL_VERSION(me_crypt_instance, OPTIGA_COMMS_PROTOCOL_VERSION_PRE_SHARED_SECRET);

        /**
         * 3. Read Platform Binding Shared secret (0xE140) data object metadata from OPTIGA
         *    using optiga_util_read_metadata.
         */
        LOG_WICED_OPTIGA("3. Read Platform Binding Shared secret (0xE140)\n");
        bytes_to_read = sizeof(platform_binding_secret_metadata);
        optiga_lib_status = OPTIGA_LIB_BUSY;

        START_PERFORMANCE_MEASUREMENT(time_taken_for_pairing);

        return_status = optiga_util_read_metadata(me_util_instance,
                                                  OPTIGA_OBJECT_ID_BINDING_PRESHARE_SECRET,
                                                  platform_binding_secret_metadata,
                                                  &bytes_to_read);

        WAIT_AND_CHECK_STATUS(return_status, optiga_lib_status);

        /**
         * 4. Validate LcsO in the metadata.
         *    Skip the rest of the procedure if LcsO is greater than or equal to operational state(0x07)
         */
        LOG_WICED_OPTIGA("4. Validate LcsO in the metadata[4]\n");
        if (platform_binding_secret_metadata[4] >= LCSO_STATE_OPERATIONAL)
        {
            // The LcsO is already greater than or equal to operational state
            break;
        }

        /**
         * 5. Generate Random using optiga_crypt_random
         *       - Specify the Random type as TRNG
         *    a. The maximum supported size of secret is 64 bytes.
         *       The minimum recommended is 32 bytes.
         *    b. If the host platform doesn't support random generation,
         *       use OPTIGA to generate the maximum size chosen.
         *       else choose the appropriate length of random to be generated by OPTIGA
         *
         */
        LOG_WICED_OPTIGA("5. Generate Random \n");
        optiga_lib_status = OPTIGA_LIB_BUSY;
        return_status = optiga_crypt_random(me_crypt_instance,
                                            OPTIGA_RNG_TYPE_TRNG,
                                            platform_binding_secret,
                                            sizeof(platform_binding_secret));
        WAIT_AND_CHECK_STATUS(return_status, optiga_lib_status);

        /**
         * 6. Generate random on Host
         *    If the host platform doesn't support, skip this step
         */

        /**
         * 7. Write random(secret) to OPTIGA platform Binding shared secret data object (0xE140)
         */
        LOG_WICED_OPTIGA("7. Write random(secret) to OPTIGA\n");
        optiga_lib_status = OPTIGA_LIB_BUSY;
        OPTIGA_UTIL_SET_COMMS_PROTECTION_LEVEL(me_util_instance, OPTIGA_COMMS_NO_PROTECTION);
        return_status = optiga_util_write_data(me_util_instance,
                                               OPTIGA_OBJECT_ID_BINDING_PRESHARE_SECRET,
                                               OPTIGA_UTIL_ERASE_AND_WRITE,
                                               0,
                                               platform_binding_secret,
                                               sizeof(platform_binding_secret));
        WAIT_AND_CHECK_STATUS(return_status, optiga_lib_status);

        /**
         * 8. Write/store the random(secret) on the Host platform
         *
         */
        LOG_WICED_OPTIGA("8. Write/store the random(secret) on the Host\n");
        pal_return_status = pal_os_datastore_write(OPTIGA_PLATFORM_BINDING_SHARED_SECRET_ID,
                                                   platform_binding_secret,
                                                   sizeof(platform_binding_secret));

        if (PAL_STATUS_SUCCESS != pal_return_status)
        {
            //Storing of Pre-shared secret on Host failed.
            optiga_lib_status = pal_return_status;
            break;
        }

        /**
         * 9. Update metadata of OPTIGA Platform Binding shared secret data object (0xE140)
         */
        LOG_WICED_OPTIGA("9. Update metadata\n");
        optiga_lib_status = OPTIGA_LIB_BUSY;
        OPTIGA_UTIL_SET_COMMS_PROTECTION_LEVEL(me_util_instance, OPTIGA_COMMS_NO_PROTECTION);
        return_status = optiga_util_write_metadata(me_util_instance,
                                                   OPTIGA_OBJECT_ID_BINDING_PRESHARE_SECRET,
                                                   platform_binding_shared_secret_metadata_final,
                                                   sizeof(platform_binding_shared_secret_metadata_final));

        WAIT_AND_CHECK_STATUS(return_status, optiga_lib_status);

        return_status = OPTIGA_LIB_SUCCESS;

    } while(FALSE);

    READ_PERFORMANCE_MEASUREMENT(time_taken_for_pairing);
    LOG_WICED_OPTIGA_PERFORMANCE(time_taken_for_pairing, return_status);

    return return_status;
}
#endif /* WICED_OPTIGA_PAIR_HOST */

void wiced_optiga_init(void)
{
    optiga_lib_status_t return_status = !OPTIGA_LIB_SUCCESS;

    do
    {
        LOG_WICED_OPTIGA("%s\n", __FUNCTION__);

        /**
         * Create OPTIGA Util and Crypt Instances
         */
        if (NULL == me_util_instance)
        {
            me_util_instance = optiga_util_create(0, optiga_lib_callback, NULL);
            if (NULL == me_util_instance)
            {
                break;
            }
        }

        if (NULL == me_crypt_instance)
        {
            me_crypt_instance = optiga_crypt_create(0, optiga_lib_callback, NULL);
            if (NULL == me_crypt_instance)
            {
                break;
            }
        }

        /**
         * Open the application on OPTIGA which is a precondition to perform any other operations
         * using optiga_util_open_application
         */
        optiga_lib_status = OPTIGA_LIB_BUSY;
        return_status = optiga_util_open_application(me_util_instance, 0);

        WAIT_AND_CHECK_STATUS(return_status, optiga_lib_status);

#ifdef WICED_OPTIGA_PAIR_HOST
        if (FALSE == host_optiga_pairing_completed)
        {
            LOG_WICED_OPTIGA("pair_host_and_optiga_using_pre_shared_secret");
            // Generate the pre-shared secret on host and write it to OPTIGA
            return_status = pair_host_and_optiga_using_pre_shared_secret();
            if (OPTIGA_LIB_SUCCESS != return_status)
            {
                //pairing of host and optiga failed
                break;
            }
            host_optiga_pairing_completed = TRUE;
        }
#endif
    } while(FALSE);
}

void wiced_optiga_deinit(void)
{
    optiga_lib_status_t return_status = !OPTIGA_LIB_SUCCESS;
    LOG_WICED_OPTIGA("%s\n", __FUNCTION__);

    if(me_crypt_instance)
    {
        /**
         * Close the application on OPTIGA after all the operations are executed
         * using optiga_util_close_application
         */
        optiga_lib_status = OPTIGA_LIB_BUSY;
        return_status = optiga_util_close_application(me_util_instance, 0);

        WAIT_AND_CHECK_STATUS(return_status, optiga_lib_status);

        optiga_util_destroy(me_util_instance);
        me_util_instance = NULL;
    }

    if(me_crypt_instance)
    {
        //Destroy the instance after the completion of usecase if not required.
        return_status = optiga_crypt_destroy(me_crypt_instance);
        me_crypt_instance = NULL;
    }
}

wiced_result_t wiced_optiga_read_data(uint16_t optiga_oid, uint8_t data_type, void *p_data, uint16_t *length)
{
    uint32_t time_taken = 0;
    optiga_lib_status_t return_status = !OPTIGA_LIB_SUCCESS;

    if (p_data == NULL)
    {
        return WICED_ERROR;
    }

    do
    {
        LOG_WICED_OPTIGA("%s\n", __FUNCTION__);

        optiga_lib_status = OPTIGA_LIB_BUSY;

        START_PERFORMANCE_MEASUREMENT(time_taken);

        if (data_type == OPTIGA_OBJECT_DATA)
        {
            return_status = optiga_util_read_data(me_util_instance,
                                                  optiga_oid,
                                                  0,
                                                  p_data,
                                                  length);
        }
        else if (data_type == OPTIGA_OBJECT_METADATA)
        {
            return_status = optiga_util_read_metadata(me_util_instance,
                                                      optiga_oid,
                                                      p_data,
                                                      length);
        }
        else
            return_status = OPTIGA_UTIL_ERROR_INVALID_INPUT;

        WAIT_AND_CHECK_STATUS(return_status, optiga_lib_status);

        READ_PERFORMANCE_MEASUREMENT(time_taken);

        LOG_WICED_OPTIGA("%s %u\n", __func__, *length);
        LOG_WICED_OPTIGA_HEX(p_data, *length);
    } while (FALSE);

    LOG_WICED_OPTIGA_PERFORMANCE(time_taken, return_status);

    return return_status == OPTIGA_LIB_SUCCESS ? WICED_SUCCESS : WICED_ERROR;
}

wiced_result_t wiced_optiga_write_data(uint16_t optiga_oid, uint8_t data_type, const void *p_data, uint16_t length)
{
    uint32_t time_taken = 0;

    optiga_lib_status_t return_status = !OPTIGA_LIB_SUCCESS;

    do
    {
        LOG_WICED_OPTIGA("%s\n", __FUNCTION__);

        OPTIGA_UTIL_SET_COMMS_PROTECTION_LEVEL(me_util_instance, OPTIGA_COMMS_NO_PROTECTION);

        optiga_lib_status = OPTIGA_LIB_BUSY;

        START_PERFORMANCE_MEASUREMENT(time_taken);

        if (data_type == OPTIGA_OBJECT_DATA)
        {
            return_status = optiga_util_write_data(me_util_instance,
                                                   optiga_oid,
                                                   OPTIGA_UTIL_ERASE_AND_WRITE,
                                                   0,
                                                   p_data,
                                                   length);
        }
        else if (data_type == OPTIGA_OBJECT_METADATA)
        {
            return_status = optiga_util_write_metadata(me_util_instance,
                                                       optiga_oid,
                                                       p_data,
                                                       length);
        }
        else
            return_status = OPTIGA_UTIL_ERROR_INVALID_INPUT;

        WAIT_AND_CHECK_STATUS(return_status, optiga_lib_status);

        READ_PERFORMANCE_MEASUREMENT(time_taken);

    } while (FALSE);

    LOG_WICED_OPTIGA_PERFORMANCE(time_taken, return_status);

    return return_status == OPTIGA_LIB_SUCCESS ? WICED_SUCCESS : WICED_ERROR;
}

wiced_result_t wiced_optiga_protected_update_start(uint8_t manifest_version, const uint8_t *manifest, uint16_t manifest_length)
{
    optiga_lib_status_t return_status = !OPTIGA_LIB_SUCCESS;
    uint32_t time_taken = 0;

    LOG_WICED_OPTIGA("%s\n", __FUNCTION__);

    optiga_lib_status = OPTIGA_LIB_BUSY;

    START_PERFORMANCE_MEASUREMENT(time_taken);

    return_status = optiga_util_protected_update_start(me_util_instance,
                                                       manifest_version,
                                                       manifest,
                                                       manifest_length);

    WAIT_AND_CHECK_STATUS(return_status, optiga_lib_status);

    READ_PERFORMANCE_MEASUREMENT(time_taken);
    LOG_WICED_OPTIGA_PERFORMANCE(time_taken, return_status);

    return return_status == OPTIGA_LIB_SUCCESS ? WICED_SUCCESS : WICED_ERROR;
}

wiced_result_t wiced_optiga_protected_update_continue(const uint8_t *fragment, uint16_t fragment_length)
{
    optiga_lib_status_t return_status = !OPTIGA_LIB_SUCCESS;
    uint32_t time_taken = 0;

    LOG_WICED_OPTIGA("%s\n", __FUNCTION__);

    optiga_lib_status = OPTIGA_LIB_BUSY;

    START_PERFORMANCE_MEASUREMENT(time_taken);

    return_status = optiga_util_protected_update_continue(me_util_instance,
                                                       fragment,
                                                       fragment_length);

    WAIT_AND_CHECK_STATUS(return_status, optiga_lib_status);

    READ_PERFORMANCE_MEASUREMENT(time_taken);
    LOG_WICED_OPTIGA_PERFORMANCE(time_taken, return_status);

    return return_status == OPTIGA_LIB_SUCCESS ? WICED_SUCCESS : WICED_ERROR;
}

wiced_result_t wiced_optiga_protected_update_final(const uint8_t *fragment, uint16_t fragment_length)
{
    optiga_lib_status_t return_status = !OPTIGA_LIB_SUCCESS;
    uint32_t time_taken = 0;

    LOG_WICED_OPTIGA("%s\n", __FUNCTION__);

    optiga_lib_status = OPTIGA_LIB_BUSY;

    START_PERFORMANCE_MEASUREMENT(time_taken);

    return_status = optiga_util_protected_update_final(me_util_instance,
                                                       fragment,
                                                       fragment_length);

    WAIT_AND_CHECK_STATUS(return_status, optiga_lib_status);

    READ_PERFORMANCE_MEASUREMENT(time_taken);
    LOG_WICED_OPTIGA_PERFORMANCE(time_taken, return_status);

    return return_status == OPTIGA_LIB_SUCCESS ? WICED_SUCCESS : WICED_ERROR;
}

wiced_result_t wiced_optiga_ecdsa_sign(const void *digest, uint8_t digest_length, uint16_t private_key, void *signature, uint16_t *signature_length)
{
    optiga_lib_status_t return_status;
    uint32_t time_taken = 0;

    LOG_WICED_OPTIGA("%s\n", __FUNCTION__);

    optiga_lib_status = OPTIGA_LIB_BUSY;

    START_PERFORMANCE_MEASUREMENT(time_taken);

    return_status = optiga_crypt_ecdsa_sign(me_crypt_instance, digest, digest_length, private_key, signature, signature_length);

    WAIT_AND_CHECK_STATUS(return_status, optiga_lib_status);

    READ_PERFORMANCE_MEASUREMENT(time_taken);
    LOG_WICED_OPTIGA_PERFORMANCE(time_taken, return_status);

    return return_status == OPTIGA_LIB_SUCCESS ? WICED_SUCCESS : WICED_ERROR;
}

wiced_result_t wiced_optiga_ecdsa_verify(const uint8_t *digest, uint8_t digest_length, const uint8_t *signature, uint16_t signature_length, uint8_t public_key_source_type, const void *public_key)
{
    optiga_lib_status_t return_status;
    uint32_t time_taken = 0;

    LOG_WICED_OPTIGA("%s\n", __FUNCTION__);

    optiga_lib_status = OPTIGA_LIB_BUSY;

    START_PERFORMANCE_MEASUREMENT(time_taken);

    return_status = optiga_crypt_ecdsa_verify(me_crypt_instance, digest, digest_length, signature, signature_length, public_key_source_type, public_key);

    WAIT_AND_CHECK_STATUS(return_status, optiga_lib_status);

    READ_PERFORMANCE_MEASUREMENT(time_taken);
    LOG_WICED_OPTIGA_PERFORMANCE(time_taken, return_status);

    return return_status == OPTIGA_LIB_SUCCESS ? WICED_SUCCESS : WICED_ERROR;
}
