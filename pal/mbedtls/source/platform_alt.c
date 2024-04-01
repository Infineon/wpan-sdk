/*
 *  Copyright (c) 2023, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <mbedtls/ctr_drbg.h>
#include <mbedtls/error.h>
#include <mbedtls/platform.h>
#include <spar_utils.h>
#include <stdbool.h>
#include <string.h>
#include <wiced_hal_platform.h>
#include <wiced_hal_rand.h>
#include <wiced_rtos.h>
#include <wiced_sleep.h>
#include <wiced_utilities.h>

static bool is_platform_ctr_drbg_initialized(void);
static int  platform_ctr_drbg_init(void);
static int  platform_entropy_func(void *data, unsigned char *output, size_t len);

PLACE_CODE_IN_RETENTION_RAM static mbedtls_ctr_drbg_context platform_ctr_drbg_context;
static wiced_mutex_t                                       *platform_ctr_drbg_mutex = NULL;

#ifdef MBEDTLS_PLATFORM_SETUP_TEARDOWN_ALT
int mbedtls_platform_setup(mbedtls_platform_context *ctx)
{
    (void)ctx;
    return 0;
}
#endif /* MBEDTLS_PLATFORM_SETUP_TEARDOWN_ALT */

wiced_result_t wiced_hal_platform_random_get(uint8_t *output, size_t length, size_t *output_length)
{
    if (!is_platform_ctr_drbg_initialized())
    {
        if (platform_ctr_drbg_init() != 0)
        {
            return WICED_NOT_AVAILABLE;
        }
    }

    if (output == NULL)
    {
        return WICED_BADARG;
    }

    if (output_length != NULL)
    {
        length = MBEDTLS_CTR_DRBG_BLOCKSIZE;
    }

    wiced_rtos_lock_mutex(platform_ctr_drbg_mutex);
    const int result = mbedtls_ctr_drbg_random(&platform_ctr_drbg_context, output, length);
    wiced_rtos_unlock_mutex(platform_ctr_drbg_mutex);
    if (result != 0)
    {
        return WICED_ERROR;
    }

    if (output_length != NULL)
    {
        *output_length = length;
    }

    return WICED_SUCCESS;
}

bool is_platform_ctr_drbg_initialized(void) { return platform_ctr_drbg_mutex != NULL; }

int platform_ctr_drbg_init(void)
{
    platform_ctr_drbg_mutex = wiced_rtos_create_mutex();
    if (platform_ctr_drbg_mutex == NULL)
    {
        return MBEDTLS_ERR_ERROR_GENERIC_ERROR;
    }
    wiced_rtos_init_mutex(platform_ctr_drbg_mutex);

    if (wiced_sleep_get_boot_mode() == WICED_SLEEP_COLD_BOOT)
    {
        mbedtls_ctr_drbg_init(&platform_ctr_drbg_context);
        return mbedtls_ctr_drbg_seed(&platform_ctr_drbg_context, platform_entropy_func, NULL, NULL, 0);
    }

    return 0;
}

int platform_entropy_func(void *data, unsigned char *output, size_t len)
{
    (void)data;
    size_t   i;
    uint32_t random;

    for (i = 0; i < len; i += sizeof(random))
    {
        random = wiced_hal_rand_gen_num();
        memcpy(output + i, &random, MIN(sizeof(random), len - i));
    }

    return 0;
}
