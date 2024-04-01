/*
 *  Copyright (c) 2021, The OpenThread Authors.
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

/*
 * Definition of CCM:
 * http://csrc.nist.gov/publications/nistpubs/800-38C/SP800-38C_updated-July20_2007.pdf
 * RFC 3610 "Counter with CBC-MAC (CCM)"
 *
 * Related:
 * RFC 5116 "An Interface and Algorithms for Authenticated Encryption"
 */

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#if defined(MBEDTLS_CCM_C)
#include "wiced_hal_crypto.h"
#include "mbedtls/ccm.h"
#include "mbedtls/error.h"
#include "mbedtls/platform_util.h"

#include <string.h>

#if defined(MBEDTLS_CCM_ALT)
#define mbedtls_calloc calloc
#define mbedtls_free free

#define CCM_VALIDATE_RET(cond) MBEDTLS_INTERNAL_VALIDATE_RET(cond, MBEDTLS_ERR_CCM_BAD_INPUT)
#define CCM_VALIDATE(cond) MBEDTLS_INTERNAL_VALIDATE(cond)

void mbedtls_ccm_init(mbedtls_ccm_context *ctx)
{
    CCM_VALIDATE(ctx != NULL);

    memset(ctx, 0, sizeof(mbedtls_ccm_context));
}

int mbedtls_ccm_setkey(mbedtls_ccm_context *ctx,
                       mbedtls_cipher_id_t  cipher,
                       const unsigned char *key,
                       unsigned int         keybits)
{
    (void)cipher;

    ctx->key_len = keybits >> 3;
    memcpy(ctx->key, key, ctx->key_len);
    return (0);
}

void mbedtls_ccm_free(mbedtls_ccm_context *ctx)
{
    if (ctx == NULL)
        return;
    mbedtls_platform_zeroize(ctx, sizeof(mbedtls_ccm_context));
}

static int ccm_auth_crypt(mbedtls_ccm_context *ctx,
                          int                  mode,
                          size_t               length,
                          const unsigned char *iv,
                          size_t               iv_len,
                          const unsigned char *add,
                          size_t               add_len,
                          const unsigned char *input,
                          unsigned char       *output,
                          unsigned char       *tag,
                          size_t               tag_len)
{
    wiced_crypto_result_t ret;
    uint8_t              *ccm_buf;

    if (tag_len == 2 || tag_len > 16 || tag_len % 2 != 0)
        return (MBEDTLS_ERR_CCM_BAD_INPUT);

    if (iv_len < 11 || iv_len > 14)
        return (MBEDTLS_ERR_CCM_BAD_INPUT);

    if (add_len > 0xFF00)
        return (MBEDTLS_ERR_CCM_BAD_INPUT);

    if ((mode != WICED_CRYPTO_CCM_ENCRYPT) && (mode != WICED_CRYPTO_CCM_DECRYPT))
        return (MBEDTLS_ERR_CCM_BAD_INPUT);

    if ((ccm_buf = mbedtls_calloc(1, add_len + length + tag_len + 1)) == NULL)
        return MBEDTLS_ERR_CCM_AUTH_FAILED;

    ret = wiced_hal_crypto_ccm_engine(mode, ctx->key, ctx->key_len, (uint8_t *)iv, iv_len, (uint8_t *)add, add_len,
                                      (uint8_t *)input, length, tag, tag_len, output, ccm_buf);

    mbedtls_free(ccm_buf);

    if (ret == WICED_CRYPTO_CCM_INPUT_ERROR)
        return MBEDTLS_ERR_CCM_BAD_INPUT;

    if (ret != WICED_CRYPTO_CCM_SUCCESS)
        return MBEDTLS_ERR_CCM_HW_ACCEL_FAILED;

    return (0);
}

/*
 * Authenticated encryption
 */
int mbedtls_ccm_star_encrypt_and_tag(mbedtls_ccm_context *ctx,
                                     size_t               length,
                                     const unsigned char *iv,
                                     size_t               iv_len,
                                     const unsigned char *add,
                                     size_t               add_len,
                                     const unsigned char *input,
                                     unsigned char       *output,
                                     unsigned char       *tag,
                                     size_t               tag_len)
{
    return (
        ccm_auth_crypt(ctx, WICED_CRYPTO_CCM_ENCRYPT, length, iv, iv_len, add, add_len, input, output, tag, tag_len));
}

int mbedtls_ccm_encrypt_and_tag(mbedtls_ccm_context *ctx,
                                size_t               length,
                                const unsigned char *iv,
                                size_t               iv_len,
                                const unsigned char *add,
                                size_t               add_len,
                                const unsigned char *input,
                                unsigned char       *output,
                                unsigned char       *tag,
                                size_t               tag_len)
{
    if (tag_len == 0)
        return (MBEDTLS_ERR_CCM_BAD_INPUT);

    return (mbedtls_ccm_star_encrypt_and_tag(ctx, length, iv, iv_len, add, add_len, input, output, tag, tag_len));
}

/*
 * Authenticated decryption
 */
int mbedtls_ccm_star_auth_decrypt(mbedtls_ccm_context *ctx,
                                  size_t               length,
                                  const unsigned char *iv,
                                  size_t               iv_len,
                                  const unsigned char *add,
                                  size_t               add_len,
                                  const unsigned char *input,
                                  unsigned char       *output,
                                  const unsigned char *tag,
                                  size_t               tag_len)
{
    int           ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    unsigned char i;
    int           diff;

    if ((ret = ccm_auth_crypt(ctx, WICED_CRYPTO_CCM_DECRYPT, length, iv, iv_len, add, add_len, input, output,
                              ctx->decrypt_tag, tag_len)) != 0)
    {
        return ret;
    }

    /* input tag NULL means upper layer does not provide authenticated data to compare */
    if (tag == NULL)
    {
        return (0);
    }

    /* Check tag in "constant-time" */
    for (diff = 0, i = 0; i < tag_len; i++)
    {
        diff |= tag[i] ^ ctx->decrypt_tag[i];
    }
    if (diff != 0)
    {
        mbedtls_platform_zeroize(output, length);
        return (MBEDTLS_ERR_CCM_AUTH_FAILED);
    }

    return (0);
}

int mbedtls_ccm_auth_decrypt(mbedtls_ccm_context *ctx,
                             size_t               length,
                             const unsigned char *iv,
                             size_t               iv_len,
                             const unsigned char *add,
                             size_t               add_len,
                             const unsigned char *input,
                             unsigned char       *output,
                             const unsigned char *tag,
                             size_t               tag_len)
{
    if (tag_len == 0)
        return (MBEDTLS_ERR_CCM_BAD_INPUT);

    return (mbedtls_ccm_star_auth_decrypt(ctx, length, iv, iv_len, add, add_len, input, output, tag, tag_len));
}

void mbedtls_ccm_get_decrypt_tag(mbedtls_ccm_context *ctx, const unsigned char *tag, size_t tag_len)
{
    memcpy((void *)tag, ctx->decrypt_tag, tag_len);
}

#endif /* !MBEDTLS_CCM_ALT */

#endif /* MBEDTLS_CCM_C */
