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

/*
 *  The SHA-256 Secure Hash Standard was published by NIST in 2002.
 *
 *  http://csrc.nist.gov/publications/fips/fips180-2/fips180-2.pdf
 */

#include "wiced_hal_crypto.h"
#include "wiced_rtos.h"
#include <string.h>
#include "mbedtls/bn_mul.h"
#include "mbedtls/ecp.h"
#include "mbedtls/error.h"
#include "mbedtls/platform_util.h"
#include "mbedtls/threading.h"

#if defined(MBEDTLS_ECP_ALT)

/* Parameter validation macros based on platform_util.h */
#define ECP_VALIDATE_RET(cond) MBEDTLS_INTERNAL_VALIDATE_RET(cond, MBEDTLS_ERR_ECP_BAD_INPUT_DATA)
#define ECP_VALIDATE(cond) MBEDTLS_INTERNAL_VALIDATE(cond)

#if defined(MBEDTLS_PLATFORM_C)
#include "mbedtls/platform.h"
#else
#include <stdio.h>
#include <stdlib.h>
#define mbedtls_printf printf
#define mbedtls_calloc calloc
#define mbedtls_free free
#endif

#include "mbedtls/ecp_internal.h"

#if !defined(MBEDTLS_ECP_NO_INTERNAL_RNG)
#if defined(MBEDTLS_HMAC_DRBG_C)
#include "mbedtls/hmac_drbg.h"
#elif defined(MBEDTLS_CTR_DRBG_C)
#include "mbedtls/ctr_drbg.h"
#else
#error "Invalid configuration detected. Include check_config.h to ensure that the configuration is valid."
#endif
#endif /* MBEDTLS_ECP_NO_INTERNAL_RNG */

#if (defined(__ARMCC_VERSION) || defined(_MSC_VER)) && !defined(inline) && !defined(__cplusplus)
#define inline __inline
#endif

#define LOAD_GROUP(G)                                                                                        \
    ecp_group_load(grp, G##_p, sizeof(G##_p), NULL, 0, G##_b, sizeof(G##_b), G##_gx, sizeof(G##_gx), G##_gy, \
                   sizeof(G##_gy), G##_n, sizeof(G##_n))

//#define NIST_MODP(P) grp->modp = ecp_mod_##P;

/*
 * Domain parameters for secp256r1
 */
#if defined(MBEDTLS_ECP_DP_SECP256R1_ENABLED)
static const mbedtls_mpi_uint secp256r1_p[] = {
    MBEDTLS_BYTES_TO_T_UINT_8(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF),
    MBEDTLS_BYTES_TO_T_UINT_8(0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00),
    MBEDTLS_BYTES_TO_T_UINT_8(0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
    MBEDTLS_BYTES_TO_T_UINT_8(0x01, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF),
};
static const mbedtls_mpi_uint secp256r1_b[] = {
    MBEDTLS_BYTES_TO_T_UINT_8(0x4B, 0x60, 0xD2, 0x27, 0x3E, 0x3C, 0xCE, 0x3B),
    MBEDTLS_BYTES_TO_T_UINT_8(0xF6, 0xB0, 0x53, 0xCC, 0xB0, 0x06, 0x1D, 0x65),
    MBEDTLS_BYTES_TO_T_UINT_8(0xBC, 0x86, 0x98, 0x76, 0x55, 0xBD, 0xEB, 0xB3),
    MBEDTLS_BYTES_TO_T_UINT_8(0xE7, 0x93, 0x3A, 0xAA, 0xD8, 0x35, 0xC6, 0x5A),
};
static const mbedtls_mpi_uint secp256r1_gx[] = {
    MBEDTLS_BYTES_TO_T_UINT_8(0x96, 0xC2, 0x98, 0xD8, 0x45, 0x39, 0xA1, 0xF4),
    MBEDTLS_BYTES_TO_T_UINT_8(0xA0, 0x33, 0xEB, 0x2D, 0x81, 0x7D, 0x03, 0x77),
    MBEDTLS_BYTES_TO_T_UINT_8(0xF2, 0x40, 0xA4, 0x63, 0xE5, 0xE6, 0xBC, 0xF8),
    MBEDTLS_BYTES_TO_T_UINT_8(0x47, 0x42, 0x2C, 0xE1, 0xF2, 0xD1, 0x17, 0x6B),
};
static const mbedtls_mpi_uint secp256r1_gy[] = {
    MBEDTLS_BYTES_TO_T_UINT_8(0xF5, 0x51, 0xBF, 0x37, 0x68, 0x40, 0xB6, 0xCB),
    MBEDTLS_BYTES_TO_T_UINT_8(0xCE, 0x5E, 0x31, 0x6B, 0x57, 0x33, 0xCE, 0x2B),
    MBEDTLS_BYTES_TO_T_UINT_8(0x16, 0x9E, 0x0F, 0x7C, 0x4A, 0xEB, 0xE7, 0x8E),
    MBEDTLS_BYTES_TO_T_UINT_8(0x9B, 0x7F, 0x1A, 0xFE, 0xE2, 0x42, 0xE3, 0x4F),
};
static const mbedtls_mpi_uint secp256r1_n[] = {
    MBEDTLS_BYTES_TO_T_UINT_8(0x51, 0x25, 0x63, 0xFC, 0xC2, 0xCA, 0xB9, 0xF3),
    MBEDTLS_BYTES_TO_T_UINT_8(0x84, 0x9E, 0x17, 0xA7, 0xAD, 0xFA, 0xE6, 0xBC),
    MBEDTLS_BYTES_TO_T_UINT_8(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF),
    MBEDTLS_BYTES_TO_T_UINT_8(0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF),
};

#endif /* MBEDTLS_ECP_DP_SECP256R1_ENABLED */

/*
 * Create an MPI from embedded constants
 * (assumes len is an exact multiple of sizeof mbedtls_mpi_uint)
 */
static inline void ecp_mpi_load(mbedtls_mpi *X, const mbedtls_mpi_uint *p, size_t len)
{
    X->s = 1;
    X->n = len / sizeof(mbedtls_mpi_uint);
    X->p = (mbedtls_mpi_uint *)p;
}

/*
 * Set an MPI to static value 1
 */
static inline void ecp_mpi_set1(mbedtls_mpi *X)
{
    static mbedtls_mpi_uint one[] = {1};
    X->s                          = 1;
    X->n                          = 1;
    X->p                          = one;
}

/*
 * Make group available from embedded constants
 */
static int ecp_group_load(mbedtls_ecp_group      *grp,
                          const mbedtls_mpi_uint *p,
                          size_t                  plen,
                          const mbedtls_mpi_uint *a,
                          size_t                  alen,
                          const mbedtls_mpi_uint *b,
                          size_t                  blen,
                          const mbedtls_mpi_uint *gx,
                          size_t                  gxlen,
                          const mbedtls_mpi_uint *gy,
                          size_t                  gylen,
                          const mbedtls_mpi_uint *n,
                          size_t                  nlen)
{
    ecp_mpi_load(&grp->P, p, plen);
    if (a != NULL)
        ecp_mpi_load(&grp->A, a, alen);
    ecp_mpi_load(&grp->B, b, blen);
    ecp_mpi_load(&grp->N, n, nlen);

    ecp_mpi_load(&grp->G.X, gx, gxlen);
    ecp_mpi_load(&grp->G.Y, gy, gylen);
    ecp_mpi_set1(&grp->G.Z);

    grp->pbits = mbedtls_mpi_bitlen(&grp->P);
    grp->nbits = mbedtls_mpi_bitlen(&grp->N);

    return (0);
}

/*
 * Set a group using well-known domain parameters
 */
int mbedtls_ecp_group_load(mbedtls_ecp_group *grp, mbedtls_ecp_group_id id)
{
    ECP_VALIDATE_RET(grp != NULL);
    mbedtls_ecp_group_free(grp);

    grp->id = id;

    switch (id)
    {
#if defined(MBEDTLS_ECP_DP_SECP256R1_ENABLED)
    case MBEDTLS_ECP_DP_SECP256R1:
        return (LOAD_GROUP(secp256r1));
#endif /* MBEDTLS_ECP_DP_SECP256R1_ENABLED */
    default:
        grp->id = MBEDTLS_ECP_DP_NONE;
        return (MBEDTLS_ERR_ECP_FEATURE_UNAVAILABLE);
    }
}

#if defined(MBEDTLS_SELF_TEST)
/*
 * Counts of point addition and doubling, and field multiplications.
 * Used to test resistance of point multiplication to simple timing attacks.
 */
static unsigned long add_count, dbl_count, mul_count;
#endif

#if !defined(MBEDTLS_ECP_NO_INTERNAL_RNG)
/*
 * Currently ecp_mul() takes a RNG function as an argument, used for
 * side-channel protection, but it can be NULL. The initial reasoning was
 * that people will pass non-NULL RNG when they care about side-channels, but
 * unfortunately we have some APIs that call ecp_mul() with a NULL RNG, with
 * no opportunity for the user to do anything about it.
 *
 * The obvious strategies for addressing that include:
 * - change those APIs so that they take RNG arguments;
 * - require a global RNG to be available to all crypto modules.
 *
 * Unfortunately those would break compatibility. So what we do instead is
 * have our own internal DRBG instance, seeded from the secret scalar.
 *
 * The following is a light-weight abstraction layer for doing that with
 * HMAC_DRBG (first choice) or CTR_DRBG.
 */

#if defined(MBEDTLS_HMAC_DRBG_C)

/* DRBG context type */
typedef mbedtls_hmac_drbg_context ecp_drbg_context;

/* DRBG context init */
static inline void ecp_drbg_init(ecp_drbg_context *ctx) { mbedtls_hmac_drbg_init(ctx); }

/* DRBG context free */
static inline void ecp_drbg_free(ecp_drbg_context *ctx) { mbedtls_hmac_drbg_free(ctx); }

/* DRBG function */
static inline int ecp_drbg_random(void *p_rng, unsigned char *output, size_t output_len)
{
    return (mbedtls_hmac_drbg_random(p_rng, output, output_len));
}

#elif defined(MBEDTLS_CTR_DRBG_C)

/* DRBG context type */
typedef mbedtls_ctr_drbg_context ecp_drbg_context;

/* DRBG context init */
static inline void ecp_drbg_init(ecp_drbg_context *ctx) { mbedtls_ctr_drbg_init(ctx); }

/* DRBG context free */
static inline void ecp_drbg_free(ecp_drbg_context *ctx) { mbedtls_ctr_drbg_free(ctx); }

/* DRBG function */
static inline int ecp_drbg_random(void *p_rng, unsigned char *output, size_t output_len)
{
    return (mbedtls_ctr_drbg_random(p_rng, output, output_len));
}

/*
 * Since CTR_DRBG doesn't have a seed_buf() function the way HMAC_DRBG does,
 * we need to pass an entropy function when seeding. So we use a dummy
 * function for that, and pass the actual entropy as customisation string.
 * (During seeding of CTR_DRBG the entropy input and customisation string are
 * concatenated before being used to update the secret state.)
 */
static int ecp_ctr_drbg_null_entropy(void *ctx, unsigned char *out, size_t len)
{
    (void)ctx;
    memset(out, 0, len);
    return (0);
}

/* DRBG context seeding */
static int ecp_drbg_seed(ecp_drbg_context *ctx, const mbedtls_mpi *secret, size_t secret_len)
{
    int           ret;
    unsigned char secret_bytes[MBEDTLS_ECP_MAX_BYTES];

    if (secret_len > MBEDTLS_ECP_MAX_BYTES)
    {
        ret = MBEDTLS_ERR_ECP_RANDOM_FAILED;
        goto cleanup;
    }

    MBEDTLS_MPI_CHK(mbedtls_mpi_write_binary(secret, secret_bytes, secret_len));

    ret = mbedtls_ctr_drbg_seed(ctx, ecp_ctr_drbg_null_entropy, NULL, secret_bytes, secret_len);

cleanup:
    mbedtls_platform_zeroize(secret_bytes, secret_len);

    return (ret);
}

#else
#error "Invalid configuration detected. Include check_config.h to ensure that the configuration is valid."
#endif /* DRBG modules */
#endif /* MBEDTLS_ECP_NO_INTERNAL_RNG */

#if defined(MBEDTLS_ECP_RESTARTABLE)
/*
 * Maximum number of "basic operations" to be done in a row.
 *
 * Default value 0 means that ECC operations will not yield.
 * Note that regardless of the value of ecp_max_ops, always at
 * least one step is performed before yielding.
 *
 * Setting ecp_max_ops=1 can be suitable for testing purposes
 * as it will interrupt computation at all possible points.
 */
static unsigned ecp_max_ops = 0;

/*
 * Set ecp_max_ops
 */
void mbedtls_ecp_set_max_ops(unsigned max_ops) { ecp_max_ops = max_ops; }

/*
 * Check if restart is enabled
 */
int mbedtls_ecp_restart_is_enabled(void) { return (ecp_max_ops != 0); }

/*
 * Restart sub-context for ecp_mul_comb()
 */
struct mbedtls_ecp_restart_mul
{
    mbedtls_ecp_point  R;      /* current intermediate result                  */
    size_t             i;      /* current index in various loops, 0 outside    */
    mbedtls_ecp_point *T;      /* table for precomputed points                 */
    unsigned char      T_size; /* number of points in table T                  */
    enum
    {                       /* what were we doing last time we returned?    */
      ecp_rsm_init = 0,     /* nothing so far, dummy initial state      */
      ecp_rsm_pre_dbl,      /* precompute 2^n multiples                 */
      ecp_rsm_pre_norm_dbl, /* normalize precomputed 2^n multiples      */
      ecp_rsm_pre_add,      /* precompute remaining points by adding    */
      ecp_rsm_pre_norm_add, /* normalize all precomputed points         */
      ecp_rsm_comb_core,    /* ecp_mul_comb_core()                      */
      ecp_rsm_final_norm,   /* do the final normalization               */
    } state;
#if !defined(MBEDTLS_ECP_NO_INTERNAL_RNG)
    ecp_drbg_context drbg_ctx;
    unsigned char    drbg_seeded;
#endif
};

/*
 * Init restart_mul sub-context
 */
static void ecp_restart_rsm_init(mbedtls_ecp_restart_mul_ctx *ctx)
{
    mbedtls_ecp_point_init(&ctx->R);
    ctx->i      = 0;
    ctx->T      = NULL;
    ctx->T_size = 0;
    ctx->state  = ecp_rsm_init;
#if !defined(MBEDTLS_ECP_NO_INTERNAL_RNG)
    ecp_drbg_init(&ctx->drbg_ctx);
    ctx->drbg_seeded = 0;
#endif
}

/*
 * Free the components of a restart_mul sub-context
 */
static void ecp_restart_rsm_free(mbedtls_ecp_restart_mul_ctx *ctx)
{
    unsigned char i;

    if (ctx == NULL)
        return;

    mbedtls_ecp_point_free(&ctx->R);

    if (ctx->T != NULL)
    {
        for (i = 0; i < ctx->T_size; i++) mbedtls_ecp_point_free(ctx->T + i);
        mbedtls_free(ctx->T);
    }

#if !defined(MBEDTLS_ECP_NO_INTERNAL_RNG)
    ecp_drbg_free(&ctx->drbg_ctx);
#endif

    ecp_restart_rsm_init(ctx);
}

/*
 * Restart context for ecp_muladd()
 */
struct mbedtls_ecp_restart_muladd
{
    mbedtls_ecp_point mP; /* mP value                             */
    mbedtls_ecp_point R;  /* R intermediate result                */
    enum
    {                    /* what should we do next?              */
      ecp_rsma_mul1 = 0, /* first multiplication                 */
      ecp_rsma_mul2,     /* second multiplication                */
      ecp_rsma_add,      /* addition                             */
      ecp_rsma_norm,     /* normalization                        */
    } state;
};

/*
 * Init restart_muladd sub-context
 */
static void ecp_restart_ma_init(mbedtls_ecp_restart_muladd_ctx *ctx)
{
    mbedtls_ecp_point_init(&ctx->mP);
    mbedtls_ecp_point_init(&ctx->R);
    ctx->state = ecp_rsma_mul1;
}

/*
 * Free the components of a restart_muladd sub-context
 */
static void ecp_restart_ma_free(mbedtls_ecp_restart_muladd_ctx *ctx)
{
    if (ctx == NULL)
        return;

    mbedtls_ecp_point_free(&ctx->mP);
    mbedtls_ecp_point_free(&ctx->R);

    ecp_restart_ma_init(ctx);
}

/*
 * Initialize a restart context
 */
void mbedtls_ecp_restart_init(mbedtls_ecp_restart_ctx *ctx)
{
    ECP_VALIDATE(ctx != NULL);
    ctx->ops_done = 0;
    ctx->depth    = 0;
    ctx->rsm      = NULL;
    ctx->ma       = NULL;
}

/*
 * Free the components of a restart context
 */
void mbedtls_ecp_restart_free(mbedtls_ecp_restart_ctx *ctx)
{
    if (ctx == NULL)
        return;

    ecp_restart_rsm_free(ctx->rsm);
    mbedtls_free(ctx->rsm);

    ecp_restart_ma_free(ctx->ma);
    mbedtls_free(ctx->ma);

    mbedtls_ecp_restart_init(ctx);
}

/*
 * Check if we can do the next step
 */
int mbedtls_ecp_check_budget(const mbedtls_ecp_group *grp, mbedtls_ecp_restart_ctx *rs_ctx, unsigned ops)
{
    ECP_VALIDATE_RET(grp != NULL);

    if (rs_ctx != NULL && ecp_max_ops != 0)
    {
        /* scale depending on curve size: the chosen reference is 256-bit,
         * and multiplication is quadratic. Round to the closest integer. */
        if (grp->pbits >= 512)
            ops *= 4;
        else if (grp->pbits >= 384)
            ops *= 2;

        /* Avoid infinite loops: always allow first step.
         * Because of that, however, it's not generally true
         * that ops_done <= ecp_max_ops, so the check
         * ops_done > ecp_max_ops below is mandatory. */
        if ((rs_ctx->ops_done != 0) && (rs_ctx->ops_done > ecp_max_ops || ops > ecp_max_ops - rs_ctx->ops_done))
        {
            return (MBEDTLS_ERR_ECP_IN_PROGRESS);
        }

        /* update running count */
        rs_ctx->ops_done += ops;
    }

    return (0);
}

/* Call this when entering a function that needs its own sub-context */
#define ECP_RS_ENTER(SUB)                                                              \
    do                                                                                 \
    {                                                                                  \
        /* reset ops count for this call if top-level */                               \
        if (rs_ctx != NULL && rs_ctx->depth++ == 0)                                    \
            rs_ctx->ops_done = 0;                                                      \
                                                                                       \
        /* set up our own sub-context if needed */                                     \
        if (mbedtls_ecp_restart_is_enabled() && rs_ctx != NULL && rs_ctx->SUB == NULL) \
        {                                                                              \
            rs_ctx->SUB = mbedtls_calloc(1, sizeof(*rs_ctx->SUB));                     \
            if (rs_ctx->SUB == NULL)                                                   \
                return (MBEDTLS_ERR_ECP_ALLOC_FAILED);                                 \
                                                                                       \
            ecp_restart_##SUB##_init(rs_ctx->SUB);                                     \
        }                                                                              \
    } while (0)

/* Call this when leaving a function that needs its own sub-context */
#define ECP_RS_LEAVE(SUB)                                                                \
    do                                                                                   \
    {                                                                                    \
        /* clear our sub-context when not in progress (done or error) */                 \
        if (rs_ctx != NULL && rs_ctx->SUB != NULL && ret != MBEDTLS_ERR_ECP_IN_PROGRESS) \
        {                                                                                \
            ecp_restart_##SUB##_free(rs_ctx->SUB);                                       \
            mbedtls_free(rs_ctx->SUB);                                                   \
            rs_ctx->SUB = NULL;                                                          \
        }                                                                                \
                                                                                         \
        if (rs_ctx != NULL)                                                              \
            rs_ctx->depth--;                                                             \
    } while (0)

#else /* MBEDTLS_ECP_RESTARTABLE */

#define ECP_RS_ENTER(sub) (void)rs_ctx;
#define ECP_RS_LEAVE(sub) (void)rs_ctx;

#endif /* MBEDTLS_ECP_RESTARTABLE */

/*
 * List of supported curves:
 *  - internal ID
 *  - TLS NamedCurve ID (RFC 4492 sec. 5.1.1, RFC 7071 sec. 2, RFC 8446 sec. 4.2.7)
 *  - size in bits
 *  - readable name
 *
 * Curves are listed in order: largest curves first, and for a given size,
 * fastest curves first. This provides the default order for the SSL module.
 *
 * Reminder: update profiles in x509_crt.c when adding a new curves!
 */
static const mbedtls_ecp_curve_info ecp_supported_curves[] = {
#if defined(MBEDTLS_ECP_DP_SECP256R1_ENABLED)
    {MBEDTLS_ECP_DP_SECP256R1, 23, 256, "secp256r1"},
#endif
    {MBEDTLS_ECP_DP_NONE, 0, 0, NULL},
};

#define ECP_NB_CURVES sizeof(ecp_supported_curves) / sizeof(ecp_supported_curves[0])

static mbedtls_ecp_group_id ecp_supported_grp_id[ECP_NB_CURVES];

/*
 * List of supported curves and associated info
 */
const mbedtls_ecp_curve_info *mbedtls_ecp_curve_list(void) { return (ecp_supported_curves); }

/*
 * List of supported curves, group ID only
 */
const mbedtls_ecp_group_id *mbedtls_ecp_grp_id_list(void)
{
    static int init_done = 0;

    if (!init_done)
    {
        size_t                        i = 0;
        const mbedtls_ecp_curve_info *curve_info;

        for (curve_info = mbedtls_ecp_curve_list(); curve_info->grp_id != MBEDTLS_ECP_DP_NONE; curve_info++)
        {
            ecp_supported_grp_id[i++] = curve_info->grp_id;
        }
        ecp_supported_grp_id[i] = MBEDTLS_ECP_DP_NONE;

        init_done = 1;
    }

    return (ecp_supported_grp_id);
}

/*
 * Get the curve info for the internal identifier
 */
const mbedtls_ecp_curve_info *mbedtls_ecp_curve_info_from_grp_id(mbedtls_ecp_group_id grp_id)
{
    const mbedtls_ecp_curve_info *curve_info;

    for (curve_info = mbedtls_ecp_curve_list(); curve_info->grp_id != MBEDTLS_ECP_DP_NONE; curve_info++)
    {
        if (curve_info->grp_id == grp_id)
            return (curve_info);
    }

    return (NULL);
}

/*
 * Get the curve info from the TLS identifier
 */
const mbedtls_ecp_curve_info *mbedtls_ecp_curve_info_from_tls_id(uint16_t tls_id)
{
    const mbedtls_ecp_curve_info *curve_info;

    for (curve_info = mbedtls_ecp_curve_list(); curve_info->grp_id != MBEDTLS_ECP_DP_NONE; curve_info++)
    {
        if (curve_info->tls_id == tls_id)
            return (curve_info);
    }

    return (NULL);
}

/*
 * Get the curve info from the name
 */
const mbedtls_ecp_curve_info *mbedtls_ecp_curve_info_from_name(const char *name)
{
    const mbedtls_ecp_curve_info *curve_info;

    if (name == NULL)
        return (NULL);

    for (curve_info = mbedtls_ecp_curve_list(); curve_info->grp_id != MBEDTLS_ECP_DP_NONE; curve_info++)
    {
        if (strcmp(curve_info->name, name) == 0)
            return (curve_info);
    }

    return (NULL);
}

/*
 * Get the type of a curve
 */
mbedtls_ecp_curve_type mbedtls_ecp_get_type(const mbedtls_ecp_group *grp)
{
    if (grp->G.X.p == NULL)
        return (MBEDTLS_ECP_TYPE_NONE);

    if (grp->G.Y.p == NULL)
        return (MBEDTLS_ECP_TYPE_MONTGOMERY);
    else
        return (MBEDTLS_ECP_TYPE_SHORT_WEIERSTRASS);
}

/*
 * Initialize (the components of) a point
 */
void mbedtls_ecp_point_init(mbedtls_ecp_point *pt)
{
    ECP_VALIDATE(pt != NULL);

    mbedtls_mpi_init(&pt->X);
    mbedtls_mpi_init(&pt->Y);
    mbedtls_mpi_init(&pt->Z);
}

/*
 * Initialize (the components of) a group
 */
void mbedtls_ecp_group_init(mbedtls_ecp_group *grp)
{
    ECP_VALIDATE(grp != NULL);

    grp->id = MBEDTLS_ECP_DP_NONE;
    mbedtls_mpi_init(&grp->P);
    mbedtls_mpi_init(&grp->A);
    mbedtls_mpi_init(&grp->B);
    mbedtls_ecp_point_init(&grp->G);
    mbedtls_mpi_init(&grp->N);
    grp->pbits = 0;
    grp->nbits = 0;
}

/*
 * Initialize (the components of) a key pair
 */
void mbedtls_ecp_keypair_init(mbedtls_ecp_keypair *key)
{
    ECP_VALIDATE(key != NULL);

    mbedtls_ecp_group_init(&key->grp);
    mbedtls_mpi_init(&key->d);
    mbedtls_ecp_point_init(&key->Q);
}

/*
 * Unallocate (the components of) a point
 */
void mbedtls_ecp_point_free(mbedtls_ecp_point *pt)
{
    if (pt == NULL)
        return;

    mbedtls_mpi_free(&(pt->X));
    mbedtls_mpi_free(&(pt->Y));
    mbedtls_mpi_free(&(pt->Z));
}

/*
 * Unallocate (the components of) a group
 */
void mbedtls_ecp_group_free(mbedtls_ecp_group *grp)
{
    if (grp == NULL)
        return;

    mbedtls_platform_zeroize(grp, sizeof(mbedtls_ecp_group));
}

/*
 * Unallocate (the components of) a key pair
 */
void mbedtls_ecp_keypair_free(mbedtls_ecp_keypair *key)
{
    if (key == NULL)
        return;

    mbedtls_ecp_group_free(&key->grp);
    mbedtls_mpi_free(&key->d);
    mbedtls_ecp_point_free(&key->Q);
}

/*
 * Copy the contents of a point
 */
int mbedtls_ecp_copy(mbedtls_ecp_point *P, const mbedtls_ecp_point *Q)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    ECP_VALIDATE_RET(P != NULL);
    ECP_VALIDATE_RET(Q != NULL);

    MBEDTLS_MPI_CHK(mbedtls_mpi_copy(&P->X, &Q->X));
    MBEDTLS_MPI_CHK(mbedtls_mpi_copy(&P->Y, &Q->Y));
    MBEDTLS_MPI_CHK(mbedtls_mpi_copy(&P->Z, &Q->Z));

cleanup:
    return (ret);
}

/*
 * Copy the contents of a group object
 */
int mbedtls_ecp_group_copy(mbedtls_ecp_group *dst, const mbedtls_ecp_group *src)
{
    ECP_VALIDATE_RET(dst != NULL);
    ECP_VALIDATE_RET(src != NULL);

    return (mbedtls_ecp_group_load(dst, src->id));
}

/*
 * Set point to zero
 */
int mbedtls_ecp_set_zero(mbedtls_ecp_point *pt)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    ECP_VALIDATE_RET(pt != NULL);

    MBEDTLS_MPI_CHK(mbedtls_mpi_lset(&pt->X, 1));
    MBEDTLS_MPI_CHK(mbedtls_mpi_lset(&pt->Y, 1));
    MBEDTLS_MPI_CHK(mbedtls_mpi_lset(&pt->Z, 0));

cleanup:
    return (ret);
}

/*
 * Tell if a point is zero
 */
int mbedtls_ecp_is_zero(mbedtls_ecp_point *pt)
{
    ECP_VALIDATE_RET(pt != NULL);

    return (mbedtls_mpi_cmp_int(&pt->Z, 0) == 0);
}

/*
 * Compare two points lazily
 */
int mbedtls_ecp_point_cmp(const mbedtls_ecp_point *P, const mbedtls_ecp_point *Q)
{
    ECP_VALIDATE_RET(P != NULL);
    ECP_VALIDATE_RET(Q != NULL);

    if (mbedtls_mpi_cmp_mpi(&P->X, &Q->X) == 0 && mbedtls_mpi_cmp_mpi(&P->Y, &Q->Y) == 0 &&
        mbedtls_mpi_cmp_mpi(&P->Z, &Q->Z) == 0)
    {
        return (0);
    }

    return (MBEDTLS_ERR_ECP_BAD_INPUT_DATA);
}

/*
 * Import a non-zero point from ASCII strings
 */
int mbedtls_ecp_point_read_string(mbedtls_ecp_point *P, int radix, const char *x, const char *y)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    ECP_VALIDATE_RET(P != NULL);
    ECP_VALIDATE_RET(x != NULL);
    ECP_VALIDATE_RET(y != NULL);

    MBEDTLS_MPI_CHK(mbedtls_mpi_read_string(&P->X, radix, x));
    MBEDTLS_MPI_CHK(mbedtls_mpi_read_string(&P->Y, radix, y));
    MBEDTLS_MPI_CHK(mbedtls_mpi_lset(&P->Z, 1));

cleanup:
    return (ret);
}

/*
 * Export a point into unsigned binary data (SEC1 2.3.3 and RFC7748)
 */
int mbedtls_ecp_point_write_binary(const mbedtls_ecp_group *grp,
                                   const mbedtls_ecp_point *P,
                                   int                      format,
                                   size_t                  *olen,
                                   unsigned char           *buf,
                                   size_t                   buflen)
{
    int    ret = MBEDTLS_ERR_ECP_FEATURE_UNAVAILABLE;
    size_t plen;
    ECP_VALIDATE_RET(grp != NULL);
    ECP_VALIDATE_RET(P != NULL);
    ECP_VALIDATE_RET(olen != NULL);
    ECP_VALIDATE_RET(buf != NULL);
    ECP_VALIDATE_RET(format == MBEDTLS_ECP_PF_UNCOMPRESSED || format == MBEDTLS_ECP_PF_COMPRESSED);

    plen = mbedtls_mpi_size(&grp->P);

#if defined(MBEDTLS_ECP_MONTGOMERY_ENABLED)
    (void)format; /* Montgomery curves always use the same point format */
    if (mbedtls_ecp_get_type(grp) == MBEDTLS_ECP_TYPE_MONTGOMERY)
    {
        *olen = plen;
        if (buflen < *olen)
            return (MBEDTLS_ERR_ECP_BUFFER_TOO_SMALL);

        MBEDTLS_MPI_CHK(mbedtls_mpi_write_binary_le(&P->X, buf, plen));
    }
#endif
#if defined(MBEDTLS_ECP_SHORT_WEIERSTRASS_ENABLED)
    if (mbedtls_ecp_get_type(grp) == MBEDTLS_ECP_TYPE_SHORT_WEIERSTRASS)
    {
        /*
         * Common case: P == 0
         */
        if (mbedtls_mpi_cmp_int(&P->Z, 0) == 0)
        {
            if (buflen < 1)
                return (MBEDTLS_ERR_ECP_BUFFER_TOO_SMALL);

            buf[0] = 0x00;
            *olen  = 1;

            return (0);
        }

        if (format == MBEDTLS_ECP_PF_UNCOMPRESSED)
        {
            *olen = 2 * plen + 1;

            if (buflen < *olen)
                return (MBEDTLS_ERR_ECP_BUFFER_TOO_SMALL);

            buf[0] = 0x04;
            MBEDTLS_MPI_CHK(mbedtls_mpi_write_binary(&P->X, buf + 1, plen));
            MBEDTLS_MPI_CHK(mbedtls_mpi_write_binary(&P->Y, buf + 1 + plen, plen));
        }
        else if (format == MBEDTLS_ECP_PF_COMPRESSED)
        {
            *olen = plen + 1;

            if (buflen < *olen)
                return (MBEDTLS_ERR_ECP_BUFFER_TOO_SMALL);

            buf[0] = 0x02 + mbedtls_mpi_get_bit(&P->Y, 0);
            MBEDTLS_MPI_CHK(mbedtls_mpi_write_binary(&P->X, buf + 1, plen));
        }
    }
#endif

cleanup:
    return (ret);
}

/*
 * Import a point from unsigned binary data (SEC1 2.3.4 and RFC7748)
 */
int mbedtls_ecp_point_read_binary(const mbedtls_ecp_group *grp,
                                  mbedtls_ecp_point       *pt,
                                  const unsigned char     *buf,
                                  size_t                   ilen)
{
    int    ret = MBEDTLS_ERR_ECP_FEATURE_UNAVAILABLE;
    size_t plen;
    ECP_VALIDATE_RET(grp != NULL);
    ECP_VALIDATE_RET(pt != NULL);
    ECP_VALIDATE_RET(buf != NULL);

    if (ilen < 1)
        return (MBEDTLS_ERR_ECP_BAD_INPUT_DATA);

    plen = mbedtls_mpi_size(&grp->P);

#if defined(MBEDTLS_ECP_MONTGOMERY_ENABLED)
    if (mbedtls_ecp_get_type(grp) == MBEDTLS_ECP_TYPE_MONTGOMERY)
    {
        if (plen != ilen)
            return (MBEDTLS_ERR_ECP_BAD_INPUT_DATA);

        MBEDTLS_MPI_CHK(mbedtls_mpi_read_binary_le(&pt->X, buf, plen));
        mbedtls_mpi_free(&pt->Y);

        if (grp->id == MBEDTLS_ECP_DP_CURVE25519)
            /* Set most significant bit to 0 as prescribed in RFC7748 §5 */
            MBEDTLS_MPI_CHK(mbedtls_mpi_set_bit(&pt->X, plen * 8 - 1, 0));

        MBEDTLS_MPI_CHK(mbedtls_mpi_lset(&pt->Z, 1));
    }
#endif
#if defined(MBEDTLS_ECP_SHORT_WEIERSTRASS_ENABLED)
    if (mbedtls_ecp_get_type(grp) == MBEDTLS_ECP_TYPE_SHORT_WEIERSTRASS)
    {
        if (buf[0] == 0x00)
        {
            if (ilen == 1)
                return (mbedtls_ecp_set_zero(pt));
            else
                return (MBEDTLS_ERR_ECP_BAD_INPUT_DATA);
        }

        if (buf[0] != 0x04)
            return (MBEDTLS_ERR_ECP_FEATURE_UNAVAILABLE);

        if (ilen != 2 * plen + 1)
            return (MBEDTLS_ERR_ECP_BAD_INPUT_DATA);

        MBEDTLS_MPI_CHK(mbedtls_mpi_read_binary(&pt->X, buf + 1, plen));
        MBEDTLS_MPI_CHK(mbedtls_mpi_read_binary(&pt->Y, buf + 1 + plen, plen));
        MBEDTLS_MPI_CHK(mbedtls_mpi_lset(&pt->Z, 1));
    }
#endif

cleanup:
    return (ret);
}

/*
 * Import a point from a TLS ECPoint record (RFC 4492)
 *      struct {
 *          opaque point <1..2^8-1>;
 *      } ECPoint;
 */
int mbedtls_ecp_tls_read_point(const mbedtls_ecp_group *grp,
                               mbedtls_ecp_point       *pt,
                               const unsigned char    **buf,
                               size_t                   buf_len)
{
    unsigned char        data_len;
    const unsigned char *buf_start;
    ECP_VALIDATE_RET(grp != NULL);
    ECP_VALIDATE_RET(pt != NULL);
    ECP_VALIDATE_RET(buf != NULL);
    ECP_VALIDATE_RET(*buf != NULL);

    /*
     * We must have at least two bytes (1 for length, at least one for data)
     */
    if (buf_len < 2)
        return (MBEDTLS_ERR_ECP_BAD_INPUT_DATA);

    data_len = *(*buf)++;
    if (data_len < 1 || data_len > buf_len - 1)
        return (MBEDTLS_ERR_ECP_BAD_INPUT_DATA);

    /*
     * Save buffer start for read_binary and update buf
     */
    buf_start = *buf;
    *buf += data_len;

    return (mbedtls_ecp_point_read_binary(grp, pt, buf_start, data_len));
}

/*
 * Export a point as a TLS ECPoint record (RFC 4492)
 *      struct {
 *          opaque point <1..2^8-1>;
 *      } ECPoint;
 */
int mbedtls_ecp_tls_write_point(const mbedtls_ecp_group *grp,
                                const mbedtls_ecp_point *pt,
                                int                      format,
                                size_t                  *olen,
                                unsigned char           *buf,
                                size_t                   blen)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    ECP_VALIDATE_RET(grp != NULL);
    ECP_VALIDATE_RET(pt != NULL);
    ECP_VALIDATE_RET(olen != NULL);
    ECP_VALIDATE_RET(buf != NULL);
    ECP_VALIDATE_RET(format == MBEDTLS_ECP_PF_UNCOMPRESSED || format == MBEDTLS_ECP_PF_COMPRESSED);

    /*
     * buffer length must be at least one, for our length byte
     */
    if (blen < 1)
        return (MBEDTLS_ERR_ECP_BAD_INPUT_DATA);

    if ((ret = mbedtls_ecp_point_write_binary(grp, pt, format, olen, buf + 1, blen - 1)) != 0)
        return (ret);

    /*
     * write length to the first byte and update total length
     */
    buf[0] = (unsigned char)*olen;
    ++*olen;

    return (0);
}

/*
 * Set a group from an ECParameters record (RFC 4492)
 */
int mbedtls_ecp_tls_read_group(mbedtls_ecp_group *grp, const unsigned char **buf, size_t len)
{
    int                  ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    mbedtls_ecp_group_id grp_id;
    ECP_VALIDATE_RET(grp != NULL);
    ECP_VALIDATE_RET(buf != NULL);
    ECP_VALIDATE_RET(*buf != NULL);

    if ((ret = mbedtls_ecp_tls_read_group_id(&grp_id, buf, len)) != 0)
        return (ret);

    return (mbedtls_ecp_group_load(grp, grp_id));
}

/*
 * Read a group id from an ECParameters record (RFC 4492) and convert it to
 * mbedtls_ecp_group_id.
 */
int mbedtls_ecp_tls_read_group_id(mbedtls_ecp_group_id *grp, const unsigned char **buf, size_t len)
{
    uint16_t                      tls_id;
    const mbedtls_ecp_curve_info *curve_info;
    ECP_VALIDATE_RET(grp != NULL);
    ECP_VALIDATE_RET(buf != NULL);
    ECP_VALIDATE_RET(*buf != NULL);

    /*
     * We expect at least three bytes (see below)
     */
    if (len < 3)
        return (MBEDTLS_ERR_ECP_BAD_INPUT_DATA);

    /*
     * First byte is curve_type; only named_curve is handled
     */
    if (*(*buf)++ != MBEDTLS_ECP_TLS_NAMED_CURVE)
        return (MBEDTLS_ERR_ECP_BAD_INPUT_DATA);

    /*
     * Next two bytes are the namedcurve value
     */
    tls_id = *(*buf)++;
    tls_id <<= 8;
    tls_id |= *(*buf)++;

    if ((curve_info = mbedtls_ecp_curve_info_from_tls_id(tls_id)) == NULL)
        return (MBEDTLS_ERR_ECP_FEATURE_UNAVAILABLE);

    *grp = curve_info->grp_id;

    return (0);
}

/*
 * Write the ECParameters record corresponding to a group (RFC 4492)
 */
int mbedtls_ecp_tls_write_group(const mbedtls_ecp_group *grp, size_t *olen, unsigned char *buf, size_t blen)
{
    const mbedtls_ecp_curve_info *curve_info;
    ECP_VALIDATE_RET(grp != NULL);
    ECP_VALIDATE_RET(buf != NULL);
    ECP_VALIDATE_RET(olen != NULL);

    if ((curve_info = mbedtls_ecp_curve_info_from_grp_id(grp->id)) == NULL)
        return (MBEDTLS_ERR_ECP_BAD_INPUT_DATA);

    /*
     * We are going to write 3 bytes (see below)
     */
    *olen = 3;
    if (blen < *olen)
        return (MBEDTLS_ERR_ECP_BUFFER_TOO_SMALL);

    /*
     * First byte is curve_type, always named_curve
     */
    *buf++ = MBEDTLS_ECP_TLS_NAMED_CURVE;

    /*
     * Next two bytes are the namedcurve value
     */
    buf[0] = (uint8_t)((curve_info->tls_id >> 8) & 0xff);
    buf[1] = (uint8_t)(curve_info->tls_id & 0xff);

    return (0);
}

/*
 * Fast mod-p functions expect their argument to be in the 0..p^2 range.
 *
 * In order to guarantee that, we need to ensure that operands of
 * mbedtls_mpi_mul_mpi are in the 0..p range. So, after each operation we will
 * bring the result back to this range.
 *
 * The following macros are shortcuts for doing that.
 */

/*
 * Reduce a mbedtls_mpi mod p in-place, general case, to use after mbedtls_mpi_mul_mpi
 */
#if defined(MBEDTLS_SELF_TEST)
#define INC_MUL_COUNT mul_count++;
#else
#define INC_MUL_COUNT
#endif

static inline int mbedtls_mpi_mul_mod(const mbedtls_ecp_group *grp,
                                      mbedtls_mpi             *X,
                                      const mbedtls_mpi       *A,
                                      const mbedtls_mpi       *B)
{
#define MAX_ECP_WORD 8

    int      ret                         = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    uint32_t tmp_a[MAX_ECP_WORD * 2 + 1] = {0};
    uint32_t tmp_b[MAX_ECP_WORD * 2 + 1] = {0};

    memcpy(tmp_a, A->p, A->n * sizeof(uint32_t));
    memcpy(tmp_b, B->p, B->n * sizeof(uint32_t));

    MBEDTLS_MPI_CHK(mbedtls_mpi_grow(X, grp->P.n * 2 + 1));

    if (wiced_hal_crypto_pka_mulmod(X->p, tmp_a, tmp_b, grp->P.p, grp->P.n) == WICED_CRYPTO_MULMOD_SUCCESS)
        ret = 0;

cleanup:
    return (ret);
}

/*
 * Reduce a mbedtls_mpi mod p in-place, to use after mbedtls_mpi_sub_mpi
 * N->s < 0 is a very fast test, which fails only if N is 0
 */
#define MOD_SUB(N) \
    while ((N).s < 0 && mbedtls_mpi_cmp_int(&(N), 0) != 0) MBEDTLS_MPI_CHK(mbedtls_mpi_add_mpi(&(N), &(N), &grp->P))

#if (defined(MBEDTLS_ECP_SHORT_WEIERSTRASS_ENABLED) &&                            \
     !(defined(MBEDTLS_ECP_NO_FALLBACK) && defined(MBEDTLS_ECP_DOUBLE_JAC_ALT) && \
       defined(MBEDTLS_ECP_ADD_MIXED_ALT))) ||                                    \
    (defined(MBEDTLS_ECP_MONTGOMERY_ENABLED) &&                                   \
     !(defined(MBEDTLS_ECP_NO_FALLBACK) && defined(MBEDTLS_ECP_DOUBLE_ADD_MXZ_ALT)))
static inline int mbedtls_mpi_sub_mod(const mbedtls_ecp_group *grp,
                                      mbedtls_mpi             *X,
                                      const mbedtls_mpi       *A,
                                      const mbedtls_mpi       *B)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    MBEDTLS_MPI_CHK(mbedtls_mpi_sub_mpi(X, A, B));
    MOD_SUB(*X);
cleanup:
    return (ret);
}
#endif /* All functions referencing mbedtls_mpi_sub_mod() are alt-implemented without fallback */

/*
 * Reduce a mbedtls_mpi mod p in-place, to use after mbedtls_mpi_add_mpi and mbedtls_mpi_mul_int.
 * We known P, N and the result are positive, so sub_abs is correct, and
 * a bit faster.
 */
#define MOD_ADD(N) \
    while (mbedtls_mpi_cmp_mpi(&(N), &grp->P) >= 0) MBEDTLS_MPI_CHK(mbedtls_mpi_sub_abs(&(N), &(N), &grp->P))

static inline int mbedtls_mpi_add_mod(const mbedtls_ecp_group *grp,
                                      mbedtls_mpi             *X,
                                      const mbedtls_mpi       *A,
                                      const mbedtls_mpi       *B)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    MBEDTLS_MPI_CHK(mbedtls_mpi_add_mpi(X, A, B));
    MOD_ADD(*X);
cleanup:
    return (ret);
}

#if defined(MBEDTLS_ECP_SHORT_WEIERSTRASS_ENABLED) && \
    !(defined(MBEDTLS_ECP_NO_FALLBACK) && defined(MBEDTLS_ECP_DOUBLE_JAC_ALT) && defined(MBEDTLS_ECP_ADD_MIXED_ALT))
static inline int mbedtls_mpi_shift_l_mod(const mbedtls_ecp_group *grp, mbedtls_mpi *X, size_t count)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    MBEDTLS_MPI_CHK(mbedtls_mpi_shift_l(X, count));
    MOD_ADD(*X);
cleanup:
    return (ret);
}
#endif /* All functions referencing mbedtls_mpi_shift_l_mod() are alt-implemented without fallback */

#if defined(MBEDTLS_ECP_SHORT_WEIERSTRASS_ENABLED)
/*
 * For curves in short Weierstrass form, we do all the internal operations in
 * Jacobian coordinates.
 *
 * For multiplication, we'll use a comb method with coutermeasueres against
 * SPA, hence timing attacks.
 */

/*
 * Normalize jacobian coordinates so that Z == 0 || Z == 1  (GECC 3.2.1)
 * Cost: 1N := 1I + 3M + 1S
 */
static int ecp_normalize_jac(const mbedtls_ecp_group *grp, mbedtls_ecp_point *pt)
{
    if (mbedtls_mpi_cmp_int(&pt->Z, 0) == 0)
        return (0);

#if defined(MBEDTLS_ECP_NORMALIZE_JAC_ALT)
    if (mbedtls_internal_ecp_grp_capable(grp))
        return (mbedtls_internal_ecp_normalize_jac(grp, pt));
#endif /* MBEDTLS_ECP_NORMALIZE_JAC_ALT */

#if defined(MBEDTLS_ECP_NO_FALLBACK) && defined(MBEDTLS_ECP_NORMALIZE_JAC_ALT)
    return (MBEDTLS_ERR_ECP_FEATURE_UNAVAILABLE);
#else
    int         ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    mbedtls_mpi Zi, ZZi;
    mbedtls_mpi_init(&Zi);
    mbedtls_mpi_init(&ZZi);

    /*
     * X = X / Z^2  mod p
     */
    MBEDTLS_MPI_CHK(mbedtls_mpi_inv_mod(&Zi, &pt->Z, &grp->P));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &ZZi, &Zi, &Zi));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &pt->X, &pt->X, &ZZi));

    /*
     * Y = Y / Z^3  mod p
     */
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &pt->Y, &pt->Y, &ZZi));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &pt->Y, &pt->Y, &Zi));

    /*
     * Z = 1
     */
    MBEDTLS_MPI_CHK(mbedtls_mpi_lset(&pt->Z, 1));

cleanup:
    mbedtls_mpi_free(&Zi);
    mbedtls_mpi_free(&ZZi);
    return (ret);

#endif /* !defined(MBEDTLS_ECP_NO_FALLBACK) || !defined(MBEDTLS_ECP_NORMALIZE_JAC_ALT) */
}

/*
 * Point doubling R = 2 P, Jacobian coordinates
 *
 * Based on http://www.hyperelliptic.org/EFD/g1p/auto-shortw-jacobian.html#doubling-dbl-1998-cmo-2 .
 *
 * We follow the variable naming fairly closely. The formula variations that trade a MUL for a SQR
 * (plus a few ADDs) aren't useful as our bignum implementation doesn't distinguish squaring.
 *
 * Standard optimizations are applied when curve parameter A is one of { 0, -3 }.
 *
 * Cost: 1D := 3M + 4S          (A ==  0)
 *             4M + 4S          (A == -3)
 *             3M + 6S + 1a     otherwise
 */
static int ecp_double_jac(const mbedtls_ecp_group *grp, mbedtls_ecp_point *R, const mbedtls_ecp_point *P)
{
#if defined(MBEDTLS_SELF_TEST)
    dbl_count++;
#endif

#if defined(MBEDTLS_ECP_DOUBLE_JAC_ALT)
    if (mbedtls_internal_ecp_grp_capable(grp))
        return (mbedtls_internal_ecp_double_jac(grp, R, P));
#endif /* MBEDTLS_ECP_DOUBLE_JAC_ALT */

#if defined(MBEDTLS_ECP_NO_FALLBACK) && defined(MBEDTLS_ECP_DOUBLE_JAC_ALT)
    return (MBEDTLS_ERR_ECP_FEATURE_UNAVAILABLE);
#else
    int         ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    mbedtls_mpi M, S, T, U;

    mbedtls_mpi_init(&M);
    mbedtls_mpi_init(&S);
    mbedtls_mpi_init(&T);
    mbedtls_mpi_init(&U);

    /* Special case for A = -3 */
    if (grp->A.p == NULL)
    {
        /* M = 3(X + Z^2)(X - Z^2) */
        MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &S, &P->Z, &P->Z));
        MBEDTLS_MPI_CHK(mbedtls_mpi_add_mod(grp, &T, &P->X, &S));
        MBEDTLS_MPI_CHK(mbedtls_mpi_sub_mod(grp, &U, &P->X, &S));
        MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &S, &T, &U));
        MBEDTLS_MPI_CHK(mbedtls_mpi_mul_int(&M, &S, 3));
        MOD_ADD(M);
    }
    else
    {
        /* M = 3.X^2 */
        MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &S, &P->X, &P->X));
        MBEDTLS_MPI_CHK(mbedtls_mpi_mul_int(&M, &S, 3));
        MOD_ADD(M);

        /* Optimize away for "koblitz" curves with A = 0 */
        if (mbedtls_mpi_cmp_int(&grp->A, 0) != 0)
        {
            /* M += A.Z^4 */
            MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &S, &P->Z, &P->Z));
            MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &T, &S, &S));
            MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &S, &T, &grp->A));
            MBEDTLS_MPI_CHK(mbedtls_mpi_add_mod(grp, &M, &M, &S));
        }
    }

    /* S = 4.X.Y^2 */
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &T, &P->Y, &P->Y));
    MBEDTLS_MPI_CHK(mbedtls_mpi_shift_l_mod(grp, &T, 1));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &S, &P->X, &T));
    MBEDTLS_MPI_CHK(mbedtls_mpi_shift_l_mod(grp, &S, 1));

    /* U = 8.Y^4 */
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &U, &T, &T));
    MBEDTLS_MPI_CHK(mbedtls_mpi_shift_l_mod(grp, &U, 1));

    /* T = M^2 - 2.S */
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &T, &M, &M));
    MBEDTLS_MPI_CHK(mbedtls_mpi_sub_mod(grp, &T, &T, &S));
    MBEDTLS_MPI_CHK(mbedtls_mpi_sub_mod(grp, &T, &T, &S));

    /* S = M(S - T) - U */
    MBEDTLS_MPI_CHK(mbedtls_mpi_sub_mod(grp, &S, &S, &T));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &S, &S, &M));
    MBEDTLS_MPI_CHK(mbedtls_mpi_sub_mod(grp, &S, &S, &U));

    /* U = 2.Y.Z */
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &U, &P->Y, &P->Z));
    MBEDTLS_MPI_CHK(mbedtls_mpi_shift_l_mod(grp, &U, 1));

    MBEDTLS_MPI_CHK(mbedtls_mpi_copy(&R->X, &T));
    MBEDTLS_MPI_CHK(mbedtls_mpi_copy(&R->Y, &S));
    MBEDTLS_MPI_CHK(mbedtls_mpi_copy(&R->Z, &U));

cleanup:
    mbedtls_mpi_free(&M);
    mbedtls_mpi_free(&S);
    mbedtls_mpi_free(&T);
    mbedtls_mpi_free(&U);

    return (ret);
#endif /* !defined(MBEDTLS_ECP_NO_FALLBACK) || !defined(MBEDTLS_ECP_DOUBLE_JAC_ALT) */
}

/*
 * Addition: R = P + Q, mixed affine-Jacobian coordinates (GECC 3.22)
 *
 * The coordinates of Q must be normalized (= affine),
 * but those of P don't need to. R is not normalized.
 *
 * Special cases: (1) P or Q is zero, (2) R is zero, (3) P == Q.
 * None of these cases can happen as intermediate step in ecp_mul_comb():
 * - at each step, P, Q and R are multiples of the base point, the factor
 *   being less than its order, so none of them is zero;
 * - Q is an odd multiple of the base point, P an even multiple,
 *   due to the choice of precomputed points in the modified comb method.
 * So branches for these cases do not leak secret information.
 *
 * We accept Q->Z being unset (saving memory in tables) as meaning 1.
 *
 * Cost: 1A := 8M + 3S
 */
static int ecp_add_mixed(const mbedtls_ecp_group *grp,
                         mbedtls_ecp_point       *R,
                         const mbedtls_ecp_point *P,
                         const mbedtls_ecp_point *Q)
{
#if defined(MBEDTLS_SELF_TEST)
    add_count++;
#endif

#if defined(MBEDTLS_ECP_ADD_MIXED_ALT)
    if (mbedtls_internal_ecp_grp_capable(grp))
        return (mbedtls_internal_ecp_add_mixed(grp, R, P, Q));
#endif /* MBEDTLS_ECP_ADD_MIXED_ALT */

#if defined(MBEDTLS_ECP_NO_FALLBACK) && defined(MBEDTLS_ECP_ADD_MIXED_ALT)
    return (MBEDTLS_ERR_ECP_FEATURE_UNAVAILABLE);
#else
    int         ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    mbedtls_mpi T1, T2, T3, T4, X, Y, Z;

    /*
     * Trivial cases: P == 0 or Q == 0 (case 1)
     */
    if (mbedtls_mpi_cmp_int(&P->Z, 0) == 0)
        return (mbedtls_ecp_copy(R, Q));

    if (Q->Z.p != NULL && mbedtls_mpi_cmp_int(&Q->Z, 0) == 0)
        return (mbedtls_ecp_copy(R, P));

    /*
     * Make sure Q coordinates are normalized
     */
    if (Q->Z.p != NULL && mbedtls_mpi_cmp_int(&Q->Z, 1) != 0)
        return (MBEDTLS_ERR_ECP_BAD_INPUT_DATA);

    mbedtls_mpi_init(&T1);
    mbedtls_mpi_init(&T2);
    mbedtls_mpi_init(&T3);
    mbedtls_mpi_init(&T4);
    mbedtls_mpi_init(&X);
    mbedtls_mpi_init(&Y);
    mbedtls_mpi_init(&Z);

    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &T1, &P->Z, &P->Z));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &T2, &T1, &P->Z));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &T1, &T1, &Q->X));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &T2, &T2, &Q->Y));
    MBEDTLS_MPI_CHK(mbedtls_mpi_sub_mod(grp, &T1, &T1, &P->X));
    MBEDTLS_MPI_CHK(mbedtls_mpi_sub_mod(grp, &T2, &T2, &P->Y));

    /* Special cases (2) and (3) */
    if (mbedtls_mpi_cmp_int(&T1, 0) == 0)
    {
        if (mbedtls_mpi_cmp_int(&T2, 0) == 0)
        {
            ret = ecp_double_jac(grp, R, P);
            goto cleanup;
        }
        else
        {
            ret = mbedtls_ecp_set_zero(R);
            goto cleanup;
        }
    }

    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &Z, &P->Z, &T1));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &T3, &T1, &T1));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &T4, &T3, &T1));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &T3, &T3, &P->X));
    MBEDTLS_MPI_CHK(mbedtls_mpi_copy(&T1, &T3));
    MBEDTLS_MPI_CHK(mbedtls_mpi_shift_l_mod(grp, &T1, 1));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &X, &T2, &T2));
    MBEDTLS_MPI_CHK(mbedtls_mpi_sub_mod(grp, &X, &X, &T1));
    MBEDTLS_MPI_CHK(mbedtls_mpi_sub_mod(grp, &X, &X, &T4));
    MBEDTLS_MPI_CHK(mbedtls_mpi_sub_mod(grp, &T3, &T3, &X));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &T3, &T3, &T2));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &T4, &T4, &P->Y));
    MBEDTLS_MPI_CHK(mbedtls_mpi_sub_mod(grp, &Y, &T3, &T4));

    MBEDTLS_MPI_CHK(mbedtls_mpi_copy(&R->X, &X));
    MBEDTLS_MPI_CHK(mbedtls_mpi_copy(&R->Y, &Y));
    MBEDTLS_MPI_CHK(mbedtls_mpi_copy(&R->Z, &Z));

cleanup:
    mbedtls_mpi_free(&T1);
    mbedtls_mpi_free(&T2);
    mbedtls_mpi_free(&T3);
    mbedtls_mpi_free(&T4);
    mbedtls_mpi_free(&X);
    mbedtls_mpi_free(&Y);
    mbedtls_mpi_free(&Z);

    return (ret);
#endif /* !defined(MBEDTLS_ECP_NO_FALLBACK) || !defined(MBEDTLS_ECP_ADD_MIXED_ALT) */
}

/*
 * Check and define parameters used by the comb method (see below for details)
 */
#if MBEDTLS_ECP_WINDOW_SIZE < 2 || MBEDTLS_ECP_WINDOW_SIZE > 7
#error "MBEDTLS_ECP_WINDOW_SIZE out of bounds"
#endif

#endif /* MBEDTLS_ECP_SHORT_WEIERSTRASS_ENABLED */

#if defined(MBEDTLS_ECP_MONTGOMERY_ENABLED)
/*
 * For Montgomery curves, we do all the internal arithmetic in projective
 * coordinates. Import/export of points uses only the x coordinates, which is
 * internaly represented as X / Z.
 *
 * For scalar multiplication, we'll use a Montgomery ladder.
 */

/*
 * Normalize Montgomery x/z coordinates: X = X/Z, Z = 1
 * Cost: 1M + 1I
 */
static int ecp_normalize_mxz(const mbedtls_ecp_group *grp, mbedtls_ecp_point *P)
{
#if defined(MBEDTLS_ECP_NORMALIZE_MXZ_ALT)
    if (mbedtls_internal_ecp_grp_capable(grp))
        return (mbedtls_internal_ecp_normalize_mxz(grp, P));
#endif /* MBEDTLS_ECP_NORMALIZE_MXZ_ALT */

#if defined(MBEDTLS_ECP_NO_FALLBACK) && defined(MBEDTLS_ECP_NORMALIZE_MXZ_ALT)
    return (MBEDTLS_ERR_ECP_FEATURE_UNAVAILABLE);
#else
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    MBEDTLS_MPI_CHK(mbedtls_mpi_inv_mod(&P->Z, &P->Z, &grp->P));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &P->X, &P->X, &P->Z));
    MBEDTLS_MPI_CHK(mbedtls_mpi_lset(&P->Z, 1));

cleanup:
    return (ret);
#endif /* !defined(MBEDTLS_ECP_NO_FALLBACK) || !defined(MBEDTLS_ECP_NORMALIZE_MXZ_ALT) */
}

/*
 * Randomize projective x/z coordinates:
 * (X, Z) -> (l X, l Z) for random l
 * This is sort of the reverse operation of ecp_normalize_mxz().
 *
 * This countermeasure was first suggested in [2].
 * Cost: 2M
 */
static int ecp_randomize_mxz(const mbedtls_ecp_group *grp,
                             mbedtls_ecp_point       *P,
                             int (*f_rng)(void *, unsigned char *, size_t),
                             void *p_rng)
{
#if defined(MBEDTLS_ECP_RANDOMIZE_MXZ_ALT)
    if (mbedtls_internal_ecp_grp_capable(grp))
        return (mbedtls_internal_ecp_randomize_mxz(grp, P, f_rng, p_rng));
#endif /* MBEDTLS_ECP_RANDOMIZE_MXZ_ALT */

#if defined(MBEDTLS_ECP_NO_FALLBACK) && defined(MBEDTLS_ECP_RANDOMIZE_MXZ_ALT)
    return (MBEDTLS_ERR_ECP_FEATURE_UNAVAILABLE);
#else
    int         ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    mbedtls_mpi l;
    mbedtls_mpi_init(&l);

    /* Generate l such that 1 < l < p */
    MBEDTLS_MPI_CHK(mbedtls_mpi_random(&l, 2, &grp->P, f_rng, p_rng));

    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &P->X, &P->X, &l));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &P->Z, &P->Z, &l));

cleanup:
    mbedtls_mpi_free(&l);

    if (ret == MBEDTLS_ERR_MPI_NOT_ACCEPTABLE)
        ret = MBEDTLS_ERR_ECP_RANDOM_FAILED;
    return (ret);
#endif /* !defined(MBEDTLS_ECP_NO_FALLBACK) || !defined(MBEDTLS_ECP_RANDOMIZE_MXZ_ALT) */
}

/*
 * Double-and-add: R = 2P, S = P + Q, with d = X(P - Q),
 * for Montgomery curves in x/z coordinates.
 *
 * http://www.hyperelliptic.org/EFD/g1p/auto-code/montgom/xz/ladder/mladd-1987-m.op3
 * with
 * d =  X1
 * P = (X2, Z2)
 * Q = (X3, Z3)
 * R = (X4, Z4)
 * S = (X5, Z5)
 * and eliminating temporary variables tO, ..., t4.
 *
 * Cost: 5M + 4S
 */
static int ecp_double_add_mxz(const mbedtls_ecp_group *grp,
                              mbedtls_ecp_point       *R,
                              mbedtls_ecp_point       *S,
                              const mbedtls_ecp_point *P,
                              const mbedtls_ecp_point *Q,
                              const mbedtls_mpi       *d)
{
#if defined(MBEDTLS_ECP_DOUBLE_ADD_MXZ_ALT)
    if (mbedtls_internal_ecp_grp_capable(grp))
        return (mbedtls_internal_ecp_double_add_mxz(grp, R, S, P, Q, d));
#endif /* MBEDTLS_ECP_DOUBLE_ADD_MXZ_ALT */

#if defined(MBEDTLS_ECP_NO_FALLBACK) && defined(MBEDTLS_ECP_DOUBLE_ADD_MXZ_ALT)
    return (MBEDTLS_ERR_ECP_FEATURE_UNAVAILABLE);
#else
    int         ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    mbedtls_mpi A, AA, B, BB, E, C, D, DA, CB;

    mbedtls_mpi_init(&A);
    mbedtls_mpi_init(&AA);
    mbedtls_mpi_init(&B);
    mbedtls_mpi_init(&BB);
    mbedtls_mpi_init(&E);
    mbedtls_mpi_init(&C);
    mbedtls_mpi_init(&D);
    mbedtls_mpi_init(&DA);
    mbedtls_mpi_init(&CB);

    MBEDTLS_MPI_CHK(mbedtls_mpi_add_mod(grp, &A, &P->X, &P->Z));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &AA, &A, &A));
    MBEDTLS_MPI_CHK(mbedtls_mpi_sub_mod(grp, &B, &P->X, &P->Z));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &BB, &B, &B));
    MBEDTLS_MPI_CHK(mbedtls_mpi_sub_mod(grp, &E, &AA, &BB));
    MBEDTLS_MPI_CHK(mbedtls_mpi_add_mod(grp, &C, &Q->X, &Q->Z));
    MBEDTLS_MPI_CHK(mbedtls_mpi_sub_mod(grp, &D, &Q->X, &Q->Z));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &DA, &D, &A));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &CB, &C, &B));
    MBEDTLS_MPI_CHK(mbedtls_mpi_add_mod(grp, &S->X, &DA, &CB));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &S->X, &S->X, &S->X));
    MBEDTLS_MPI_CHK(mbedtls_mpi_sub_mod(grp, &S->Z, &DA, &CB));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &S->Z, &S->Z, &S->Z));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &S->Z, d, &S->Z));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &R->X, &AA, &BB));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &R->Z, &grp->A, &E));
    MBEDTLS_MPI_CHK(mbedtls_mpi_add_mod(grp, &R->Z, &BB, &R->Z));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &R->Z, &E, &R->Z));

cleanup:
    mbedtls_mpi_free(&A);
    mbedtls_mpi_free(&AA);
    mbedtls_mpi_free(&B);
    mbedtls_mpi_free(&BB);
    mbedtls_mpi_free(&E);
    mbedtls_mpi_free(&C);
    mbedtls_mpi_free(&D);
    mbedtls_mpi_free(&DA);
    mbedtls_mpi_free(&CB);

    return (ret);
#endif /* !defined(MBEDTLS_ECP_NO_FALLBACK) || !defined(MBEDTLS_ECP_DOUBLE_ADD_MXZ_ALT) */
}

/*
 * Multiplication with Montgomery ladder in x/z coordinates,
 * for curves in Montgomery form
 */
static int ecp_mul_mxz(mbedtls_ecp_group       *grp,
                       mbedtls_ecp_point       *R,
                       const mbedtls_mpi       *m,
                       const mbedtls_ecp_point *P,
                       int (*f_rng)(void *, unsigned char *, size_t),
                       void *p_rng)
{
    int               ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    size_t            i;
    unsigned char     b;
    mbedtls_ecp_point RP;
    mbedtls_mpi       PX;
#if !defined(MBEDTLS_ECP_NO_INTERNAL_RNG)
    ecp_drbg_context drbg_ctx;

    ecp_drbg_init(&drbg_ctx);
#endif
    mbedtls_ecp_point_init(&RP);
    mbedtls_mpi_init(&PX);

#if !defined(MBEDTLS_ECP_NO_INTERNAL_RNG)
    if (f_rng == NULL)
    {
        const size_t m_len = (grp->nbits + 7) / 8;
        MBEDTLS_MPI_CHK(ecp_drbg_seed(&drbg_ctx, m, m_len));
        f_rng = &ecp_drbg_random;
        p_rng = &drbg_ctx;
    }
#endif /* !MBEDTLS_ECP_NO_INTERNAL_RNG */

    /* Save PX and read from P before writing to R, in case P == R */
    MBEDTLS_MPI_CHK(mbedtls_mpi_copy(&PX, &P->X));
    MBEDTLS_MPI_CHK(mbedtls_ecp_copy(&RP, P));

    /* Set R to zero in modified x/z coordinates */
    MBEDTLS_MPI_CHK(mbedtls_mpi_lset(&R->X, 1));
    MBEDTLS_MPI_CHK(mbedtls_mpi_lset(&R->Z, 0));
    mbedtls_mpi_free(&R->Y);

    /* RP.X might be sligtly larger than P, so reduce it */
    MOD_ADD(RP.X);

    /* Randomize coordinates of the starting point */
#if defined(MBEDTLS_ECP_NO_INTERNAL_RNG)
    if (f_rng != NULL)
#endif
        MBEDTLS_MPI_CHK(ecp_randomize_mxz(grp, &RP, f_rng, p_rng));

    /* Loop invariant: R = result so far, RP = R + P */
    i = mbedtls_mpi_bitlen(m); /* one past the (zero-based) most significant bit */
    while (i-- > 0)
    {
        b = mbedtls_mpi_get_bit(m, i);
        /*
         *  if (b) R = 2R + P else R = 2R,
         * which is:
         *  if (b) double_add(RP, R, RP, R)
         *  else   double_add(R, RP, R, RP)
         * but using safe conditional swaps to avoid leaks
         */
        MBEDTLS_MPI_CHK(mbedtls_mpi_safe_cond_swap(&R->X, &RP.X, b));
        MBEDTLS_MPI_CHK(mbedtls_mpi_safe_cond_swap(&R->Z, &RP.Z, b));
        MBEDTLS_MPI_CHK(ecp_double_add_mxz(grp, R, &RP, R, &RP, &PX));
        MBEDTLS_MPI_CHK(mbedtls_mpi_safe_cond_swap(&R->X, &RP.X, b));
        MBEDTLS_MPI_CHK(mbedtls_mpi_safe_cond_swap(&R->Z, &RP.Z, b));
    }

    /*
     * Knowledge of the projective coordinates may leak the last few bits of the
     * scalar [1], and since our MPI implementation isn't constant-flow,
     * inversion (used for coordinate normalization) may leak the full value
     * of its input via side-channels [2].
     *
     * [1] https://eprint.iacr.org/2003/191
     * [2] https://eprint.iacr.org/2020/055
     *
     * Avoid the leak by randomizing coordinates before we normalize them.
     */
#if defined(MBEDTLS_ECP_NO_INTERNAL_RNG)
    if (f_rng != NULL)
#endif
        MBEDTLS_MPI_CHK(ecp_randomize_mxz(grp, R, f_rng, p_rng));

    MBEDTLS_MPI_CHK(ecp_normalize_mxz(grp, R));

cleanup:
#if !defined(MBEDTLS_ECP_NO_INTERNAL_RNG)
    ecp_drbg_free(&drbg_ctx);
#endif

    mbedtls_ecp_point_free(&RP);
    mbedtls_mpi_free(&PX);

    return (ret);
}

#endif /* MBEDTLS_ECP_MONTGOMERY_ENABLED */
static wiced_event_flags_t *p_handle = NULL;
static wiced_mutex_t       *mutex    = NULL;
#define ECP_MULT_EVENT_CODE (1 << 0)
static void ecc_point_mul_cb(wiced_crypto_ecc_point_mul_t *p_ecc_pt_mul)
{
    if (p_ecc_pt_mul->status == WICED_CRYPTO_ECC_POINT_MUL_SUCCESS)
    {
        wiced_rtos_set_event_flags(p_handle, ECP_MULT_EVENT_CODE);
    }
}
static void ecp_alt_rcopy(uint32_t *dst, uint32_t *src, uint32_t size)
{
    do
    {
        uint32_t *__dst  = dst;
        uint32_t  __size = size;
        uint32_t *__src  = src + __size - 1;
        uint32_t  __i;
        for (__i = 0; __i < __size; __i++)
        {
            *__dst = *__src;
            __dst++;
            __src--;
        }
    } while (0);
}
#define HW_ECP_MULT_CHK(f)                                                                     \
    do                                                                                         \
    {                                                                                          \
        if (f == WICED_CRYPTO_ECC_POINT_MUL_FAIL)                                              \
        {                                                                                      \
            ret = MBEDTLS_ERR_ECP_HW_ACCEL_FAILED;                                             \
            mbedtls_printf("wiced_hal_crypto_ecc_point_mul() error code: %lu", (uint32_t)ret); \
            goto cleanup;                                                                      \
        }                                                                                      \
    } while (0)
/*
 * Restartable multiplication R = m * P
 */
int mbedtls_ecp_mul_restartable(mbedtls_ecp_group       *grp,
                                mbedtls_ecp_point       *R,
                                const mbedtls_mpi       *m,
                                const mbedtls_ecp_point *P,
                                int (*f_rng)(void *, unsigned char *, size_t),
                                void                    *p_rng,
                                mbedtls_ecp_restart_ctx *rs_ctx)
{
    int ret = MBEDTLS_ERR_ECP_BAD_INPUT_DATA;

    ECP_VALIDATE_RET(grp != NULL);
    ECP_VALIDATE_RET(R != NULL);
    ECP_VALIDATE_RET(m != NULL);
    ECP_VALIDATE_RET(P != NULL);

    (void)rs_ctx;
    (void)f_rng;
    (void)p_rng;

    /* only for p256r1 (TODO:add other curves' parameters) */
    uint32_t reverse_curve_a[] = {0xffffffff, 0x00000001, 0x00000000, 0x00000000,
                                  0x00000000, 0xffffffff, 0xffffffff, 0xfffffffc};

    wiced_crypto_ecc_point_mul_t p_ecc_pt_mul = {0};
    mbedtls_mpi                  reverse_curve_b, reverse_curve_p;
    mbedtls_mpi                  reverse_m;
    mbedtls_ecp_point            reverse_P, reverse_tmp_result;
    wiced_result_t               result_event;
    uint32_t                     flags_set = 0;

    /* PKA driver in ROM needs "grp->P.n" bytes for P->Z, but mbedtls only has 1 byte */
    mbedtls_mpi P_Z;

    /* create mutex */
    if (mutex == NULL)
    {
        mutex = wiced_rtos_create_mutex();
        if (mutex == NULL)
        {
            goto cleanup;
        }
        wiced_rtos_init_mutex(mutex);
    }

    /* Initialized */
    mbedtls_ecp_point_init(&reverse_P);
    mbedtls_ecp_point_init(&reverse_tmp_result);
    mbedtls_mpi_init(&reverse_curve_b);
    mbedtls_mpi_init(&reverse_curve_p);
    mbedtls_mpi_init(&reverse_m);
    mbedtls_mpi_init(&P_Z);

    uint32_t size = grp->P.n;

    MBEDTLS_MPI_CHK(mbedtls_mpi_grow(&reverse_P.X, size));
    MBEDTLS_MPI_CHK(mbedtls_mpi_grow(&reverse_P.Y, size));
    MBEDTLS_MPI_CHK(mbedtls_mpi_grow(&reverse_P.Z, size));
    MBEDTLS_MPI_CHK(mbedtls_mpi_grow(&reverse_tmp_result.X, size));
    MBEDTLS_MPI_CHK(mbedtls_mpi_grow(&reverse_tmp_result.Y, size));
    MBEDTLS_MPI_CHK(mbedtls_mpi_grow(&reverse_curve_b, size));
    MBEDTLS_MPI_CHK(mbedtls_mpi_grow(&reverse_curve_p, size));
    MBEDTLS_MPI_CHK(mbedtls_mpi_grow(&reverse_m, size));
    MBEDTLS_MPI_CHK(mbedtls_mpi_grow(&P_Z, size));

    P_Z.p[0] = 1;

    /* Reverse input data*/
    ecp_alt_rcopy(reverse_P.X.p, P->X.p, size);
    ecp_alt_rcopy(reverse_P.Y.p, P->Y.p, size);
    ecp_alt_rcopy(reverse_P.Z.p, P_Z.p, size);
    ecp_alt_rcopy(reverse_m.p, m->p, size);
    ecp_alt_rcopy(reverse_curve_b.p, grp->B.p, size);
    ecp_alt_rcopy(reverse_curve_p.p, grp->P.p, size);

    /* setup the parameters of PKA driver in ROM */
    p_ecc_pt_mul.hw_pka_skip_check = 0;
    p_ecc_pt_mul.cur_a             = reverse_curve_a;
    p_ecc_pt_mul.cur_b             = reverse_curve_b.p;
    p_ecc_pt_mul.cur_p             = reverse_curve_p.p;
    p_ecc_pt_mul.src_x             = reverse_P.X.p;
    p_ecc_pt_mul.src_y             = reverse_P.Y.p;
    p_ecc_pt_mul.src_z             = reverse_P.Z.p;
    p_ecc_pt_mul.src_len           = grp->P.n;
    p_ecc_pt_mul.k_len             = grp->P.n;
    p_ecc_pt_mul.order             = -1;
    p_ecc_pt_mul.dest_x            = reverse_tmp_result.X.p;
    p_ecc_pt_mul.dest_y            = reverse_tmp_result.Y.p;
    p_ecc_pt_mul.dest_z            = 0;
    p_ecc_pt_mul.k                 = reverse_m.p;
    p_ecc_pt_mul.callback          = ecc_point_mul_cb;
    p_ecc_pt_mul.status            = 0xff;

    wiced_rtos_lock_mutex(mutex);

    /* PKA Event init function*/
    if (p_handle == NULL)
    {
        /* Create PKA event flag handle. */
        p_handle = wiced_rtos_create_event_flags();
        if (!p_handle)
        {
            printf("%s: Fail to create PKA event flag handle.\n", __FUNCTION__);
            wiced_rtos_unlock_mutex(mutex);
            goto cleanup;
        }
        /* Initialize the PKA event flags. */
        result_event = wiced_rtos_init_event_flags(p_handle);
        if (result_event != WICED_SUCCESS)
        {
            printf("%s: Fail to init. PKA event flags.\n", __FUNCTION__);
            wiced_rtos_unlock_mutex(mutex);
            goto cleanup;
        }
    }

    /* start PKA engine */
    HW_ECP_MULT_CHK(wiced_hal_crypto_ecc_point_mul(&p_ecc_pt_mul));

    /* PKA has finished whole ECP_MUL operations */
    wiced_rtos_wait_for_event_flags(p_handle, ECP_MULT_EVENT_CODE, &flags_set, WICED_TRUE, WAIT_FOR_ALL_EVENTS,
                                    WICED_WAIT_FOREVER);

    wiced_rtos_unlock_mutex(mutex);

    /* fill result to mbedtls */
    MBEDTLS_MPI_CHK(mbedtls_mpi_grow(&R->X, size));
    ecp_alt_rcopy(R->X.p, reverse_tmp_result.X.p, size);

    MBEDTLS_MPI_CHK(mbedtls_mpi_grow(&R->Y, size));
    ecp_alt_rcopy(R->Y.p, reverse_tmp_result.Y.p, size);

    MBEDTLS_MPI_CHK(mbedtls_mpi_lset(&R->Z, 1));

    mbedtls_ecp_point_free(&reverse_P);
    mbedtls_ecp_point_free(&reverse_tmp_result);
    mbedtls_mpi_free(&reverse_curve_b);
    mbedtls_mpi_free(&reverse_curve_p);
    mbedtls_mpi_free(&reverse_m);

cleanup:
    return ret;
}

/*
 * Multiplication R = m * P
 */
int mbedtls_ecp_mul(mbedtls_ecp_group       *grp,
                    mbedtls_ecp_point       *R,
                    const mbedtls_mpi       *m,
                    const mbedtls_ecp_point *P,
                    int (*f_rng)(void *, unsigned char *, size_t),
                    void *p_rng)
{
    ECP_VALIDATE_RET(grp != NULL);
    ECP_VALIDATE_RET(R != NULL);
    ECP_VALIDATE_RET(m != NULL);
    ECP_VALIDATE_RET(P != NULL);
    return (mbedtls_ecp_mul_restartable(grp, R, m, P, f_rng, p_rng, NULL));
}

#if defined(MBEDTLS_ECP_SHORT_WEIERSTRASS_ENABLED)
/*
 * Check that an affine point is valid as a public key,
 * short weierstrass curves (SEC1 3.2.3.1)
 */
static int ecp_check_pubkey_sw(const mbedtls_ecp_group *grp, const mbedtls_ecp_point *pt)
{
    int         ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    mbedtls_mpi YY, RHS;

    /* pt coordinates must be normalized for our checks */
    if (mbedtls_mpi_cmp_int(&pt->X, 0) < 0 || mbedtls_mpi_cmp_int(&pt->Y, 0) < 0 ||
        mbedtls_mpi_cmp_mpi(&pt->X, &grp->P) >= 0 || mbedtls_mpi_cmp_mpi(&pt->Y, &grp->P) >= 0)
        return (MBEDTLS_ERR_ECP_INVALID_KEY);

    mbedtls_mpi_init(&YY);
    mbedtls_mpi_init(&RHS);

    /*
     * YY = Y^2
     * RHS = X (X^2 + A) + B = X^3 + A X + B
     */
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &YY, &pt->Y, &pt->Y));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &RHS, &pt->X, &pt->X));

    /* Special case for A = -3 */
    if (grp->A.p == NULL)
    {
        MBEDTLS_MPI_CHK(mbedtls_mpi_sub_int(&RHS, &RHS, 3));
        MOD_SUB(RHS);
    }
    else
    {
        MBEDTLS_MPI_CHK(mbedtls_mpi_add_mod(grp, &RHS, &RHS, &grp->A));
    }

    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mod(grp, &RHS, &RHS, &pt->X));
    MBEDTLS_MPI_CHK(mbedtls_mpi_add_mod(grp, &RHS, &RHS, &grp->B));

    if (mbedtls_mpi_cmp_mpi(&YY, &RHS) != 0)
        ret = MBEDTLS_ERR_ECP_INVALID_KEY;

cleanup:
    mbedtls_mpi_free(&YY);
    mbedtls_mpi_free(&RHS);

    return (ret);
}
#endif /* MBEDTLS_ECP_SHORT_WEIERSTRASS_ENABLED */

#if defined(MBEDTLS_ECP_SHORT_WEIERSTRASS_ENABLED)
/*
 * R = m * P with shortcuts for m == 0, m == 1 and m == -1
 * NOT constant-time - ONLY for short Weierstrass!
 */
static int mbedtls_ecp_mul_shortcuts(mbedtls_ecp_group       *grp,
                                     mbedtls_ecp_point       *R,
                                     const mbedtls_mpi       *m,
                                     const mbedtls_ecp_point *P,
                                     mbedtls_ecp_restart_ctx *rs_ctx)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;

    if (mbedtls_mpi_cmp_int(m, 0) == 0)
    {
        MBEDTLS_MPI_CHK(mbedtls_ecp_set_zero(R));
    }
    else if (mbedtls_mpi_cmp_int(m, 1) == 0)
    {
        MBEDTLS_MPI_CHK(mbedtls_ecp_copy(R, P));
    }
    else if (mbedtls_mpi_cmp_int(m, -1) == 0)
    {
        MBEDTLS_MPI_CHK(mbedtls_ecp_copy(R, P));
        if (mbedtls_mpi_cmp_int(&R->Y, 0) != 0)
            MBEDTLS_MPI_CHK(mbedtls_mpi_sub_mpi(&R->Y, &grp->P, &R->Y));
    }
    else
    {
        MBEDTLS_MPI_CHK(mbedtls_ecp_mul_restartable(grp, R, m, P, NULL, NULL, rs_ctx));
    }

cleanup:
    return (ret);
}

/*
 * Restartable linear combination
 * NOT constant-time
 */
int mbedtls_ecp_muladd_restartable(mbedtls_ecp_group       *grp,
                                   mbedtls_ecp_point       *R,
                                   const mbedtls_mpi       *m,
                                   const mbedtls_ecp_point *P,
                                   const mbedtls_mpi       *n,
                                   const mbedtls_ecp_point *Q,
                                   mbedtls_ecp_restart_ctx *rs_ctx)
{
    int                ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    mbedtls_ecp_point  mP;
    mbedtls_ecp_point *pmP = &mP;
    mbedtls_ecp_point *pR  = R;
#if defined(MBEDTLS_ECP_INTERNAL_ALT)
    char is_grp_capable = 0;
#endif
    ECP_VALIDATE_RET(grp != NULL);
    ECP_VALIDATE_RET(R != NULL);
    ECP_VALIDATE_RET(m != NULL);
    ECP_VALIDATE_RET(P != NULL);
    ECP_VALIDATE_RET(n != NULL);
    ECP_VALIDATE_RET(Q != NULL);

    if (mbedtls_ecp_get_type(grp) != MBEDTLS_ECP_TYPE_SHORT_WEIERSTRASS)
        return (MBEDTLS_ERR_ECP_FEATURE_UNAVAILABLE);

    mbedtls_ecp_point_init(&mP);

    ECP_RS_ENTER(ma);

#if defined(MBEDTLS_ECP_RESTARTABLE)
    if (rs_ctx != NULL && rs_ctx->ma != NULL)
    {
        /* redirect intermediate results to restart context */
        pmP = &rs_ctx->ma->mP;
        pR  = &rs_ctx->ma->R;

        /* jump to next operation */
        if (rs_ctx->ma->state == ecp_rsma_mul2)
            goto mul2;
        if (rs_ctx->ma->state == ecp_rsma_add)
            goto add;
        if (rs_ctx->ma->state == ecp_rsma_norm)
            goto norm;
    }
#endif /* MBEDTLS_ECP_RESTARTABLE */

    MBEDTLS_MPI_CHK(mbedtls_ecp_mul_shortcuts(grp, pmP, m, P, rs_ctx));

#if defined(MBEDTLS_ECP_RESTARTABLE)
    if (rs_ctx != NULL && rs_ctx->ma != NULL)
        rs_ctx->ma->state = ecp_rsma_mul2;

mul2:
#endif
    MBEDTLS_MPI_CHK(mbedtls_ecp_mul_shortcuts(grp, pR, n, Q, rs_ctx));

#if defined(MBEDTLS_ECP_INTERNAL_ALT)
    if ((is_grp_capable = mbedtls_internal_ecp_grp_capable(grp)))
        MBEDTLS_MPI_CHK(mbedtls_internal_ecp_init(grp));
#endif /* MBEDTLS_ECP_INTERNAL_ALT */

#if defined(MBEDTLS_ECP_RESTARTABLE)
    if (rs_ctx != NULL && rs_ctx->ma != NULL)
        rs_ctx->ma->state = ecp_rsma_add;

add:
#endif
    MBEDTLS_ECP_BUDGET(MBEDTLS_ECP_OPS_ADD);
    MBEDTLS_MPI_CHK(ecp_add_mixed(grp, pR, pmP, pR));
#if defined(MBEDTLS_ECP_RESTARTABLE)
    if (rs_ctx != NULL && rs_ctx->ma != NULL)
        rs_ctx->ma->state = ecp_rsma_norm;

norm:
#endif
    MBEDTLS_ECP_BUDGET(MBEDTLS_ECP_OPS_INV);
    MBEDTLS_MPI_CHK(ecp_normalize_jac(grp, pR));

#if defined(MBEDTLS_ECP_RESTARTABLE)
    if (rs_ctx != NULL && rs_ctx->ma != NULL)
        MBEDTLS_MPI_CHK(mbedtls_ecp_copy(R, pR));
#endif

cleanup:
#if defined(MBEDTLS_ECP_INTERNAL_ALT)
    if (is_grp_capable)
        mbedtls_internal_ecp_free(grp);
#endif /* MBEDTLS_ECP_INTERNAL_ALT */

    mbedtls_ecp_point_free(&mP);

    ECP_RS_LEAVE(ma);

    return (ret);
}

/*
 * Linear combination
 * NOT constant-time
 */
int mbedtls_ecp_muladd(mbedtls_ecp_group       *grp,
                       mbedtls_ecp_point       *R,
                       const mbedtls_mpi       *m,
                       const mbedtls_ecp_point *P,
                       const mbedtls_mpi       *n,
                       const mbedtls_ecp_point *Q)
{
    ECP_VALIDATE_RET(grp != NULL);
    ECP_VALIDATE_RET(R != NULL);
    ECP_VALIDATE_RET(m != NULL);
    ECP_VALIDATE_RET(P != NULL);
    ECP_VALIDATE_RET(n != NULL);
    ECP_VALIDATE_RET(Q != NULL);
    return (mbedtls_ecp_muladd_restartable(grp, R, m, P, n, Q, NULL));
}
#endif /* MBEDTLS_ECP_SHORT_WEIERSTRASS_ENABLED */

#if defined(MBEDTLS_ECP_MONTGOMERY_ENABLED)
#if defined(MBEDTLS_ECP_DP_CURVE25519_ENABLED)
#define ECP_MPI_INIT(s, n, p)           \
    {                                   \
        s, (n), (mbedtls_mpi_uint *)(p) \
    }
#define ECP_MPI_INIT_ARRAY(x) ECP_MPI_INIT(1, sizeof(x) / sizeof(mbedtls_mpi_uint), x)
/*
 * Constants for the two points other than 0, 1, -1 (mod p) in
 * https://cr.yp.to/ecdh.html#validate
 * See ecp_check_pubkey_x25519().
 */
static const mbedtls_mpi_uint x25519_bad_point_1[] = {
    MBEDTLS_BYTES_TO_T_UINT_8(0xe0, 0xeb, 0x7a, 0x7c, 0x3b, 0x41, 0xb8, 0xae),
    MBEDTLS_BYTES_TO_T_UINT_8(0x16, 0x56, 0xe3, 0xfa, 0xf1, 0x9f, 0xc4, 0x6a),
    MBEDTLS_BYTES_TO_T_UINT_8(0xda, 0x09, 0x8d, 0xeb, 0x9c, 0x32, 0xb1, 0xfd),
    MBEDTLS_BYTES_TO_T_UINT_8(0x86, 0x62, 0x05, 0x16, 0x5f, 0x49, 0xb8, 0x00),
};
static const mbedtls_mpi_uint x25519_bad_point_2[] = {
    MBEDTLS_BYTES_TO_T_UINT_8(0x5f, 0x9c, 0x95, 0xbc, 0xa3, 0x50, 0x8c, 0x24),
    MBEDTLS_BYTES_TO_T_UINT_8(0xb1, 0xd0, 0xb1, 0x55, 0x9c, 0x83, 0xef, 0x5b),
    MBEDTLS_BYTES_TO_T_UINT_8(0x04, 0x44, 0x5c, 0xc4, 0x58, 0x1c, 0x8e, 0x86),
    MBEDTLS_BYTES_TO_T_UINT_8(0xd8, 0x22, 0x4e, 0xdd, 0xd0, 0x9f, 0x11, 0x57),
};
static const mbedtls_mpi ecp_x25519_bad_point_1 = ECP_MPI_INIT_ARRAY(x25519_bad_point_1);
static const mbedtls_mpi ecp_x25519_bad_point_2 = ECP_MPI_INIT_ARRAY(x25519_bad_point_2);
#endif /* MBEDTLS_ECP_DP_CURVE25519_ENABLED */

/*
 * Check that the input point is not one of the low-order points.
 * This is recommended by the "May the Fourth" paper:
 * https://eprint.iacr.org/2017/806.pdf
 * Those points are never sent by an honest peer.
 */
static int ecp_check_bad_points_mx(const mbedtls_mpi *X, const mbedtls_mpi *P, const mbedtls_ecp_group_id grp_id)
{
    int         ret;
    mbedtls_mpi XmP;

    mbedtls_mpi_init(&XmP);

    /* Reduce X mod P so that we only need to check values less than P.
     * We know X < 2^256 so we can proceed by subtraction. */
    MBEDTLS_MPI_CHK(mbedtls_mpi_copy(&XmP, X));
    while (mbedtls_mpi_cmp_mpi(&XmP, P) >= 0) MBEDTLS_MPI_CHK(mbedtls_mpi_sub_mpi(&XmP, &XmP, P));

    /* Check against the known bad values that are less than P. For Curve448
     * these are 0, 1 and -1. For Curve25519 we check the values less than P
     * from the following list: https://cr.yp.to/ecdh.html#validate */
    if (mbedtls_mpi_cmp_int(&XmP, 1) <= 0) /* takes care of 0 and 1 */
    {
        ret = MBEDTLS_ERR_ECP_INVALID_KEY;
        goto cleanup;
    }

#if defined(MBEDTLS_ECP_DP_CURVE25519_ENABLED)
    if (grp_id == MBEDTLS_ECP_DP_CURVE25519)
    {
        if (mbedtls_mpi_cmp_mpi(&XmP, &ecp_x25519_bad_point_1) == 0)
        {
            ret = MBEDTLS_ERR_ECP_INVALID_KEY;
            goto cleanup;
        }

        if (mbedtls_mpi_cmp_mpi(&XmP, &ecp_x25519_bad_point_2) == 0)
        {
            ret = MBEDTLS_ERR_ECP_INVALID_KEY;
            goto cleanup;
        }
    }
#else
    (void)grp_id;
#endif

    /* Final check: check if XmP + 1 is P (final because it changes XmP!) */
    MBEDTLS_MPI_CHK(mbedtls_mpi_add_int(&XmP, &XmP, 1));
    if (mbedtls_mpi_cmp_mpi(&XmP, P) == 0)
    {
        ret = MBEDTLS_ERR_ECP_INVALID_KEY;
        goto cleanup;
    }

    ret = 0;

cleanup:
    mbedtls_mpi_free(&XmP);

    return (ret);
}

/*
 * Check validity of a public key for Montgomery curves with x-only schemes
 */
static int ecp_check_pubkey_mx(const mbedtls_ecp_group *grp, const mbedtls_ecp_point *pt)
{
    /* [Curve25519 p. 5] Just check X is the correct number of bytes */
    /* Allow any public value, if it's too big then we'll just reduce it mod p
     * (RFC 7748 sec. 5 para. 3). */
    if (mbedtls_mpi_size(&pt->X) > (grp->nbits + 7) / 8)
        return (MBEDTLS_ERR_ECP_INVALID_KEY);

    /* Implicit in all standards (as they don't consider negative numbers):
     * X must be non-negative. This is normally ensured by the way it's
     * encoded for transmission, but let's be extra sure. */
    if (mbedtls_mpi_cmp_int(&pt->X, 0) < 0)
        return (MBEDTLS_ERR_ECP_INVALID_KEY);

    return (ecp_check_bad_points_mx(&pt->X, &grp->P, grp->id));
}
#endif /* MBEDTLS_ECP_MONTGOMERY_ENABLED */

/*
 * Check that a point is valid as a public key
 */
int mbedtls_ecp_check_pubkey(const mbedtls_ecp_group *grp, const mbedtls_ecp_point *pt)
{
    ECP_VALIDATE_RET(grp != NULL);
    ECP_VALIDATE_RET(pt != NULL);

    /* Must use affine coordinates */
    if (mbedtls_mpi_cmp_int(&pt->Z, 1) != 0)
        return (MBEDTLS_ERR_ECP_INVALID_KEY);

#if defined(MBEDTLS_ECP_MONTGOMERY_ENABLED)
    if (mbedtls_ecp_get_type(grp) == MBEDTLS_ECP_TYPE_MONTGOMERY)
        return (ecp_check_pubkey_mx(grp, pt));
#endif
#if defined(MBEDTLS_ECP_SHORT_WEIERSTRASS_ENABLED)
    if (mbedtls_ecp_get_type(grp) == MBEDTLS_ECP_TYPE_SHORT_WEIERSTRASS)
        return (ecp_check_pubkey_sw(grp, pt));
#endif

    return (MBEDTLS_ERR_ECP_BAD_INPUT_DATA);
}

/*
 * Check that an mbedtls_mpi is valid as a private key
 */
int mbedtls_ecp_check_privkey(const mbedtls_ecp_group *grp, const mbedtls_mpi *d)
{
    ECP_VALIDATE_RET(grp != NULL);
    ECP_VALIDATE_RET(d != NULL);

#if defined(MBEDTLS_ECP_MONTGOMERY_ENABLED)
    if (mbedtls_ecp_get_type(grp) == MBEDTLS_ECP_TYPE_MONTGOMERY)
    {
        /* see RFC 7748 sec. 5 para. 5 */
        if (mbedtls_mpi_get_bit(d, 0) != 0 || mbedtls_mpi_get_bit(d, 1) != 0 ||
            mbedtls_mpi_bitlen(d) - 1 != grp->nbits) /* mbedtls_mpi_bitlen is one-based! */
            return (MBEDTLS_ERR_ECP_INVALID_KEY);

        /* see [Curve25519] page 5 */
        if (grp->nbits == 254 && mbedtls_mpi_get_bit(d, 2) != 0)
            return (MBEDTLS_ERR_ECP_INVALID_KEY);

        return (0);
    }
#endif /* MBEDTLS_ECP_MONTGOMERY_ENABLED */
#if defined(MBEDTLS_ECP_SHORT_WEIERSTRASS_ENABLED)
    if (mbedtls_ecp_get_type(grp) == MBEDTLS_ECP_TYPE_SHORT_WEIERSTRASS)
    {
        /* see SEC1 3.2 */
        if (mbedtls_mpi_cmp_int(d, 1) < 0 || mbedtls_mpi_cmp_mpi(d, &grp->N) >= 0)
            return (MBEDTLS_ERR_ECP_INVALID_KEY);
        else
            return (0);
    }
#endif /* MBEDTLS_ECP_SHORT_WEIERSTRASS_ENABLED */

    return (MBEDTLS_ERR_ECP_BAD_INPUT_DATA);
}

#if defined(MBEDTLS_ECP_MONTGOMERY_ENABLED)
MBEDTLS_STATIC_TESTABLE
int mbedtls_ecp_gen_privkey_mx(size_t       high_bit,
                               mbedtls_mpi *d,
                               int (*f_rng)(void *, unsigned char *, size_t),
                               void *p_rng)
{
    int    ret            = MBEDTLS_ERR_ECP_BAD_INPUT_DATA;
    size_t n_random_bytes = high_bit / 8 + 1;

    /* [Curve25519] page 5 */
    /* Generate a (high_bit+1)-bit random number by generating just enough
     * random bytes, then shifting out extra bits from the top (necessary
     * when (high_bit+1) is not a multiple of 8). */
    MBEDTLS_MPI_CHK(mbedtls_mpi_fill_random(d, n_random_bytes, f_rng, p_rng));
    MBEDTLS_MPI_CHK(mbedtls_mpi_shift_r(d, 8 * n_random_bytes - high_bit - 1));

    MBEDTLS_MPI_CHK(mbedtls_mpi_set_bit(d, high_bit, 1));

    /* Make sure the last two bits are unset for Curve448, three bits for
       Curve25519 */
    MBEDTLS_MPI_CHK(mbedtls_mpi_set_bit(d, 0, 0));
    MBEDTLS_MPI_CHK(mbedtls_mpi_set_bit(d, 1, 0));
    if (high_bit == 254)
    {
        MBEDTLS_MPI_CHK(mbedtls_mpi_set_bit(d, 2, 0));
    }

cleanup:
    return (ret);
}
#endif /* MBEDTLS_ECP_MONTGOMERY_ENABLED */

#if defined(MBEDTLS_ECP_SHORT_WEIERSTRASS_ENABLED)
static int mbedtls_ecp_gen_privkey_sw(const mbedtls_mpi *N,
                                      mbedtls_mpi       *d,
                                      int (*f_rng)(void *, unsigned char *, size_t),
                                      void *p_rng)
{
    int ret = mbedtls_mpi_random(d, 1, N, f_rng, p_rng);
    switch (ret)
    {
    case MBEDTLS_ERR_MPI_NOT_ACCEPTABLE:
        return (MBEDTLS_ERR_ECP_RANDOM_FAILED);
    default:
        return (ret);
    }
}
#endif /* MBEDTLS_ECP_SHORT_WEIERSTRASS_ENABLED */

/*
 * Generate a private key
 */
int mbedtls_ecp_gen_privkey(const mbedtls_ecp_group *grp,
                            mbedtls_mpi             *d,
                            int (*f_rng)(void *, unsigned char *, size_t),
                            void *p_rng)
{
    ECP_VALIDATE_RET(grp != NULL);
    ECP_VALIDATE_RET(d != NULL);
    ECP_VALIDATE_RET(f_rng != NULL);

#if defined(MBEDTLS_ECP_MONTGOMERY_ENABLED)
    if (mbedtls_ecp_get_type(grp) == MBEDTLS_ECP_TYPE_MONTGOMERY)
        return (mbedtls_ecp_gen_privkey_mx(grp->nbits, d, f_rng, p_rng));
#endif /* MBEDTLS_ECP_MONTGOMERY_ENABLED */

#if defined(MBEDTLS_ECP_SHORT_WEIERSTRASS_ENABLED)
    if (mbedtls_ecp_get_type(grp) == MBEDTLS_ECP_TYPE_SHORT_WEIERSTRASS)
        return (mbedtls_ecp_gen_privkey_sw(&grp->N, d, f_rng, p_rng));
#endif /* MBEDTLS_ECP_SHORT_WEIERSTRASS_ENABLED */

    return (MBEDTLS_ERR_ECP_BAD_INPUT_DATA);
}

/*
 * Generate a keypair with configurable base point
 */
int mbedtls_ecp_gen_keypair_base(mbedtls_ecp_group       *grp,
                                 const mbedtls_ecp_point *G,
                                 mbedtls_mpi             *d,
                                 mbedtls_ecp_point       *Q,
                                 int (*f_rng)(void *, unsigned char *, size_t),
                                 void *p_rng)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    ECP_VALIDATE_RET(grp != NULL);
    ECP_VALIDATE_RET(d != NULL);
    ECP_VALIDATE_RET(G != NULL);
    ECP_VALIDATE_RET(Q != NULL);
    ECP_VALIDATE_RET(f_rng != NULL);

    MBEDTLS_MPI_CHK(mbedtls_ecp_gen_privkey(grp, d, f_rng, p_rng));
    MBEDTLS_MPI_CHK(mbedtls_ecp_mul(grp, Q, d, G, f_rng, p_rng));

cleanup:
    return (ret);
}

/*
 * Generate key pair, wrapper for conventional base point
 */
int mbedtls_ecp_gen_keypair(mbedtls_ecp_group *grp,
                            mbedtls_mpi       *d,
                            mbedtls_ecp_point *Q,
                            int (*f_rng)(void *, unsigned char *, size_t),
                            void *p_rng)
{
    ECP_VALIDATE_RET(grp != NULL);
    ECP_VALIDATE_RET(d != NULL);
    ECP_VALIDATE_RET(Q != NULL);
    ECP_VALIDATE_RET(f_rng != NULL);

    return (mbedtls_ecp_gen_keypair_base(grp, &grp->G, d, Q, f_rng, p_rng));
}

/*
 * Generate a keypair, prettier wrapper
 */
int mbedtls_ecp_gen_key(mbedtls_ecp_group_id grp_id,
                        mbedtls_ecp_keypair *key,
                        int (*f_rng)(void *, unsigned char *, size_t),
                        void *p_rng)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    ECP_VALIDATE_RET(key != NULL);
    ECP_VALIDATE_RET(f_rng != NULL);

    if ((ret = mbedtls_ecp_group_load(&key->grp, grp_id)) != 0)
        return (ret);

    return (mbedtls_ecp_gen_keypair(&key->grp, &key->d, &key->Q, f_rng, p_rng));
}

#define ECP_CURVE25519_KEY_SIZE 32
/*
 * Read a private key.
 */
int mbedtls_ecp_read_key(mbedtls_ecp_group_id grp_id, mbedtls_ecp_keypair *key, const unsigned char *buf, size_t buflen)
{
    int ret = 0;

    ECP_VALIDATE_RET(key != NULL);
    ECP_VALIDATE_RET(buf != NULL);

    if ((ret = mbedtls_ecp_group_load(&key->grp, grp_id)) != 0)
        return (ret);

    ret = MBEDTLS_ERR_ECP_FEATURE_UNAVAILABLE;

#if defined(MBEDTLS_ECP_MONTGOMERY_ENABLED)
    if (mbedtls_ecp_get_type(&key->grp) == MBEDTLS_ECP_TYPE_MONTGOMERY)
    {
        /*
         * If it is Curve25519 curve then mask the key as mandated by RFC7748
         */
        if (grp_id == MBEDTLS_ECP_DP_CURVE25519)
        {
            if (buflen != ECP_CURVE25519_KEY_SIZE)
                return MBEDTLS_ERR_ECP_INVALID_KEY;

            MBEDTLS_MPI_CHK(mbedtls_mpi_read_binary_le(&key->d, buf, buflen));

            /* Set the three least significant bits to 0 */
            MBEDTLS_MPI_CHK(mbedtls_mpi_set_bit(&key->d, 0, 0));
            MBEDTLS_MPI_CHK(mbedtls_mpi_set_bit(&key->d, 1, 0));
            MBEDTLS_MPI_CHK(mbedtls_mpi_set_bit(&key->d, 2, 0));

            /* Set the most significant bit to 0 */
            MBEDTLS_MPI_CHK(mbedtls_mpi_set_bit(&key->d, ECP_CURVE25519_KEY_SIZE * 8 - 1, 0));

            /* Set the second most significant bit to 1 */
            MBEDTLS_MPI_CHK(mbedtls_mpi_set_bit(&key->d, ECP_CURVE25519_KEY_SIZE * 8 - 2, 1));
        }
        else
            ret = MBEDTLS_ERR_ECP_FEATURE_UNAVAILABLE;
    }

#endif
#if defined(MBEDTLS_ECP_SHORT_WEIERSTRASS_ENABLED)
    if (mbedtls_ecp_get_type(&key->grp) == MBEDTLS_ECP_TYPE_SHORT_WEIERSTRASS)
    {
        MBEDTLS_MPI_CHK(mbedtls_mpi_read_binary(&key->d, buf, buflen));
        MBEDTLS_MPI_CHK(mbedtls_ecp_check_privkey(&key->grp, &key->d));
    }
#endif

cleanup:
    if (ret != 0)
        mbedtls_mpi_free(&key->d);

    return (ret);
}

/*
 * Write a private key.
 */
int mbedtls_ecp_write_key(mbedtls_ecp_keypair *key, unsigned char *buf, size_t buflen)
{
    int ret = MBEDTLS_ERR_ECP_FEATURE_UNAVAILABLE;

    ECP_VALIDATE_RET(key != NULL);
    ECP_VALIDATE_RET(buf != NULL);

#if defined(MBEDTLS_ECP_MONTGOMERY_ENABLED)
    if (mbedtls_ecp_get_type(&key->grp) == MBEDTLS_ECP_TYPE_MONTGOMERY)
    {
        if (key->grp.id == MBEDTLS_ECP_DP_CURVE25519)
        {
            if (buflen < ECP_CURVE25519_KEY_SIZE)
                return MBEDTLS_ERR_ECP_BUFFER_TOO_SMALL;

            MBEDTLS_MPI_CHK(mbedtls_mpi_write_binary_le(&key->d, buf, buflen));
        }
        else
            ret = MBEDTLS_ERR_ECP_FEATURE_UNAVAILABLE;
    }

#endif
#if defined(MBEDTLS_ECP_SHORT_WEIERSTRASS_ENABLED)
    if (mbedtls_ecp_get_type(&key->grp) == MBEDTLS_ECP_TYPE_SHORT_WEIERSTRASS)
    {
        MBEDTLS_MPI_CHK(mbedtls_mpi_write_binary(&key->d, buf, buflen));
    }

#endif

cleanup:
    return (ret);
}

/*
 * Check a public-private key pair
 */
int mbedtls_ecp_check_pub_priv(const mbedtls_ecp_keypair *pub, const mbedtls_ecp_keypair *prv)
{
    int               ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    mbedtls_ecp_point Q;
    mbedtls_ecp_group grp;
    ECP_VALIDATE_RET(pub != NULL);
    ECP_VALIDATE_RET(prv != NULL);

    if (pub->grp.id == MBEDTLS_ECP_DP_NONE || pub->grp.id != prv->grp.id || mbedtls_mpi_cmp_mpi(&pub->Q.X, &prv->Q.X) ||
        mbedtls_mpi_cmp_mpi(&pub->Q.Y, &prv->Q.Y) || mbedtls_mpi_cmp_mpi(&pub->Q.Z, &prv->Q.Z))
    {
        return (MBEDTLS_ERR_ECP_BAD_INPUT_DATA);
    }

    mbedtls_ecp_point_init(&Q);
    mbedtls_ecp_group_init(&grp);

    /* mbedtls_ecp_mul() needs a non-const group... */
    mbedtls_ecp_group_copy(&grp, &prv->grp);

    /* Also checks d is valid */
    MBEDTLS_MPI_CHK(mbedtls_ecp_mul(&grp, &Q, &prv->d, &prv->grp.G, NULL, NULL));

    if (mbedtls_mpi_cmp_mpi(&Q.X, &prv->Q.X) || mbedtls_mpi_cmp_mpi(&Q.Y, &prv->Q.Y) ||
        mbedtls_mpi_cmp_mpi(&Q.Z, &prv->Q.Z))
    {
        ret = MBEDTLS_ERR_ECP_BAD_INPUT_DATA;
        goto cleanup;
    }

cleanup:
    mbedtls_ecp_point_free(&Q);
    mbedtls_ecp_group_free(&grp);

    return (ret);
}

#endif /* !MBEDTLS_ECP_ALT */
