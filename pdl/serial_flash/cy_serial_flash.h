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

#include <stdint.h>

#define _PACKED_                __attribute__((packed))
/****************************************************************************************
 * Nor Flash SPI Definition
 ****************************************************************************************/
/* Target SPI Interface. */
#ifndef FLASH_NOR_SPI_INTERFACE
#define FLASH_NOR_SPI_INTERFACE SPI2
#endif // GD25WQ64E_SPI_INTERFACE

/* SPI Interface Clock Rate. */
#ifndef FLASH_NOR_SPI_CLOCK_RATE
#define FLASH_NOR_SPI_CLOCK_RATE 24000000
#endif // GD25WQ64E_SPI_CLOCK_RATE

#define SPIFFY2_RX_BUF_LEN      64

/****************************************************************************************
 * Nor Flash Vendor Definition
 ****************************************************************************************/
#define FLASH_NOR_VENDOR_MXIC                   (0xC2)
#define FLASH_NOR_VENDOR_GD                     (0xC8)
#define FLASH_NOR_VENDOR_WINBOND                (0xEF)

/****************************************************************************************
 * Nor Flash Vendor (GD) Definition
 ****************************************************************************************/
#define SERIAL_FLASH_DEVICE_ID_GD_GD25WQ16E     (0x6515)
#define SERIAL_FLASH_DEVICE_ID_GD_GD25WQ64E     (0x6517)

/****************************************************************************************
 * Nor Flash Device Definition
 ****************************************************************************************/
#define FLASH_NOR_PAGE_SIZE                     (256)
#define FLASH_NOR_SECTOR_SIZE                   (0x1000)
#define FLASH_NOR_BLOCK_SIZE                    (0x10000)
#define FLASH_NOR_STATUS_WIP_BIT                (0x01)
#define FLASH_NOR_CMD_LENGTH                    (1)
#define FLASH_NOR_ADDR_LENGTH                   (3)
/****************************************************************************************
 * Nor Flash Macro
 ****************************************************************************************/
#define FLASH_NOR_QUERY_INFO_TABLE_ENTRY_NUM            (sizeof(flash_nor_query_info_table) / sizeof(flash_nor_query_info_table_struct_t))
#define IS_SAME_FLASH_NOR_PAGE(addr0, addr1)            (((addr0) & 0xFFFFFF00) == ((addr1) & 0xFFFFFF00))
#define ADDR_ENDIAN_TRANS(addr, flash_addr)             do {\
                                                            flash_addr[0] = (uint8_t) ((addr) >> 16);\
                                                            flash_addr[1] = (uint8_t) ((addr) >> 8); \
                                                            flash_addr[2] = (uint8_t) (addr);        \
                                                        } while (0);
/****************************************************************************************
 * Nor Flash Enumeration
 ****************************************************************************************/
 typedef enum
{
    FLASH_NOR_RET_SUCCESS,
    FLASH_NOR_RET_UNKNOWN,
    FLASH_NOR_RET_NOT_EXIST_BASIC_CMD,
    FLASH_NOR_RET_NOT_EXIST_ADV_CMD,
    FLASH_NOR_RET_NOT_EXIST_QUERY_INFO,
    FLASH_NOR_RET_CMD_NOT_SUPPORT,
    FLASH_NOR_RET_DEV_NOT_SUPPORT,
    FLASH_NOR_RET_VENDOR_NOT_SUPPORT,
    FLASH_NOR_RET_ADDR_LARGER_THAN_FLASH_SIZE,
    FLASH_NOR_RET_WAIT_BUSY_FAILED,
    FLASH_NOR_RET_PARAM_INVALID
} cy_serial_flash_ret_type_t;

typedef enum
{
    FLASH_NOR_ERASE_SECTOR  = 1,
    FLASH_NOR_ERASE_BLOCK   = 2,
    FLASH_NOR_ERASE_CHIP    = 4,
} flash_nor_erase_mode_t;

typedef enum
{
    FLASH_NOR_EXIST_NONE        = 0,
    FLASH_NOR_EXIST_BASIC_CMD   = 1,
    FLASH_NOR_EXIST_ADV_CMD     = 2,
    FLASH_NOR_EXIST_QUERY_INFO  = 4,
    FLASH_NOR_EXIST_ALL         = 7,
} flash_nor_exist_lv_t;

typedef enum
{
    FLASH_NOR_1_BIT_MODE,
    FLASH_NOR_2_BIT_MODE,
    FLASH_NOR_4_BIT_MODE,
} flash_nor_bit_mode_t;

/****************************************************************************************
 * SPIC Structure
 ****************************************************************************************/
typedef struct _PACKED_ _SPIC_TX
{
    uint8_t cmd;
    uint8_t addr[3];
    uint8_t data[FLASH_NOR_PAGE_SIZE];
} spic_tx_struct_t ;

typedef struct _SPIC_ACCESS_INFO
{
    spic_tx_struct_t spi_tx;
    uint32_t total_len;
} spic_access_info_struct_t;
/****************************************************************************************
 * Nor Flash Structure
 ****************************************************************************************/
typedef struct _PACKED_ _FLASH_NOR_BASIC_CMD_STRUCT
{
    const uint8_t cmd_rd_id;        /* Read nor flash ID cmd */
    uint8_t cmd_rd_sr;              /* Read status register */
    uint8_t cmd_wr_sr;              /* Write status register */
    uint8_t cmd_rd_data;            /* Single read cmd */
    uint8_t cmd_pp;                 /* Single page program cmd */
    uint8_t cmd_sector_e;           /* Erase cmd for nor flash sector (4kB) */
    uint8_t cmd_block_e;            /* Erase cmd for nor flash block (64KB) */
    uint8_t cmd_chip_e;             /* Erase cmd for whole nor flash */
    uint8_t cmd_wr_en;              /* Write enable cmd */
    uint8_t cmd_wr_en_vol;          /* Write enable for volatile status register cmd */
    uint8_t cmd_wr_di;              /* Write disable cmd */
    uint8_t cmd_dp;                 /* Enter deep power down mode cmd */
    uint8_t cmd_dp_release;         /* Release from deep power down mode cmd */
    const uint8_t cmd_en_reset;     /* Enable software reset cmd */
    const uint8_t cmd_reset;        /* Software reset cmd */
} flash_nor_basic_cmd_struct_t;

typedef struct _PACKED_ _FLASH_NOR_ADV_CMD_STRUCT
{
    uint8_t cmd_rd_dual_o;          /* Dual data read cmd */
    uint8_t cmd_rd_dual_io;         /* Dual data/addr read cmd */
    uint8_t cmd_rd_quad_o;          /* Quad data read cmd */
    uint8_t cmd_rd_quad_io;         /* Quad data/addr read cmd */
    uint8_t cmd_pp_quad_i;          /* Quad page program cmd (1-1-4) */
    uint8_t cmd_pp_quad_ii;         /* Quad page program cmd (1-4-4) */
    uint8_t cmd_rd_sr2;             /* Read status register 2 */
    uint8_t cmd_rd_sr3;             /* Read status register 3 */
    uint8_t cmd_rd_scur;            /* Read secure register */
    uint8_t cmd_wr_sr2;             /* Write status register 2 */
    uint8_t cmd_wr_sr3;             /* Write status register 3 */
    uint8_t cmd_wr_scur;            /* Write secure register */
    uint8_t cmd_rd_cr;              /* Read config cmd */
    uint8_t cmd_hpm;                /* High-performance or continuous read cmd */
    uint8_t cmd_suspend;            /* Suspend cmd */
    uint8_t cmd_resume;             /* Resume cmd */
} flash_nor_adv_cmd_struct_t;

typedef struct _PACKED_ _FLASH_NOR_QUERY_INFO_STRUCT
{
    uint32_t flash_size;                /* Nor flash size in byte */
    uint8_t qebo;                       /* QE bit offset in status register */
    uint8_t wsbo;                       /* Write suspend flag bit offset in status register */
    uint8_t esbo;                       /* Erase suspend flag bit offset in status register */
    uint8_t tbbo;                       /* Top-bottom bit offset in status register */
    uint8_t cmpbo;                      /* Complement bit offset in status register */
    uint8_t adsbo;                      /* Current address mode bit offset in status register */
    uint8_t bp_all;                     /* Block protect all level in status register */
    uint8_t bp_mask;                    /* Block protect configurable bit mask in status register */
    uint32_t resume_to_suspend_delay_us;/* Delay time for a resume command to next suspend command */
} flash_nor_query_info_struct_t;

typedef struct _PACKED_ _FLASH_NOR_INFO_STRUCT
{
    uint8_t manu_id;
    uint16_t device_id;

    flash_nor_basic_cmd_struct_t basic;
    flash_nor_adv_cmd_struct_t adv;
    const flash_nor_query_info_struct_t *query;
} flash_nor_info_struct_t;

typedef struct _FLASH_SETTINGS_CFG
{
    uint32_t wait_max_retry;        /* Maximum retry for both SPIC busy bit & flash WIP bit */
    uint32_t erase_max_retry;
    uint32_t sw_reset_delay_us;
} flash_settings_cfg_t;

typedef struct _PACKED_ _FLASH_NOR_QUERY_INFO_TABLE_STRUCT
{
    uint8_t manu_id;
    uint16_t device_id;
    flash_nor_query_info_struct_t query;
} flash_nor_query_info_table_struct_t;

/****************************************************************************************
 * Nor Flash Function Prototype
 ****************************************************************************************/
cy_serial_flash_ret_type_t cy_serial_flash_init(void);
cy_serial_flash_ret_type_t cy_serial_flash_erase(uint32_t addr, flash_nor_erase_mode_t mode);
uint32_t cy_serial_flash_get_flash_size(void);
cy_serial_flash_ret_type_t cy_serial_flash_write(uint32_t addr, uint8_t *data, uint32_t byte_len);
cy_serial_flash_ret_type_t cy_serial_flash_read(uint32_t addr, uint8_t *data, uint32_t byte_len);
cy_serial_flash_ret_type_t cy_serial_flash_deep_power_down(void);
cy_serial_flash_ret_type_t cy_serial_flash_deep_power_down_release(void);
