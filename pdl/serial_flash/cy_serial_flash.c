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

//==================================================================================================
// Include
//==================================================================================================
#include <wiced_hal_pspi.h>
#include <wiced_rtos.h>
#include <stdbool.h>
#include "cy_serial_flash.h"
#include <wiced_bt_trace.h>
#include <wiced_utilities.h>

static flash_nor_exist_lv_t flash_exists = {FLASH_NOR_EXIST_NONE};
static flash_nor_info_struct_t flash_nor_info =
{
    .basic.cmd_rd_id = 0x9F,
    .basic.cmd_rd_sr = 0x05,
    .basic.cmd_wr_sr = 0x01,
    .basic.cmd_rd_data = 0x03,
    .basic.cmd_pp = 0x02,
    .basic.cmd_chip_e = 0x60,
    .basic.cmd_block_e = 0xD8,          //64KB block erase
    .basic.cmd_sector_e = 0x20,
    .basic.cmd_wr_en = 0x06,
    .basic.cmd_wr_en_vol = 0x50,
    .basic.cmd_wr_di = 0x04,
    .basic.cmd_dp = 0xB9,
    .basic.cmd_dp_release = 0xAB,
    .basic.cmd_en_reset = 0x66,
    .basic.cmd_reset = 0x99,
    .adv.cmd_rd_dual_o = 0x3B,
    .adv.cmd_rd_dual_io = 0xBB,
    .adv.cmd_rd_quad_o = 0x6B,
    .adv.cmd_rd_quad_io = 0xEB,
    .adv.cmd_pp_quad_i = 0x32,
    .adv.cmd_pp_quad_ii = 0x38,
    .adv.cmd_suspend = 0x75,
    .adv.cmd_resume = 0x7A
};
const flash_settings_cfg_t flash_cfg =
{
    .wait_max_retry = 10000000,
    .sw_reset_delay_us = 60,        /* GD requires maximum delay, which is about 60 us */
};

static const flash_nor_query_info_table_struct_t flash_nor_query_info_table[] =
{
    /* GD */
    {
        .manu_id = FLASH_NOR_VENDOR_GD,
        .device_id = SERIAL_FLASH_DEVICE_ID_GD_GD25WQ64E,
        .query = {
            .flash_size = 8 * 1024 * 1024,
            .qebo = 0x09,
            .wsbo = 0x0A,
            .esbo = 0x0F,
            .tbbo = 0x05,
            .cmpbo = 0x0E,
            .adsbo = 0xFF,
            .bp_all = 0x06,
            .bp_mask = 0x07,
            .resume_to_suspend_delay_us = 100,
        },
    },
    {
        .manu_id = FLASH_NOR_VENDOR_GD,
        .device_id = SERIAL_FLASH_DEVICE_ID_GD_GD25WQ16E,
        .query = {
            .flash_size = 2 * 1024 * 1024,
            .qebo = 0x09,
            .wsbo = 0x0F,
            .esbo = 0x0F,
            .tbbo = 0x05,
            .cmpbo = 0x0E,
            .adsbo = 0xFF,
            .bp_all = 0x05,
            .bp_mask = 0x07,
            .resume_to_suspend_delay_us = 400,
        },
    },
};

__attribute__((section(".text_in_ram"))) void flash_nor_set_exist(flash_nor_exist_lv_t exist_lv)
{
    flash_exists |= exist_lv;
}

__attribute__((section(".text_in_ram"))) flash_nor_exist_lv_t flash_nor_get_exist(void)
{
    return flash_exists;
}

__attribute__((section(".text_in_ram"))) void flash_nor_reset_exist(void)
{
    flash_exists = FLASH_NOR_EXIST_NONE;
}

__attribute__((section(".text_in_ram"))) cy_serial_flash_ret_type_t flash_nor_cmd_tx(uint8_t cmd, const uint32_t *addr, const void *data, size_t length)
{
    spic_tx_struct_t tx_buffer = {
        .cmd = cmd,
    };

    uint32_t total_length = FLASH_NOR_CMD_LENGTH;

    if (addr != NULL)
    {
        ADDR_ENDIAN_TRANS(*addr, tx_buffer.addr);
        total_length += FLASH_NOR_ADDR_LENGTH;
    }

    if (length > 0)
    {
        if (length > sizeof(tx_buffer.data))
        {
            return FLASH_NOR_RET_PARAM_INVALID;
        }

        memcpy(tx_buffer.data, data, length);
        total_length += length;
    }

    wiced_hal_pspi_tx_data(FLASH_NOR_SPI_INTERFACE, total_length, (uint8_t *)&tx_buffer);

    return FLASH_NOR_RET_SUCCESS;
}

__attribute__((section(".text_in_ram"))) cy_serial_flash_ret_type_t flash_nor_cmd_rx(uint8_t cmd, const uint32_t *addr, size_t length, uint8_t *buf)
{
    spic_tx_struct_t tx_buffer = {
        .cmd = cmd,
    };

    uint32_t total_length = FLASH_NOR_CMD_LENGTH;
    uint32_t data_offset = total_length;
    uint8_t rx_buffer[SPIFFY2_RX_BUF_LEN];

    if (addr != NULL)
    {
        ADDR_ENDIAN_TRANS(*addr, tx_buffer.addr);
        total_length += FLASH_NOR_ADDR_LENGTH;
        data_offset = total_length;
    }

    if (length > 0)
    {
        total_length += length;

        if (total_length > SPIFFY2_RX_BUF_LEN)
        {
            return FLASH_NOR_RET_PARAM_INVALID;
        }
    }

    wiced_hal_pspi_exchange_data(FLASH_NOR_SPI_INTERFACE, total_length, (uint8_t *)&tx_buffer, rx_buffer);

    if (length > 0)
    {
        memcpy(buf, &rx_buffer[data_offset], length);
    }

    return FLASH_NOR_RET_SUCCESS;
}

__attribute__((section(".text_in_ram"))) cy_serial_flash_ret_type_t flash_nor_sw_reset(void)
{
    cy_serial_flash_ret_type_t ret = FLASH_NOR_RET_UNKNOWN;

    if((ret = flash_nor_cmd_tx(flash_nor_info.basic.cmd_en_reset, NULL, NULL, 0)) != FLASH_NOR_RET_SUCCESS)
    {
        return ret;
    }

    if((ret = flash_nor_cmd_tx(flash_nor_info.basic.cmd_reset, NULL, NULL, 0)) != FLASH_NOR_RET_SUCCESS)
    {
        return ret;
    }

    wiced_rtos_delay_microseconds(flash_cfg.sw_reset_delay_us);

    return ret;
}

__attribute__((section(".text_in_ram"))) uint32_t flash_nor_get_rdid(void)
{
    uint8_t rx_buffer[3] = {0}; // 3 => manufacture ID + memory_type + capacity

    flash_nor_cmd_rx(flash_nor_info.basic.cmd_rd_id, NULL, 3, rx_buffer);

    return ((rx_buffer[0] << 16) | (rx_buffer[1] << 8) | rx_buffer[2]);  //manufacture ID[8] | memory_type[8] | capacity[8]
}

__attribute__((section(".text_in_ram"))) cy_serial_flash_ret_type_t flash_nor_update_gd_cmd(void)
{
    cy_serial_flash_ret_type_t ret = FLASH_NOR_RET_SUCCESS;

    switch (flash_nor_info.device_id)
    {
        case SERIAL_FLASH_DEVICE_ID_GD_GD25WQ16E:
            flash_nor_info.adv.cmd_rd_sr2 = 0x35;
            break;
        case SERIAL_FLASH_DEVICE_ID_GD_GD25WQ64E:
            flash_nor_info.adv.cmd_rd_sr2 = 0x35;
            flash_nor_info.adv.cmd_rd_sr3 = 0x15;
            flash_nor_info.adv.cmd_wr_sr2 = 0x31;
            flash_nor_info.adv.cmd_wr_sr3 = 0x11;
            break;
        default:
            ret = FLASH_NOR_RET_DEV_NOT_SUPPORT;
            break;
    }

    return ret;
}

__attribute__((section(".text_in_ram"))) cy_serial_flash_ret_type_t flash_nor_update_mxic_cmd(void)
{
    cy_serial_flash_ret_type_t ret = FLASH_NOR_RET_SUCCESS;

    switch (flash_nor_info.device_id)
    {
        default:
            ret = FLASH_NOR_RET_DEV_NOT_SUPPORT;
            break;
    }

    return ret;
}

__attribute__((section(".text_in_ram"))) cy_serial_flash_ret_type_t flash_nor_update_winbond_cmd(void)
{
    cy_serial_flash_ret_type_t ret = FLASH_NOR_RET_SUCCESS;

    switch (flash_nor_info.device_id)
    {
        default:
            ret = FLASH_NOR_RET_DEV_NOT_SUPPORT;
            break;
    }

    return ret;
}

__attribute__((section(".text_in_ram"))) cy_serial_flash_ret_type_t flash_nor_update_new_vendor_cmd(void)
{
    return FLASH_NOR_RET_VENDOR_NOT_SUPPORT;
}

__attribute__((section(".text_in_ram"))) cy_serial_flash_ret_type_t flash_nor_update_cmd(void)
{
    if ((flash_nor_get_exist() & FLASH_NOR_EXIST_BASIC_CMD) == 0)
    {
        return FLASH_NOR_RET_DEV_NOT_SUPPORT;
    }

    cy_serial_flash_ret_type_t ret = FLASH_NOR_RET_SUCCESS;

    if (flash_nor_get_exist() & FLASH_NOR_EXIST_ADV_CMD)
    {
        return ret;
    }

    switch (flash_nor_info.manu_id)
    {
        case FLASH_NOR_VENDOR_GD:
            ret = flash_nor_update_gd_cmd();
            break;
        case FLASH_NOR_VENDOR_MXIC:
            ret = flash_nor_update_mxic_cmd();
            break;
        case FLASH_NOR_VENDOR_WINBOND:
            ret = flash_nor_update_winbond_cmd();
            break;
        default:
            ret = flash_nor_update_new_vendor_cmd();
            break;
    }

    if (ret == FLASH_NOR_RET_SUCCESS)
    {
        flash_nor_set_exist(FLASH_NOR_EXIST_ADV_CMD);
    }

    return ret;
}

__attribute__((section(".text_in_ram"))) cy_serial_flash_ret_type_t flash_nor_load_query_info(void)
{
    flash_nor_exist_lv_t flash_exists_lv = flash_nor_get_exist();
    if ((flash_exists_lv & FLASH_NOR_EXIST_BASIC_CMD) == 0)
    {
        return FLASH_NOR_RET_NOT_EXIST_BASIC_CMD;
    }

    if (flash_exists_lv >= FLASH_NOR_EXIST_QUERY_INFO)
    {
        return FLASH_NOR_RET_SUCCESS;
    }

    uint8_t manu_id = flash_nor_info.manu_id;
    uint16_t device_id = flash_nor_info.device_id;

    for (uint8_t i = 0; i < _countof(flash_nor_query_info_table); i++)
    {
        if ((manu_id == flash_nor_query_info_table[i].manu_id) &&
            (device_id == flash_nor_query_info_table[i].device_id))
        {
            flash_nor_info.query = &flash_nor_query_info_table[i].query;
            goto query_info_load_successfully;
        }
    }

    return FLASH_NOR_RET_NOT_EXIST_QUERY_INFO;

query_info_load_successfully:

    flash_nor_set_exist(FLASH_NOR_EXIST_QUERY_INFO);
    return FLASH_NOR_RET_SUCCESS;
}

cy_serial_flash_ret_type_t flash_nor_write_enable(void)
{
    cy_serial_flash_ret_type_t ret = FLASH_NOR_RET_UNKNOWN;

    ret = flash_nor_cmd_tx(flash_nor_info.basic.cmd_wr_en, NULL, NULL, 0);

    return ret;
}

cy_serial_flash_ret_type_t flash_nor_write_disable(void)
{
    cy_serial_flash_ret_type_t ret = FLASH_NOR_RET_UNKNOWN;

    ret = flash_nor_cmd_tx(flash_nor_info.basic.cmd_wr_di, NULL, NULL, 0);

    return ret;
}

__attribute__((section(".text_in_ram"))) cy_serial_flash_ret_type_t flash_nor_wait_busy(void)
{
    bool is_wip = TRUE;
    uint32_t max_retry = flash_cfg.wait_max_retry;
    cy_serial_flash_ret_type_t ret = FLASH_NOR_RET_UNKNOWN;
    uint8_t sr = 0;

    do
    {

        flash_nor_cmd_rx(flash_nor_info.basic.cmd_rd_sr, NULL, 1, &sr);
        is_wip = (sr & FLASH_NOR_STATUS_WIP_BIT) == FLASH_NOR_STATUS_WIP_BIT;

        max_retry--;
    }
    while ((is_wip) && (max_retry));

    ret = (max_retry ? FLASH_NOR_RET_SUCCESS : FLASH_NOR_RET_WAIT_BUSY_FAILED);

    return ret;
}

 __attribute__((section(".text_in_ram"))) uint32_t cy_serial_flash_get_flash_size(void)
{
    if ((flash_nor_get_exist() & FLASH_NOR_EXIST_QUERY_INFO) == 0)
    {
        return 0;
    }

    return flash_nor_info.query->flash_size;
}

__attribute__((section(".text_in_ram"))) cy_serial_flash_ret_type_t cy_serial_flash_init(void)
{
    cy_serial_flash_ret_type_t ret = FLASH_NOR_RET_UNKNOWN;

    flash_nor_reset_exist();

    if((ret = flash_nor_sw_reset()) != FLASH_NOR_RET_SUCCESS)
    {
        return ret;
    }

    uint32_t rdid = flash_nor_get_rdid();
    if (rdid == 0)
    {
        return FLASH_NOR_RET_CMD_NOT_SUPPORT;
    }

    flash_nor_set_exist(FLASH_NOR_EXIST_BASIC_CMD);
    flash_nor_info.manu_id = (rdid >> 16) & 0xFF;
    flash_nor_info.device_id = rdid & 0xFFFF;

    if ((ret = flash_nor_update_cmd()) != FLASH_NOR_RET_SUCCESS)
    {
        return ret;
    }
    if ((ret = flash_nor_load_query_info()) != FLASH_NOR_RET_SUCCESS)
    {
        return ret;
    }

    return FLASH_NOR_RET_SUCCESS;
}

__attribute__((section(".text_in_ram"))) cy_serial_flash_ret_type_t cy_serial_flash_erase(uint32_t addr, flash_nor_erase_mode_t mode)
{
    cy_serial_flash_ret_type_t ret = FLASH_NOR_RET_UNKNOWN;
    uint32_t flash_size = cy_serial_flash_get_flash_size();

    if (addr >= flash_size)
    {
        return FLASH_NOR_RET_ADDR_LARGER_THAN_FLASH_SIZE;
    }

    uint8_t cmd = (mode == FLASH_NOR_ERASE_SECTOR) ? flash_nor_info.basic.cmd_sector_e :
                      (mode == FLASH_NOR_ERASE_BLOCK) ? flash_nor_info.basic.cmd_block_e :
                      flash_nor_info.basic.cmd_chip_e;

    if ((ret = flash_nor_write_enable()) != FLASH_NOR_RET_SUCCESS)
    {
        return ret;
    }

    if (mode != FLASH_NOR_ERASE_CHIP)
    {
        uint32_t align = 0;
        if (mode == FLASH_NOR_ERASE_SECTOR)
        {
            align = FLASH_NOR_SECTOR_SIZE;
        }
        else
        {
            align = FLASH_NOR_BLOCK_SIZE;
        }
        addr &= ~(align - 1);

        ret = flash_nor_cmd_tx(cmd, &addr, NULL, 0);
    }
    else
    {
        ret = flash_nor_cmd_tx(cmd, NULL, NULL, 0);
    }

    if(ret != FLASH_NOR_RET_SUCCESS)
    {
        return ret;
    }

    ret = flash_nor_wait_busy();

    return ret;
}

__attribute__((section(".text_in_ram"))) cy_serial_flash_ret_type_t cy_serial_flash_write(uint32_t addr, uint8_t *data, uint32_t byte_len)
{
    if (data == NULL)
    {
        return FLASH_NOR_RET_PARAM_INVALID;
    }

    cy_serial_flash_ret_type_t ret = FLASH_NOR_RET_UNKNOWN;
    uint32_t flash_size = cy_serial_flash_get_flash_size();
    uint32_t remain_byte_len = byte_len;
    uint32_t write_idx = 0;
    uint32_t write_byte_len = 0;
    uint32_t current_addr = 0;

    if (addr >= flash_size || (addr + byte_len > flash_size))
    {
        return FLASH_NOR_RET_ADDR_LARGER_THAN_FLASH_SIZE;
    }

    while(remain_byte_len > 0)
    {
        write_byte_len = FLASH_NOR_PAGE_SIZE;

        current_addr = addr + write_idx;

        if (write_byte_len > remain_byte_len)
        {
            write_byte_len = remain_byte_len;
        }

        if (IS_SAME_FLASH_NOR_PAGE(current_addr, current_addr + write_byte_len) == FALSE)
        {
            write_byte_len = write_byte_len - (current_addr + write_byte_len) % FLASH_NOR_PAGE_SIZE;
        }

        if ((ret = flash_nor_write_enable()) != FLASH_NOR_RET_SUCCESS)
        {
            return ret;
        }

        if ((ret = flash_nor_cmd_tx(flash_nor_info.basic.cmd_pp, &current_addr, &data[write_idx], write_byte_len)) != FLASH_NOR_RET_SUCCESS)
        {
            return ret;
        }

        if ((ret = flash_nor_wait_busy()) != FLASH_NOR_RET_SUCCESS)
        {
            return ret;
        }

        write_idx += write_byte_len;
        remain_byte_len -= write_byte_len;
   }

   ret = flash_nor_write_disable();

   return ret;
}

__attribute__((section(".text_in_ram"))) cy_serial_flash_ret_type_t cy_serial_flash_read(uint32_t addr, uint8_t *data, uint32_t byte_len)
{
    if (data == NULL)
    {
        return FLASH_NOR_RET_PARAM_INVALID;
    }

    uint32_t flash_size = cy_serial_flash_get_flash_size();
    uint32_t read_idx = 0;
    uint32_t read_byte_len = 0;
    uint32_t remain_byte_len = byte_len;
    uint32_t current_addr = 0;

    if (addr >= flash_size || (addr + byte_len > flash_size))
    {
        return FLASH_NOR_RET_ADDR_LARGER_THAN_FLASH_SIZE;
    }

    read_byte_len = SPIFFY2_RX_BUF_LEN - (FLASH_NOR_CMD_LENGTH + FLASH_NOR_ADDR_LENGTH);

    while (remain_byte_len > read_byte_len)
    {
        current_addr = addr + read_idx;
        flash_nor_cmd_rx(flash_nor_info.basic.cmd_rd_data, &current_addr, read_byte_len, &data[read_idx]);

        read_idx += read_byte_len;
        remain_byte_len -= read_byte_len;
    }

    current_addr = addr + read_idx;
    flash_nor_cmd_rx(flash_nor_info.basic.cmd_rd_data, &current_addr, remain_byte_len, &data[read_idx]);

    return FLASH_NOR_RET_SUCCESS;
}

__attribute__((section(".text_in_ram"))) cy_serial_flash_ret_type_t cy_serial_flash_deep_power_down(void)
{
    cy_serial_flash_ret_type_t ret = FLASH_NOR_RET_UNKNOWN;

    ret = flash_nor_cmd_tx(flash_nor_info.basic.cmd_dp, NULL, NULL, 0);

    return ret;
}

__attribute__((section(".text_in_ram"))) cy_serial_flash_ret_type_t cy_serial_flash_deep_power_down_release(void)
{
    cy_serial_flash_ret_type_t ret = FLASH_NOR_RET_UNKNOWN;

    ret = flash_nor_cmd_tx(flash_nor_info.basic.cmd_dp_release, NULL, NULL, 0);

    return ret;
}
