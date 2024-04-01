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

#include "display-oled-ssd1306.h"
#include "mtb_ssd1306_i2c.h"

/*********************************************************************
*
*       wiced_hal_i2c_write
*
* Purpose:
*   Initializes the display controller
*/
void CYW9BTMDM2BASE1_InitController(void)
{
    mtb_ssd1306_write_command_byte(OLED_DISPLAY_OFF);

    for(uint8_t i=0;i<8;i++)
    {   mtb_ssd1306_write_command_byte(0xb0+i);
        mtb_ssd1306_write_command_byte(0x00);
        mtb_ssd1306_write_command_byte(0x10);
        for(uint8_t n=0;n<128;n++)
            mtb_ssd1306_write_data_byte(0);                         //Clear screen
    }

    mtb_ssd1306_write_command_byte(OLED_SET_DISPLAY_CLOCK_DIV);
    mtb_ssd1306_write_command_byte(OLED_DISPLAY_CLOCK_DIVIDE_RATIO |
                                  (OLED_OSCILLATOR_FREQUENCY << 4));

    mtb_ssd1306_write_command_byte(OLED_SET_MULTIPLEX);
    mtb_ssd1306_write_command_byte(YSIZE_PHYS - 1);

    mtb_ssd1306_write_command_byte(OLED_SET_DISPLAY_OFFSET);
    mtb_ssd1306_write_command_byte(OLED_DISPLAY_OFFSET);

    mtb_ssd1306_write_command_byte(OLED_SET_START_LINE | 0x0);     // line #0

    mtb_ssd1306_write_command_byte(OLED_CHARGE_PUMP);
    mtb_ssd1306_write_command_byte(OLED_ENABLE_CHARGE_PUMP);

    mtb_ssd1306_write_command_byte(OLED_MEMORY_MODE);
    mtb_ssd1306_write_command_byte(OLED_MEMORY_MODE_PAGE_ADDRESSING);

    mtb_ssd1306_write_command_byte(OLED_SEGMENT_REMAP | 0x1);

    mtb_ssd1306_write_command_byte(OLED_SET_COM_SCAN_DIRECTION);

    mtb_ssd1306_write_command_byte(OLED_SET_COM_PINS);
    mtb_ssd1306_write_command_byte(OLED_SET_COM_PINS_YSIZE);

    mtb_ssd1306_write_command_byte(OLED_SET_CONTRAST);
    mtb_ssd1306_write_command_byte(OLED_CONTRAST);

    mtb_ssd1306_write_command_byte(OLED_SET_PRECHARGE);
    mtb_ssd1306_write_command_byte(OLED_PRECHARGE);

    mtb_ssd1306_write_command_byte(OLED_SET_VCOMH_DESELECT_LEVEL);
    mtb_ssd1306_write_command_byte(OLED_VCOMH_DESELECT_LEVEL);

    mtb_ssd1306_write_command_byte(OLED_ENTIRE_DISPLAY_ON_RESUME);

    mtb_ssd1306_write_command_byte(OLED_NORMAL_DISPLAY);

    mtb_ssd1306_write_command_byte(OLED_DEACTIVATE_SCROLL);

    mtb_ssd1306_write_command_byte(OLED_DISPLAY_ON);
}
