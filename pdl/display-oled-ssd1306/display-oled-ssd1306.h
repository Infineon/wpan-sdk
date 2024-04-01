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

#include "mtb_ssd1306_i2c.h"

/*********************************************************************
*
*       Layer configuration
*
**********************************************************************
*/
//
// Physical display size
//
#define XSIZE_PHYS MTB_DISPLAY_SIZE_X
#define YSIZE_PHYS MTB_DISPLAY_SIZE_Y

//
// Color conversion
//
#define COLOR_CONVERSION GUICC_1

//
// Display driver, display data cache enabled
//
#define DISPLAY_DRIVER GUIDRV_SPAGE_1C1

/*********************************************************************
*
*       Configuration checking
*
**********************************************************************
*/
#ifndef   VXSIZE_PHYS
  #define VXSIZE_PHYS XSIZE_PHYS
#endif
#ifndef   VYSIZE_PHYS
  #define VYSIZE_PHYS YSIZE_PHYS
#endif
#ifndef   XSIZE_PHYS
  #error Physical X size of display is not defined!
#endif
#ifndef   YSIZE_PHYS
  #error Physical Y size of display is not defined!
#endif
#ifndef   COLOR_CONVERSION
  #error Color conversion not defined!
#endif
#ifndef   DISPLAY_DRIVER
  #error No display driver defined!
#endif

/* OLED display controller commands */
#define OLED_SET_CONTRAST             0x81
#define OLED_ENTIRE_DISPLAY_ON_RESUME 0xA4
#define OLED_ENTIRE_DISPLAY_ON        0xA5
#define OLED_NORMAL_DISPLAY           0xA6
#define OLED_INVERT_DISPLAY           0xA7
#define OLED_DISPLAY_OFF              0xAE
#define OLED_DISPLAY_ON               0xAF
#define OLED_SET_DISPLAY_OFFSET       0xD3
#define OLED_SET_COM_PINS             0xDA
#define OLED_SET_VCOMH_DESELECT_LEVEL 0xDB
#define OLED_SET_DISPLAY_CLOCK_DIV    0xD5
#define OLED_SET_PRECHARGE            0xD9
#define OLED_SET_MULTIPLEX            0xA8
#define OLED_SET_LOW_COLUMN           0x00
#define OLED_SET_HIGH_COLUMN          0x10
#define OLED_SET_START_LINE           0x40
#define OLED_MEMORY_MODE              0x20
#define OLED_COLUMN_ADDR              0x21
#define OLED_PAGE_ADDR                0x22
#define OLED_COM_SCAN_INC             0xC0
#define OLED_SET_COM_SCAN_DIRECTION   0xC8
#define OLED_SEGMENT_REMAP            0xA0
#define OLED_CHARGE_PUMP              0x8D
#define OLED_EXTERNAL_VCC             0x01
#define OLED_SWITCH_CAP_VCC           0x02

/*********************************************************************
*
*       Defines: Configuration
*
**********************************************************************/
/* Misc settings */
#define OLED_DISPLAY_CLOCK_DIVIDE_RATIO        (0x0)
#define OLED_OSCILLATOR_FREQUENCY              (0xF)
#define OLED_DISPLAY_OFFSET                    (0x00)
#define OLED_ENABLE_CHARGE_PUMP                (0x14)
#define OLED_MEMORY_MODE_HORIZONTAL_ADDRESSING (0x00)
#define OLED_MEMORY_MODE_VERTICAL_ADDRESSING   (0x01)
#define OLED_MEMORY_MODE_PAGE_ADDRESSING       (0x02)
#define OLED_CONTRAST                          (0x8F)
#define OLED_PRECHARGE                         (0xF1)
#define OLED_VCOMH_DESELECT_LEVEL              (0x20)

#if YSIZE_PHYS == 64
    #define OLED_SET_COM_PINS_YSIZE (0x12)
#elif YSIZE_PHYS == 32
    #define OLED_SET_COM_PINS_YSIZE (0x02)
#endif

/* Scrolling #defines */
#define OLED_ACTIVATE_SCROLL                         0x2F
#define OLED_DEACTIVATE_SCROLL                       0x2E
#define OLED_SET_VERTICAL_SCROLL_AREA                0xA3
#define OLED_RIGHT_HORIZONTAL_SCROLL                 0x26
#define OLED_LEFT_HORIZONTAL_SCROLL                  0x27
#define OLED_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL    0x29
#define OLED_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL     0x2A

/*********************************************************************
*
*       wiced_hal_i2c_write
*
* Purpose:
*   Initializes the display controller
*/
void CYW9BTMDM2BASE1_InitController(void);
