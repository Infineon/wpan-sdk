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
        HDC2010.h
        Created By: Brandon Fisher, August 1st 2017

        This code is release AS-IS into the public domain, no guarantee or warranty is given.

        Description: This header file accompanies HDC2010.cpp, and declares all methods, fields,
        and constants used in the source code.
*/

#ifndef __HDC2010_H__
#define __HDC2010_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

//  Constants for setting measurement resolution
#define WICED_HDC2010_FOURTEEN_BIT 0
#define WICED_HDC2010_ELEVEN_BIT 1
#define WICED_HDC2010_NINE_BIT 2

//  Constants for setting sensor mode
#define WICED_HDC2010_TEMP_AND_HUMID 0
#define WICED_HDC2010_TEMP_ONLY 1
#define WICED_HDC2010_HUMID_ONLY 2
#define WICED_HDC2010_ACTIVE_LOW 0
#define WICED_HDC2010_ACTIVE_HIGH 1
#define WICED_HDC2010_LEVEL_MODE 0
#define WICED_HDC2010_COMPARATOR_MODE 1

//  Constants for setting sample rate
#define WICED_HDC2010_MANUAL 0
#define WICED_HDC2010_TWO_MINS 1
#define WICED_HDC2010_ONE_MINS 2
#define WICED_HDC2010_TEN_SECONDS 3
#define WICED_HDC2010_FIVE_SECONDS 4
#define WICED_HDC2010_ONE_HZ 5
#define WICED_HDC2010_TWO_HZ 6
#define WICED_HDC2010_FIVE_HZ 7

void wiced_hdc2010_address(uint8_t addr);          // Initialize the hdc2010
void wiced_hdc2010_begin(void);                    // Join I2C bus
int16_t wiced_hdc2010_read_temp(void);             // Returns the temperature in degrees C
uint8_t wiced_hdc2010_read_humidity(void);         // Returns the relative humidity
void wiced_hdc2010_trigger_measurement(void);      // Triggers a manual temperature/humidity reading
void wiced_hdc2010_reset(void);                    // Triggers a software reset
void wiced_hdc2010_enable_interrupt(void);         // Enables the interrupt/DRDY pin
void wiced_hdc2010_disable_interrupt(void);        // Disables the interrupt/DRDY pin (High Z)
uint8_t wiced_hdc2010_read_interrupt_status(void); // Reads the status of the interrupt register
void wiced_hdc2010_enable_DRDY_interrupt(void);    // Enables data ready interrupt
void wiced_hdc2010_disable_DRDY_interrupt(void);   // Disables data ready interrupt

/* Sets Temperature & Humidity Resolution, 3 options
        0 - 14 bit
        1 - 11 bit
        2 - 9 bit
        default - 14 bit							*/
void wiced_hdc2010_set_temp_res(int resolution);
void wiced_hdc2010_set_humid_res(int resolution);

/* Sets measurement mode, 3 options
        0 - Temperature and Humidity
        1 - Temperature only
        2 - Humidity only
        default - Temperature & Humidity			*/
void wiced_hdc2010_set_measurement_mode(int mode);

/* Sets reading rate, 8 options
        0 - Manual
        1 - reading every 2 minutes
        2 - reading every minute
        3 - reading every ten seconds
        4 - reading every 5 seconds
        5 - reading every second
        6 - reading at 2Hz
        7 - reading at 5Hz
        default - Manual		*/
void wiced_hdc2010_set_rate(int rate);

/* Sets Interrupt polarity, 2 options
        0 - Active Low
        1 - Active High
        default - Active Low			*/
void wiced_hdc2010_set_interrupt_polarity(int polarity);

/* Sets Interrupt mode, 2 options
        0 - Level sensitive
        1 - Comparator mode
        default - Level sensitive	*/
void wiced_hdc2010_set_interrupt_mode(int polarity);

uint8_t wiced_hdc2010_read_reg(uint8_t reg);             // Reads a given register, returns 1 byte
void wiced_hdc2010_write_reg(uint8_t reg, uint8_t data); // Writes a byte of data to one register
void wiced_hdc2010_temp_adjust(uint8_t data);
void wiced_hdc2010_humidity_adjust(uint8_t data);

#if defined(__cplusplus)
}
#endif
#endif /* __HDC2010_H__ */
