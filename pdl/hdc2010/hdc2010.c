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
        HDC2010.cpp
        Created By: Brandon Fisher, August 1st 2017

        This code is release AS-IS into the public domain, no guarantee or warranty is given.

        Description: This library facilitates communication with, and configuration of,
        the HDC2010 Temperature and Humidity Sensor. It makes extensive use of the
        Wire.H library, and should be useable with both Arduino and Energia.
*/

#include "hdc2010.h"

#include "cy_utils.h"
#include "cyhal_i2c.h"
#include "wiced_rtos.h"

// Define Register Map
#define TEMP_LOW 0x00
#define TEMP_HIGH 0x01
#define HUMID_LOW 0x02
#define HUMID_HIGH 0x03
#define INTERRUPT_DRDY 0x04
#define TEMP_MAX 0x05
#define HUMID_MAX 0x06
#define INTERRUPT_CONFIG 0x07
#define TEMP_OFFSET_ADJUST 0x08
#define HUM_OFFSET_ADJUST 0x09
#define TEMP_THR_L 0x0A
#define TEMP_THR_H 0x0B
#define HUMID_THR_L 0x0C
#define HUMID_THR_H 0x0D
#define CONFIG 0x0E
#define MEASUREMENT_CONFIG 0x0F
#define MID_L 0xFC
#define MID_H 0xFD
#define DEVICE_ID_L 0xFE
#define DEVICE_ID_H 0xFF

#define WRITE_BUF_SIZE 2
#define WAIT_POST_RESET 3 /* Wait time in ms after reset        */

static int _addr; // Address of sensor

void wiced_hdc2010_address(uint8_t addr)
{
    _addr = addr;
}

int16_t wiced_hdc2010_read_temp(void)
{
    uint8_t byte[2];
    uint16_t temp;
    byte[0] = wiced_hdc2010_read_reg(TEMP_LOW);
    byte[1] = wiced_hdc2010_read_reg(TEMP_HIGH);

    temp = (unsigned int) byte[1] << 8 | byte[0];

    return (temp * 165 / 65536) - 40;
}

uint8_t wiced_hdc2010_read_humidity(void)
{
    uint8_t byte[2];
    uint16_t humidity;
    byte[0] = wiced_hdc2010_read_reg(HUMID_LOW);
    byte[1] = wiced_hdc2010_read_reg(HUMID_HIGH);

    humidity = (unsigned int) byte[1] << 8 | byte[0];
    return (humidity * 100) / 65536;
}

uint8_t wiced_hdc2010_read_reg(uint8_t reg)
{
    uint8_t reading; // holds byte of read data
    cy_rslt_t rslt;

    rslt = cyhal_i2c_master_write(NULL, _addr, &reg, 1, 0, true);
    CY_UNUSED_PARAMETER(rslt); // CY_ASSERT only processes in DEBUG, ignores for others
    CY_ASSERT(CY_RSLT_SUCCESS == rslt);

    rslt = cyhal_i2c_master_read(NULL, _addr, &reading, 1, 0, true);
    CY_UNUSED_PARAMETER(rslt); // CY_ASSERT only processes in DEBUG, ignores for others
    CY_ASSERT(CY_RSLT_SUCCESS == rslt);

    return reading;
}

void wiced_hdc2010_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t writeBuffer[WRITE_BUF_SIZE] = { reg, data };

    // Write the buffer to display controller
    cy_rslt_t rslt = cyhal_i2c_master_write(NULL, _addr, writeBuffer, WRITE_BUF_SIZE, 0, true);
    CY_UNUSED_PARAMETER(rslt); // CY_ASSERT only processes in DEBUG, ignores for others
    CY_ASSERT(CY_RSLT_SUCCESS == rslt);
}

/* Upper two bits of the MEASUREMENT_CONFIG register controls
   the temperature resolution*/
void wiced_hdc2010_set_temp_res(int resolution)
{
    uint8_t configContents;
    configContents = wiced_hdc2010_read_reg(MEASUREMENT_CONFIG);

    switch (resolution)
    {
    case WICED_HDC2010_FOURTEEN_BIT:
        configContents = (configContents & 0x3F);
        break;

    case WICED_HDC2010_ELEVEN_BIT:
        configContents = (configContents & 0x7F);
        configContents = (configContents | 0x40);
        break;

    case WICED_HDC2010_NINE_BIT:
        configContents = (configContents & 0xBF);
        configContents = (configContents | 0x80);
        break;

    default:
        configContents = (configContents & 0x3F);
    }

    wiced_hdc2010_write_reg(MEASUREMENT_CONFIG, configContents);
}
/*  Bits 5 and 6 of the MEASUREMENT_CONFIG register controls
    the humidity resolution*/
void wiced_hdc2010_set_humid_res(int resolution)
{
    uint8_t configContents;
    configContents = wiced_hdc2010_read_reg(MEASUREMENT_CONFIG);

    switch (resolution)
    {
    case WICED_HDC2010_FOURTEEN_BIT:
        configContents = (configContents & 0xCF);
        break;

    case WICED_HDC2010_ELEVEN_BIT:
        configContents = (configContents & 0xDF);
        configContents = (configContents | 0x10);
        break;

    case WICED_HDC2010_NINE_BIT:
        configContents = (configContents & 0xEF);
        configContents = (configContents | 0x20);
        break;

    default:
        configContents = (configContents & 0xCF);
    }

    wiced_hdc2010_write_reg(MEASUREMENT_CONFIG, configContents);
}

/*  Bits 2 and 1 of the MEASUREMENT_CONFIG register controls
    the measurement mode  */
void wiced_hdc2010_set_measurement_mode(int mode)
{
    uint8_t configContents;
    configContents = wiced_hdc2010_read_reg(MEASUREMENT_CONFIG);

    switch (mode)
    {
    case WICED_HDC2010_TEMP_AND_HUMID:
        configContents = (configContents & 0xF9);
        break;

    case WICED_HDC2010_TEMP_ONLY:
        configContents = (configContents & 0xFC);
        configContents = (configContents | 0x02);
        break;

    case WICED_HDC2010_HUMID_ONLY:
        configContents = (configContents & 0xFD);
        configContents = (configContents | 0x04);
        break;

    default:
        configContents = (configContents & 0xF9);
    }

    wiced_hdc2010_write_reg(MEASUREMENT_CONFIG, configContents);
}

/*  Bit 0 of the MEASUREMENT_CONFIG register can be used
    to trigger measurements  */
void wiced_hdc2010_trigger_measurement(void)
{
    uint8_t configContents;
    configContents = wiced_hdc2010_read_reg(MEASUREMENT_CONFIG);

    configContents = (configContents | 0x01);
    wiced_hdc2010_write_reg(MEASUREMENT_CONFIG, configContents);
}

/*  Bit 7 of the CONFIG register can be used to trigger a
    soft reset  */
void wiced_hdc2010_reset(void)
{
    uint8_t configContents;
    configContents = wiced_hdc2010_read_reg(CONFIG);

    configContents = (configContents | 0x80);
    wiced_hdc2010_write_reg(CONFIG, configContents);
    wiced_rtos_delay_milliseconds(WAIT_POST_RESET, ALLOW_THREAD_TO_SLEEP);
}

/*  Bit 2 of the CONFIG register can be used to enable/disable
    the interrupt pin  */
void wiced_hdc2010_enable_interrupt(void)
{
    uint8_t configContents;
    configContents = wiced_hdc2010_read_reg(CONFIG);

    configContents = (configContents | 0x04);
    wiced_hdc2010_write_reg(CONFIG, configContents);
}

/*  Bit 2 of the CONFIG register can be used to enable/disable
    the interrupt pin  */
void wiced_hdc2010_disable_interrupt(void)
{
    uint8_t configContents;
    configContents = wiced_hdc2010_read_reg(CONFIG);

    configContents = (configContents & 0xFB);
    wiced_hdc2010_write_reg(CONFIG, configContents);
}

/*  Bits 6-4  of the CONFIG register controls the measurement
    rate  */
void wiced_hdc2010_set_rate(int rate)
{
    uint8_t configContents;
    configContents = wiced_hdc2010_read_reg(CONFIG);

    switch (rate)
    {
    case WICED_HDC2010_MANUAL:
        configContents = (configContents & 0x8F);
        break;

    case WICED_HDC2010_TWO_MINS:
        configContents = (configContents & 0x9F);
        configContents = (configContents | 0x10);
        break;

    case WICED_HDC2010_ONE_MINS:
        configContents = (configContents & 0xAF);
        configContents = (configContents | 0x20);
        break;

    case WICED_HDC2010_TEN_SECONDS:
        configContents = (configContents & 0xBF);
        configContents = (configContents | 0x30);
        break;

    case WICED_HDC2010_FIVE_SECONDS:
        configContents = (configContents & 0xCF);
        configContents = (configContents | 0x40);
        break;

    case WICED_HDC2010_ONE_HZ:
        configContents = (configContents & 0xDF);
        configContents = (configContents | 0x50);
        break;

    case WICED_HDC2010_TWO_HZ:
        configContents = (configContents & 0xEF);
        configContents = (configContents | 0x60);
        break;

    case WICED_HDC2010_FIVE_HZ:
        configContents = (configContents | 0x70);
        break;

    default:
        configContents = (configContents & 0x8F);
    }

    wiced_hdc2010_write_reg(CONFIG, configContents);
}

/*  Bit 1 of the CONFIG register can be used to control the
    the interrupt pins polarity */
void wiced_hdc2010_set_interrupt_polarity(int polarity)
{
    uint8_t configContents;
    configContents = wiced_hdc2010_read_reg(CONFIG);

    switch (polarity)
    {
    case WICED_HDC2010_ACTIVE_LOW:
        configContents = (configContents & 0xFD);
        break;

    case WICED_HDC2010_ACTIVE_HIGH:
        configContents = (configContents | 0x02);
        break;

    default:
        configContents = (configContents & 0xFD);
    }

    wiced_hdc2010_write_reg(CONFIG, configContents);
}

/*  Bit 0 of the CONFIG register can be used to control the
    the interrupt pin's mode */
void wiced_hdc2010_set_interrupt_mode(int mode)
{
    uint8_t configContents;
    configContents = wiced_hdc2010_read_reg(CONFIG);

    switch (mode)
    {
    case WICED_HDC2010_LEVEL_MODE:
        configContents = (configContents & 0xFE);
        break;

    case WICED_HDC2010_COMPARATOR_MODE:
        configContents = (configContents | 0x01);
        break;

    default:
        configContents = (configContents & 0xFE);
    }

    wiced_hdc2010_write_reg(CONFIG, configContents);
}

uint8_t wiced_hdc2010_read_interrupt_status(void)
{
    uint8_t regContents;
    regContents = wiced_hdc2010_read_reg(INTERRUPT_DRDY);
    return regContents;
}

// enables the interrupt pin for DRDY operation
void wiced_hdc2010_enable_DRDY_interrupt(void)
{
    uint8_t regContents;
    regContents = wiced_hdc2010_read_reg(INTERRUPT_CONFIG);

    regContents = (regContents | 0x80);

    wiced_hdc2010_write_reg(INTERRUPT_CONFIG, regContents);
}

// disables the interrupt pin for DRDY operation
void wiced_hdc2010_disable_DRDY_interrupt(void)
{
    uint8_t regContents;
    regContents = wiced_hdc2010_read_reg(INTERRUPT_CONFIG);

    regContents = (regContents & 0x7F);

    wiced_hdc2010_write_reg(INTERRUPT_CONFIG, regContents);
}

void wiced_hdc2010_temp_adjust(uint8_t data)
{
    wiced_hdc2010_write_reg(TEMP_OFFSET_ADJUST, data);
}

void wiced_hdc2010_humidity_adjust(uint8_t data)
{
    wiced_hdc2010_write_reg(HUM_OFFSET_ADJUST, data);
}
