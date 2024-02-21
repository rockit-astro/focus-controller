//**********************************************************************************
//  Adapted from https://gist.github.com/stecman/9ec74de5e8a5c3c6341c791d9c233adc
//  which was released under the Creative Commons Zero licence.
//  Modifications copyright 2023, 2024 Paul Chote, All Rights Reserved
//**********************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <util/crc16.h>
#include <util/delay.h>
#include "onewire.h"

static const uint8_t kConvertT = 0x44;
static const uint8_t kConvertV = 0xB4;
static const uint8_t kRecallPage = 0xB8;
static const uint8_t kReadScatchPad = 0xBE;

static uint8_t crc8(uint8_t* data, uint8_t len)
{
    uint8_t crc = 0;

    for (uint8_t i = 0; i < len; ++i) {
        crc = _crc_ibutton_update(crc, data[i]);
    }

    return crc;
}

static bool onewire_reset(const gpin_t* io)
{
    // Configure for output
    gpio_output_set_high(io);
    gpio_configure_output(io);

    // Pull low for >480uS (master reset pulse)
    gpio_output_set_low(io);
    _delay_us(480);

    // Configure for input
    gpio_configure_input_hiz(io);
    _delay_us(70);

    // Look for the line pulled low by a slave
    uint8_t result = gpio_input_read(io);

    // Wait for the presence pulse to finish
    // This should be less than 240uS, but the master is expected to stay
    // in Rx mode for a minimum of 480uS in total
    _delay_us(460);

    return result == 0;
}

/**
 * Output a Write-0 or Write-1 slot on the One Wire bus
 * A Write-1 slot is generated unless the passed value is zero
 */
static void onewire_write_bit(const gpin_t* io, uint8_t bit)
{
    if (bit != 0) { // Write high

        // Pull low for less than 15uS to write a high
        gpio_output_set_low(io);
        _delay_us(5);
        gpio_output_set_high(io);

        // Wait for the rest of the minimum slot time
        _delay_us(55);

    } else { // Write low

        // Pull low for 60 - 120uS to write a low
        gpio_output_set_low(io);
        _delay_us(55);

        // Stop pulling down line
        gpio_output_set_high(io);

        // Recovery time between slots
        _delay_us(5);
    }
}

// One Wire timing is based on this Maxim application note
// https://www.maximintegrated.com/en/app-notes/index.mvp/id/126
static void onewire_write(const gpin_t* io, uint8_t byte)
{
    // Configure for output
    gpio_output_set_low(io);
    gpio_configure_output(io);

    for (uint8_t i = 8; i != 0; --i) {
        onewire_write_bit(io, byte & 0x1);

        // Next bit (LSB first)
        byte >>= 1;
    }
}

/**
 * Generate a read slot on the One Wire bus and return the bit value
 * Return 0x0 or 0x1
 */
static uint8_t onewire_read_bit(const gpin_t* io)
{
    // Pull the 1-wire bus low for >1uS to generate a read slot
    gpio_output_set_low(io);
    gpio_configure_output(io);
    _delay_us(1);

    // Configure for reading (releases the line)
    gpio_configure_input_hiz(io);

    // Wait for value to stabilise (bit must be read within 15uS of read slot)
    _delay_us(10);

    uint8_t result = gpio_input_read(io) != 0;

    // Wait for the end of the read slot
    _delay_us(50);

    return result;
}

static uint8_t onewire_read(const gpin_t* io)
{
    uint8_t buffer = 0x0;
    gpio_configure_input_hiz(io);
    for (uint8_t bit = 0x01; bit; bit <<= 1)
        if (onewire_read_bit(io))
            buffer |= bit;

    return buffer;
}

static bool ds2438_read(const gpin_t* io, uint8_t page, uint8_t buffer[9])
{
    // Confirm the device is still alive. Abort if no reply
    if (!onewire_reset(io))
        return false;

    onewire_write(io, 0xCC);
    onewire_write(io, kRecallPage);
    onewire_write(io, page);

    if (!onewire_reset(io))
        return false;

    onewire_write(io, 0xCC);
    onewire_write(io, kReadScatchPad);
    onewire_write(io, page);

    for (int8_t i = 0; i < 9; ++i)
        buffer[i] = onewire_read(io);

    if (crc8(buffer, 8) != buffer[8])
        return false;

    return true;
}

bool ds2438_measure(const gpin_t* io, char output[20])
{
    uint8_t buffer[9];
    if (!onewire_reset(io))
        return false;

    onewire_write(io, 0xCC);

    // Switch to VAD
    onewire_write(io, 0x4E);
    onewire_write(io, 0x00);
    onewire_write(io, 0x00);

    _delay_ms(20);
    if (!onewire_reset(io))
        return false;

    onewire_write(io, 0xCC);
    onewire_write(io, kConvertT);
    _delay_ms(20);

    if (!onewire_reset(io))
        return false;

    onewire_write(io, 0xCC);
    onewire_write(io, kConvertV);
    _delay_ms(20);

    if (!ds2438_read(io, 0, buffer))
        return false;

    const uint16_t vad = ((buffer[4] << 8) | buffer[3]) * 10;

    _delay_ms(20);
    if (!onewire_reset(io))
        return false;

    onewire_write(io, 0xCC);

    // Switch to VDD
    onewire_write(io, 0x4E);
    onewire_write(io, 0x00);
    onewire_write(io, 0x08);
	
    if (!onewire_reset(io))
        return false;

    onewire_write(io, 0xCC);
    onewire_write(io, kConvertV);
    _delay_ms(20);

    if (!onewire_reset(io))
        return false;

    if (!ds2438_read(io, 0, buffer))
        return false;

    const uint16_t vdd = ((buffer[4] << 8) | buffer[3]) * 10;
    const uint8_t temp_integer = buffer[2];
    const uint16_t temp_frac = (buffer[1] >> 3) * 32;

    float temperature = temp_integer + temp_frac / 1000.0f;
    float sensor_rh = (vad * 1.0f / vdd - 0.16f) / (0.0062f * (1.0546f - 0.00216f * temperature));

    sprintf(output, "TH;%0.3f;%0.3f\r\n", temperature, sensor_rh);
    return true;
}

static bool ds1820_read(const gpin_t* io, uint8_t buffer[9])
{
    // Confirm the device is still alive. Abort if no reply
    if (!onewire_reset(io))
        return false;

    onewire_write(io, 0xCC);
    onewire_write(io, kReadScatchPad);

    for (int8_t i = 0; i < 9; ++i)
        buffer[i] = onewire_read(io);

    if (crc8(buffer, 8) != buffer[8])
        return false;

    return true;
}

bool ds1820_measure(const gpin_t* io, char output[20], uint8_t bits)
{
    uint8_t buffer[9];
    if (!onewire_reset(io))
        return false;

    onewire_write(io, 0xCC);
    onewire_write(io, kConvertT);

    if (!ds1820_read(io, buffer))
        return false;

    if (bits == 9)
        sprintf(output, "T;%0.1f\r\n", ((buffer[1] << 8) | buffer[0]) * 0.5f);
    else
        sprintf(output, "T;%0.3f\r\n", ((buffer[1] << 8) | buffer[0]) * 0.0625f);
    return true;
}

void onewire_measure(const gpin_t* io, char output[20])
{
    if (!onewire_reset(io))
    {
        strcpy(output, "NONE\r\n");
        return;
    }

    onewire_write(io, 0x33);
    uint8_t type = onewire_read(io);
    switch (type)
    {
        case 0x10: ds1820_measure(io, output, 9); break;
        case 0x26: ds2438_measure(io, output); break;
        case 0x28: ds1820_measure(io, output, 12); break;
        default: sprintf(output, "UNKNOWN;0x%02X\r\n", type);
    }
}
