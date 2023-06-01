//**********************************************************************************
//  Copyright 2016, 2017, 2022 Paul Chote, All Rights Reserved
//**********************************************************************************

#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "usb.h"

#define F_CPU 16000000UL
#include <util/delay.h>

#define length(array) (sizeof(array)/sizeof(*(array)))

typedef struct
{
    uint8_t enable_port;
    uint8_t enable_pin;
    
    uint8_t step_port;
    uint8_t step_pin;
    
    uint8_t dir_port;
    uint8_t dir_pin;
} channel;

channel channels[] = {
#if MODEL == 1
    {
        .enable_port = 'B', .enable_pin = 1,
        .step_port = 'B', .step_pin = 0,
        .dir_port = 'B', .dir_pin = 2
    }
#elif MODEL == 0
    {
        .enable_port = 'B', .enable_pin = 4,
        .step_port = 'D', .step_pin = 1,
        .dir_port = 'C', .dir_pin = 6
    },
    {
        .enable_port = 'B', .enable_pin = 4,
        .step_port = 'D', .step_pin = 0,
        .dir_port = 'D', .dir_pin = 7
    }
#endif
};

// GRBL board shares the same enable pin for multiple motors
// Defining GLOBAL_ENABLE_PIN changes the enable behaviour
// to work over the set of all channels
#if MODEL == 0
#define GLOBAL_ENABLE_PIN 1
#endif

#define CHANNEL_COUNT (sizeof(channels)/sizeof(*(channels)))

// The raw motor resolution is too fine to be useful
// Work internally at 64x resolution, which allows 7 digits of external resolution.

#define DOWNSAMPLE_BITS 4

volatile bool led_active;
char output[256];

uint8_t command_length = 0;
char command_buffer[16];

int32_t target_steps[CHANNEL_COUNT] = {};
int32_t current_steps[CHANNEL_COUNT] = {};
bool enabled[CHANNEL_COUNT] = {};
bool step_high[CHANNEL_COUNT] = {};

static void update_eeprom(uint8_t i, int32_t target)
{
    // Save the current absolute position so we can recover
    // the absolute position after a power cycle.
    // TODO: Implement a wear levelling strategy
    eeprom_update_dword((uint32_t*)(4 * i), target);
}

static int32_t read_eeprom(uint8_t i)
{
    return eeprom_read_dword((uint32_t*)(4 * i));
}

static void print_string(char *message)
{
    usb_write_data(message, strlen(message));
}

static void loop(void)
{
    while (usb_can_read())
    {
        int16_t value = usb_read();
        if (value < 0)
            break;

        if (command_length > 0 && (value == '\r' || value == '\n'))
        {
            char *cb = command_buffer;
            if (command_length == 1 && cb[0] == '?')
            {
                for (uint8_t i = 0; i < CHANNEL_COUNT; i++)
                {
                    
                    sprintf(output + i * 22, "T%01d=%+07ld,C%01d=%+07ld,",
                        i + 1,
                        target_steps[i] >> DOWNSAMPLE_BITS,
                        i + 1,
                        current_steps[i] >> DOWNSAMPLE_BITS);
                }
                
                sprintf(output + CHANNEL_COUNT * 22 - 1, "\r\n");
                
                print_string(output);
            }
            else if (cb[0] > '0' && cb[0] <= '0' + CHANNEL_COUNT)
            {
                // 0-indexed channel number
                uint8_t i = cb[0] - '1';

                // Stop at current position: [1..9]S\r\n
                if (command_length == 2 && cb[1] == 'S')
                {
                    cli();

                    target_steps[i] = current_steps[i];
                    update_eeprom(i, target_steps[i]);

                    sei();
                    print_string("$\r\n");
                }
                // Zero at current position: [1..9]Z\r\n
                else if (command_length == 2 && cb[1] == 'Z')
                {
                    cli();

                    target_steps[i] = current_steps[i] = 0;
                    update_eeprom(i, 0);

                    sei();
                    print_string("$\r\n");
                }
                // Move to position: [1..9][+-]1234567\r\n
                else if (command_length > 2 && (cb[1] == '+' || cb[1] == '-'))
                {
                    bool is_number = command_length <= 9;
                    for (uint8_t i = 2; i < command_length; i++)
                    {
                        if (cb[i] < '0' || cb[i] > '9')
                        {
                            is_number = false;
                            break;
                        }
                    }
                    
                    if (is_number)
                    {
                        // atoi expects a null terminated string
                        command_buffer[command_length++] = '\0';
                        int32_t target = atol(&cb[1]);
                        cli();

                        target_steps[i] = target << DOWNSAMPLE_BITS;
                        update_eeprom(i, target_steps[i]);

                        sei();
                        print_string("$\r\n");
                    }
                    else
                        print_string("?\r\n");
                }
            }
            else
                print_string("?\r\n");

            command_length = 0;
        }
        else
        {
            command_buffer[command_length] = (uint8_t)value;
            if (command_length < sizeof(command_buffer))
                command_length++;
        }
    }
}

volatile uint8_t *resolve_port(uint8_t port)
{
    if (port == 'B')
        return &PORTB;
    if (port == 'C')
        return &PORTC;
    if (port == 'D')
        return &PORTD;

    return &PORTB;
}

volatile uint8_t *resolve_ddr(uint8_t port)
{
    if (port == 'B')
        return &DDRB;
    if (port == 'C')
        return &DDRC;
    if (port == 'D')
        return &DDRD;

    return &DDRB;
}

int main(void)
{
    OCR1A = 4;
    TCCR1B = _BV(CS12) | _BV(CS10) | _BV(WGM12);
    TIMSK1 |= _BV(OCIE1A);

    for (uint8_t i = 0; i < CHANNEL_COUNT; i++)
    {
        channel *c = &channels[i];
        *resolve_ddr(c->enable_port) |= _BV(c->enable_pin);
        *resolve_port(c->enable_port) |= _BV(c->enable_pin);
        *resolve_ddr(c->step_port) |= _BV(c->step_pin);
        *resolve_port(c->step_port) &= ~_BV(c->step_pin);
        *resolve_ddr(c->dir_port) |= _BV(c->dir_pin);
        *resolve_port(c->dir_port) &= ~_BV(c->dir_pin);
        target_steps[i] = read_eeprom(i);
    }

    usb_initialize();

    sei();
    for (;;)
        loop();
}

ISR(TIMER1_COMPA_vect)
{    
#ifdef GLOBAL_ENABLE_PIN
    bool any_to_move;
    for (uint8_t i = 0; i < CHANNEL_COUNT; i++)
        if (current_steps[i] != target_steps[i])
            any_to_move = true;

    if (any_to_move && !enabled[0])
    {
        *resolve_port(c->enable_port) &= ~_BV(c->enable_pin);
        enabled[0] = true;

        // Skip a step when enabling a motor to avoid losing a count while it powers up
        return;
    }
    
    if (!any_to_move && enabled[0])
    {
        *resolve_port(c->enable_port) |= _BV(c->enable_pin);
        enabled[0] = false;
    }
#endif

    for (uint8_t i = 0; i < CHANNEL_COUNT; i++)
    {
        channel *c = &channels[i];
#ifndef GLOBAL_ENABLE_PIN
        if (!enabled[i] && current_steps[i] != target_steps[i])
        {
            enabled[i] = true;
            step_high[i] = false;
            *resolve_port(c->step_port) |= _BV(c->step_pin);
            *resolve_port(c->enable_port) &= ~_BV(c->enable_pin);

            // Skip a step when enabling a motor to avoid losing a count while it powers up
            continue;
        }
#endif
        else if (current_steps[i] < target_steps[i])
        {
            *resolve_port(c->dir_port) |= _BV(c->dir_pin);
            if (!step_high[i])
            {
                *resolve_port(c->step_port) |= _BV(c->step_pin);
                current_steps[i]++;
            }
            else
                *resolve_port(c->step_port) &= ~_BV(c->step_pin);

            step_high[i] ^= true;
        }
        else if (current_steps[i] > target_steps[i])
        {
            *resolve_port(c->dir_port) &= ~_BV(c->dir_pin);
            if (!step_high[i])
            {
                *resolve_port(c->step_port) |= _BV(c->step_pin);
                current_steps[i]--;
            }
            else
                *resolve_port(c->step_port) &= ~_BV(c->step_pin);

            step_high[i] ^= true;
        }
#ifndef GLOBAL_ENABLE_PIN
        else if (enabled[i])
        {
            enabled[i] = false;
            *resolve_port(c->enable_port) |= _BV(c->enable_pin);
        }
#endif
    }
}
