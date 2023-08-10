//**********************************************************************************
//  Copyright 2016, 2017, 2022, 2023 Paul Chote, All Rights Reserved
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
#include "gpio.h"
#include "usb.h"

#define F_CPU 16000000UL
#include <util/delay.h>

#define length(array) (sizeof(array)/sizeof(*(array)))

typedef struct
{
    gpin_t enable;
    gpin_t step;
    gpin_t dir;
} channel;

channel channels[] = {
#if MODEL == 1
    {
        .enable = { &PORTB, &PINB, &DDRB, PB1 },
        .step = { &PORTB, &PINB, &DDRB, PB0 },
        .dir = { &PORTB, &PINB, &DDRB, PB2 }
    }
#elif MODEL == 0
    {
        .enable = { &PORTB, &PINB, &DDRB, PB4 },
        .step = { &PORTD, &PIND, &DDRD, PD1 },
        .dir = { &PORTC, &PINC, &DDRC, PC6 }
    },
    {
        .enable = { &PORTB, &PINB, &DDRB, PB4 },
        .step = { &PORTD, &PIND, &DDRD, PD0 },
        .dir = { &PORTD, &PIND, &DDRD, PD7 }
    }
#endif
};

gpin_t usb_conn_led = { &PORTC, &PINC, &DDRC, PD7 };
gpin_t usb_rx_led = { &PORTB, &PINB, &DDRB, PB0 };
gpin_t usb_tx_led = { &PORTD, &PIND, &DDRD, PD5 };

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

int main(void)
{
    OCR1A = 4;
    TCCR1B = _BV(CS12) | _BV(CS10) | _BV(WGM12);
    TIMSK1 |= _BV(OCIE1A);

    for (uint8_t i = 0; i < CHANNEL_COUNT; i++)
    {
        channel *c = &channels[i];
        gpio_configure_output(&c->enable);
        gpio_output_set_high(&c->enable);

        gpio_configure_output(&c->step);
        gpio_output_set_low(&c->step);

        gpio_configure_output(&c->dir);
        gpio_output_set_low(&c->dir);

        target_steps[i] = current_steps[i] = read_eeprom(i);
    }

    usb_initialize(&usb_conn_led, &usb_rx_led, &usb_tx_led);

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
        gpio_output_set_low(&c->enable);
        enabled[0] = true;

        // Skip a step when enabling a motor to avoid losing a count while it powers up
        return;
    }
    
    if (!any_to_move && enabled[0])
    {
        gpio_output_set_high(&c->enable);
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
            step_high[i] = true;
            gpio_output_set_high(&c->step);
            gpio_output_set_low(&c->enable);

            // Skip a step when enabling a motor to avoid losing a count while it powers up
            continue;
        }
        else 
#endif
        if (current_steps[i] < target_steps[i])
        {
            gpio_output_set_high(&c->dir);
            if (!step_high[i])
            {
                gpio_output_set_high(&c->step);
                current_steps[i]++;
            }
            else
                gpio_output_set_low(&c->step);

            step_high[i] ^= true;
        }
        else if (current_steps[i] > target_steps[i])
        {
            gpio_output_set_low(&c->dir);
            if (!step_high[i])
            {
                gpio_output_set_high(&c->step);
                current_steps[i]--;
            }
            else
                gpio_output_set_low(&c->step);

            step_high[i] ^= true;
        }
#ifndef GLOBAL_ENABLE_PIN
        else if (enabled[i])
        {
            enabled[i] = false;
            gpio_output_set_high(&c->enable);
        }
#endif
    }
}
