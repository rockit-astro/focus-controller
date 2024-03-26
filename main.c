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
#include "ds18b20.h"
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
} focuser;

focuser focusers[] = {
    {
        .enable = { &PORTD, &PIND, &DDRD, PD1 },
        .step = { &PORTB, &PINB, &DDRB, PB2 },
        .dir = { &PORTB, &PINB, &DDRB, PB1 }
    },
#if FOCUSERS == 2
    {
        .enable = { &PORTB, &PINB, &DDRB, PB6 },
        .step = { &PORTD, &PIND, &DDRD, PD4 },
        .dir = { &PORTD, &PIND, &DDRD, PD0 }
    }
#elif FOCUSERS > 2
    #error Only 1 or 2 focusers are supported
#endif
};

#define FOCUSER_COUNT (sizeof(focusers)/sizeof(*(focusers)))

gpin_t usb_conn_led = { &PORTC, &PINC, &DDRC, PD7 };
gpin_t usb_rx_led = { &PORTB, &PINB, &DDRB, PB0 };
gpin_t usb_tx_led = { &PORTD, &PIND, &DDRD, PD5 };

gpin_t fans = { &PORTB, &PINB, &DDRB, PB5 };
gpin_t onewire_bus = { &PORTF, &PINF, &DDRF, PF1 };

// The raw motor resolution is too fine to be useful
// Work internally at 64x resolution, which allows 7 digits of external resolution.

#define DOWNSAMPLE_BITS 4

volatile bool led_active;
char output[256];

uint8_t command_length = 0;
char command_buffer[20];

int32_t focuser_target_steps[FOCUSER_COUNT] = {};
int32_t focuser_current_steps[FOCUSER_COUNT] = {};
bool focuser_enabled[FOCUSER_COUNT] = {};
bool focuser_step_high[FOCUSER_COUNT] = {};

bool fans_enabled = false;

static void update_focuser_eeprom(uint8_t i, int32_t target)
{
    // Save the current absolute position so we can recover
    // the absolute position after a power cycle.
    // TODO: Implement a wear levelling strategy
    eeprom_update_dword((uint32_t*)(4 * i), target);
}

static int32_t read_focuser_eeprom(uint8_t i)
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
            // Report stepper motor status
            char *cb = command_buffer;
            if (command_length == 1 && cb[0] == '?')
            {
                for (uint8_t i = 0; i < FOCUSER_COUNT; i++)
                {
                    sprintf(output + i * 22, "T%01d=%+07ld,C%01d=%+07ld,",
                        i + 1,
                        focuser_target_steps[i] >> DOWNSAMPLE_BITS,
                        i + 1,
                        focuser_current_steps[i] >> DOWNSAMPLE_BITS);
                }
                
                sprintf(output + FOCUSER_COUNT * 22 - 1, "\r\n");
                
                print_string(output);
            }
            else if (command_length == 1 && cb[0] == '#')
            {
                // Report fan status
                sprintf(output, "%d\r\n", fans_enabled);
                print_string(output);
            }
            else if (command_length == 2 && cb[0] == '#' && (cb[1] == '0' || cb[1] == '1'))
            {
                // Switch fans on or off
                fans_enabled = cb[1] == '1';

                if (fans_enabled)
                    gpio_output_set_high(&fans);
                else
                    gpio_output_set_low(&fans);

                print_string("$\r\n");
            }
            else if (command_length == 1 && cb[0] == '@')
            {
                // Allocate space to find up to 4 sensors
                uint8_t addresses[4*8];
                uint8_t found;
                ds18b20_search(&onewire_bus, &found, addresses, sizeof(addresses));

                for (uint8_t i = 0; i < found; i++)
                {
                    for (uint8_t j = 0; j < 8; j++)
                        sprintf(output + i * 17 + 2 * j, "%02X", addresses[i * 8 + j]);
                    output[i * 17 + 16] = ',';
                }

                sprintf(output + found * 17 - 1, "\r\n");
                print_string(output);
            }
            else if (command_length == 17 && cb[0] == '@')
            {
                uint8_t address[8];
                bool failed = false;
                for (uint8_t i = 0; i < 8; i++)
                {
                    int temp;
                    if (!sscanf(command_buffer + 2 * i + 1, "%02X", &temp))
                    {
                        failed = true;
                        break;
                    }

                    address[i] = (uint8_t)temp;
                }

                if (!failed)
                {
                    char temp[10];
                    if (ds18b20_measure(&onewire_bus, address, temp))
                    {
                        sprintf(output, "%s\r\n", temp);
                        print_string(output);
                    }
                    else
                        print_string("FAILED\r\n");
                }
                else
                    print_string("?\r\n");
            }
            else if (cb[0] > '0' && cb[0] <= '0' + FOCUSER_COUNT)
            {
                // 0-indexed focuser number
                uint8_t i = cb[0] - '1';

                // Stop at current position: [1..9]S\r\n
                if (command_length == 2 && cb[1] == 'S')
                {
                    cli();

                    focuser_target_steps[i] = focuser_current_steps[i];
                    update_focuser_eeprom(i, focuser_target_steps[i]);

                    sei();
                    print_string("$\r\n");
                }
                // Zero at current position: [1..9]Z\r\n
                else if (command_length == 2 && cb[1] == 'Z')
                {
                    cli();

                    focuser_target_steps[i] = focuser_current_steps[i] = 0;
                    update_focuser_eeprom(i, 0);

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

                        focuser_target_steps[i] = target << DOWNSAMPLE_BITS;
                        update_focuser_eeprom(i, focuser_target_steps[i]);

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

    for (uint8_t i = 0; i < FOCUSER_COUNT; i++)
    {
        focuser *f = &focusers[i];
        gpio_output_set_high(&f->enable);
        gpio_configure_output(&f->enable);

        gpio_output_set_low(&f->step);
        gpio_configure_output(&f->step);

        gpio_output_set_low(&f->dir);
        gpio_configure_output(&f->dir);

        focuser_target_steps[i] = focuser_current_steps[i] = read_focuser_eeprom(i);
    }

    gpio_output_set_low(&fans);
    gpio_configure_output(&fans);

    usb_initialize(&usb_conn_led, &usb_rx_led, &usb_tx_led);

    sei();
    for (;;)
        loop();
}

ISR(TIMER1_COMPA_vect)
{
    for (uint8_t i = 0; i < FOCUSER_COUNT; i++)
    {
        focuser *f = &focusers[i];
        if (!focuser_enabled[i] && focuser_current_steps[i] != focuser_target_steps[i])
        {
            focuser_enabled[i] = true;
            focuser_step_high[i] = true;
            gpio_output_set_high(&f->step);
            gpio_output_set_low(&f->enable);

            // Skip a step when enabling a motor to avoid losing a count while it powers up
            continue;
        }
        else if (focuser_current_steps[i] < focuser_target_steps[i])
        {
            gpio_output_set_high(&f->dir);
            if (!focuser_step_high[i])
            {
                gpio_output_set_high(&f->step);
                focuser_current_steps[i]++;
            }
            else
                gpio_output_set_low(&f->step);

            focuser_step_high[i] ^= true;
        }
        else if (focuser_current_steps[i] > focuser_target_steps[i])
        {
            gpio_output_set_low(&f->dir);
            if (!focuser_step_high[i])
            {
                gpio_output_set_high(&f->step);
                focuser_current_steps[i]--;
            }
            else
                gpio_output_set_low(&f->step);

            focuser_step_high[i] ^= true;
        }
        else if (focuser_enabled[i])
        {
            focuser_enabled[i] = false;
            gpio_output_set_high(&f->enable);
        }
    }
}
