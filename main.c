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

#define ENABLE_LOW   PORTB &= ~_BV(PB4)
#define ENABLE_HIGH  PORTB |= _BV(PB4)
#define ENABLE_INIT  DDRB |= _BV(DDB4), ENABLE_HIGH

#define CH1_STEP_LOW   PORTD &= ~_BV(PD1)
#define CH1_STEP_HIGH  PORTD |= _BV(PD1)
#define CH1_STEP_INIT  DDRD |= _BV(DDD1), CH1_STEP_LOW

#define CH1_DIR_LOW   PORTC &= ~_BV(PC6)
#define CH1_DIR_HIGH  PORTC |= _BV(PC6)
#define CH1_DIR_INIT  DDRC |= _BV(DDC6), CH1_DIR_LOW

#define CH2_STEP_LOW   PORTD &= ~_BV(PD0)
#define CH2_STEP_HIGH  PORTD |= _BV(PD0)
#define CH2_STEP_INIT  DDRD |= _BV(DDD0), CH2_STEP_LOW

#define CH2_DIR_LOW   PORTD &= ~_BV(PD7)
#define CH2_DIR_HIGH  PORTD |= _BV(PD7)
#define CH2_DIR_INIT  DDRD |= _BV(DDD7), CH2_DIR_LOW

#define DOWNSAMPLE_BITS 4

volatile bool enabled;
volatile bool step_high;
volatile bool led_active;
char output[128];

uint8_t command_length = 0;
char command_buffer[16];

int32_t ch1_target_steps = 0;
int32_t ch1_current_steps = 0;
int32_t ch2_target_steps = 0;
int32_t ch2_current_steps = 0;

static void update_eeprom(uint8_t channel, int32_t target)
{
    // Save the current absolute position so we can recover
    // the absolute position after a power cycle.
    // TODO: Implement a wear levelling strategy
    eeprom_update_dword((uint32_t*)(channel == 1 ? 0 : 4), target);
}

static int32_t read_eeprom(uint8_t channel)
{
    return eeprom_read_dword((uint32_t*)(channel == 1 ? 0 : 4));
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
                snprintf(output, 128, "T1=%+07ld,C1=%+07ld,T2=%+07ld,C2=%+07ld\r\n",
                    ch1_target_steps >> DOWNSAMPLE_BITS,
                    ch1_current_steps >> DOWNSAMPLE_BITS,
                    ch2_target_steps >> DOWNSAMPLE_BITS,
                    ch2_current_steps >> DOWNSAMPLE_BITS);
                print_string(output);
            }
            else if (cb[0] == '1' || cb[0] == '2')
            {
                // Stop at current position: [12]S\r\n
                if (command_length == 2 && cb[1] == 'S')
                {
                    cli();
                    if (cb[0] == '1')
                    {
                        ch1_target_steps = ch1_current_steps;
                        update_eeprom(1, ch1_target_steps);
                    }
                    else
                    {
                        ch2_target_steps = ch2_current_steps;
                        update_eeprom(2, ch2_target_steps);
                    }
                    sei();
                    print_string("$\r\n");
                }
                // Zero at current position: [12]Z\r\n
                else if (command_length == 2 && cb[1] == 'Z')
                {
                    cli();
                    if (cb[0] == '1')
                    {
                        ch1_target_steps = ch1_current_steps = 0;
                        update_eeprom(1, ch1_target_steps);
                    }
                    else
                    {
                        ch2_target_steps = ch2_current_steps = 0;
                        update_eeprom(2, ch2_target_steps);
                    }
                    sei();
                    print_string("$\r\n");
                }
                // Move to position: [12][+-]1234567\r\n
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
                        // The raw motor resolution is too fine to be useful
                        // Work internally at 64x resolution, which allows 7 digits of external resolution.
                        if (cb[0] == '1')
                        {
                            ch1_target_steps = target << DOWNSAMPLE_BITS;
                            update_eeprom(1, ch1_target_steps);
                        }
                        else
                        {
                            ch2_target_steps = target << DOWNSAMPLE_BITS;
                            update_eeprom(2, ch2_target_steps);
                        }
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
    OCR1A = 1;
    TCCR1B = _BV(CS12) | _BV(CS10) | _BV(WGM12);
    TIMSK1 |= _BV(OCIE1A);

    ch1_target_steps = ch1_current_steps = read_eeprom(1);
    ch2_target_steps = ch2_current_steps = read_eeprom(2);

    ENABLE_INIT;
    CH1_STEP_INIT;
    CH1_DIR_INIT;
    CH2_STEP_INIT;
    CH2_DIR_INIT;
    
    CH2_DIR_HIGH;

    usb_initialize();

    sei();
    for (;;)
        loop();
}

ISR(TIMER1_COMPA_vect)
{
    if (!enabled && (ch1_current_steps != ch1_target_steps || ch2_current_steps != ch2_target_steps))
    {
        enabled = true;
        ENABLE_LOW;
        return;
    }
    else if (enabled && ch1_current_steps == ch1_target_steps && ch2_current_steps == ch2_target_steps)
    {
        enabled = false;
        ENABLE_HIGH;
    }

    if (ch1_current_steps < ch1_target_steps)
    {
        CH1_DIR_HIGH;
        if (!step_high)
        {
            CH1_STEP_HIGH;
            ch1_current_steps++;
        }
        else
            CH1_STEP_LOW;
    }
    else if (ch1_current_steps > ch1_target_steps)
    {
        CH1_DIR_LOW;
        if (!step_high)
        {
            CH1_STEP_HIGH;
            ch1_current_steps--;
        }
        else
            CH1_STEP_LOW;
    }

    if (ch2_current_steps < ch2_target_steps)
    {
        CH2_DIR_HIGH;
        if (!step_high)
        {
            CH2_STEP_HIGH;
            ch2_current_steps++;
        }
        else
            CH2_STEP_LOW;
    }
    else if (ch2_current_steps > ch2_target_steps)
    {
        CH2_DIR_LOW;
        if (!step_high)
        {
            CH2_STEP_HIGH;
            ch2_current_steps--;
        }
        else
            CH2_STEP_LOW;
    }

    step_high ^= true;
}
