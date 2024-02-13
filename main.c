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
#include "ds2438.h"
#include "gpio.h"
#include "usb.h"

#define F_CPU 16000000UL
#include <util/delay.h>

#define length(array) (sizeof(array)/sizeof(*(array)))

gpin_t channels[] = {
   { &PORTF, &PINF, &DDRF, PF7 },
   { &PORTF, &PINF, &DDRF, PF6 },
   { &PORTF, &PINF, &DDRF, PF5 },
   { &PORTF, &PINF, &DDRF, PF4 }
};

#define CHANNEL_COUNT (sizeof(channels)/sizeof(*(channels)))

gpin_t usb_conn_led = { &PORTC, &PINC, &DDRC, PD7 };
gpin_t usb_rx_led = { &PORTB, &PINB, &DDRB, PB0 };
gpin_t usb_tx_led = { &PORTD, &PIND, &DDRD, PD5 };
volatile bool measure = false;


// The raw motor resolution is too fine to be useful
// Work internally at 64x resolution, which allows 7 digits of external resolution.

volatile bool led_active;
char output[256];

uint8_t command_length = 0;
char command_buffer[20];

static void print_string(char *message)
{
    usb_write_data(message, strlen(message));
}

static void loop(void)
{
    if (measure)
    {
        measure = false;
        for (uint8_t i = 0; i < CHANNEL_COUNT; i++)
        {
            sprintf(output, "%d:", i);
            if (!ds2438_measure(&channels[i], output + 2))
                strcat(output, "FAIL\r\n");
            print_string(output);
        }
    }

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
    // Configure timer1 to interrupt every second
    OCR1A = 2*7812;
    TCCR1B = _BV(CS12) | _BV(CS10) | _BV(WGM12);
    TIMSK1 |= _BV(OCIE1A);

    usb_initialize(&usb_conn_led, &usb_rx_led, &usb_tx_led);

    sei();
    for (;;)
        loop();
}


ISR(TIMER1_COMPA_vect)
{
    measure = true;
}