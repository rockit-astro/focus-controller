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
#include "onewire.h"
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

volatile bool led_active;
char output[256];

static void loop(void)
{
    if (measure)
    {
        measure = false;
        for (uint8_t i = 0; i < CHANNEL_COUNT; i++)
        {
            sprintf(output, "%d:\r\n", i);
            onewire_measure(&channels[i], output + 2);
            usb_write_data(output, strlen(output));
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