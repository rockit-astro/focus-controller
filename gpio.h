//**********************************************************************************
//  Adapted from https://gist.github.com/stecman/9ec74de5e8a5c3c6341c791d9c233adc
//  which was released under the Creative Commons Zero licence.
//  Modifications copyright 2023 Paul Chote, All Rights Reserved
//**********************************************************************************

#include <avr/io.h>
#include <stdint.h>

#ifndef FOCUSER_GPIO_H
#define FOCUSER_GPIO_H

typedef struct gpin_t {
    // Pointers to PORT and PIN and DDR registers
    volatile uint8_t *port;
    volatile uint8_t *pin;
    volatile uint8_t *ddr;

    // Bit number in PORT
    uint8_t bit;
} gpin_t;

void gpio_configure_input_pullup(const gpin_t* pin);
void gpio_configure_input_hiz(const gpin_t* pin);
uint8_t gpio_input_read(const gpin_t* pin);

void gpio_configure_output(const gpin_t* pin);
void gpio_output_set_high(const gpin_t* pin);
void gpio_output_set_low(const gpin_t* pin);

#endif