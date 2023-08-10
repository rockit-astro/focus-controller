//**********************************************************************************
//  Adapted from https://gist.github.com/stecman/9ec74de5e8a5c3c6341c791d9c233adc
//  which was released under the Creative Commons Zero licence.
//  Modifications copyright 2023 Paul Chote, All Rights Reserved
//**********************************************************************************

#include "gpio.h"

void gpio_configure_input_pullup(const gpin_t* pin) {
    *(pin->ddr) &= ~_BV(pin->bit);
    *(pin->port) |= _BV(pin->bit);
}

void gpio_configure_input_hiz(const gpin_t* pin) {
    *(pin->ddr) &= ~_BV(pin->bit);
    *(pin->port) &= ~_BV(pin->bit);
}

uint8_t gpio_input_read(const gpin_t* pin) {
    return *(pin->pin) & _BV(pin->bit);
}

void gpio_configure_output(const gpin_t* pin) {
    *(pin->ddr) |= _BV(pin->bit);
}

void gpio_output_set_high(const gpin_t* pin) {
    *(pin->port) |= _BV(pin->bit);
}

void gpio_output_set_low(const gpin_t* pin) {
    *(pin->port) &= ~_BV(pin->bit);
}
