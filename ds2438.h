//**********************************************************************************
//  Copyright 2023 Paul Chote, All Rights Reserved
//**********************************************************************************

#include <stdint.h>
#include "gpio.h"

#ifndef FOCUSER_DS18B20_H
#define FOCUSER_DS18B20_H

void ds2438_search(const gpin_t* io, uint8_t *found, uint8_t *buf, uint16_t len);
bool ds2438_measure(const gpin_t* io, char output[20]);

#endif
