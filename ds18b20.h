//**********************************************************************************
//  Copyright 2023 Paul Chote, All Rights Reserved
//**********************************************************************************

#include <stdint.h>
#include "gpio.h"

#ifndef FOCUSER_DS18B20_H
#define FOCUSER_DS18B20_H

void ds18b20_search(const gpin_t* io, uint8_t *found, uint8_t *buf, uint16_t len);
bool ds18b20_measure(const gpin_t* io, uint8_t address[8], char output[10]);

#endif
