//**********************************************************************************
//  Copyright 2023 Paul Chote, All Rights Reserved
//**********************************************************************************

#include <stdint.h>
#include "gpio.h"

#ifndef FOCUSER_DS18B20_H
#define FOCUSER_DS18B20_H

void onewire_measure(const gpin_t* io, char output[20]);

#endif
