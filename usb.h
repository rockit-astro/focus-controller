//**********************************************************************************
//  Copyright 2016, 2017, 2022 Paul Chote, All Rights Reserved
//**********************************************************************************

#include <stdarg.h>
#include <stdint.h>

#ifndef FOCUSER_USB_H
#define FOCUSER_USB_H

void usb_initialize(void);
bool usb_can_read(void);
int16_t usb_read(void);
void usb_write(uint8_t b);
void usb_write_data(void *buf, uint16_t len);
#endif
