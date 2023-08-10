//**********************************************************************************
//  Copyright 2016, 2017, 2022, 2023 Paul Chote, All Rights Reserved
//**********************************************************************************

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdint.h>
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Common/Common.h>
#include "usb_descriptors.h"
#include "gpio.h"

USB_ClassInfo_CDC_Device_t interface =
{
    .Config =
    {
        .ControlInterfaceNumber = INTERFACE_ID_CDC_CCI,
        .DataINEndpoint         =
        {
            .Address            = CDC_TX_EPADDR,
            .Size               = CDC_TXRX_EPSIZE,
            .Banks              = 1,
        },
        .DataOUTEndpoint        =
        {
            .Address            = CDC_RX_EPADDR,
            .Size               = CDC_TXRX_EPSIZE,
            .Banks              = 1,
        },
        .NotificationEndpoint   =
        {
            .Address            = CDC_NOTIFICATION_EPADDR,
            .Size               = CDC_NOTIFICATION_EPSIZE,
            .Banks              = 1,
        },
    },
};

// Counters (in milliseconds) for blinking the TX/RX LEDs
#define TX_RX_LED_PULSE_MS 100

volatile uint8_t tx_led_pulse;
volatile uint8_t rx_led_pulse;
static gpin_t *conn_led;
static gpin_t *rx_led;
static gpin_t *tx_led;

void usb_initialize(gpin_t *usb_conn_led, gpin_t *usb_rx_led, gpin_t *usb_tx_led)
{
    conn_led = usb_conn_led;
    rx_led = usb_rx_led;
    tx_led = usb_tx_led;
    
    gpio_configure_output(conn_led);
    gpio_output_set_low(conn_led);

    gpio_configure_output(rx_led);
    gpio_output_set_low(rx_led);

    gpio_configure_output(tx_led);
    gpio_output_set_low(tx_led);

    USB_Init();
}

bool usb_can_read(void)
{
    return CDC_Device_BytesReceived(&interface) > 0;
}

// Read a byte from the receive buffer
// Will return negative if unable to read
int16_t usb_read(void)
{
    int16_t ret = CDC_Device_ReceiveByte(&interface);

    // Flash the RX LED
    if (ret >= 0)
    {
        gpio_output_set_high(rx_led);
        rx_led_pulse = TX_RX_LED_PULSE_MS;
        USB_Device_EnableSOFEvents();
    }

    return ret;
}

// Add a byte to the send buffer.
// Will block if the buffer is full
void usb_write(uint8_t b)
{
    // Note: This is ignoring any errors (e.g. send failed)
    // We are only sending single byte packets, so there's no
    // real benefits to handling them properly
    if (CDC_Device_SendByte(&interface, b) != ENDPOINT_READYWAIT_NoError)
        return;

    if (CDC_Device_Flush(&interface) != ENDPOINT_READYWAIT_NoError)
        return;

    // Flash the TX LED
    gpio_output_set_high(tx_led);
    tx_led_pulse = TX_RX_LED_PULSE_MS;
    USB_Device_EnableSOFEvents();
}

void usb_write_data(void *buf, uint16_t len)
{
    // Note: This is ignoring any errors (e.g. send failed)
    // We are only sending single byte packets, so there's no
    // real benefits to handling them properly
    if (CDC_Device_SendData(&interface, buf, len) != ENDPOINT_READYWAIT_NoError)
        return;

    if (CDC_Device_Flush(&interface) != ENDPOINT_READYWAIT_NoError)
        return;

    // Flash the TX LED
    gpio_output_set_high(tx_led);
    tx_led_pulse = TX_RX_LED_PULSE_MS;
    USB_Device_EnableSOFEvents();
}

void EVENT_USB_Device_ConfigurationChanged(void)
{
    CDC_Device_ConfigureEndpoints(&interface);
}

void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
    if (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR)
        gpio_output_set_high(conn_led);
    else
        gpio_output_set_low(conn_led);
}

void EVENT_USB_Device_Connect(void)
{

}

void EVENT_USB_Device_Disconnect(void)
{
    // The SOF event will not fire while the device is disconnected
    // so make sure that the TX/RX LEDs are turned off now
    gpio_output_set_low(tx_led);
    gpio_output_set_low(rx_led);
}

void EVENT_USB_Device_ControlRequest(void)
{
    CDC_Device_ProcessControlRequest(&interface);
}

void EVENT_USB_Device_StartOfFrame(void)
{
    // SOF event runs once per millisecond when enabled
    // Use this to count down and turn off the RX/TX LEDs.
    if (tx_led_pulse && !(--tx_led_pulse))
        gpio_output_set_low(tx_led);
    if (rx_led_pulse && !(--rx_led_pulse))
        gpio_output_set_low(rx_led);

    // Disable SOF event while both LEDs are disabled
    if (!tx_led_pulse && !rx_led_pulse)
        USB_Device_DisableSOFEvents();
}
