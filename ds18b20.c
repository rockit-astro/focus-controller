//**********************************************************************************
//  Adapted from https://gist.github.com/stecman/9ec74de5e8a5c3c6341c791d9c233adc
//  which was released under the Creative Commons Zero licence.
//  Modifications copyright 2023 Paul Chote, All Rights Reserved
//**********************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <util/crc16.h>
#include <util/delay.h>
#include "ds18b20.h"

// Command bytes
static const uint8_t kConvertCommand = 0x44;
static const uint8_t kReadScatchPad = 0xBE;

// Scratch pad data indexes
static const uint8_t kScratchPad_tempLSB = 0;
static const uint8_t kScratchPad_tempMSB = 1;
static const uint8_t kScratchPad_crc = 8;

// Special return values
static const uint16_t kDS18B20_DeviceNotFound = 0xA800;
static const uint16_t kDS18B20_CrcCheckFailed = 0x5000;

/**
 * State for the onewire_search function
 * This must be initialised with onewire_search_init() before use.
 */
typedef struct onewire_search_state {

    // The highest bit position where a bit was ambiguous and a zero was written
    int8_t lastZeroBranch;

    // Internal flag to indicate if the search is complete
    // This flag is set once there are no more branches to search
    bool done;

    // Discovered 64-bit device address (LSB first)
    // After a successful search, this contains the found device address.
    // During a search this is overwritten LSB-first with a new address.
    uint8_t address[8];

} onewire_search_state;

static uint8_t crc8(uint8_t* data, uint8_t len)
{
    uint8_t crc = 0;

    for (uint8_t i = 0; i < len; ++i) {
        crc = _crc_ibutton_update(crc, data[i]);
    }

    return crc;
}

static bool onewire_reset(const gpin_t* io)
{
    // Configure for output
    gpio_output_set_high(io);
    gpio_configure_output(io);

    // Pull low for >480uS (master reset pulse)
    gpio_output_set_low(io);
    _delay_us(480);

    // Configure for input
    gpio_configure_input_hiz(io);
    _delay_us(70);

    // Look for the line pulled low by a slave
    uint8_t result = gpio_input_read(io);

    // Wait for the presence pulse to finish
    // This should be less than 240uS, but the master is expected to stay
    // in Rx mode for a minimum of 480uS in total
    _delay_us(460);

    return result == 0;
}

/**
 * Output a Write-0 or Write-1 slot on the One Wire bus
 * A Write-1 slot is generated unless the passed value is zero
 */
static void onewire_write_bit(const gpin_t* io, uint8_t bit)
{
    if (bit != 0) { // Write high

        // Pull low for less than 15uS to write a high
        gpio_output_set_low(io);
        _delay_us(5);
        gpio_output_set_high(io);

        // Wait for the rest of the minimum slot time
        _delay_us(55);

    } else { // Write low

        // Pull low for 60 - 120uS to write a low
        gpio_output_set_low(io);
        _delay_us(55);

        // Stop pulling down line
        gpio_output_set_high(io);

        // Recovery time between slots
        _delay_us(5);
    }
}

// One Wire timing is based on this Maxim application note
// https://www.maximintegrated.com/en/app-notes/index.mvp/id/126
static void onewire_write(const gpin_t* io, uint8_t byte)
{
    // Configure for output
    gpio_output_set_low(io);
    gpio_configure_output(io);

    for (uint8_t i = 8; i != 0; --i) {

        onewire_write_bit(io, byte & 0x1);

        // Next bit (LSB first)
        byte >>= 1;
    }
}

/**
 * Generate a read slot on the One Wire bus and return the bit value
 * Return 0x0 or 0x1
 */
static uint8_t onewire_read_bit(const gpin_t* io)
{
    // Pull the 1-wire bus low for >1uS to generate a read slot
    gpio_output_set_low(io);
    gpio_configure_output(io);
    _delay_us(1);

    // Configure for reading (releases the line)
    gpio_configure_input_hiz(io);

    // Wait for value to stabilise (bit must be read within 15uS of read slot)
    _delay_us(10);

    uint8_t result = gpio_input_read(io) != 0;

    // Wait for the end of the read slot
    _delay_us(50);

    return result;
}

static uint8_t onewire_read(const gpin_t* io)
{
    uint8_t buffer = 0x0;

    // Configure for input
    gpio_configure_input_hiz(io);

    // Read 8 bits (LSB first)
    for (uint8_t bit = 0x01; bit; bit <<= 1) {

        // Copy read bit to least significant bit of buffer
        if (onewire_read_bit(io)) {
            buffer |= bit;
        }
    }

    return buffer;
}

static void onewire_match_rom(const gpin_t* io, uint8_t* address)
{
    // Write Match Rom command on bus
    onewire_write(io, 0x55);

    // Send the passed address
    for (uint8_t i = 0; i < 8; ++i) {
        onewire_write(io, address[i]);
    }
}

static void onewire_skiprom(const gpin_t* io)
{
    onewire_write(io, 0xCC);
}

/**
 * Search procedure for the next ROM addresses
 *
 * This algorithm is bit difficult to understand from the diagrams in Maxim's
 * datasheets and app notes, though its reasonably straight forward once
 * understood.  I've used the name "last zero branch" instead of Maxim's name
 * "last discrepancy", since it describes how this variable is used.
 *
 * A device address has 64 bits. With multiple devices on the bus, some bits
 * are ambiguous.  Each time an ambiguous bit is encountered, a zero is written
 * and the position is marked.  In subsequent searches at ambiguous bits, a one
 * is written at this mark, zeros are written after the mark, and the bit in
 * the previous address is copied before the mark. This effectively steps
 * through all addresses present on the bus.
 *
 * For reference, see either of these documents:
 *
 *  - Maxim application note 187: 1-Wire Search Algorithm
 *    https://www.maximintegrated.com/en/app-notes/index.mvp/id/187
 *
 *  - Maxim application note 937: Book of iButton® Standards (pages 51-54)
 *    https://www.maximintegrated.com/en/app-notes/index.mvp/id/937
 *
 * @see onewire_search()
 * @returns true if a new address was found
 */
static bool _search_next(const gpin_t* io, onewire_search_state* state)
{
    // States of ROM search reads
    enum {
        kConflict = 0b00,
        kZero = 0b10,
        kOne = 0b01,
    };

    // Value to write to the current position
    uint8_t bitValue = 0;

    // Keep track of the last zero branch within this search
    // If this value is not updated, the search is complete
    int8_t localLastZeroBranch = -1;

    for (int8_t bitPosition = 0; bitPosition < 64; ++bitPosition) {

        // Calculate bitPosition as an index in the address array
        // This is written as-is for readability. Compilers should reduce this to bit shifts and tests
        uint8_t byteIndex = bitPosition / 8;
        uint8_t bitIndex = bitPosition % 8;

        // Configure bus pin for reading
        gpio_configure_input_hiz(io);

        // Read the current bit and its complement from the bus
        uint8_t reading = 0;
        reading |= onewire_read_bit(io); // Bit
        reading |= onewire_read_bit(io) << 1; // Complement of bit (negated)

        switch (reading) {
            case kZero:
            case kOne:
                // Bit was the same on all responding devices: it is a known value
                // The first bit is the value we want to write (rather than its complement)
                bitValue = (reading & 0x1);
                break;

            case kConflict:
                // Both 0 and 1 were written to the bus
                // Use the search state to continue walking through devices
                if (bitPosition == state->lastZeroBranch) {
                    // Current bit is the last position the previous search chose a zero: send one
                    bitValue = 1;

                } else if (bitPosition < state->lastZeroBranch) {
                    // Before the lastZeroBranch position, repeat the same choices as the previous search
                    bitValue = state->address[byteIndex] & (1 << bitIndex);

                } else {
                    // Current bit is past the lastZeroBranch in the previous search: send zero
                    bitValue = 0;
                }

                // Remember the last branch where a zero was written for the next search
                if (bitValue == 0) {
                    localLastZeroBranch = bitPosition;
                }

                break;

            default:
                // If we see "11" there was a problem on the bus (no devices pulled it low)
                return false;
        }

        // Write bit into address
        if (bitValue == 0) {
            state->address[byteIndex] &= ~(1 << bitIndex);
        } else {
            state->address[byteIndex] |= (bitValue << bitIndex);
        }

        // Configure for output
        gpio_output_set_high(io);
        gpio_configure_output(io);

        // Write bit to the bus to continue the search
        onewire_write_bit(io, bitValue);
    }

    // If the no branch points were found, mark the search as done.
    // Otherwise, mark the last zero branch we found for the next search
    if (localLastZeroBranch == -1) {
        state->done = true;
    } else {
        state->lastZeroBranch = localLastZeroBranch;
    }

    // Read a whole address - return success
    return true;
}

static inline bool _search_devices(uint8_t command, const gpin_t* io, onewire_search_state* state)
{
    // Bail out if the previous search was the end
    if (state->done) {
        return false;
    }

    if (!onewire_reset(io)) {
        // No devices present on the bus
        return false;
    }

    onewire_write(io, command);
    return _search_next(io, state);
}

static bool onewire_search(const gpin_t* io, onewire_search_state* state)
{
    // Search with "Search ROM" command
    return _search_devices(0xF0, io, state);
}

static bool onewire_check_rom_crc(onewire_search_state* state)
{
    // Validate bits 0..56 (bytes 0 - 6) against the CRC in byte 7 (bits 57..63)
    return state->address[7] == crc8(state->address, 7);
}

void ds18b20_search(const gpin_t* io, uint8_t *found, uint8_t *buf, uint16_t len)
{
    onewire_search_state state;
    state.lastZeroBranch = -1;
    state.done = false;
    memset(state.address, 0, sizeof(state.address));

    uint8_t i = 0;
    while (onewire_search(io, &state) && 8 * (i + 1) <= len)
        if (onewire_check_rom_crc(&state))
            memcpy(&buf[8 * i++], &state.address, 8);

    *found = i;
}

static uint16_t ds18b20_readScratchPad(const gpin_t* io)
{
    // Read scratchpad into buffer (LSB byte first)
    static const int8_t kScratchPadLength = 9;
    uint8_t buffer[kScratchPadLength];

    for (int8_t i = 0; i < kScratchPadLength; ++i) {
        buffer[i] = onewire_read(io);
    }

    // Check the CRC (9th byte) against the 8 bytes of data
    if (crc8(buffer, 8) != buffer[kScratchPad_crc]) {
        return kDS18B20_CrcCheckFailed;
    }

    // Return the raw 9 to 12-bit temperature value
    return (buffer[kScratchPad_tempMSB] << 8) | buffer[kScratchPad_tempLSB];
}

static uint16_t ds18b20_read_slave(const gpin_t* io, uint8_t* address)
{
    // Confirm the device is still alive. Abort if no reply
    if (!onewire_reset(io)) {
        return kDS18B20_DeviceNotFound;
    }

    onewire_match_rom(io, address);
    onewire_write(io, kReadScatchPad);

    // Read the data from the scratch pad
    return ds18b20_readScratchPad(io);
}

void ds18b20_convert(const gpin_t* io)
{
    // Send convert command to all devices (this has no response)
    onewire_skiprom(io);
    onewire_write(io, kConvertCommand);
}

bool ds18b20_measure(const gpin_t* io, uint8_t address[8], char output[10])
{
    onewire_reset(io);

    ds18b20_convert(io);
    _delay_ms(750);

    uint16_t reading = ds18b20_read_slave(io, address);
    if (reading == kDS18B20_CrcCheckFailed)
        return false;

    if (reading == kDS18B20_DeviceNotFound)
        return false;

    memset(output, '\0', 10);

    const uint16_t integer = (reading >> 4);
    const uint16_t frac = (reading & 0x0F) * 625;

    itoa(integer, output, 10);
    output += strlen(output);
    (*output++) = '.';

    if (frac == 0) {
        memset(output, '0', 4);
        return true;
    }

    if (frac < 1000)
        (*output++) = '0';

    itoa(frac, output, 10);
    return true;
}
