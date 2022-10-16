## Multi-channel Focus Controller

Firmware for Arduino-based stepper motor focus controller for the twin RASA telescopes on CLASP.

The hardware supports up to 4 parallel focus channels, but the current firmware only implements two.

TODO: The focuser maintains its own absolute scale across power cycles, so positions should be repeatable provided the focus is not moved manually.

### Hardware:

 * Arduino Leonardo
 * GRBL 0.9 CNC Shield
 * Pancake stepper motors

### Protocol Commands:

| Command             | Use                                  |
|---------------------|--------------------------------------|
| `?\n`               | Query current status                 |
| `[12]S\n`           | Stop channel 1/2 at current position |
| `[12]Z\n`           | Zero channel 1/2 at current position |
| `[12][+-]1234567\n` | Set channel 1/2 target position      |

Note: Positions are limited to 7 digits.

### Protocol Responses:

| Response                                              | Meaning                           |
|-------------------------------------------------------|-----------------------------------|
| `?\r\n`                                               | Unknown command                   |
| `$\r\n`                                               | Command acknowledged (except `?`) |
| `T1=+0000000,C1=+0000000,T2=+0000000,C2=+0000000\r\n` | Current status (response to `?`)  |
