<!---
Datasheet text for the Tiny Tapeout project page.
-->

## How it works

The chip has two halves that share the clock, the reset and the pins.

**Peripheral hub.** A host sends command bytes over UART (115200 baud at the 25 MHz clock). A dispatcher decodes
them and runs an SPI master (modes 0-3, up to 16 bytes with CS held low) or an I2C master (write, read, or
write-then-read with a repeated START, up to 16 bytes). Results go back over UART: one byte per SPI byte, and for I2C
each read byte followed by a status byte (0x00 ok, 0xFF NACK).

Command byte: `[7:6]` engine (00 SPI, 01 I2C).
- SPI: `[5]` CPOL, `[4]` CPHA, `[3:0]` length-1, then `length` data bytes.
- I2C write (`[5]`=0): `[3:0]` length-1, then device address (7-bit), then `length` data bytes.
- I2C read (`[5]`=1): `[3:0]` read count-1, then device address, write count (0-15), then the write bytes.

**eFPGA.** 16 registered LUT4 cells, three hard blocks (8-bit shift register, 8-bit programmable divider, 8-bit CRC
with programmable polynomial) and windowed routing muxes. A 507-bit bitstream is shifted in on `CFG_DATA` / `CFG_CLK`.
The fabric sees `ui_in[7:0]` and four hub events (UART byte received, SPI done, I2C done, UART byte transmit start)
and drives `uo_out[7:1]`. `CFG_OUT` returns the chain tail so a written bitstream can be read back.

## How to test

1. Reset the chip (`rst_n` low for a few clocks).
2. Hub: send `0x00 0x11` over `ui[0]` (SPI, 1 byte) and watch `uio[3..5]` and `uo[0]`.
3. Fabric: build a bitstream with `test/cfgimage.py`, put each bit on `CFG_DATA`, pulse `CFG_CLK` (keep it slower than
   a quarter of the system clock), then pulse `rst_n` once so the cells start from zero.
4. Read the bitstream back by shifting a second one in while sampling `CFG_OUT` before each `CFG_CLK` rising edge.

## External hardware

UART adapter (3.3 V logic) on `ui[0]` / `uo[0]`; any SPI device on `uio[3..5]` + `ui[1]`; any I2C device on
`uio[6]` / `uio[7]` with 4.7 kOhm pull-ups to 3.3 V; a way to clock the config pins (an RP2040 on the Tiny Tapeout
demo board works).
