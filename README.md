# tt_efpga_hub: eFPGA + UART/SPI/I2C hub for Tiny Tapeout (SKY 26d, 2x2)

Top module: `tt_um_ayush130305_efpga_hub`. Clock 25 MHz. Target: ttsky26d, sky130, 2x2 tiles (about 335 x 226 um).

```
 UART rx --> dispatcher --> SPI master --> SPI pins
                       \--> I2C master --> I2C pins
 UART tx <--/
                  hub events (rx_valid, spi_done, i2c_done, tx_start)
                                   |
 ui_in[7:0] ---------------------> eFPGA (16 LUT4+FF cells, shift8 / div8 / crc8) --> uo_out[7:1]
                                   ^
              cfg_data / cfg_clk --+--> cfg_out (read-back)
```

## Pins

| Pin | Function | Pin | Function |
|---|---|---|---|
| ui[0] | UART RX (also fabric input) | uo[0] | UART TX |
| ui[1] | SPI MISO (also fabric input) | uo[7:1] | fabric outputs, or shift-register bits 6..0 when `uo_mode`=1 |
| ui[7:2] | fabric inputs | uio[0] | CFG_DATA in |
| uio[1] | CFG_CLK in | uio[2] | CFG_OUT out |
| uio[3] | SPI MOSI | uio[4] | SPI SCLK |
| uio[5] | SPI CS_N | uio[6] | I2C SCL (open drain, drives 0 only) |
| uio[7] | I2C SDA (open drain, bidirectional) | | |

I2C needs external pull-ups (4.7 kOhm to 3.3 V). Keep `ui[0]` high when idle (UART idle level), or the hub will see a start bit.

## Hub protocol (your original v1 bridge, unchanged)

Command byte `[7:6]` selects the engine.
- **SPI** (`00`): `[5]` CPOL, `[4]` CPHA, `[3:0]` = length-1. Then `length` data bytes. One result byte returns per data byte, CS held low throughout.
- **I2C write** (`01`, `[5]`=0): `[3:0]` = length-1. Then 7-bit device address, then `length` data bytes. One status byte returns (0x00 ok, 0xFF NACK).
- **I2C read** (`01`, `[5]`=1): `[3:0]` = read count-1. Then device address, then write count (0-15), then the write bytes. Each read byte returns, then a status byte.

The hub RTL (`uart_rx`, `uart_tx`, `spi_master`, `i2c_master`) is verbatim from your repo. **One change:** `dispatcher.v` now has a single shared data buffer (`BUF_DEPTH`, default 16) instead of two 16-byte buffers. SPI and I2C never run together, so behaviour is identical at depth 16; your `tb_bridge_integration_full.v` produces byte-identical output with it. This saves about 3.9k um2. Setting `BUF_DEPTH` to 8 or 4 saves more, but transactions then must not exceed that many bytes.

## eFPGA

- 16 cells (`NC`), each a LUT4 plus a flop (registered output only, so no combinational loops).
- Hard blocks: 8-bit shift register with parallel load from `ui_in`, 8-bit programmable down-counter divider, 8-bit CRC LFSR with programmable polynomial.
- Source vector: const0, const1, `ui_in[7:0]`, 4 hub events, 16 cell outputs, 3 hard-block outputs (33 sources).
- Routing muxes see a window of 8 candidates, not the whole vector. Cell input 0 sees cells `i-3..i+4`, input 1 sees `i-7..i`, input 2 sees `i..i+7`, input 3 sees 8 consecutive sources of the full vector starting at `2i`. Pins therefore reach only the low-numbered cells, and outputs `j` sees cell-region entries `3j..3j+7`. `test/cfgimage.py` knows all of this and raises an error if a route is impossible.
- Bitstream: 507 bits. Layout is in `efpga_core.v` and mirrored in `cfgimage.py`.

### Loading a bitstream

1. Build it: `Fabric(16).set_cell(...)`, `set_out(...)`, then `stream()` returns the bits to shift in, first bit first.
2. For every bit: put it on `uio[0]`, wait, raise `uio[1]`, wait, lower `uio[1]`. Hold each level for at least 4 system clocks (`CFG_MODE=0` synchronises `cfg_clk` into the clock domain and shifts on its detected rising edge).
3. Pulse `rst_n` low once afterwards so every cell starts from zero. Outputs are garbage while loading and until that reset. The bitstream is volatile: load it after every power-up.
4. Read-back: shift a second bitstream in while sampling `uio[2]` just before each rising edge of `uio[1]`. The captured bits equal the first bitstream.

Example (3-bit counter on uo_out[3:1]): see `test_04_counter3` in `test/test.py`.

## Tests (`test/`)

Run: `cd test && pip install -r requirements.txt && make` (needs iverilog). CI runs the same through `.github/workflows/test.yaml`.

| # | Test | What it proves |
|---|---|---|
| 1 | reset_state | UART TX idles high, CS_N high, `uio_oe` = 0x3C, open-drain pins only drive 0 |
| 2 | config_chain_readback | chain is exactly 507 bits and order-preserving (random pattern written, read back from CFG_OUT) |
| 3 | toggle_cell_and_reset | one cell inverting itself toggles every clock; reset holds it at 0 |
| 4 | counter3 | multi-cell routing: a 3-bit binary counter counts 0..7 and wraps |
| 5 | pin_passthrough | `ui_in` reaches a cell and a pin output |
| 6 | divider_hard_block | div8 reload 3 gives a tick every 4 clocks |
| 7 | crc_hard_block | crc8 MSB matches a Python model over 48 random bits |
| 8 | shift_register_to_pins | `uo_mode`=1 and parallel load put `ui_in` on `uo_out[7:1]` |
| 9 | hub_spi | UART command -> 3-byte SPI transfer, MOSI and the returned bytes both checked |
| 10 | hub_i2c_write | 3-byte I2C write, slave sees 0xA0 AA BB CC, status 0x00 back |
| 11 | hub_i2c_register_read | write register, repeated START, read byte (WHO_AM_I pattern) |
| 12 | random_routing_vs_model | 3 random bitstreams (random LUTs, routing, hard-block wiring) vs a cycle-accurate Python model |
| 13 | fabric_watches_hub | the `rx_valid` event reaches the fabric |

In addition, with a slightly broken routing window (one mux offset changed by one), test 12 fails. Tests 2 to 7 alone did not catch it, which is why test 12 exists.

## Tiny Tapeout flow (what the GitHub Actions do on every push)

1. `test`: cocotb on the RTL (the tests above).
2. `gds`: LibreLane hardens the design into the 2x2 tile (synthesis, floorplan, placement, clock tree, routing, DRC/LVS), using `src/config.json`.
3. `precheck`: the shuttle's own checks on the GDS (layers, pins, power, density).
4. `gl_test`: re-runs the same cocotb tests on the gate-level netlist. This is why `test.py` always loads a bitstream before looking at fabric outputs: the config flops have no reset and are X in simulation until loaded.
5. `viewer` and `docs`: a 3D/2D view and the datasheet page from `docs/info.md`.
After all green, submit the repo on the Tiny Tapeout site for the SKY 26d shuttle.

## Area: what is measured and what is not

Yosys + `abc` against `sky130_fd_sc_hd__tt_025C_1v80`, full chip (hub + fabric), 2x2 tile = 75,603 um2. "Fill" = synth area x 1.10 / tile area. The 1.10 is my assumption for placement overhead; it is not a measurement.

| Variant | Synth area (um2) | Fill |
|---|---|---|
| **Default: NC=16, CFG_MODE=0, BUF_DEPTH=16** | **44,742** | **65%** |
| NC=16, CFG_MODE=0, BUF_DEPTH=8 | 41,784 | 61% |
| NC=16, CFG_MODE=1, BUF_DEPTH=16 | 38,941 | 57% |
| NC=24, CFG_MODE=1, BUF_DEPTH=8 | 43,389 | 63% |
| NC=24, CFG_MODE=0, BUF_DEPTH=16 | 54,625 | 80% |

Hub alone (UART+SPI+I2C+dispatcher): 22.1k at BUF_DEPTH=16, 19.2k at 8, 17.6k at 4.

`CFG_MODE=1` shifts the 507 config flops directly on the `cfg_clk` pin. It is about 5.8k smaller, but the template's CTS only balances `clk`, so a 500-flop net on a raw pin is a hold-timing risk I could not check without place-and-route. `CFG_MODE=0` (default) keeps one clock domain.

`config.json` sets `PL_TARGET_DENSITY_PCT` to 75 (template default 60). At 65% estimated fill, 60 would likely fail global placement (GPL-0302). 75 is a judgement, not a tested value.

**Not verified:** place-and-route, timing (CLOCK_PERIOD stays 20 ns / 50 MHz; the hub was written for 100 MHz and runs at 25 MHz here), DRC, and the gate-level test. The first real answer comes from the first GitHub Actions `gds` run.

If `gds` fails on fit or congestion, try in this order: raise `PL_TARGET_DENSITY_PCT` to 80; `BUF_DEPTH` 16 -> 8 (restrict transactions to 8 bytes); `NC` 16 -> 12; `CFG_MODE` 1 (accept the hold risk). `NC`, `BUF_DEPTH`, `CFG_MODE` are localparams at the top of `project.v`; keep `NC` in `test/test.py` in step.

## Before you push

- Put your Discord handle in `info.yaml` if you want the Tapeout role; check the author name.
- Enable GitHub Pages for the repo (needed by the `viewer` job).
- Keep `PROJECT_SOURCES` in `test/Makefile` and `source_files` in `info.yaml` in step if you add files.
- `docs/info.md` is the datasheet text; add a wiring photo later if you like.
