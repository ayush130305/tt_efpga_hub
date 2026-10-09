# test.py
# cocotb tests for tt_um_ayush130305_efpga_hub (RTL and gate level).
# Hub tests drive the pins as a UART host, an SPI slave and an I2C slave.
# Fabric tests build bitstreams with cfgimage.py, load them through the pins, and check behaviour.

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge

from cfgimage import Fabric

CLK_NS = 40                  # 25 MHz, matches clock_hz in info.yaml
BIT_CLKS = 217               # 25 MHz / 115200 baud, same rounding as uart_rx / uart_tx
NC = 12                      # must match NC in src/project.v

# uio_in bit positions
CFG_DATA, CFG_CLK, SDA_IN = 0, 1, 7


def bit(sig, n):
    """Return bit n of a signal as '0', '1', 'x' or 'z' (safe when other bits are still X)."""
    s = str(sig.value)
    return s[len(s) - 1 - n]


class Pins:
    """Shared owner of ui_in / uio_in so several coroutines can each drive their own bits."""

    def __init__(self, dut):
        self.dut = dut
        self.ui = 0x01       # uart rx idles high on ui_in[0]
        self.uio = 0x80      # i2c sda released (pulled high)
        self.push()

    def push(self):
        self.dut.ui_in.value = self.ui
        self.dut.uio_in.value = self.uio

    def set_ui(self, n, v):
        self.ui = (self.ui & ~(1 << n)) | ((v & 1) << n)
        self.push()

    def set_uio(self, n, v):
        self.uio = (self.uio & ~(1 << n)) | ((v & 1) << n)
        self.push()


async def start(dut):
    """Start the clock, create the pin owner and apply reset."""
    cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())
    dut.ena.value = 1
    pins = Pins(dut)
    await reset(dut)
    return pins


async def reset(dut, cycles=6):
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, cycles)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)


# ------------------------------------------------------------------ config chain

async def load_cfg(dut, pins, stream, capture=None):
    """Shift a bitstream in through cfg_data / cfg_clk. Optionally capture cfg_out before each edge."""
    for b in stream:
        pins.set_uio(CFG_DATA, b)
        await ClockCycles(dut.clk, 4)
        if capture is not None:
            capture.append(bit(dut.uio_out, 2))
        pins.set_uio(CFG_CLK, 1)
        await ClockCycles(dut.clk, 4)
        pins.set_uio(CFG_CLK, 0)
        await ClockCycles(dut.clk, 1)
    await ClockCycles(dut.clk, 6)


async def configure(dut, pins, fab):
    """Load a fabric configuration, then reset so every cell starts from zero."""
    await load_cfg(dut, pins, fab.stream())
    await reset(dut)


# ------------------------------------------------------------------ UART host

async def host_send_byte(dut, pins, data):
    pins.set_ui(0, 0)
    await ClockCycles(dut.clk, BIT_CLKS)
    for i in range(8):
        pins.set_ui(0, (data >> i) & 1)
        await ClockCycles(dut.clk, BIT_CLKS)
    pins.set_ui(0, 1)
    await ClockCycles(dut.clk, BIT_CLKS)


async def host_receive_bytes(dut, out):
    while True:
        await RisingEdge(dut.clk)
        if bit(dut.uo_out, 0) == '0':
            await ClockCycles(dut.clk, BIT_CLKS + BIT_CLKS // 2)
            v = 0
            for i in range(8):
                v |= int(bit(dut.uo_out, 0)) << i
                await ClockCycles(dut.clk, BIT_CLKS)
            out.append(v)


# ------------------------------------------------------------------ SPI slave (mode 0)
# uio[3]=mosi, uio[4]=sclk, uio[5]=cs_n, miso is ui_in[1]

async def spi_slave(dut, pins, canned, received):
    idx, active, prev = 0, False, '0'
    tx = rx = nbits = 0
    while True:
        await RisingEdge(dut.clk)
        cs, sclk, mosi = bit(dut.uio_out, 5), bit(dut.uio_out, 4), bit(dut.uio_out, 3)
        if cs == '1' or cs not in '01':
            active = False
            continue
        if not active:
            active, prev = True, sclk
            tx, rx, nbits = canned[idx % len(canned)], 0, 0
            pins.set_ui(1, (tx >> 7) & 1)
            continue
        if sclk != prev:
            if sclk == '1':                      # rising edge: sample mosi
                rx = ((rx << 1) | int(mosi)) & 0xFF
                nbits += 1
                if nbits == 8:
                    received.append(rx)
                    idx += 1
            else:                                # falling edge: next miso bit
                if nbits == 8:
                    tx, rx, nbits = canned[idx % len(canned)], 0, 0
                else:
                    tx = (tx << 1) & 0xFF
                pins.set_ui(1, (tx >> 7) & 1)
            prev = sclk


# ------------------------------------------------------------------ I2C slave
# uio[6]=scl (open drain), uio[7]=sda (open drain, bidirectional)

def drive_sda(dut, pins, shared):
    master_low = bit(dut.uio_oe, 7) == '1' and bit(dut.uio_out, 7) == '0'
    pins.set_uio(SDA_IN, 0 if (master_low or shared["slave_sda_oe"]) else 1)


async def sda_bus_driver(dut, pins, shared):
    while True:
        await RisingEdge(dut.clk)
        drive_sda(dut, pins, shared)


async def i2c_slave(dut, pins, shared, write_bytes_seen, read_data_to_send):
    prev_scl, prev_sda = 0, 1
    active, mode, slave_shift, bits_seen = False, 0, 0, 0
    driving_ack, is_first, pending_switch = False, True, False
    send_shift, send_done, send_idx = 0, 0, 0

    def set_oe(v):
        shared["slave_sda_oe"] = v
        drive_sda(dut, pins, shared)

    while True:
        await RisingEdge(dut.clk)
        scl = 1 if bit(dut.uio_oe, 6) != '1' else 0
        sda = int(pins.uio >> SDA_IN) & 1
        if prev_scl == 1 and scl == 1 and prev_sda == 1 and sda == 0:
            active, bits_seen, mode, driving_ack = True, 0, 0, False
            is_first, pending_switch = True, False
        elif prev_scl == 1 and scl == 1 and prev_sda == 0 and sda == 1:
            active = False
            set_oe(False)
        elif active and prev_scl == 0 and scl == 1:
            if mode == 0 and bits_seen < 8:
                slave_shift = ((slave_shift << 1) | sda) & 0xFF
                bits_seen += 1
        elif active and prev_scl == 1 and scl == 0:
            if mode == 0:
                if bits_seen == 8 and not driving_ack:
                    set_oe(True)
                    driving_ack = True
                    write_bytes_seen.append(slave_shift)
                    if is_first and (slave_shift & 1) == 1:
                        pending_switch = True
                    is_first = False
                elif driving_ack:
                    set_oe(False)
                    driving_ack, bits_seen = False, 0
                    if pending_switch:
                        mode, pending_switch = 1, False
                        first = read_data_to_send[send_idx % len(read_data_to_send)]
                        set_oe(((first >> 7) & 1) == 0)
                        send_shift, send_done = (first << 1) & 0xFF, 1
                        send_idx += 1
            else:
                if send_done < 8:
                    set_oe(((send_shift >> 7) & 1) == 0)
                    send_shift = (send_shift << 1) & 0xFF
                    send_done += 1
                else:
                    set_oe(False)
                    send_done = 0
                    send_shift = read_data_to_send[send_idx % len(read_data_to_send)]
                    send_idx += 1
        prev_scl, prev_sda = scl, sda


# ================================================================== tests

@cocotb.test()
async def test_01_reset_state(dut):
    """After reset: uart tx idles high, spi cs_n high, config pins are inputs, bus pins are released."""
    pins = await start(dut)
    assert bit(dut.uo_out, 0) == '1', "uart tx must idle high"
    assert bit(dut.uio_out, 5) == '1', "spi cs_n must idle high"
    assert dut.uio_oe.value == 0b00111100, f"uio_oe = {dut.uio_oe.value}"
    assert bit(dut.uio_out, 7) == '0' and bit(dut.uio_out, 6) == '0', "open-drain pins drive 0 only"


@cocotb.test()
async def test_02_config_chain_readback(dut):
    """The chain is exactly NCFG long and shifts in order: what goes in comes out of cfg_out unchanged."""
    pins = await start(dut)
    fab = Fabric(NC)
    rng = random.Random(1234)
    a = [rng.getrandbits(1) for _ in range(fab.ncfg)]
    b = [rng.getrandbits(1) for _ in range(fab.ncfg)]
    await load_cfg(dut, pins, a)
    got = []
    await load_cfg(dut, pins, b, capture=got)
    assert ''.join(got) == ''.join(map(str, a)), "read-back of config chain differs from what was written"


@cocotb.test()
async def test_03_toggle_cell_and_reset(dut):
    """Cell 0 inverts itself: uo_out[1] toggles every clock; reset forces it to 0."""
    pins = await start(dut)
    fab = Fabric(NC)
    fab.set_cell(0, [fab.cell(0)], lambda a: 1 - a)
    fab.set_out(0, fab.cell(0))
    await configure(dut, pins, fab)
    seen = []
    for _ in range(8):
        await FallingEdge(dut.clk)
        seen.append(bit(dut.uo_out, 1))
    assert ''.join(seen) in ('01010101', '10101010'), f"no toggling: {seen}"
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 3)
    await FallingEdge(dut.clk)
    assert bit(dut.uo_out, 1) == '0', "cell must be held at 0 in reset"
    dut.rst_n.value = 1


@cocotb.test()
async def test_04_counter3(dut):
    """3-bit binary counter in cells 4,5,6 shows 0..7 on uo_out[3:1]."""
    pins = await start(dut)
    fab = Fabric(NC)
    q = [fab.cell(4), fab.cell(5), fab.cell(6)]
    fab.set_cell(4, [q[0]], lambda a: 1 - a)
    fab.set_cell(5, [q[1], q[0]], lambda a, b: a ^ b)
    fab.set_cell(6, [q[2], q[1], q[0]], lambda a, b, c: a ^ (b & c))
    for j in range(3):
        fab.set_out(j, q[j])
    await configure(dut, pins, fab)
    vals = []
    for _ in range(18):
        await FallingEdge(dut.clk)
        vals.append((int(bit(dut.uo_out, 3)) << 2) | (int(bit(dut.uo_out, 2)) << 1) | int(bit(dut.uo_out, 1)))
    start_v = vals[0]
    assert vals == [(start_v + k) % 8 for k in range(18)], f"counter sequence wrong: {vals}"


@cocotb.test()
async def test_05_pin_passthrough(dut):
    """ui_in[4] -> cell 3 (registered) -> uo_out[2]."""
    pins = await start(dut)
    fab = Fabric(NC)
    fab.set_cell(3, [fab.pin(4)], lambda a: a)
    fab.set_out(1, fab.cell(3))
    await configure(dut, pins, fab)
    for v in (1, 0, 1, 1, 0):
        pins.set_ui(4, v)
        await ClockCycles(dut.clk, 2)
        await FallingEdge(dut.clk)
        assert bit(dut.uo_out, 2) == str(v), f"passthrough failed for {v}"


@cocotb.test()
async def test_06_divider_hard_block(dut):
    """div8 with reload 3 ticks once every 4 clocks (tick on uo_out[5])."""
    pins = await start(dut)
    fab = Fabric(NC)
    fab.set_cell(0, [], lambda: 1)          # constant 1 cell drives the divider enable
    fab.set_sink(3, fab.cell(0))
    fab.set_div(3)
    fab.set_out(4, fab.hb(1))
    await configure(dut, pins, fab)
    ticks = []
    for k in range(40):
        await FallingEdge(dut.clk)
        if bit(dut.uo_out, 5) == '1':
            ticks.append(k)
    gaps = {b - a for a, b in zip(ticks, ticks[1:])}
    assert len(ticks) >= 8 and gaps == {4}, f"tick positions {ticks}"


@cocotb.test()
async def test_07_crc_hard_block(dut):
    """crc8 (poly 0x07) fed from a pin through cell 3; MSB on uo_out[5] must match a Python model."""
    pins = await start(dut)
    fab = Fabric(NC)
    poly = 0x07
    fab.set_cell(3, [fab.pin(4)], lambda a: a)   # data path
    fab.set_cell(7, [], lambda: 1)               # constant 1 -> crc enable
    fab.set_sink(4, fab.cell(3))
    fab.set_sink(5, fab.cell(7))
    fab.set_crc_poly(poly)
    fab.set_out(4, fab.hb(2))
    await configure(dut, pins, fab)
    rng = random.Random(7)
    crc, q3, q7 = 0, 0, 0
    for n in range(48):
        await FallingEdge(dut.clk)
        # state seen here includes every rising edge so far
        assert int(bit(dut.uo_out, 5)) == (crc >> 7) & 1, f"crc msb mismatch at cycle {n}"
        d = rng.getrandbits(1)
        pins.set_ui(4, d)
        # model the next rising edge: it uses the registered values from before it
        if q7:
            fb = ((crc >> 7) ^ q3) & 1
            crc = ((crc << 1) & 0xFF) ^ (poly if fb else 0)
        q3, q7 = d, 1


@cocotb.test()
async def test_08_shift_register_to_pins(dut):
    """uo_mode=1 puts the shift register byte on uo_out[7:1]; ld from a constant-1 cell loads ui_in."""
    pins = await start(dut)
    fab = Fabric(NC)
    fab.set_cell(0, [], lambda: 1)
    fab.set_sink(2, fab.cell(0))
    fab.set_uo_mode(1)
    await configure(dut, pins, fab)
    pins.ui = 0xAB
    pins.push()
    await ClockCycles(dut.clk, 4)
    await FallingEdge(dut.clk)
    assert dut.uo_out.value == ((0xAB & 0x7F) << 1) | 1, f"uo_out = {dut.uo_out.value}"


@cocotb.test()
async def test_09_hub_spi(dut):
    """UART command -> SPI transaction (3 bytes, mode 0) -> results back over UART."""
    pins = await start(dut)
    got, mosi_seen = [], []
    cocotb.start_soon(host_receive_bytes(dut, got))
    cocotb.start_soon(spi_slave(dut, pins, [0x99, 0x5A, 0xC3], mosi_seen))
    await host_send_byte(dut, pins, 0b00_0_0_0010)       # SPI, mode 0, 3 bytes
    for d in (0x11, 0x22, 0x33):
        await host_send_byte(dut, pins, d)
    await ClockCycles(dut.clk, 12 * BIT_CLKS * 3)
    assert mosi_seen == [0x11, 0x22, 0x33], f"slave saw {[hex(x) for x in mosi_seen]}"
    assert got == [0x99, 0x5A, 0xC3], f"host got {[hex(x) for x in got]}"


@cocotb.test()
async def test_10_hub_i2c_write(dut):
    """UART command -> I2C write of 3 bytes -> status 0x00 back over UART."""
    pins = await start(dut)
    got, seen, shared = [], [], {"slave_sda_oe": False}
    cocotb.start_soon(host_receive_bytes(dut, got))
    cocotb.start_soon(sda_bus_driver(dut, pins, shared))
    cocotb.start_soon(i2c_slave(dut, pins, shared, seen, [0x00]))
    await host_send_byte(dut, pins, 0b01_0_0_0010)       # I2C write, 3 data bytes
    for d in (0x50, 0xAA, 0xBB, 0xCC):
        await host_send_byte(dut, pins, d)
    await ClockCycles(dut.clk, 25_000_000 // 1000 * 2)   # 2 ms
    assert seen == [0xA0, 0xAA, 0xBB, 0xCC], f"slave saw {[hex(x) for x in seen]}"
    assert got == [0x00], f"host got {[hex(x) for x in got]}"


@cocotb.test()
async def test_11_hub_i2c_register_read(dut):
    """UART command -> I2C write register address, repeated START, read 1 byte (WHO_AM_I pattern)."""
    pins = await start(dut)
    got, seen, shared = [], [], {"slave_sda_oe": False}
    cocotb.start_soon(host_receive_bytes(dut, got))
    cocotb.start_soon(sda_bus_driver(dut, pins, shared))
    cocotb.start_soon(i2c_slave(dut, pins, shared, seen, [0x68]))
    await host_send_byte(dut, pins, 0b01_1_0_0000)       # I2C read-capable, 1 read byte
    for d in (0x68, 1, 0x75):                            # device, write count, register
        await host_send_byte(dut, pins, d)
    await ClockCycles(dut.clk, 25_000_000 // 1000 * 2)
    assert got == [0x68, 0x00], f"host got {[hex(x) for x in got]}"
    assert seen == [0xD0, 0x75, 0xD1], f"slave saw {[hex(x) for x in seen]}"


class FabricModel:
    """Cycle-accurate Python model of efpga_core.v driven by a Fabric bitstream (hub events held at 0)."""

    def __init__(self, fab):
        self.f = fab
        self.q = [0] * fab.nc
        self.sh = self.cnt = self.tick = self.crc = 0

    def field(self, off, width):
        return sum(self.f.v[off + b] << b for b in range(width))

    def sources(self, ui):
        f = self.f
        s = [0, 1] + [(ui >> k) & 1 for k in range(8)] + [0, 0, 0, 0] + list(self.q)
        return s + [(self.sh >> 7) & 1, self.tick, (self.crc >> 7) & 1]

    def outputs(self, ui):
        f, s = self.f, self.sources(ui)
        fab = [s[f.out_cands(j)[self.field(f.off_out + j * f.swo, f.swo)]] for j in range(f.NOUT)]
        sh7 = [(self.sh >> k) & 1 for k in range(7)]
        pins = sh7 if self.field(f.off_uom, 1) else fab
        return sum(b << (k + 1) for k, b in enumerate(pins))

    def step(self, ui):
        f, s = self.f, self.sources(ui)
        new_q = []
        for i in range(f.nc):
            base = i * f.cellb
            idx = 0
            for j in range(4):
                sel = self.field(base + 16 + j * f.sw, f.sw)
                idx |= s[f.cell_cands(i, j)[sel]] << j
            new_q.append((self.field(base, 16) >> idx) & 1)
        k = [s[f.sink_cands(j)[self.field(f.off_sink + j * f.sw, f.sw)]] for j in range(f.NSINK)]
        sin, sh_en, ld, div_en, csin, cen, cclr = k
        if ld: self.sh = ui
        elif sh_en: self.sh = ((self.sh << 1) | sin) & 0xFF
        reload = self.field(f.off_div, 8)
        if not div_en: self.cnt, self.tick = reload, 0
        elif self.cnt == 0: self.cnt, self.tick = reload, 1
        else: self.cnt, self.tick = self.cnt - 1, 0
        poly = self.field(f.off_crc, 8)
        if cclr: self.crc = 0
        elif cen:
            fb = ((self.crc >> 7) ^ csin) & 1
            self.crc = ((self.crc << 1) & 0xFF) ^ (poly if fb else 0)
        self.q = new_q


@cocotb.test()
async def test_12_random_routing_vs_model(dut):
    """Random bitstreams (random LUTs, routing, hard-block wiring) must match the Python model cycle by cycle."""
    pins = await start(dut)
    fab = Fabric(NC)
    for seed in (11, 22, 33):
        rng = random.Random(seed)
        fab.v = [rng.getrandbits(1) for _ in range(fab.ncfg)]
        await load_cfg(dut, pins, fab.stream())
        dut.rst_n.value = 0
        await ClockCycles(dut.clk, 3)
        await FallingEdge(dut.clk)
        dut.rst_n.value = 1                  # released mid-cycle; the next rising edge is the first live one
        model = FabricModel(fab)
        model.step(pins.ui)
        for n in range(50):
            await FallingEdge(dut.clk)
            text = str(dut.uo_out.value)[:7]
            assert 'x' not in text.lower() and 'z' not in text.lower(), f"X on uo_out[7:1], seed {seed} cycle {n}"
            got, exp = int(text, 2), model.outputs(pins.ui) >> 1
            assert got == exp, f"seed {seed} cycle {n}: rtl {got:07b} model {exp:07b}"
            pins.ui = rng.getrandbits(8) | 1
            pins.push()
            model.step(pins.ui)


@cocotb.test()
async def test_13_fabric_watches_hub(dut):
    """Fabric cell 4 latches the hub's rx_valid event: uo_out[1] goes high after the first UART byte."""
    pins = await start(dut)
    fab = Fabric(NC)
    fab.set_cell(4, [fab.cell(4), fab.event(0)], lambda a, b: a | b)
    fab.set_out(0, fab.cell(4))
    await configure(dut, pins, fab)
    await FallingEdge(dut.clk)
    assert bit(dut.uo_out, 1) == '0'
    await host_send_byte(dut, pins, 0xFF)                # command byte 0xFF is ignored by the dispatcher
    await ClockCycles(dut.clk, 4)
    assert bit(dut.uo_out, 1) == '1', "rx_valid event never reached the fabric"
