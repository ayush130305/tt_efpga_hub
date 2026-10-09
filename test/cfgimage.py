# cfgimage.py
# Bitstream builder for the eFPGA in src/efpga_core.v. It mirrors the Verilog window and layout
# rules exactly; test.py proves that by simulating bitstreams made here against the real RTL.
import itertools


def clog2(v):
    n, x = 0, v - 1
    while x > 0:
        n, x = n + 1, x >> 1
    return n


class Fabric:
    NI = 12   # 8 ui_in pins + 4 hub events
    NH = 3    # hard-block outputs
    NOUT = 7
    NSINK = 7

    def __init__(self, nc=16, w=8, wo=8):
        self.nc = nc
        self.cb = 2 + self.NI
        self.ns = self.cb + nc + self.NH
        self.lo = nc + self.NH
        self.we = min(w, nc)
        self.woe = min(wo, self.lo)
        self.sw = clog2(self.we)
        self.swo = clog2(self.woe)
        self.cellb = 16 + 4 * self.sw
        self.off_sink = nc * self.cellb
        self.off_div = self.off_sink + self.NSINK * self.sw
        self.off_crc = self.off_div + 8
        self.off_out = self.off_crc + 8
        self.off_uom = self.off_out + self.NOUT * self.swo
        self.ncfg = self.off_uom + 1
        self.v = [0] * self.ncfg  # v[k] is cfg[k] in the Verilog

    # ---- source indices ----
    def const(self, b): return b
    def pin(self, k): return 2 + k           # ui_in[k], k = 0..7
    def event(self, k): return 10 + k        # 0 rx_valid, 1 spi_done, 2 i2c_done, 3 uart_tx_start
    def cell(self, i): return self.cb + i
    def hb(self, k): return self.cb + self.nc + k  # 0 shift msb, 1 div tick, 2 crc msb

    # ---- candidate windows (same arithmetic as the Verilog) ----
    @staticmethod
    def _win(r, l, off, win):
        return [r + ((off + k) % l) for k in range(win)]

    def cell_cands(self, i, j):
        nc, we = self.nc, self.we
        if j == 0: return self._win(self.cb, nc, (i - 3 + 2 * nc) % nc, we)
        if j == 1: return self._win(self.cb, nc, (i - we + 1 + 2 * nc) % nc, we)
        if j == 2: return self._win(self.cb, nc, i % nc, we)
        return self._win(0, self.ns, (2 * i) % self.ns, we)

    def sink_cands(self, j): return self._win(0, self.ns, (4 * j + 1) % self.ns, self.we)
    def out_cands(self, j): return self._win(self.cb, self.lo, (3 * j) % self.lo, self.woe)

    # ---- field helpers ----
    def _set(self, off, width, val):
        for b in range(width):
            self.v[off + b] = (val >> b) & 1

    def set_cell(self, i, srcs, fn):
        """Cell i computes fn(*bits of srcs) registered. srcs: up to 4 source indices."""
        n = len(srcs)
        for slots in itertools.permutations(range(4), n):
            if all(s in self.cell_cands(i, j) for s, j in zip(srcs, slots)):
                break
        else:
            raise ValueError(f"cell {i}: sources {srcs} not reachable by distinct inputs")
        base = i * self.cellb
        lut = 0
        for idx in range(16):
            bits = [(idx >> j) & 1 for j in slots]
            if fn(*bits) & 1:
                lut |= 1 << idx
        self._set(base, 16, lut)
        for j in range(4):
            sel = 0
            if j in slots:
                sel = self.cell_cands(i, j).index(srcs[slots.index(j)])
            self._set(base + 16 + j * self.sw, self.sw, sel)

    def _route(self, cands, src, what):
        if src not in cands:
            raise ValueError(f"{what}: source {src} not in window {cands}")
        return cands.index(src)

    def set_sink(self, j, src):
        """Hard-block control: 0 shift sin, 1 sh_en, 2 ld, 3 div en, 4 crc sin, 5 crc en, 6 crc clr."""
        sel = self._route(self.sink_cands(j), src, f"sink {j}")
        self._set(self.off_sink + j * self.sw, self.sw, sel)

    def set_out(self, j, src):
        """Fabric output j drives uo_out[j+1]."""
        sel = self._route(self.out_cands(j), src, f"out {j}")
        self._set(self.off_out + j * self.swo, self.swo, sel)

    def set_div(self, reload): self._set(self.off_div, 8, reload)
    def set_crc_poly(self, poly): self._set(self.off_crc, 8, poly)
    def set_uo_mode(self, b): self._set(self.off_uom, 1, b)

    def stream(self):
        """Bits in the order they must be shifted in: cfg[N-1] first, cfg[0] last."""
        return [self.v[self.ncfg - 1 - k] for k in range(self.ncfg)]
