#!/usr/bin/env python3
"""Decode SPI from a Siglent SDS2000X HD by pulling raw waveforms and decoding locally.

The scope's own :DECode:BUS<n>:RESult? query returns an incomplete list, so this
downloads the CLK/MOSI/MISO/CS channels and decodes them here instead.
Channel sources, bit order and word length are read from the scope's decode setup.

Thresholds are in volts. By default each one is placed midway between the two
logic levels seen on that channel; if a channel doesn't toggle (levels closer
than --min-swing) it falls back to --default-threshold.

CS glitches shorter than --cs-min-width (default: one clock period, measured
from the clock) are ignored.

Usage: spi_decode.py [--host scope] [--bus 1] [--falling] [-o out.csv]
                     [--cs-threshold V] [--cs-min-width S]
                     [--save DIR | --load DIR]
"""
import argparse
import csv
import json
import os
import socket
import statistics
import struct
import sys
from collections import Counter

LINES = ("clk", "mosi", "miso", "cs")

p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
p.add_argument("--host", default="scope")
p.add_argument("--bus", default="1")
p.add_argument("--falling", action="store_true", help="sample on falling clock edge (default rising)")
p.add_argument("-o", "--output", help="CSV file (default stdout)")
for line in LINES:
    p.add_argument(f"--{line}-threshold", type=float, metavar="V",
                   help=f"{line.upper()} logic threshold in volts (default: auto)")
p.add_argument("--cs-min-width", type=float, metavar="S",
               help="ignore CS pulses shorter than this many seconds (default: one clock period; 0 disables)")
p.add_argument("--min-swing", type=float, default=1.0, metavar="V",
               help="auto threshold needs levels at least this far apart (default 1.0)")
p.add_argument("--default-threshold", type=float, default=1.4, metavar="V",
               help="threshold used when a line doesn't toggle (default 1.4)")
p.add_argument("--save", metavar="DIR", help="also save the raw capture to DIR")
p.add_argument("--load", metavar="DIR", help="decode a capture saved with --save instead of reading the scope")
p.add_argument("-v", "--verbose", action="store_true", help="print levels/thresholds/CS windows to stderr")
args = p.parse_args()

TO_UNSIGNED = bytes(v ^ 0x80 for v in range(256))  # signed int8 -> order-preserving unsigned


def log(*a):
    print(*a, file=sys.stderr)


class Scope:
    def __init__(self, host):
        self.s = socket.create_connection((host, 5025), timeout=20)

    def send(self, cmd):
        self.s.sendall((cmd + "\n").encode())

    def _recv_exact(self, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = self.s.recv(min(1 << 20, n - len(buf)))
            if not chunk:
                raise ConnectionError("scope closed connection")
            buf += chunk
        return bytes(buf)

    def query(self, cmd):
        self.send(cmd)
        d = b""
        while not d.endswith(b"\n"):
            d += self.s.recv(4096)
        return d.decode().strip()

    def query_block(self, cmd):
        self.send(cmd)
        head = self._recv_exact(2)
        if head[:1] != b"#":
            raise ValueError(f"unexpected reply to {cmd}: {head!r}")
        size = int(self._recv_exact(int(head[1:2])))
        data = self._recv_exact(size)
        self._recv_exact(1)  # trailing "\n" (waveform data has a second one)
        self.s.settimeout(0.2)
        try:
            self.s.recv(16)
        except socket.timeout:
            pass
        self.s.settimeout(20)
        return data

    def scaling(self, ch):
        """Volts = code * gain - offset, from the waveform preamble (byte-wide data)."""
        self.send(f":WAVeform:SOURce {ch}")
        pre = self.query_block(":WAVeform:PREamble?")
        vdiv, ofs, code_per_div = struct.unpack_from("<fff", pre, 156)
        probe = struct.unpack_from("<f", pre, 328)[0]
        # code_per_div is given for 16-bit words; byte data is the top 8 bits
        return vdiv * probe / (code_per_div / 256), ofs * probe

    def waveform(self, ch, total, maxp):
        self.send(f":WAVeform:SOURce {ch}")
        out = bytearray()
        while len(out) < total:
            self.send(f":WAVeform:STARt {len(out)}")
            self.send(f":WAVeform:POINt {min(maxp, total - len(out))}")
            blk = self.query_block(":WAVeform:DATA?")
            if not blk:
                break
            out += blk
        return bytes(out)

    def close(self):
        self.s.close()


def read_scope():
    sc = Scope(args.host)
    b = f":DECode:BUS{args.bus}:SPI"
    meta = {
        "src": {
            "clk": sc.query(f"{b}:CLKSource?"),
            "mosi": sc.query(f"{b}:MOSISource?"),
            "miso": sc.query(f"{b}:MISOSource?"),
            "cs": sc.query(f"{b}:NCSSource?"),
        },
        "msb_first": sc.query(f"{b}:BITorder?").upper().startswith("MSB"),
        "dlen": int(sc.query(f"{b}:DLENgth?")),
    }
    sc.send(":WAVeform:WIDTh BYTE")
    sc.send(":WAVeform:INTerval 1")
    total = int(float(sc.query(":ACQuire:POINts?")))
    maxp = int(float(sc.query(":WAVeform:MAXPoint?")))
    meta["srate"] = float(sc.query(":ACQuire:SRATe?"))
    meta["scale"] = {line: sc.scaling(ch) for line, ch in meta["src"].items()}
    raw = {line: sc.waveform(ch, total, maxp) for line, ch in meta["src"].items()}
    sc.close()
    return meta, raw


if args.load:
    meta = json.load(open(os.path.join(args.load, "meta.json")))
    raw = {line: open(os.path.join(args.load, f"{line}.bin"), "rb").read() for line in LINES}
else:
    meta, raw = read_scope()
    if args.save:
        os.makedirs(args.save, exist_ok=True)
        json.dump(meta, open(os.path.join(args.save, "meta.json"), "w"), indent=1)
        for line, d in raw.items():
            open(os.path.join(args.save, f"{line}.bin"), "wb").write(d)

srate, dlen, msb_first = meta["srate"], meta["dlen"], meta["msb_first"]
total = len(raw["clk"])
wave = {line: d.translate(TO_UNSIGNED) for line, d in raw.items()}  # unsigned code = signed + 128


def to_volts(line, u):
    gain, ofs = meta["scale"][line]
    return (u - 128) * gain - ofs


def to_code(line, v):
    gain, ofs = meta["scale"][line]
    return (v + ofs) / gain + 128


def auto_levels(data, mask=None):
    """The two logic levels (unsigned codes): the most common value, and the most
    common value at least 40% of the full span away from it.

    Only samples where the signal holds steady (within 1 code) for ~20 ns are
    counted, so short noise spikes don't register as a logic level."""
    k = max(1, round(20e-9 * srate))
    idx = range(0, len(data) - k, 3) if mask is None else (i for i in mask if i + k < len(data))
    c = Counter(data[i] for i in idx if abs(data[i] - data[i + k]) <= 1)
    if not c:
        c = Counter(data)
    a = max(c, key=c.get)
    span = max(c) - min(c)
    far = {v: k for v, k in c.items() if abs(v - a) >= 0.4 * span}
    return a, (max(far, key=far.get) if far else a)


def pick_threshold(line, mask=None):
    """Returns (lo, hi) hysteresis thresholds in unsigned codes."""
    gain = meta["scale"][line][0]
    given = getattr(args, f"{line}_threshold")
    if given is not None:
        how, mid = "manual", to_code(line, given)
        swing = 0.5 / gain  # ±~0.075 V hysteresis
    else:
        a, b = auto_levels(wave[line], mask)
        va, vb = to_volts(line, a), to_volts(line, b)
        if abs(vb - va) >= args.min_swing:
            how, mid, swing = f"auto (levels {min(va, vb):.2f} V / {max(va, vb):.2f} V)", (a + b) / 2, abs(b - a)
        else:
            how, mid = f"default (no toggling, sits at {va:.2f} V)", to_code(line, args.default_threshold)
            swing = 0.5 / gain
    hyst = max(1, swing * 0.15)
    if args.verbose:
        log(f"{line.upper():4} threshold {to_volts(line, mid):.2f} V  {how}")
    return mid - hyst, mid + hyst


def transitions(data, lo, hi):
    """Hysteresis edge finder -> ([(index, new_state_is_high)], initial_state_is_high)."""
    above = data.translate(bytes(1 if v >= hi else 0 for v in range(256)))
    below = data.translate(bytes(1 if v <= lo else 0 for v in range(256)))
    initial = data[0] >= (lo + hi) / 2
    state, out, i = initial, [], 0
    while True:
        i = (below if state else above).find(1, i)
        if i < 0:
            return out, initial
        state = not state
        out.append((i, state))


def deglitch(edges, min_samples):
    """Drop pulses shorter than min_samples: an edge arriving too soon after the
    previous kept edge cancels it (the line is back where it was)."""
    out = []
    for i, st in edges:
        if out and i - out[-1][0] < min_samples:
            out.pop()
        else:
            out.append((i, st))
    return out


# Clock first: its period sets the default CS glitch width
clk_edges, _ = transitions(wave["clk"], *pick_threshold("clk"))
rising = [i for i, high in clk_edges if high]
period = statistics.median(b - a for a, b in zip(rising, rising[1:])) if len(rising) > 2 else 0

min_width = period if args.cs_min_width is None else args.cs_min_width * srate
cs_raw, cs_start_high = transitions(wave["cs"], *pick_threshold("cs"))
cs_edges = deglitch(cs_raw, min_width) if min_width > 0 else cs_raw
if args.verbose:
    log(f"clock period {period / srate * 1e9:.0f} ns; CS: {len(cs_raw)} raw edges, "
        f"{len(cs_edges)} after ignoring pulses < {min_width / srate * 1e9:.0f} ns")

# Chip select windows (active low)
windows, start = [], (None if cs_start_high else 0)
for i, high in cs_edges:
    if not high:
        start = i
    elif start is not None:
        windows.append((start, i))
        start = None
if start is not None:
    windows.append((start, total))
if args.verbose:
    log("CS windows:", ", ".join(f"{a / srate * 1e6:.3f}-{e / srate * 1e6:.3f} us" for a, e in windows))

# MISO is often tri-stated outside CS; base its auto threshold only on samples inside CS windows
inside = [i for a, e in windows for i in range(a, e, 37)] or None
th = {line: sum(pick_threshold(line, inside if line == "miso" else None)) / 2 for line in ("mosi", "miso")}
want_high = not args.falling


def word(bits):
    if not msb_first:
        bits = bits[::-1]
    return int("".join(map(str, bits)), 2)


rows = []
for frame, (a, e) in enumerate(windows):
    bits = {"mosi": [], "miso": []}
    t0 = None
    for i, high in clk_edges:
        if a < i < e and high == want_high:
            if len(bits["mosi"]) % dlen == 0:
                t0 = i
            for line in bits:
                bits[line].append(1 if wave[line][i] >= th[line] else 0)
            if len(bits["mosi"]) % dlen == 0:
                k = len(bits["mosi"]) - dlen
                rows.append([frame, f"{t0 / srate:.9f}",
                             f"0x{word(bits['mosi'][k:]):0{(dlen + 3) // 4}X}",
                             f"0x{word(bits['miso'][k:]):0{(dlen + 3) // 4}X}"])
    leftover = len(bits["mosi"]) % dlen
    if leftover:
        log(f"warning: CS frame {frame} ({a / srate * 1e6:.3f}-{e / srate * 1e6:.3f} us) "
            f"has {leftover} trailing bits")

f = open(args.output, "w", newline="") if args.output else sys.stdout
w = csv.writer(f)
w.writerow(["frame", "time_s", "mosi", "miso"])
w.writerows(rows)
