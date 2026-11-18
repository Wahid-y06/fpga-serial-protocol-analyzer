"""Host-side decoder for captures streamed from the FPGA analyzer.

    python host/la_decode.py capture.bin
    python host/la_decode.py --port COM5 --baud 115200      # needs pyserial
    python host/la_decode.py capture.bin --csv capture.csv

Stream format (see rtl/capture_buffer.v):
    header : 'L' 'A' count(u16 BE) trig_idx(u16 BE)
    record : 0xA5 timestamp(u32 BE) tag d0 d1
"""
from __future__ import annotations

import argparse
import csv
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

HEADER = b"LA"
SYNC = 0xA5
RECORD_LEN = 8
PROTOCOLS = {0: "UART", 1: "SPI", 2: "I2C"}
I2C_KINDS = {0: "START", 1: "RESTART", 2: "STOP", 3: "ADDR", 4: "DATA"}


class CaptureError(ValueError):
    pass


@dataclass(frozen=True)
class Record:
    timestamp: int
    proto: int
    kind: int
    flag_a: bool
    flag_b: bool
    d0: int
    d1: int

    @classmethod
    def from_bytes(cls, raw: bytes) -> "Record":
        sync, ts, tag, d0, d1 = struct.unpack(">BIBBB", raw)
        if sync != SYNC:
            raise CaptureError(f"bad record sync byte 0x{sync:02X}")
        return cls(ts, tag >> 6, (tag >> 3) & 0x7, bool(tag & 0x4), bool(tag & 0x2), d0, d1)

    @property
    def protocol(self) -> str:
        return PROTOCOLS.get(self.proto, f"?{self.proto}")

    @property
    def is_error(self) -> bool:
        return self.flag_a or (self.proto == 0 and self.flag_b)

    def describe(self) -> str:
        if self.proto == 0:
            errs = [n for n, f in (("FRAMING ERROR", self.flag_a),
                                   ("PARITY ERROR", self.flag_b)) if f]
            ch = chr(self.d0) if 32 <= self.d0 < 127 else "."
            return f"0x{self.d0:02X} '{ch}'" + ("  " + ", ".join(errs) if errs else "")
        if self.proto == 1:
            if self.flag_a:
                return (f"INCOMPLETE ({self.kind} bits) MOSI=0x{self.d0:02X} "
                        f"MISO=0x{self.d1:02X}")
            return f"MOSI=0x{self.d0:02X} MISO=0x{self.d1:02X}"
        if self.proto == 2:
            kind = I2C_KINDS.get(self.kind, f"?{self.kind}")
            if self.kind == 3:
                rw = "R" if self.d0 & 1 else "W"
                return f"ADDR 0x{self.d0 >> 1:02X} {rw}  " + ("NACK" if self.flag_a else "ACK")
            if self.kind == 4:
                return f"DATA 0x{self.d0:02X}  " + ("NACK" if self.flag_a else "ACK")
            return kind
        return f"tag?{self.proto} d0=0x{self.d0:02X} d1=0x{self.d1:02X}"


@dataclass(frozen=True)
class Capture:
    records: list[Record]
    trigger_index: int

    @property
    def trigger(self) -> Record | None:
        if 0 <= self.trigger_index < len(self.records):
            return self.records[self.trigger_index]
        return None


def parse(data: bytes) -> Capture:
    start = data.find(HEADER)
    if start < 0:
        raise CaptureError("capture header 'LA' not found")
    if len(data) < start + 6:
        raise CaptureError("truncated header")
    count, trig = struct.unpack(">HH", data[start + 2:start + 6])
    body = data[start + 6:]
    if len(body) < count * RECORD_LEN:
        raise CaptureError(f"expected {count} records, got {len(body) // RECORD_LEN}")
    records = [Record.from_bytes(body[i * RECORD_LEN:(i + 1) * RECORD_LEN])
               for i in range(count)]
    return Capture(records, trig)


def format_table(cap: Capture, tick_s: float) -> str:
    lines = [f"{'#':>4}  {'time (us)':>12}  {'proto':<5} event"]
    t0 = cap.records[0].timestamp if cap.records else 0
    for i, r in enumerate(cap.records):
        mark = ">>" if i == cap.trigger_index else "  "
        t_us = (r.timestamp - t0) * tick_s * 1e6
        lines.append(f"{mark}{i:>2}  {t_us:>12.2f}  {r.protocol:<5} {r.describe()}")
    lines.append(f"{len(cap.records)} records, trigger at #{cap.trigger_index}")
    return "\n".join(lines)


def write_csv(cap: Capture, path: Path, tick_s: float) -> None:
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["index", "timestamp", "time_us", "protocol", "event", "error", "trigger"])
        t0 = cap.records[0].timestamp if cap.records else 0
        for i, r in enumerate(cap.records):
            w.writerow([i, r.timestamp, f"{(r.timestamp - t0) * tick_s * 1e6:.3f}",
                        r.protocol, r.describe(), int(r.is_error), int(i == cap.trigger_index)])


def read_serial(port: str, baud: int, timeout: float) -> bytes:
    try:
        import serial  # type: ignore
    except ImportError:
        sys.exit("pyserial is required for --port (pip install pyserial)")
    with serial.Serial(port, baud, timeout=timeout) as ser:
        data = bytearray()
        while chunk := ser.read(4096):
            data += chunk
        return bytes(data)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Decode an FPGA analyzer capture")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("file", nargs="?", type=Path, help="raw capture file")
    src.add_argument("--port", help="serial port to read the dump from")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=2.0)
    ap.add_argument("--clk", type=float, default=50e6, help="FPGA clock in Hz")
    ap.add_argument("--prescale", type=int, default=0, help="ts_prescale setting")
    ap.add_argument("--csv", type=Path)
    args = ap.parse_args(argv)

    data = read_serial(args.port, args.baud, args.timeout) if args.port else args.file.read_bytes()
    cap = parse(data)
    tick = (args.prescale + 1) / args.clk
    print(format_table(cap, tick))
    if args.csv:
        write_csv(cap, args.csv, tick)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
