"""Tests for the host-side capture decoder and an RTL lint check."""
import shutil
import struct
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "host"))

from la_decode import CaptureError, Record, format_table, main, parse  # noqa: E402


def record(ts, proto, kind=0, a=0, b=0, d0=0, d1=0):
    tag = (proto << 6) | (kind << 3) | (a << 2) | (b << 1)
    return struct.pack(">BIBBB", 0xA5, ts, tag, d0, d1)


def capture(records, trig):
    return b"LA" + struct.pack(">HH", len(records), trig) + b"".join(records)


def test_parse_round_trip_and_descriptions():
    raw = capture([
        record(100, 0, d0=ord("H")),
        record(150, 0, a=1, d0=0x55),
        record(200, 1, d0=0x9F, d1=0xEF),
        record(250, 1, kind=5, a=1, d0=0x15, d1=0x02),
        record(300, 2, kind=0),
        record(310, 2, kind=3, d0=0xA1),
        record(320, 2, kind=4, a=1, d0=0x12),
        record(330, 2, kind=2),
    ], trig=6)
    cap = parse(b"\x00\xff" + raw)                  # leading garbage is skipped
    assert [r.describe() for r in cap.records] == [
        "0x48 'H'", "0x55 'U'  FRAMING ERROR", "MOSI=0x9F MISO=0xEF",
        "INCOMPLETE (5 bits) MOSI=0x15 MISO=0x02", "START", "ADDR 0x50 R  ACK",
        "DATA 0x12  NACK", "STOP"]
    assert cap.trigger.describe() == "DATA 0x12  NACK"
    assert [r.is_error for r in cap.records] == [0, 1, 0, 1, 0, 0, 1, 0]


def test_parity_error_flag():
    r = Record.from_bytes(record(0, 0, b=1, d0=0x41))
    assert r.describe() == "0x41 'A'  PARITY ERROR" and r.is_error


def test_table_marks_trigger_and_converts_time():
    cap = parse(capture([record(0, 0, d0=0x31), record(50, 0, d0=0x32)], trig=1))
    table = format_table(cap, tick_s=1e-6)
    assert ">> 1" in table and "50.00" in table


@pytest.mark.parametrize("data, msg", [
    (b"nothing here", "header"),
    (b"LA\x00", "truncated"),
    (capture([record(0, 0)], 0)[:-3], "expected 1 records"),
    (capture([b"\x00" + record(0, 0)[1:]], 0), "sync"),
])
def test_bad_captures_rejected(data, msg):
    with pytest.raises(CaptureError, match=msg):
        parse(data)


def test_cli_with_csv(tmp_path, capsys):
    f = tmp_path / "cap.bin"
    f.write_bytes(capture([record(0, 2, kind=0), record(10, 2, kind=2)], trig=0))
    out_csv = tmp_path / "cap.csv"
    assert main([str(f), "--csv", str(out_csv), "--prescale", "49"]) == 0
    assert "START" in capsys.readouterr().out
    assert out_csv.read_text().splitlines()[2].startswith("1,10,10.000,I2C,STOP")


@pytest.mark.skipif(shutil.which("iverilog") is None and not Path("C:/iverilog/bin").exists(),
                    reason="Icarus Verilog not installed")
def test_board_wrapper_compiles_without_warnings(tmp_path):
    iverilog = shutil.which("iverilog") or "C:/iverilog/bin/iverilog.exe"
    sources = sorted((ROOT / "rtl").glob("*.v")) + [ROOT / "board" / "de10lite_top.v"]
    result = subprocess.run([iverilog, "-Wall", "-g2005", "-s", "de10lite_top",
                             "-o", str(tmp_path / "a.out"), *map(str, sources)],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert "warning" not in result.stderr.lower(), result.stderr
