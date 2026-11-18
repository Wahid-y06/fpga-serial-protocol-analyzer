"""cocotb tests for uart_rx_decoder."""
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer

from bfm import delay, uart_send

CLK_NS = 20                 # 50 MHz


async def setup(dut, cpb=50, parity_en=0, parity_odd=0):
    cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())
    dut.rx.value = 1
    dut.clks_per_bit.value = cpb
    dut.parity_en.value = parity_en
    dut.parity_odd.value = parity_odd
    dut.rst.value = 1
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)
    await Timer(3, unit="ns")          # drive stimulus away from clock edges
    events = []

    async def monitor():
        while True:
            await RisingEdge(dut.clk)
            if int(dut.valid.value):
                events.append((int(dut.data.value), int(dut.frame_err.value),
                               int(dut.parity_err.value)))

    cocotb.start_soon(monitor())
    return events, cpb * CLK_NS


@cocotb.test()
async def decodes_bytes(dut):
    events, bit_ns = await setup(dut)
    payload = [0x55, 0x00, 0xFF, 0xA3, 0x01, 0x80]
    for b in payload:
        await uart_send(dut.rx, b, bit_ns)
    await Timer(2 * bit_ns, unit="ns")
    assert events == [(b, 0, 0) for b in payload]


@cocotb.test()
async def random_bytes_back_to_back(dut):
    events, bit_ns = await setup(dut, cpb=16)
    rng = random.Random(1)
    payload = [rng.randrange(256) for _ in range(40)]
    for b in payload:
        await uart_send(dut.rx, b, bit_ns)
    await Timer(2 * bit_ns, unit="ns")
    assert [e[0] for e in events] == payload
    assert all(e[1:] == (0, 0) for e in events)


@cocotb.test()
async def parity_checking(dut):
    events, bit_ns = await setup(dut, parity_en=1, parity_odd=0)
    await uart_send(dut.rx, 0x37, bit_ns, parity="even")
    await uart_send(dut.rx, 0x37, bit_ns, parity="even", force_parity=0)  # wrong: 0x37 has 5 ones
    dut.parity_odd.value = 1
    await uart_send(dut.rx, 0x37, bit_ns, parity="odd")
    await Timer(2 * bit_ns, unit="ns")
    assert events == [(0x37, 0, 0), (0x37, 0, 1), (0x37, 0, 0)]


@cocotb.test()
async def framing_error_then_recovers(dut):
    events, bit_ns = await setup(dut)
    await uart_send(dut.rx, 0x5A, bit_ns, stop_bit=0)
    await Timer(3 * bit_ns, unit="ns")
    await uart_send(dut.rx, 0xC3, bit_ns)
    await Timer(2 * bit_ns, unit="ns")
    assert events == [(0x5A, 1, 0), (0xC3, 0, 0)]


@cocotb.test()
async def break_condition_reports_once(dut):
    events, bit_ns = await setup(dut)
    dut.rx.value = 0
    await Timer(30 * bit_ns, unit="ns")
    dut.rx.value = 1
    await Timer(2 * bit_ns, unit="ns")
    await uart_send(dut.rx, 0x42, bit_ns)
    await Timer(2 * bit_ns, unit="ns")
    assert events == [(0x00, 1, 0), (0x42, 0, 0)]


@cocotb.test()
async def short_glitch_ignored(dut):
    events, bit_ns = await setup(dut)
    dut.rx.value = 0
    await Timer(bit_ns // 4, unit="ns")
    dut.rx.value = 1
    await Timer(12 * bit_ns, unit="ns")
    assert events == []


@cocotb.test()
async def tolerates_baud_mismatch(dut):
    events, bit_ns = await setup(dut, cpb=100)
    for err in (-0.03, +0.03):
        await uart_send(dut.rx, 0xB6, bit_ns * (1 + err))
        await Timer(2 * bit_ns, unit="ns")
    assert events == [(0xB6, 0, 0), (0xB6, 0, 0)]


@cocotb.test()
async def standard_115200_baud(dut):
    cpb = round(50_000_000 / 115_200)          # 434
    events, _ = await setup(dut, cpb=cpb)
    bit_ns = 1e9 / 115_200
    await uart_send(dut.rx, 0x7E, bit_ns)
    await delay(2 * bit_ns)
    assert events == [(0x7E, 0, 0)]
