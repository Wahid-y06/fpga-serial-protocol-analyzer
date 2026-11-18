"""cocotb tests for spi_decoder."""
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer

from bfm import spi_transfer

CLK_NS = 20
HALF_NS = 100               # 5 MHz SCLK


async def setup(dut, mode):
    cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())
    dut.mode.value = mode
    dut.sclk.value = mode >> 1
    dut.mosi.value = 0
    dut.miso.value = 0
    dut.cs_n.value = 1
    dut.rst.value = 1
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)
    await Timer(3, unit="ns")
    events = []

    async def monitor():
        while True:
            await RisingEdge(dut.clk)
            if int(dut.valid.value):
                events.append(("byte", int(dut.mosi_byte.value), int(dut.miso_byte.value)))
            if int(dut.incomplete.value):
                events.append(("partial", int(dut.partial_bits.value),
                               int(dut.mosi_byte.value), int(dut.miso_byte.value)))

    cocotb.start_soon(monitor())
    return events


def pins(dut):
    return dut.sclk, dut.mosi, dut.miso, dut.cs_n


@cocotb.test()
async def all_four_modes(dut):
    rng = random.Random(7)
    for mode in range(4):
        events = await setup(dut, mode)
        mo = [rng.randrange(256) for _ in range(6)]
        mi = [rng.randrange(256) for _ in range(6)]
        await spi_transfer(*pins(dut), mo, mi, mode, HALF_NS)
        await ClockCycles(dut.clk, 5)
        assert events == [("byte", a, b) for a, b in zip(mo, mi)], f"mode {mode}"
        dut._log.info(f"mode {mode}: {len(events)} bytes OK")


@cocotb.test()
async def chip_select_released_mid_byte(dut):
    events = await setup(dut, 0)
    await spi_transfer(*pins(dut), [0xF0, 0xAA], [0x11, 0x22], 0, HALF_NS, n_bits=13)
    await ClockCycles(dut.clk, 5)
    # first byte complete, then 5 bits of 0xAA (10101) and 0x22 (00100)
    assert events == [("byte", 0xF0, 0x11), ("partial", 5, 0b10101, 0b00100)]


@cocotb.test()
async def separate_transactions(dut):
    events = await setup(dut, 3)
    await spi_transfer(*pins(dut), [0x9F], [0x00], 3, HALF_NS)
    await spi_transfer(*pins(dut), [0x00, 0x00], [0xEF, 0x40], 3, HALF_NS)
    await ClockCycles(dut.clk, 5)
    assert events == [("byte", 0x9F, 0x00), ("byte", 0x00, 0xEF), ("byte", 0x00, 0x40)]


@cocotb.test()
async def fastest_supported_sclk(dut):
    """f_sclk just under f_clk/4 (12.5 MHz -> use 10 MHz)."""
    events = await setup(dut, 0)
    await spi_transfer(*pins(dut), [0xC9, 0x36], [0x5A, 0xA5], 0, half_ns=50)
    await ClockCycles(dut.clk, 5)
    assert events == [("byte", 0xC9, 0x5A), ("byte", 0x36, 0xA5)]
