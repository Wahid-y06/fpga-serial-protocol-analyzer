"""cocotb tests for uart_tx."""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

from bfm import delay, uart_receiver

CLK_NS = 20


async def setup(dut, cpb):
    cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())
    dut.clks_per_bit.value = cpb
    dut.start.value = 0
    dut.data.value = 0
    dut.rst.value = 1
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)
    received = []
    cocotb.start_soon(uart_receiver(dut.tx, cpb * CLK_NS, received))
    return received


async def send(dut, byte):
    while not int(dut.ready.value):
        await RisingEdge(dut.clk)
    dut.data.value = byte
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0
    await RisingEdge(dut.clk)
    assert not int(dut.ready.value), "transmitter should be busy after start"


@cocotb.test()
async def sends_bytes_back_to_back(dut):
    cpb = 10
    received = await setup(dut, cpb)
    payload = [0x00, 0xFF, 0x55, 0xAA, 0x4C, 0x41, 0x81]
    for b in payload:
        await send(dut, b)
    while not int(dut.ready.value):
        await RisingEdge(dut.clk)
    await delay(2 * cpb * CLK_NS)
    assert received == payload


@cocotb.test()
async def frame_timing_is_ten_bits(dut):
    cpb = 20
    await setup(dut, cpb)
    await send(dut, 0x3C)
    cycles = 1                       # already one clock into the frame
    while not int(dut.ready.value):
        await RisingEdge(dut.clk)
        cycles += 1
    assert abs(cycles - 10 * cpb) <= 2, cycles


@cocotb.test()
async def idle_line_is_high(dut):
    await setup(dut, 10)
    for _ in range(50):
        await RisingEdge(dut.clk)
        assert int(dut.tx.value) == 1
