"""cocotb tests for i2c_decoder."""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer

from bfm import I2cMaster

CLK_NS = 20
Q_NS = 625                  # quarter period -> 400 kHz fast mode
START, RESTART, STOP, ADDR, DATA = range(5)


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())
    dut.scl.value = 1
    dut.sda_in.value = 1
    dut.rst.value = 1
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 4)
    await Timer(3, unit="ns")
    events = []

    async def monitor():
        while True:
            await RisingEdge(dut.clk)
            if int(dut.valid.value):
                events.append((int(dut.kind.value), int(dut.data.value), int(dut.nack.value)))

    cocotb.start_soon(monitor())
    return events


@cocotb.test()
async def write_transaction(dut):
    events = await setup(dut)
    m = I2cMaster(dut.scl, dut.sda_in, Q_NS)
    await m.start()
    await m.write_byte(0x50 << 1)            # address 0x50, write
    await m.write_byte(0x12)
    await m.write_byte(0x34, ack=False)       # receiver NACKs the last byte
    await m.stop()
    await ClockCycles(dut.clk, 10)
    assert events == [(START, 0, 0), (ADDR, 0xA0, 0), (DATA, 0x12, 0),
                      (DATA, 0x34, 1), (STOP, 0, 0)]


@cocotb.test()
async def repeated_start_read(dut):
    events = await setup(dut)
    m = I2cMaster(dut.scl, dut.sda_in, Q_NS)
    await m.start()
    await m.write_byte(0x68 << 1)
    await m.write_byte(0x75)                  # register pointer
    await m.restart()
    await m.write_byte((0x68 << 1) | 1)       # read
    await m.write_byte(0x71, ack=False)       # master NACKs the final read byte
    await m.stop()
    await ClockCycles(dut.clk, 10)
    assert events == [(START, 0, 0), (ADDR, 0xD0, 0), (DATA, 0x75, 0),
                      (RESTART, 0, 0), (ADDR, 0xD1, 0), (DATA, 0x71, 1), (STOP, 0, 0)]


@cocotb.test()
async def address_not_acknowledged(dut):
    events = await setup(dut)
    m = I2cMaster(dut.scl, dut.sda_in, Q_NS)
    await m.start()
    await m.write_byte(0x3C << 1, ack=False)  # no device at 0x3C
    await m.stop()
    await ClockCycles(dut.clk, 10)
    assert events == [(START, 0, 0), (ADDR, 0x78, 1), (STOP, 0, 0)]


@cocotb.test()
async def sda_skew_does_not_fake_start_stop(dut):
    """Zero hold time plus <1 clock of SCL/SDA skew: SDA changes 15 ns before
    SCL is seen falling. Without the SDA delay in the decoder, data bits
    would be misread as START/STOP conditions."""
    events = await setup(dut)
    m = I2cMaster(dut.scl, dut.sda_in, Q_NS, sda_lead_ns=15)
    await m.start()
    await m.write_byte(0xA6)
    await m.write_byte(0x5B)
    await m.write_byte(0x0F, ack=False)
    await m.stop()
    await ClockCycles(dut.clk, 10)
    assert events == [(START, 0, 0), (ADDR, 0xA6, 0), (DATA, 0x5B, 0),
                      (DATA, 0x0F, 1), (STOP, 0, 0)]


@cocotb.test()
async def standard_mode_100khz(dut):
    events = await setup(dut)
    m = I2cMaster(dut.scl, dut.sda_in, 2500)
    await m.start()
    await m.write_byte(0x90)
    await m.stop()
    await ClockCycles(dut.clk, 10)
    assert events == [(START, 0, 0), (ADDR, 0x90, 0), (STOP, 0, 0)]
