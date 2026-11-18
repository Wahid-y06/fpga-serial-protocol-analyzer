"""System-level cocotb tests for analyzer_top.

Traffic is driven on the probe pins, the capture is dumped over the host
UART, received by a UART model and decoded with host/la_decode.py - the
same path a real capture takes to the PC. Built with AW=4 (16-entry buffer)
so ring-buffer wrap-around is quick to exercise.
"""
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

from bfm import I2cMaster, delay, spi_transfer, uart_receiver, uart_send
from la_decode import parse

CLK_NS = 20
UART_CPB = 25                   # probed UART: 2 Mbaud
UART_BIT = UART_CPB * CLK_NS
HOST_CPB = 8                    # readout UART (fast, to keep simulations short)
SPI_HALF = 100
I2C_Q = 250                     # quarter period -> 1 MHz SCL (Fast-mode Plus)
DEPTH = 16

TRIG_FRAMING, TRIG_PARITY, TRIG_NACK, TRIG_SPI_PARTIAL, TRIG_MATCH, TRIG_ANY = (
    1 << i for i in range(6))


class Analyzer:
    def __init__(self, dut):
        self.dut = dut
        self.rx_bytes = []

    async def start(self, trig_mask, post_count=0, match=0, prescale=0, spi_mode=0,
                    parity_en=0):
        d = self.dut
        cocotb.start_soon(Clock(d.clk, CLK_NS, unit="ns").start())
        for pin, idle in (("uart_rx_pin", 1), ("spi_sclk_pin", spi_mode >> 1),
                          ("spi_mosi_pin", 0), ("spi_miso_pin", 0), ("spi_cs_n_pin", 1),
                          ("i2c_scl_pin", 1), ("i2c_sda_pin", 1)):
            getattr(d, pin).value = idle
        d.uart_cpb.value = UART_CPB
        d.uart_parity_en.value = parity_en
        d.uart_parity_odd.value = 0
        d.spi_mode.value = spi_mode
        d.trig_mask.value = trig_mask
        d.match_value.value = match
        d.post_count.value = post_count
        d.ts_prescale.value = prescale
        d.host_cpb.value = HOST_CPB
        d.arm.value = 0
        d.dump.value = 0
        d.rst.value = 1
        await ClockCycles(d.clk, 4)
        d.rst.value = 0
        await ClockCycles(d.clk, 4)
        cocotb.start_soon(uart_receiver(d.host_tx, HOST_CPB * CLK_NS, self.rx_bytes))
        await self.pulse("arm")
        assert int(d.armed.value)
        await delay(3)

    async def pulse(self, name):
        """One-clock pulse; returns once the DUT has registered it."""
        sig = getattr(self.dut, name)
        await RisingEdge(self.dut.clk)
        sig.value = 1
        await RisingEdge(self.dut.clk)       # DUT samples the pulse here
        sig.value = 0
        await RisingEdge(self.dut.clk)       # registered outputs now visible

    async def dump(self):
        d = self.dut
        await ClockCycles(d.clk, 20)
        assert int(d.frozen.value), "analyzer did not freeze"
        self.rx_bytes.clear()
        await self.pulse("dump")
        while int(d.dumping.value):
            await RisingEdge(d.clk)
        await delay(12 * HOST_CPB * CLK_NS)
        assert not int(d.dropped.value)
        return parse(bytes(self.rx_bytes))


def describe(cap):
    return [(r.protocol, r.describe()) for r in cap.records]


@cocotb.test()
async def mixed_traffic_trigger_on_i2c_nack(dut):
    la = Analyzer(dut)
    await la.start(TRIG_NACK, post_count=2)
    d = dut
    for ch in b"Hi":
        await uart_send(d.uart_rx_pin, ch, UART_BIT)
    await spi_transfer(d.spi_sclk_pin, d.spi_mosi_pin, d.spi_miso_pin, d.spi_cs_n_pin,
                       [0x9F], [0xEF], 0, SPI_HALF)
    m = I2cMaster(d.i2c_scl_pin, d.i2c_sda_pin, I2C_Q)
    await m.start()
    await m.write_byte(0x50 << 1)
    await m.write_byte(0x12, ack=False)          # trigger
    await m.stop()                               # post-trigger 1
    for ch in b"XYZ":                            # only 'X' fits in post_count=2
        await uart_send(d.uart_rx_pin, ch, UART_BIT)
    assert int(d.frozen.value)

    cap = await la.dump()
    # keep the raw stream so it can be inspected with host/la_decode.py
    Path("sample_capture.bin").write_bytes(bytes(la.rx_bytes))
    assert describe(cap) == [
        ("UART", "0x48 'H'"), ("UART", "0x69 'i'"),
        ("SPI", "MOSI=0x9F MISO=0xEF"),
        ("I2C", "START"), ("I2C", "ADDR 0x50 W  ACK"),
        ("I2C", "DATA 0x12  NACK"),
        ("I2C", "STOP"), ("UART", "0x58 'X'"),
    ]
    assert cap.trigger_index == 5
    ts = [r.timestamp for r in cap.records]
    assert ts == sorted(ts)


@cocotb.test()
async def concurrent_traffic_on_all_buses(dut):
    """All three buses active at the same time: nothing lost, order per bus kept."""
    la = Analyzer(dut)
    await la.start(TRIG_MATCH, match=0xEE, post_count=0)
    d = dut

    async def uart_traffic():
        for ch in b"ABCD":
            await uart_send(d.uart_rx_pin, ch, UART_BIT)

    async def spi_traffic():
        await spi_transfer(d.spi_sclk_pin, d.spi_mosi_pin, d.spi_miso_pin, d.spi_cs_n_pin,
                           [0x01, 0x02, 0x03], [0x10, 0x20, 0x30], 0, SPI_HALF)

    async def i2c_traffic():
        m = I2cMaster(d.i2c_scl_pin, d.i2c_sda_pin, I2C_Q)
        await m.start()
        await m.write_byte(0x20 << 1)
        await m.write_byte(0x7F)
        await m.stop()

    tasks = [cocotb.start_soon(c()) for c in (uart_traffic, spi_traffic, i2c_traffic)]
    for t in tasks:
        await t
    await uart_send(d.uart_rx_pin, 0xEE, UART_BIT)   # trigger: freeze now

    cap = await la.dump()
    by_proto = {}
    for proto, text in describe(cap):
        by_proto.setdefault(proto, []).append(text)
    assert by_proto["UART"] == ["0x41 'A'", "0x42 'B'", "0x43 'C'", "0x44 'D'", "0xEE '.'"]
    assert by_proto["SPI"] == ["MOSI=0x01 MISO=0x10", "MOSI=0x02 MISO=0x20",
                               "MOSI=0x03 MISO=0x30"]
    assert by_proto["I2C"] == ["START", "ADDR 0x20 W  ACK", "DATA 0x7F  ACK", "STOP"]
    assert cap.trigger_index == len(cap.records) - 1
    ts = [r.timestamp for r in cap.records]
    assert ts == sorted(ts)


@cocotb.test()
async def framing_error_trigger_keeps_pretrigger_history(dut):
    la = Analyzer(dut)
    await la.start(TRIG_FRAMING, post_count=0)
    d = dut
    for b in range(5):
        await uart_send(d.uart_rx_pin, 0x30 + b, UART_BIT)
    await uart_send(d.uart_rx_pin, 0x55, UART_BIT, stop_bit=0)
    await delay(2 * UART_BIT)
    cap = await la.dump()
    assert [r.d0 for r in cap.records] == [0x30, 0x31, 0x32, 0x33, 0x34, 0x55]
    assert cap.trigger_index == 5
    assert cap.trigger.flag_a and "FRAMING ERROR" in cap.trigger.describe()


@cocotb.test()
async def ring_buffer_wraps_and_keeps_newest(dut):
    la = Analyzer(dut)
    await la.start(TRIG_MATCH, match=0xFF, post_count=0)
    d = dut
    payload = list(range(1, 21)) + [0xFF]           # 21 events into a 16-entry buffer
    for b in payload:
        await uart_send(d.uart_rx_pin, b, UART_BIT)
    cap = await la.dump()
    assert len(cap.records) == DEPTH
    assert [r.d0 for r in cap.records] == payload[-DEPTH:]
    assert cap.trigger_index == DEPTH - 1


@cocotb.test()
async def post_count_clamped_to_buffer_depth(dut):
    """post_count larger than the buffer must not overwrite the trigger record."""
    la = Analyzer(dut)
    await la.start(TRIG_MATCH, match=0xAB, post_count=1000)
    d = dut
    for b in [0x01, 0xAB] + list(range(0x10, 0x10 + 20)):
        await uart_send(d.uart_rx_pin, b, UART_BIT)
        if int(d.frozen.value):
            break
    cap = await la.dump()
    assert len(cap.records) == DEPTH
    assert cap.trigger_index == 0
    assert cap.trigger.d0 == 0xAB


@cocotb.test()
async def spi_incomplete_word_trigger(dut):
    la = Analyzer(dut)
    await la.start(TRIG_SPI_PARTIAL, post_count=0, spi_mode=3)
    d = dut
    await spi_transfer(d.spi_sclk_pin, d.spi_mosi_pin, d.spi_miso_pin, d.spi_cs_n_pin,
                       [0x3C, 0xF0], [0xC3, 0x0F], 3, SPI_HALF, n_bits=12)
    cap = await la.dump()
    assert describe(cap) == [("SPI", "MOSI=0x3C MISO=0xC3"),
                             ("SPI", "INCOMPLETE (4 bits) MOSI=0x0F MISO=0x00")]
    assert cap.trigger_index == 1


@cocotb.test()
async def parity_error_trigger(dut):
    la = Analyzer(dut)
    await la.start(TRIG_PARITY, post_count=0, parity_en=1)
    d = dut
    await uart_send(d.uart_rx_pin, 0x41, UART_BIT, parity="even")
    await uart_send(d.uart_rx_pin, 0x41, UART_BIT, parity="even", force_parity=1)
    await delay(2 * UART_BIT)
    cap = await la.dump()
    assert describe(cap) == [("UART", "0x41 'A'"), ("UART", "0x41 'A'  PARITY ERROR")]


@cocotb.test()
async def timestamps_measure_real_time(dut):
    """Gap between two UART bytes sent 50 us apart, with prescale 49 (1 tick = 1 us)."""
    la = Analyzer(dut)
    await la.start(TRIG_MATCH, match=0x02, post_count=0, prescale=49)
    d = dut
    await uart_send(d.uart_rx_pin, 0x01, UART_BIT)
    await delay(50_000 - 10 * UART_BIT)
    await uart_send(d.uart_rx_pin, 0x02, UART_BIT)
    cap = await la.dump()
    gap = cap.records[1].timestamp - cap.records[0].timestamp
    assert abs(gap - 50) <= 1, gap


@cocotb.test()
async def rearm_clears_previous_capture(dut):
    la = Analyzer(dut)
    await la.start(TRIG_ANY, post_count=0)
    d = dut
    await uart_send(d.uart_rx_pin, 0x11, UART_BIT)
    first = await la.dump()
    assert [r.d0 for r in first.records] == [0x11]
    await la.pulse("arm")
    await ClockCycles(d.clk, 2)
    assert int(d.armed.value) and not int(d.triggered.value)
    assert int(d.event_count.value) == 0
    await uart_send(d.uart_rx_pin, 0x22, UART_BIT)
    second = await la.dump()
    assert [r.d0 for r in second.records] == [0x22]
