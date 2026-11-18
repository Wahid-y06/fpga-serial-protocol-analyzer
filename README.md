# FPGA Serial Protocol Analyzer

A Verilog logic analyzer that passively listens to **UART, SPI and I2C**
buses at the same time, decodes the traffic in hardware, timestamps every
event and stores it in a triggerable ring buffer. When a trigger condition
(framing error, missing I2C ACK, data match...) fires, the buffer freezes
with the history leading up to the problem, and the capture is streamed to
a PC over UART and decoded by a Python tool.

Target board: Terasic **DE10-Lite** (Intel MAX 10, 50 MHz). Verified with
**cocotb + Icarus Verilog** simulation (29 testbench tests + 9 host tests).

```
   #     time (us)  proto event
   0          0.00  UART  0x48 'H'
   1          5.00  UART  0x69 'i'
   2          6.86  SPI   MOSI=0x9F MISO=0xEF
   3          7.44  I2C   START
   4         16.16  I2C   ADDR 0x50 W  ACK
>> 5         25.16  I2C   DATA 0x12  NACK
   6         26.44  I2C   STOP
   7         31.40  UART  0x58 'X'
8 records, trigger at #5
```
*A real capture from simulation ([`docs/sample_capture.bin`](docs/sample_capture.bin)),
decoded with `host/la_decode.py`: triggered on the I2C NACK, with
pre-trigger history and two post-trigger events.*

## Architecture

```
 probe pins          2-flop             protocol decoders        arbiter +          capture buffer          host link
                  synchronizers                                  timestamps
 UART RX  ──────►  sync2  ──────►  uart_rx_decoder ──┐
 SCLK/MOSI/       ┌───────┐                          │      ┌────────────┐    ┌──────────────────┐    ┌─────────┐
 MISO/CS_n ─────► │ sync2 │ ─────► spi_decoder ──────┼────► │ 1 pending  │──► │ 2^AW x 56-bit    │──► │ uart_tx │──► PC
 SCL/SDA  ──────► └───────┘ ─────► i2c_decoder ──────┘      │ per source │    │ ring + trigger   │    └─────────┘
                                                            └────────────┘    └──────────────────┘
                                                                  ▲ 32-bit timestamp (prescaled)
```

| Module | What it does |
|---|---|
| [`sync2.v`](rtl/sync2.v) | Two-flop synchronizer for every asynchronous probe input |
| [`uart_rx_decoder.v`](rtl/uart_rx_decoder.v) | Runtime-configurable baud, mid-bit sampling, start-bit glitch rejection, optional even/odd parity, framing-error and break detection |
| [`spi_decoder.v`](rtl/spi_decoder.v) | All four SPI modes, MOSI and MISO captured together, reports words cut short by CS release (with bit count) |
| [`i2c_decoder.v`](rtl/i2c_decoder.v) | START / repeated START / STOP, address vs data bytes, ACK/NACK; SDA delayed 2 clocks so SCL/SDA skew can't fake a START/STOP |
| [`capture_buffer.v`](rtl/capture_buffer.v) | Circular buffer (written in the synchronous-RAM style that maps to block RAM), 6 trigger conditions, post-trigger count, freeze, UART readout |
| [`analyzer_top.v`](rtl/analyzer_top.v) | Wires it together; per-source event holding registers so simultaneous events on different buses are never lost |
| [`uart_tx.v`](rtl/uart_tx.v) | 8N1 transmitter for the PC link |
| [`board/de10lite_top.v`](board/de10lite_top.v) | DE10-Lite wrapper: GPIO probes, switches for config, buttons for arm/dump, LEDs for status |

## Triggering

The analyzer records continuously while armed, keeping the newest 2^AW
events. Trigger conditions (any combination via `trig_mask`):

| Bit | Condition |
|---|---|
| 0 | UART framing error (stop bit low) |
| 1 | UART parity error |
| 2 | I2C NACK on an address or data byte (missing acknowledgment) |
| 3 | SPI word cut short by chip select |
| 4 | Any decoded data byte equals `match_value` |
| 5 | Any event |

After the trigger, `post_count` further events are recorded (clamped so the
trigger record is never overwritten), then the buffer freezes until it is
dumped or re-armed.

## Capture format

Each event is a 56-bit record: 32-bit timestamp, tag byte (protocol, event
kind, error flags) and two data bytes (e.g. MOSI and MISO). Over the wire:

```
header : 'L' 'A' count(u16) trigger_index(u16)
record : 0xA5 timestamp(u32) tag d0 d1        (oldest first, big-endian)
```

Timestamps tick every `ts_prescale + 1` clocks (20 ns to 1.3 ms per tick),
so a 32-bit counter covers from 86 seconds up to 65 days.

## Verification

Each module has its own cocotb testbench, driven by Python bus-functional
models of UART, SPI and I2C devices ([`tb/bfm.py`](tb/bfm.py)):

| Testbench | Tests | Covers |
|---|---|---|
| `cocotb_uart` | 8 | Random back-to-back bytes, parity, framing error recovery, break, glitch rejection, ±3% baud mismatch, 115200 baud |
| `cocotb_spi` | 4 | Random data in all 4 modes, CS released mid-word, multiple transactions, 10 MHz SCLK |
| `cocotb_i2c` | 5 | Write, repeated-start read, address NACK, SCL/SDA skew, 100 kHz and 1 MHz |
| `cocotb_uart_tx` | 3 | Back-to-back bytes, 10-bit frame timing, idle level |
| `cocotb_top` | 9 | All three buses at once, every trigger type, pre-trigger history, ring wrap-around, post-count clamp, timestamp accuracy, re-arm |

The system tests read the capture back through the UART link and decode it
with the same host script a user would run, so the whole path is checked.

To make sure the tests actually catch bugs, faults were deliberately
injected: an off-by-one in the post-trigger counter fails 2 system tests,
and removing the I2C SDA delay fails the SCL/SDA-skew test.

Bugs found by the testbenches during development:
- **SPI:** when CS was released mid-word, bits left over from the previous
  word leaked into the partial value. Fixed by clearing the shift registers
  at each word boundary.
- **Testbench:** sub-picosecond rounding of baud-rate periods (115200 baud
  = 8680.555... ns) is rejected by the simulator; delays are now rounded to
  whole picoseconds.

## Running

Requirements: Python 3.10+, [Icarus Verilog](https://bleyer.org/icarus/) 12
on `PATH`, and:

```bash
pip install -r requirements.txt
python -m pytest              # all simulations + host tests
WAVES=1 python -m pytest      # also write waveforms to sim_build/ for GTKWave
```

Decode a capture:

```bash
python host/la_decode.py docs/sample_capture.bin
python host/la_decode.py --port COM5 --baud 115200 --prescale 49   # from the board
```

## Hardware status

Everything above is verified in simulation, and the DE10-Lite wrapper
compiles cleanly with `iverilog -Wall`. It has **not yet been synthesized in
Quartus or run on the board**. To try it: create the project pin assignments
with Terasic's DE10-Lite System Builder, add `rtl/*.v` and
`board/de10lite_top.v`, connect a 3.3 V USB-UART adapter to `GPIO[7]`, and
probe 3.3 V signals only.

## Limits

- SPI SCLK must be below f_clk / 4 (12.5 MHz at 50 MHz) because it is oversampled.
- UART: 8 data bits, 1 stop bit; `clks_per_bit` >= 4.
- I2C bits are sampled on SCL edges, so clock stretching should be followed, but it is not specifically tested; 10-bit addresses are shown as two bytes.
