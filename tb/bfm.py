"""Bus functional models: drive UART, SPI and I2C traffic onto DUT pins."""
from cocotb.triggers import FallingEdge, Timer


def delay(ns: float) -> Timer:
    """Timer rounded to whole picoseconds (e.g. 115200 baud = 8680.555... ns)."""
    return Timer(round(ns * 1000), unit="ps")


def parity_bit(byte: int, mode: str) -> int:
    ones = bin(byte).count("1") & 1
    return ones if mode == "even" else ones ^ 1


async def uart_send(sig, byte: int, bit_ns: float, parity: str | None = None,
                    stop_bit: int = 1, force_parity: int | None = None) -> None:
    bits = [0] + [(byte >> i) & 1 for i in range(8)]
    if parity is not None:
        bits.append(parity_bit(byte, parity) if force_parity is None else force_parity)
    bits.append(stop_bit)
    for b in bits:
        sig.value = b
        await delay(bit_ns)
    sig.value = 1


async def uart_receiver(sig, bit_ns: float, out: list) -> None:
    """Run forever, appending each received 8N1 byte to ``out``.
    Bytes with a bad stop bit are appended as ``("FRAMING", byte)``."""
    while True:
        await FallingEdge(sig)
        await delay(1.5 * bit_ns)
        byte = 0
        for i in range(8):
            byte |= int(sig.value) << i
            await delay(bit_ns)
        out.append(byte if int(sig.value) else ("FRAMING", byte))
        await delay(0.4 * bit_ns)


async def spi_transfer(sclk, mosi, miso, cs_n, mosi_bytes, miso_bytes, mode: int,
                       half_ns: float, n_bits: int | None = None) -> None:
    """Act as both master (MOSI) and slave (MISO). ``n_bits`` < 8*len aborts early."""
    cpol, cpha = mode >> 1, mode & 1
    mo_bits = [(b >> (7 - i)) & 1 for b in mosi_bytes for i in range(8)]
    mi_bits = [(b >> (7 - i)) & 1 for b in miso_bytes for i in range(8)]
    total = len(mo_bits) if n_bits is None else n_bits

    sclk.value = cpol
    cs_n.value = 0
    await delay(half_ns)
    for i in range(total):
        if cpha == 0:                    # data valid before the leading (sampling) edge
            mosi.value, miso.value = mo_bits[i], mi_bits[i]
            await delay(half_ns)
            sclk.value = 1 - cpol
            await delay(half_ns)
            sclk.value = cpol
        else:                            # data changes on leading edge, sampled on trailing
            sclk.value = 1 - cpol
            mosi.value, miso.value = mo_bits[i], mi_bits[i]
            await delay(half_ns)
            sclk.value = cpol
            await delay(half_ns)
    await delay(half_ns)
    cs_n.value = 1
    await delay(half_ns)


class I2cMaster:
    """Drives SCL and SDA directly (the analyzer only listens).

    ``q`` is a quarter of the SCL period in ns.

    ``sda_lead_ns`` > 0 models zero SDA hold time *plus* skew: the next data
    bit appears on SDA ``sda_lead_ns`` before the analyzer sees SCL fall. On
    hardware this happens because SCL and SDA go through separate
    synchronizers (which can resolve near-simultaneous edges a clock apart)
    and have different rise/fall times. Without compensation it looks
    exactly like a START or STOP condition."""

    def __init__(self, scl, sda, q_ns: float, sda_lead_ns: float = 0.0) -> None:
        self.scl, self.sda, self.q, self.lead = scl, sda, q_ns, sda_lead_ns

    async def _wait(self, n: int = 1) -> None:
        await delay(n * self.q)

    async def start(self) -> None:
        self.sda.value = 1
        self.scl.value = 1
        await self._wait()
        self.sda.value = 0
        await self._wait()
        self.scl.value = 0
        await self._wait()

    async def restart(self) -> None:
        self.sda.value = 1
        await self._wait()
        self.scl.value = 1
        await self._wait()
        self.sda.value = 0
        await self._wait()
        self.scl.value = 0
        await self._wait()

    async def _bit(self, bit: int) -> None:
        """One SCL period (4q): setup q, SCL high 2q, then hold q after SCL falls."""
        self.sda.value = bit
        await self._wait()
        self.scl.value = 1
        await self._wait(2)
        self.scl.value = 0
        await self._wait()

    async def write_byte(self, byte: int, ack: bool = True) -> None:
        bits = [(byte >> (7 - i)) & 1 for i in range(8)] + [0 if ack else 1]
        if not self.lead:
            for bit in bits:
                await self._bit(bit)
            return
        self.sda.value = bits[0]
        for i in range(len(bits)):
            await self._wait()
            self.scl.value = 1
            await self._wait(2)
            # next bit (or 0, ready for STOP) appears just before SCL falls
            self.sda.value = bits[i + 1] if i + 1 < len(bits) else 0
            await delay(self.lead)
            self.scl.value = 0
            await delay(self.q - self.lead)

    async def stop(self) -> None:
        self.sda.value = 0
        await self._wait()
        self.scl.value = 1
        await self._wait()
        self.sda.value = 1
        await self._wait()
