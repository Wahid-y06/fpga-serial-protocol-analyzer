"""Builds each RTL module with Icarus Verilog and runs its cocotb testbench.

    python -m pytest            # all simulations
    WAVES=1 python -m pytest    # also dump waveforms (open with GTKWave)
"""
import os
import shutil
import sys
from pathlib import Path

import pytest
from cocotb_tools.runner import get_results, get_runner

ROOT = Path(__file__).resolve().parent.parent
RTL = ROOT / "rtl"
TB = ROOT / "tb"
sys.path.insert(0, str(TB))
sys.path.insert(0, str(ROOT / "host"))

if shutil.which("iverilog") is None and Path("C:/iverilog/bin").exists():
    os.environ["PATH"] = "C:/iverilog/bin" + os.pathsep + os.environ["PATH"]

ALL_RTL = ["sync2.v", "uart_rx_decoder.v", "spi_decoder.v", "i2c_decoder.v",
           "uart_tx.v", "capture_buffer.v", "analyzer_top.v"]

CASES = {
    # toplevel: (sources, cocotb module, parameters)
    "uart_rx_decoder": (["uart_rx_decoder.v"], "cocotb_uart", {}),
    "spi_decoder": (["spi_decoder.v"], "cocotb_spi", {}),
    "i2c_decoder": (["i2c_decoder.v"], "cocotb_i2c", {}),
    "uart_tx": (["uart_tx.v"], "cocotb_uart_tx", {}),
    "analyzer_top": (ALL_RTL, "cocotb_top", {"AW": 4}),
}


@pytest.mark.skipif(shutil.which("iverilog") is None, reason="Icarus Verilog not installed")
@pytest.mark.parametrize("toplevel", list(CASES))
def test_rtl(toplevel):
    sources, module, params = CASES[toplevel]
    waves = os.environ.get("WAVES") == "1"
    build_dir = ROOT / "sim_build" / toplevel
    runner = get_runner("icarus")
    runner.build(sources=[RTL / s for s in sources], hdl_toplevel=toplevel,
                 build_dir=build_dir, always=True, waves=waves, parameters=params,
                 build_args=["-Wall"], timescale=("1ns", "1ps"))
    results = runner.test(hdl_toplevel=toplevel, test_module=module,
                          build_dir=build_dir, test_dir=build_dir, waves=waves)
    total, failed = get_results(results)
    assert total > 0, "no cocotb tests ran"
    assert failed == 0, f"{failed} of {total} cocotb tests failed for {toplevel}"
