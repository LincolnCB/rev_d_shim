import cocotb
from cocotb.triggers import RisingEdge
import random
from sync_coherent_base import sync_coherent_base

async def setup_testbench(dut, in_clk_period=4, out_clk_period=4, time_unit="ns"):
    tb = sync_coherent_base(dut, in_clk_period, out_clk_period, time_unit)
    await tb.start_clocks()
    await tb.reset_both()
    # Let a value still in flight from a previous test land before the coherence monitor starts
    await tb.settle()
    return tb

# dout follows a sequence of held values at equal, faster-in and slower-in clock ratios
@cocotb.test()
async def test_tracks_din(dut):
    for in_period, out_period in ((4, 4), (4, 10), (10, 4)):
        tb = await setup_testbench(dut, in_period, out_period)
        monitor = cocotb.start_soon(tb.coherence_monitor())
        for value in (0x11, 0x22, 0xC3, 0x00, 0xFF):
            await tb.drive(value)
            await tb.settle()
            await tb.expect_dout(value, f"after holding din (in {in_period} ns, out {out_period} ns)")
        monitor.cancel()
        await tb.kill_clocks()

# An output-only reset (the SPI side of spi_cfg_sync on every SPI power cycle) refills dout
# from the current din without din changing
@cocotb.test()
async def test_out_only_reset_refills(dut):
    tb = await setup_testbench(dut, 6, 8)
    monitor = cocotb.start_soon(tb.coherence_monitor())
    await tb.drive(0x5A)
    await tb.settle()
    await tb.expect_dout(0x5A, "before the output reset")
    for _ in range(3):
        await tb.out_reset()
        await tb.settle()
        await tb.expect_dout(0x5A, "after an output-only reset with din unchanged")
    monitor.cancel()
    await tb.kill_clocks()

# An input-only reset pauses snapshots: dout keeps its value, then follows din once released
@cocotb.test()
async def test_in_only_reset_holds(dut):
    tb = await setup_testbench(dut, 8, 6)
    monitor = cocotb.start_soon(tb.coherence_monitor())
    await tb.drive(0x3C)
    await tb.settle()
    await tb.expect_dout(0x3C, "before the input reset")
    await tb.in_reset_start()
    tb.set_din(0xC3)
    await tb.settle()
    await tb.expect_dout(0x3C, "while the input side is in reset")
    await tb.in_reset_end()
    await tb.settle()
    await tb.expect_dout(0xC3, "after the input reset is released")
    monitor.cancel()
    await tb.kill_clocks()

# din changing every input cycle: dout skips values but only ever shows ones that were driven
@cocotb.test()
async def test_fast_changing_din(dut):
    random.seed(1234)
    for in_period, out_period in ((4, 10), (10, 4), (7, 7)):
        tb = await setup_testbench(dut, in_period, out_period)
        monitor = cocotb.start_soon(tb.coherence_monitor())
        await tb.random_driver(steps=200, max_hold=0)
        final = random.randint(0, tb.MAX_DATA_VALUE)
        await tb.drive(final)
        await tb.settle()
        await tb.expect_dout(final, "after din stops changing")
        monitor.cancel()
        await tb.kill_clocks()

# Random clock ratios, random din timing, and resets on either side mid-stream
@cocotb.test()
async def test_random(dut):
    random.seed(4321)
    for i in range(10):
        in_period = random.randint(4, 20)
        out_period = random.randint(4, 20)
        tb = await setup_testbench(dut, in_period, out_period)
        tb.dut._log.info(f"Random iteration {i + 1}: in {in_period} ns, out {out_period} ns")
        monitor = cocotb.start_soon(tb.coherence_monitor())
        await tb.random_driver(steps=40, max_hold=12)
        monitor.cancel()
        if random.random() < 0.5:
            await tb.out_reset()
        else:
            await tb.in_reset_start()
            for _ in range(random.randint(1, 10)):
                await RisingEdge(dut.in_clk)
            await tb.in_reset_end()
        monitor = cocotb.start_soon(tb.coherence_monitor())
        await tb.random_driver(steps=40, max_hold=12)
        final = random.randint(0, tb.MAX_DATA_VALUE)
        await tb.drive(final)
        await tb.settle()
        await tb.expect_dout(final, f"at the end of random iteration {i + 1}")
        monitor.cancel()
        await tb.kill_clocks()


