import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, Timer
import random

# Drives sync_coherent's two clock domains and checks the handshake behavior: dout only ever
# shows a value that was actually driven on din (or dout_default), follows din within a bounded
# settle time, refills after an output-only reset without din changing, and holds its value
# through an input-only reset.
class sync_coherent_base:

    def __init__(self, dut, in_clk_period, out_clk_period, time_unit="ns"):
        self.dut = dut
        self.in_clk_period = in_clk_period
        self.out_clk_period = out_clk_period
        self.time_unit = time_unit

        self.WIDTH = int(dut.WIDTH.value)
        self.MAX_DATA_VALUE = (1 << self.WIDTH) - 1
        self.DEFAULT = self.MAX_DATA_VALUE - 1
        # One full req/ack round trip plus a fresh snapshot, with margin, in the slower clock
        self.settle_time = 12 * max(in_clk_period, out_clk_period)

        self.dut._log.info(f"DUT WIDTH={self.WIDTH}, in_clk_period={in_clk_period}, out_clk_period={out_clk_period} {time_unit}")

        # Every value ever driven on din; dout must always be one of these or the default
        self.driven = set()

        self.in_clk_task = None
        self.out_clk_task = None

        self.dut.in_resetn.value = 1
        self.dut.out_resetn.value = 1
        self.dut.dout_default.value = self.DEFAULT
        self.set_din(0)

    def set_din(self, value):
        self.driven.add(value)
        self.dut.din.value = value

    async def start_clocks(self):
        self.in_clk_task = cocotb.start_soon(Clock(self.dut.in_clk, self.in_clk_period, unit=self.time_unit).start(start_high=False))
        self.out_clk_task = cocotb.start_soon(Clock(self.dut.out_clk, self.out_clk_period, unit=self.time_unit).start(start_high=False))

    async def kill_clocks(self):
        for task in (self.in_clk_task, self.out_clk_task):
            if task and not task.done():
                task.cancel()
        self.in_clk_task = None
        self.out_clk_task = None

    async def settle(self):
        await Timer(self.settle_time, unit=self.time_unit)

    async def drive(self, value):
        await RisingEdge(self.dut.in_clk)
        self.set_din(value)

    async def out_reset(self, cycles=3):
        await RisingEdge(self.dut.out_clk)
        self.dut.out_resetn.value = 0
        for _ in range(cycles):
            await RisingEdge(self.dut.out_clk)
        await ReadOnly()
        assert int(self.dut.dout.value) == self.DEFAULT, "dout should sit at dout_default during an output reset"
        await RisingEdge(self.dut.out_clk)
        self.dut.out_resetn.value = 1

    async def in_reset_start(self):
        await RisingEdge(self.dut.in_clk)
        self.dut.in_resetn.value = 0

    async def in_reset_end(self):
        await RisingEdge(self.dut.in_clk)
        self.dut.in_resetn.value = 1

    async def reset_both(self):
        await self.in_reset_start()
        await self.out_reset()
        await self.in_reset_end()

    async def expect_dout(self, value, msg=""):
        await ReadOnly()
        got = int(self.dut.dout.value)
        assert got == value, f"dout {got:#x}, expected {value:#x} {msg}"
        await RisingEdge(self.dut.out_clk)

    async def coherence_monitor(self):
        # A copy taken while the hold register was changing would show up as a value never driven
        while True:
            await RisingEdge(self.dut.out_clk)
            await ReadOnly()
            got = int(self.dut.dout.value)
            assert got in self.driven or got == self.DEFAULT, f"dout {got:#x} was never driven on din"

    async def random_driver(self, steps, max_hold):
        for _ in range(steps):
            await self.drive(random.randint(0, self.MAX_DATA_VALUE))
            for _ in range(random.randint(0, max_hold)):
                await RisingEdge(self.dut.in_clk)

