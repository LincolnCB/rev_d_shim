import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, ReadWrite
import random

class sync_coherent_base:

    def __init__(self, dut, in_clk_period, out_clk_period, time_unit="ns"):
        self.dut = dut
        self.in_clk_period = in_clk_period
        self.out_clk_period = out_clk_period
        self.time_unit = time_unit

        # Parameters from DUT
        self.WIDTH = int(dut.WIDTH.value)
        self.BUF_ADDR_WIDTH = int(dut.BUF_ADDR_WIDTH.value)
        self.DEPTH = 2 ** self.BUF_ADDR_WIDTH
        self.MAX_DATA_VALUE = (1 << self.WIDTH) - 1

        # Log the initial parameters
        self.dut._log.info(f"DUT Initialized with WIDTH: {self.WIDTH}")
        self.dut._log.info(f"DUT Initialized with BUF_ADDR_WIDTH: {self.BUF_ADDR_WIDTH}")
        self.dut._log.info(f"DUT Initialized with in_clk_period: {self.in_clk_period} {self.time_unit}")
        self.dut._log.info(f"DUT Initialized with out_clk_period: {self.out_clk_period} {self.time_unit}")

        # Coalesced value streams for the producer/consumer scoreboard, and their sync flag.
        self.expected_seq = []
        self.dout_seq = []
        self.producer_done = False

        # Clock tasks
        self.in_clk_task = None
        self.out_clk_task = None

        # Initialize input signals
        self.dut.din.value = self.MAX_DATA_VALUE
        self.dut.dout_default.value = self.MAX_DATA_VALUE - 1

    async def start_clocks(self):
        """Starts the in_clk and out_clk and stores their Tasks."""
        if self.in_clk_task and self.in_clk_task.done(): # Check if previous task is done/killed
            self.in_clk_task = None # Clear reference if task is no longer running
        if self.out_clk_task and self.out_clk_task.done():
            self.out_clk_task = None

        self.in_clk_task = cocotb.start_soon(Clock(self.dut.in_clk, self.in_clk_period, unit=self.time_unit).start(start_high=False))
        self.out_clk_task = cocotb.start_soon(Clock(self.dut.out_clk, self.out_clk_period, unit=self.time_unit).start(start_high=False))
        self.dut._log.info("Clocks started.")

    async def kill_clocks(self):
        """Kills the running read and write clock tasks."""
        if self.in_clk_task and not self.in_clk_task.done():
            self.in_clk_task.cancel()
            self.dut._log.info("In clock killed.")
        else:
            self.dut._log.info("In clock task not active or already done.")

        if self.out_clk_task and not self.out_clk_task.done():
            self.out_clk_task.cancel()
            self.dut._log.info("Out clock killed.")
        else:
            self.dut._log.info("Out clock task not active or already done.")

        self.in_clk_task = None  # Clear references after killing
        self.out_clk_task = None
        self.dut._log.info("All clock tasks cleared.")

    async def in_side_reset(self):
        """
        Resets in side of the DUT for 2 clk cycles.
        """
        await RisingEdge(self.dut.in_clk)
        self.dut._log.info("STARTING IN SIDE RESET")
        self.dut.in_resetn.value = 0  # Assert active-low reset
        self.expected_seq = []  # Clear expected stream on reset
        await RisingEdge(self.dut.in_clk)
        await RisingEdge(self.dut.in_clk)
        self.dut.in_resetn.value = 1  # Deassert reset
        self.dut._log.info("IN SIDE RESET COMPLETE")

    async def out_side_reset(self):
        """
        Resets out side of the DUT for 2 clk cycles.
        """
        await RisingEdge(self.dut.out_clk)
        self.dut._log.info("STARTING OUT SIDE RESET")
        self.dut.out_resetn.value = 0  # Assert active-low reset
        self.expected_seq = []  # Clear expected stream on reset
        await RisingEdge(self.dut.out_clk)
        await RisingEdge(self.dut.out_clk)
        self.dut.out_resetn.value = 1  # Deassert reset

        await ReadOnly()  # Ensure all signals are updated
        assert self.dut.dout.value == self.dut.dout_default.value, "DOUT should be reset to default value."
        # Leave the ReadOnly phase so coroutines started right after this reset can await ReadOnly.
        await RisingEdge(self.dut.out_clk)
        self.dut._log.info("OUT SIDE RESET COMPLETE")

    # Drives `cycles` values on din, one per in_clk cycle, and builds the expected COALESCED
    # stream of values the DUT commits to the FIFO. Consecutive duplicates are merged: the DUT
    # re-commits the current din whenever the FIFO drains (wr_en's ||empty term), and those
    # repeats surface on dout as an unchanged value. A value is dropped when the FIFO is full
    # (wr_en low) and is then absent from both streams. The FIFO is empty right after reset, so
    # the reset-held din is always the first commit -- seed it.
    async def static_din_driver_and_monitor(self, cycles=10, initial_data=1):
        await self._drive_and_record([initial_data + i for i in range(cycles)])

    async def random_din_driver_and_monitor(self, cycles=10):
        await self._drive_and_record([random.randint(0, self.MAX_DATA_VALUE) for _ in range(cycles)])

    async def _drive_and_record(self, values):
        self.producer_done = False
        seed = int(self.dut.din.value)
        self.expected_seq = [seed]
        last = seed
        for v in values:
            await RisingEdge(self.dut.in_clk)
            self.dut.din.value = v
            await ReadOnly()
            # wr_en is combinational; sampled here it predicts whether the NEXT edge commits v.
            if int(self.dut.wr_en.value) and v != last:
                self.expected_seq.append(v)
                last = v
        await RisingEdge(self.dut.in_clk)   # let the final predicted write commit
        self.producer_done = True

    async def dout_scoreboard(self):
        # Build the coalesced stream of values seen on dout (one sample per non-empty out read,
        # consecutive duplicates merged) and compare it to the expected commit stream. fifo_empty
        # is out_clk-domain and only changes on out_clk edges, so the value read just after one
        # edge gates the next edge's dout update. One ReadOnly per iteration (cocotb requirement).
        self.dout_seq = []
        last = None
        empty_before = None
        while True:
            await RisingEdge(self.dut.out_clk)
            await ReadOnly()
            if empty_before is False:
                val = int(self.dut.dout.value)
                if val != last:
                    self.dout_seq.append(val)
                    last = val
            empty_before = (int(self.dut.fifo_empty.value) == 1)
            if self.producer_done and len(self.dout_seq) >= len(self.expected_seq):
                break
        assert self.dout_seq == self.expected_seq, \
            f"stream mismatch:\n  expected {self.expected_seq}\n  got      {self.dout_seq}"

    async def prev_din_and_wr_en_scoreboard(self):
        while True:
            await RisingEdge(self.dut.in_clk)
            prev_fifo_full = int(self.dut.fifo_full.value)
            prev_din_tracker = int(self.dut.din.value)
            await ReadOnly()

            if prev_fifo_full == 0:
                assert prev_din_tracker == int(self.dut.prev_din.value), f"prev_din mismatch: expected {prev_din_tracker}, got {int(self.dut.prev_din.value)}"

            wr_en_condition = (int(self.dut.din.value) != int(self.dut.prev_din.value) and int(self.dut.fifo_full.value) == 0) or int(self.dut.fifo_empty.value) == 1

            if wr_en_condition:
                assert int(self.dut.wr_en.value) == 1, \
                    f"wr_en should be 1, but got {self.dut.wr_en.value}"
            else:
                assert int(self.dut.wr_en.value) == 0, \
                    f"wr_en should be 0, but got {self.dut.wr_en.value}"

