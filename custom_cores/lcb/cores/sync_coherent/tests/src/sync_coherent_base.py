import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, ReadWrite
import random
from collections import deque

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

        # Expected data queue
        self.expected_data_q = deque()
        self.num_expected_data = 1

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

        self.in_clk_task = cocotb.start_soon(Clock(self.dut.in_clk, self.in_clk_period, units=self.time_unit).start(start_high=False))
        self.out_clk_task = cocotb.start_soon(Clock(self.dut.out_clk, self.out_clk_period, units=self.time_unit).start(start_high=False))
        self.dut._log.info("Clocks started.")

    async def kill_clocks(self):
        """Kills the running read and write clock tasks."""
        if self.in_clk_task and not self.in_clk_task.done():
            self.in_clk_task.kill()
            self.dut._log.info("In clock killed.")
        else:
            self.dut._log.info("In clock task not active or already done.")

        if self.out_clk_task and not self.out_clk_task.done():
            self.out_clk_task.kill()
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
        self.expected_data_q.clear()  # Clear expected data queue on reset
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
        self.expected_data_q.clear()  # Clear expected data queue on reset
        await RisingEdge(self.dut.out_clk)
        await RisingEdge(self.dut.out_clk)
        self.dut.out_resetn.value = 1  # Deassert reset

        await ReadOnly()  # Ensure all signals are updated
        assert self.dut.dout.value == self.dut.dout_default.value, "DOUT should be reset to default value."
        self.dut._log.info("OUT SIDE RESET COMPLETE")

    # Does not handle WIDTH parameter while driving!
    async def static_din_driver_and_monitor(self, cycles=10, initial_data=1, expect_dummy=True):
        # DUT will always write to the FIFO, even before a valid data we drive comes.
        # Therefore, first append a dummy value to the expected data queue.
        if expect_dummy:
            self.expected_data_q.append(self.MAX_DATA_VALUE)
        self.num_expected_data = cycles + 1 if expect_dummy else cycles

        for i in range(cycles):
            await RisingEdge(self.dut.in_clk)
            data = initial_data + i
            self.dut.din.value = data
            self.dut._log.info(f"DIN Driver: Driving din with value {data}.")

            await ReadOnly()
            if (self.dut.wr_en.value):
                self.dut._log.info(f"DIN Driver/Monitor: Current din going to expected data queue is {data}.")
                self.expected_data_q.append(data)
                self.dut._log.info(f"Expected data queue updated: {list(self.expected_data_q)}")
                if len(self.expected_data_q) > self.DEPTH:
                    self.dut._log.warning(f"Expected data queue exceeded depth: {len(self.expected_data_q)}")
            else:
                self.dut._log.info(f"DIN Driver/Monitor: wr_en is low, not writing din to expected data queue. wr_en: {self.dut.wr_en.value}")

    async def random_din_driver_and_monitor(self, cycles=10):
        # DUT will always write to the FIFO, even before a valid data we drive comes.
        # Therefore, first append a dummy value to the expected data queue.
        self.expected_data_q.append(self.MAX_DATA_VALUE)
        self.num_expected_data = cycles + 1  # Include the initial dummy value

        for _ in range(cycles):
            await RisingEdge(self.dut.in_clk)
            data = random.randint(0, self.MAX_DATA_VALUE)
            self.dut.din.value = data
            self.dut._log.info(f"DIN Driver: Driving din with value {data}.")

            await ReadOnly()
            if (self.dut.wr_en.value):
                self.dut._log.info(f"DIN Driver/Monitor: Current din going to expected data queue is {data}.")
                self.expected_data_q.append(data)
                self.dut._log.info(f"Expected data queue updated: {list(self.expected_data_q)}")
                if len(self.expected_data_q) > self.DEPTH:
                    self.dut._log.warning(f"Expected data queue exceeded depth: {len(self.expected_data_q)}")
            else:
                self.dut._log.info(f"DIN Driver/Monitor: wr_en is low, not writing din to expected data queue. wr_en: {self.dut.wr_en.value}")

        await RisingEdge(self.dut.in_clk)
        self.dut.din.value = 0  # Reset din to 0 after writing

    async def dout_scoreboard(self):
        num_expected_data_read = 0
        while True:
            await RisingEdge(self.dut.out_clk)
            # Sample previous fifo_empty value
            prev_fifo_empty = int(self.dut.fifo_empty.value)
            await ReadOnly()

            if (prev_fifo_empty == 1):
                self.dut._log.info("Attempted to read dout, but async fifo is empty, will try again later.")
            else:
                expected_data = self.expected_data_q.popleft()
                num_expected_data_read += 1
                actual_data = int(self.dut.dout.value)
                self.dut._log.info(f"Expected data: {expected_data}, Actual data: {actual_data}")
                self.dut._log.info(f"Expected data queue: {list(self.expected_data_q)}")
                assert actual_data == expected_data, f"Data mismatch: expected {expected_data}, got {actual_data}"

            if num_expected_data_read == self.num_expected_data:
                self.dut._log.info("All expected data has been read.")
                break

    async def prev_din_and_wr_en_scoreboard(self):
        while True:
            await RisingEdge(self.dut.in_clk)
            prev_fifo_full = int(self.dut.fifo_full.value)
            prev_din_tracker = int(self.dut.din.value)
            await ReadOnly()

            if prev_fifo_full == 0:
                assert prev_din_tracker == self.dut.prev_din.value, f"prev_din mismatch: expected {prev_din_tracker}, got {self.dut.prev_din.value}"

            wr_en_condition = (int(self.dut.din.value) != int(self.dut.prev_din.value) and int(self.dut.fifo_full) == 0) or int(self.dut.fifo_empty) == 1

            if wr_en_condition:
                assert int(self.dut.wr_en.value) == 1, \
                    f"wr_en should be 1, but got {self.dut.wr_en.value}"
            else:
                assert int(self.dut.wr_en.value) == 0, \
                    f"wr_en should be 0, but got {self.dut.wr_en.value}"

