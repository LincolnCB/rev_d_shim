import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, ReadWrite
import random
from collections import deque


class fifo_sync_base:

    def __init__(self, dut, clk_period=4, time_unit="ns"):
        self.dut = dut

        # Get parameters from the DUT
        self.DATA_WIDTH = int(self.dut.DATA_WIDTH.value)
        self.ADDR_WIDTH = int(self.dut.ADDR_WIDTH.value)
        self.ALMOST_FULL_THRESHOLD = int(self.dut.ALMOST_FULL_THRESHOLD.value)
        self.ALMOST_EMPTY_THRESHOLD = int(self.dut.ALMOST_EMPTY_THRESHOLD.value)
        self.FIFO_DEPTH = 2**self.ADDR_WIDTH
        self.MAX_DATA_VALUE = (2**self.DATA_WIDTH) - 1

        self.dut._log.info(f"FIFO PARAMETERS: DATA_WIDTH={self.DATA_WIDTH}, ADDR_WIDTH={self.ADDR_WIDTH}, "
                      f"DEPTH={self.FIFO_DEPTH}, ALMOST_FULL_THRESHOLD={self.ALMOST_FULL_THRESHOLD}, "
                      f"ALMOST_EMPTY_THRESHOLD={self.ALMOST_EMPTY_THRESHOLD}")

        # Queue to store expected data for verification
        self.expected_data_q = deque()

        # Start the clock
        cocotb.start_soon(Clock(self.dut.clk, clk_period, units=time_unit).start()) 

        # Initialize input signals
        self.dut.wr_en.value = 0
        self.dut.rd_en.value = 0
        self.dut.wr_data.value = 0

    async def reset(self):
        """
        Resets the DUT for 2 clk cycles.
        Verifies the FIFO is empty after reset.
        """
        await RisingEdge(self.dut.clk)
        self.dut._log.info("STARTING RESET")
        self.dut.resetn.value = 0  # Assert active-low reset
        self.expected_data_q.clear()  # Clear expected data queue on reset
        await RisingEdge(self.dut.clk)
        await RisingEdge(self.dut.clk)
        self.dut.resetn.value = 1  # Deassert reset
        self.dut._log.info("RESET COMPLETE")

    async def write(self, data):
        """
        Writes a single data item to the FIFO.
        Will not write if the FIFO is full.
        Args:
            data (int): The data to write.
        Returns:
            bool: True if write was successful, False if FIFO was full.
        """
        await RisingEdge(self.dut.clk)
        await ReadWrite()

        if self.dut.full.value == 1:
            self.dut._log.info(f"Attempted to write 0x{data:X} but FIFO is full. Skipping write.")
            return False

        self.dut._log.info(f"Writing data: 0x{data:X}")
        self.dut.wr_data.value = data
        self.dut.wr_en.value = 1

        self.expected_data_q.append(data) # Add to expected queue immediately

        await RisingEdge(self.dut.clk)
        self.dut.wr_en.value = 0 # Deassert write enable

        return True
    
    async def read(self):
        """
        Reads a single data item from the FWFT sync FIFO.
        Will not read if the FIFO is empty.
        Returns:
            tuple: (read_value, expected_value, True) if read successful, (None, None, False) if FIFO was empty.
        """
        await RisingEdge(self.dut.clk)
        await ReadWrite()

        if self.dut.empty.value == 1:
            self.dut._log.info("Attempted to read but FIFO is empty. Skipping read.")
            return (None, None, False)
        
        self.dut.rd_en.value = 1

        await ReadOnly()  # Wait for combinational logic to settle
        # In FWFT with registered output, rd_data presents the current head item
        # when rd_en is low (or before the clock edge where rd_en is asserted)
        read_val = self.dut.rd_data.value
        expected_val = self.expected_data_q.popleft() # Pop from expected queue before asserting rd_en

        self.dut._log.info(f"Reading data. Expected: 0x{expected_val:X}, Actual: 0x{int(read_val):X}")

        

        await RisingEdge(self.dut.clk) # Wait for the read enable to take effect (pointer update)
        self.dut.rd_en.value = 0 # Deassert read enable

        return (read_val, expected_val, True)

    async def write_burst(self, data_list):
        """
        Writes a burst of data items to the FIFO.
        Will not write if the FIFO is full.
        Args:
            data_list (list): List of integers to write.
        Returns:
            int: Number of successful writes.
        """
        written_count = 0
        self.dut._log.info(f"Starting burst write with {len(data_list)} items.")

        for data in data_list:
            await RisingEdge(self.dut.clk)
            await ReadWrite()

            if self.dut.full.value == 1:
                self.dut._log.info(f"FIFO full during burst write after {written_count} items. Skipping remaining writes including 0x{data:X}.")
                self.dut.wr_en.value = 0
                break

            self.dut._log.info(f"Writing data: 0x{data:X}")
            self.dut.wr_data.value = data
            self.dut.wr_en.value = 1
            self.expected_data_q.append(data)  # Add to expected queue immediately
            written_count += 1
        
        await RisingEdge(self.dut.clk)
        self.dut.wr_en.value = 0  # Deassert write enable after burst write
        self.dut._log.info(f"Burst write complete. Total items written: {written_count}")
        return written_count
    
    async def read_burst(self, count):
        """
        Reads a burst of data items from the FIFO.
        Will not read if the FIFO is empty.
        Args:
            count (int): Number of items to read.
        Returns:
            list: List of tuples (read_value, expected_value) for each read item.
        """
        read_items = []
        self.dut._log.info(f"Starting burst read for {count} items.")

        for _ in range(count):
            await RisingEdge(self.dut.clk)
            await ReadWrite()

            if self.dut.empty.value == 1:
                self.dut._log.info("FIFO empty during burst read. Stopping reads.")
                self.dut.rd_en.value = 0
                break

            self.dut.rd_en.value = 1

            await ReadOnly()
            read_value = self.dut.rd_data.value
            expected_value = self.expected_data_q.popleft()
            read_items.append((int(read_value), expected_value))

            self.dut._log.info(f"Reading data. Expected: 0x{expected_value:X}, Actual: 0x{int(read_value):X}")

        await RisingEdge(self.dut.clk)
        self.dut.rd_en.value = 0  # Deassert read enable after burst read
        self.dut._log.info(f"Burst read complete. Total items read: {len(read_items)}")
        return read_items
    
    async def print_fifo_status(self):
        """
        Prints the current status of the FIFO including full, empty, almost full, and almost empty flags.
        """
        await ReadWrite()
        self.dut._log.info(f"FIFO Status - Full: {self.dut.full.value}")
        self.dut._log.info(f"FIFO Status - Empty: {self.dut.empty.value}")
        self.dut._log.info(f"FIFO Status - Almost Full: {self.dut.almost_full.value}")
        self.dut._log.info(f"FIFO Status - Almost Empty: {self.dut.almost_empty.value}")
        self.dut._log.info(f"FIFO Status - Number of items left in the FIFO: {len(self.expected_data_q)}")

    async def generate_random_data(self, count):
        """
        Generates a list of random data values within the DATA_WIDTH range.
        Args:
            count (int): The number of random data values to generate.
        Returns:
            list: A list of random integers.
        """
        return [random.randint(0, self.MAX_DATA_VALUE) for _ in range(count)]
    
    def print_expected_data(self):
        """
        Prints the current expected data queue for debugging.
        """
        self.dut._log.info("Current expected data in FIFO:")
        for data in self.expected_data_q:
            self.dut._log.info(f"0x{data:X}")

                           
    