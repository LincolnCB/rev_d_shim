import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, ReadWrite
import random
from collections import deque


class fifo_async_base:

    def __init__(self, dut, rd_clk_period=4, wr_clk_period=4, time_unit="ns"):
        self.dut = dut

        # Get parameters from the DUT
        self.DATA_WIDTH = int(self.dut.DATA_WIDTH.value)
        self.ADDR_WIDTH = int(self.dut.ADDR_WIDTH.value)
        self.ALMOST_FULL_THRESHOLD = int(self.dut.ALMOST_FULL_THRESHOLD.value)
        self.ALMOST_EMPTY_THRESHOLD = int(self.dut.ALMOST_EMPTY_THRESHOLD.value)
        self.FIFO_DEPTH = 2**self.ADDR_WIDTH
        self.MAX_DATA_VALUE = (2**self.DATA_WIDTH) - 1

        self.dut._log.info(f"STARTING FIFO PARAMETERS: DATA_WIDTH={self.DATA_WIDTH}, ADDR_WIDTH={self.ADDR_WIDTH}, "
                      f"DEPTH={self.FIFO_DEPTH}, ALMOST_FULL_THRESHOLD={self.ALMOST_FULL_THRESHOLD}, "
                      f"ALMOST_EMPTY_THRESHOLD={self.ALMOST_EMPTY_THRESHOLD}")
        self.dut._log.info(f"Read Clock Period: {rd_clk_period} {time_unit}") 
        self.dut._log.info(f"Write Clock Period: {wr_clk_period} {time_unit}")                   

        # Queue to store expected data for verification
        self.expected_data_q = deque()

        self.rd_clk_period = rd_clk_period
        self.wr_clk_period = wr_clk_period
        self.time_unit = time_unit
        self.rd_clk_task = None
        self.wr_clk_task = None

        # Initialize input signals
        self.dut.wr_en.value = 0
        self.dut.rd_en.value = 0
        self.dut.wr_data.value = 0

    async def start_clocks(self):
        """Starts the read and write clocks and stores their Tasks."""
        if self.rd_clk_task and self.rd_clk_task.done(): # Check if previous task is done/killed
            self.rd_clk_task = None # Clear reference if task is no longer running
        if self.wr_clk_task and self.wr_clk_task.done():
            self.wr_clk_task = None

        self.rd_clk_task = cocotb.start_soon(Clock(self.dut.rd_clk, self.rd_clk_period, units=self.time_unit).start())
        self.wr_clk_task = cocotb.start_soon(Clock(self.dut.wr_clk, self.wr_clk_period, units=self.time_unit).start())
        self.dut._log.info("Clocks started.")

    async def kill_clocks(self):
        """Kills the running read and write clock tasks."""
        if self.rd_clk_task and not self.rd_clk_task.done():
            self.rd_clk_task.kill()
            self.dut._log.info("Read clock killed.")
        else:
            self.dut._log.info("Read clock task not active or already done.")

        if self.wr_clk_task and not self.wr_clk_task.done():
            self.wr_clk_task.kill()
            self.dut._log.info("Write clock killed.")
        else:
            self.dut._log.info("Write clock task not active or already done.")

        self.rd_clk_task = None  # Clear references after killing
        self.wr_clk_task = None
        self.dut._log.info("All clock tasks cleared.")

    async def wr_reset(self):
        """
        Resets the DUT for 2 clk cycles.
        Verifies the FIFO is empty after reset.
        """
        await RisingEdge(self.dut.wr_clk)
        self.dut._log.info("STARTING WRITE RESET")
        self.dut.wr_rst_n.value = 0  # Assert active-low reset
        self.expected_data_q.clear()  # Clear expected data queue on reset
        await RisingEdge(self.dut.wr_clk)
        await RisingEdge(self.dut.wr_clk)
        self.dut.wr_rst_n.value = 1  # Deassert reset
        self.dut._log.info("WRITE RESET COMPLETE")

    async def rd_reset(self):
        """
        Resets the DUT for 2 clk cycles.
        Verifies the FIFO is empty after reset.
        """
        await RisingEdge(self.dut.rd_clk)
        self.dut._log.info("STARTING READ RESET")
        self.dut.rd_rst_n.value = 0  # Assert active-low reset
        self.expected_data_q.clear()  # Clear expected data queue on reset
        await RisingEdge(self.dut.rd_clk)
        await RisingEdge(self.dut.rd_clk)
        self.dut.rd_rst_n.value = 1  # Deassert reset
        self.dut._log.info("READ RESET COMPLETE")

    async def write(self, data):
        """
        Writes a single data item to the FIFO.
        Will try to write again in the next cycle if the FIFO is full.
        Args:
            data (int): The data to write.
        """
        while True:
            await RisingEdge(self.dut.wr_clk)
            await ReadWrite()
            fifo_full = False

            if self.dut.full.value == 1:
                self.dut._log.info(f"Want to write 0x{data:X} but FIFO is full. Will try again in the next write cycle.")
                fifo_full = True

            if not fifo_full:
                self.dut._log.info(f"Writing data: 0x{data:X}")
                self.dut.wr_data.value = data
                self.dut.wr_en.value = 1

                self.expected_data_q.append(data) # Add to expected queue immediately

                await RisingEdge(self.dut.wr_clk)
                self.dut.wr_en.value = 0 # Deassert write enable
                break  # Exit loop after successful write
    
    async def read(self):
        """
        Reads a single data item from the FWFT sync FIFO.
        Will try to read in the next clock cycle if the FIFO is empty.
        Returns:
            tuple: (read_value, expected_value)
        """
        while True:
            await RisingEdge(self.dut.rd_clk)
            await ReadWrite()
            fifo_empty = False

            if self.dut.empty.value == 1:
                self.dut._log.info("Want to read but FIFO is empty. Will try again in the next read cycle.")
                fifo_empty = True
            
            if not fifo_empty:
                self.dut.rd_en.value = 1

                await ReadOnly()  # Wait for combinational logic to settle
                # In FWFT with registered output, rd_data presents the current head item
                # when rd_en is low (or before the clock edge where rd_en is asserted)
                read_val = self.dut.rd_data.value
                expected_val = self.expected_data_q.popleft() # Pop from expected queue before asserting rd_en

                self.dut._log.info(f"Reading data. Expected: 0x{expected_val:X}, Actual: 0x{int(read_val):X}")

                await RisingEdge(self.dut.rd_clk) # Wait for the read enable to take effect (pointer update)
                self.dut.rd_en.value = 0 # Deassert read enable
                return (read_val, expected_val)

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
            await RisingEdge(self.dut.wr_clk)
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
        
        await RisingEdge(self.dut.wr_clk)
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
            await RisingEdge(self.dut.rd_clk)
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

        await RisingEdge(self.dut.rd_clk)
        self.dut.rd_en.value = 0  # Deassert read enable after burst read
        self.dut._log.info(f"Burst read complete. Total items read: {len(read_items)}")
        return read_items
    
    async def write_back_to_back(self, data_list):
        """
        Writes a list of data items to the FIFO back-to-back.
        Will try to write again in the next cycle if the FIFO is full.
        Args:
            data_list (list): List of integers to write.
        """
        number_of_writes = len(data_list)
        data_index = 0

        while data_index < number_of_writes:
            await RisingEdge(self.dut.wr_clk) # Wait for clock edge
            self.dut.wr_data.value = data_list[data_index] 
            
            await ReadWrite() # Allow 'full' to settle after the edge

            if self.dut.full.value == 1:
                self.dut._log.info(f"Want to write 0x{data_list[data_index]:X} but FIFO is full. Will try again in the next write cycle.")
                self.dut.wr_en.value = 0 # Keep wr_en de-asserted if full
            else:
                # FIFO is not full, proceed with write
                self.dut.wr_en.value = 1 # Assert wr_en for this cycle
                self.expected_data_q.append(int(data_list[data_index])) # Add to expected queue
                self.dut._log.info(f"Writing data: 0x{data_list[data_index]:X}")
                data_index += 1 # Advance to next data only on successful write
        
        # After all data has been attempted to be written, ensure wr_en is de-asserted.
        # Wait one more cycle to ensure the last asserted wr_en takes effect.
        await RisingEdge(self.dut.wr_clk)
        self.dut.wr_en.value = 0

    async def read_back_to_back(self, count):
        """
        Reads a specified number of data items from the FIFO back-to-back.
        Will try to read in the next clock cycle if the FIFO is empty.
        Args:
            count (int): Number of items to read.
        Returns:
            list: List of tuples (read_value, expected_value) for each read item.
        """
        read_index = 0
        read_items = []

        while read_index < count:
            await RisingEdge(self.dut.rd_clk) # Wait for clock edge
            await ReadWrite() # Allow 'empty' to settle after the edge

            if self.dut.empty.value == 1:
                self.dut._log.info("Want to read but FIFO is empty. Will try again in the next read cycle.")
                self.dut.rd_en.value = 0 # Keep rd_en de-asserted if empty
            else:
                # FIFO is not empty, proceed with read
                self.dut.rd_en.value = 1 # Assert rd_en for this cycle
                await ReadOnly() # Wait for combinational logic for rd_data to settle
                
                read_val = self.dut.rd_data.value
                expected_val = self.expected_data_q.popleft() # Pop from expected queue
                read_items.append((int(read_val), int(expected_val)))
                self.dut._log.info(f"Reading data. Expected: 0x{expected_val:X}, Actual: 0x{int(read_val):X}")
                read_index += 1 # Advance read count
        
        # After all reads are complete, ensure rd_en is de-asserted.
        # Wait one more cycle to ensure the last asserted rd_en takes effect.
        await RisingEdge(self.dut.rd_clk)
        self.dut.rd_en.value = 0
        return read_items
    
    async def read_side_flag_monitor(self):
        """
        Monitors and checks the FIFO read side flags (empty, almost empty) depending on fifo_count_rd_clk.
        """
        while True:
            await RisingEdge(self.dut.rd_clk)
            await ReadOnly()

            if int(self.dut.fifo_count_rd_clk.value) == 0:
                assert self.dut.empty.value == 1, "FIFO should be empty when fifo_count_rd_clk is 0"
                assert self.dut.almost_empty.value == 1, "FIFO should be almost empty when fifo_count_rd_clk is 0"

            if int(self.dut.fifo_count_rd_clk.value) == 1:
                assert self.dut.empty.value == 0, "FIFO should not be empty when fifo_count_rd_clk is 1"

            if int(self.dut.fifo_count_rd_clk.value) <= int(self.ALMOST_EMPTY_THRESHOLD):
                assert self.dut.almost_empty.value == 1, "FIFO should be almost empty when fifo_count_rd_clk is less than almost empty threshold"

            if int(self.dut.fifo_count_rd_clk.value) > int(self.ALMOST_EMPTY_THRESHOLD):
                assert self.dut.almost_empty.value == 0, "FIFO should not be almost empty when fifo_count_rd_clk is greater than or equal to almost empty threshold"
                assert self.dut.empty.value == 0, "FIFO should not be empty when fifo_count_rd_clk is greater than or equal to almost empty threshold"

    async def write_side_flag_monitor(self):
        """
        Monitors and checks the FIFO write side flags (full, almost full) depending on fifo_count_wr_clk.
        """
        while True:
            await RisingEdge(self.dut.wr_clk)
            await ReadOnly()

            if int(self.dut.fifo_count_wr_clk.value) == int(self.FIFO_DEPTH):
                assert self.dut.full.value == 1, "FIFO should be full when fifo_count_wr_clk is equal to FIFO depth"
                assert self.dut.almost_full.value == 1, "FIFO should be almost full when fifo_count_wr_clk is equal to FIFO depth"

            if int(self.dut.fifo_count_wr_clk.value) == int(self.FIFO_DEPTH) - 1:
                assert self.dut.full.value == 0, "FIFO should not be full when fifo_count_wr_clk is one less than FIFO depth"

            if int(self.dut.fifo_count_wr_clk.value) >= int(self.FIFO_DEPTH) - int(self.ALMOST_FULL_THRESHOLD):
                assert self.dut.almost_full.value == 1, "FIFO should be almost full when fifo_count_wr_clk is greater than or equal to almost full threshold"

            if int(self.dut.fifo_count_wr_clk.value) < int(self.FIFO_DEPTH) - int(self.ALMOST_FULL_THRESHOLD):
                assert self.dut.almost_full.value == 0, "FIFO should not be almost full when fifo_count_wr_clk is less than almost full threshold"
                assert self.dut.full.value == 0, "FIFO should not be full when fifo_count_wr_clk is less than almost full threshold"

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

                           
    