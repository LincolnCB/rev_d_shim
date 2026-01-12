import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, ReadWrite, Join
import random
from fifo_async_base import fifo_async_base
from fifo_async_coverage import start_coverage_monitor

async def setup_testbench(dut, rd_clk_period=4, wr_clk_period=4):
    tb = fifo_async_base(dut, rd_clk_period, wr_clk_period, time_unit="ns")
    return tb

# Test for FIFO reset with different clock periods, FIFO should be empty after reset and not full
@cocotb.test()
async def test_fifo_async_reset(dut):
    for i in range(10):
        rd_clk_period = random.randint(4, 20)
        wr_clk_period = random.randint(4, 20)

        tb = await setup_testbench(dut, rd_clk_period, wr_clk_period)
        tb.dut._log.info(f"STARTING TEST: Async FIFO Reset Iteration: {i+1}")
        await tb.start_clocks()
        start_coverage_monitor(dut)  # Start coverage monitoring

        # Perform reset
        wr_reset_task = cocotb.start_soon(tb.wr_reset())
        rd_reset_task = cocotb.start_soon(tb.rd_reset())
        await wr_reset_task
        await rd_reset_task

        # Verify FIFO state after reset
        await ReadOnly()
        assert dut.empty.value == 1, "FIFO should be empty after reset"
        assert dut.full.value == 0, "FIFO should not be full after reset"
        assert dut.almost_empty.value == 1, "FIFO should be almost empty after reset"
        assert dut.almost_full.value == 0, "FIFO should not be almost full after reset"

        # Ensure we don't collide with the new iteration
        await RisingEdge(dut.rd_clk)
        await RisingEdge(dut.wr_clk)
        await tb.kill_clocks()

# Test for basic write and read operations with different clock periods
@cocotb.test()
async def test_fifo_async_basic_write_read(dut):
    for i in range(10):
        rd_clk_period = random.randint(4, 20)
        wr_clk_period = random.randint(4, 20)

        tb = await setup_testbench(dut, rd_clk_period, wr_clk_period)
        tb.dut._log.info(f"STARTING TEST: Async FIFO Basic Write/Read Iteration: {i+1}")
        await tb.start_clocks()
        start_coverage_monitor(dut)  # Start coverage monitoring

        # Perform reset
        wr_reset_task = cocotb.start_soon(tb.wr_reset())
        rd_reset_task = cocotb.start_soon(tb.rd_reset())
        await wr_reset_task
        await rd_reset_task

        # Randomly generate data to write
        test_data = random.randint(0, tb.MAX_DATA_VALUE)

        # Start write and read tasks
        wr_task = cocotb.start_soon(tb.write(test_data))
        rd_task = cocotb.start_soon(tb.read())

        # Wait for write and read tasks to complete
        await wr_task
        read_data, expected_data = await rd_task
        assert read_data == expected_data, f"Read data {read_data} does not match expected {expected_data} and test data {test_data}"

        await ReadOnly()
        assert dut.empty.value == 1, "FIFO should be empty after read"

        # Ensure we don't collide with the new iteration
        await RisingEdge(dut.rd_clk)
        await RisingEdge(dut.wr_clk)
        await tb.kill_clocks()


# Test FIFO full and empty conditions explicitly, filling the FIFO to capacity and then reading it until it is empty
@cocotb.test()
async def test_full_and_empty_conditions(dut):
    for i in range(10):
        rd_clk_period = random.randint(4, 20)
        wr_clk_period = random.randint(4, 20)

        tb = await setup_testbench(dut, rd_clk_period, wr_clk_period)
        tb.dut._log.info(f"STARTING TEST: Async FIFO Full and Empty Conditions Iteration: {i+1}")
        await tb.start_clocks()
        start_coverage_monitor(dut)  # Start coverage monitoring

        # Perform reset
        wr_reset_task = cocotb.start_soon(tb.wr_reset())
        rd_reset_task = cocotb.start_soon(tb.rd_reset())
        await wr_reset_task
        await rd_reset_task

        # Flag monitors and checkers
        write_side_flag_checker = cocotb.start_soon(tb.write_side_flag_monitor())
        read_side_flag_checker = cocotb.start_soon(tb.read_side_flag_monitor())
        
        # Work on write domain to fill the FIFO
        fill_data = await tb.generate_random_data(tb.FIFO_DEPTH)
        written_count = await tb.write_burst(fill_data)
        assert written_count == tb.FIFO_DEPTH, f"Expected to write {tb.FIFO_DEPTH} items, but wrote {written_count}"

        await ReadOnly()
        assert dut.full.value == 1, "FIFO should be full one clock cycle later after writing maximum items"
        assert dut.empty.value == 0, "FIFO should not be empty after writing items"
        tb.dut._log.info(f"FIFO full status: {dut.full.value}, empty status: {dut.empty.value}")

        # Work on read domain to empty the FIFO
        read_results = await tb.read_burst(tb.FIFO_DEPTH)
        assert len(read_results) == tb.FIFO_DEPTH, f"Expected to read {tb.FIFO_DEPTH} items, but read {len(read_results)}"

        for read_value, expected_value in read_results:
            assert read_value == expected_value, f"Data mismatch: read=0x{read_value:X}, expected=0x{expected_value:X}"

        await ReadOnly()
        assert dut.empty.value == 1, "FIFO should be empty one clock cycle later after reading all items"
        assert dut.full.value == 0, "FIFO should not be full after reading all items"
        tb.dut._log.info(f"FIFO empty status: {dut.empty.value}, full status: {dut.full.value}")

        # Ensure we don't collide with the new iteration
        await RisingEdge(dut.wr_clk)
        await RisingEdge(dut.rd_clk)

        write_side_flag_checker.kill()
        read_side_flag_checker.kill()
        await tb.kill_clocks()

# Test for almost full and almost empty conditions explicitly, filling FIFO to almost full and reading until almost empty
@cocotb.test()
async def test_almost_full_empty_conditions(dut):
    for i in range(10):
        rd_clk_period = random.randint(4, 20)
        wr_clk_period = random.randint(4, 20)

        tb = await setup_testbench(dut, rd_clk_period, wr_clk_period)
        tb.dut._log.info(f"STARTING TEST: Async FIFO Almost Full and Almost Empty Conditions Iteration: {i+1}")
        await tb.start_clocks()
        start_coverage_monitor(dut)  # Start coverage monitoring

        # Perform reset
        wr_reset_task = cocotb.start_soon(tb.wr_reset())
        rd_reset_task = cocotb.start_soon(tb.rd_reset())
        await wr_reset_task
        await rd_reset_task
        
        # Work on write domain to fill FIFO to almost full
        fill_data = await tb.generate_random_data(tb.FIFO_DEPTH - tb.ALMOST_FULL_THRESHOLD)
        written_count = await tb.write_burst(fill_data)
        assert written_count == tb.FIFO_DEPTH - tb.ALMOST_FULL_THRESHOLD, f"Expected to write {tb.FIFO_DEPTH - tb.ALMOST_FULL_THRESHOLD} items, but wrote {written_count}"
        
        await ReadOnly()
        assert dut.almost_full.value == 1, "FIFO should be almost full after writing items"
        assert dut.fifo_count_wr_clk.value == tb.FIFO_DEPTH - tb.ALMOST_FULL_THRESHOLD, "FIFO count should match FIFO_DEPTH - ALMOST_FULL_THRESHOLD"
        assert dut.full.value == 0, "FIFO should not be full after writing items"
        tb.dut._log.info(f"FIFO almost full status: {dut.almost_full.value}, full status: {dut.full.value}")

        # Read from FIFO until almost empty
        read_count = tb.FIFO_DEPTH - tb.ALMOST_EMPTY_THRESHOLD - tb.ALMOST_FULL_THRESHOLD
        read_results = await tb.read_burst(read_count)
        assert len(read_results) == read_count, f"Expected to read {read_count} items, but read {len(read_results)}"

        for read_value, expected_value in read_results:
            assert read_value == expected_value, f"Data mismatch: read=0x{read_value:X}, expected=0x{expected_value:X}"
        
        await ReadOnly()
        assert dut.almost_empty.value == 1, "FIFO should be almost empty after reading items"
        assert dut.fifo_count_rd_clk.value == tb.ALMOST_EMPTY_THRESHOLD, "FIFO count should match ALMOST_EMPTY_THRESHOLD"
        assert dut.empty.value == 0, "FIFO should not be empty after reading items"
        tb.dut._log.info(f"FIFO almost empty status: {dut.almost_empty.value}, empty status: {dut.empty.value}")

        # Ensure we don't collide with the new iteration
        await RisingEdge(dut.wr_clk)
        await RisingEdge(dut.rd_clk)

        await tb.kill_clocks()

# Test for simultaneous read and write operations with different clock periods
@cocotb.test()
async def test_fifo_async_simultaneous_read_write(dut):
    for i in range(10):
        rd_clk_period = random.randint(4, 20)
        wr_clk_period = random.randint(4, 20)

        tb = await setup_testbench(dut, rd_clk_period, wr_clk_period)
        tb.dut._log.info(f"STARTING TEST: Async FIFO Simultaneous Read/Write Iteration: {i+1}")
        await tb.start_clocks()
        start_coverage_monitor(dut)  # Start coverage monitoring

        # Perform reset
        wr_reset_task = cocotb.start_soon(tb.wr_reset())
        rd_reset_task = cocotb.start_soon(tb.rd_reset())
        await wr_reset_task
        await rd_reset_task

        # Flag monitors and checkers, check write and read side flags throughout the test
        write_side_flag_checker = cocotb.start_soon(tb.write_side_flag_monitor())
        read_side_flag_checker = cocotb.start_soon(tb.read_side_flag_monitor())

        # Start simultaneous write and read tasks, number of reads should be equal to the number of writes or it will hang.
        data_list = await tb.generate_random_data(150)
        simultaneous_wr_task = cocotb.start_soon(tb.write_back_to_back(data_list))
        simultaneous_rd_task = cocotb.start_soon(tb.read_back_to_back(150))
        await simultaneous_wr_task
        read_results = await simultaneous_rd_task

        # Verify the read results
        for read_value, expected_value in read_results:
            assert read_value == expected_value, f"Data mismatch: read=0x{read_value:X}, expected=0x{expected_value:X} at index: {read_results.index((read_value, expected_value))}"

        tb.dut._log.info(f"Final FIFO status after iteration {i + 1}:")
        await tb.print_fifo_status()

        # Ensure we don't collide with the new iteration
        await RisingEdge(dut.wr_clk)
        await RisingEdge(dut.rd_clk)

        write_side_flag_checker.kill()
        read_side_flag_checker.kill()
        await tb.kill_clocks()
