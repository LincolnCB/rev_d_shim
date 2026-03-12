import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, ReadWrite
import random
from fifo_sync_base import fifo_sync_base
from fifo_sync_coverage import start_coverage_monitor

# Create a setup function that can be called by each test
async def setup_testbench(dut):
    tb = fifo_sync_base(dut, clk_period=4, time_unit="ns")
    return tb

# Test for FIFO with synchronous reset, FIFO should be empty after reset and not full
@cocotb.test()
async def test_fifo_sync_reset(dut):
    tb = await setup_testbench(dut)
    start_coverage_monitor(dut)  # Start coverage monitoring
    tb.dut._log.info("STARTING TEST: FIFO Synchronous Reset")

    # Perform reset
    await tb.reset()

    # Verify FIFO state after reset
    await ReadOnly()
    assert dut.empty.value == 1, "FIFO should be empty after reset"
    assert dut.full.value == 0, "FIFO should not be full after reset"
    assert dut.almost_empty.value == 1, "FIFO should be almost empty after reset"
    assert dut.almost_full.value == 0, "FIFO should not be almost full after reset"

# Test for basic write and read operation
@cocotb.test()
async def test_basic_write_read(dut):
    tb = await setup_testbench(dut)
    start_coverage_monitor(dut)  # Start coverage monitoring
    await tb.reset()
    tb.dut._log.info("STARTING TEST: Basic Write/Read Operation")
    
    # Test single write/read
    test_data = random.randint(0, tb.MAX_DATA_VALUE)
    success = await tb.write(test_data)
    assert success, "Write should succeed on empty FIFO"
    
    # Read the data back
    read_data, expected_data, success = await tb.read()
    assert success, "Read should succeed on non-empty FIFO"
    assert read_data == expected_data == test_data, f"Data mismatch: read=0x{read_data:X}, expected=0x{expected_data:X}"

    await ReadOnly()
    assert dut.empty.value == 1, "FIFO should be empty after read"

# Direct back to back read after write
@cocotb.test()
async def back_to_back_read_after_write(dut):
    tb = await setup_testbench(dut)
    start_coverage_monitor(dut)  # Start coverage monitoring
    await tb.reset()
    tb.dut._log.info("STARTING TEST: Back-to-Back Read After Write")
    
    test_data = random.randint(0, tb.MAX_DATA_VALUE)
    
    # Write single data item
    await RisingEdge(dut.clk)
    dut.wr_data.value = test_data
    dut.wr_en.value = 1
    tb.dut._log.info(f"Writing data: 0x{test_data:X}")
    
    # Read data will be available after two clock cycles
    await RisingEdge(dut.clk)
    dut.wr_en.value = 0

    await ReadOnly()
    assert dut.empty.value == 0, "FIFO should not be empty after write"

    # When rd_en is asserted, after combinational logic settles, read data should be available due to FWFT.
    await RisingEdge(dut.clk)
    dut.rd_en.value = 1  # Enable read

    await ReadOnly() # Wait for combinational logic to settle
    read_data = int(dut.rd_data.value)
    tb.dut._log.info(f"Read data: 0x{read_data:X}")
    assert read_data == test_data, f"Data mismatch: read=0x{read_data:X}, expected=0x{test_data:X}"

    # Empty is asserted after one clock cycle later after last read
    await RisingEdge(dut.clk)
    await ReadOnly()
    tb.dut._log.info(f"FIFO empty status: {dut.empty.value}")
    assert dut.empty.value == 1, "FIFO should be empty after read"

#Test First Word Fall Through (FWFT) behavior
@cocotb.test()
async def test_fwft_behavior(dut):
    tb = await setup_testbench(dut)
    start_coverage_monitor(dut)  # Start coverage monitoring
    await tb.reset()
    tb.dut._log.info("STARTING TEST: First Word Fall Through (FWFT) Behavior")
    
    # Write first data item
    test_data1 = random.randint(0, tb.MAX_DATA_VALUE)
    test_data2 = random.randint(0, tb.MAX_DATA_VALUE)

    await tb.write(test_data1)
    # In FWFT mode, data should be immediately available on rd_data
    await RisingEdge(dut.clk)
    await ReadWrite()  # Wait for combinational logic to settle
    read_data = int(dut.rd_data.value)
    assert read_data == test_data1, f"FWFT: Data not immediately available. Expected 0x{test_data1:X}, got 0x{read_data:X}"
    
    # Write second data item
    await tb.write(test_data2)
    
    # rd_data should still show first item
    await RisingEdge(dut.clk)
    await ReadWrite()
    read_data = int(dut.rd_data.value)
    assert read_data == test_data1, f"FWFT: First data should still be visible. Expected 0x{test_data1:X}, got 0x{read_data:X}"
    
    # Read first item
    read_data, expected_data, success = await tb.read()
    assert success and read_data == test_data1, "First read failed"
    
    # Second item should now be visible
    await ReadWrite()
    read_data = int(dut.rd_data.value)
    assert read_data == test_data2, f"FWFT: Second data should now be visible. Expected 0x{test_data2:X}, got 0x{read_data:X}"

#Test FIFO full and empty conditions, filling the FIFO to capacity and then reading it until it is empty
@cocotb.test()
async def test_full_and_empty_conditions(dut):
    tb = await setup_testbench(dut)
    start_coverage_monitor(dut)  # Start coverage monitoring
    await tb.reset()
    tb.dut._log.info("STARTING TEST: Full and Empty Conditions")
    
    fill_data = await tb.generate_random_data(tb.FIFO_DEPTH)
    written_count = await tb.write_burst(fill_data)
    assert written_count == tb.FIFO_DEPTH, f"Expected to write {tb.FIFO_DEPTH} items, but wrote {written_count}"

    await ReadOnly()
    assert dut.full.value == 1, "FIFO should be full one clock cycle later after writing maximum items"
    assert dut.empty.value == 0, "FIFO should not be empty after writing items"
    tb.dut._log.info(f"FIFO full status: {dut.full.value}, empty status: {dut.empty.value}")

    read_results = await tb.read_burst(tb.FIFO_DEPTH)
    assert len(read_results) == tb.FIFO_DEPTH, f"Expected to read {tb.FIFO_DEPTH} items, but read {len(read_results)}"

    for read_value, expected_value in read_results:
        assert read_value == expected_value, f"Data mismatch: read=0x{read_value:X}, expected=0x{expected_value:X}"

    await ReadOnly()
    assert dut.empty.value == 1, "FIFO should be empty one clock cycle later after reading all items"
    assert dut.full.value == 0, "FIFO should not be full after reading all items"
    tb.dut._log.info(f"FIFO empty status: {dut.empty.value}, full status: {dut.full.value}")


# Test FIFO almost full and almost empty conditions
@cocotb.test()
async def test_almost_full_empty_conditions(dut):
    tb = await setup_testbench(dut)
    await tb.reset()
    tb.dut._log.info("STARTING TEST: Almost Full and Almost Empty Conditions")
    
    # Fill FIFO to almost full
    fill_data = await tb.generate_random_data(tb.FIFO_DEPTH - tb.ALMOST_FULL_THRESHOLD)
    written_count = await tb.write_burst(fill_data)
    assert written_count == tb.FIFO_DEPTH - tb.ALMOST_FULL_THRESHOLD, f"Expected to write {tb.FIFO_DEPTH - tb.ALMOST_FULL_THRESHOLD} items, but wrote {written_count}"
    
    await ReadOnly()
    assert dut.almost_full.value == 1, "FIFO should be almost full after writing items"
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
    assert dut.empty.value == 0, "FIFO should not be empty after reading items"
    tb.dut._log.info(f"FIFO almost empty status: {dut.almost_empty.value}, empty status: {dut.empty.value}")

# Test simultaneous read and write operations
@cocotb.test()
async def test_random_simultaneous_read_write(dut):
    tb = await setup_testbench(dut)
    start_coverage_monitor(dut)  # Start coverage monitoring

    # Random test setup
    iterations = 20

    for i in range(iterations):
        await tb.reset()
        tb.dut._log.info(f"STARTING TEST: Random Simultaneous Read and Write Operations Iteration: {i + 1}")

        number_of_initial_data = random.randint(2,10)  # Random number of initial data items
        number_of_random_writes = random.randint(50, 300)  # Random number of writes in the burst
        number_of_random_reads = random.randint(50, 300)  # Random number of reads in the burst
        tb.dut._log.info(f"Initial data count: {number_of_initial_data}, Random writes: {number_of_random_writes}, Random reads: {number_of_random_reads}")

        # Write the initial data to the FIFO
        initial_data = await tb.generate_random_data(number_of_initial_data)
        for data in initial_data:
            await tb.write(data)

        # Create random data for additional writes
        random_data = await tb.generate_random_data(number_of_random_writes)

        # Start simultaneous writes
        write_task = cocotb.start_soon(tb.write_burst(random_data))
        
        # Start simultaneous reads 
        read_task = cocotb.start_soon(tb.read_burst(number_of_random_reads))

        # Wait for the write task to complete
        await write_task

        # Wait for the read task to complete
        read_results = await read_task
        
        # Verify the read results
        for read_value, expected_value in read_results:
            #assert read_value in initial_data or read_value in random_data, f"Unexpected read value: 0x{read_value:X}"
            assert read_value == expected_value, f"Data mismatch: read=0x{read_value:X}, expected=0x{expected_value:X} at {read_results.index((read_value, expected_value)) + 1}. read"

        # Final FIFO status
        tb.dut._log.info(f"Final FIFO status after iteration {i + 1}:")
        await tb.print_fifo_status()

# Test simultaneous read and write operations with one initial data in the FIFO
@cocotb.test()
async def test_random_simultaneous_read_write_w_one_initial_data(dut):
    tb = await setup_testbench(dut)
    start_coverage_monitor(dut)  # Start coverage monitoring

    # Random test setup
    iterations = 20

    for i in range(iterations):
        await tb.reset()
        tb.dut._log.info(f"STARTING TEST: Random Simultaneous Read and Write Operations with Initial. Iteration: {i + 1}")

        number_of_random_writes = random.randint(50, 300)  # Random number of writes in the burst
        number_of_random_reads = random.randint(50, 300)  # Random number of reads in the burst
        tb.dut._log.info(f"Initial data count: 0, Random writes: {number_of_random_writes}, Random reads: {number_of_random_reads}")

        await RisingEdge(dut.clk)
        dut.wr_data.value = 0x1234  # Hand-written initial data
        dut.wr_en.value = 1
        tb.dut._log.info(f"Writing initial data: 0x{int(dut.wr_data.value):X}")
        tb.expected_data_q.append(0x1234)  # Add to expected queue immediately

        await RisingEdge(dut.clk)
        dut.wr_en.value = 0

        # Create random data for additional writes
        random_data = await tb.generate_random_data(number_of_random_writes)

        # Start simultaneous writes
        write_task = cocotb.start_soon(tb.write_burst(random_data))

        # Start simultaneous reads 
        await RisingEdge(dut.clk)
        read_task = cocotb.start_soon(tb.read_burst(number_of_random_reads))

        # Wait for the write task to complete
        await write_task

        # Wait for the read task to complete
        read_results = await read_task
        
        # Verify the read results
        for read_value, expected_value in read_results:
            #assert read_value in initial_data or read_value in random_data, f"Unexpected read value: 0x{read_value:X}"
            assert read_value == expected_value, f"Data mismatch: read=0x{read_value:X}, expected=0x{expected_value:X} at {read_results.index((read_value, expected_value)) + 1}th read"

        # Final FIFO status
        tb.dut._log.info(f"Final FIFO status after iteration {i + 1}:")
        await tb.print_fifo_status()