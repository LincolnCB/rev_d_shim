import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, ReadWrite
import random
from shutdown_sense_base import shutdown_sense_base
from shutdown_sense_coverage import start_coverage_monitor

async def setup_testbench(dut, connected=0b11111111, clk_period=4, time_unit='ns'):
    tb = shutdown_sense_base(dut, connected, clk_period, time_unit)
    return tb

# Test to drive shutdown_sense to all high one by one
@cocotb.test()
async def test_latch_all_high(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_latch_all_high")

    # Start the coverage monitor
    start_coverage_monitor(dut)

    # Have the DUT at a known state for each test
    await tb.shutdown_sense_en_drive_low()

    # Then start the monitor and scoreboard task
    monitor_and_scoreboard_task = cocotb.start_soon(tb.monitor_and_scoreboard())

    for _ in range(8 * tb.DWELL_CYCLES):
        await RisingEdge(dut.clk)
        dut.shutdown_sense_en.value = 1
        dut.shutdown_sense_pin.value = 1

    # Give time to finish monitoring and kill the task
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    monitor_and_scoreboard_task.cancel()
    assert int(dut.shutdown_sense_o.value) == 0xFF, f"Expected every board latched, got {int(dut.shutdown_sense_o.value):08b}"

# Test to drive each single bit in shutdown_sense to high one by one
@cocotb.test()
async def test_drive_single_bit_high(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_drive_single_bit_high")

    # Start the coverage monitor
    start_coverage_monitor(dut)

    # Have the DUT at a known state for each test
    await tb.shutdown_sense_en_drive_low()

    # Then start the monitor and scoreboard task
    monitor_and_scoreboard_task = cocotb.start_soon(tb.monitor_and_scoreboard())

    for i in range(8):
        await tb.shutdown_sense_en_drive_low()

        # Hold the pin high only through board i's select window
        for j in range(8):
            for _ in range(tb.DWELL_CYCLES):
                await RisingEdge(dut.clk)
                dut.shutdown_sense_en.value = 1
                dut.shutdown_sense_pin.value = 1 if j == i else 0

        await RisingEdge(dut.clk)
        await ReadOnly()
        assert int(dut.shutdown_sense_o.value) == (1 << i), f"Expected only board {i} latched, got {int(dut.shutdown_sense_o.value):08b}"
        await RisingEdge(dut.clk)

    # Give time to finish monitoring and kill the task
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    monitor_and_scoreboard_task.cancel()

# Test to confirm no latching when shutdown_sense_connected is disabled for each channel
@cocotb.test()
async def test_no_latching_when_disabled(dut):
    tb = await setup_testbench(dut, connected=0b00000000)
    tb.dut._log.info("STARTING TEST: test_no_latching_when_disabled")

    # Start the coverage monitor
    start_coverage_monitor(dut)

    # Have the DUT at a known state for each test
    await tb.shutdown_sense_en_drive_low()

    tb.dut._log.info(f"Shutdown sense connected == {int(dut.shutdown_sense_connected.value):08b}")
    tb.dut._log.info(f"Shutdown sense at start  == {int(dut.shutdown_sense_o.value):08b}")
    for i in range(8 * tb.DWELL_CYCLES):
        await RisingEdge(dut.clk)
        dut.shutdown_sense_en.value = 1
        dut.shutdown_sense_pin.value = 1
        # No latching should occur since all channels are disabled
        assert int(dut.shutdown_sense_o.value) == 0, "Expected no latching when all channels are disabled."

# Test numerous random operations
@cocotb.test()
async def test_random_operations(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_random_operations")

    # Start the coverage monitor
    start_coverage_monitor(dut)

    # Have the DUT at a known state for each test
    await tb.shutdown_sense_en_drive_low()

    # Then start the monitor and scoreboard task
    monitor_and_scoreboard_task = cocotb.start_soon(tb.monitor_and_scoreboard())

    # Enable drops rarely enough that full scans (8 * DWELL_CYCLES) still complete
    for _ in range(10 * tb.DWELL_CYCLES * 8):
        await RisingEdge(dut.clk)
        dut.shutdown_sense_en.value = 0 if random.random() < 0.002 else 1
        dut.shutdown_sense_pin.value = 0 if random.random() < 0.9 else 1

    # Give time to finish monitoring and kill the task
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    monitor_and_scoreboard_task.cancel()

