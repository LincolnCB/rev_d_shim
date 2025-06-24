import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, ReadWrite, Join
import random
from shim_shutdown_sense_base import shim_shutdown_sense_base
from shim_shutdown_sense_coverage import start_coverage_monitor

async def setup_testbench(dut, clk_period=4, time_unit='ns'):
    tb = shim_shutdown_sense_base(dut, clk_period, time_unit)
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

    for _ in range(8):
        await RisingEdge(dut.clk)
        dut.shutdown_sense_en.value = 1
        dut.shutdown_sense_pin.value = 1

    # Give time to finish monitoring and kill the task
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    monitor_and_scoreboard_task.kill()

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

        for j in range(8):
            await RisingEdge(dut.clk)
            dut.shutdown_sense_en.value = 1
            if j != i:
                dut.shutdown_sense_pin.value = 0
            else:
                dut.shutdown_sense_pin.value = 1
                continue


    # Give time to finish monitoring and kill the task
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    monitor_and_scoreboard_task.kill()

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

    for _ in range(200):
        await RisingEdge(dut.clk)
        # Make 0 more rare for shutdown_sense_en (e.g., 10% chance for 0, 90% for 1)
        dut.shutdown_sense_en.value = 0 if random.random() < 0.1 else 1
        dut.shutdown_sense_pin.value = random.choice([0, 1])

    # Give time to finish monitoring and kill the task
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    monitor_and_scoreboard_task.kill()

