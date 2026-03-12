import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, ReadWrite, Combine
import random
from sync_coherent_base import sync_coherent_base

async def setup_testbench(dut, in_clk_period=4, out_clk_period=4, time_unit="ns"):
    tb = sync_coherent_base(dut, in_clk_period, out_clk_period, time_unit)
    return tb

@cocotb.test()
async def test_prev_din_after_reset(dut):
    in_clk_period = 4
    out_clk_period = 4
    tb = await setup_testbench(dut, in_clk_period, out_clk_period)
    tb.dut._log.info("STARTING TEST: Previous din after reset")
    await tb.start_clocks()

    # Perform reset
    in_side_reset_task1 = cocotb.start_soon(tb.in_side_reset())
    out_side_reset_task1 = cocotb.start_soon(tb.out_side_reset())
    prev_din_and_wr_en_scoreboard_task = cocotb.start_soon(tb.prev_din_and_wr_en_scoreboard())

    await in_side_reset_task1
    await out_side_reset_task1

    # Drive din with known values and check dout
    # Last data written will be 10
    din_driver_task1 = cocotb.start_soon(tb.static_din_driver_and_monitor(cycles=10, initial_data=1))
    dout_scoreboard_task1 = cocotb.start_soon(tb.dout_scoreboard())

    await din_driver_task1
    await dout_scoreboard_task1

    # Perform reset
    in_side_reset_task2 = cocotb.start_soon(tb.in_side_reset())
    out_side_reset_task2 = cocotb.start_soon(tb.out_side_reset())

    await in_side_reset_task2
    await out_side_reset_task2

    # Now start driving with prev_din after reset
    din_driver_task2 = cocotb.start_soon(tb.static_din_driver_and_monitor(cycles=10, initial_data=10, expect_dummy=False))
    dout_scoreboard_task2 = cocotb.start_soon(tb.dout_scoreboard())

    await din_driver_task2
    await dout_scoreboard_task2

    # Ensure we don't collide with other tests
    await RisingEdge(dut.in_clk)
    await RisingEdge(dut.out_clk)
    await RisingEdge(dut.in_clk)
    await RisingEdge(dut.out_clk)
    await tb.kill_clocks()
    din_driver_task1.kill()
    dout_scoreboard_task1.kill()
    din_driver_task2.kill()
    dout_scoreboard_task2.kill()
    prev_din_and_wr_en_scoreboard_task.kill()

@cocotb.test()
async def test_faster_in_clk(dut):
    # Random seed
    seed = 1234
    random.seed(seed)
    dut._log.info(f"Random seed set to: {seed}")

    in_clk_period = 4 # 250MHz
    out_clk_period = 10 # 100MHz

    tb = await setup_testbench(dut, in_clk_period, out_clk_period)
    tb.dut._log.info(f"STARTING TEST: Test Faster In Clk")
    await tb.start_clocks()

    # Perform reset
    in_side_reset_task = cocotb.start_soon(tb.in_side_reset())
    out_side_reset_task = cocotb.start_soon(tb.out_side_reset())
    prev_din_and_wr_en_scoreboard_task = cocotb.start_soon(tb.prev_din_and_wr_en_scoreboard())

    await in_side_reset_task
    await out_side_reset_task

    # Start coroutines
    din_driver_and_monitor_task = cocotb.start_soon(tb.random_din_driver_and_monitor(cycles=50))
    dout_scoreboard_task = cocotb.start_soon(tb.dout_scoreboard())

    # Wait for coroutines to complete
    await din_driver_and_monitor_task
    await dout_scoreboard_task

    # Ensure we don't collide with the new tests
    await RisingEdge(dut.in_clk)
    await RisingEdge(dut.out_clk)
    await RisingEdge(dut.in_clk)
    await RisingEdge(dut.out_clk)
    await tb.kill_clocks()
    din_driver_and_monitor_task.kill()
    dout_scoreboard_task.kill()
    prev_din_and_wr_en_scoreboard_task.kill()

@cocotb.test()
async def test_slower_in_clk(dut):
    # Random seed
    seed = 1234
    random.seed(seed)
    dut._log.info(f"Random seed set to: {seed}")

    in_clk_period = 10 # 100MHz
    out_clk_period = 4 # 250MHz

    tb = await setup_testbench(dut, in_clk_period, out_clk_period)
    tb.dut._log.info(f"STARTING TEST: Test Slower In Clk")
    await tb.start_clocks()

    # Perform reset
    in_side_reset_task = cocotb.start_soon(tb.in_side_reset())
    out_side_reset_task = cocotb.start_soon(tb.out_side_reset())
    prev_din_and_wr_en_scoreboard_task = cocotb.start_soon(tb.prev_din_and_wr_en_scoreboard())

    await in_side_reset_task
    await out_side_reset_task

    # Start coroutines
    din_driver_and_monitor_task = cocotb.start_soon(tb.random_din_driver_and_monitor(cycles=50))
    dout_scoreboard_task = cocotb.start_soon(tb.dout_scoreboard())

    # Wait for coroutines to complete
    await din_driver_and_monitor_task
    await dout_scoreboard_task

    # Ensure we don't collide with the new tests
    await RisingEdge(dut.in_clk)
    await RisingEdge(dut.out_clk)
    await RisingEdge(dut.in_clk)
    await RisingEdge(dut.out_clk)
    await tb.kill_clocks()
    din_driver_and_monitor_task.kill()
    dout_scoreboard_task.kill()
    prev_din_and_wr_en_scoreboard_task.kill()

@cocotb.test()
async def test_random(dut):
    # Random seed
    seed = 1234
    random.seed(seed)
    dut._log.info(f"Random seed set to: {seed}")

    # Test Iteration
    for i in range(10):
        in_clk_period = random.randint(4, 20)
        out_clk_period = random.randint(4, 20)

        tb = await setup_testbench(dut, in_clk_period, out_clk_period)
        tb.dut._log.info(f"STARTING TEST: Random Tests Iteration: {i+1}")
        await tb.start_clocks()

        # Perform reset
        in_side_reset_task = cocotb.start_soon(tb.in_side_reset())
        out_side_reset_task = cocotb.start_soon(tb.out_side_reset())
        prev_din_and_wr_en_scoreboard_task = cocotb.start_soon(tb.prev_din_and_wr_en_scoreboard())

        await in_side_reset_task
        await out_side_reset_task

        # Start coroutines
        din_driver_and_monitor_task = cocotb.start_soon(tb.random_din_driver_and_monitor(cycles=100))
        dout_scoreboard_task = cocotb.start_soon(tb.dout_scoreboard())

        # Wait for coroutines to complete
        await din_driver_and_monitor_task
        await dout_scoreboard_task

        # Ensure we don't collide with the new iteration
        await RisingEdge(dut.in_clk)
        await RisingEdge(dut.out_clk)
        await RisingEdge(dut.in_clk)
        await RisingEdge(dut.out_clk)
        await tb.kill_clocks()
        din_driver_and_monitor_task.kill()
        dout_scoreboard_task.kill()
        prev_din_and_wr_en_scoreboard_task.kill()


