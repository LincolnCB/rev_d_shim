import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, ReadWrite
import random
from threshold_integrator_base import threshold_integrator_base

async def setup_testbench(dut, clk_period=4, time_unit='ns'):
    tb = threshold_integrator_base(dut, clk_period, time_unit)
    return tb

@cocotb.test()
async def test_reset(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_reset")

    # First have the DUT at a known state
    await tb.reset()

    # Then start the monitor and scoreboard tasks
    state_transition_monitor_and_scoreboard_task = cocotb.start_soon(tb.state_transition_monitor_and_scoreboard())

    # Then apply the actual reset
    await tb.reset()

    # Give time to coroutines to finish and kill their tasks
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    state_transition_monitor_and_scoreboard_task.kill()

@cocotb.test()
async def test_idle_to_out_of_bounds(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_idle_to_out_of_bounds")

    # First have the DUT at a known state
    await tb.reset()

    # Then start the monitor and scoreboard tasks
    state_transition_monitor_and_scoreboard_task = cocotb.start_soon(tb.state_transition_monitor_and_scoreboard())

    # Then apply the actual reset
    await tb.reset()

    await RisingEdge(dut.clk)
    await ReadWrite()

    # After reset, DUT should be in IDLE state
    assert int(dut.state.value) == 0, f"Expected state after reset: 0 (IDLE), got: {int(tb.dut.state.value)} ({tb.get_state_name(tb.dut.state.value)})"

    # Enable the DUT
    dut.enable.value = 1

    # Set the window to the highest value that will cause out_of_bounds
    dut.window.value = 2**11 - 1

    # over_thresh should be asserted and DUT should transition to OUT_OF_BOUNDS state
    await RisingEdge(dut.clk)
    await ReadOnly()

    assert int(dut.state.value) == 4, f"Expected state after disallowed window size: 4 (OUT_OF_BOUNDS), got: {int(tb.dut.state.value)} ({tb.get_state_name(tb.dut.state.value)})"
    assert dut.over_thresh.value == 1, "Expected over_thresh to be asserted after disallowed window size" 

    # Give time to coroutines to finish and kill their tasks
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    state_transition_monitor_and_scoreboard_task.kill()


@cocotb.test()
async def test_idle_to_running(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_idle_to_running")

    # First have the DUT at a known state
    await tb.reset()

    # Then start the monitor and scoreboard tasks
    state_transition_monitor_and_scoreboard_task = cocotb.start_soon(tb.state_transition_monitor_and_scoreboard())

    # Then apply the actual reset
    await tb.reset()

    await tb.idle_to_running_state()

    # Give time to coroutines to finish and kill their tasks
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    state_transition_monitor_and_scoreboard_task.kill()

@cocotb.test()
async def test_reset_to_running_to_reset_to_running(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_reset_to_running_to_reset_to_running")

    # First have the DUT at a known state
    await tb.reset()

    # Then start the monitor and scoreboard tasks
    state_transition_monitor_and_scoreboard_task = cocotb.start_soon(tb.state_transition_monitor_and_scoreboard())

    # Then apply the actual reset
    await tb.reset()

    # Transition to RUNNING state
    await tb.idle_to_running_state(window_value=2**13)

    # Reset again while in RUNNING state
    await tb.reset()

    # Transition back to RUNNING state
    await tb.idle_to_running_state(window_value=2**14)

    # Give time to coroutines to finish and kill their tasks
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    state_transition_monitor_and_scoreboard_task.kill()


@cocotb.test(skip=True)
async def print_expected_fifo_overflows(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: print_expected_fifo_overflows")

    for i in range(31, 10, -1):
        tb.dut._log.info(f"i={i}")
        window = 2**(i+1)-1
        await tb.fifo_will_overflow(window)

@cocotb.test()
async def test_running_state_fifo_overflow(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_running_state_fifo_overflow")

    # First have the DUT at a known state
    await tb.reset()

    # Then start the monitor and scoreboard tasks
    state_transition_monitor_and_scoreboard_task = cocotb.start_soon(tb.state_transition_monitor_and_scoreboard())

    # Then apply the actual reset
    await tb.reset()

    window = 8191
    threshold_average = 2**15-1

    await tb.idle_to_running_state(window_value=window, threshold_average_value=threshold_average)

    await tb.running_state_model_and_scoreboard(mode="low_abs_sample_concat_values")

    # Give time to coroutines to finish and kill their tasks
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    state_transition_monitor_and_scoreboard_task.kill()

@cocotb.test()
async def test_running_state_over_threshold(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_running_state_over_threshold")

    # First have the DUT at a known state
    await tb.reset()

    # Then start the monitor and scoreboard tasks
    state_transition_monitor_and_scoreboard_task = cocotb.start_soon(tb.state_transition_monitor_and_scoreboard())

    # Then apply the actual reset
    await tb.reset()

    window = 2**12
    threshold_average = (2**8) - 1

    await tb.idle_to_running_state(window_value=window, threshold_average_value=threshold_average)

    await tb.running_state_model_and_scoreboard(mode="high_abs_sample_concat_values")

    # Give time to coroutines to finish and kill their tasks
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    state_transition_monitor_and_scoreboard_task.kill()

@cocotb.test()
async def test_running_state_w_set_window_and_max_threshold_average(dut):
    for i in range(11, 15):
        tb = await setup_testbench(dut)
        tb.dut._log.info("STARTING TEST: test_running_state_w_set_window_and_max_threshold_average {i}")

        # First have the DUT at a known state
        await tb.reset()

        # Then start the monitor and scoreboard tasks
        state_transition_monitor_and_scoreboard_task = cocotb.start_soon(tb.state_transition_monitor_and_scoreboard())

        # Then apply the actual reset
        await tb.reset()

        window = 2**i
        threshold_average = (2**15) - 1

        await tb.idle_to_running_state(window_value=window, threshold_average_value=threshold_average)

        await tb.running_state_model_and_scoreboard(mode="low_abs_sample_concat_values")

        # Give time to coroutines to finish and kill their tasks
        await RisingEdge(dut.clk)
        await RisingEdge(dut.clk)
        state_transition_monitor_and_scoreboard_task.kill()


@cocotb.test()
async def test_running_state_w_set_window_and_set_threshold_average(dut):
    for i in range(11, 15):
        tb = await setup_testbench(dut)
        tb.dut._log.info("STARTING TEST: test_running_state_w_set_window_and_set_threshold_average {i}")

        # First have the DUT at a known state
        await tb.reset()

        # Then start the monitor and scoreboard tasks
        state_transition_monitor_and_scoreboard_task = cocotb.start_soon(tb.state_transition_monitor_and_scoreboard())

        # Then apply the actual reset
        await tb.reset()

        window = 2**i
        threshold_average = (2**11) - 1

        await tb.idle_to_running_state(window_value=window, threshold_average_value=threshold_average)

        await tb.running_state_model_and_scoreboard(mode="random_abs_sample_concat_values")

        # Give time to coroutines to finish and kill their tasks
        await RisingEdge(dut.clk)
        await RisingEdge(dut.clk)
        state_transition_monitor_and_scoreboard_task.kill()

@cocotb.test()
async def test_running_state_w_random_window_and_random_threshold_average(dut):

    modes = [
        "random_abs_sample_concat_values",
        "max_abs_sample_concat_values",
        "high_abs_sample_concat_values",
        "low_abs_sample_concat_values",
    ]

    for _ in range(10):
        tb = await setup_testbench(dut)
        tb.dut._log.info("STARTING TEST: test_running_state_w_random_window_and_set_threshold_average {i}")

        # First have the DUT at a known state
        await tb.reset()

        # Then start the monitor and scoreboard tasks
        state_transition_monitor_and_scoreboard_task = cocotb.start_soon(tb.state_transition_monitor_and_scoreboard())

        # Then apply the actual reset
        await tb.reset()

        window = random.randint(2**11, 2**15)
        threshold_average = random.randint(1, (2**15) - 1)

        await tb.idle_to_running_state(window_value=window, threshold_average_value=threshold_average)

        selected_mode = random.choice(modes)

        await tb.running_state_model_and_scoreboard(mode=selected_mode)

        # Give time to coroutines to finish and kill their tasks
        await RisingEdge(dut.clk)
        await RisingEdge(dut.clk)
        state_transition_monitor_and_scoreboard_task.kill()

