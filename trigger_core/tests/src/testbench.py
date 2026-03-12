import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, ReadWrite
import random

from trigger_core_base import trigger_core_base

async def setup_testbench(dut, clk_period=4, time_unit='ns'):
    tb = trigger_core_base(dut, clk_period, time_unit)
    return tb

# DIRECTED TESTS
@cocotb.test()
async def test_reset(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_reset")

    # Reset the DUT
    await tb.reset()

    # Give time before ending the test
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)

@cocotb.test()
async def test_set_lockout_cmd(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_set_lockout_cmd")

    # First have the DUT at a known state
    await tb.reset()

    # Start monitor_cmd_done and monitor_state_transitions tasks
    monitor_cmd_done_task = cocotb.start_soon(tb.monitor_cmd_done())
    monitor_state_transitions_task = cocotb.start_soon(tb.monitor_state_transitions())

    # Actual Reset
    await tb.reset()

    cmd_list = []

    # Command to set lockout with a valid value
    cmd_list.append(tb.command_word_generator(2, tb.TRIGGER_LOCKOUT_MIN))

    # Command to set lockout with an invalid value
    cmd_list.append(tb.command_word_generator(2, tb.TRIGGER_LOCKOUT_MIN -1))

    # Start the command buffer model, data buffer model and trig timer tracker
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    data_buf_task = cocotb.start_soon(tb.data_buf_model())
    trig_timer_task = cocotb.start_soon(tb.trig_timer_tracker())

    # Start the scoreboard to monitor command execution
    scoreboard_executing_cmd_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_list)))

    # Send the commands to the command buffer
    await tb.send_commands(cmd_list)
    await scoreboard_executing_cmd_task

    # Start data buffer scoreboard to check the expected trigger timing data
    data_buf_scoreboard_task = cocotb.start_soon(tb.data_buf_scoreboard())
    await data_buf_scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.kill()
    monitor_cmd_done_task.kill()
    monitor_state_transitions_task.kill()
    scoreboard_executing_cmd_task.kill()
    trig_timer_task.kill()
    data_buf_task.kill()
    data_buf_scoreboard_task.kill()

@cocotb.test()
async def test_sync_ch_cmd(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_sync_ch_cmd")

    # First have the DUT at a known state
    await tb.reset()

    # Start monitor_cmd_done and monitor_state_transitions tasks
    monitor_cmd_done_task = cocotb.start_soon(tb.monitor_cmd_done())
    monitor_state_transitions_task = cocotb.start_soon(tb.monitor_state_transitions())

    # Actual Reset
    await tb.reset()

    cmd_list = []
    # Command to sync channels
    cmd_list.append(tb.command_word_generator(1, 0))

    # Start the command buffer model, data buffer model and trig timer tracker
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    data_buf_task = cocotb.start_soon(tb.data_buf_model())
    trig_timer_task = cocotb.start_soon(tb.trig_timer_tracker())

    # Start the scoreboard to monitor command execution
    scoreboard_executing_cmd_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_list)))

    # Send the commands to the command buffer
    await tb.send_commands(cmd_list)

    # Drive dac and adc waiting signals to all 1 to allow SYNC_CH to complete after some time
    for _ in range(20):
        await RisingEdge(dut.clk)

    tb.dut.dac_waiting_for_trig.value = 0xFF
    tb.dut.adc_waiting_for_trig.value = 0xFF

    await scoreboard_executing_cmd_task

    # Start data buffer scoreboard to check the expected trigger timing data
    data_buf_scoreboard_task = cocotb.start_soon(tb.data_buf_scoreboard())
    await data_buf_scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.kill()
    monitor_cmd_done_task.kill()
    monitor_state_transitions_task.kill()
    scoreboard_executing_cmd_task.kill()
    trig_timer_task.kill()
    data_buf_task.kill()
    data_buf_scoreboard_task.kill()

@cocotb.test()
async def test_expect_ext_trig_cmd(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_expect_ext_trig_cmd")

    # First have the DUT at a known state
    await tb.reset()

    # Start monitor_cmd_done and monitor_state_transitions tasks
    monitor_cmd_done_task = cocotb.start_soon(tb.monitor_cmd_done())
    monitor_state_transitions_task = cocotb.start_soon(tb.monitor_state_transitions())

    # Actual Reset
    await tb.reset()
    
    cmd_list = []

    # Command to set trig_lockout with a valid value
    cmd_list.append(tb.command_word_generator(2, tb.TRIGGER_LOCKOUT_MIN + 5))

    # Command to expect external trigger
    cmd_list.append(tb.command_word_generator(3, 5))

    # Another expect external trigger command to test multiple in a row
    cmd_list.append(tb.command_word_generator(3, 3))

    # Command to set external trigger with 0 value (edge case)
    cmd_list.append(tb.command_word_generator(3, 0))

    # Start the command buffer model, data buffer model and trig timer tracker
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    data_buf_task = cocotb.start_soon(tb.data_buf_model())
    trig_timer_task = cocotb.start_soon(tb.trig_timer_tracker())

    # Start the scoreboard to monitor command execution
    scoreboard_executing_cmd_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_list)))

    # Send the commands to the command buffer
    await tb.send_commands(cmd_list)

    # Drive external trigger signal after some time
    for _ in range(10):
        await RisingEdge(dut.clk)

    tb.dut.ext_trig.value = 1

    await scoreboard_executing_cmd_task

    # Start data buffer scoreboard to check the expected trigger timing data
    data_buf_scoreboard_task = cocotb.start_soon(tb.data_buf_scoreboard())
    await data_buf_scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.kill()
    monitor_cmd_done_task.kill()
    monitor_state_transitions_task.kill()
    scoreboard_executing_cmd_task.kill()
    trig_timer_task.kill()
    data_buf_task.kill()
    data_buf_scoreboard_task.kill()

@cocotb.test()
async def test_delay_cmd(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_delay_cmd")

    # First have the DUT at a known state
    await tb.reset()

    # Start monitor_cmd_done and monitor_state_transitions tasks
    monitor_cmd_done_task = cocotb.start_soon(tb.monitor_cmd_done())
    monitor_state_transitions_task = cocotb.start_soon(tb.monitor_state_transitions())

    # Actual Reset
    await tb.reset()

    cmd_list = []

    # Set delay to 20 clock cycles
    cmd_list.append(tb.command_word_generator(4, 20))

    # Set delay to 10 clock cycles (edge case)
    cmd_list.append(tb.command_word_generator(4, 10))

    # Set delay to 0 clock cycles (edge case)
    cmd_list.append(tb.command_word_generator(4, 0))

    # Start the command buffer model
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())

    # Start the scoreboard to monitor command execution
    scoreboard_executing_cmd_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_list)))

    # Send the commands to the command buffer
    await tb.send_commands(cmd_list)

    await scoreboard_executing_cmd_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.kill()
    monitor_cmd_done_task.kill()
    monitor_state_transitions_task.kill()
    scoreboard_executing_cmd_task.kill()

@cocotb.test()
async def test_force_trig_cmd(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_force_trig_cmd")

    # First have the DUT at a known state
    await tb.reset()

    # Start monitor_cmd_done and monitor_state_transitions tasks
    monitor_cmd_done_task = cocotb.start_soon(tb.monitor_cmd_done())
    monitor_state_transitions_task = cocotb.start_soon(tb.monitor_state_transitions())

    # Actual Reset
    await tb.reset()

    cmd_list = []

    # Command to force trigger
    cmd_list.append(tb.command_word_generator(5, 0))

    # Start the command buffer model, data buffer model and trig timer tracker
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    data_buf_task = cocotb.start_soon(tb.data_buf_model())
    trig_timer_task = cocotb.start_soon(tb.trig_timer_tracker())

    # Start the scoreboard to monitor command execution
    scoreboard_executing_cmd_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_list)))

    # Send the commands to the command buffer
    await tb.send_commands(cmd_list)

    await scoreboard_executing_cmd_task

    # Start data buffer scoreboard to check the expected trigger timing data
    data_buf_scoreboard_task = cocotb.start_soon(tb.data_buf_scoreboard())
    await data_buf_scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.kill()
    monitor_cmd_done_task.kill()
    monitor_state_transitions_task.kill()
    scoreboard_executing_cmd_task.kill()
    trig_timer_task.kill()
    data_buf_task.kill()
    data_buf_scoreboard_task.kill()

@cocotb.test()
async def test_cancel_cmd(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_cancel_cmd")

    # First have the DUT at a known state
    await tb.reset()

    # Start monitor_cmd_done and monitor_state_transitions tasks
    monitor_cmd_done_task = cocotb.start_soon(tb.monitor_cmd_done())
    monitor_state_transitions_task = cocotb.start_soon(tb.monitor_state_transitions())

    # Actual Reset
    await tb.reset()

    cmd_list = []

    # Command to send delay of 50 clock cycles
    cmd_list.append(tb.command_word_generator(4, 50))

    # Start the command buffer model, data buffer model and trig timer tracker
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    data_buf_task = cocotb.start_soon(tb.data_buf_model())
    trig_timer_task = cocotb.start_soon(tb.trig_timer_tracker())

    # Start the scoreboard to monitor command execution
    scoreboard_executing_cmd_task = cocotb.start_soon(tb.executing_command_scoreboard(2))

    # Send the commands to the command buffer
    await tb.send_commands(cmd_list)

    # Wait for some time and then send cancel command
    for _ in range(10):
        await RisingEdge(dut.clk)

    cmd_list.clear()
    cmd_list.append(tb.command_word_generator(7, 0))
    await tb.send_commands(cmd_list)

    await scoreboard_executing_cmd_task

    # Start data buffer scoreboard to check the expected trigger timing data
    data_buf_scoreboard_task = cocotb.start_soon(tb.data_buf_scoreboard())
    await data_buf_scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.kill()
    monitor_cmd_done_task.kill()
    monitor_state_transitions_task.kill()
    scoreboard_executing_cmd_task.kill()
    trig_timer_task.kill()
    data_buf_task.kill()
    data_buf_scoreboard_task.kill()


@cocotb.test()
async def test_unexpected_cmd(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_unexpected_cmd")

    # First have the DUT at a known state
    await tb.reset()

    # Start monitor_cmd_done and monitor_state_transitions tasks
    monitor_cmd_done_task = cocotb.start_soon(tb.monitor_cmd_done())
    monitor_state_transitions_task = cocotb.start_soon(tb.monitor_state_transitions())

    # Actual Reset
    await tb.reset()

    cmd_list = []

    # Command with an unexpected command type (e.g., 6)
    cmd_list.append(tb.command_word_generator(6, 0x1234567))

    # Start the command buffer model, data buffer model and trig timer tracker
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    data_buf_task = cocotb.start_soon(tb.data_buf_model())
    trig_timer_task = cocotb.start_soon(tb.trig_timer_tracker())

    # Start the scoreboard to monitor command execution
    scoreboard_executing_cmd_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_list)))

    # Send the commands to the command buffer
    await tb.send_commands(cmd_list)

    await scoreboard_executing_cmd_task

    # Start data buffer scoreboard to check the expected trigger timing data
    data_buf_scoreboard_task = cocotb.start_soon(tb.data_buf_scoreboard())
    await data_buf_scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.kill()
    monitor_cmd_done_task.kill()
    monitor_state_transitions_task.kill()
    scoreboard_executing_cmd_task.kill()
    trig_timer_task.kill()
    data_buf_task.kill()
    data_buf_scoreboard_task.kill()

@cocotb.test()
async def test_back_to_back_force_trig_cmd(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_back_to_back_force_trig_cmd")

    # First have the DUT at a known state
    await tb.reset()

    # Start monitor_cmd_done and monitor_state_transitions tasks
    monitor_cmd_done_task = cocotb.start_soon(tb.monitor_cmd_done())
    monitor_state_transitions_task = cocotb.start_soon(tb.monitor_state_transitions())

    # Actual Reset
    await tb.reset()

    cmd_list = []
    
    for _i in range(100):
        # Command to force trigger
        cmd_list.append(tb.command_word_generator(5, 0))

    # Start the command buffer model, data buffer model and trig timer tracker
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    data_buf_task = cocotb.start_soon(tb.data_buf_model())
    trig_timer_task = cocotb.start_soon(tb.trig_timer_tracker())

    # Start the scoreboard to monitor command execution
    scoreboard_executing_cmd_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_list)))

    # Send the commands to the command buffer
    await tb.send_commands(cmd_list)

    await scoreboard_executing_cmd_task

    # Start data buffer scoreboard to check the expected trigger timing data
    data_buf_scoreboard_task = cocotb.start_soon(tb.data_buf_scoreboard())
    await data_buf_scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.kill()
    monitor_cmd_done_task.kill()
    monitor_state_transitions_task.kill()
    scoreboard_executing_cmd_task.kill()
    trig_timer_task.kill()
    data_buf_task.kill()
    data_buf_scoreboard_task.kill()

@cocotb.test()
async def test_random_cmd_sequence(dut):
    seed = 1234
    random.seed(seed)
    dut._log.info(f"Random seed: {seed}")

    for i in range(100):
        tb = await setup_testbench(dut)
        tb.dut._log.info(f"STARTING TEST: test_random_cmd_sequence ITERATION: {i+1}")

        # First have the DUT at a known state
        await tb.reset()

        # Start monitor_cmd_done and monitor_state_transitions tasks
        monitor_cmd_done_task = cocotb.start_soon(tb.monitor_cmd_done())
        monitor_state_transitions_task = cocotb.start_soon(tb.monitor_state_transitions())

        # Actual Reset
        await tb.reset()

        # Generate a random sequence of 50 commands
        cmd_list = tb.random_command_word_generator(50)

        # Start the command buffer model, data buffer model and trig timer tracker
        await RisingEdge(dut.clk)
        cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
        data_buf_task = cocotb.start_soon(tb.data_buf_model())
        trig_timer_task = cocotb.start_soon(tb.trig_timer_tracker())

        # Randomly drive dac/adc waiting signals during the test
        waiting_for_trig_task = cocotb.start_soon(tb.random_waiting_for_trig_driver())

        # Randomly drive ext_trig signal during the test
        ext_trig_task = cocotb.start_soon(tb.random_ext_trig_driver())

        # Start the scoreboard to monitor command execution
        scoreboard_executing_cmd_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_list)))

        # Send the commands to the command buffer
        await tb.send_commands(cmd_list)

        await scoreboard_executing_cmd_task

        # Start data buffer scoreboard to check the expected trigger timing data
        data_buf_scoreboard_task = cocotb.start_soon(tb.data_buf_scoreboard())
        await data_buf_scoreboard_task

        # Give time before ending the test and ensure we don't collide with other tests
        await RisingEdge(dut.clk)
        await RisingEdge(dut.clk)
        await RisingEdge(dut.clk)
        await RisingEdge(dut.clk)
        cmd_buf_task.kill()
        monitor_cmd_done_task.kill()
        monitor_state_transitions_task.kill()
        scoreboard_executing_cmd_task.kill()
        trig_timer_task.kill()
        data_buf_task.kill()
        data_buf_scoreboard_task.kill()
        waiting_for_trig_task.kill()
        ext_trig_task.kill()
