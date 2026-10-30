import cocotb
import os
from cocotb.triggers import RisingEdge, ReadOnly
import random

from shim_ad5676_dac_ctrl_base import shim_ad5676_dac_ctrl_base

async def setup_testbench(dut, clk_period=4, miso_sck_period=4, time_unit="ns"):
    tb = shim_ad5676_dac_ctrl_base(dut, clk_period, miso_sck_period, time_unit)
    return tb

@cocotb.test()
async def test_reset(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_reset")

    await tb.reset()

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)

@cocotb.test()
async def test_dac_wr(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_dac_wr")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    await tb.reset()

    # Build the DAC_WR command sequence
    cmd_word_list = []
    cmd_word_list.append(tb.build_dac_wr_header(trig_wait=1, cont=0, ldac=1, value=10))
    for _ in range(4):
        value = 1000
        cmd_word_list.append(tb.build_dac_pair(v_lo_chN=value, v_hi_chNp1=value))

    # Start the command buffer model and scoreboard
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_word_list)))

    # The header sets TRIGGER WAIT, so drive triggers to satisfy the post-write wait
    trigger_driver_task = cocotb.start_soon(tb.random_trigger_driver())

    # Send commands and wait for completion
    await tb.send_commands(cmd_word_list)
    await scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.cancel()
    scoreboard_task.cancel()
    transition_monitor_task.cancel()
    trigger_driver_task.cancel()

@cocotb.test()
async def test_dac_wr_ch(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_dac_wr_ch")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    await tb.reset()

    # Build the DAC_WR_CH command sequence
    cmd_word_list = []
    cmd_word_list.append(tb.build_dac_wr_ch(7, 1000))

    # Start the command buffer model and scoreboard
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_word_list)))

    # Send commands and wait for completion
    await tb.send_commands(cmd_word_list)
    await scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.cancel()
    scoreboard_task.cancel()
    transition_monitor_task.cancel()

@cocotb.test()
async def test_zero(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_zero")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    await tb.reset()

    # Build the ZERO command sequence
    cmd_word_list = []
    cmd_word_list.append(tb.build_zero())

    # Start the command buffer model and scoreboard
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_word_list)))

    # Send commands and wait for completion
    await tb.send_commands(cmd_word_list)
    await scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.cancel()
    scoreboard_task.cancel()
    transition_monitor_task.cancel()

@cocotb.test()
async def test_noop_trigger_wait(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_noop_trigger_wait")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    await tb.reset()

    # Build the NO_OP command sequence
    cmd_word_list = []
    cmd_word_list.append(tb.build_noop(trig_wait=1, cont=0, ldac=1, value=10))

    # Start the command buffer model and scoreboard
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_word_list)))

    # Start the random trigger driver
    trigger_driver_task = cocotb.start_soon(tb.random_trigger_driver())

    # Send commands and wait for completion
    await tb.send_commands(cmd_word_list)
    await scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.cancel()
    scoreboard_task.cancel()
    transition_monitor_task.cancel()
    trigger_driver_task.cancel()

@cocotb.test()
async def test_noop_trigger_wait_no_ldac(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_noop_trigger_wait_no_ldac")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    await tb.reset()

    # Build the NO_OP command sequence
    cmd_word_list = []
    cmd_word_list.append(tb.build_noop(trig_wait=1, cont=0, ldac=0, value=10))

    # Start the command buffer model and scoreboard
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_word_list)))

    # Start the random trigger driver
    trigger_driver_task = cocotb.start_soon(tb.random_trigger_driver())

    # Send commands and wait for completion
    await tb.send_commands(cmd_word_list)
    await scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.cancel()
    scoreboard_task.cancel()
    transition_monitor_task.cancel()
    trigger_driver_task.cancel()

@cocotb.test()
async def test_noop_delay(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_noop_delay")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    await tb.reset()

    # Build the NO_OP command sequence
    cmd_word_list = []
    cmd_word_list.append(tb.build_noop(trig_wait=0, cont=0, ldac=1, value=20))

    # Start the command buffer model and scoreboard
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_word_list)))

    # Send commands and wait for completion
    await tb.send_commands(cmd_word_list)
    await scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.cancel()
    scoreboard_task.cancel()
    transition_monitor_task.cancel()

@cocotb.test()
async def test_noop_delay_no_ldac(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_noop_delay_no_ldac")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    await tb.reset()

    # Build the NO_OP command sequence
    cmd_word_list = []
    cmd_word_list.append(tb.build_noop(trig_wait=0, cont=0, ldac=0, value=20))

    # Start the command buffer model and scoreboard
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_word_list)))

    # Send commands and wait for completion
    await tb.send_commands(cmd_word_list)
    await scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.cancel()
    scoreboard_task.cancel()
    transition_monitor_task.cancel()

@cocotb.test()
async def test_set_cal(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_set_cal")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    await tb.reset()

    # Build the SET_CAL command sequence
    cmd_word_list = []
    cmd_word_list.append(tb.build_set_cal(ch=3, cal_signed_16=-1234))

    # Start the command buffer model and scoreboard
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_word_list)))

    # Send commands and wait for completion
    await tb.send_commands(cmd_word_list)
    await scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.cancel()
    scoreboard_task.cancel()
    transition_monitor_task.cancel()

@cocotb.test()
async def test_get_cal(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_get_cal")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    await tb.reset()

    # Build the GET_CAL command sequence
    cmd_word_list = []
    cmd_word_list.append(tb.build_get_cal(ch=3))

    # Start the command buffer model and scoreboard
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_word_list)))

    # Send commands and wait for completion
    await tb.send_commands(cmd_word_list)
    await scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.cancel()
    scoreboard_task.cancel()
    transition_monitor_task.cancel()

@cocotb.test()
async def test_bad_cmd(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_bad_cmd")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    await tb.reset()

    # Build the BAD_CMD command sequence
    cmd_value = 0b110  # Invalid command type
    cmd_word_list = []
    cmd_word_list.append((cmd_value << 29))

    # This test intentionally drives the DUT into S_ERROR.
    tb.expect_error = True

    # Start the command buffer model and scoreboard
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_word_list)))

    # Send commands and wait for completion
    await tb.send_commands(cmd_word_list)
    await scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.cancel()
    scoreboard_task.cancel()
    transition_monitor_task.cancel()

@cocotb.test()
async def test_cancel(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_cancel")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    await tb.reset()

    # Delay wait command and cancel command sequence
    delay_cmd = tb.build_noop(trig_wait=0, cont=0, ldac=1, value=50)
    cancel_cmd = tb.build_cancel()
    cmd_word_list = []
    cmd_word_list.append(delay_cmd)
    cmd_word_list.append(cancel_cmd)

    # Start the command buffer model and scoreboard
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_word_list)))

    # Send commands and wait for completion
    await tb.send_commands(cmd_word_list)
    await scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.cancel()
    scoreboard_task.cancel()
    transition_monitor_task.cancel()

# Free-running waveform sim (no pass/fail). Skipped in the automated suite so `make tests`
# stays bounded; run on demand with RUN_EXAMPLE_SIMS=1 (e.g.
# RUN_EXAMPLE_SIMS=1 ./scripts/make/test_core.sh rev_d_shim shim ad5676_dac_ctrl).
@cocotb.test()
async def test_debug_lane(dut):
    """Debug words go to the debug port only, GET_CAL readback to the data port; a full debug
    buffer drops debug words with a sticky flag and never halts the core."""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_debug_lane")
    dut.debug.value = 1
    counts = {"debug": 0, "data": 0}
    data_words = []

    async def count_writes():
        while True:
            await RisingEdge(dut.clk)
            await ReadOnly()
            if int(dut.debug_buf_wr_en.value):
                counts["debug"] += 1
                assert (int(dut.debug_word.value) >> 28) != tb.CAL_DATA, "calibration word on the debug port"
            if int(dut.data_buf_wr_en.value):
                counts["data"] += 1
                data_words.append(int(dut.data_word.value))

    await tb.reset()
    counter_task = cocotb.start_soon(count_writes())
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())

    # Boot sweep and a write produce debug words; only GET_CAL reaches the data port
    await tb.send_commands([tb.build_dac_wr_ch(7, 1000), tb.build_get_cal(ch=3)])
    for _ in range(300):
        await RisingEdge(dut.clk)
    assert counts["debug"] > 0, "debug mode produced no debug words"
    assert counts["data"] == 1 and (data_words[0] >> 28) == tb.CAL_DATA, f"data port carried {data_words}"
    assert int(dut.debug_buf_overflow.value) == 0, "debug_buf_overflow set with room in the buffer"

    # Full debug buffer: debug words dropped, sticky flag set, GET_CAL readback unaffected
    dut.debug_buf_full.value = 1
    debug_before = counts["debug"]
    await tb.send_commands([tb.build_dac_wr_ch(5, 2000), tb.build_get_cal(ch=5)])
    for _ in range(300):
        await RisingEdge(dut.clk)
    assert counts["debug"] == debug_before, "debug word written while the debug buffer was full"
    assert int(dut.debug_buf_overflow.value) == 1, "debug_buf_overflow not set"
    assert counts["data"] == 2, "GET_CAL readback lost while the debug buffer was full"
    assert int(dut.data_buf_overflow.value) == 0, "data_buf_overflow set"
    assert int(dut.state.value) != tb.STATE_ENCODING['S_ERROR'], "full debug buffer halted the core"

    counter_task.cancel()
    cmd_buf_task.cancel()

@cocotb.test(skip=not os.environ.get("RUN_EXAMPLE_SIMS"))
async def example_simulation(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: example_simulation")

    # Build a sequence of commands to simulate
    cmd_word_list = []
    for ch in range(8):
        for it in range(5):
            value = -3000 + it * 1500 # Example values for testing
            cmd_word_list.append(tb.build_dac_wr_ch(ch, value))
        cmd_word_list.append(tb.build_dac_wr_ch(ch, 0)) # Reset to 0 after the sequence

    simulation_task = cocotb.start_soon(tb.simulate_dut(cmd_word_list, clk_cycles=1000))
    await simulation_task
