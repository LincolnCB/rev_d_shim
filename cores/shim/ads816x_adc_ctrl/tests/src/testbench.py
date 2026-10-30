import cocotb
import os
from cocotb.triggers import RisingEdge, ReadOnly, ClockCycles
import random

from shim_ads816x_adc_ctrl_base import shim_ads816x_adc_ctrl_base

async def setup_testbench(dut, clk_period=4, miso_sck_period=4, time_unit="ns"):
    tb = shim_ads816x_adc_ctrl_base(dut, clk_period, miso_sck_period, time_unit)
    return tb

@cocotb.test()
async def test_reset(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_reset")

    await tb.reset()

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)

@cocotb.test()
async def test_set_ord(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_set_ord")

    # Have the DUT at a known state
    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    miso_transition_monitor_task = cocotb.start_soon(tb.miso_transition_monitor())

    # Actual reset
    await tb.reset()

    channels = [3,4,5,6,7,0,1,2]

    # Build set ORD command
    cmd_word_list = []
    cmd_word_list.append(tb.build_set_ord(channels=channels))

    # Start the command buffer model and scoreboard
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(cmd_word_list))

    # Send commands and wait for completion
    await tb.send_commands(cmd_word_list)
    await scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.cancel()
    transition_monitor_task.cancel()
    scoreboard_task.cancel()
    miso_transition_monitor_task.cancel()

@cocotb.test()
async def test_noop_trigger_wait(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_noop_trigger_wait")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    miso_transition_monitor_task = cocotb.start_soon(tb.miso_transition_monitor())
    await tb.reset()

    # Build the NO_OP command sequence
    cmd_word_list = []
    cmd_word_list.append(tb.build_noop(trig_wait=1, cont=0, value=10))

    # Start the command buffer model and scoreboard
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(cmd_word_list))

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
    miso_transition_monitor_task.cancel()

@cocotb.test()
async def test_noop_delay(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_noop_delay")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    miso_transition_monitor_task = cocotb.start_soon(tb.miso_transition_monitor())
    await tb.reset()

    # Build the NO_OP command sequence
    cmd_word_list = []
    cmd_word_list.append(tb.build_noop(trig_wait=0, cont=0, value=20))

    # Start the command buffer model and scoreboard
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(cmd_word_list))

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
    miso_transition_monitor_task.cancel()

@cocotb.test()
async def test_noop_delay_repeating(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_noop_delay_repeating")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    miso_transition_monitor_task = cocotb.start_soon(tb.miso_transition_monitor())
    await tb.reset()

    # A delay NO_OP repeated: one initial + 3 repeats = 4 delay waits of 20 cycles.
    cmd_word_list = []
    cmd_word_list.append(tb.build_noop(trig_wait=0, cont=0, value=20, repeat=1))
    cmd_word_list.append(tb.build_repeat_count(3))

    # Start the command buffer model and scoreboard
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(cmd_word_list))

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
    miso_transition_monitor_task.cancel()

@cocotb.test()
async def test_noop_trig_repeating(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_noop_trig_repeating")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    miso_transition_monitor_task = cocotb.start_soon(tb.miso_transition_monitor())
    await tb.reset()

    # A trigger-wait NO_OP repeated: one initial + 2 repeats = 3 single-trigger waits.
    cmd_word_list = []
    cmd_word_list.append(tb.build_noop(trig_wait=1, cont=0, value=1, repeat=1))
    cmd_word_list.append(tb.build_repeat_count(2))

    # Start the command buffer model and scoreboard
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(cmd_word_list))

    # Start the random trigger driver (waits for setup_done before firing)
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
    miso_transition_monitor_task.cancel()

@cocotb.test()
async def test_bad_cmd(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_bad_cmd")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    miso_transition_monitor_task = cocotb.start_soon(tb.miso_transition_monitor())
    await tb.reset()

    # Build the BAD_CMD command sequence
    cmd_type = 0b110  # Invalid command type
    cmd_word_list = []
    cmd_word_list.append((cmd_type << 29))

    # This test intentionally drives the DUT into S_ERROR.
    tb.expect_error = True

    # Start the command buffer model and scoreboard
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(cmd_word_list))

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
    miso_transition_monitor_task.cancel()

@cocotb.test()
async def test_cancel(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_cancel")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    miso_transition_monitor_task = cocotb.start_soon(tb.miso_transition_monitor())
    await tb.reset()

    # Delay wait command and cancel command sequence
    delay_cmd = tb.build_noop(trig_wait=0, cont=0, value=50)
    cancel_cmd = tb.build_cancel()
    cmd_word_list = []
    cmd_word_list.append(delay_cmd)
    cmd_word_list.append(cancel_cmd)

    # Start the command buffer model and scoreboard
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(cmd_word_list))

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
    miso_transition_monitor_task.cancel()

@cocotb.test()
async def test_adc_rd_ch_no_repeat(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_adc_rd_ch_no_repeat")

    await tb.reset_both_domains()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    miso_transition_monitor_task = cocotb.start_soon(tb.miso_transition_monitor())
    await tb.reset_both_domains()

    # Build the ADC_RD_CH command sequence
    cmd_word_list = []
    cmd_word_list.append(tb.build_adc_rd_ch(ch=5, repeat=0))

    # Start the command buffer model, scoreboard and data buffer model
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    data_buf_task = cocotb.start_soon(tb.data_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(cmd_word_list))

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
    data_buf_task.cancel()
    miso_transition_monitor_task.cancel()

@cocotb.test()
async def test_adc_rd_ch_no_repeat_back_to_back(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_adc_rd_ch_no_repeat_back_to_back")

    await tb.reset_both_domains()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    miso_transition_monitor_task = cocotb.start_soon(tb.miso_transition_monitor())
    await tb.reset_both_domains()

    # Build the ADC_RD_CH command sequence
    cmd_word_list = []
    cmd_word_list.append(tb.build_adc_rd_ch(ch=5, repeat=0))
    cmd_word_list.append(tb.build_adc_rd_ch(ch=2, repeat=0))
    cmd_word_list.append(tb.build_adc_rd_ch(ch=7, repeat=0))

    # Start the command buffer model, scoreboard and data buffer model
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    data_buf_task = cocotb.start_soon(tb.data_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(cmd_word_list))

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
    data_buf_task.cancel()
    miso_transition_monitor_task.cancel()

@cocotb.test()
async def test_adc_rd_ch_repeating(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_adc_rd_ch_repeating")

    await tb.reset_both_domains()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    miso_transition_monitor_task = cocotb.start_soon(tb.miso_transition_monitor())
    await tb.reset_both_domains()

    # Build the ADC_RD_CH repeating command sequence
    cmd_word_list = []
    cmd_word_list.append(tb.build_adc_rd_ch(repeat=1, ch=3))
    cmd_word_list.append(tb.build_repeat_count(5))

    # Start the command buffer model, scoreboard and data buffer model
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    data_buf_task = cocotb.start_soon(tb.data_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(cmd_word_list))

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
    data_buf_task.cancel()
    miso_transition_monitor_task.cancel()

@cocotb.test()
async def test_adc_rd_no_repeat(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_adc_rd_no_repeat")

    await tb.reset_both_domains()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    miso_transition_monitor_task = cocotb.start_soon(tb.miso_transition_monitor())
    await tb.reset_both_domains()

    # Build the ADC_RD command sequence
    cmd_word_list = []
    cmd_word_list.append(tb.build_adc_rd(trig_wait=0, cont=0, repeat=0, value=1000))

    # Start the command buffer model, scoreboard and data buffer model
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    data_buf_task = cocotb.start_soon(tb.data_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(cmd_word_list))

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
    data_buf_task.cancel()
    miso_transition_monitor_task.cancel()

@cocotb.test()
async def test_adc_rd_no_repeat_back_to_back(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_adc_rd_no_repeat_back_to_back")

    await tb.reset_both_domains()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    miso_transition_monitor_task = cocotb.start_soon(tb.miso_transition_monitor())
    await tb.reset_both_domains()

    # Build the ADC_RD command sequence
    cmd_word_list = []
    cmd_word_list.append(tb.build_adc_rd(trig_wait=0, cont=0, repeat=0, value=1000))
    cmd_word_list.append(tb.build_adc_rd(trig_wait=0, cont=0, repeat=0, value=2000))
    cmd_word_list.append(tb.build_adc_rd(trig_wait=0, cont=0, repeat=0, value=3000))

    # Start the command buffer model, scoreboard and data buffer model
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    data_buf_task = cocotb.start_soon(tb.data_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(cmd_word_list))

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
    data_buf_task.cancel()
    miso_transition_monitor_task.cancel()

@cocotb.test()
async def test_adc_rd_repeating(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_adc_rd_repeating")

    await tb.reset_both_domains()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    miso_transition_monitor_task = cocotb.start_soon(tb.miso_transition_monitor())
    await tb.reset_both_domains()

    # Build the ADC_RD command sequence
    cmd_word_list = []
    cmd_word_list.append(tb.build_adc_rd(trig_wait=0, cont=0, repeat=1, value=1000))
    cmd_word_list.append(tb.build_repeat_count(3))

    # Start the command buffer model, scoreboard and data buffer model
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    data_buf_task = cocotb.start_soon(tb.data_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(cmd_word_list))

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
    miso_transition_monitor_task.cancel()
    data_buf_task.cancel()

@cocotb.test()
async def test_adc_rd_repeat_count_late(dut):
    """The repeat count word may arrive after a repeating ADC_RD starts, as long as it's in the
    buffer before the first iteration finishes. Repeat timing is unchanged."""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_adc_rd_repeat_count_late")

    await tb.reset_both_domains()
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    miso_transition_monitor_task = cocotb.start_soon(tb.miso_transition_monitor())
    await tb.reset_both_domains()

    cmd_word_list = []
    cmd_word_list.append(tb.build_adc_rd(trig_wait=0, cont=0, repeat=1, value=1000))
    cmd_word_list.append(tb.build_repeat_count(2))

    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    data_buf_task = cocotb.start_soon(tb.data_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(cmd_word_list))

    # Count word lands partway through the first 1000-cycle iteration
    await tb.send_commands(cmd_word_list[:1])
    await ClockCycles(dut.clk, 500)
    assert int(dut.repeat_pending.value) == 1, "Expected repeat_pending while the count word is missing"
    await tb.send_commands(cmd_word_list[1:])
    await scoreboard_task

    assert int(dut.cmd_buf_underflow.value) == 0, "Late repeat count (before first iteration ended) flagged as underflow"

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.cancel()
    scoreboard_task.cancel()
    transition_monitor_task.cancel()
    miso_transition_monitor_task.cancel()
    data_buf_task.cancel()

@cocotb.test()
async def test_adc_rd_repeat_count_missing(dut):
    """A repeating ADC_RD whose count word isn't in the buffer by the end of the first iteration
    halts with a command buffer underflow at that point, not earlier."""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_adc_rd_repeat_count_missing")

    await tb.reset_both_domains()
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    miso_transition_monitor_task = cocotb.start_soon(tb.miso_transition_monitor())
    await tb.reset_both_domains()

    delay = 1000
    tb.expect_error = True

    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    data_buf_task = cocotb.start_soon(tb.data_buf_model())

    await tb.send_commands([tb.build_adc_rd(trig_wait=0, cont=0, repeat=1, value=delay)])

    # Wait for the command to be popped, then count cycles until the underflow flag
    while True:
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.cmd_buf_rd_en.value) == 1:
            break
    cycles = 0
    while int(dut.cmd_buf_underflow.value) == 0:
        await RisingEdge(dut.clk)
        await ReadOnly()
        cycles += 1
        assert cycles < 4 * delay, "Missing repeat count never flagged as underflow"

    assert cycles >= delay, f"Underflow flagged after {cycles} cycles, before the first {delay}-cycle iteration finished"
    assert int(dut.state.value) == tb.STATE_ENCODING['S_ERROR'], \
        f"Expected S_ERROR after underflow, got {tb.get_state_name(int(dut.state.value))}"

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.cancel()
    transition_monitor_task.cancel()
    miso_transition_monitor_task.cancel()
    data_buf_task.cancel()

# Free-running waveform sim (no pass/fail). Skipped in the automated suite so `make tests`
# stays bounded; run on demand with RUN_EXAMPLE_SIMS=1 (e.g.
# RUN_EXAMPLE_SIMS=1 ./scripts/make/test_core.sh rev_d_shim shim ads816x_adc_ctrl).
@cocotb.test()
async def test_debug_lane(dut):
    """Debug words go to the debug port only, samples to the data port; a full debug buffer
    drops debug words with a sticky flag and never halts the core."""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_debug_lane")
    dut.debug.value = 1
    counts = {"debug": 0, "data": 0}

    async def count_writes():
        while True:
            await RisingEdge(dut.clk)
            await ReadOnly()
            if int(dut.debug_buf_wr_en.value):
                counts["debug"] += 1
                assert (int(dut.debug_word.value) >> 28) != 0, "untagged word on the debug port"
            if int(dut.data_buf_wr_en.value):
                counts["data"] += 1
                assert (int(dut.data_word.value) >> 16) == 0, "debug-tagged word on the data port"

    await tb.reset_both_domains()
    counter_task = cocotb.start_soon(count_writes())
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())

    # Boot and a single-channel read produce debug words; only the sample reaches the data port
    await tb.send_commands([tb.build_adc_rd_ch(ch=5, repeat=0)])
    for _ in range(300):
        await RisingEdge(dut.clk)
    assert counts["debug"] > 0, "debug mode produced no debug words"
    assert counts["data"] == 1, f"expected one sample word, got {counts['data']}"
    assert int(dut.debug_buf_overflow.value) == 0, "debug_buf_overflow set with room in the buffer"

    # Full debug buffer: debug words dropped, sticky flag set, samples unaffected
    dut.debug_buf_full.value = 1
    debug_before = counts["debug"]
    await tb.send_commands([tb.build_adc_rd_ch(ch=2, repeat=0)])
    for _ in range(300):
        await RisingEdge(dut.clk)
    assert counts["debug"] == debug_before, "debug word written while the debug buffer was full"
    assert int(dut.debug_buf_overflow.value) == 1, "debug_buf_overflow not set"
    assert counts["data"] == 2, "sample lost while the debug buffer was full"
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
    cmd_word_list.append(tb.build_set_ord(channels=[0,1,2,3,4,5,6,7]))
    cmd_word_list.append(tb.build_noop(trig_wait=1, cont=0, value=10))
    cmd_word_list.append(tb.build_noop(trig_wait=0, cont=0, value=20))

    simulation_task = cocotb.start_soon(tb.simulate_dut(cmd_word_list, clk_cycles=10000))
    await simulation_task
