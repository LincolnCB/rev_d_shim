import cocotb
from cocotb.triggers import RisingEdge
import random

from shim_ads816x_adc_ctrl_base import shim_ads816x_adc_ctrl_base

async def setup_testbench(dut, clk_period=4, miso_sck_period=4, time_unit="ns"):
    tb = shim_ads816x_adc_ctrl_base(dut, clk_period, miso_sck_period, time_unit)
    return tb

@cocotb.test(skip=True)
async def test_reset(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_reset")

    await tb.reset()

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)

@cocotb.test(skip=True)
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
    cmd = tb.build_set_ord(channels=channels)
    cmd_list = []
    cmd_list.append(cmd)

    # Start the command buffer model and scoreboard
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_list)))

    # Send commands and wait for completion
    await tb.send_commands(cmd_list)
    await scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.kill()
    transition_monitor_task.kill()
    scoreboard_task.kill()
    miso_transition_monitor_task.kill()

@cocotb.test(skip=True)
async def test_noop_trigger_wait(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_noop_trigger_wait")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    miso_transition_monitor_task = cocotb.start_soon(tb.miso_transition_monitor())
    await tb.reset()

    # Build the NO_OP command sequence
    cmd_list = []
    cmd_list.append(tb.build_noop(trig_wait=1, cont=0, value=10))

    # Start the command buffer model and scoreboard
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_list)))

    # Start the random trigger driver
    trigger_driver_task = cocotb.start_soon(tb.random_trigger_driver())

    # Send commands and wait for completion
    await tb.send_commands(cmd_list)
    await scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.kill()
    scoreboard_task.kill()
    transition_monitor_task.kill()
    trigger_driver_task.kill()
    miso_transition_monitor_task.kill()

@cocotb.test(skip=True)
async def test_noop_delay(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_noop_delay")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    miso_transition_monitor_task = cocotb.start_soon(tb.miso_transition_monitor())
    await tb.reset()

    # Build the NO_OP command sequence
    cmd_list = []
    cmd_list.append(tb.build_noop(trig_wait=0, cont=0, value=20))

    # Start the command buffer model and scoreboard
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_list)))

    # Send commands and wait for completion
    await tb.send_commands(cmd_list)
    await scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.kill()
    scoreboard_task.kill()
    transition_monitor_task.kill()
    miso_transition_monitor_task.kill()

@cocotb.test(skip=True)
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
    cmd = (cmd_type << 29)
    cmd_list = []
    cmd_list.append(cmd)

    # Start the command buffer model and scoreboard
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_list)))

    # Send commands and wait for completion
    await tb.send_commands(cmd_list)
    await scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.kill()
    scoreboard_task.kill()
    transition_monitor_task.kill()
    miso_transition_monitor_task.kill()

@cocotb.test(skip=True)
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
    cmd_list = []
    cmd_list.append(delay_cmd)
    cmd_list.append(cancel_cmd)

    # Start the command buffer model and scoreboard
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_list)))

    # Send commands and wait for completion
    await tb.send_commands(cmd_list)
    await scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.kill()
    scoreboard_task.kill()
    transition_monitor_task.kill()
    miso_transition_monitor_task.kill()

@cocotb.test(skip=True)
async def test_adc_rd_ch_no_repeat(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_adc_rd_ch_no_repeat")

    await tb.reset_both_domains()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    miso_transition_monitor_task = cocotb.start_soon(tb.miso_transition_monitor())
    await tb.reset_both_domains()

    # Build the ADC_RD_CH command sequence
    cmd = tb.build_adc_rd_ch(ch=5, repeat=0)
    cmd_list = []
    cmd_list.append(cmd)

    # Start the command buffer model, scoreboard and data buffer model
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    data_buf_task = cocotb.start_soon(tb.data_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_list)))

    # Send commands and wait for completion
    await tb.send_commands(cmd_list)
    await scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.kill()
    scoreboard_task.kill()
    transition_monitor_task.kill()
    data_buf_task.kill()
    miso_transition_monitor_task.kill()

@cocotb.test(skip=True)
async def test_adc_rd_ch_no_repeat_back_to_back(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_adc_rd_ch_no_repeat_back_to_back")

    await tb.reset_both_domains()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    miso_transition_monitor_task = cocotb.start_soon(tb.miso_transition_monitor())
    await tb.reset_both_domains()

    # Build the ADC_RD_CH command sequence
    cmd = tb.build_adc_rd_ch(ch=5, repeat=0)
    cmd_list = []
    cmd_list.append(cmd)
    cmd = tb.build_adc_rd_ch(ch=2, repeat=0)
    cmd_list.append(cmd)
    cmd = tb.build_adc_rd_ch(ch=7, repeat=0)
    cmd_list.append(cmd)

    # Start the command buffer model, scoreboard and data buffer model
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    data_buf_task = cocotb.start_soon(tb.data_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_list)))

    # Send commands and wait for completion
    await tb.send_commands(cmd_list)
    await scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.kill()
    scoreboard_task.kill()
    transition_monitor_task.kill()
    data_buf_task.kill()
    miso_transition_monitor_task.kill()

@cocotb.test(skip=True)
async def test_adc_rd_ch_repeating(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_adc_rd_ch_repeating")

    await tb.reset_both_domains()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    miso_transition_monitor_task = cocotb.start_soon(tb.miso_transition_monitor())
    await tb.reset_both_domains()

    # Build the ADC_RD_CH repeating command sequence
    cmd = tb.build_adc_rd_ch(repeat=1, ch=3)
    cmd_list = []
    cmd_list.append(cmd)
    cmd = tb.build_repeat_count(5)
    cmd_list.append(cmd)

    # Start the command buffer model, scoreboard and data buffer model
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    data_buf_task = cocotb.start_soon(tb.data_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_list)))

    # Send commands and wait for completion
    await tb.send_commands(cmd_list)
    await scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.kill()
    scoreboard_task.kill()
    transition_monitor_task.kill()
    data_buf_task.kill()
    miso_transition_monitor_task.kill()

@cocotb.test(skip=True)
async def test_adc_rd_no_repeat(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_adc_rd_no_repeat")

    await tb.reset_both_domains()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    miso_transition_monitor_task = cocotb.start_soon(tb.miso_transition_monitor())
    await tb.reset_both_domains()

    # Build the ADC_RD command sequence
    cmd = tb.build_adc_rd(trig_wait=0, cont=0, repeat=0, value=1000)
    cmd_list = []
    cmd_list.append(cmd)

    # Start the command buffer model, scoreboard and data buffer model
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    data_buf_task = cocotb.start_soon(tb.data_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_list)))

    # Send commands and wait for completion
    await tb.send_commands(cmd_list)
    await scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.kill()
    scoreboard_task.kill()
    transition_monitor_task.kill()
    data_buf_task.kill()
    miso_transition_monitor_task.kill()

@cocotb.test(skip=True)
async def test_adc_rd_no_repeat_back_to_back(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_adc_rd_no_repeat_back_to_back")

    await tb.reset_both_domains()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    miso_transition_monitor_task = cocotb.start_soon(tb.miso_transition_monitor())
    await tb.reset_both_domains()

    # Build the ADC_RD command sequence
    cmd = tb.build_adc_rd(trig_wait=0, cont=0, repeat=0, value=1000)
    cmd_list = []
    cmd_list.append(cmd)
    cmd = tb.build_adc_rd(trig_wait=0, cont=0, repeat=0, value=2000)
    cmd_list.append(cmd)
    cmd = tb.build_adc_rd(trig_wait=0, cont=0, repeat=0, value=3000)
    cmd_list.append(cmd)

    # Start the command buffer model, scoreboard and data buffer model
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    data_buf_task = cocotb.start_soon(tb.data_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_list)))

    # Send commands and wait for completion
    await tb.send_commands(cmd_list)
    await scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.kill()
    scoreboard_task.kill()
    transition_monitor_task.kill()
    data_buf_task.kill()
    miso_transition_monitor_task.kill()

@cocotb.test(skip=True)
async def test_adc_rd_repeating(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_adc_rd_repeating")

    await tb.reset_both_domains()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    miso_transition_monitor_task = cocotb.start_soon(tb.miso_transition_monitor())
    await tb.reset_both_domains()

    # Build the ADC_RD command sequence
    cmd = tb.build_adc_rd(trig_wait=0, cont=0, repeat=1, value=1000)
    cmd_list = []
    cmd_list.append(cmd)
    cmd = tb.build_repeat_count(3)
    cmd_list.append(cmd)

    # Start the command buffer model, scoreboard and data buffer model
    await RisingEdge(dut.clk)
    cmd_buf_task = cocotb.start_soon(tb.command_buf_model())
    data_buf_task = cocotb.start_soon(tb.data_buf_model())
    scoreboard_task = cocotb.start_soon(tb.executing_command_scoreboard(len(cmd_list)))

    # Send commands and wait for completion
    await tb.send_commands(cmd_list)
    await scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    cmd_buf_task.kill()
    scoreboard_task.kill()
    transition_monitor_task.kill()
    miso_transition_monitor_task.kill()
    data_buf_task.kill()

@cocotb.test()
async def example_simulation(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: example_simulation")
    
    # Build a sequence of commands to simulate
    cmd_list = []
    cmd_list.append(tb.build_set_ord(channels=[0,1,2,3,4,5,6,7]))
    cmd_list.append(tb.build_noop(trig_wait=1, cont=0, value=10))
    cmd_list.append(tb.build_noop(trig_wait=0, cont=0, value=20))

    simulation_task = cocotb.start_soon(tb.simulate_dut(cmd_list, clk_cycles=10000))
    await simulation_task