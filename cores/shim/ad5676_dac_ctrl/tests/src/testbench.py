import cocotb
from cocotb.triggers import RisingEdge
import random

from shim_ad5676_dac_ctrl_base import shim_ad5676_dac_ctrl_base

async def setup_testbench(dut, clk_period=4, miso_sck_period=4, time_unit="ns"):
    tb = shim_ad5676_dac_ctrl_base(dut, clk_period, miso_sck_period, time_unit)
    return tb

@cocotb.test(skip=True)
async def test_reset(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_reset")

    await tb.reset()

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)

@cocotb.test(skip=True)
async def test_dac_wr(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_dac_wr")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    await tb.reset()

    # Build the DAC_WR command sequence
    cmd_list = []
    cmd_list.append(tb.build_dac_wr_header(trig_wait=1, cont=0, ldac=1, value=10))
    for _ in range(4):
        value = 1000
        cmd_list.append(tb.build_dac_pair(v_lo_chN=value, v_hi_chNp1=value))

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

@cocotb.test(skip=True)
async def test_dac_wr_ch(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_dac_wr_ch")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    await tb.reset()

    # Build the DAC_WR_CH command sequence
    cmd_list = []
    cmd_list.append(tb.build_dac_wr_ch(7, 1000))

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

@cocotb.test(skip=True)
async def test_zero(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_zero")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    await tb.reset()

    # Build the ZERO command sequence
    cmd_list = []
    cmd_list.append(tb.build_zero())

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

@cocotb.test(skip=True)
async def test_noop_trigger_wait(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_noop_trigger_wait")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    await tb.reset()

    # Build the NO_OP command sequence
    cmd_list = []
    cmd_list.append(tb.build_noop(trig_wait=1, cont=0, ldac=1, value=10))

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

@cocotb.test(skip=True)
async def test_noop_trigger_wait_no_ldac(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_noop_trigger_wait_no_ldac")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    await tb.reset()

    # Build the NO_OP command sequence
    cmd_list = []
    cmd_list.append(tb.build_noop(trig_wait=1, cont=0, ldac=0, value=10))

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

@cocotb.test(skip=True)
async def test_noop_delay(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_noop_delay")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    await tb.reset()

    # Build the NO_OP command sequence
    cmd_list = []
    cmd_list.append(tb.build_noop(trig_wait=0, cont=0, ldac=1, value=20))

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

@cocotb.test(skip=True)
async def test_noop_delay_no_ldac(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_noop_delay_no_ldac")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    await tb.reset()

    # Build the NO_OP command sequence
    cmd_list = []
    cmd_list.append(tb.build_noop(trig_wait=0, cont=0, ldac=0, value=20))

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

@cocotb.test(skip=True)
async def test_set_cal(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_set_cal")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    await tb.reset()

    # Build the SET_CAL command sequence
    cmd_list = []
    cmd_list.append(tb.build_set_cal(ch=3, cal_signed_16=-1234))

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

@cocotb.test(skip=True)
async def test_get_cal(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_get_cal")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
    await tb.reset()

    # Build the GET_CAL command sequence
    cmd_list = []
    cmd_list.append(tb.build_get_cal(ch=3))

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

@cocotb.test(skip=True)
async def test_bad_cmd(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_bad_cmd")

    await tb.reset()
    # Start the transition monitor
    transition_monitor_task = cocotb.start_soon(tb.transition_monitor())
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

@cocotb.test(skip=True)
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
    transition_monitor_task.kill()\
    
@cocotb.test()
async def example_simulation(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: example_simulation")

    # Build a sequence of commands to simulate
    cmd_list = []
    cmd_list.append(tb.build_dac_wr_ch(0, 500))
    cmd_list.append(tb.build_dac_wr_ch(1, 1000))
    cmd_list.append(tb.build_dac_wr_ch(2, 1500))
    cmd_list.append(tb.build_dac_wr_ch(3, 2000))
    cmd_list.append(tb.build_noop(trig_wait=1, cont=0, ldac=1, value=10))
    cmd_list.append(tb.build_set_cal(ch=0, cal_signed_16=-500))
    cmd_list.append(tb.build_get_cal(ch=0))

    simulation_task = cocotb.start_soon(tb.simulate_dut(cmd_list, clk_cycles=1000))
    await simulation_task
