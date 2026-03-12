import cocotb
from cocotb.triggers import RisingEdge, Timer, ReadOnly, ReadWrite, FallingEdge, Combine
from hw_manager_base import hw_manager_base
from hw_manager_coverage import start_coverage_monitor
import random

RANDOM_SEED = 42

# Create a setup function that can be called by each test
async def setup_testbench(dut):
    # Delay parameters should be the same with hw_manager.v
    tb = hw_manager_base(dut, clk_period=4, time_unit="ns")
    return tb

@cocotb.test()
async def test_idle(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test idle")

    # Reset
    await tb.reset()
    
    state_scoreboard_task = cocotb.start_soon(tb.state_scoreboard_dispatcher())
    await state_scoreboard_task

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    state_scoreboard_task.kill()

@cocotb.test()
async def idle_to_running_to_halted(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: idle_to_running_to_halted")

    forked_tasks = []
    state_scoreboard_task = cocotb.start_soon(tb.state_scoreboard_dispatcher())
    forked_tasks.append(state_scoreboard_task)

    reach_s_halted_task = cocotb.start_soon(tb.reach_s_halted())
    forked_tasks.append(reach_s_halted_task)

    await Combine(*forked_tasks)

    # Give time before ending the test and ensure we don't collide with other tests
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    state_scoreboard_task.kill()
    reach_s_halted_task.kill()

# Helper: run state_scoreboard_dispatcher concurrently with a drive coroutine,
# await both, then kill any residual tasks.
async def _run_with_scoreboard(dut, tb, drive_coro):
    state_task = cocotb.start_soon(tb.state_scoreboard_dispatcher())
    drive_task = cocotb.start_soon(drive_coro)
    await Combine(state_task, drive_task)
    for _ in range(4):
        await RisingEdge(dut.clk)
    state_task.kill()
    drive_task.kill()


# -----------------------------------------------------------------------------
# S_IDLE error conditions
# -----------------------------------------------------------------------------

@cocotb.test()
async def test_idle_ext_en_low(dut):
    """S_IDLE: ctrl_en=1 but ext_en=0 → STS_EXT_SHUTDOWN → S_HALTED"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_idle_ext_en_low")

    async def drive():
        await tb.reach_s_idle()
        # ext_en stays 0 (reset default); assert ctrl_en to trigger S_IDLE check
        dut.ctrl_en.value = 1
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())


@cocotb.test()
async def test_idle_oob_errors(dut):
    """S_IDLE: every OOB signal causes the correct halt status code."""
    oob_cases = [
        ("ctrl_en_oob",          "STS_CTRL_EN_OOB"),
        ("pow_en_oob",           "STS_POW_EN_OOB"),
        ("cmd_buf_reset_oob",    "STS_CMD_BUF_RESET_OOB"),
        ("data_buf_reset_oob",   "STS_DATA_BUF_RESET_OOB"),
        ("integ_thresh_avg_oob", "STS_INTEG_THRESH_AVG_OOB"),
        ("integ_window_oob",     "STS_INTEG_WINDOW_OOB"),
        ("integ_en_oob",         "STS_INTEG_EN_OOB"),
        ("boot_test_skip_oob",   "STS_BOOT_TEST_SKIP_OOB"),
        ("debug_oob",            "STS_DEBUG_OOB"),
        ("mosi_sck_pol_oob",     "STS_MOSI_SCK_POL_OOB"),
        ("miso_sck_pol_oob",     "STS_MISO_SCK_POL_OOB"),
        ("dac_cal_init_oob",     "STS_DAC_CAL_INIT_OOB"),
    ]
    for signal_name, status_name in oob_cases:
        tb = await setup_testbench(dut)
        tb.dut._log.info(f"test_idle_oob_errors: {signal_name} → {status_name}")

        async def drive(sig=signal_name):
            await tb.reach_s_idle()
            getattr(dut, sig).value = 1
            dut.ctrl_en.value = 1
            dut.ext_en.value = 1
            await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

        await _run_with_scoreboard(dut, tb, drive())
        getattr(dut, signal_name).value = 0


# -----------------------------------------------------------------------------
# S_CONFIRM_SPI_RST error conditions
# -----------------------------------------------------------------------------

@cocotb.test()
async def test_confirm_spi_rst_ext_en_low(dut):
    """S_CONFIRM_SPI_RST: ext_en drops → STS_EXT_SHUTDOWN"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_confirm_spi_rst_ext_en_low")

    async def drive():
        await tb.reach_s_confirm_spi_rst()
        dut.ext_en.value = 0
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())


@cocotb.test()
async def test_confirm_spi_rst_ctrl_en_low(dut):
    """S_CONFIRM_SPI_RST: ctrl_en drops → STS_PS_SHUTDOWN"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_confirm_spi_rst_ctrl_en_low")

    async def drive():
        await tb.reach_s_confirm_spi_rst()
        dut.ctrl_en.value = 0
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())


@cocotb.test()
async def test_confirm_spi_rst_timeout(dut):
    """S_CONFIRM_SPI_RST: spi_off=0 throughout → STS_SPI_RESET_TIMEOUT"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_confirm_spi_rst_timeout")

    async def drive():
        await tb.reach_s_idle()
        dut.ctrl_en.value = 1
        dut.ext_en.value = 1
        dut.spi_off.value = 0   # SPI appears running, reset never confirmed
        dut.calc_n_cs_done.value = 0
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())


# -----------------------------------------------------------------------------
# S_POWER_ON_CRTL_BRD error conditions
# -----------------------------------------------------------------------------

@cocotb.test()
async def test_power_on_ctrl_brd_ext_en_low(dut):
    """S_POWER_ON_CRTL_BRD: ext_en drops → STS_EXT_SHUTDOWN"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_power_on_ctrl_brd_ext_en_low")

    async def drive():
        await tb.reach_s_power_on_ctrl_brd()
        dut.ext_en.value = 0
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())


@cocotb.test()
async def test_power_on_ctrl_brd_ctrl_en_low(dut):
    """S_POWER_ON_CRTL_BRD: ctrl_en drops → STS_PS_SHUTDOWN"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_power_on_ctrl_brd_ctrl_en_low")

    async def drive():
        await tb.reach_s_power_on_ctrl_brd()
        dut.ctrl_en.value = 0
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())


# -----------------------------------------------------------------------------
# S_CONFIRM_SPI_START error conditions
# -----------------------------------------------------------------------------

@cocotb.test()
async def test_confirm_spi_start_ext_en_low(dut):
    """S_CONFIRM_SPI_START: ext_en drops → STS_EXT_SHUTDOWN"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_confirm_spi_start_ext_en_low")

    async def drive():
        await tb.reach_s_confirm_spi_start()
        dut.ext_en.value = 0
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())


@cocotb.test()
async def test_confirm_spi_start_ctrl_en_low(dut):
    """S_CONFIRM_SPI_START: ctrl_en drops → STS_PS_SHUTDOWN"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_confirm_spi_start_ctrl_en_low")

    async def drive():
        await tb.reach_s_confirm_spi_start()
        dut.ctrl_en.value = 0
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())


@cocotb.test()
async def test_confirm_spi_start_dac_boot_fail(dut):
    """S_CONFIRM_SPI_START: dac_boot_fail[b] → STS_DAC_BOOT_FAIL, board_num==b for all b"""
    for board in range(8):
        tb = await setup_testbench(dut)
        tb.dut._log.info(f"test_confirm_spi_start_dac_boot_fail: board {board}")

        async def drive(b=board):
            await tb.reach_s_confirm_spi_start()
            dut.dac_boot_fail.value = 1 << b
            await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

        await _run_with_scoreboard(dut, tb, drive())
        dut.dac_boot_fail.value = 0


@cocotb.test()
async def test_confirm_spi_start_adc_boot_fail(dut):
    """S_CONFIRM_SPI_START: adc_boot_fail[b] → STS_ADC_BOOT_FAIL, board_num==b for all b"""
    for board in range(8):
        tb = await setup_testbench(dut)
        tb.dut._log.info(f"test_confirm_spi_start_adc_boot_fail: board {board}")

        async def drive(b=board):
            await tb.reach_s_confirm_spi_start()
            dut.adc_boot_fail.value = 1 << b
            await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

        await _run_with_scoreboard(dut, tb, drive())
        dut.adc_boot_fail.value = 0


@cocotb.test()
async def test_confirm_spi_start_timeout(dut):
    """S_CONFIRM_SPI_START: spi_off stays 1 (SPI never starts) → STS_SPI_START_TIMEOUT"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_confirm_spi_start_timeout")

    async def drive():
        # reach_s_power_on_ctrl_brd sets spi_off=1; reach_s_confirm_spi_start
        # does not change it, so SPI never transitions off → timeout fires.
        await tb.reach_s_confirm_spi_start()
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())


# -----------------------------------------------------------------------------
# S_WAIT_FOR_POW_EN error conditions
# -----------------------------------------------------------------------------

@cocotb.test()
async def test_wait_for_pow_en_ext_en_low(dut):
    """S_WAIT_FOR_POW_EN: ext_en drops → STS_EXT_SHUTDOWN"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_wait_for_pow_en_ext_en_low")

    async def drive():
        await tb.reach_s_wait_for_pow_en()
        dut.ext_en.value = 0
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())


@cocotb.test()
async def test_wait_for_pow_en_ctrl_en_low(dut):
    """S_WAIT_FOR_POW_EN: ctrl_en drops → STS_PS_SHUTDOWN"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_wait_for_pow_en_ctrl_en_low")

    async def drive():
        await tb.reach_s_wait_for_pow_en()
        dut.ctrl_en.value = 0
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())


# -----------------------------------------------------------------------------
# S_POWER_ON_AMP_BRD error conditions
# -----------------------------------------------------------------------------

@cocotb.test()
async def test_power_on_amp_brd_ext_en_low(dut):
    """S_POWER_ON_AMP_BRD: ext_en drops → STS_EXT_SHUTDOWN"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_power_on_amp_brd_ext_en_low")

    async def drive():
        await tb.reach_s_power_on_amp_brd()
        dut.ext_en.value = 0
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())


@cocotb.test()
async def test_power_on_amp_brd_ctrl_en_low(dut):
    """S_POWER_ON_AMP_BRD: ctrl_en drops → STS_PS_SHUTDOWN"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_power_on_amp_brd_ctrl_en_low")

    async def drive():
        await tb.reach_s_power_on_amp_brd()
        dut.ctrl_en.value = 0
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())


@cocotb.test()
async def test_power_on_amp_brd_pow_en_low(dut):
    """S_POWER_ON_AMP_BRD: pow_en drops → STS_PS_SHUTDOWN"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_power_on_amp_brd_pow_en_low")

    async def drive():
        await tb.reach_s_power_on_amp_brd()
        dut.pow_en.value = 0
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())


# -----------------------------------------------------------------------------
# S_AMP_POWER_WAIT error conditions
# -----------------------------------------------------------------------------

@cocotb.test()
async def test_amp_power_wait_ext_en_low(dut):
    """S_AMP_POWER_WAIT: ext_en drops → STS_EXT_SHUTDOWN"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_amp_power_wait_ext_en_low")

    async def drive():
        await tb.reach_s_amp_power_wait()
        dut.ext_en.value = 0
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())


@cocotb.test()
async def test_amp_power_wait_ctrl_en_low(dut):
    """S_AMP_POWER_WAIT: ctrl_en drops → STS_PS_SHUTDOWN"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_amp_power_wait_ctrl_en_low")

    async def drive():
        await tb.reach_s_amp_power_wait()
        dut.ctrl_en.value = 0
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())


@cocotb.test()
async def test_amp_power_wait_pow_en_low(dut):
    """S_AMP_POWER_WAIT: pow_en drops → STS_PS_SHUTDOWN"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_amp_power_wait_pow_en_low")

    async def drive():
        await tb.reach_s_amp_power_wait()
        dut.pow_en.value = 0
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())


# -----------------------------------------------------------------------------
# S_RUNNING error conditions — scalar signals
# -----------------------------------------------------------------------------
# Helper: reach S_RUNNING, consume the ps_interrupt-clearing cycle, inject an
# error signal, wait for S_HALTED.  The running_scoreboard (forked by the
# dispatcher) verifies the correct S_HALTING status code.
async def _running_error_drive(tb, dut, signal_name, value):
    await tb.reach_s_running()
    await RisingEdge(dut.clk)   # interrupt-clearing cycle
    await ReadWrite()
    getattr(dut, signal_name).value = value
    await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")


@cocotb.test()
async def test_running_ctrl_en_low(dut):
    """S_RUNNING: ctrl_en drops → STS_PS_SHUTDOWN"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_running_ctrl_en_low")
    await _run_with_scoreboard(dut, tb,
        _running_error_drive(tb, dut, "ctrl_en", 0))


@cocotb.test()
async def test_running_pow_en_low(dut):
    """S_RUNNING: pow_en drops → STS_PS_SHUTDOWN"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_running_pow_en_low")
    await _run_with_scoreboard(dut, tb,
        _running_error_drive(tb, dut, "pow_en", 0))


@cocotb.test()
async def test_running_lock_viol(dut):
    """S_RUNNING: lock_viol → STS_LOCK_VIOL"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_running_lock_viol")
    await _run_with_scoreboard(dut, tb,
        _running_error_drive(tb, dut, "lock_viol", 1))


@cocotb.test()
async def test_running_mosi_sck_pol_oob(dut):
    """S_RUNNING: mosi_sck_pol_oob → STS_MOSI_SCK_POL_OOB"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_running_mosi_sck_pol_oob")
    await _run_with_scoreboard(dut, tb,
        _running_error_drive(tb, dut, "mosi_sck_pol_oob", 1))


@cocotb.test()
async def test_running_miso_sck_pol_oob(dut):
    """S_RUNNING: miso_sck_pol_oob → STS_MISO_SCK_POL_OOB"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_running_miso_sck_pol_oob")
    await _run_with_scoreboard(dut, tb,
        _running_error_drive(tb, dut, "miso_sck_pol_oob", 1))


@cocotb.test()
async def test_running_ext_en_low(dut):
    """S_RUNNING: ext_en drops → STS_EXT_SHUTDOWN"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_running_ext_en_low")
    await _run_with_scoreboard(dut, tb,
        _running_error_drive(tb, dut, "ext_en", 0))


@cocotb.test()
async def test_running_bad_trig_cmd(dut):
    """S_RUNNING: bad_trig_cmd → STS_BAD_TRIG_CMD"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_running_bad_trig_cmd")
    await _run_with_scoreboard(dut, tb,
        _running_error_drive(tb, dut, "bad_trig_cmd", 1))


@cocotb.test()
async def test_running_trig_cmd_buf_overflow(dut):
    """S_RUNNING: trig_cmd_buf_overflow → STS_TRIG_CMD_BUF_OVERFLOW"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_running_trig_cmd_buf_overflow")
    await _run_with_scoreboard(dut, tb,
        _running_error_drive(tb, dut, "trig_cmd_buf_overflow", 1))


@cocotb.test()
async def test_running_trig_data_buf_underflow(dut):
    """S_RUNNING: trig_data_buf_underflow → STS_TRIG_DATA_BUF_UNDERFLOW"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_running_trig_data_buf_underflow")
    await _run_with_scoreboard(dut, tb,
        _running_error_drive(tb, dut, "trig_data_buf_underflow", 1))


@cocotb.test()
async def test_running_trig_data_buf_overflow(dut):
    """S_RUNNING: trig_data_buf_overflow → STS_TRIG_DATA_BUF_OVERFLOW"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_running_trig_data_buf_overflow")
    await _run_with_scoreboard(dut, tb,
        _running_error_drive(tb, dut, "trig_data_buf_overflow", 1))


# -----------------------------------------------------------------------------
# S_RUNNING per-board errors — all signals, all 8 board positions
# The running_scoreboard verifies the status code AND board number via
# extract_board_num, so iterating all 8 bit positions gives full coverage.
# -----------------------------------------------------------------------------

@cocotb.test()
async def test_running_per_board_errors(dut):
    """S_RUNNING: every per-board error signal at every board position (0-7).

    For each (signal, board) pair the running_scoreboard (dispatched
    concurrently) verifies:
      - state  == S_HALTING with the correct status code
      - board_num == board (lowest set bit)
    """
    per_board_cases = [
        ("shutdown_sense",         "STS_SHUTDOWN_SENSE"),
        ("over_thresh",            "STS_OVER_THRESH"),
        ("thresh_underflow",       "STS_THRESH_UNDERFLOW"),
        ("thresh_overflow",        "STS_THRESH_OVERFLOW"),
        ("bad_dac_cmd",            "STS_BAD_DAC_CMD"),
        ("dac_cal_oob",            "STS_DAC_CAL_OOB"),
        ("dac_val_oob",            "STS_DAC_VAL_OOB"),
        ("dac_cmd_buf_underflow",  "STS_DAC_CMD_BUF_UNDERFLOW"),
        ("dac_cmd_buf_overflow",   "STS_DAC_CMD_BUF_OVERFLOW"),
        ("dac_data_buf_underflow", "STS_DAC_DATA_BUF_UNDERFLOW"),
        ("dac_data_buf_overflow",  "STS_DAC_DATA_BUF_OVERFLOW"),
        ("unexp_dac_trig",         "STS_UNEXP_DAC_TRIG"),
        ("ldac_misalign",          "STS_LDAC_MISALIGN"),
        ("dac_delay_too_short",    "STS_DAC_DELAY_TOO_SHORT"),
        ("bad_adc_cmd",            "STS_BAD_ADC_CMD"),
        ("adc_cmd_buf_underflow",  "STS_ADC_CMD_BUF_UNDERFLOW"),
        ("adc_cmd_buf_overflow",   "STS_ADC_CMD_BUF_OVERFLOW"),
        ("adc_data_buf_underflow", "STS_ADC_DATA_BUF_UNDERFLOW"),
        ("adc_data_buf_overflow",  "STS_ADC_DATA_BUF_OVERFLOW"),
        ("unexp_adc_trig",         "STS_UNEXP_ADC_TRIG"),
        ("adc_delay_too_short",    "STS_ADC_DELAY_TOO_SHORT"),
    ]

    for sig_name, sts_name in per_board_cases:
        for board in range(8):
            tb = await setup_testbench(dut)
            tb.dut._log.info(
                f"test_running_per_board_errors: {sig_name}[{board}] → {sts_name}")

            async def drive(sig=sig_name, b=board):
                await tb.reach_s_running()
                await RisingEdge(dut.clk)   # interrupt-clearing cycle
                await ReadWrite()
                getattr(dut, sig).value = 1 << b
                await tb._wait_for_state_rw(
                    tb.get_state_value("S_HALTED"), "S_HALTED")

            await _run_with_scoreboard(dut, tb, drive())
            getattr(dut, sig_name).value = 0


# -----------------------------------------------------------------------------
# S_RUNNING error priority
# The Verilog if/else-if chain gives ctrl_en/pow_en the highest priority.
# Verify that when multiple errors fire simultaneously the highest-priority
# status code is reported.
# -----------------------------------------------------------------------------

@cocotb.test()
async def test_running_ps_shutdown_priority_over_lock_viol(dut):
    """S_RUNNING: ctrl_en=0 + lock_viol=1 simultaneously → STS_PS_SHUTDOWN wins"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_running_ps_shutdown_priority_over_lock_viol")

    async def drive():
        await tb.reach_s_running()
        await RisingEdge(dut.clk)   # interrupt-clearing cycle
        await ReadWrite()
        dut.lock_viol.value = 1
        dut.ctrl_en.value = 0       # higher priority than lock_viol
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())


@cocotb.test()
async def test_running_lock_viol_priority_over_shutdown_sense(dut):
    """S_RUNNING: lock_viol=1 + shutdown_sense=1 → STS_LOCK_VIOL wins"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_running_lock_viol_priority_over_shutdown_sense")

    async def drive():
        await tb.reach_s_running()
        await RisingEdge(dut.clk)
        await ReadWrite()
        dut.shutdown_sense.value = 0xFF
        dut.lock_viol.value = 1         # higher priority than shutdown_sense
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())


@cocotb.test()
async def test_running_shutdown_sense_priority_over_ext_en(dut):
    """S_RUNNING: shutdown_sense=1 + ext_en=0 → STS_SHUTDOWN_SENSE wins"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_running_shutdown_sense_priority_over_ext_en")

    async def drive():
        await tb.reach_s_running()
        await RisingEdge(dut.clk)
        await ReadWrite()
        dut.ext_en.value = 0
        dut.shutdown_sense.value = 0x01     # higher priority than ext_en
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())


# -----------------------------------------------------------------------------
# S_HALTED exit conditions
# -----------------------------------------------------------------------------

@cocotb.test()
async def test_halted_only_ctrl_en_low_stays_halted(dut):
    """S_HALTED: ctrl_en=0 alone (pow_en still 1) does NOT return to S_IDLE"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_halted_only_ctrl_en_low_stays_halted")

    await tb.reach_s_halted()
    # ctrl_en already 0 from reach_s_halting; pow_en=1; DUT must stay in S_HALTED
    for _ in range(10):
        await RisingEdge(dut.clk)
        await ReadWrite()
        await tb.check_state(tb.get_state_value("S_HALTED"))


@cocotb.test()
async def test_halted_only_pow_en_low_stays_halted(dut):
    """S_HALTED: pow_en=0 alone (ctrl_en still 1) does NOT return to S_IDLE"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_halted_only_pow_en_low_stays_halted")

    # Reach S_HALTED via a route that keeps ctrl_en=1 but drops pow_en=0.
    # Use reach_s_running then drop pow_en (which has lower priority than ctrl_en
    # in the running error chain, still causes PS_SHUTDOWN halt).
    async def drive():
        await tb.reach_s_running()
        await RisingEdge(dut.clk)
        await ReadWrite()
        dut.pow_en.value = 0
        dut.ctrl_en.value = 1   # keep ctrl_en high so only pow_en is low
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")
        # Now ctrl_en=1, pow_en=0 — DUT must stay in S_HALTED
        for _ in range(10):
            await RisingEdge(dut.clk)
            await ReadWrite()
            await tb.check_state(tb.get_state_value("S_HALTED"))

    state_task = cocotb.start_soon(tb.state_scoreboard_dispatcher())
    drive_task = cocotb.start_soon(drive())
    await Combine(state_task, drive_task)
    for _ in range(4):
        await RisingEdge(dut.clk)
    state_task.kill()
    drive_task.kill()


@cocotb.test()
async def test_halted_to_idle_both_enables_low(dut):
    """S_HALTED: both ctrl_en=0 and pow_en=0 → S_IDLE with STS_OK, board_num=0"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_halted_to_idle_both_enables_low")

    async def drive():
        await tb.reach_s_halted()
        # ctrl_en already 0; consume interrupt-clearing cycle then drop pow_en
        await RisingEdge(dut.clk)
        await ReadWrite()
        dut.pow_en.value = 0
        await tb._wait_for_state_rw(tb.get_state_value("S_IDLE"), "S_IDLE")
        await tb.check_state_and_status(
            expected_state=tb.get_state_value("S_IDLE"),
            expected_status_code=tb.get_status_value("STS_OK"),
            expected_board_num=0
        )
        assert dut.unlock_cfg.value == 1, "unlock_cfg should be 1 in S_IDLE"

    # The halted_scoreboard in the dispatcher also verifies the S_IDLE transition
    # once both enables are low.
    await _run_with_scoreboard(dut, tb, drive())


@cocotb.test()
async def test_halted_to_idle_clears_board_num(dut):
    """S_HALTED→S_IDLE: board_num is cleared to 0 when returning to S_IDLE"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_halted_to_idle_clears_board_num")

    async def drive():
        # Halt with a per-board error so board_num != 0 in S_HALTED
        await tb.reach_s_running()
        await RisingEdge(dut.clk)
        await ReadWrite()
        dut.over_thresh.value = 0b00001000   # board 3
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")
        dut.over_thresh.value = 0

        # Verify board_num=3 in S_HALTED
        info = tb.extract_state_and_status()
        assert info["board_num"] == 3, \
            f"Expected board_num=3 in S_HALTED, got {info['board_num']}"

        # Drop both enables → S_IDLE
        await RisingEdge(dut.clk)
        await ReadWrite()
        dut.ctrl_en.value = 0
        dut.pow_en.value = 0
        await tb._wait_for_state_rw(tb.get_state_value("S_IDLE"), "S_IDLE")

        # board_num must be cleared
        info = tb.extract_state_and_status()
        assert info["board_num"] == 0, \
            f"Expected board_num=0 after returning to S_IDLE, got {info['board_num']}"
        assert info["status_code"] == tb.get_status_value("STS_OK"), \
            f"Expected STS_OK in S_IDLE, got {info['status_name']}"

    await _run_with_scoreboard(dut, tb, drive())


# -----------------------------------------------------------------------------
# Output signal correctness at key state transitions
# -----------------------------------------------------------------------------

@cocotb.test()
async def test_output_signals_at_s_running(dut):
    """Verify all output signals are set correctly on entry to S_RUNNING"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_output_signals_at_s_running")

    await tb.reach_s_running()

    assert dut.n_shutdown_force.value == 1, "n_shutdown_force should be 1 in S_RUNNING"
    assert dut.shutdown_rst.value == 0,     "shutdown_rst should be 0 in S_RUNNING"
    assert dut.shutdown_sense_en.value == 1, "shutdown_sense_en should be 1 in S_RUNNING"
    assert dut.spi_en.value == 1,           "spi_en should be 1 in S_RUNNING"
    assert dut.spi_clk_gate.value == 1,     "spi_clk_gate should be 1 in S_RUNNING"
    assert dut.block_bufs.value == 0,       "block_bufs should be 0 in S_RUNNING"
    assert dut.unlock_cfg.value == 0,       "unlock_cfg should be 0 in S_RUNNING"
    assert dut.ps_interrupt.value == 1,     "ps_interrupt should be 1 on entry to S_RUNNING"

    # Interrupt clears on the next cycle
    await RisingEdge(dut.clk)
    await ReadWrite()
    assert dut.ps_interrupt.value == 0, "ps_interrupt should clear after one cycle in S_RUNNING"


@cocotb.test()
async def test_output_signals_at_s_halted(dut):
    """Verify all output signals are reset correctly in S_HALTED"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_output_signals_at_s_halted")

    await tb.reach_s_halted()

    assert dut.n_shutdown_force.value == 0,  "n_shutdown_force should be 0 in S_HALTED"
    assert dut.shutdown_rst.value == 0,      "shutdown_rst should be 0 in S_HALTED"
    assert dut.shutdown_sense_en.value == 0, "shutdown_sense_en should be 0 in S_HALTED"
    assert dut.spi_en.value == 0,            "spi_en should be 0 in S_HALTED"
    assert dut.spi_clk_gate.value == 0,      "spi_clk_gate should be 0 in S_HALTED"
    assert dut.block_bufs.value == 1,        "block_bufs should be 1 in S_HALTED"
    assert dut.unlock_cfg.value == 1,        "unlock_cfg should be 1 in S_HALTED"
    assert dut.ps_interrupt.value == 1,      "ps_interrupt should be 1 on entry to S_HALTED"

    # Interrupt clears on the next cycle
    await RisingEdge(dut.clk)
    await ReadWrite()
    assert dut.ps_interrupt.value == 0, "ps_interrupt should clear after one cycle in S_HALTED"


@cocotb.test()
async def test_cfg_locked_in_startup_and_running(dut):
    """unlock_cfg=0 from S_CONFIRM_SPI_RST through S_RUNNING; =1 after halt"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_cfg_locked_in_startup_and_running")

    await tb.reach_s_confirm_spi_rst()
    assert dut.unlock_cfg.value == 0, "unlock_cfg should be 0 in S_CONFIRM_SPI_RST"

    await tb.reach_s_running()
    assert dut.unlock_cfg.value == 0, "unlock_cfg should be 0 in S_RUNNING"

    await tb.reach_s_halted()
    assert dut.unlock_cfg.value == 1, "unlock_cfg should be 1 in S_HALTED"


# -----------------------------------------------------------------------------
# S_IDLE: stays idle without ctrl_en
# -----------------------------------------------------------------------------

@cocotb.test()
async def test_idle_stays_idle_without_ctrl_en(dut):
    """S_IDLE: with ctrl_en=0 the DUT stays in S_IDLE indefinitely"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_idle_stays_idle_without_ctrl_en")

    await tb.reach_s_idle()
    # ctrl_en remains 0 (reset default)
    for _ in range(20):
        await RisingEdge(dut.clk)
        await ReadWrite()
        await tb.check_state(tb.get_state_value("S_IDLE"))


# -----------------------------------------------------------------------------
# S_IDLE OOB priority
# The if/else chain in S_IDLE means the first OOB signal in priority order wins.
# -----------------------------------------------------------------------------

@cocotb.test()
async def test_idle_oob_priority(dut):
    """S_IDLE: higher-priority OOB wins when two are asserted simultaneously"""
    # Each tuple: (higher-priority signal, lower-priority signal, expected status)
    priority_pairs = [
        ("ctrl_en_oob",       "pow_en_oob",        "STS_CTRL_EN_OOB"),
        ("pow_en_oob",        "cmd_buf_reset_oob",  "STS_POW_EN_OOB"),
        ("cmd_buf_reset_oob", "data_buf_reset_oob", "STS_CMD_BUF_RESET_OOB"),
        ("mosi_sck_pol_oob",  "miso_sck_pol_oob",  "STS_MOSI_SCK_POL_OOB"),
    ]
    for high_sig, low_sig, expected_status in priority_pairs:
        tb = await setup_testbench(dut)
        tb.dut._log.info(
            f"test_idle_oob_priority: {high_sig} over {low_sig} → {expected_status}")

        async def drive(h=high_sig, l=low_sig):
            await tb.reach_s_idle()
            getattr(dut, h).value = 1
            getattr(dut, l).value = 1
            dut.ctrl_en.value = 1
            dut.ext_en.value = 1
            await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

        await _run_with_scoreboard(dut, tb, drive())
        getattr(dut, high_sig).value = 0
        getattr(dut, low_sig).value = 0


# -----------------------------------------------------------------------------
# S_CONFIRM_SPI_RST: timer guard — transition requires timer >= 10
# -----------------------------------------------------------------------------

@cocotb.test()
async def test_confirm_spi_rst_timer_guard(dut):
    """S_CONFIRM_SPI_RST: even with spi_off=1 and calc_n_cs_done=1 the DUT
    must count at least 10 cycles before advancing to S_POWER_ON_CRTL_BRD"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_confirm_spi_rst_timer_guard")

    await tb.reach_s_confirm_spi_rst()
    # Assert calc_n_cs_done immediately; spi_off is already 1 from defaults
    dut.calc_n_cs_done.value = 1
    # For the first 9 cycles (timer = 1..9) the DUT must stay in S_CONFIRM_SPI_RST
    for _ in range(9):
        await RisingEdge(dut.clk)
        await ReadWrite()
        await tb.check_state(tb.get_state_value("S_CONFIRM_SPI_RST"))

    # Clean exit: drop ctrl_en so the DUT halts without running the full timeout
    dut.ctrl_en.value = 0
    await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")
    dut.calc_n_cs_done.value = 0


@cocotb.test()
async def test_confirm_spi_rst_no_transition_without_calc_n_cs_done(dut):
    """S_CONFIRM_SPI_RST: spi_off=1 but calc_n_cs_done never asserts → STS_SPI_RESET_TIMEOUT"""
    tb = await setup_testbench(dut)
    tb.dut._log.info(
        "STARTING TEST: test_confirm_spi_rst_no_transition_without_calc_n_cs_done")

    async def drive():
        await tb.reach_s_idle()
        dut.ctrl_en.value = 1
        dut.ext_en.value = 1
        # spi_off=1 (default); calc_n_cs_done stays 0 → timer expires
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())


# -----------------------------------------------------------------------------
# S_CONFIRM_SPI_START: dac_boot_fail takes priority over adc_boot_fail
# -----------------------------------------------------------------------------

@cocotb.test()
async def test_confirm_spi_start_dac_boot_fail_over_adc_boot_fail(dut):
    """S_CONFIRM_SPI_START: dac_boot_fail and adc_boot_fail both asserted →
    STS_DAC_BOOT_FAIL wins (dac checked first in Verilog if/else chain)"""
    tb = await setup_testbench(dut)
    tb.dut._log.info(
        "STARTING TEST: test_confirm_spi_start_dac_boot_fail_over_adc_boot_fail")

    async def drive():
        await tb.reach_s_confirm_spi_start()
        dut.adc_boot_fail.value = 0x01   # lower priority
        dut.dac_boot_fail.value = 0x01   # higher priority
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())
    dut.dac_boot_fail.value = 0
    dut.adc_boot_fail.value = 0


# -----------------------------------------------------------------------------
# S_WAIT_FOR_POW_EN: stays waiting until pow_en goes high
# -----------------------------------------------------------------------------

@cocotb.test()
async def test_wait_for_pow_en_stays_waiting(dut):
    """S_WAIT_FOR_POW_EN: with pow_en=0 the DUT stays in state for many cycles,
    then advances when pow_en goes high"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_wait_for_pow_en_stays_waiting")

    async def drive():
        await tb.reach_s_wait_for_pow_en()
        # pow_en is 0 (not yet asserted by reach); verify we stay here
        for _ in range(20):
            await RisingEdge(dut.clk)
            await ReadWrite()
            await tb.check_state(tb.get_state_value("S_WAIT_FOR_POW_EN"))
        # Now assert pow_en to advance the state machine
        dut.pow_en.value = 1
        await tb._wait_for_state_rw(tb.get_state_value("S_RUNNING"), "S_RUNNING")

    await _run_with_scoreboard(dut, tb, drive())


# -----------------------------------------------------------------------------
# S_RUNNING priority tests
# -----------------------------------------------------------------------------

@cocotb.test()
async def test_running_mosi_priority_over_miso(dut):
    """S_RUNNING: mosi_sck_pol_oob + miso_sck_pol_oob → STS_MOSI_SCK_POL_OOB wins"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_running_mosi_priority_over_miso")
    await tb.reset()

    async def drive():
        await tb.reach_s_running()
        await RisingEdge(dut.clk)
        await ReadWrite()
        dut.miso_sck_pol_oob.value = 1
        dut.mosi_sck_pol_oob.value = 1   # higher priority
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())
    dut.mosi_sck_pol_oob.value = 0
    dut.miso_sck_pol_oob.value = 0


@cocotb.test()
async def test_running_miso_priority_over_shutdown_sense(dut):
    """S_RUNNING: miso_sck_pol_oob + shutdown_sense → STS_MISO_SCK_POL_OOB wins"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_running_miso_priority_over_shutdown_sense")

    async def drive():
        await tb.reach_s_running()
        await RisingEdge(dut.clk)
        await ReadWrite()
        dut.shutdown_sense.value = 0x01
        dut.miso_sck_pol_oob.value = 1   # higher priority
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())
    dut.miso_sck_pol_oob.value = 0
    dut.shutdown_sense.value = 0


@cocotb.test()
async def test_running_ext_en_priority_over_over_thresh(dut):
    """S_RUNNING: ext_en=0 + over_thresh → STS_EXT_SHUTDOWN wins"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_running_ext_en_priority_over_over_thresh")

    async def drive():
        await tb.reach_s_running()
        await RisingEdge(dut.clk)
        await ReadWrite()
        dut.over_thresh.value = 0x01
        dut.ext_en.value = 0             # higher priority than over_thresh
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())
    dut.over_thresh.value = 0


@cocotb.test()
async def test_running_over_thresh_priority_over_thresh_underflow(dut):
    """S_RUNNING: over_thresh + thresh_underflow → STS_OVER_THRESH wins"""
    tb = await setup_testbench(dut)
    tb.dut._log.info(
        "STARTING TEST: test_running_over_thresh_priority_over_thresh_underflow")

    async def drive():
        await tb.reach_s_running()
        await RisingEdge(dut.clk)
        await ReadWrite()
        dut.thresh_underflow.value = 0x01
        dut.over_thresh.value = 0x01     # higher priority
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())
    dut.over_thresh.value = 0
    dut.thresh_underflow.value = 0


@cocotb.test()
async def test_running_bad_trig_priority_over_trig_buf_overflow(dut):
    """S_RUNNING: bad_trig_cmd + trig_cmd_buf_overflow → STS_BAD_TRIG_CMD wins"""
    tb = await setup_testbench(dut)
    tb.dut._log.info(
        "STARTING TEST: test_running_bad_trig_priority_over_trig_buf_overflow")

    async def drive():
        await tb.reach_s_running()
        await RisingEdge(dut.clk)
        await ReadWrite()
        dut.trig_cmd_buf_overflow.value = 1
        dut.bad_trig_cmd.value = 1       # higher priority
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())
    dut.bad_trig_cmd.value = 0
    dut.trig_cmd_buf_overflow.value = 0


@cocotb.test()
async def test_running_bad_dac_cmd_priority_over_adc_delay(dut):
    """S_RUNNING: bad_dac_cmd + adc_delay_too_short → STS_BAD_DAC_CMD wins"""
    tb = await setup_testbench(dut)
    tb.dut._log.info(
        "STARTING TEST: test_running_bad_dac_cmd_priority_over_adc_delay")

    async def drive():
        await tb.reach_s_running()
        await RisingEdge(dut.clk)
        await ReadWrite()
        dut.adc_delay_too_short.value = 0x01
        dut.bad_dac_cmd.value = 0x01     # higher priority
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())
    dut.bad_dac_cmd.value = 0
    dut.adc_delay_too_short.value = 0


# -----------------------------------------------------------------------------
# Board number: extract_board_num returns the lowest set bit
# -----------------------------------------------------------------------------

@cocotb.test()
async def test_board_num_multiple_bits_lowest_wins(dut):
    """S_RUNNING: when multiple board bits set in over_thresh, board_num is the
    lowest set bit (extract_board_num returns first set bit 0-7)"""
    # Test vectors: (over_thresh value, expected lowest bit)
    vectors = [
        (0b00000011, 0),   # bits 0 and 1 set → board 0
        (0b00000110, 1),   # bits 1 and 2 set → board 1
        (0b00001100, 2),   # bits 2 and 3 set → board 2
        (0b00011000, 3),   # bits 3 and 4 set → board 3
        (0b11000000, 6),   # bits 6 and 7 set → board 6
        (0b10000000, 7),   # bit 7 only         → board 7
        (0xFF,       0),   # all bits set       → board 0
    ]
    for mask, expected_board in vectors:
        tb = await setup_testbench(dut)
        tb.dut._log.info(
            f"test_board_num_multiple_bits_lowest_wins: mask=0x{mask:02X} → board {expected_board}")

        async def drive(m=mask, eb=expected_board):
            await tb.reach_s_running()
            await RisingEdge(dut.clk)
            await ReadWrite()
            dut.over_thresh.value = m
            await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")
            dut.over_thresh.value = 0
            info = tb.extract_state_and_status()
            assert info["board_num"] == eb, \
                f"mask=0x{m:02X}: expected board_num={eb}, got {info['board_num']}"

        await _run_with_scoreboard(dut, tb, drive())


# -----------------------------------------------------------------------------
# Status word bit composition
# -----------------------------------------------------------------------------

@cocotb.test()
async def test_status_word_composition(dut):
    """Status word [31:29]=board_num, [28:4]=status_code, [3:0]=state is
    correctly packed in S_HALTED after a per-board error"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_status_word_composition")

    target_board = 5

    async def drive():
        await tb.reach_s_running()
        await RisingEdge(dut.clk)
        await ReadWrite()
        dut.over_thresh.value = 1 << target_board
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")
        dut.over_thresh.value = 0

        raw = int(dut.status_word.value)
        extracted_state  = raw & 0xF
        extracted_status = (raw >> 4) & 0x1FFFFFF
        extracted_board  = (raw >> 29) & 0x7

        assert extracted_state == tb.get_state_value("S_HALTED"), \
            f"status_word state bits: expected {tb.get_state_value('S_HALTED')}, got {extracted_state}"
        assert extracted_status == tb.get_status_value("STS_OVER_THRESH"), \
            f"status_word status bits: expected STS_OVER_THRESH " \
            f"(0x{tb.get_status_value('STS_OVER_THRESH'):05X}), got 0x{extracted_status:05X}"
        assert extracted_board == target_board, \
            f"status_word board bits: expected {target_board}, got {extracted_board}"

        # Allow halted_scoreboard to see the S_IDLE transition
        await RisingEdge(dut.clk)
        await ReadWrite()
        dut.ctrl_en.value = 0
        dut.pow_en.value = 0
        await tb._wait_for_state_rw(tb.get_state_value("S_IDLE"), "S_IDLE")

    await _run_with_scoreboard(dut, tb, drive())


# -----------------------------------------------------------------------------
# Repeated start/halt cycle
# -----------------------------------------------------------------------------

@cocotb.test()
async def test_repeated_start_halt_cycle(dut):
    """Run three complete IDLE→RUNNING→HALTED→IDLE cycles verifying clean
    state (STS_OK, board_num=0) on each return to S_IDLE"""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_repeated_start_halt_cycle")

    for cycle in range(3):
        tb.dut._log.info(f"test_repeated_start_halt_cycle: cycle {cycle}")

        async def drive(c=cycle):
            await tb.reach_s_running()
            await RisingEdge(dut.clk)
            await ReadWrite()
            dut.ctrl_en.value = 0   # PS_SHUTDOWN halt
            await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")
            await RisingEdge(dut.clk)
            await ReadWrite()
            dut.pow_en.value = 0
            await tb._wait_for_state_rw(tb.get_state_value("S_IDLE"), "S_IDLE")
            info = tb.extract_state_and_status()
            assert info["status_code"] == tb.get_status_value("STS_OK"), \
                f"Cycle {c}: expected STS_OK in S_IDLE, got {info['status_name']}"
            assert info["board_num"] == 0, \
                f"Cycle {c}: expected board_num=0 in S_IDLE, got {info['board_num']}"

        await _run_with_scoreboard(dut, tb, drive())


# -----------------------------------------------------------------------------
# Additional edge cases
# -----------------------------------------------------------------------------

@cocotb.test()
async def test_idle_ext_en_priority_over_oob(dut):
    """S_IDLE: ext_en=0 and OOB asserted together → STS_EXT_SHUTDOWN wins."""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_idle_ext_en_priority_over_oob")

    async def drive():
        await tb.reach_s_idle()
        dut.ctrl_en.value = 1
        dut.ext_en.value = 0
        dut.ctrl_en_oob.value = 1
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")
        await tb.check_state_and_status(
            expected_state=tb.get_state_value("S_HALTED"),
            expected_status_code=tb.get_status_value("STS_EXT_SHUTDOWN")
        )

    await _run_with_scoreboard(dut, tb, drive())
    dut.ctrl_en_oob.value = 0


@cocotb.test()
async def test_confirm_spi_start_spi_off_priority_over_boot_fail(dut):
    """S_CONFIRM_SPI_START: spi_off=0 wins over boot-fail flags in same cycle."""
    tb = await setup_testbench(dut)
    tb.dut._log.info(
        "STARTING TEST: test_confirm_spi_start_spi_off_priority_over_boot_fail")

    async def drive():
        await tb.reach_s_confirm_spi_start()
        # In this state, !spi_off is checked before boot-fail conditions.
        dut.dac_boot_fail.value = 0x80
        dut.adc_boot_fail.value = 0x01
        dut.spi_off.value = 0
        await tb._wait_for_state_rw(tb.get_state_value("S_WAIT_FOR_POW_EN"), "S_WAIT_FOR_POW_EN")
        await tb.check_state_and_status(
            expected_state=tb.get_state_value("S_WAIT_FOR_POW_EN"),
            expected_status_code=tb.get_status_value("STS_OK"),
            expected_board_num=0
        )

        # Clean halt so scoreboard can complete.
        dut.dac_boot_fail.value = 0
        dut.adc_boot_fail.value = 0
        dut.ctrl_en.value = 0
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())


@cocotb.test()
async def test_running_entry_interrupt_masks_single_cycle_error_pulse(dut):
    """S_RUNNING entry: one-cycle error pulse during interrupt clear is ignored."""
    tb = await setup_testbench(dut)
    tb.dut._log.info(
        "STARTING TEST: test_running_entry_interrupt_masks_single_cycle_error_pulse")

    async def drive():
        await tb.reach_s_running()
        # ps_interrupt=1 on entry; this cycle clears interrupt and skips error checks.
        dut.lock_viol.value = 1
        await RisingEdge(dut.clk)
        await ReadWrite()
        await tb.check_state_and_status(
            expected_state=tb.get_state_value("S_RUNNING"),
            expected_status_code=tb.get_status_value("STS_OK"),
            expected_board_num=0
        )
        assert dut.ps_interrupt.value == 0, "ps_interrupt should clear on first S_RUNNING cycle"

        # Clear the pulse before the next cycle; DUT should keep running.
        dut.lock_viol.value = 0
        for _ in range(4):
            await RisingEdge(dut.clk)
            await ReadWrite()
            await tb.check_state(tb.get_state_value("S_RUNNING"))

        # End test by halting cleanly.
        dut.ctrl_en.value = 0
        await tb._wait_for_state_rw(tb.get_state_value("S_HALTED"), "S_HALTED")

    await _run_with_scoreboard(dut, tb, drive())

@cocotb.test()
async def test_illegal_state_default_branch_recovery(dut):
    """Inject an illegal state and verify default-branch safe halt behavior."""
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_illegal_state_default_branch_recovery")

    await tb.reach_s_idle()
    dut.state.value = 0  # Not a valid encoded state; should hit default branch.
    await RisingEdge(dut.clk)
    await ReadWrite()

    await tb.check_state_and_status(
        expected_state=tb.get_state_value("S_HALTED"),
        expected_status_code=tb.get_status_value("STS_EMPTY"),
        expected_board_num=0
    )
    assert dut.n_shutdown_force.value == 0, "n_shutdown_force should be 0 in default halt path"
    assert dut.shutdown_rst.value == 0, "shutdown_rst should be 0 in default halt path"
    assert dut.shutdown_sense_en.value == 0, "shutdown_sense_en should be 0 in default halt path"
    assert dut.unlock_cfg.value == 1, "unlock_cfg should be 1 in default halt path"
    assert dut.spi_clk_gate.value == 0, "spi_clk_gate should be 0 in default halt path"
    assert dut.spi_en.value == 0, "spi_en should be 0 in default halt path"
    assert dut.block_bufs.value == 1, "block_bufs should be 1 in default halt path"
    assert dut.ps_interrupt.value == 1, "ps_interrupt should assert in default halt path"

    # With both enables low, next S_HALTED cycle returns to IDLE and clears status.
    dut.ctrl_en.value = 0
    dut.pow_en.value = 0
    await RisingEdge(dut.clk)
    await ReadWrite()
    await tb.check_state_and_status(
        expected_state=tb.get_state_value("S_IDLE"),
        expected_status_code=tb.get_status_value("STS_OK"),
        expected_board_num=0
    )
