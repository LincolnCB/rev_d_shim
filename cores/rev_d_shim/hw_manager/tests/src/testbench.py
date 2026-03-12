import cocotb
from cocotb.triggers import RisingEdge , Timer, ReadOnly, FallingEdge
from hw_manager_base import hw_manager_base
from hw_manager_coverage import start_coverage_monitor
import random

RANDOM_SEED = 42

# Create a setup function that can be called by each test
async def setup_testbench(dut):
    # Delay parameters should be the same with hw_manager.v
    tb = hw_manager_base(dut, clk_period=4, time_unit="ns",
                        SHUTDOWN_FORCE_DELAY=250,
                        SHUTDOWN_RESET_PULSE=25,
                        SHUTDOWN_RESET_DELAY=250,
                        SPI_RESET_WAIT=2500,
                        SPI_START_WAIT=2500)
    return tb

# Helper function to set up the testbench and go to RUNNING state
async def normal_start_up_with_no_log(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("SYSTEM WILL GO TO S_RUNNING STATE: normal_start_up_with_no_log")

    # Initialize and reset
    await tb.reset()

    # Enable the system
    dut.sys_en.value = 1
    dut.spi_off.value = 1  # SPI initially off

    # Wait for S_CONFIRM_SPI_START state
    await tb.wait_for_state(tb.get_state_value("S_CONFIRM_SPI_START"),
                            None, True,
                            tb.SPI_RESET_WAIT + tb.SHUTDOWN_FORCE_DELAY + 5)
    
    # Simulate SPI subsystem being off (reset)
    await RisingEdge(dut.clk)
    dut.spi_off.value = 0  # SPI is running
    await RisingEdge(dut.clk)

    # Go through the power-on sequence
    # Wait for S_RUNNING state
    await tb.wait_for_state(tb.get_state_value("S_RUNNING"),
                            None, True,
                            tb.SHUTDOWN_RESET_DELAY + tb.SHUTDOWN_RESET_PULSE + 5)

    # Now should be in S_RUNNING state
    # wait one cycle for interrupt to be cleared
    state = int(dut.state.value)
    assert state == tb.get_state_value("S_RUNNING"), f"Expected state S_RUNNING, got {tb.get_state_name(state)}"
    await RisingEdge(dut.clk) # One extra clock to clear the interrupt
    await RisingEdge(dut.clk)
    assert dut.ps_interrupt.value == 0, "Interrupt should be cleared"
    tb.dut._log.info("now in RUNNING state")
    return tb


# At the S_IDLE state if there is an error in the configuration, the system should go to S_HALTED state.
@cocotb.test()
async def test_configuration_errors(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_configuration_errors")
    
    # Start coverage monitor
    start_coverage_monitor(dut)

    # Error conditions to test for
    error_conditions = [
        (dut.integ_thresh_avg_oob, "STS_INTEG_THRESH_AVG_OOB"),
        (dut.integ_window_oob, "STS_INTEG_WINDOW_OOB"),
        (dut.integ_en_oob, "STS_INTEG_EN_OOB"),
        (dut.sys_en_oob, "STS_SYS_EN_OOB")
    ]

    # Iterate through each error condition
    for error_signal, status_name in error_conditions:
        expected_status = tb.get_status_value(status_name)
        await tb.reset()
        tb.dut._log.info(f"TESTING ERROR CONDITION: {status_name}")
        error_signal.value = 1  # Set the error signal to 1
        dut.sys_en.value = 1  # Enable the system
        tb.dut._log.info(f"sys_en = {dut.sys_en.value}, {status_name} = {error_signal.value}")
        await tb.check_state_and_status(1, 1)  # S_IDLE = 1, STS_OK = 1

        await tb.wait_for_state(tb.get_state_value("S_HALTING"))
        await tb.wait_for_state(tb.get_state_value("S_HALTED"))
        tb.dut._log.info(f"sys_en = {dut.sys_en.value}, {status_name} = {error_signal.value}")
        await tb.check_state_and_status(tb.get_state_value("S_HALTED"), expected_status)  # Check that we are in S_HALTED state with the expected status
        await RisingEdge(dut.clk)
        error_signal.value = 0 # Reset the error signal
        dut.sys_en.value = 0  # Reset the sys_en signal

# In a normal startup, the system should go from S_IDLE to RUNNING state. This test checks the state transitions for that.
@cocotb.test()
async def test_normal_startup(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_normal_startup")

    # STS_OK value
    STS_OK = tb.get_status_value("STS_OK")

    # Initialize and reset
    await tb.reset()

    # Start coverage monitor
    start_coverage_monitor(dut)
    
    # Verify we start in S_IDLE state
    await tb.check_state_and_status(tb.get_state_value("S_IDLE"), STS_OK)  # S_IDLE = 1, STS_OK = 1
    
    # Enable the system
    dut.sys_en.value = 1
    dut.spi_off.value = 1  # SPI initially off (clock not running)
    
    # Wait for S_CONFIRM_SPI_RST state
    await tb.wait_for_state(tb.get_state_value("S_CONFIRM_SPI_RST"))
    await tb.check_state_and_status(tb.get_state_value("S_CONFIRM_SPI_RST"), STS_OK)
    
    # Simulate SPI being reset (off)
    await tb.wait_cycles(10)
    dut.spi_off.value = 1  # Confirm SPI is off (reset)
    
    # Wait for S_POWER_ON_CRTL_BRD state
    await tb.wait_for_state(tb.get_state_value("S_POWER_ON_CRTL_BRD"), None, False, tb.SPI_RESET_WAIT + 1000)
    await tb.check_state_and_status(tb.get_state_value("S_POWER_ON_CRTL_BRD"), STS_OK)

    # Wait for S_CONFIRM_SPI_START state
    await tb.wait_for_state(tb.get_state_value("S_CONFIRM_SPI_START"), None, False, tb.SHUTDOWN_FORCE_DELAY + 1000)
    await tb.check_state_and_status(tb.get_state_value("S_CONFIRM_SPI_START"), STS_OK)
    await tb.wait_cycles(100)  # Wait for a few cycles to set spi_running
    dut.spi_off.value = 0  # Set SPI running (not off)

    # Wait for S_POWER_ON_AMP_BRD state
    await tb.wait_for_state(tb.get_state_value("S_POWER_ON_AMP_BRD"), None, False, tb.SPI_START_WAIT + 1000)
    await tb.check_state_and_status(tb.get_state_value("S_POWER_ON_AMP_BRD"), STS_OK)

    # Wait for S_AMP_POWER_WAIT state
    await tb.wait_for_state(tb.get_state_value("S_AMP_POWER_WAIT"), None, False, tb.SHUTDOWN_RESET_PULSE + 1000)
    await tb.check_state_and_status(tb.get_state_value("S_AMP_POWER_WAIT"), STS_OK)

    # Should reach S_RUNNING state
    await tb.wait_for_state(tb.get_state_value("S_RUNNING"), None, False, tb.SHUTDOWN_RESET_DELAY + 1000)

    # Check that system is properly enabled
    assert dut.n_shutdown_force.value == 1, "Shutdown force should be released"
    assert dut.shutdown_rst.value == 0, "Shutdown reset should be released"
    assert dut.shutdown_sense_en.value == 1, "Shutdown sense should be enabled"
    assert dut.spi_en.value == 1, "SPI should be enabled"
    assert dut.block_bufs.value == 0, "Buffers should be unblocked"
    assert dut.ps_interrupt.value == 1, "Interrupt should be asserted"

    await RisingEdge(dut.clk)
    await ReadOnly()  # Ensure all signals are updated
    assert dut.ps_interrupt.value == 0, "Interrupt should be cleared"

    # Verify we're in S_RUNNING state with STS_OK
    await tb.check_state_and_status(tb.get_state_value("S_RUNNING"), STS_OK)

# When in S_HALTED state, the system should be able to go back to S_IDLE state when sys_en goes low
@cocotb.test()
async def test_halted_to_idle(dut):
    tb = await normal_start_up_with_no_log(dut)
    tb.dut._log.info("STARTING TEST: test_halted_to_idle")

    # Start coverage monitor
    start_coverage_monitor(dut)

    # Simulate a runtime error
    await RisingEdge(dut.clk)
    dut.lock_viol.value = 1
    await RisingEdge(dut.clk) # where the error is detected
    await RisingEdge(dut.clk) # where the state is updated
    # Check that the system is in S_HALTING state
    await tb.check_state_and_status(tb.get_state_value("S_HALTING"), tb.get_status_value("STS_LOCK_VIOL"))
    await RisingEdge(dut.clk) # where the state is updated
    # Check that the system is in S_HALTED state
    await tb.check_state_and_status(tb.get_state_value("S_HALTED"), tb.get_status_value("STS_LOCK_VIOL"))
    assert dut.n_shutdown_force.value == 0, "Expected shutdown force"
    assert dut.shutdown_sense_en.value == 0, "Expected shutdown sense disabled"
    assert dut.spi_clk_gate.value == 0, "Expected SPI clock gate disabled"
    assert dut.spi_en.value == 0, "Expected SPI disabled"
    assert dut.block_bufs.value == 1, "Expected buffers to be blocked"
    assert dut.ps_interrupt.value == 1, "Expected interrupt asserted"

    await RisingEdge(dut.clk)
    dut.sys_en.value = 0  # Enable signal goes low
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await tb.check_state_and_status(tb.get_state_value("S_IDLE"), tb.get_status_value("STS_OK"))
    assert dut.unlock_cfg.value == 1, "Expected unlock_cfg to be 1"


# Test runtime errors in S_RUNNING state, system should go to S_HALTED state
@cocotb.test()
async def test_runtime_errors(dut):

    # Start coverage monitor
    start_coverage_monitor(dut)
    error_conditions = [
        (dut.sys_en,                  "STS_PS_SHUTDOWN"),
        (dut.lock_viol,               "STS_LOCK_VIOL"),
        (dut.shutdown_sense,          "STS_SHUTDOWN_SENSE"),
        (dut.ext_shutdown,            "STS_EXT_SHUTDOWN"),
        (dut.over_thresh,             "STS_OVER_THRESH"),
        (dut.thresh_underflow,        "STS_THRESH_UNDERFLOW"),
        (dut.thresh_overflow,         "STS_THRESH_OVERFLOW"),
        (dut.bad_trig_cmd,            "STS_BAD_TRIG_CMD"),
        (dut.trig_cmd_buf_overflow,   "STS_TRIG_CMD_BUF_OVERFLOW"),
        (dut.trig_data_buf_underflow, "STS_TRIG_DATA_BUF_UNDERFLOW"),
        (dut.trig_data_buf_overflow,  "STS_TRIG_DATA_BUF_OVERFLOW"),
        (dut.bad_dac_cmd,             "STS_BAD_DAC_CMD"),
        (dut.dac_cal_oob,             "STS_DAC_CAL_OOB"),
        (dut.dac_val_oob,             "STS_DAC_VAL_OOB"),
        (dut.dac_cmd_buf_underflow,   "STS_DAC_BUF_UNDERFLOW"),
        (dut.dac_cmd_buf_overflow,    "STS_DAC_BUF_OVERFLOW"),
        (dut.unexp_dac_trig,          "STS_UNEXP_DAC_TRIG"),
        (dut.bad_adc_cmd,             "STS_BAD_ADC_CMD"),
        (dut.adc_cmd_buf_underflow,   "STS_ADC_CMD_BUF_UNDERFLOW"),
        (dut.adc_cmd_buf_overflow,    "STS_ADC_CMD_BUF_OVERFLOW"),
        (dut.adc_data_buf_underflow,  "STS_ADC_DATA_BUF_UNDERFLOW"),
        (dut.adc_data_buf_overflow,   "STS_ADC_DATA_BUF_OVERFLOW"),
        (dut.unexp_adc_trig,          "STS_UNEXP_ADC_TRIG")
    ]

    for error_signal, status_name in error_conditions:
        tb = await normal_start_up_with_no_log(dut)
        expected_status = tb.get_status_value(status_name)

        tb.dut._log.info(f"Testing error condition: {status_name}")

        # Set the error signal
        await RisingEdge(dut.clk)
        if expected_status == 0x0002:
            error_signal.value = 0
        else:
            error_signal.value = 1

        await RisingEdge(dut.clk) # where the error is detected

        await RisingEdge(dut.clk) # where the state is updated
        # Check that we are in S_HALTING state
        await tb.check_state_and_status(tb.get_state_value("S_HALTING"), expected_status)
        await RisingEdge(dut.clk) # where the state is updated
        # Check that we are in S_HALTED state
        await tb.check_state_and_status(tb.get_state_value("S_HALTED"), expected_status)

        assert dut.n_shutdown_force.value == 0, "Expected shutdown force"
        assert dut.shutdown_sense_en.value == 0, "Expected shutdown sense disabled"
        assert dut.spi_clk_gate.value == 0, "Expected SPI clock gate disabled"
        assert dut.spi_en.value == 0, "Expected SPI disabled"
        assert dut.block_bufs.value == 1, "Expected buffers to be blocked"
        assert dut.ps_interrupt.value == 1, "Expected interrupt asserted"

# Test per board errors explicitly in S_RUNNING state, system should go to S_HALTED state with the error on corresponding board
# Don't check every board, just check a random non-zero board number for each to see that some board number is extracted correctly
# Full board number extraction tested separately in general
# This test assumes that some non-zero board extraction means it uses that working system
@cocotb.test()
async def test_per_board_errors(dut):

    # Start coverage monitor
    start_coverage_monitor(dut)

    # Each tuple: (signal, status_code, status_name)
    error_conditions = [
        (dut.shutdown_sense,          "STS_SHUTDOWN_SENSE"),
        (dut.over_thresh,             "STS_OVER_THRESH"),
        (dut.thresh_underflow,        "STS_THRESH_UNDERFLOW"),
        (dut.thresh_overflow,         "STS_THRESH_OVERFLOW"),
        (dut.bad_dac_cmd,             "STS_BAD_DAC_CMD"),
        (dut.dac_cal_oob,             "STS_DAC_CAL_OOB"),
        (dut.dac_val_oob,             "STS_DAC_VAL_OOB"),
        (dut.dac_cmd_buf_underflow,   "STS_DAC_BUF_UNDERFLOW"),
        (dut.dac_cmd_buf_overflow,    "STS_DAC_BUF_OVERFLOW"),
        (dut.unexp_dac_trig,          "STS_UNEXP_DAC_TRIG"),
        (dut.bad_adc_cmd,             "STS_BAD_ADC_CMD"),
        (dut.adc_cmd_buf_underflow,   "STS_ADC_CMD_BUF_UNDERFLOW"),
        (dut.adc_cmd_buf_overflow,    "STS_ADC_CMD_BUF_OVERFLOW"),
        (dut.adc_data_buf_underflow,  "STS_ADC_DATA_BUF_UNDERFLOW"),
        (dut.adc_data_buf_overflow,   "STS_ADC_DATA_BUF_OVERFLOW"),
        (dut.unexp_adc_trig,          "STS_UNEXP_ADC_TRIG")
    ]

    for error_signal, status_name in error_conditions:
        # For each error condition, randomly choose a board number between 1 and 7 
        # (0 would be the default, which would happen if the board number extraction fails)
        random.seed(RANDOM_SEED)
        i = random.randint(1, 7)  # Random board number from 1 to 7
        tb = await normal_start_up_with_no_log(dut)
        tb.dut._log.info(f"Testing per board error condition: {status_name} (board {i})")

        # Set the error signal for board i
        await RisingEdge(dut.clk)
        error_signal.value = 1 << i

        await RisingEdge(dut.clk) # where the error is detected

        expected_status = tb.get_status_value(status_name)
        await RisingEdge(dut.clk) # where the state is updated
        # Check that we are in S_HALTING state with correct board_num
        await tb.check_state_and_status(tb.get_state_value("S_HALTING"), expected_status, expected_board_num=i)
        await RisingEdge(dut.clk) # where the state is updated
        # Check that we are in S_HALTED state with correct board_num
        await tb.check_state_and_status(tb.get_state_value("S_HALTED"), expected_status, expected_board_num=i)

        assert dut.n_shutdown_force.value == 0, "Expected shutdown force"
        assert dut.shutdown_sense_en.value == 0, "Expected shutdown sense disabled"
        assert dut.spi_clk_gate.value == 0, "Expected SPI clock gate disabled"
        assert dut.spi_en.value == 0, "Expected SPI disabled"
        assert dut.block_bufs.value == 1, "Expected buffers to be blocked"
        assert dut.ps_interrupt.value == 1, "Expected interrupt asserted"

# Test SPI reset timeout, the system should go to S_HALTED state
@cocotb.test()
async def test_spi_reset_timeout(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_spi_reset_timeout")

    # Initialize and reset
    await tb.reset()

    # Start coverage monitor
    start_coverage_monitor(dut)

    # Enable the system
    dut.sys_en.value = 1
    dut.spi_off.value = 0  # SPI is not off (should trigger timeout)

    # Wait for S_CONFIRM_SPI_RST state
    await tb.wait_for_state(tb.get_state_value("S_CONFIRM_SPI_RST"), None, True, 10)
    await RisingEdge(dut.clk)

    # Simulate SPI reset timeout
    tb.dut._log.info(f"SPI_RESET_WAIT parameter is {tb.SPI_RESET_WAIT}")
    tb.dut._log.info(f"Simulating SPI reset timeout, waiting for {tb.SPI_RESET_WAIT} cycles")
    await tb.wait_cycles(tb.SPI_RESET_WAIT)
    tb.dut._log.info(f"timer value is {int(dut.timer.value)}, maximum wait time was {tb.SPI_RESET_WAIT}")

    await RisingEdge(dut.clk)
    await tb.check_state_and_status(tb.get_state_value("S_HALTING"), tb.get_status_value("STS_SPI_RESET_TIMEOUT"))
    await RisingEdge(dut.clk)
    await tb.check_state_and_status(tb.get_state_value("S_HALTED"), tb.get_status_value("STS_SPI_RESET_TIMEOUT"))
    assert dut.n_shutdown_force.value == 0, "Expected shutdown force"
    assert dut.shutdown_sense_en.value == 0, "Expected shutdown sense disabled"
    assert dut.spi_clk_gate.value == 0, "Expected SPI clock gate disabled"
    assert dut.spi_en.value == 0, "Expected SPI disabled"
    assert dut.block_bufs.value == 1, "Expected buffers to be blocked"
    assert dut.ps_interrupt.value == 1, "Expected interrupt asserted"

# Test SPI start timeout, the system should go to S_HALTED state
@cocotb.test()
async def test_spi_start_timeout(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_spi_start_timeout")

    # Initialize and reset
    await tb.reset()

    # Start coverage monitor
    start_coverage_monitor(dut)

    # Enable the system
    dut.sys_en.value = 1
    dut.spi_off.value = 1  # SPI starts off

    # Wait for S_CONFIRM_SPI_START state
    await tb.wait_for_state(tb.get_state_value("S_CONFIRM_SPI_START"),
                            None, True,
                            tb.SPI_RESET_WAIT + tb.SHUTDOWN_FORCE_DELAY + 5)

    # Keep SPI off (doesn't start)
    await RisingEdge(dut.clk)
    dut.spi_off.value = 1

    # Simulate SPI start timeout
    tb.dut._log.info(f"SPI_START_WAIT parameter is {tb.SPI_START_WAIT}")
    tb.dut._log.info(f"Simulating SPI start timeout, waiting for {tb.SPI_START_WAIT} cycles")
    await tb.wait_cycles(tb.SPI_START_WAIT)
    tb.dut._log.info(f"timer value is {int(dut.timer.value)}, maximum wait time was {tb.SPI_START_WAIT}")

    await RisingEdge(dut.clk)
    await tb.check_state_and_status(tb.get_state_value("S_HALTING"), tb.get_status_value("STS_SPI_START_TIMEOUT"))
    await RisingEdge(dut.clk)
    await tb.check_state_and_status(tb.get_state_value("S_HALTED"), tb.get_status_value("STS_SPI_START_TIMEOUT"))
    assert dut.n_shutdown_force.value == 0, "Expected shutdown force"
    assert dut.shutdown_sense_en.value == 0, "Expected shutdown sense disabled"
    assert dut.spi_clk_gate.value == 0, "Expected SPI clock gate disabled"
    assert dut.spi_en.value == 0, "Expected SPI disabled"
    assert dut.ps_interrupt.value == 1, "Expected interrupt asserted"

# Test to see if extract_board_num function in hw_manager.v works correctly
# The board number should be the position of the lowest set bit in the input vector
@cocotb.test()
async def test_extract_board_number(dut):
    tb = await normal_start_up_with_no_log(dut)
    tb.dut._log.info("STARTING TEST: test_extract_board_number")

    # Start coverage monitor
    start_coverage_monitor(dut)

    # Test each bit position for over_thresh (matches extract_board_num in hw_manager.v)
    for i in range(8):
        dut.over_thresh.value = 1 << i
        await RisingEdge(dut.clk)

        await RisingEdge(dut.clk)
        await tb.check_state(tb.get_state_value("S_HALTING"))
        await RisingEdge(dut.clk)
        await tb.check_state(tb.get_state_value("S_HALTED"))
        tb.dut._log.info(f"Checking board number {i}, got {int(dut.board_num.value)}")
        assert dut.board_num.value == i, f"Expected board number {i}, got {int(dut.board_num.value)}"

        # Reset for next iteration
        await normal_start_up_with_no_log(dut)
    
# Test to see if extract_board_num function in hw_manager.v works correctly when multiple bits are set
# The board number extracted should be the position of the lowest set bit
@cocotb.test()
async def test_extract_board_num_multiple_bits(dut):
    tb = await normal_start_up_with_no_log(dut)
    tb.dut._log.info("STARTING TEST: test_extract_board_num_multiple_bits")

    # Start coverage monitor
    start_coverage_monitor(dut)

    # Set multiple bits (e.g., bits 0, 2, 4, 5)
    dut.over_thresh.value = 0b00110101
    await RisingEdge(dut.clk)

    # Should halt and extract the lowest bit (bit 0)
    await RisingEdge(dut.clk)
    await tb.check_state(tb.get_state_value("S_HALTING"))
    await RisingEdge(dut.clk)
    await tb.check_state(tb.get_state_value("S_HALTED"))
    assert dut.board_num.value == 0, f"Expected board number 0 (lowest set bit), got {int(dut.board_num.value)}"
