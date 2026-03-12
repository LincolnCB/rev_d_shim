import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, Timer, ReadOnly
from cocotb.regression import TestFactory
from cocotb.result import TestFailure
from cocotb_coverage.coverage import CoverPoint, coverage_db, CoverCross

class hw_manager_base:
    """
    Hardware Manager cocotb Base Class
    """

    # State encoding dictionary
    STATES = {
        1: "S_IDLE",
        2: "S_CONFIRM_SPI_RST",
        3: "S_POWER_ON_CRTL_BRD",
        4: "S_CONFIRM_SPI_START",
        5: "S_POWER_ON_AMP_BRD",
        6: "S_AMP_POWER_WAIT",
        7: "S_RUNNING",
        8: "S_HALTING",
        9: "S_HALTED"
    }
    
    # Status codes dictionary
    STATUS_CODES = {
        0x0000: "STS_EMPTY",
        0x0001: "STS_OK",
        0x0002: "STS_PS_SHUTDOWN",
        0x0100: "STS_SPI_RESET_TIMEOUT",
        0x0101: "STS_SPI_START_TIMEOUT",
        0x0200: "STS_INTEG_THRESH_AVG_OOB",
        0x0201: "STS_INTEG_WINDOW_OOB",
        0x0202: "STS_INTEG_EN_OOB",
        0x0203: "STS_SYS_EN_OOB",
        0x0204: "STS_LOCK_VIOL",
        0x0300: "STS_SHUTDOWN_SENSE",
        0x0301: "STS_EXT_SHUTDOWN",
        0x0400: "STS_OVER_THRESH",
        0x0401: "STS_THRESH_UNDERFLOW",
        0x0402: "STS_THRESH_OVERFLOW",
        0x0500: "STS_BAD_TRIG_CMD",
        0x0501: "STS_TRIG_CMD_BUF_OVERFLOW",
        0x0502: "STS_TRIG_DATA_BUF_UNDERFLOW",
        0x0503: "STS_TRIG_DATA_BUF_OVERFLOW",
        0x0600: "STS_DAC_BOOT_FAIL",
        0x0601: "STS_BAD_DAC_CMD",
        0x0602: "STS_DAC_CAL_OOB",
        0x0603: "STS_DAC_VAL_OOB",
        0x0604: "STS_DAC_BUF_UNDERFLOW",
        0x0605: "STS_DAC_BUF_OVERFLOW",
        0x0606: "STS_UNEXP_DAC_TRIG",
        0x0700: "STS_ADC_BOOT_FAIL",
        0x0701: "STS_BAD_ADC_CMD",
        0x0702: "STS_ADC_CMD_BUF_UNDERFLOW",
        0x0703: "STS_ADC_CMD_BUF_OVERFLOW",
        0x0704: "STS_ADC_DATA_BUF_UNDERFLOW",
        0x0705: "STS_ADC_DATA_BUF_OVERFLOW",
        0x0706: "STS_UNEXP_ADC_TRIG",
    }

    def __init__(self, dut, clk_period = 4, time_unit = "ns", 
                 SHUTDOWN_FORCE_DELAY = 25000000,
                 SHUTDOWN_RESET_PULSE = 25000,
                 SHUTDOWN_RESET_DELAY = 25000000,
                 SPI_RESET_WAIT = 25000000,
                 SPI_START_WAIT = 25000000
                 ):
        """
        Initialize the testbench
        
        Args:
            dut: The Verilog/VHDL design under test
        """
        self.time_unit = time_unit
        
        self.SHUTDOWN_FORCE_DELAY = SHUTDOWN_FORCE_DELAY
        self.SHUTDOWN_RESET_PULSE = SHUTDOWN_RESET_PULSE
        self.SHUTDOWN_RESET_DELAY = SHUTDOWN_RESET_DELAY
        self.SPI_RESET_WAIT = SPI_RESET_WAIT
        self.SPI_START_WAIT = SPI_START_WAIT
        
        self.clk_period = clk_period
        self.dut = dut
        
        # Create clock
        cocotb.start_soon(Clock(self.dut.clk, clk_period, units=time_unit).start())  # Default is 250 MHz clock
        
        # Set default values for inputs
        self.dut.sys_en.value = 0
        self.dut.spi_off.value = 1  # SPI starts powered off
        self.dut.ext_shutdown.value = 0

        # Pre-start configuration values
        self.dut.integ_thresh_avg_oob.value = 0
        self.dut.integ_window_oob.value = 0
        self.dut.integ_en_oob.value = 0
        self.dut.sys_en_oob.value = 0
        self.dut.lock_viol.value = 0

        # Shutdown sense (8-bit per board)
        self.dut.shutdown_sense.value = 0x00

        # Integrator (8-bit per board)
        self.dut.over_thresh.value = 0x00
        self.dut.thresh_underflow.value = 0x00
        self.dut.thresh_overflow.value = 0x00

        # Trigger buffer and commands
        self.dut.bad_trig_cmd.value = 0
        self.dut.trig_cmd_buf_overflow.value = 0
        self.dut.trig_data_buf_underflow.value = 0
        self.dut.trig_data_buf_overflow.value = 0

        # DAC buffers and commands (8-bit per board)
        self.dut.dac_boot_fail.value = 0x00
        self.dut.bad_dac_cmd.value = 0x00
        self.dut.dac_cal_oob.value = 0x00
        self.dut.dac_val_oob.value = 0x00
        self.dut.dac_cmd_buf_underflow.value = 0x00
        self.dut.dac_cmd_buf_overflow.value = 0x00
        self.dut.unexp_dac_trig.value = 0x00

        # ADC buffers and commands (8-bit per board)
        self.dut.adc_boot_fail.value = 0x00
        self.dut.bad_adc_cmd.value = 0x00
        self.dut.adc_cmd_buf_underflow.value = 0x00
        self.dut.adc_cmd_buf_overflow.value = 0x00
        self.dut.adc_data_buf_underflow.value = 0x00
        self.dut.adc_data_buf_overflow.value = 0x00
        self.dut.unexp_adc_trig.value = 0x00

    def get_state_name(self, state_value):
        """Convert state value to state name"""
        # Convert BinaryValue to int
        state_int = int(state_value)
        return self.STATES.get(state_int, f"UNKNOWN_STATE({state_int})")

    def get_state_value(self, state_name):
        """Convert state name to state value"""
        for value, name in self.STATES.items():
            if name == state_name:
                return value
        raise ValueError(f"UNKNOWN_STATE_NAME: {state_name}")

    def get_status_name(self, status_value):
        """Convert status code value to status name"""
        # Convert BinaryValue to int
        status_int = int(status_value)
        return self.STATUS_CODES.get(status_int, f"UNKNOWN_STATUS({status_int})")

    def get_status_value(self, status_name):
        """Convert status name to status code value"""
        for value, name in self.STATUS_CODES.items():
            if name == status_name:
                return value
        raise ValueError(f"UNKNOWN_STATUS_NAME: {status_name}")
    
    def get_board_num_from_status_word(self, status_word):
        """Extract board number from status word"""
        # Convert BinaryValue to int
        status_word_int = int(status_word)
        return (status_word_int >> 29) & 0x7
    
    def extract_state_and_status(self):
        """Extract and decode state and status from the DUT"""
        state_val = int(self.dut.state.value)  # Convert to int
        status_word = int(self.dut.status_word.value)  # Convert to int
        status_code = (status_word >> 4) & 0x1FFFFFF
        board_num = (status_word >> 29) & 0x7
        
        state_name = self.get_state_name(state_val)
        status_name = self.get_status_name(status_code)
        
        return {
            "state_value": state_val,
            "state_name": state_name,
            "status_code": status_code,
            "status_name": status_name,
            "board_num": board_num,
            "status_word": status_word
        }
    
    def print_current_status(self):
        """Print the current status information in a human-readable format"""
        status_info = self.extract_state_and_status()
        time = cocotb.utils.get_sim_time(units=self.time_unit)
        
        self.dut._log.info(f"------------ CURRENT STATUS AT TIME = {time}  ------------")
        self.dut._log.info(f"State: {status_info['state_name']} ({status_info['state_value']})")
        self.dut._log.info(f"Status: {status_info['status_name']} ({status_info['status_code']})")
        if status_info['board_num'] > 0:
            self.dut._log.info(f"Board: {status_info['board_num']}")
        self.dut._log.info(f"------------ OUTPUT SIGNALS AT TIME = {time}  ------------")
        self.dut._log.info(f"  unlock_cfg: {self.dut.unlock_cfg.value}")
        self.dut._log.info(f"  spi_clk_gate: {self.dut.spi_clk_gate.value}")
        self.dut._log.info(f"  spi_en: {self.dut.spi_en.value}")
        self.dut._log.info(f"  shutdown_sense_en: {self.dut.shutdown_sense_en.value}")
        self.dut._log.info(f"  block_bufs: {self.dut.block_bufs.value}")
        self.dut._log.info(f"  n_shutdown_force: {self.dut.n_shutdown_force.value}")
        self.dut._log.info(f"  shutdown_rst: {self.dut.shutdown_rst.value}")
        self.dut._log.info(f"  ps_interrupt: {self.dut.ps_interrupt.value}")
        self.dut._log.info("---------------------------------------------")

    async def reset(self):
        """Reset the DUT using active low reset"""
        await RisingEdge(self.dut.clk)  # Wait for clock to stabilize
        self.dut.aresetn.value = 0  # Assert active low reset
        await RisingEdge(self.dut.clk)  # Hold reset for one clock
        await RisingEdge(self.dut.clk)  # Hold reset for a second clock
        self.dut.aresetn.value = 1  # Deassert reset
        await RisingEdge(self.dut.clk)  # Wait for reset deassertion to propagate
        self.dut._log.info("RESET COMPLETE")

    async def check_state_and_status(self, expected_state, expected_status_code, expected_board_num=0):
        """Check the status outputs and the state of the hardware manager.
        Only log detailed information if there's a mismatch."""
        status_info = self.extract_state_and_status()
        state = status_info["state_value"]
        status_code = status_info["status_code"]
        board_num = status_info["board_num"]

        # Compare values first
        mismatch = (
            state != expected_state
            or status_code != expected_status_code
            or board_num != expected_board_num
        )

        if mismatch:
            time = cocotb.utils.get_sim_time(units=self.time_unit)
            self.dut._log.info(f"------------STATUS CHECK FAILED AT TIME = {time} ------------")
            self.dut._log.info(f"Expected State: {self.get_state_name(expected_state)} ({expected_state})")
            self.dut._log.info(f"Expected Status: {self.get_status_name(expected_status_code)} ({expected_status_code})")
            if expected_board_num > 0:
                self.dut._log.info(f"Expected Board: {expected_board_num}")
            # Log current status
            self.print_current_status()

            # Now raise the assertion with detailed message
            if state != expected_state:
                raise AssertionError(
                    f"Expected state {self.get_state_name(expected_state)} ({expected_state}), "
                    f"got {self.get_state_name(state)} ({state})"
                )
            if status_code != expected_status_code:
                raise AssertionError(
                    f"Expected status code {self.get_status_name(expected_status_code)} ({expected_status_code}), "
                    f"got {self.get_status_name(status_code)} ({status_code})"
                )
            if board_num != expected_board_num:
                raise AssertionError(
                    f"Expected board number {expected_board_num}, got {board_num}"
                )

        return state, status_code, board_num

    
    async def check_state(self, expected_state):
        """Check the state of the hardware manager"""
        status_info = self.extract_state_and_status()
        state = status_info["state_value"]
        
        # Pass the test if the state matches the expected value
        assert state == expected_state, f"Expected state {self.get_state_name(expected_state)}({expected_state}), " \
            f"got {self.get_state_name(state)}({state})"
        
        return state
    
    async def wait_cycles(self, cycles):
        """Wait for specified number of clock cycles"""
        for _ in range(cycles):
            await RisingEdge(self.dut.clk)

    async def wait_for_state(self, expected_state, timeout_ns=1000000, allow_intermediate_states=False, max_wait_cycles=None):
        """Wait until the state machine reaches a specific state

        Args:
        expected_state: The state value to wait for
        timeout_ns: Maximum time to wait in nanoseconds before failing
        allow_intermediate_states: If True, allows the state to change to other states before reaching expected_state
        max_wait_cycles: Optional maximum number of cycles to wait, overrides timeout_ns if specified
        """
        expected_state_name = self.get_state_name(expected_state)
        self.dut._log.info(f"System should reach state: {expected_state_name}({expected_state})")

        # Calculate max cycles based on either max_wait_cycles or timeout_ns
        max_cycles = max_wait_cycles if max_wait_cycles is not None else (timeout_ns // 4)

        # Track the current state for logging purposes
        last_state = None
        last_state_name = None
        inital_state = self.dut.state.value

        for i in range(max_cycles):
            
            await ReadOnly()  # Ensure all signals are updated before checking state
            current_state = self.dut.state.value

            # Log state transitions for debugging
            if current_state != last_state and last_state is not None:
                last_state = current_state
                last_state_name = self.get_state_name(current_state)
                self.dut._log.info(f"State changed to {last_state_name}({int(current_state)}) at cycle {i}")

            if current_state == expected_state:
                self.dut._log.info(f"Reached state {expected_state_name}({expected_state}) after {i} cycles")
                return

            # If not allowing intermediate states, fail if state changes to something other than expected
            if not allow_intermediate_states and i > 0 and current_state != inital_state:
                assert current_state == expected_state, \
                    f"Reached unexpected state {last_state_name}({current_state}) while waiting for {expected_state_name}({expected_state})"
                
            await RisingEdge(self.dut.clk)
            
        # If we reach here, we timed out waiting for the expected state
        self.dut._log.info(f"BEFORE FAILURE:")
        self.print_current_status()  # Print final status before failure
        assert False, f"Timeout waiting for state {expected_state_name}({expected_state}) after {max_cycles} cycles"

