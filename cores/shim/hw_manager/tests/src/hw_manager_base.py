import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, ReadWrite, Combine, with_timeout
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
        5: "S_WAIT_FOR_POW_EN",
        6: "S_POWER_ON_AMP_BRD",
        7: "S_AMP_POWER_WAIT",
        8: "S_RUNNING",
        9: "S_HALTING",
        10: "S_HALTED"
    }
    
    # Status codes dictionary
    STATUS_CODES = {
        0x0000: "STS_EMPTY",
        0x0001: "STS_OK",
        0x0002: "STS_PS_SHUTDOWN",
        0x0100: "STS_SPI_RESET_TIMEOUT",
        0x0101: "STS_SPI_START_TIMEOUT",
        0x0200: "STS_LOCK_VIOL",
        0x0201: "STS_CTRL_EN_OOB",
        0x0202: "STS_POW_EN_OOB",
        0x0203: "STS_CMD_BUF_RESET_OOB",
        0x0204: "STS_DATA_BUF_RESET_OOB",
        0x0205: "STS_INTEG_THRESH_AVG_OOB",
        0x0206: "STS_INTEG_WINDOW_OOB",
        0x0207: "STS_INTEG_EN_OOB",
        0x0208: "STS_BOOT_TEST_SKIP_OOB",
        0x0209: "STS_DEBUG_OOB",
        0x020A: "STS_MOSI_SCK_POL_OOB",
        0x020B: "STS_MISO_SCK_POL_OOB",
        0x020C: "STS_DAC_CAL_INIT_OOB",
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
        0x0604: "STS_DAC_CMD_BUF_UNDERFLOW",
        0x0605: "STS_DAC_CMD_BUF_OVERFLOW",
        0x0606: "STS_DAC_DATA_BUF_UNDERFLOW",
        0x0607: "STS_DAC_DATA_BUF_OVERFLOW",
        0x0608: "STS_UNEXP_DAC_TRIG",
        0x0609: "STS_LDAC_MISALIGN",
        0x060A: "STS_DAC_DELAY_TOO_SHORT",
        0x0700: "STS_ADC_BOOT_FAIL",
        0x0701: "STS_BAD_ADC_CMD",
        0x0702: "STS_ADC_CMD_BUF_UNDERFLOW",
        0x0703: "STS_ADC_CMD_BUF_OVERFLOW",
        0x0704: "STS_ADC_DATA_BUF_UNDERFLOW",
        0x0705: "STS_ADC_DATA_BUF_OVERFLOW",
        0x0706: "STS_UNEXP_ADC_TRIG",
        0x0707: "STS_ADC_DELAY_TOO_SHORT",
    }

    def __init__(self, dut, clk_period = 4, time_unit = "ns"):
        self.time_unit = time_unit
        self.clk_period = clk_period
        self.dut = dut
        
        # Get parameters from the DUT
        self.SHUTDOWN_FORCE_DELAY = int(self.dut.SHUTDOWN_FORCE_DELAY.value)
        self.SHUTDOWN_RESET_PULSE = int(self.dut.SHUTDOWN_RESET_PULSE.value)
        self.SHUTDOWN_RESET_DELAY = int(self.dut.SHUTDOWN_RESET_DELAY.value)
        self.SPI_RESET_WAIT =       int(self.dut.SPI_RESET_WAIT.value)
        self.SPI_START_WAIT =       int(self.dut.SPI_START_WAIT.value)

        # Log the parameters
        self.dut._log.info(f"Parameters:")
        self.dut._log.info(f"  SHUTDOWN_FORCE_DELAY: {self.SHUTDOWN_FORCE_DELAY}")
        self.dut._log.info(f"  SHUTDOWN_RESET_PULSE: {self.SHUTDOWN_RESET_PULSE}")
        self.dut._log.info(f"  SHUTDOWN_RESET_DELAY: {self.SHUTDOWN_RESET_DELAY}")
        self.dut._log.info(f"  SPI_RESET_WAIT: {self.SPI_RESET_WAIT}")
        self.dut._log.info(f"  SPI_START_WAIT: {self.SPI_START_WAIT}")

        # Set timeout time, should be bigger than max(parameters) * clk_period
        # since we are allowed to wait that much.
        self.timeout_time = max([
            self.SHUTDOWN_FORCE_DELAY,
            self.SHUTDOWN_RESET_PULSE,
            self.SHUTDOWN_RESET_DELAY,
            self.SPI_RESET_WAIT,
            self.SPI_START_WAIT
        ]) * self.clk_period + clk_period * 10

        # Create clock
        cocotb.start_soon(Clock(self.dut.clk, clk_period, units=time_unit).start(start_high=False))  # Default is 250 MHz clock
        
        # Set default values for all inputs
        self.dut.ctrl_en.value = 0
        self.dut.pow_en.value = 0
        self.dut.spi_off.value = 1  # SPI starts powered off
        self.dut.calc_n_cs_done.value = 0
        self.dut.ext_en.value = 0

        # Pre-start configuration values
        self.dut.lock_viol.value = 0
        self.dut.ctrl_en_oob.value = 0
        self.dut.pow_en_oob.value = 0
        self.dut.cmd_buf_reset_oob.value = 0
        self.dut.data_buf_reset_oob.value = 0
        self.dut.integ_thresh_avg_oob.value = 0
        self.dut.integ_window_oob.value = 0
        self.dut.integ_en_oob.value = 0
        self.dut.boot_test_skip_oob.value = 0
        self.dut.debug_oob.value = 0
        self.dut.mosi_sck_pol_oob.value = 0
        self.dut.miso_sck_pol_oob.value = 0
        self.dut.dac_cal_init_oob.value = 0

        # Shutdown sense (8-bit per board)
        self.dut.shutdown_sense.value = 0

        # Integrator (8-bit per board)
        self.dut.over_thresh.value = 0
        self.dut.thresh_underflow.value = 0
        self.dut.thresh_overflow.value = 0

        # Trigger buffer and commands
        self.dut.bad_trig_cmd.value = 0
        self.dut.trig_cmd_buf_overflow.value = 0
        self.dut.trig_data_buf_underflow.value = 0
        self.dut.trig_data_buf_overflow.value = 0

        # DAC buffers and commands (8-bit per board)
        self.dut.dac_boot_fail.value = 0
        self.dut.bad_dac_cmd.value = 0
        self.dut.dac_cal_oob.value = 0
        self.dut.dac_val_oob.value = 0
        self.dut.dac_cmd_buf_underflow.value = 0
        self.dut.dac_cmd_buf_overflow.value = 0
        self.dut.dac_data_buf_underflow.value = 0
        self.dut.dac_data_buf_overflow.value = 0
        self.dut.unexp_dac_trig.value = 0
        self.dut.ldac_misalign.value = 0
        self.dut.dac_delay_too_short.value = 0

        # ADC buffers and commands (8-bit per board)
        self.dut.adc_boot_fail.value = 0
        self.dut.bad_adc_cmd.value = 0
        self.dut.adc_cmd_buf_underflow.value = 0
        self.dut.adc_cmd_buf_overflow.value = 0
        self.dut.adc_data_buf_underflow.value = 0
        self.dut.adc_data_buf_overflow.value = 0
        self.dut.unexp_adc_trig.value = 0
        self.dut.adc_delay_too_short.value = 0

    def get_state_name(self, state_value):
        state_int = int(state_value)
        return self.STATES.get(state_int, f"UNKNOWN_STATE({state_int})")

    def get_state_value(self, state_name):
        for value, name in self.STATES.items():
            if name == state_name:
                return value
        raise ValueError(f"UNKNOWN_STATE_NAME: {state_name}")

    def get_status_name(self, status_value):
        status_int = int(status_value)
        return self.STATUS_CODES.get(status_int, f"UNKNOWN_STATUS({status_int})")

    def get_status_value(self, status_name):
        for value, name in self.STATUS_CODES.items():
            if name == status_name:
                return value
        raise ValueError(f"UNKNOWN_STATUS_NAME: {status_name}")
    
    def get_board_num_from_status_word(self, status_word):
        status_word_int = int(status_word)
        return (status_word_int >> 29) & 0x7
    
    def extract_board_num(self, signal):
        signal_int = int(signal) & 0xFF
        for i in range(8):
            if signal_int & (1 << i):
                return i
        return 0
    
    def extract_state_and_status(self):
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
        """Reset the DUT for two clock cycles"""
        await RisingEdge(self.dut.clk)
        self.dut.aresetn.value = 0
        self.dut._log.info("STARTING RESET")
        await RisingEdge(self.dut.clk)
        await RisingEdge(self.dut.clk)
        self.dut.aresetn.value = 1
        self.dut._log.info("RESET COMPLETE")

    async def check_state_and_status(self, expected_state, expected_status_code, expected_board_num=0):
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
                self.dut._log.info(f"Expected Board: {expected_board_num}")\

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
        max_cycles = max_wait_cycles if max_wait_cycles is not None else (timeout_ns // self.clk_period)

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

    # =========================================================================
    # Scoreboard dispatcher and scoreboards for each state
    # =========================================================================

    async def state_scoreboard_dispatcher(self):
        """
        Dispatches a scoreboard for each state if the previous state is not the same as the current state to test execution of each state.
        Each scoreboard also monitors the relevant input wires of the DUT. 
        """
        forked_tasks = []
        prev_state = None
        curr_state = None
        idle_forked = False
        running_forked = False
        halted_forked = False
        timed_out = False

        # Since state transitions depend on driven input signals,
        # this is a safety feature to break the infinite while loop.
        async def sb_with_timeout(coro, timeout_time: int, name: str):
            nonlocal timed_out
            try:
                await with_timeout(coro, timeout_time, self.time_unit)
            except cocotb.result.SimTimeoutError:
                timed_out = True
                self.dut._log.info(f"Scoreboard {name} timed out after {timeout_time} {self.time_unit}")

        while True:
            if running_forked or halted_forked or timed_out: # Stop after running or halted scoreboards have been forked or we timeout.
                break

            await RisingEdge(self.dut.clk)
            prev_state = self.dut.state.value.integer
            await ReadOnly()
            curr_state = self.dut.state.value.integer

            task = None
            if prev_state != curr_state or not idle_forked:
                if curr_state == self.get_state_value("S_IDLE"):
                    task = cocotb.start_soon(sb_with_timeout(self.idle_scoreboard(), self.timeout_time, "S_IDLE"))
                    idle_forked = True
                    self.dut._log.info("Forked IDLE scoreboard")

                elif curr_state == self.get_state_value("S_CONFIRM_SPI_RST"):
                    task = cocotb.start_soon(sb_with_timeout(self.confirm_spi_rst_scoreboard(), self.timeout_time, "S_CONFIRM_SPI_RST"))
                    self.dut._log.info("Forked CONFIRM_SPI_RST scoreboard")

                elif curr_state == self.get_state_value("S_POWER_ON_CRTL_BRD"):
                    task = cocotb.start_soon(sb_with_timeout(self.power_on_crtl_brd_scoreboard(), self.timeout_time, "S_POWER_ON_CRTL_BRD"))
                    self.dut._log.info("Forked POWER_ON_CRTL_BRD scoreboard")

                elif curr_state == self.get_state_value("S_CONFIRM_SPI_START"):
                    task = cocotb.start_soon(sb_with_timeout(self.confirm_spi_start_scoreboard(), self.timeout_time, "S_CONFIRM_SPI_START"))
                    self.dut._log.info("Forked CONFIRM_SPI_START scoreboard")

                elif curr_state == self.get_state_value("S_WAIT_FOR_POW_EN"):
                    task = cocotb.start_soon(sb_with_timeout(self.wait_for_pow_en_scoreboard(), self.timeout_time, "S_WAIT_FOR_POW_EN"))
                    self.dut._log.info("Forked WAIT_FOR_POW_EN scoreboard")

                elif curr_state == self.get_state_value("S_POWER_ON_AMP_BRD"):
                    task = cocotb.start_soon(sb_with_timeout(self.power_on_amp_brd_scoreboard(), self.timeout_time, "S_POWER_ON_AMP_BRD"))
                    self.dut._log.info("Forked POWER_ON_AMP_BRD scoreboard")

                elif curr_state == self.get_state_value("S_AMP_POWER_WAIT"):
                    task = cocotb.start_soon(sb_with_timeout(self.amp_power_wait_scoreboard(), self.timeout_time, "S_AMP_POWER_WAIT"))
                    self.dut._log.info("Forked AMP_POWER_WAIT scoreboard")

                elif curr_state == self.get_state_value("S_RUNNING"):
                    task = cocotb.start_soon(sb_with_timeout(self.running_scoreboard(), self.timeout_time, "S_RUNNING"))
                    running_forked = True
                    self.dut._log.info("Forked RUNNING scoreboard")

                elif curr_state == self.get_state_value("S_HALTING"):
                    task = cocotb.start_soon(sb_with_timeout(self.halting_scoreboard(), self.timeout_time, "S_HALTING"))
                    self.dut._log.info("Forked HALTING scoreboard")

                elif curr_state == self.get_state_value("S_HALTED"):
                    task = cocotb.start_soon(sb_with_timeout(self.halted_scoreboard(), self.timeout_time, "S_HALTED"))
                    halted_forked = True
                    self.dut._log.info("Forked HALTED scoreboard")

                # Default to S_HALTED
                else:
                    task = cocotb.start_soon(sb_with_timeout(self.halted_scoreboard(), self.timeout_time, "HALTED"))
                    halted_forked = True
                    self.dut._log.info(f"Reached unknown state {curr_state}, defaulting to HALTED scoreboard")
                
            if task is not None:
                forked_tasks.append(task)

        if forked_tasks:
            await Combine(*forked_tasks)

    async def idle_scoreboard(self): # HAS INDEFINITE WAIT
        # State should be S_IDLE
        await self.check_state_and_status(
            expected_state=self.get_state_value("S_IDLE"),
            expected_status_code=self.get_status_value("STS_OK"),
            expected_board_num=0
        )
        assert self.dut.timer.value.integer == 0, \
        f"Expected timer to be 0 in S_IDLE, got {self.dut.timer.value.integer}"
        assert self.dut.n_shutdown_force.value.integer == 0, \
        f"Expected n_shutdown_force to be 0 in S_IDLE, got {self.dut.n_shutdown_force.value.integer}"
        assert self.dut.shutdown_rst.value.integer == 0, \
        f"Expected shutdown_rst to be 0 in S_IDLE, got {self.dut.shutdown_rst.value.integer}"
        assert self.dut.shutdown_sense_en.value.integer == 0, \
        f"Expected shutdown_sense_en to be 0 in S_IDLE, got {self.dut.shutdown_sense_en.value.integer}"
        assert self.dut.unlock_cfg.value.integer == 1, \
        f"Expected unlock_cfg to be 1 in S_IDLE, got {self.dut.unlock_cfg.value.integer}"
        assert self.dut.spi_clk_gate.value.integer == 0, \
        f"Expected spi_clk_gate to be 0 in S_IDLE, got {self.dut.spi_clk_gate.value.integer}"
        assert self.dut.spi_en.value.integer == 0, \
        f"Expected spi_en to be 0 in S_IDLE, got {self.dut.spi_en.value.integer}"
        assert self.dut.block_bufs.value.integer == 1, \
        f"Expected block_bufs to be 1 in S_IDLE, got {self.dut.block_bufs.value.integer}"
        assert self.dut.ps_interrupt.value.integer == 0, \
        f"Expected ps_interrupt to be 0 in S_IDLE, got {self.dut.ps_interrupt.value.integer}"

        while True:
            await RisingEdge(self.dut.clk)
            # Following are input wires of the DUT this state depends on.
            prev_ctrl_en = self.dut.ctrl_en.value.integer
            prev_ext_en = self.dut.ext_en.value.integer
            prev_ctrl_en_oob = self.dut.ctrl_en_oob.value.integer
            prev_pow_en_oob = self.dut.pow_en_oob.value.integer
            prev_cmd_buf_reset_oob = self.dut.cmd_buf_reset_oob.value.integer
            prev_data_buf_reset_oob = self.dut.data_buf_reset_oob.value.integer
            prev_integ_thresh_avg_oob = self.dut.integ_thresh_avg_oob.value.integer
            prev_integ_window_oob = self.dut.integ_window_oob.value.integer
            prev_integ_en_oob = self.dut.integ_en_oob.value.integer
            prev_boot_test_skip_oob= self.dut.boot_test_skip_oob.value.integer
            prev_debug_oob= self.dut.debug_oob.value.integer
            prev_mosi_sck_pol_oob= self.dut.mosi_sck_pol_oob.value.integer
            prev_miso_sck_pol_oob= self.dut.miso_sck_pol_oob.value.integer
            prev_dac_cal_init_oob= self.dut.dac_cal_init_oob.value.integer
            await ReadOnly()

            if(prev_ctrl_en):
                if(prev_ext_en == 0):
                    await self.check_state_and_status(
                    expected_state=self.get_state_value("S_HALTING"),
                    expected_status_code=self.get_status_value("STS_EXT_SHUTDOWN")
                    )
                    break
                elif(
                    prev_ctrl_en_oob
                    or prev_pow_en_oob
                    or prev_cmd_buf_reset_oob
                    or prev_data_buf_reset_oob
                    or prev_integ_thresh_avg_oob
                    or prev_integ_window_oob
                    or prev_integ_en_oob
                    or prev_boot_test_skip_oob
                    or prev_debug_oob
                    or prev_mosi_sck_pol_oob
                    or prev_miso_sck_pol_oob
                    or prev_dac_cal_init_oob
                ):
                    if(prev_ctrl_en_oob):
                        await self.check_state_and_status(
                            expected_state=self.get_state_value("S_HALTING"),
                            expected_status_code=self.get_status_value("STS_CTRL_EN_OOB")
                        )
                        break
                    elif(prev_pow_en_oob):
                        await self.check_state_and_status(
                            expected_state=self.get_state_value("S_HALTING"),
                            expected_status_code=self.get_status_value("STS_POW_EN_OOB")
                        )
                        break
                    elif(prev_cmd_buf_reset_oob):
                        await self.check_state_and_status(
                            expected_state=self.get_state_value("S_HALTING"),
                            expected_status_code=self.get_status_value("STS_CMD_BUF_RESET_OOB")
                        )
                        break
                    elif(prev_data_buf_reset_oob):
                        await self.check_state_and_status(
                            expected_state=self.get_state_value("S_HALTING"),
                            expected_status_code=self.get_status_value("STS_DATA_BUF_RESET_OOB")
                        )
                        break
                    elif(prev_integ_thresh_avg_oob):
                        await self.check_state_and_status(
                            expected_state=self.get_state_value("S_HALTING"),
                            expected_status_code=self.get_status_value("STS_INTEG_THRESH_AVG_OOB")
                        )
                        break
                    elif(prev_integ_window_oob):
                        await self.check_state_and_status(
                            expected_state=self.get_state_value("S_HALTING"),
                            expected_status_code=self.get_status_value("STS_INTEG_WINDOW_OOB")
                        )
                        break
                    elif(prev_integ_en_oob):
                        await self.check_state_and_status(
                            expected_state=self.get_state_value("S_HALTING"),
                            expected_status_code=self.get_status_value("STS_INTEG_EN_OOB")
                        )
                        break
                    elif(prev_boot_test_skip_oob):
                        await self.check_state_and_status(
                            expected_state=self.get_state_value("S_HALTING"),
                            expected_status_code=self.get_status_value("STS_BOOT_TEST_SKIP_OOB")
                        )
                        break
                    elif(prev_debug_oob):
                        await self.check_state_and_status(
                            expected_state=self.get_state_value("S_HALTING"),
                            expected_status_code=self.get_status_value("STS_DEBUG_OOB")
                        )
                        break
                    elif(prev_mosi_sck_pol_oob):
                        await self.check_state_and_status(
                            expected_state=self.get_state_value("S_HALTING"),
                            expected_status_code=self.get_status_value("STS_MOSI_SCK_POL_OOB")
                        )
                        break
                    elif(prev_miso_sck_pol_oob):
                        await self.check_state_and_status(
                            expected_state=self.get_state_value("S_HALTING"),
                            expected_status_code=self.get_status_value("STS_MISO_SCK_POL_OOB")
                        )
                        break
                    elif(prev_dac_cal_init_oob):
                        await self.check_state_and_status(
                            expected_state=self.get_state_value("S_HALTING"),
                            expected_status_code=self.get_status_value("STS_DAC_CAL_INIT_OOB")
                        )
                        break
                else:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_CONFIRM_SPI_RST"),
                        expected_status_code=self.get_status_value("STS_OK")
                    )
                    assert self.dut.timer.value.integer == 0, \
                    f"Expected timer to be 0 in S_CONFIRM_SPI_RST, got {self.dut.timer.value.integer}"
                    assert self.dut.unlock_cfg.value.integer == 0, \
                    f"Expected unlock_cfg to be 0 in S_CONFIRM_SPI_RST, got {self.dut.unlock_cfg.value.integer}"
                    break
        return

    async def confirm_spi_rst_scoreboard(self):
        # State should be S_CONFIRM_SPI_RST
        await self.check_state(self.get_state_value("S_CONFIRM_SPI_RST"))

        # Initially timer should be 0.
        expected_timer = 0
        assert self.dut.timer.value.integer == expected_timer, \
        f"Expected timer to be 0 in S_CONFIRM_SPI_RST, got {self.dut.timer.value.integer}"

        # Initially unlock_cfg should be 0.
        assert self.dut.unlock_cfg.value.integer == 0, \
        f"Expected unlock_cfg to be 0 in S_CONFIRM_SPI_RST, got {self.dut.unlock_cfg.value.integer}"

        while True: 
            await RisingEdge(self.dut.clk)
            # Following are input wires of the DUT this state depends on.
            prev_ext_en = self.dut.ext_en.value.integer
            prev_ctrl_en = self.dut.ctrl_en.value.integer
            prev_spi_off = self.dut.spi_off.value.integer
            prev_calc_n_cs_done = self.dut.calc_n_cs_done.value.integer
            await ReadOnly()
            #curr_ext_en = self.dut.ext_en.value.integer
            #curr_ctrl_en = self.dut.ctrl_en.value.integer
            #curr_spi_off = self.dut.spi_off.value.integer
            #curr_calc_n_cs_done = self.dut.calc_n_cs_done.value.integer

            if(prev_ext_en == 0):
                await self.check_state_and_status(
                    expected_state=self.get_state_value("S_HALTING"),
                    expected_status_code=self.get_status_value("STS_EXT_SHUTDOWN")
                )
                break
            elif(prev_ctrl_en == 0):
                await self.check_state_and_status(
                    expected_state=self.get_state_value("S_HALTING"),
                    expected_status_code=self.get_status_value("STS_PS_SHUTDOWN")
                )
                break
            elif(expected_timer >= 10 and prev_spi_off == 1 and prev_calc_n_cs_done == 1):
                await self.check_state_and_status(
                    expected_state=self.get_state_value("S_POWER_ON_CRTL_BRD"),
                    expected_status_code=self.get_status_value("STS_OK")
                )
                expected_timer = 0  # Timer should reset when transitioning to next state
                assert self.dut.timer.value.integer == expected_timer, \
                f"Expected timer to reset to 0 when transitioning to S_POWER_ON_CRTL_BRD, got {self.dut.timer.value.integer}"
                assert self.dut.n_shutdown_force.value.integer == 1, \
                f"Expected n_shutdown_force to be 1 in S_POWER_ON_CRTL_BRD, got {self.dut.n_shutdown_force.value.integer}"
                break
            elif(expected_timer >= self.SPI_RESET_WAIT):
                await self.check_state_and_status(
                    expected_state=self.get_state_value("S_HALTING"),
                    expected_status_code=self.get_status_value("STS_SPI_RESET_TIMEOUT")
                )
                break
            else:
                expected_timer += 1
                assert self.dut.timer.value.integer == expected_timer, \
                f"Expected timer to be {expected_timer} in S_CONFIRM_SPI_RST, got {self.dut.timer.value.integer}"
        return

    async def power_on_crtl_brd_scoreboard(self):
        # State should be S_POWER_ON_CRTL_BRD
        await self.check_state(self.get_state_value("S_POWER_ON_CRTL_BRD"))

        # Initially timer should be 0.
        expected_timer = 0
        assert self.dut.timer.value.integer == expected_timer, \
        f"Expected timer to be 0 in S_POWER_ON_CRTL_BRD, got {self.dut.timer.value.integer}"

        # Initially n_shutdown_force should be 1.
        assert self.dut.n_shutdown_force.value.integer == 1, \
        f"Expected n_shutdown_force to be 1 in S_POWER_ON_CRTL_BRD, got {self.dut.n_shutdown_force.value.integer}"

        while True:
            await RisingEdge(self.dut.clk)
            # Following are input wires of the DUT this state depends on.
            prev_ext_en = self.dut.ext_en.value.integer
            prev_ctrl_en = self.dut.ctrl_en.value.integer
            await ReadOnly()

            if(prev_ext_en == 0):
                await self.check_state_and_status(
                    expected_state=self.get_state_value("S_HALTING"),
                    expected_status_code=self.get_status_value("STS_EXT_SHUTDOWN")
                )
                break
            elif(prev_ctrl_en == 0):
                await self.check_state_and_status(
                    expected_state=self.get_state_value("S_HALTING"),
                    expected_status_code=self.get_status_value("STS_PS_SHUTDOWN")
                )
                break
            elif(expected_timer >= self.SHUTDOWN_RESET_DELAY):
                await self.check_state_and_status(
                    expected_state=self.get_state_value("S_CONFIRM_SPI_START"),
                    expected_status_code=self.get_status_value("STS_OK")
                )
                expected_timer = 0  # Timer should reset when transitioning to next state
                assert self.dut.timer.value.integer == expected_timer, \
                f"Expected timer to reset to 0 when transitioning to S_CONFIRM_SPI_START, got {self.dut.timer.value.integer}"
                assert self.dut.spi_en.value.integer == 1, \
                f"Expected spi_en to be 1 in S_CONFIRM_SPI_START, got {self.dut.spi_en.value.integer}"
                assert self.dut.spi_clk_gate.value.integer == 1, \
                f"Expected spi_clk_gate to be 1 in S_CONFIRM_SPI_START, got {self.dut.spi_clk_gate.value.integer}"
                break
            else:
                expected_timer += 1
                assert self.dut.timer.value.integer == expected_timer, \
                f"Expected timer to be {expected_timer} in S_POWER_ON_CRTL_BRD, got {self.dut.timer.value.integer}"
        return

    async def confirm_spi_start_scoreboard(self):
        # State should be S_CONFIRM_SPI_START
        await self.check_state(self.get_state_value("S_CONFIRM_SPI_START"))

        # Initially timer should be 0.
        expected_timer = 0
        assert self.dut.timer.value.integer == expected_timer, \
        f"Expected timer to be 0 in S_CONFIRM_SPI_START, got {self.dut.timer.value.integer}"

        # Initially spi_en should be 1.
        assert self.dut.spi_en.value.integer == 1, \
        f"Expected spi_en to be 1 in S_CONFIRM_SPI_START, got {self.dut.spi_en.value.integer}"

        # Initially spi_clk_gate should be 1.
        assert self.dut.spi_clk_gate.value.integer == 1, \
        f"Expected spi_clk_gate to be 1 in S_CONFIRM_SPI_START, got {self.dut.spi_clk_gate.value.integer}"

        while True:
            await RisingEdge(self.dut.clk)
            # Following are input wires of the DUT this state depends on.
            prev_ext_en = self.dut.ext_en.value.integer
            prev_ctrl_en = self.dut.ctrl_en.value.integer
            prev_spi_off = self.dut.spi_off.value.integer
            prev_dac_boot_fail = self.dut.dac_boot_fail.value.integer
            prev_adc_boot_fail = self.dut.adc_boot_fail.value.integer
            await ReadOnly()
            if(prev_ext_en == 0):
                await self.check_state_and_status(
                    expected_state=self.get_state_value("S_HALTING"),
                    expected_status_code=self.get_status_value("STS_EXT_SHUTDOWN")
                )
                break
            elif(prev_ctrl_en == 0):
                await self.check_state_and_status(
                    expected_state=self.get_state_value("S_HALTING"),
                    expected_status_code=self.get_status_value("STS_PS_SHUTDOWN")
                )
                break
            elif(prev_spi_off == 0):
                await self.check_state_and_status(
                    expected_state=self.get_state_value("S_WAIT_FOR_POW_EN"),
                    expected_status_code=self.get_status_value("STS_OK")
                )
                expected_timer = 0  # Timer should reset when transitioning to next state
                assert self.dut.timer.value.integer == expected_timer, \
                f"Expected timer to reset to 0 when transitioning to S_WAIT_FOR_POW_EN, got {self.dut.timer.value.integer}"
                assert self.dut.shutdown_rst.value.integer == 1, \
                f"Expected shutdown_rst to be 1 in S_WAIT_FOR_POW_EN, got {self.dut.shutdown_rst.value.integer}"
                break
            elif(prev_dac_boot_fail != 0 or prev_adc_boot_fail != 0 or expected_timer >= self.SPI_START_WAIT):
                expected_timer = 0  # Timer should reset when transitioning to halting state
                assert self.dut.timer.value.integer == expected_timer, \
                f"Expected timer to reset to 0 when transitioning to S_HALTING, got {self.dut.timer.value.integer}"
                if(prev_dac_boot_fail != 0):
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_DAC_BOOT_FAIL"),
                        expected_board_num=self.extract_board_num(self.dut.dac_boot_fail.value)
                    )
                elif(prev_adc_boot_fail != 0):
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_ADC_BOOT_FAIL"),
                        expected_board_num=self.extract_board_num(self.dut.adc_boot_fail.value)
                    )
                else:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_SPI_START_TIMEOUT")
                    )
                break
            else:
                expected_timer += 1
                assert self.dut.timer.value.integer == expected_timer, \
                f"Expected timer to be {expected_timer} in S_CONFIRM_SPI_START, got {self.dut.timer.value.integer}"
        return

    async def wait_for_pow_en_scoreboard(self): # HAS INDEFINITE WAIT
        # State should be S_WAIT_FOR_POW_EN
        await self.check_state(self.get_state_value("S_WAIT_FOR_POW_EN"))

        # Initially timer should be 0.
        expected_timer = 0
        assert self.dut.timer.value.integer == expected_timer, \
        f"Expected timer to be 0 in S_WAIT_FOR_POW_EN, got {self.dut.timer.value.integer}"

        # Initially shutdown_rst should be 1.
        assert self.dut.shutdown_rst.value.integer == 1, \
        f"Expected shutdown_rst to be 1 in S_WAIT_FOR_POW_EN, got {self.dut.shutdown_rst.value.integer}"

        while True:
            await RisingEdge(self.dut.clk)
            # Following are input wires of the DUT this state depends on.
            prev_ext_en = self.dut.ext_en.value.integer
            prev_ctrl_en = self.dut.ctrl_en.value.integer
            prev_pow_en = self.dut.pow_en.value.integer
            await ReadOnly()

            if(prev_ext_en == 0):
                await self.check_state_and_status(
                    expected_state=self.get_state_value("S_HALTING"),
                    expected_status_code=self.get_status_value("STS_EXT_SHUTDOWN")
                )
                break
            elif(prev_ctrl_en == 0):
                await self.check_state_and_status(
                    expected_state=self.get_state_value("S_HALTING"),
                    expected_status_code=self.get_status_value("STS_PS_SHUTDOWN")
                )
                break
            elif(prev_pow_en == 1):
                await self.check_state_and_status(
                    expected_state=self.get_state_value("S_POWER_ON_AMP_BRD"),
                    expected_status_code=self.get_status_value("STS_OK")
                )
                expected_timer = 0  # Timer should reset when transitioning to next state
                assert self.dut.timer.value.integer == expected_timer, \
                f"Expected timer to reset to 0 when transitioning to S_POWER_ON_AMP_BRD, got {self.dut.timer.value.integer}"
                break
        return

    async def power_on_amp_brd_scoreboard(self):
        # State should be S_POWER_ON_AMP_BRD
        await self.check_state(self.get_state_value("S_POWER_ON_AMP_BRD"))

        # Initially timer sohuld be 0.
        expected_timer = 0
        assert self.dut.timer.value.integer == expected_timer, \
        f"Expected timer to be 0 in S_POWER_ON_AMP_BRD, got {self.dut.timer.value.integer}"

        while True:
            await RisingEdge(self.dut.clk)
            # Following are input wires of the DUT this state depends on.
            prev_ext_en = self.dut.ext_en.value.integer
            prev_ctrl_en = self.dut.ctrl_en.value.integer
            prev_pow_en = self.dut.pow_en.value.integer
            await ReadOnly()

            if(prev_ext_en == 0):
                await self.check_state_and_status(
                    expected_state=self.get_state_value("S_HALTING"),
                    expected_status_code=self.get_status_value("STS_EXT_SHUTDOWN")
                )
                break
            elif(prev_ctrl_en == 0 or prev_pow_en == 0):
                await self.check_state_and_status(
                    expected_state=self.get_state_value("S_HALTING"),
                    expected_status_code=self.get_status_value("STS_PS_SHUTDOWN")
                )
                break
            elif(expected_timer >= self.SHUTDOWN_RESET_PULSE):
                await self.check_state_and_status(
                    expected_state=self.get_state_value("S_AMP_POWER_WAIT"),
                    expected_status_code=self.get_status_value("STS_OK")
                )
                expected_timer = 0  # Timer should reset when transitioning to next state
                assert self.dut.timer.value.integer == expected_timer, \
                f"Expected timer to reset to 0 when transitioning to S_AMP_POWER_WAIT, got {self.dut.timer.value.integer}"
                assert self.dut.shutdown_rst.value.integer == 0, \
                f"Expected shutdown_rst to be 0 in S_AMP_POWER_WAIT, got {self.dut.shutdown_rst.value.integer}"
                break
            else:
                expected_timer += 1
                assert self.dut.timer.value.integer == expected_timer, \
                f"Expected timer to be {expected_timer} in S_POWER_ON_AMP_BRD, got {self.dut.timer.value.integer}"
        return

    async def amp_power_wait_scoreboard(self):
        # State should be S_AMP_POWER_WAIT
        await self.check_state(self.get_state_value("S_AMP_POWER_WAIT"))

        # Initially timer should be 0.
        expected_timer = 0
        assert self.dut.timer.value.integer == expected_timer, \
        f"Expected timer to be 0 in S_AMP_POWER_WAIT, got {self.dut.timer.value.integer}"

        # Initially shutdown_rst should be 0.
        assert self.dut.shutdown_rst.value.integer == 0, \
        f"Expected shutdown_rst to be 0 in S_AMP_POWER_WAIT, got {self.dut.shutdown_rst.value.integer}"

        while True:
            await RisingEdge(self.dut.clk)
            # Following are input wires of the DUT this state depends on.
            prev_ext_en = self.dut.ext_en.value.integer
            prev_ctrl_en = self.dut.ctrl_en.value.integer
            prev_pow_en = self.dut.pow_en.value.integer
            await ReadOnly()

            if(prev_ext_en == 0):
                await self.check_state_and_status(
                    expected_state=self.get_state_value("S_HALTING"),
                    expected_status_code=self.get_status_value("STS_EXT_SHUTDOWN")
                )
                break
            elif(prev_ctrl_en == 0 or prev_pow_en == 0):
                await self.check_state_and_status(
                    expected_state=self.get_state_value("S_HALTING"),
                    expected_status_code=self.get_status_value("STS_PS_SHUTDOWN")
                )
                break
            elif(expected_timer >= self.SHUTDOWN_FORCE_DELAY):
                await self.check_state_and_status(
                    expected_state=self.get_state_value("S_RUNNING"),
                    expected_status_code=self.get_status_value("STS_OK")
                )
                expected_timer = 0  # Timer should reset when transitioning to next state
                assert self.dut.timer.value.integer == expected_timer, \
                f"Expected timer to reset to 0 when transitioning to S_RUNNING, got {self.dut.timer.value.integer}"
                assert self.dut.shutdown_sense_en.value.integer == 1, \
                f"Expected shutdown_sense_en to be 1 in S_RUNNING, got {self.dut.shutdown_sense_en.value.integer}"
                assert self.dut.block_bufs.value.integer == 0, \
                f"Expected block_bufs to be 0 in S_RUNNING, got {self.dut.block_bufs.value.integer}"
                assert self.dut.ps_interrupt.value.integer == 1, \
                f"Expected ps_interrupt to be 1 in S_RUNNING, got {self.dut.ps_interrupt.value.integer}"
                break
            else:
                expected_timer += 1
                assert self.dut.timer.value.integer == expected_timer, \
                f"Expected timer to be {expected_timer} in S_AMP_POWER_WAIT, got {self.dut.timer.value.integer}"
        return

    async def running_scoreboard(self): # HAS INDEFINITE WAIT
        # State should be S_RUNNING
        await self.check_state(self.get_state_value("S_RUNNING"))

        # Initially shutdown_sense_en should be 1.
        assert self.dut.shutdown_sense_en.value.integer == 1, \
        f"Expected shutdown_sense_en to be 1 in S_RUNNING, got {self.dut.shutdown_sense_en.value.integer}"

        # Initially block_bufs should be 0.
        assert self.dut.block_bufs.value.integer == 0, \
        f"Expected block_bufs to be 0 in S_RUNNING, got {self.dut.block_bufs.value.integer}"

        # Initially ps_interrupt should be 1 (set when entering S_RUNNING from S_AMP_POWER_WAIT).
        assert self.dut.ps_interrupt.value.integer == 1, \
        f"Expected ps_interrupt to be 1 when entering S_RUNNING, got {self.dut.ps_interrupt.value.integer}"

        while True:
            await RisingEdge(self.dut.clk)
            # Following are input wires of the DUT this state depends on.
            prev_ps_interrupt          = self.dut.ps_interrupt.value.integer
            prev_ctrl_en               = self.dut.ctrl_en.value.integer
            prev_pow_en                = self.dut.pow_en.value.integer
            prev_lock_viol             = self.dut.lock_viol.value.integer
            prev_mosi_sck_pol_oob      = self.dut.mosi_sck_pol_oob.value.integer
            prev_miso_sck_pol_oob      = self.dut.miso_sck_pol_oob.value.integer
            prev_shutdown_sense        = self.dut.shutdown_sense.value.integer
            prev_ext_en                = self.dut.ext_en.value.integer
            prev_over_thresh           = self.dut.over_thresh.value.integer
            prev_thresh_underflow      = self.dut.thresh_underflow.value.integer
            prev_thresh_overflow       = self.dut.thresh_overflow.value.integer
            prev_bad_trig_cmd          = self.dut.bad_trig_cmd.value.integer
            prev_trig_cmd_buf_overflow = self.dut.trig_cmd_buf_overflow.value.integer
            prev_trig_data_buf_underflow = self.dut.trig_data_buf_underflow.value.integer
            prev_trig_data_buf_overflow  = self.dut.trig_data_buf_overflow.value.integer
            prev_bad_dac_cmd           = self.dut.bad_dac_cmd.value.integer
            prev_dac_cal_oob           = self.dut.dac_cal_oob.value.integer
            prev_dac_val_oob           = self.dut.dac_val_oob.value.integer
            prev_dac_cmd_buf_underflow = self.dut.dac_cmd_buf_underflow.value.integer
            prev_dac_cmd_buf_overflow  = self.dut.dac_cmd_buf_overflow.value.integer
            prev_dac_data_buf_underflow = self.dut.dac_data_buf_underflow.value.integer
            prev_dac_data_buf_overflow  = self.dut.dac_data_buf_overflow.value.integer
            prev_unexp_dac_trig        = self.dut.unexp_dac_trig.value.integer
            prev_ldac_misalign         = self.dut.ldac_misalign.value.integer
            prev_dac_delay_too_short   = self.dut.dac_delay_too_short.value.integer
            prev_bad_adc_cmd           = self.dut.bad_adc_cmd.value.integer
            prev_adc_cmd_buf_underflow = self.dut.adc_cmd_buf_underflow.value.integer
            prev_adc_cmd_buf_overflow  = self.dut.adc_cmd_buf_overflow.value.integer
            prev_adc_data_buf_underflow = self.dut.adc_data_buf_underflow.value.integer
            prev_adc_data_buf_overflow  = self.dut.adc_data_buf_overflow.value.integer
            prev_unexp_adc_trig        = self.dut.unexp_adc_trig.value.integer
            prev_adc_delay_too_short   = self.dut.adc_delay_too_short.value.integer
            await ReadOnly()

            if prev_ps_interrupt:
                # Interrupt should be cleared before checking for errors
                assert self.dut.ps_interrupt.value.integer == 0, \
                f"Expected ps_interrupt to be cleared in S_RUNNING, got {self.dut.ps_interrupt.value.integer}"
            elif (
                not prev_ctrl_en or not prev_pow_en
                or prev_lock_viol
                or prev_mosi_sck_pol_oob
                or prev_miso_sck_pol_oob
                or prev_shutdown_sense
                or not prev_ext_en
                or prev_over_thresh
                or prev_thresh_underflow
                or prev_thresh_overflow
                or prev_bad_trig_cmd
                or prev_trig_cmd_buf_overflow
                or prev_trig_data_buf_underflow
                or prev_trig_data_buf_overflow
                or prev_bad_dac_cmd
                or prev_dac_cal_oob
                or prev_dac_val_oob
                or prev_dac_cmd_buf_underflow
                or prev_dac_cmd_buf_overflow
                or prev_dac_data_buf_underflow
                or prev_dac_data_buf_overflow
                or prev_unexp_dac_trig
                or prev_ldac_misalign
                or prev_dac_delay_too_short
                or prev_bad_adc_cmd
                or prev_adc_cmd_buf_underflow
                or prev_adc_cmd_buf_overflow
                or prev_adc_data_buf_underflow
                or prev_adc_data_buf_overflow
                or prev_unexp_adc_trig
                or prev_adc_delay_too_short
            ):
                # Determine expected status code in priority order (matches Verilog if/else if chain)
                if not prev_ctrl_en or not prev_pow_en:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_PS_SHUTDOWN")
                    )
                elif prev_lock_viol:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_LOCK_VIOL")
                    )
                elif prev_mosi_sck_pol_oob:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_MOSI_SCK_POL_OOB")
                    )
                elif prev_miso_sck_pol_oob:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_MISO_SCK_POL_OOB")
                    )
                elif prev_shutdown_sense:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_SHUTDOWN_SENSE"),
                        expected_board_num=self.extract_board_num(self.dut.shutdown_sense.value)
                    )
                elif not prev_ext_en:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_EXT_SHUTDOWN")
                    )
                elif prev_over_thresh:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_OVER_THRESH"),
                        expected_board_num=self.extract_board_num(self.dut.over_thresh.value)
                    )
                elif prev_thresh_underflow:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_THRESH_UNDERFLOW"),
                        expected_board_num=self.extract_board_num(self.dut.thresh_underflow.value)
                    )
                elif prev_thresh_overflow:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_THRESH_OVERFLOW"),
                        expected_board_num=self.extract_board_num(self.dut.thresh_overflow.value)
                    )
                elif prev_bad_trig_cmd:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_BAD_TRIG_CMD")
                    )
                elif prev_trig_cmd_buf_overflow:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_TRIG_CMD_BUF_OVERFLOW")
                    )
                elif prev_trig_data_buf_underflow:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_TRIG_DATA_BUF_UNDERFLOW")
                    )
                elif prev_trig_data_buf_overflow:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_TRIG_DATA_BUF_OVERFLOW")
                    )
                elif prev_bad_dac_cmd:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_BAD_DAC_CMD"),
                        expected_board_num=self.extract_board_num(self.dut.bad_dac_cmd.value)
                    )
                elif prev_dac_cal_oob:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_DAC_CAL_OOB"),
                        expected_board_num=self.extract_board_num(self.dut.dac_cal_oob.value)
                    )
                elif prev_dac_val_oob:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_DAC_VAL_OOB"),
                        expected_board_num=self.extract_board_num(self.dut.dac_val_oob.value)
                    )
                elif prev_dac_cmd_buf_underflow:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_DAC_CMD_BUF_UNDERFLOW"),
                        expected_board_num=self.extract_board_num(self.dut.dac_cmd_buf_underflow.value)
                    )
                elif prev_dac_cmd_buf_overflow:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_DAC_CMD_BUF_OVERFLOW"),
                        expected_board_num=self.extract_board_num(self.dut.dac_cmd_buf_overflow.value)
                    )
                elif prev_dac_data_buf_underflow:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_DAC_DATA_BUF_UNDERFLOW"),
                        expected_board_num=self.extract_board_num(self.dut.dac_data_buf_underflow.value)
                    )
                elif prev_dac_data_buf_overflow:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_DAC_DATA_BUF_OVERFLOW"),
                        expected_board_num=self.extract_board_num(self.dut.dac_data_buf_overflow.value)
                    )
                elif prev_unexp_dac_trig:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_UNEXP_DAC_TRIG"),
                        expected_board_num=self.extract_board_num(self.dut.unexp_dac_trig.value)
                    )
                elif prev_ldac_misalign:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_LDAC_MISALIGN"),
                        expected_board_num=self.extract_board_num(self.dut.ldac_misalign.value)
                    )
                elif prev_dac_delay_too_short:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_DAC_DELAY_TOO_SHORT"),
                        expected_board_num=self.extract_board_num(self.dut.dac_delay_too_short.value)
                    )
                elif prev_bad_adc_cmd:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_BAD_ADC_CMD"),
                        expected_board_num=self.extract_board_num(self.dut.bad_adc_cmd.value)
                    )
                elif prev_adc_cmd_buf_underflow:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_ADC_CMD_BUF_UNDERFLOW"),
                        expected_board_num=self.extract_board_num(self.dut.adc_cmd_buf_underflow.value)
                    )
                elif prev_adc_cmd_buf_overflow:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_ADC_CMD_BUF_OVERFLOW"),
                        expected_board_num=self.extract_board_num(self.dut.adc_cmd_buf_overflow.value)
                    )
                elif prev_adc_data_buf_underflow:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_ADC_DATA_BUF_UNDERFLOW"),
                        expected_board_num=self.extract_board_num(self.dut.adc_data_buf_underflow.value)
                    )
                elif prev_adc_data_buf_overflow:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_ADC_DATA_BUF_OVERFLOW"),
                        expected_board_num=self.extract_board_num(self.dut.adc_data_buf_overflow.value)
                    )
                elif prev_unexp_adc_trig:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_UNEXP_ADC_TRIG"),
                        expected_board_num=self.extract_board_num(self.dut.unexp_adc_trig.value)
                    )
                elif prev_adc_delay_too_short:
                    await self.check_state_and_status(
                        expected_state=self.get_state_value("S_HALTING"),
                        expected_status_code=self.get_status_value("STS_ADC_DELAY_TOO_SHORT"),
                        expected_board_num=self.extract_board_num(self.dut.adc_delay_too_short.value)
                    )
                break
        return

    async def halting_scoreboard(self):
        # State should be S_HALTING
        await self.check_state(self.get_state_value("S_HALTING"))

        await RisingEdge(self.dut.clk)
        await ReadOnly()

        await self.check_state(self.get_state_value("S_HALTED"))
        assert self.dut.timer.value.integer == 0, \
        f"Expected timer to be 0 in S_HALTED, got {self.dut.timer.value.integer}"
        assert self.dut.n_shutdown_force.value.integer == 0, \
        f"Expected n_shutdown_force to be 0 in S_HALTED, got {self.dut.n_shutdown_force.value.integer}"
        assert self.dut.shutdown_rst.value.integer == 0, \
        f"Expected shutdown_rst to be 0 in S_HALTED, got {self.dut.shutdown_rst.value.integer}"
        assert self.dut.shutdown_sense_en.value.integer == 0, \
        f"Expected shutdown_sense_en to be 0 in S_HALTED, got {self.dut.shutdown_sense_en.value.integer}"
        assert self.dut.unlock_cfg.value.integer == 1, \
        f"Expected unlock_cfg to be 1 in S_HALTED, got {self.dut.unlock_cfg.value.integer}"
        assert self.dut.spi_clk_gate.value.integer == 0, \
        f"Expected spi_clk_gate to be 0 in S_HALTED, got {self.dut.spi_clk_gate.value.integer}"
        assert self.dut.spi_en.value.integer == 0, \
        f"Expected spi_en to be 0 in S_HALTED, got {self.dut.spi_en.value.integer}"
        assert self.dut.block_bufs.value.integer == 1, \
        f"Expected block_bufs to be 1 in S_HALTED, got {self.dut.block_bufs.value.integer}"
        assert self.dut.ps_interrupt.value.integer == 1, \
        f"Expected ps_interrupt to be 1 in S_HALTED, got {self.dut.ps_interrupt.value.integer}"
        return

    async def halted_scoreboard(self): # HAS INDEFINITE WAIT
        # State should be S_HALTED
        await self.check_state(self.get_state_value("S_HALTED"))
        assert self.dut.timer.value.integer == 0, \
        f"Expected timer to be 0 in S_HALTED, got {self.dut.timer.value.integer}"
        assert self.dut.n_shutdown_force.value.integer == 0, \
        f"Expected n_shutdown_force to be 0 in S_HALTED, got {self.dut.n_shutdown_force.value.integer}"
        assert self.dut.shutdown_rst.value.integer == 0, \
        f"Expected shutdown_rst to be 0 in S_HALTED, got {self.dut.shutdown_rst.value.integer}"
        assert self.dut.shutdown_sense_en.value.integer == 0, \
        f"Expected shutdown_sense_en to be 0 in S_HALTED, got {self.dut.shutdown_sense_en.value.integer}"
        assert self.dut.unlock_cfg.value.integer == 1, \
        f"Expected unlock_cfg to be 1 in S_HALTED, got {self.dut.unlock_cfg.value.integer}"
        assert self.dut.spi_clk_gate.value.integer == 0, \
        f"Expected spi_clk_gate to be 0 in S_HALTED, got {self.dut.spi_clk_gate.value.integer}"
        assert self.dut.spi_en.value.integer == 0, \
        f"Expected spi_en to be 0 in S_HALTED, got {self.dut.spi_en.value.integer}"
        assert self.dut.block_bufs.value.integer == 1, \
        f"Expected block_bufs to be 1 in S_HALTED, got {self.dut.block_bufs.value.integer}"
        assert self.dut.ps_interrupt.value.integer == 1, \
        f"Expected ps_interrupt to be 1 in S_HALTED, got {self.dut.ps_interrupt.value.integer}"

        while True:
            await RisingEdge(self.dut.clk)
            prev_ctrl_en = self.dut.ctrl_en.value.integer
            prev_pow_en = self.dut.pow_en.value.integer
            prev_ps_interrupt = self.dut.ps_interrupt.value.integer
            await ReadOnly()
            
            if(prev_ps_interrupt):
                # Interrupt should be reset
                assert self.dut.ps_interrupt.value.integer == 0, \
                f"Expected ps_interrupt to be 0 after one cycle in S_HALTED, got {self.dut.ps_interrupt.value.integer}"

            if(prev_ctrl_en == 0 and prev_pow_en == 0):
                await self.check_state_and_status(
                    expected_state=self.get_state_value("S_IDLE"),
                    expected_status_code=self.get_status_value("STS_OK"),
                    expected_board_num=0
                )
                assert self.dut.unlock_cfg.value.integer == 1, \
                f"Expected unlock_cfg to be 1 in S_IDLE, got {self.dut.unlock_cfg.value.integer}"
                break
        return


    # =========================================================================
    # Reach methods
    # Each method drives the DUT to the very start of a target state and
    # returns in ReadWrite phase, ready for the caller to drive inputs.
    #
    # Recursion / composition:
    #   Each method calls the one that reaches the preceding state, so
    #   reach_s_running() implicitly walks the full startup sequence.
    # =========================================================================

    async def _wait_for_state_rw(self, target_state, name):
        """
        Helper: spin rising edges until the DUT enters target_state,
        then leave in ReadWrite phase.  Raises on timeout.
        """
        max_cycles = self.timeout_time // self.clk_period
        for _ in range(max_cycles):
            await RisingEdge(self.dut.clk)
            await ReadWrite()
            if int(self.dut.state.value) == target_state:
                return
        raise AssertionError(
            f"Timeout waiting to reach {name} after {max_cycles} cycles"
        )

    async def reach_s_idle(self):
        """
        Reset the DUT and return at the start of S_IDLE in ReadWrite phase.
        All outputs are in their reset state; all inputs are at the __init__
        defaults (ctrl_en=0, pow_en=0, ext_en=0, spi_off=1, …).
        """
        await self.reset()
        await ReadWrite()
        await self.check_state(self.get_state_value("S_IDLE"))

    async def reach_s_confirm_spi_rst(self):
        """
        Drive DUT to the start of S_CONFIRM_SPI_RST in ReadWrite phase.
        Drives: ctrl_en=1, ext_en=1 (all OOB signals remain 0).
        One clock edge: S_IDLE → S_CONFIRM_SPI_RST.
        """
        await self.reach_s_idle()
        self.dut.ctrl_en.value = 1
        self.dut.ext_en.value = 1
        await RisingEdge(self.dut.clk)
        await ReadWrite()
        await self.check_state(self.get_state_value("S_CONFIRM_SPI_RST"))

    async def reach_s_power_on_ctrl_brd(self):
        """
        Drive DUT to the start of S_POWER_ON_CRTL_BRD in ReadWrite phase.
        Drives: spi_off=1, calc_n_cs_done=1; waits for timer >= 10 cycles.
        """
        await self.reach_s_confirm_spi_rst()
        self.dut.spi_off.value = 1
        self.dut.calc_n_cs_done.value = 1
        await self._wait_for_state_rw(
            self.get_state_value("S_POWER_ON_CRTL_BRD"), "S_POWER_ON_CRTL_BRD"
        )

    async def reach_s_confirm_spi_start(self):
        """
        Drive DUT to the start of S_CONFIRM_SPI_START in ReadWrite phase.
        No additional inputs needed; waits for SHUTDOWN_FORCE_DELAY cycles.
        """
        await self.reach_s_power_on_ctrl_brd()
        await self._wait_for_state_rw(
            self.get_state_value("S_CONFIRM_SPI_START"), "S_CONFIRM_SPI_START"
        )

    async def reach_s_wait_for_pow_en(self):
        """
        Drive DUT to the start of S_WAIT_FOR_POW_EN in ReadWrite phase.
        Drives: spi_off=0 (signals the SPI subsystem has started).
        One clock edge: S_CONFIRM_SPI_START → S_WAIT_FOR_POW_EN.
        """
        await self.reach_s_confirm_spi_start()
        self.dut.spi_off.value = 0
        await RisingEdge(self.dut.clk)
        await ReadWrite()
        await self.check_state(self.get_state_value("S_WAIT_FOR_POW_EN"))

    async def reach_s_power_on_amp_brd(self):
        """
        Drive DUT to the start of S_POWER_ON_AMP_BRD in ReadWrite phase.
        Drives: pow_en=1.
        One clock edge: S_WAIT_FOR_POW_EN → S_POWER_ON_AMP_BRD.
        """
        await self.reach_s_wait_for_pow_en()
        self.dut.pow_en.value = 1
        await RisingEdge(self.dut.clk)
        await ReadWrite()
        await self.check_state(self.get_state_value("S_POWER_ON_AMP_BRD"))

    async def reach_s_amp_power_wait(self):
        """
        Drive DUT to the start of S_AMP_POWER_WAIT in ReadWrite phase.
        Waits for SHUTDOWN_RESET_PULSE cycles.
        """
        await self.reach_s_power_on_amp_brd()
        await self._wait_for_state_rw(
            self.get_state_value("S_AMP_POWER_WAIT"), "S_AMP_POWER_WAIT"
        )

    async def reach_s_running(self):
        """
        Drive DUT to the start of S_RUNNING in ReadWrite phase.
        Waits for SHUTDOWN_RESET_DELAY cycles.
        Note: ps_interrupt=1 on entry — S_RUNNING's first clock cycle clears it
        before any error checks fire.
        """
        await self.reach_s_amp_power_wait()
        await self._wait_for_state_rw(
            self.get_state_value("S_RUNNING"), "S_RUNNING"
        )

    async def reach_s_halting(self):
        """
        Drive DUT to the start of S_HALTING in ReadWrite phase.
        Reaches S_RUNNING, lets the ps_interrupt-clearing cycle complete,
        then drops ctrl_en to trigger STS_PS_SHUTDOWN → S_HALTING.
        """
        await self.reach_s_running()
        # Consume the interrupt-clearing cycle (first cycle in S_RUNNING).
        # ps_interrupt is 1; the DUT clears it and does not check errors yet.
        await RisingEdge(self.dut.clk)
        await ReadWrite()
        # Now ps_interrupt=0; drop ctrl_en to trigger STS_PS_SHUTDOWN.
        self.dut.ctrl_en.value = 0
        await RisingEdge(self.dut.clk)
        await ReadWrite()
        await self.check_state(self.get_state_value("S_HALTING"))

    async def reach_s_halted(self):
        """
        Drive DUT to the start of S_HALTED in ReadWrite phase.
        Reaches S_HALTING; S_HALTING always transitions to S_HALTED in one cycle.
        Note: ps_interrupt=1 on entry to S_HALTED.
        """
        await self.reach_s_halting()
        await RisingEdge(self.dut.clk)
        await ReadWrite()
        await self.check_state(self.get_state_value("S_HALTED"))

            
