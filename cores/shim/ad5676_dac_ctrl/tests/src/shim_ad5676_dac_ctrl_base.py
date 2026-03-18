import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, ReadWrite, Combine
from collections import deque
from fwft_fifo_model import fwft_fifo_model
import random

class shim_ad5676_dac_ctrl_base:

    CMD_ENCODING = {
        'NO_OP'    : 0,  # 3'd0: Delay or trigger wait, optional LDAC pulse
        'SET_CAL'  : 1,  # 3'd1: Set calibration value for a channel
        'DAC_WR'   : 2,  # 3'd2: Write DAC values (4 words, 2 channels each)
        'DAC_WR_CH': 3,  # 3'd3: Write single DAC channel
        'GET_CAL'  : 4,  # 3'd4: Read calibration value for a channel
        'ZERO'     : 5,  # 3'd5: Set all channels to their calibrated (midrange) zero values.
        'CANCEL'   : 7   # 3'd7: Cancel current wait or delay
    }

    STATE_ENCODING = {
        'S_RESET'    : 0,  # Reset state; waits for resetn deassertion.
        'S_INIT'     : 1,  # Initialization; starts SPI register write for boot test.
        'S_TEST_WR'  : 2,  # Boot test: writes test value to DAC register.
        'S_REQ_RD'   : 3,  # Boot test: requests readback of test value.
        'S_TEST_RD'  : 4,  # Boot test: reads back register value and checks for match.
        'S_SET_MID'  : 5,  # Boot test: sets all DAC channels to midrange value after successful test.
        'S_IDLE'     : 6,  # Idle; waits for new command from buffer.
        'S_DELAY'    : 7,  # Delay timer; waits for specified cycles before next command.
        'S_TRIG_WAIT': 8,  # Waits for external trigger signal.
        'S_DAC_WR'   : 9,  # Performs DAC write sequence for all channels.
        'S_DAC_WR_CH': 10, # Immediately and simply write to a single DAC channel.
        'S_ERROR'    : 15  # Error state; indicates boot/readback failure or invalid command/condition.
    }

    DAC_LOADING_STAGES = {
        'DAC_LOAD_STAGE_INIT': 0,
        'DAC_LOAD_STAGE_CAL' : 1,
        'DAC_LOAD_STAGE_CONV': 2
    }

    DEBUG_ENCODING = {
        'DBG_MISO_DATA'       : 1,
        'DBG_STATE_TRANSITION': 2,
        'DBG_N_CS_TIMER'      : 3,
        'DBG_SPI_BIT'         : 4,
        'CAL_DATA'            : 8
    }

    def __init__ (self, dut, clk_period=4, miso_sck_period=4, time_unit='ns', cmd_buf_depth=16):
        self.dut = dut
        self.clk_period = clk_period
        self.miso_sck_period = miso_sck_period
        self.time_unit = time_unit

        # Parameters
        self.ABS_CAL_MAX = int(self.dut.ABS_CAL_MAX.value)
        self.TRIG_BIT = 28
        self.CONT_BIT = 27
        self.LDAC_BIT = 26
        self.SPI_CMD_BIT_WIDTH = 24
        self.DAC_MID_RANGE = 0x8000
        self.SPI_CMD_LDAC_WRITE = 0b0001
        self.SPI_CMD_IMMED_WRITE = 0b0011
        self.CAL_DATA = 8
        self.SPI_BIT_MISO_START = 18

        # Initialize clocks
        cocotb.start_soon(Clock(dut.clk, clk_period, time_unit).start(start_high=False))
        cocotb.start_soon(Clock(dut.miso_sck, miso_sck_period, time_unit).start(start_high=False))

        self.dut._log.info(f"ABS_CAL_MAX set to : {self.ABS_CAL_MAX}")

        # Initialize input signals
        self.dut.boot_test_skip.value = 1
        self.dut.debug.value = 0
        self.dut.do_pre_delay.value = 1
        self.dut.n_cs_high_time.value = 16
        self.dut.cal_init_val.value = 0
        self.dut.cmd_buf_word.value = 0
        self.dut.cmd_buf_empty.value = 1
        self.dut.trigger.value = 0
        self.dut.ldac_shared.value = 0
        self.dut.miso.value = 0
        self.dut.data_buf_full.value = 0

        # Interface FIFOs
        self.cmd_buf_depth = cmd_buf_depth
        self.cmd_buf = fwft_fifo_model(dut, "CMD_FIFO_MODEL", DEPTH=cmd_buf_depth)

        # Queue to keep track of current executing command in the DUT
        self.executing_cmd_queue = deque()

    def get_cmd_name(self, cmd_value):
        for cmd_name, cmd_num in self.CMD_ENCODING.items():
            if cmd_num == cmd_value:
                return cmd_name
        return f"UNKNOWN_CMD_{cmd_value}"

    def get_state_name(self, state_value):
        for state_name, state_num in self.STATE_ENCODING.items():
            if state_num == state_value:
                return state_name
        else:
            return f"UNKNOWN_STATE_{state_value}"
        
    def get_loading_stage_name(self, stage_value):
        for stage_name, stage_num in self.DAC_LOADING_STAGES.items():
            if stage_num == stage_value:
                return stage_name
        else:
            return f"UNKNOWN_STAGE_{stage_value}"
        
    def get_debug_name(self, debug_value):
        for debug_name, debug_num in self.DEBUG_ENCODING.items():
            if debug_num == debug_value:
                return debug_name
        else:
            return f"UNKNOWN_DEBUG_{debug_value}"
        
    async def reset(self):
        """ Reset the DUT, hold the reset for 2 clock cycles """
        await RisingEdge(self.dut.clk)
        self.dut.resetn.value = 0
        self.dut._log.info("STARTING RESET")
        self.cmd_buf.reset()
        self.executing_cmd_queue.clear()
        await RisingEdge(self.dut.clk)
        await RisingEdge(self.dut.clk)
        self.dut.resetn.value = 1
        await ReadOnly()
        self.dut._log.info("RESET COMPLETE")

        ## CHECK COMMAND WORD RELATED SIGNALS
        #assert self.dut.do_ldac.value == 0, "do_ldac should be low after reset"
        #assert self.dut.wait_for_trig.value == 0, "wait_for_trig should be low after reset"
        #assert self.dut.expect_next.value == 0, "expect_next should be low after reset"

        ## CHECK STATE and SETUP DONE
        #assert self.dut.state.value == self.STATE_ENCODING['S_RESET'], f"State should be S_RESET ({self.STATE_ENCODING['S_RESET']}) after reset, got {self.get_state_name(int(self.dut.state.value))}"
        #assert self.dut.setup_done.value == 0, "setup_done should be low after reset"

        ## CHECK COUNTERS
        #assert self.dut.delay_timer.value == 0, "delay_timer should be 0 after reset"
        #assert self.dut.trigger_counter.value == 0, "trigger_counter should be 0 after reset"

        ## CHECK ERROR FLAGS
        #assert self.dut.boot_fail.value == 0, "boot_fail should be low after reset"
        #assert self.dut.unexp_trig.value == 0, "unexp_trig should be low after reset"
        #assert self.dut.delay_too_short.value == 0, "delay_too_short should be low after reset"
        #assert self.dut.ldac_misalign.value == 0, "ldac_misalign should be low after reset"
        #assert self.dut.bad_cmd.value == 0, "bad_cmd should be low after reset"
        #assert self.dut.cmd_buf_underflow.value == 0, "cmd_buf_underflow should be low after reset"
        #assert self.dut.data_buf_overflow.value == 0, "data_buf_overflow should be low after reset"
        #assert self.dut.dac_val_oob.value == 0, "dac_val_oob should be low after reset"

        ## CHECK LDAC activation and absolute DAC values
        #assert self.dut.ldac.value == 0, "ldac should be low after reset"
        #assert self.dut.abs_dac_val_concat.value == 0, "abs_dac_val_concat should be 0 after reset"

        ## CHECK CALIBRATION VALUES
        #assert self.dut.cal_oob.value == 0, "cal_oob should be low after reset"
        #for ch in range(8):
        #    assert self.dut.cal_val[ch].value == self.dut.cal_init_val.value, f"cal_val[{ch}] should be cal_init_val after reset"

        ## CHECK DAC WORD SEQUENCING RELATED SIGNALS
        #assert self.dut.read_next_dac_val_pair.value == 0, "read_next_dac_val_pair should be low after reset"
        #assert self.dut.dac_wr_done.value == 0, "dac_wr_done should be low after reset"
        #assert self.dut.dac_channel.value == 0, "dac_channel should be 0 after reset"
        ##assert self.dut.first_dac_val_signed.value == 0, "first_dac_val_signed should be low after reset"
        #assert self.dut.first_dac_val_cal_signed.value == 0, "first_dac_val_cal_signed should be low after reset"
        #assert self.dut.second_dac_val_signed.value == 0, "second_dac_val_signed should be low after reset"
        #assert self.dut.second_dac_val_cal_signed.value == 0, "second_dac_val_cal_signed should be low after reset"
        #assert self.dut.dac_load_stage.value == 0, "dac_load_stage should be DAC_LOAD_STAGE_INIT (0) after reset"

        ## CHECK SPI MOSI CONTROL
        #assert self.dut.n_cs_high_time_latched.value == 0, "n_cs_high_time_latched should be 0 after reset"
        #assert self.dut.n_cs_timer.value == 0, "n_cs_timer should be 0 after reset"
        #assert self.dut.n_cs.value == 1, "n_cs should be high after reset"
        #assert self.dut.spi_bit.value == 0, "spi_bit should be 0 after reset"
        #assert self.dut.mosi_shift_reg.value == 0, "mosi_shift_reg should be 0 after reset"
        #assert self.dut.start_miso_mosi_clk.value == 0, "start_miso_mosi_clk should be low after reset"

        # CHECK DAC DATA OUTPUT RELATED SIGNALS
        #assert self.dut.data_buf_wr_en.value == 0, "data_buf_wr_en should be low after reset"
        #assert self.dut.data_word.value == 0, "data_word should be 0 after reset"
    
    # ---------------------------
    # Random input drivers/generators 
    # ---------------------------
    async def random_trigger_driver(self):
        """ Randomly assert the trigger input signal. """
        for _ in range(10):  # Initial delay before starting
            await RisingEdge(self.dut.clk)
        while True:
            await RisingEdge(self.dut.clk)
            # 30% chance to toggle trigger each clock cycle
            if random.random() < 0.3:
                self.dut.trigger.value = 1
                self.dut._log.info("Trigger asserted")
                await RisingEdge(self.dut.clk)
                self.dut.trigger.value = 0
                self.dut._log.info("Trigger deasserted")

    # ---------------------------
    # Command driver / command buffer model
    # ---------------------------

    async def send_commands(self, cmd_word_list):
        """
        Send a list of commands to the DUT via the command buffer model.
        If the command buffer is full, retry writing the same command on the next clock cycle.
        """
        self.dut._log.info(f"Sending {len(cmd_word_list)} commands to command buffer")

        for cmd_word in cmd_word_list:
            written = False
            while not written:
                await RisingEdge(self.dut.clk)
                # If buffer not full, write and move to next command word
                if not self.cmd_buf.is_full():
                    self.cmd_buf.write_item(cmd_word)
                    written = True
                else:
                    # Buffer full: wait and retry on next cycle
                    self.dut._log.debug("cmd_buf full, retrying write on next cycle")

        self.dut._log.info(f"All {len(cmd_word_list)} commands sent to command buffer")

    async def command_buf_model(self):
        """
        Model of the command buffer. Connected to DUT's cmd_buf_rd_en, cmd_buf_word, and cmd_buf_empty.
        The DUT will read commands from this buffer.
        """
        while True:
            await RisingEdge(self.dut.clk)

            # Update Buffer status signals
            self.dut.cmd_buf_empty.value = 1 if self.cmd_buf.is_empty() else 0

            # FWFT behavior: always present the next item on cmd_buf_word
            fwft_data = self.cmd_buf.peek_item() if not self.cmd_buf.is_empty() else None
            self.dut.cmd_buf_word.value = fwft_data if fwft_data is not None else 0

            await ReadOnly() # Wait for combinational logic to settle
            # Buffer reads
            if self.dut.cmd_buf_rd_en.value == 1 and not self.cmd_buf.is_empty():
                self.executing_cmd_queue.append(self.cmd_buf.pop_item())

    # ---------------------------
    # Command builders / decoder
    # ---------------------------

    def build_noop(self, *, trig_wait: int, cont: int, ldac: int, value: int) -> int:
        """NO_OP header: [31:29]=0, [TRIG]=trig_wait, [CONT]=cont, [LDAC]=ldac, [24:0]=value."""
        cmd_word = 0
        cmd_word |= (self.CMD_ENCODING['NO_OP'] & 0x7) << 29
        cmd_word |= (1 if trig_wait else 0) << self.TRIG_BIT
        cmd_word |= (1 if cont else 0) << self.CONT_BIT
        cmd_word |= (1 if ldac else 0) << self.LDAC_BIT
        cmd_word |= (value & 0x1FFFFFF)
        return cmd_word
    
    def build_set_cal(self, ch: int, cal_signed_16: int) -> int:
        """SET_CAL: [31:29]=1, [18:16]=ch, [15:0]=signed cal (2's complement)."""
        cmd_word = (self.CMD_ENCODING['SET_CAL'] & 0x7) << 29
        cmd_word |= ((ch & 0x7) << 16)
        cmd_word |= (cal_signed_16 & 0xFFFF)
        return cmd_word
    
    def build_dac_wr_header(self, *, trig_wait: int, cont: int, ldac: int, value: int) -> int:
        """DAC_WR header: [31:29]=2, TRIG/CONT/LDAC bits set, [24:0]=delay or trigger count."""
        cmd_word = (self.CMD_ENCODING['DAC_WR'] & 0x7) << 29
        cmd_word |= (1 if trig_wait else 0) << self.TRIG_BIT
        cmd_word |= (1 if cont else 0) << self.CONT_BIT
        cmd_word |= (1 if ldac else 0) << self.LDAC_BIT
        cmd_word |= (value & 0x1FFFFFF)
        return cmd_word
    
    def build_dac_pair(self, v_lo_chN: int, v_hi_chNp1: int) -> int:
        """Payload word for DAC_WR: [31:16]=ch(N+1) value, [15:0]=ch N value (offset format)."""
        return ((v_hi_chNp1 & 0xFFFF) << 16) | (v_lo_chN & 0xFFFF)
    
    def build_dac_wr_ch(self, ch: int, value: int) -> int:
        """DAC_WR_CH: [31:29]=3, [18:16]=ch, [15:0]=value (offset format)."""
        cmd_word = (self.CMD_ENCODING['DAC_WR_CH'] & 0x7) << 29
        cmd_word |= ((ch & 0x7) << 16)
        cmd_word |= (value & 0xFFFF)
        return cmd_word
    
    def build_get_cal(self, ch: int) -> int:
        """GET_CAL: [31:29]=4, [18:16]=ch."""
        cmd_word = (self.CMD_ENCODING['GET_CAL'] & 0x7) << 29
        cmd_word |= ((ch & 0x7) << 16)
        return cmd_word
    
    def build_zero(self) -> int:
        """ZERO: [31:29]=5."""
        return (self.CMD_ENCODING['ZERO'] & 0x7) << 29

    def build_cancel(self) -> int:
        """CANCEL: [31:29]=7."""
        return (self.CMD_ENCODING['CANCEL'] & 0x7) << 29
    
    def decode_cmd(self, cmd_word: int) -> dict:
        """Decode a raw command word into a structured dict."""
        cmd_val = (cmd_word >> 29) & 0x7
        info = {"cmd": cmd_val, "name": self.get_cmd_name(cmd_val)}

        if cmd_val == self.CMD_ENCODING['NO_OP']:
            info.update({
                "trig": (cmd_word >> self.TRIG_BIT) & 1,
                "cont": (cmd_word >> self.CONT_BIT) & 1,
                "ldac": (cmd_word >> self.LDAC_BIT) & 1,
                "value": cmd_word & 0x1FFFFFF,
            })
        elif cmd_val == self.CMD_ENCODING['SET_CAL']:
            info.update({"ch": (cmd_word >> 16) & 0x7, "cal": cmd_word & 0xFFFF})
        elif cmd_val == self.CMD_ENCODING['DAC_WR']:
            info.update({
                "trig": (cmd_word >> self.TRIG_BIT) & 1,
                "cont": (cmd_word >> self.CONT_BIT) & 1,
                "ldac": (cmd_word >> self.LDAC_BIT) & 1,
                "value": cmd_word & 0x1FFFFFF,  # delay or trigger count after write
            })
        elif cmd_val == self.CMD_ENCODING['DAC_WR_CH']:
            info.update({"ch": (cmd_word >> 16) & 0x7, "value": cmd_word & 0xFFFF})
        elif cmd_val == self.CMD_ENCODING['GET_CAL']:
            info.update({"ch": (cmd_word >> 16) & 0x7})
        elif cmd_val == self.CMD_ENCODING['ZERO']:
            pass
        elif cmd_val == self.CMD_ENCODING['CANCEL']:
            pass
        else:
            # Unknown/bad command
            pass

        return info
    
    # --------------------------------------
    # Executing command scoreboard dispatcher
    # --------------------------------------

    async def executing_command_scoreboard(self, num_of_commands: int):
        """
        Pop entries from executing_cmd_queue (filled when cmd_buf_rd_en is asserted)
        and run per-command scoreboards. For DAC_WR, also consumes and verifies the 4 payload words.
        """
        if num_of_commands == 0:
            self.dut._log.info("Executing command scoreboard finished: 0 commands expected.")
            return

        processed = 0
        forked = []

        while processed < num_of_commands:
            # Wait until DUT pops one
            await RisingEdge(self.dut.clk)
            await ReadOnly()

            if len(self.executing_cmd_queue) == 0:
                continue

            popped_cmd_word = self.executing_cmd_queue.popleft()
            popped_cmd_value = (popped >> 29) & 0x7
            dut_cmd_word = int(self.dut.cmd_word.value)
            dut_cmd_value = int(self.dut.command.value)
            assert popped_cmd_word == dut_cmd_word, f"Cmd word mismatch: expected 0x{popped_cmd_word:08X} got 0x{dut_cmd_word:08X}"
            assert popped_cmd_value == dut_cmd_value, f"Cmd type mismatch: expected {self.get_cmd_name(popped_cmd_value)} got {self.get_cmd_name(dut_cmd_value)}"

            decoded = self.decode_cmd(popped_cmd_word)
            idx = processed
            processed += 1

            if decoded["cmd"] == self.CMD_ENCODING['NO_OP']:
                forked.append(cocotb.start_soon(self._sb_noop(decoded, idx)))
            elif decoded["cmd"] == self.CMD_ENCODING['SET_CAL']:
                forked.append(cocotb.start_soon(self._sb_set_cal(decoded, idx)))
            elif decoded["cmd"] == self.CMD_ENCODING['DAC_WR']:
                forked.append(cocotb.start_soon(self._sb_dac_wr_header(decoded, idx)))

                for pair_idx in range(4):
                    while True:
                        await RisingEdge(self.dut.clk)
                        await ReadOnly()
                        if len(self.executing_cmd_queue) > 0:
                            break

                    popped_payload_word = self.executing_cmd_queue.popleft()
                    dut_payload_word = int(self.dut.cmd_word.value)
                    assert popped_payload_word == dut_payload_word, f"DAC_WR payload word mismatch: expected 0x{popped_payload_word:08X} got 0x{dut_payload_word:08X}"
                    processed += 1
                    forked.append(cocotb.start_soon(self._sb_dac_spi_word_sequencing(pair_idx, popped_payload_word)))

            elif decoded["cmd"] == self.CMD_ENCODING['DAC_WR_CH']:
                forked.append(cocotb.start_soon(self._sb_dac_wr_ch(decoded, idx)))
            elif decoded["cmd"] == self.CMD_ENCODING['GET_CAL']:
                forked.append(cocotb.start_soon(self._sb_get_cal(decoded, idx)))
            elif decoded["cmd"] == self.CMD_ENCODING['ZERO']:
                forked.append(cocotb.start_soon(self._sb_zero(idx)))
            elif decoded["cmd"] == self.CMD_ENCODING['CANCEL']:
                forked.append(cocotb.start_soon(self._sb_cancel(idx)))
            else:
                forked.append(cocotb.start_soon(self._sb_bad_cmd(idx)))

        self.dut._log.info(f"All {num_of_commands} commands are sent/processed, waiting for scoreboards to complete...")
        if forked:
            await Combine(*forked)
        self.dut._log.info("All scoreboards completed.")

    # ----------------------------
    # Per-command scoreboards
    # ----------------------------

    async def _sb_noop(self, info: dict, i: int):
        """Verify NO_OP command execution."""

        self.dut._log.info(f"[{i}] NO_OP: trig={info['trig']} cont={info['cont']} ldac={info['ldac']} val={info['value']}")

        # We should be in IDLE state when starting NO_OP and be ready to this command
        assert int(self.dut.state.value) == self.STATE_ENCODING['S_IDLE'], \
              f"[{i}] NO_OP: Expected state IDLE (6), got {self.get_state_name(int(self.dut.state.value))}"
        assert int(self.dut.do_next_cmd.value) == 1, \
              f"[{i}] NO_OP: do_next_cmd should be asserted when starting NO_OP"

        # Depending on the TRIG_BIT, next_cmd_state should be either S_TRIG_WAIT or S_DELAY
        expected_next_state = self.STATE_ENCODING['S_TRIG_WAIT'] if info['trig'] == 1 else self.STATE_ENCODING['S_DELAY']
        assert int(self.dut.next_cmd_state.value) == expected_next_state, \
             f"[{i}] NO_OP: Expected next_cmd_state {expected_next_state}, got {self.get_state_name(int(self.dut.next_cmd_state.value))}"
        
        await RisingEdge(self.dut.clk)
        await ReadOnly()

        # Return if cancel was issued
        if int(self.dut.cancel_wait.value) == 1:
            self.dut._log.info(f"[{i}] NO_OP: Cancel detected immediately after command fetch, exiting NO_OP scoreboard.")
            return

        # Expected do_ldac, wait_for_trig, expect_next values should be set according to command bits
        expected_do_ldac = 1 if info['ldac'] == 1 else 0
        expected_wait_for_trig = 1 if info['trig'] == 1 else 0
        expected_expect_next = 1 if info['cont'] == 1 else 0

        assert int(self.dut.do_ldac.value) == expected_do_ldac, \
            f"[{i}] NO_OP: do_ldac should be {expected_do_ldac}, got {int(self.dut.do_ldac.value)}"
        assert int(self.dut.wait_for_trig.value) == expected_wait_for_trig, \
            f"[{i}] NO_OP: wait_for_trig should be {expected_wait_for_trig}, got {int(self.dut.wait_for_trig.value)}"
        assert int(self.dut.expect_next.value) == expected_expect_next, \
            f"[{i}] NO_OP: expect_next should be {expected_expect_next}, got {int(self.dut.expect_next.value)}"
        
        # If TRIG_BIT is 1 trigger_counter should be set to value field otherwise, delay_timer should be set to value field
        expected_trigger_counter = info['value'] if info['trig'] == 1 else 0
        expected_delay_timer = info['value'] if info['trig'] == 0 else 0

        if expected_wait_for_trig:
            assert int(self.dut.trigger_counter.value) == expected_trigger_counter, \
                f"[{i}] NO_OP: trigger_counter should be {expected_trigger_counter}, got {int(self.dut.trigger_counter.value)}"
            
            # If expected_trigger_counter 0 to begin with, trig_wait_done should be asserted immediately
            if expected_trigger_counter == 0:
                assert int(self.dut.trig_wait_done.value) == 1, \
                    f"[{i}] NO_OP: trig_wait_done should be asserted when trigger_counter is 0"
            
            while expected_trigger_counter > 0:
                await RisingEdge(self.dut.clk)
                previous_external_trigger = int(self.dut.trigger.value)
                await ReadOnly()
                current_external_trigger = int(self.dut.trigger.value)

                # Return if cancel was issued
                if int(self.dut.cancel_wait.value) == 1:
                    self.dut._log.info(f"[{i}] NO_OP: Cancel detected immediately after command fetch, exiting NO_OP scoreboard.")
                    return

                # When final trigger is received trig_wait_done should be asserted and cmd_done should be asserted
                if current_external_trigger == 1 and expected_trigger_counter == 1:
                    self.dut._log.info(f"[{i}] NO_OP: Final trigger received")
                    assert int(self.dut.trig_wait_done.value) == 1, \
                        f"[{i}] NO_OP: trig_wait_done should be asserted when final trigger is received"
                    assert int(self.dut.cmd_done.value) == 1, \
                        f"[{i}] NO_OP: cmd_done should be asserted when final trigger is received"

                # When an external trigger is received, the trigger_counter should decrement
                if previous_external_trigger == 1:
                    expected_trigger_counter -= 1
                    self.dut._log.info(f"[{i}] NO_OP: Trigger received, decremented trigger_counter to {expected_trigger_counter}")

                assert int(self.dut.trigger_counter.value) == expected_trigger_counter, \
                    f"[{i}] NO_OP: trigger_counter should be {expected_trigger_counter}, got {int(self.dut.trigger_counter.value)}"
                
            # When we exit the loop trigger_counter should be 0
            assert int(self.dut.trigger_counter.value) == 0, \
                f"[{i}] NO_OP: trigger_counter should be 0 after completing trigger wait"
            
            # Now, we should be in S_IDLE
            assert int(self.dut.state.value) == self.STATE_ENCODING['S_IDLE'], \
                f"[{i}] NO_OP: Expected state S_IDLE (6), got {self.get_state_name(int(self.dut.state.value))}"
            
            # After cmd_done if expected_do_ldac is set, ldac should be asserted
            if expected_do_ldac:
                assert int(self.dut.ldac.value) == 1, \
                    f"[{i}] NO_OP: ldac should be asserted after cmd_done if do_ldac is set"
            return
        
        else:
            assert int(self.dut.delay_timer.value) == expected_delay_timer, \
                f"[{i}] NO_OP: delay_timer should be {expected_delay_timer}, got {int(self.dut.delay_timer.value)}"
            
            while expected_delay_timer > 0:
                await RisingEdge(self.dut.clk)
                await ReadOnly()

                # Return if cancel was issued
                if int(self.dut.cancel_wait.value) == 1:
                    self.dut._log.info(f"[{i}] NO_OP: Cancel detected immediately after command fetch, exiting NO_OP scoreboard.")
                    return

                expected_delay_timer -= 1

                # log the current delay_timer and its expected value
                self.dut._log.info(f"[{i}] NO_OP: Expected delay_timer: {expected_delay_timer}, Current delay_timer: {int(self.dut.delay_timer.value)}")

                assert int(self.dut.delay_timer.value) == expected_delay_timer, \
                    f"[{i}] NO_OP: delay_timer should be {expected_delay_timer}, got {int(self.dut.delay_timer.value)}"
                
            # When we exit the loop delay_timer should be 0
            assert int(self.dut.delay_timer.value) == 0, \
                f"[{i}] NO_OP: delay_timer should be 0 after completing delay"
            
            # Now, we should still be in the S_DELAY state and cmd_done should be asserted
            assert int(self.dut.state.value) == self.STATE_ENCODING['S_DELAY'], \
                f"[{i}] NO_OP: Expected state S_DELAY (7), got {self.get_state_name(int(self.dut.state.value))}"
            assert int(self.dut.cmd_done.value) == 1, \
                f"[{i}] NO_OP: cmd_done should be asserted after completing delay"
            
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            # After cmd_done if expected_do_ldac is set, ldac should be asserted
            if expected_do_ldac:
                assert int(self.dut.ldac.value) == 1, \
                    f"[{i}] NO_OP: ldac should be asserted after cmd_done if do_ldac is set"
            return

    async def _sb_set_cal(self, info: dict, i: int):
        """Verify SET_CAL command execution."""

        self.dut._log.info(f"[{i}] SET_CAL: ch={info['ch']} cal=0x{info['cal']:04X}")

        # We should be in IDLE state when starting SET_CAL and be ready to this command
        assert int(self.dut.state.value) == self.STATE_ENCODING['S_IDLE'], \
              f"[{i}] SET_CAL: Expected state IDLE (6), got {self.get_state_name(int(self.dut.state.value))}"
        assert int(self.dut.do_next_cmd.value) == 1, \
              f"[{i}] SET_CAL: do_next_cmd should be asserted when starting SET_CAL"
        
        # next_cmd_state should be S_IDLE after SET_CAL
        expected_next_state = self.STATE_ENCODING['S_IDLE']
        assert int(self.dut.next_cmd_state.value) == expected_next_state, \
             f"[{i}] SET_CAL: Expected next_cmd_state {expected_next_state}, got {self.get_state_name(int(self.dut.next_cmd_state.value))}"
        
        expected_ch_index = info['ch']
        raw_val = info['cal']

        # Convert 16-bit 2's complement to signed integer
        if raw_val & 0x8000:
            expected_cal_signed_16 = raw_val - 0x10000
        else:
            expected_cal_signed_16 = raw_val

        expected_do_ldac = 0
        expected_wait_for_trig = 0
        expected_expect_next = 0

        await RisingEdge(self.dut.clk)
        await ReadOnly()

        # Expected do_ldac, wait_for_trig, expect_next values should be set to 0
        assert int(self.dut.do_ldac.value) == expected_do_ldac, \
            f"[{i}] SET_CAL: do_ldac should be {expected_do_ldac}, got {int(self.dut.do_ldac.value)}"
        assert int(self.dut.wait_for_trig.value) == expected_wait_for_trig, \
            f"[{i}] SET_CAL: wait_for_trig should be {expected_wait_for_trig}, got {int(self.dut.wait_for_trig.value)}"
        assert int(self.dut.expect_next.value) == expected_expect_next, \
            f"[{i}] SET_CAL: expect_next should be {expected_expect_next}, got {int(self.dut.expect_next.value)}"
        
        # The cal_val for the specified channel should be updated to the new value if its in between -ABS_CAL_MAX to +ABS_CAL_MAX
        # Otherwise, cal_oob should be asserted and cal_val should remain unchanged and error flag should be asserted, and one cycle later state should be S_ERROR
        current_cal_val_signed_16 = int(self.dut.cal_val[expected_ch_index].value.signed_integer)
        self.dut._log.info(f"[{i}] SET_CAL: Current cal_val[{expected_ch_index}] = {current_cal_val_signed_16}, Expected cal = {expected_cal_signed_16}")

        if -self.ABS_CAL_MAX <= expected_cal_signed_16 <= self.ABS_CAL_MAX:
            assert int(self.dut.cal_oob.value) == 0, \
                f"[{i}] SET_CAL: cal_oob should be 0 for in-bounds cal value"
            assert current_cal_val_signed_16 == expected_cal_signed_16, \
                f"[{i}] SET_CAL: cal_val[{expected_ch_index}] should be updated to {expected_cal_signed_16}, got {current_cal_val_signed_16}"
        else:
            assert int(self.dut.cal_oob.value) == 1, \
                f"[{i}] SET_CAL: cal_oob should be 1 for out-of-bounds cal value"
            assert current_cal_val_signed_16 == int(self.dut.cal_init_val.value.signed_integer), \
                f"[{i}] SET_CAL: cal_val[{expected_ch_index}] should remain unchanged, got {current_cal_val_signed_16}"
            assert int(self.dut.error.value) == 1, \
                f"[{i}] SET_CAL: error flag should be asserted for out-of-bounds cal value"
            
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            assert int(self.dut.state.value) == self.STATE_ENCODING['S_ERROR'], \
                f"[{i}] SET_CAL: Expected state S_ERROR (15) after out-of-bounds cal, got {self.get_state_name(int(self.dut.state.value))}"
        return

    async def _sb_dac_wr_header(self, info: dict, i: int):
        """Verify DAC_WR header command execution."""

        self.dut._log.info(f"[{i}] DAC_WR Header: trig={info['trig']} cont={info['cont']} ldac={info['ldac']} val={info['value']}")
    
        # We should be ready to take this command
        assert int(self.dut.do_next_cmd.value) == 1, \
              f"[{i}] DAC_WR: do_next_cmd should be asserted when starting DAC_WR"
        
        await RisingEdge(self.dut.clk)
        await ReadOnly()

        # Expected do_ldac, wait_for_trig, expect_next values should be set according to command bits
        expected_do_ldac = 1 if info['ldac'] == 1 else 0
        expected_wait_for_trig = 1 if info['trig'] == 1 else 0
        expected_expect_next = 1 if info['cont'] == 1 else 0

        assert int(self.dut.do_ldac.value) == expected_do_ldac, \
            f"[{i}] DAC_WR: do_ldac should be {expected_do_ldac}, got {int(self.dut.do_ldac.value)}"
        assert int(self.dut.wait_for_trig.value) == expected_wait_for_trig, \
            f"[{i}] DAC_WR: wait_for_trig should be {expected_wait_for_trig}, got {int(self.dut.wait_for_trig.value)}"
        assert int(self.dut.expect_next.value) == expected_expect_next, \
            f"[{i}] DAC_WR: expect_next should be {expected_expect_next}, got {int(self.dut.expect_next.value)}"
        
        # If TRIG_BIT is 1 trigger_counter should be set to value field otherwise, delay_timer should be set to value field
        expected_trigger_counter = info['value'] if info['trig'] == 1 else 0
        expected_delay_timer = info['value'] if info['trig'] == 0 else 0

        # Even though state will be S_DAC_WR here, delay or trigger wait counters will start immediately.
        # When the counters are complete, we expect that DAC_WR payload words will have already been sent. 
        if expected_wait_for_trig:
            assert int(self.dut.trigger_counter.value) == expected_trigger_counter, \
                f"[{i}] DAC_WR: trigger_counter should be {expected_trigger_counter}, got {int(self.dut.trigger_counter.value)}"
            
            # If expected_trigger_counter 0 to begin with, trig_wait_done should be asserted immediately
            if expected_trigger_counter == 0:
                assert int(self.dut.trig_wait_done.value) == 1, \
                    f"[{i}] DAC_WR: trig_wait_done should be asserted when trigger_counter is 0"
            
            while expected_trigger_counter > 0:
                await RisingEdge(self.dut.clk)
                await ReadOnly()

                # Return if cancel was issued
                if int(self.dut.cancel_wait.value) == 1:
                    self.dut._log.info(f"[{i}] NO_OP: Cancel detected immediately after command fetch, exiting NO_OP scoreboard.")
                    return

                external_trigger = int(self.dut.trigger.value)

                # When final trigger is received trig_wait_done should be asserted
                if external_trigger == 1 and expected_trigger_counter == 1:
                    self.dut._log.info(f"[{i}] DAC_WR: Final trigger received")
                    assert int(self.dut.trig_wait_done.value) == 1, \
                        f"[{i}] DAC_WR: trig_wait_done should be asserted when final trigger is received"

                # When an external trigger is received, the trigger_counter should decrement
                if external_trigger == 1:
                    expected_trigger_counter -= 1
                    self.dut._log.info(f"[{i}] DAC_WR: Trigger received, decremented trigger_counter to {expected_trigger_counter}")

                assert int(self.dut.trigger_counter.value) == expected_trigger_counter, \
                    f"[{i}] DAC_WR: trigger_counter should be {expected_trigger_counter}, got {int(self.dut.trigger_counter.value)}"
                
            # When we exit the loop trigger_counter should be 0
            assert int(self.dut.trigger_counter.value) == 0, \
                f"[{i}] DAC_WR: trigger_counter should be 0 after completing trigger wait"
            
            # cmd_done should be asserted and we should either be in S_DAC_WR (edge case) or S_TRIG_WAIT state
            assert int(self.dut.cmd_done.value) == 1, \
                f"[{i}] DAC_WR: cmd_done should be asserted after completing trigger wait"
            assert int(self.dut.state.value) in [self.STATE_ENCODING['S_TRIG_WAIT'], self.STATE_ENCODING['S_DAC_WR']], \
                f"[{i}] DAC_WR: Expected state S_TRIG_WAIT (8) or S_DAC_WR (9), got {self.get_state_name(int(self.dut.state.value))}"
            
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            # After cmd_done if expected_do_ldac is set, ldac should be asserted
            if expected_do_ldac:
                assert int(self.dut.ldac.value) == 1, \
                    f"[{i}] DAC_WR: ldac should be asserted after cmd_done if do_ldac is set"
            
            return
        
        else:
            assert int(self.dut.delay_timer.value) == expected_delay_timer, \
                f"[{i}] DAC_WR: delay_timer should be {expected_delay_timer}, got {int(self.dut.delay_timer.value)}"
            
            while expected_delay_timer > 0:
                await RisingEdge(self.dut.clk)
                await ReadOnly()

                # Return if cancel was issued
                if int(self.dut.cancel_wait.value) == 1:
                    self.dut._log.info(f"[{i}] NO_OP: Cancel detected immediately after command fetch, exiting NO_OP scoreboard.")
                    return

                expected_delay_timer -= 1
                assert int(self.dut.delay_timer.value) == expected_delay_timer, \
                    f"[{i}] DAC_WR: delay_timer should be {expected_delay_timer}, got {int(self.dut.delay_timer.value)}"
                
            # When we exit the loop delay_timer should be 0
            assert int(self.dut.delay_timer.value) == 0, \
                f"[{i}] DAC_WR: delay_timer should be 0 after completing delay"
            
            # cmd_done should be asserted and we should either be in S_DAC_WR (edge case) or S_DELAY state
            assert int(self.dut.cmd_done.value) == 1, \
                f"[{i}] DAC_WR: cmd_done should be asserted after completing delay"
            assert int(self.dut.state.value) in [self.STATE_ENCODING['S_DELAY'], self.STATE_ENCODING['S_DAC_WR']], \
                f"[{i}] DAC_WR: Expected state S_DELAY (7) or S_DAC_WR (9), got {self.get_state_name(int(self.dut.state.value))}"
            
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            # After cmd_done if expected_do_ldac is set, ldac should be asserted
            if expected_do_ldac:
                assert int(self.dut.ldac.value) == 1, \
                    f"[{i}] DAC_WR: ldac should be asserted after cmd_done if do_ldac is set"
            
            return
        
    async def _sb_dac_spi_word_sequencing(self, dac_word_pair_number, payload_word):
        """Scoreboard for verifying DAC SPI word sequencing for a given DAC word pair."""

        first_dac_val = payload_word & 0xFFFF
        second_dac_val = (payload_word >> 16) & 0xFFFF
        dac_channel_n = dac_word_pair_number * 2
        dac_channel_np1 = dac_channel_n + 1
        spi_word_list = []

        self.dut._log.info(f"[DAC_WR] DAC word pair {dac_word_pair_number}: ch{dac_channel_n} val=0x{first_dac_val:04X}, ch{dac_channel_np1} val=0x{second_dac_val:04X}")

        # Expected conversion values
        expected_first_dac_val_signed = self.offset_to_signed(first_dac_val)
        expected_second_dac_val_signed = self.offset_to_signed(second_dac_val)

        expected_first_dac_val_cal_signed = expected_first_dac_val_signed + int(self.dut.cal_val[dac_channel_n].value.signed_integer)
        expected_second_dac_val_cal_signed = expected_second_dac_val_signed + int(self.dut.cal_val[dac_channel_np1].value.signed_integer)

        expected_abs_dac_val_chN = self.signed_to_abs(expected_first_dac_val_cal_signed)
        expected_abs_dac_val_chNp1 = self.signed_to_abs(expected_second_dac_val_cal_signed)

        # Expected SPI commands
        # Command format is: [23:20] = SPI_CMD_LDAC_WRITE, [19] = 0, [18:16] = channel, [15:0] = dac_value
        expected_spi_word_chN = (self.SPI_CMD_LDAC_WRITE << 20) | (0 << 19) | (dac_channel_n << 16) | self.signed_to_offset(expected_first_dac_val_cal_signed)
        expected_spi_word_chNp1 = (self.SPI_CMD_LDAC_WRITE << 20) | (0 << 19) | (dac_channel_np1 << 16) | self.signed_to_offset(expected_second_dac_val_cal_signed)

        # Log the expected values line by line
        self.dut._log.info(f"[DAC_WR] Expected SPI word chN: 0x{expected_spi_word_chN:06X}, chNp1: 0x{expected_spi_word_chNp1:06X}")
        self.dut._log.info(f"[DAC_WR] Expected first_dac_val_signed: {expected_first_dac_val_signed}, first_dac_val_cal_signed: {expected_first_dac_val_cal_signed}")
        self.dut._log.info(f"[DAC_WR] Expected second_dac_val_signed: {expected_second_dac_val_signed}, second_dac_val_cal_signed: {expected_second_dac_val_cal_signed}")
        self.dut._log.info(f"[DAC_WR] Expected abs_dac_val[{dac_channel_n}]: {expected_abs_dac_val_chN}, abs_dac_val[{dac_channel_np1}]: {expected_abs_dac_val_chNp1}")

        # Sample SPI words sent via MOSI for channel N and N+1
        while True:
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            
            spi_bit_counter = 24
            spi_word = 0

            if int(self.dut.cs_wait_done.value) == 1:
                while spi_bit_counter > 0:
                    await RisingEdge(self.dut.clk)
                    await ReadOnly()
                    mosi = int(self.dut.mosi.value)
                    spi_word = (spi_word << 1) | mosi
                    spi_bit_counter -= 1
                spi_word_list.append(spi_word)
                spi_word = 0

                if len(spi_word_list) == 2:
                    break
        
            if len(spi_word_list) == 2:
                break
        
        if first_dac_val != 0 and second_dac_val != 0:
            # Check that the pair is sent
            assert int(self.dut.dac_spi_cmd_done.value) == 1, \
                f"[DAC_WR] DAC word pair {dac_word_pair_number}: dac_spi_cmd_done should be asserted after SPI words are sent"
            assert int(self.dut.second_dac_channel_of_pair.value) == 1, \
                f"[DAC_WR] DAC word pair {dac_word_pair_number}: second_dac_channel_of_pair should be 1 after sending both words"
            # Check if sent values via MOSI match expected SPI commands and check the conversions

            assert expected_first_dac_val_signed == int(self.dut.first_dac_val_signed.value.signed_integer), \
                f"[DAC_WR] DAC word pair {dac_word_pair_number}: first_dac_val_signed mismatch: expected {expected_first_dac_val_signed}, got {int(self.dut.first_dac_val_signed.value.signed_integer)}"
            assert expected_first_dac_val_cal_signed == int(self.dut.first_dac_val_cal_signed.value.signed_integer), \
                f"[DAC_WR] DAC word pair {dac_word_pair_number}: first_dac_val_cal_signed mismatch: expected {expected_first_dac_val_cal_signed}, got {int(self.dut.first_dac_val_cal_signed.value.signed_integer)}"
            
            assert expected_second_dac_val_signed == int(self.dut.second_dac_val_signed.value.signed_integer), \
                f"[DAC_WR] DAC word pair {dac_word_pair_number}: second_dac_val_signed mismatch: expected {expected_second_dac_val_signed}, got {int(self.dut.second_dac_val_signed.value.signed_integer)}"
            assert expected_second_dac_val_cal_signed == int(self.dut.second_dac_val_cal_signed.value.signed_integer), \
                f"[DAC_WR] DAC word pair {dac_word_pair_number}: second_dac_val_cal_signed mismatch: expected {expected_second_dac_val_cal_signed}, got {int(self.dut.second_dac_val_cal_signed.value.signed_integer)}"
            
            assert spi_word_list[0] == expected_spi_word_chN, \
                f"[DAC_WR] DAC word pair {dac_word_pair_number}: SPI word for ch{dac_channel_n} mismatch: expected 0x{expected_spi_word_chN:06X}, got 0x{spi_word_list[0]:06X}"
            assert spi_word_list[1] == expected_spi_word_chNp1, \
                f"[DAC_WR] DAC word pair {dac_word_pair_number}: SPI word for ch{dac_channel_np1} mismatch: expected 0x{expected_spi_word_chNp1:06X}, got 0x{spi_word_list[1]:06X}"
            
            assert expected_abs_dac_val_chN == int(self.dut.abs_dac_val[dac_channel_n].value), \
                f"[DAC_WR] DAC word pair {dac_word_pair_number}: abs_dac_val[{dac_channel_n}] mismatch: expected {expected_abs_dac_val_chN}, got {int(self.dut.abs_dac_val[dac_channel_n].value)}"
            assert expected_abs_dac_val_chNp1 == int(self.dut.abs_dac_val[dac_channel_np1].value), \
                f"[DAC_WR] DAC word pair {dac_word_pair_number}: abs_dac_val[{dac_channel_np1}] mismatch: expected {expected_abs_dac_val_chNp1}, got {int(self.dut.abs_dac_val[dac_channel_np1].value)}"
            
        else:
            self.dut._log.info(f"[DAC_WR] DAC word pair {dac_word_pair_number}: One or both DAC values are oob. Returning without checking.")
            return

        # If last pair, check that dac_wr_done is asserted
        if dac_word_pair_number == 3:
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            assert int(self.dut.dac_wr_done.value) == 1, \
                f"[DAC_WR] DAC word pair {dac_word_pair_number}: dac_wr_done should be asserted after final DAC word pair is sent"
        return
    
    async def _sb_dac_wr_ch(self, info: dict, i: int):
        """Verify DAC_WR_CH command execution."""
        self.dut._log.info(f"[{i}] DAC_WR_CH: ch={info['ch']} val=0x{info['value']:04X}")

        # At the start of DAC_WR_CH we should be ready to accept the command and next_cmd_state should be S_DAC_WR_CH
        assert int(self.dut.do_next_cmd.value) == 1, \
              f"[{i}] DAC_WR_CH: do_next_cmd should be asserted when starting DAC_WR_CH"
        assert int(self.dut.next_cmd_state.value) == self.STATE_ENCODING['S_DAC_WR_CH'], \
             f"[{i}] DAC_WR_CH: Expected next_cmd_state S_DAC_WR_CH (10), got {self.get_state_name(int(self.dut.next_cmd_state.value))}"
        
        # Next SPI command should be started immediately
        assert int(self.dut.start_spi_cmd.value) == 1, \
              f"[{i}] DAC_WR_CH: start_spi_cmd should be asserted at the start of DAC_WR_CH"
        
        # Calculate expected values of the conversions and SPI command
        expected_dac_val = info['value']
        expected_dac_channel = info['ch']
        
        expected_dac_val_signed = self.offset_to_signed(expected_dac_val)
        expected_dac_val_cal_signed = expected_dac_val_signed + int(self.dut.cal_val[expected_dac_channel].value.signed_integer)
        expected_abs_dac_val = self.signed_to_abs(expected_dac_val_cal_signed)
        # Command format is: [23:20] = SPI_CMD_IMMED_WRITE, [19] = 0, [18:16] = channel, [15:0] = dac_value
        expected_spi_word = (self.SPI_CMD_IMMED_WRITE << 20) | (0 << 19) | (expected_dac_channel << 16) | self.signed_to_offset(expected_dac_val_cal_signed)

        # Log the expected values line by line
        self.dut._log.info(f"[{i}] DAC_WR_CH: Expected SPI word: 0x{expected_spi_word:06X}")
        self.dut._log.info(f"[{i}] DAC_WR_CH: Expected dac_val_signed: {expected_dac_val_signed}, dac_val_cal_signed: {expected_dac_val_cal_signed}")
        self.dut._log.info(f"[{i}] DAC_WR_CH: Expected abs_dac_val[{expected_dac_channel}]: {expected_abs_dac_val}")

        # Sample the SPI word being sent via MOSI
        while True:
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            
            spi_bit_counter = 24
            spi_word = 0
            sampled_flag = 0

            if int(self.dut.cs_wait_done.value) == 1:
                while spi_bit_counter > 0:
                    await RisingEdge(self.dut.clk)
                    await ReadOnly()
                    mosi = int(self.dut.mosi.value)
                    spi_word = (spi_word << 1) | mosi
                    spi_bit_counter -= 1
                sampled_flag = 1
            
            if sampled_flag == 1:
                break

        self.dut._log.info(f"n_cs = {int(self.dut.n_cs.value)}")
        self.dut._log.info(f"running_n_cs_timer = {int(self.dut.running_n_cs_timer.value)}")
        self.dut._log.info(f"spi_bit = {int(self.dut.spi_bit.value)}")
        self.dut._log.info(f"state = {self.get_state_name(int(self.dut.state.value))}")
        
        if expected_dac_val != 0:
            # Check that the command is sent
            assert int(self.dut.dac_spi_cmd_done.value) == 1, \
                f"[{i}] DAC_WR_CH: dac_spi_cmd_done should be asserted after SPI word is sent"
            
            # Check if sent value via MOSI match expected SPI command and check the conversions
            assert expected_dac_val_signed == int(self.dut.first_dac_val_signed.value.signed_integer), \
                f"[{i}] DAC_WR_CH: dac_val_signed mismatch: expected {expected_dac_val_signed}, got {int(self.dut.first_dac_val_signed.value.signed_integer)}"
            assert expected_dac_val_cal_signed == int(self.dut.first_dac_val_cal_signed.value.signed_integer), \
                f"[{i}] DAC_WR_CH: dac_val_cal_signed mismatch: expected {expected_dac_val_cal_signed}, got {int(self.dut.first_dac_val_cal_signed.value.signed_integer)}"
            assert spi_word == expected_spi_word, \
                f"[{i}] DAC_WR_CH: SPI word mismatch: expected 0x{expected_spi_word:06X}, got 0x{spi_word:06X}"
            assert expected_abs_dac_val == int(self.dut.abs_dac_val[expected_dac_channel].value), \
                f"[{i}] DAC_WR_CH: abs_dac_val[{expected_dac_channel}] mismatch: expected {expected_abs_dac_val}, got {int(self.dut.abs_dac_val[expected_dac_channel].value)}"
        else:
            self.dut._log.info(f"[{i}] DAC_WR_CH: DAC value is oob. Returning without checking.")
            return
        
        # Check that dac_wr_done is asserted and cmd_done is asserted
        await RisingEdge(self.dut.clk)
        await ReadOnly()
        assert int(self.dut.dac_wr_done.value) == 1, \
            f"[{i}] DAC_WR_CH: dac_wr_done should be asserted after DAC_WR_CH is complete"
        assert int(self.dut.cmd_done.value) == 1, \
            f"[{i}] DAC_WR_CH: cmd_done should be asserted after DAC_WR_CH is complete"
        
        await RisingEdge(self.dut.clk)
        await ReadOnly()
        # Check that ldac is asserted
        assert int(self.dut.ldac.value) == 1, \
            f"[{i}] DAC_WR_CH: ldac should be asserted after DAC_WR_CH is complete"

        return
    
    async def _sb_get_cal(self, info: dict, i: int):
        """Verify GET_CAL command execution."""
        self.dut._log.info(f"[{i}] GET_CAL: ch={info['ch']}")
        
        # write_cal value should be 1.
        assert int(self.dut.write_cal.value) == 1, \
              f"[{i}] GET_CAL: write_cal should be asserted when starting GET_CAL"
        
        # try_data_write should be asserted
        assert int(self.dut.try_data_write.value) == 1, \
              f"[{i}] GET_CAL: try_data_write should be asserted when starting GET_CAL"
        
        # sample data_buf_full
        data_buf_full = int(self.dut.data_buf_full.value)
        
        await RisingEdge(self.dut.clk)
        await ReadOnly()

        # if data buf wasn't full, data_buf_wr_en should be 1,
        # data_word should be 32-bit : {CAL_DATA, 9'd0, cmd_word[18:16], cal_val[cmd_word[18:16]]} Write calibration value with channel number and debug code
        if data_buf_full == 0:
            assert int(self.dut.data_buf_wr_en.value) == 1, \
              f"[{i}] GET_CAL: data_buf_wr_en should be asserted when data buffer is not full"
            
            # CAL_DATA occupies [31:28], then 9'd0 at [27:19], channel [18:16], cal_val [15:0]
            expected_data_word = (self.CAL_DATA << 28) | (0 << 19) | (info['ch'] << 16) | (int(self.dut.cal_val[info['ch']].value.signed_integer) & 0xFFFF)
            
            assert int(self.dut.data_word.value) == expected_data_word, \
                  f"[{i}] GET_CAL: data_word mismatch: expected 0x{expected_data_word:08X}, got 0x{int(self.dut.data_word.value):08X}"
        else:
            assert int(self.dut.data_buf_wr_en.value) == 0, \
                  f"[{i}] GET_CAL: data_buf_wr_en should not be asserted when data buffer is full"
            
        # state should be S_IDLE after GET_CAL
        assert int(self.dut.state.value) == self.STATE_ENCODING['S_IDLE'], \
              f"[{i}] GET_CAL: Expected state IDLE (6) after GET_CAL, got {self.get_state_name(int(self.dut.state.value))}"
        return
            
    async def _sb_cancel(self, i: int):
        """Verify CANCEL command execution."""
        self.dut._log.info(f"[{i}] CANCEL command received.")
        return
    
    async def _sb_bad_cmd(self, i: int):
        """Verify BAD_CMD command execution."""
        self.dut._log.info(f"[{i}] BAD_CMD detected.")
        # next_cmd_state should be S_ERROR
        assert int(self.dut.next_cmd_state.value) == self.STATE_ENCODING['S_ERROR'], \
              f"[{i}] BAD_CMD: Expected next_cmd_state S_ERROR (15), got {self.get_state_name(int(self.dut.next_cmd_state.value))}"
        # error flag should be asserted
        assert int(self.dut.error.value) == 1, \
              f"[{i}] BAD_CMD: error flag should be asserted"
        
        await RisingEdge(self.dut.clk)
        await ReadOnly()
        # state should be S_ERROR
        assert int(self.dut.state.value) == self.STATE_ENCODING['S_ERROR'], \
              f"[{i}] BAD_CMD: Expected state S_ERROR (15), got {self.get_state_name(int(self.dut.state.value))}"
        return
    
    async def _sb_zero(self, i: int):
        """Verify ZERO command execution."""
        self.dut._log.info(f"[{i}] ZERO command received.")

        # check control flags
        cocotb.start_soon(self._sb_zero_check_control_flags())

        # next_cmd_state should be S_SET_MID
        assert int(self.dut.next_cmd_state.value) == self.STATE_ENCODING['S_SET_MID'], \
              f"[{i}] ZERO: Expected next_cmd_state S_SET_MID (5), got {self.get_state_name(int(self.dut.next_cmd_state.value))}"
        
        # start_spi_cmd should be asserted
        assert int(self.dut.start_spi_cmd.value) == 1, \
              f"[{i}] ZERO: start_spi_cmd should be asserted when starting ZERO"
        
        # Every channel should be set to mid-range value via SPI.
        # The order is channel 0, 1, 2, ..., 7

        # Command for the first two channels are initiated immediately with: 
        # spi_write_cmd(0, 0, cal_midrange[0]), spi_write_cmd(0, 1, cal_midrange[1])

        # Subsequent channels are made with: 
        # spi_write_cmd(0, dac_channel + 1, cal_midrange[dac_channel + 1]), spi_write_cmd(0, dac_channel + 2, cal_midrange[dac_channel + 2])

        # [23:0] spi_write_cmd is: 
        # spi_write_cmd(input ldac_wait, input [2:0] channel, input [15:0] dac_val),
        # where spi_write_cmd = {(ldac_wait ? SPI_CMD_LDAC_WRITE : SPI_CMD_IMMED_WRITE), 1'b0, channel, dac_val}

        # Expected cal_midrange is:
        # cal_midrange[i] = signed_to_offset(cal_val[i])

        # Expected values for each channel
        expected_cal_midrange = []
        expected_spi_word = []
        spi_word_list = []
        for ch in range(8):
            expected_cal_midrange.append(self.signed_to_offset(int(self.dut.cal_val[ch].value.signed_integer)))
            expected_spi_word_i = (self.SPI_CMD_IMMED_WRITE << 20) | (0 << 19) | (ch << 16) | expected_cal_midrange[ch]
            expected_spi_word.append(expected_spi_word_i)
            self.dut._log.info(f"[{i}] ZERO: Expected cal_midrange[{ch}]: 0x{expected_cal_midrange[ch]:04X}, SPI word: 0x{expected_spi_word_i:06X}")

        # Sample SPI words sent via MOSI
        while True:
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            
            spi_bit_counter = 24
            spi_word = 0

            if int(self.dut.cs_wait_done.value) == 1:
                while spi_bit_counter > 0:
                    await RisingEdge(self.dut.clk)
                    await ReadOnly()
                    mosi = int(self.dut.mosi.value)
                    spi_word = (spi_word << 1) | mosi
                    spi_bit_counter -= 1
                spi_word_list.append(spi_word)
                spi_word = 0

                if len(spi_word_list) == 8:
                    break
        
            if len(spi_word_list) == 8:
                break
        
        # Check that the SPI words match expected values
        for ch in range(8):
            assert spi_word_list[ch] == expected_spi_word[ch], \
                f"[{i}] ZERO: SPI word for channel {ch} mismatch: expected 0x{expected_spi_word[ch]:06X}, got 0x{spi_word_list[ch]:06X}"
            
        await RisingEdge(self.dut.clk)
        await ReadOnly()

        # After all channels are set, dac_wr_done should be asserted
        assert int(self.dut.dac_wr_done.value) == 1, \
            f"[{i}] ZERO: dac_wr_done should be asserted after all channels are set to mid-range"
        
        # cmd_done should be asserted
        assert int(self.dut.cmd_done.value) == 1, \
            f"[{i}] ZERO: cmd_done should be asserted after ZERO command is complete"
        
        await RisingEdge(self.dut.clk)
        await ReadOnly()
        # ldac should be asserted
        assert int(self.dut.ldac.value) == 1, \
            f"[{i}] ZERO: ldac should be asserted after ZERO command is complete"
        
        return

    async def _sb_zero_check_control_flags(self):
        await RisingEdge(self.dut.clk)
        await ReadOnly()
        # wait_for_trig should be 1
        assert int(self.dut.wait_for_trig.value) == 1, \
                f"ZERO: wait_for_trig should be 1 during ZERO command execution"
        # expect_next should be 0
        assert int(self.dut.expect_next.value) == 0, \
                f"ZERO: expect_next should be 0 during ZERO command execution"
        # trigger_counter should be 0
        assert int(self.dut.trigger_counter.value) == 0, \
                f"ZERO: trigger_counter should be 0 during ZERO command execution"
        # dac_channel should be 0
        assert int(self.dut.dac_channel.value) == 0, \
                f"ZERO: dac_channel should be 0 during ZERO command execution"
        return

    # ----------------------------
    # Conversion Helper Functions
    # ----------------------------

    # Convert from offset to signed     
    # Given a 16-bit 0-65535 number, treat 32768 (0x8000) as 0, 1 as -32767, and 65535 (0xFFFF) as 32767
    # Note that 0x0000 is considered out of bounds and should be rejected before calling this function
    # 0x0000 will map to 0 instead of -32768 as a failsafe to prevent going to the rails
    def offset_to_signed(self, offset_16):
        val = int(offset_16) & 0xFFFF
        if val == 0:
            return 0
        return val - 0x8000
    
    # Convert signed value to offset (0-65535) representation.
    # Takes a signed 17-bit value (-65536 to 65535) to handle out of bounds (-32767 to 32767)
    # Inverse of offset_to_signed: offset = signed_val + 32768 (0x8000)
    # Should handle out of bounds before calling, but will return DAC_MIDRANGE if out of bounds
    def signed_to_offset(self, signed_16):
        val = signed_16 & 0x1FFFF
        if val < -32767 or val > 32767:
            return self.DAC_MID_RANGE
        return (val + 0x8000) & 0xFFFF

    # Convert the signed value to absolute value
    def signed_to_abs(self, signed_16):
        val = int(signed_16)
        return abs(val)

    # --------------------------------------
    # Transition Monitors
    # --------------------------------------

    async def transition_monitor(self):
        while True:
            await RisingEdge(self.dut.clk)
            # Sample previous values
            prev_state = int(self.dut.state.value)
            prev_resetn = int(self.dut.resetn.value)
            prev_error = int(self.dut.error.value)
            prev_boot_test_skip = int(self.dut.boot_test_skip.value)
            prev_dac_spi_cmd_done = int(self.dut.dac_spi_cmd_done.value)
            prev_n_miso_data_ready_mosi_clk = int(self.dut.n_miso_data_ready_mosi_clk.value)
            prev_boot_readback_match = int(self.dut.boot_readback_match.value)
            prev_cancel_wait = int(self.dut.cancel_wait.value)
            prev_cmd_done = int(self.dut.cmd_done.value)
            prev_next_cmd_state = int(self.dut.next_cmd_state.value)
            prev_dac_wr_done = int(self.dut.dac_wr_done.value)
            prev_wait_for_trig = int(self.dut.wait_for_trig.value)
            prev_delay_timer = int(self.dut.delay_timer.value)
            prev_trigger_counter = int(self.dut.trigger_counter.value)
            prev_do_next_cmd = int(self.dut.do_next_cmd.value)
            prev_command = int(self.dut.command.value)
            prev_cmd_word = int(self.dut.cmd_word.value)
            prev_trigger = int(self.dut.trigger.value)
            prev_delay_wait_done = int(self.dut.delay_wait_done.value)
            prev_ldac_shared = int(self.dut.ldac_shared.value)
            prev_ldac = int(self.dut.ldac.value)
            prev_expect_next = int(self.dut.expect_next.value)
            prev_read_next_dac_val_pair = int(self.dut.read_next_dac_val_pair.value)
            prev_next_cmd_ready = int(self.dut.next_cmd_ready.value)
            prev_try_data_write = int(self.dut.try_data_write.value)
            prev_data_buf_full = int(self.dut.data_buf_full.value)
            prev_unexp_trig = int(self.dut.unexp_trig.value)
            prev_delay_too_short = int(self.dut.delay_too_short.value)
            prev_ldac_misalign = int(self.dut.ldac_misalign.value)
            prev_bad_cmd = int(self.dut.bad_cmd.value)
            prev_cmd_buf_underflow = int(self.dut.cmd_buf_underflow.value)
            prev_data_buf_overflow = int(self.dut.data_buf_overflow.value)
            prev_dac_channel = int(self.dut.dac_channel.value)
            prev_n_cs = int(self.dut.n_cs.value)
            prev_spi_bit = int(self.dut.spi_bit.value)
            prev_last_dac_channel = int(self.dut.last_dac_channel.value)
            prev_second_dac_channel_of_pair = int(self.dut.second_dac_channel_of_pair.value)
            prev_do_ldac = int(self.dut.do_ldac.value)
            prev_n_cs_high_time = int(self.dut.n_cs_high_time.value)
            prev_n_cs_high_time_latched = int(self.dut.n_cs_high_time_latched.value)
            prev_n_cs_timer = int(self.dut.n_cs_timer.value)
            prev_start_spi_cmd = int(self.dut.start_spi_cmd.value)
            prev_cs_wait_done = int(self.dut.cs_wait_done.value)
            await ReadOnly()
            # Sample current values
            current_state = int(self.dut.state.value)
            current_next_cmd_ready = int(self.dut.next_cmd_ready.value)
            current_read_next_dac_val_pair = int(self.dut.read_next_dac_val_pair.value)
            current_cmd_done = int(self.dut.cmd_done.value)
            current_cancel_wait = int(self.dut.cancel_wait.value)
            current_dac_wr_done = int(self.dut.dac_wr_done.value)
            current_command = int(self.dut.command.value)
            current_trigger = int(self.dut.trigger.value)
            current_trigger_counter = int(self.dut.trigger_counter.value)
            current_delay_timer = int(self.dut.delay_timer.value)
            current_wait_for_trig = int(self.dut.wait_for_trig.value)
            current_trig_wait_done = int(self.dut.trig_wait_done.value)
            current_delay_wait_done = int(self.dut.delay_wait_done.value)
            current_do_next_cmd = int(self.dut.do_next_cmd.value)
            current_expect_next = int(self.dut.expect_next.value)
            current_cmd_word = int(self.dut.cmd_word.value)
            current_next_cmd_state = int(self.dut.next_cmd_state.value)
            current_waiting_for_trig = int(self.dut.waiting_for_trig.value)
            current_n_miso_data_ready_mosi_clk = int(self.dut.n_miso_data_ready_mosi_clk.value)
            current_boot_readback_match = int(self.dut.boot_readback_match.value)
            current_ldac_shared = int(self.dut.ldac_shared.value)
            current_ldac = int(self.dut.ldac.value)
            current_try_data_write = int(self.dut.try_data_write.value)
            current_data_buf_full = int(self.dut.data_buf_full.value)
            current_cal_oob = int(self.dut.cal_oob.value)
            current_dac_val_oob = int(self.dut.dac_val_oob.value)
            current_error_out = int(self.dut.error.value)
            current_unexp_trig = int(self.dut.unexp_trig.value)
            current_delay_too_short = int(self.dut.delay_too_short.value)
            current_ldac_misalign = int(self.dut.ldac_misalign.value)
            current_bad_cmd = int(self.dut.bad_cmd.value)
            current_cmd_buf_underflow = int(self.dut.cmd_buf_underflow.value)
            current_data_buf_overflow = int(self.dut.data_buf_overflow.value)
            current_dac_channel = int(self.dut.dac_channel.value)
            current_last_dac_channel = int(self.dut.last_dac_channel.value)
            current_second_dac_channel_of_pair = int(self.dut.second_dac_channel_of_pair.value)
            current_dac_spi_cmd_done = int(self.dut.dac_spi_cmd_done.value)
            current_n_cs = int(self.dut.n_cs.value)
            current_running_n_cs_timer = int(self.dut.running_n_cs_timer.value)
            current_spi_bit = int(self.dut.spi_bit.value)
            current_start_spi_cmd = int(self.dut.start_spi_cmd.value)
            current_n_cs_high_time_latched = int(self.dut.n_cs_high_time_latched.value)
            current_n_cs_timer = int(self.dut.n_cs_timer.value)
            current_cs_wait_done = int(self.dut.cs_wait_done.value)
            current_running_spi_bit = int(self.dut.running_spi_bit.value)

            # check cmd_buf_rd_en
            assert int(self.dut.cmd_buf_rd_en.value) == (
                (current_state != self.STATE_ENCODING['S_ERROR']) and 
                current_next_cmd_ready == 1 and 
                (current_read_next_dac_val_pair == 1 or current_cmd_done == 1 or current_cancel_wait == 1)
            ), f"cmd_buf_rd_en assertion failed: {self.dut.cmd_buf_rd_en.value}, {current_state}, {current_next_cmd_ready}, {current_read_next_dac_val_pair}, {current_cmd_done}, {current_cancel_wait}"

            # check cancel_wait
            assert int(self.dut.cancel_wait.value) == (
                (current_state == self.STATE_ENCODING['S_DELAY'] or current_state == self.STATE_ENCODING['S_TRIG_WAIT'] or (current_state == self.STATE_ENCODING['S_DAC_WR'] and current_dac_wr_done == 1)) and
                current_next_cmd_ready == 1 and
                current_command == self.CMD_ENCODING['CANCEL']
            ), f"cancel_wait assertion failed: {self.dut.cancel_wait.value}, {current_state}, {current_dac_wr_done}, {current_next_cmd_ready}, {current_command}"

            # check trig_wait_done
            expected_trig_wait_done = ((current_trigger == 1 and current_trigger_counter == 1) or current_trigger_counter == 0)
            assert current_trig_wait_done == expected_trig_wait_done, \
                f"trig_wait_done assertion failed: {current_trig_wait_done}, Trig: {current_trigger}, Cnt: {current_trigger_counter}"

            # check delay_wait_done
            assert current_delay_wait_done == (current_delay_timer == 0), \
                f"delay_wait_done assertion failed: {current_delay_wait_done}, Timer: {current_delay_timer}"

            # check cmd_done
            # Helper for the ternary logic: (wait_for_trig ? trig_wait_done : delay_wait_done)
            wait_condition_met = (current_trig_wait_done == 1) if (current_wait_for_trig == 1) else (current_delay_wait_done == 1)

            assert int(self.dut.cmd_done.value) == (
                (current_state == self.STATE_ENCODING['S_IDLE'] and current_next_cmd_ready == 1) or
                (current_state == self.STATE_ENCODING['S_DELAY'] and current_delay_wait_done == 1) or
                (current_state == self.STATE_ENCODING['S_TRIG_WAIT'] and current_trig_wait_done == 1) or
                (current_state == self.STATE_ENCODING['S_DAC_WR'] and current_dac_wr_done == 1 and wait_condition_met) or
                (current_state == self.STATE_ENCODING['S_DAC_WR_CH'] and current_dac_wr_done == 1) or
                (current_state == self.STATE_ENCODING['S_SET_MID'] and current_dac_wr_done == 1)
            ), f"cmd_done assertion failed: {self.dut.cmd_done.value}, State: {current_state}"

            # check do_next_cmd
            assert current_do_next_cmd == (current_cmd_done == 1 and current_next_cmd_ready == 1), \
                f"do_next_cmd assertion failed: {current_do_next_cmd}, Done: {current_cmd_done}, Ready: {current_next_cmd_ready}"

            # check waiting_for_trig
            assert current_waiting_for_trig == (current_state == self.STATE_ENCODING['S_TRIG_WAIT']), \
                f"waiting_for_trig assertion failed: {current_waiting_for_trig}, State: {current_state}"

            # check next_cmd_state
            if current_next_cmd_ready == 0:
                expected_next_state = self.STATE_ENCODING['S_ERROR'] if (current_expect_next == 1) else self.STATE_ENCODING['S_IDLE']
            elif current_command == self.CMD_ENCODING['NO_OP']:
                trig_bit_set = (current_cmd_word >> self.TRIG_BIT) & 1
                expected_next_state = self.STATE_ENCODING['S_TRIG_WAIT'] if trig_bit_set else self.STATE_ENCODING['S_DELAY']
            elif current_command == self.CMD_ENCODING['SET_CAL']:
                expected_next_state = self.STATE_ENCODING['S_IDLE']
            elif current_command == self.CMD_ENCODING['DAC_WR']:
                expected_next_state = self.STATE_ENCODING['S_DAC_WR']
            elif current_command == self.CMD_ENCODING['DAC_WR_CH']:
                expected_next_state = self.STATE_ENCODING['S_DAC_WR_CH']
            elif current_command == self.CMD_ENCODING['CANCEL']:
                expected_next_state = self.STATE_ENCODING['S_IDLE']
            elif current_command == self.CMD_ENCODING['GET_CAL']:
                expected_next_state = self.STATE_ENCODING['S_IDLE']
            elif current_command == self.CMD_ENCODING['ZERO']:
                expected_next_state = self.STATE_ENCODING['S_SET_MID']
            else:
                expected_next_state = self.STATE_ENCODING['S_ERROR']

            assert current_next_cmd_state == expected_next_state, \
                f"next_cmd_state assertion failed: {current_next_cmd_state} != {expected_next_state}. " \
                f"Cmd: {current_command}, Ready: {current_next_cmd_ready}, ExpectNext: {current_expect_next}"
            
            # check state
            # We verify the transition from 'prev_state' to 'current_state' based on 'prev_*' inputs
            if prev_resetn == 0:
                exp_state = self.STATE_ENCODING['S_RESET']
            elif prev_error == 1:
                exp_state = self.STATE_ENCODING['S_ERROR']
            elif prev_state == self.STATE_ENCODING['S_RESET']:
                exp_state = self.STATE_ENCODING['S_IDLE'] if prev_boot_test_skip else self.STATE_ENCODING['S_INIT']
            elif prev_state == self.STATE_ENCODING['S_INIT']:
                exp_state = self.STATE_ENCODING['S_TEST_WR']
            elif prev_state == self.STATE_ENCODING['S_TEST_WR'] and prev_dac_spi_cmd_done:
                exp_state = self.STATE_ENCODING['S_REQ_RD']
            elif prev_state == self.STATE_ENCODING['S_REQ_RD'] and prev_dac_spi_cmd_done:
                exp_state = self.STATE_ENCODING['S_TEST_RD']
            elif prev_state == self.STATE_ENCODING['S_TEST_RD'] and prev_n_miso_data_ready_mosi_clk == 0:
                exp_state = self.STATE_ENCODING['S_SET_MID'] if prev_boot_readback_match else self.STATE_ENCODING['S_ERROR']
            elif prev_cancel_wait:
                exp_state = self.STATE_ENCODING['S_IDLE']
            elif prev_cmd_done:
                exp_state = prev_next_cmd_state
            elif prev_state == self.STATE_ENCODING['S_DAC_WR'] and prev_dac_wr_done:
                exp_state = self.STATE_ENCODING['S_TRIG_WAIT'] if prev_wait_for_trig else self.STATE_ENCODING['S_DELAY']
            else:
                exp_state = prev_state # Hold state
            
            assert current_state == exp_state, \
                f"State transition assertion failed! Prev: {prev_state} -> Curr: {current_state}, Exp: {exp_state}"
            
            # check delay_timer
            trig_bit_set = (prev_cmd_word >> self.TRIG_BIT) & 1
            if prev_resetn == 0 or prev_state == self.STATE_ENCODING['S_ERROR'] or prev_cancel_wait == 1:
                exp_delay = 0
            elif (prev_do_next_cmd == 1 and 
                  (prev_command == self.CMD_ENCODING['DAC_WR'] or prev_command == self.CMD_ENCODING['NO_OP']) and 
                  trig_bit_set == 0):
                exp_delay = prev_cmd_word & 0x1FFFFFF # Keep lower 25 bits
            elif prev_delay_timer > 0:
                exp_delay = prev_delay_timer - 1
            else:
                exp_delay = prev_delay_timer # Hold value
            
            assert current_delay_timer == exp_delay, \
                f"delay_timer assertion failed! Prev: {prev_delay_timer} -> Curr: {current_delay_timer}, Exp: {exp_delay}"
            
            # check trigger_counter 
            if prev_resetn == 0 or prev_state == self.STATE_ENCODING['S_ERROR'] or prev_cancel_wait == 1:
                exp_trig_cnt = 0
            elif (prev_do_next_cmd == 1 and 
                  (prev_command == self.CMD_ENCODING['DAC_WR'] or prev_command == self.CMD_ENCODING['NO_OP']) and 
                  trig_bit_set == 1):
                exp_trig_cnt = prev_cmd_word & 0x1FFFFFF # Keep lower 25 bits
            elif (prev_do_next_cmd == 1 and (prev_command == self.CMD_ENCODING['DAC_WR_CH'] or prev_command == self.CMD_ENCODING['ZERO'])):
                exp_trig_cnt = 0
            elif prev_trigger_counter > 0 and prev_trigger == 1:
                exp_trig_cnt = prev_trigger_counter - 1
            else:
                exp_trig_cnt = prev_trigger_counter

            assert current_trigger_counter == exp_trig_cnt, \
                f"trigger_counter assertion failed! Prev: {prev_trigger_counter} -> Curr: {current_trigger_counter}, Exp: {exp_trig_cnt}"
            
             # check error flag
            cond_boot_fail = (current_state == self.STATE_ENCODING['S_TEST_RD'] and current_n_miso_data_ready_mosi_clk == 0 and current_boot_readback_match == 0)
            cond_unexpected_trigger = (current_state != self.STATE_ENCODING['S_TRIG_WAIT'] and current_trigger == 1 and current_trigger_counter <= 1)
            cond_delay_short = (current_state == self.STATE_ENCODING['S_DAC_WR'] and current_dac_wr_done == 0 and current_wait_for_trig == 0 and current_delay_wait_done == 1)
            cond_ldac_misalign = ((current_state == self.STATE_ENCODING['S_DAC_WR'] or current_state == self.STATE_ENCODING['S_DAC_WR_CH']) and current_ldac_shared == 1 and current_ldac == 0)
            cond_bad_cmd = (current_do_next_cmd == 1 and current_next_cmd_state == self.STATE_ENCODING['S_ERROR'])
            cond_underflow = (((current_cmd_done == 1 and current_expect_next == 1) or current_read_next_dac_val_pair == 1) and current_next_cmd_ready == 0)
            cond_overflow = (current_try_data_write == 1 and current_data_buf_full == 1)
            
            expected_error = (cond_boot_fail or cond_unexpected_trigger or cond_delay_short or cond_ldac_misalign or 
                              cond_bad_cmd or cond_underflow or cond_overflow or current_cal_oob == 1 or current_dac_val_oob == 1)

            assert current_error_out == expected_error, \
                f"error signal assertion failed! Expected {expected_error}, Got {current_error_out}. Details: Boot:{cond_boot_fail} Trig:{cond_unexpected_trigger} Delay:{cond_delay_short} LDAC:{cond_ldac_misalign} Cmd:{cond_bad_cmd} Under:{cond_underflow} Over:{cond_overflow}"
            
            # If we have an error, log the details
            if current_error_out == 1:
                error_reasons = []
                if cond_boot_fail:
                    error_reasons.append("Boot Readback Mismatch")
                if cond_unexpected_trigger:
                    error_reasons.append("Unexpected Trigger")
                if cond_delay_short:
                    error_reasons.append("Delay Too Short")
                if cond_ldac_misalign:
                    error_reasons.append("LDAC Misalignment")
                if cond_bad_cmd:
                    error_reasons.append("Bad Command")
                if cond_underflow:
                    error_reasons.append("Command Buffer Underflow")
                if cond_overflow:
                    error_reasons.append("Data Buffer Overflow")
                if current_cal_oob == 1:
                    error_reasons.append("Calibration Value Out of Bounds")
                if current_dac_val_oob == 1:
                    error_reasons.append("DAC Value Out of Bounds")
                self.dut._log.warning(f"Error detected at state {self.get_state_name(current_state)}: " + ", ".join(error_reasons))

            # check unexp_trig
            if prev_resetn == 0:
                exp_unexp_trig = 0
            elif prev_state != self.STATE_ENCODING['S_TRIG_WAIT'] and prev_trigger == 1 and prev_trigger_counter <= 1:
                exp_unexp_trig = 1
            else:
                exp_unexp_trig = prev_unexp_trig
            
            assert current_unexp_trig == exp_unexp_trig, \
                f"unexp_trig assertion failed! Prev: {prev_unexp_trig} -> Curr: {current_unexp_trig}, Exp: {exp_unexp_trig}"
            
            # Log the unexpected trigger event since the core has no failsafe for it and it might occur for DAC_WR command.
            if current_unexp_trig == 1:
                self.dut._log.warning(f"Unexpected trigger event detected at state {self.get_state_name(prev_state)}!")

            # check delay_too_short
            if prev_resetn == 0:
                exp_delay_too_short = 0
            elif prev_state == self.STATE_ENCODING['S_DAC_WR'] and prev_dac_wr_done == 0 and prev_wait_for_trig == 0 and prev_delay_wait_done == 1:
                exp_delay_too_short = 1
            else:
                exp_delay_too_short = prev_delay_too_short
            
            assert current_delay_too_short == exp_delay_too_short, \
                f"delay_too_short assertion failed! Prev: {prev_delay_too_short} -> Curr: {current_delay_too_short}, Exp: {exp_delay_too_short}"
            
            # Log the delay too short event since it might also happen for DAC_WR command.
            if current_delay_too_short == 1:
                self.dut._log.warning(f"Delay too short event detected at state {self.get_state_name(prev_state)}!")

            # check ldac_misalign 
            if prev_resetn == 0:
                exp_ldac_misalign = 0
            elif (prev_state == self.STATE_ENCODING['S_DAC_WR'] or prev_state == self.STATE_ENCODING['S_DAC_WR_CH']) and prev_ldac_shared == 1 and prev_ldac == 0:
                exp_ldac_misalign = 1
            else:
                exp_ldac_misalign = prev_ldac_misalign
            
            assert current_ldac_misalign == exp_ldac_misalign, \
                f"ldac_misalign assertion failed! Prev: {prev_ldac_misalign} -> Curr: {current_ldac_misalign}, Exp: {exp_ldac_misalign}"

            # check bad_cmd
            if prev_resetn == 0:
                exp_bad_cmd = 0
            elif prev_do_next_cmd == 1 and prev_next_cmd_state == self.STATE_ENCODING['S_ERROR']:
                exp_bad_cmd = 1
            else:
                exp_bad_cmd = prev_bad_cmd
            
            assert current_bad_cmd == exp_bad_cmd, \
                f"bad_cmd assertion failed! Prev: {prev_bad_cmd} -> Curr: {current_bad_cmd}, Exp: {exp_bad_cmd}"

            # check cmd_buf_underflow
            if prev_resetn == 0:
                exp_cmd_buf_underflow = 0
            elif ((prev_cmd_done == 1 and prev_expect_next == 1) or prev_read_next_dac_val_pair == 1) and prev_next_cmd_ready == 0:
                exp_cmd_buf_underflow = 1
            else:
                exp_cmd_buf_underflow = prev_cmd_buf_underflow
            
            assert current_cmd_buf_underflow == exp_cmd_buf_underflow, \
                f"cmd_buf_underflow assertion failed! Prev: {prev_cmd_buf_underflow} -> Curr: {current_cmd_buf_underflow}, Exp: {exp_cmd_buf_underflow}"

            # check data_buf_overflow
            if prev_resetn == 0:
                exp_data_buf_overflow = 0
            elif prev_try_data_write == 1 and prev_data_buf_full == 1:
                exp_data_buf_overflow = 1
            else:
                exp_data_buf_overflow = prev_data_buf_overflow
            
            assert current_data_buf_overflow == exp_data_buf_overflow, \
                f"data_buf_overflow assertion failed! Prev: {prev_data_buf_overflow} -> Curr: {current_data_buf_overflow}, Exp: {exp_data_buf_overflow}"
            
            # check last_dac_channel
            assert current_last_dac_channel == (current_dac_channel == 7), \
                f"last_dac_channel assertion failed: {current_last_dac_channel}, Ch: {current_dac_channel}"

            # check second_dac_channel_of_pair
            # Even channel is when the least significant bit is set (bit 0 is 1)
            assert current_second_dac_channel_of_pair == ((current_dac_channel & 1) == 1), \
                f"second_dac_channel_of_pair assertion failed: {current_second_dac_channel_of_pair}, Ch: {current_dac_channel}"

            # check dac_spi_cmd_done
            spi_cmd_states = [self.STATE_ENCODING['S_DAC_WR'], self.STATE_ENCODING['S_DAC_WR_CH'], self.STATE_ENCODING['S_TEST_WR'], self.STATE_ENCODING['S_REQ_RD'], self.STATE_ENCODING['S_TEST_RD'], self.STATE_ENCODING['S_SET_MID']]
            exp_dac_spi_cmd_done = (
                (current_state in spi_cmd_states) and
                current_n_cs == 0 and
                current_running_n_cs_timer == 0 and
                current_spi_bit == 0
            )
            assert current_dac_spi_cmd_done == exp_dac_spi_cmd_done, \
                f"dac_spi_cmd_done assertion failed: {current_dac_spi_cmd_done} != {exp_dac_spi_cmd_done}. State: {current_state}, CS: {current_n_cs}, Timer: {current_running_n_cs_timer}, Bit: {current_spi_bit}"

            # check read_next_dac_val_pair
            if prev_resetn == 0 or prev_state == self.STATE_ENCODING['S_ERROR']:
                exp_read_next = 0
            elif prev_do_next_cmd == 1 and prev_command == self.CMD_ENCODING['DAC_WR']:
                exp_read_next = 1
            elif (prev_state == self.STATE_ENCODING['S_DAC_WR'] and 
                  prev_dac_spi_cmd_done == 1 and 
                  prev_second_dac_channel_of_pair == 1 and 
                  prev_last_dac_channel == 0):
                exp_read_next = 1
            else:
                exp_read_next = 0
            
            assert current_read_next_dac_val_pair == exp_read_next, \
                f"read_next_dac_val_pair assertion failed! Prev: {current_read_next_dac_val_pair}, Exp: {exp_read_next}"

            # check dac_wr_done
            if prev_resetn == 0 or prev_state == self.STATE_ENCODING['S_ERROR']:
                exp_dac_wr_done = 0
            elif (prev_state == self.STATE_ENCODING['S_DAC_WR'] or prev_state == self.STATE_ENCODING['S_SET_MID']) and prev_dac_spi_cmd_done == 1 and prev_last_dac_channel == 1:
                exp_dac_wr_done = 1
            elif prev_state == self.STATE_ENCODING['S_DAC_WR_CH'] and prev_dac_spi_cmd_done == 1:
                exp_dac_wr_done = 1
            else:
                exp_dac_wr_done = 0
                
            assert current_dac_wr_done == exp_dac_wr_done, \
                f"dac_wr_done assertion failed! Prev: {current_dac_wr_done}, Exp: {exp_dac_wr_done}"

            # check dac_channel
            if prev_resetn == 0 or prev_state == self.STATE_ENCODING['S_ERROR']:
                exp_dac_channel = 0
            elif prev_do_next_cmd == 1 and prev_command == self.CMD_ENCODING['DAC_WR']:
                exp_dac_channel = 0
            elif prev_do_next_cmd == 1 and prev_command == self.CMD_ENCODING['ZERO']:
                exp_dac_channel = 0
            elif prev_do_next_cmd == 1 and prev_command == self.CMD_ENCODING['DAC_WR_CH']:
                exp_dac_channel = (prev_cmd_word >> 16) & 0x7 # bits 18:16
            elif (prev_state == self.STATE_ENCODING['S_DAC_WR'] or prev_state == self.STATE_ENCODING['S_SET_MID']) and prev_dac_spi_cmd_done == 1:
                exp_dac_channel = (prev_dac_channel + 1) & 0x7 # 3-bit wrap
            else:
                exp_dac_channel = prev_dac_channel
            
            assert current_dac_channel == exp_dac_channel, \
                f"dac_channel assertion failed! Prev: {prev_dac_channel} -> Curr: {current_dac_channel}, Exp: {exp_dac_channel}"
            
            # check ldac
            if prev_resetn == 0 or prev_state == self.STATE_ENCODING['S_ERROR']:
                exp_ldac = 0
            elif prev_do_ldac == 1 and prev_cmd_done == 1 and prev_cancel_wait == 0:
                exp_ldac = 1
            else:
                exp_ldac = 0
            
            assert current_ldac == exp_ldac, \
                f"ldac assertion failed! Prev: {current_ldac}, Exp: {exp_ldac}"

            # check start_spi_cmd
            cond1 = (current_do_next_cmd == 1 and (current_command == self.CMD_ENCODING['DAC_WR'] or current_command == self.CMD_ENCODING['DAC_WR_CH'] or current_command == self.CMD_ENCODING['ZERO']))
            cond2 = (current_state == self.STATE_ENCODING['S_INIT'])
            cond3 = ((current_state == self.STATE_ENCODING['S_TEST_WR'] or current_state == self.STATE_ENCODING['S_REQ_RD'] or current_state == self.STATE_ENCODING['S_TEST_RD']) and current_dac_spi_cmd_done == 1)
            cond4 = ((current_state == self.STATE_ENCODING['S_SET_MID'] or current_state == self.STATE_ENCODING['S_DAC_WR']) and current_dac_spi_cmd_done == 1 and current_last_dac_channel == 0)
            exp_start_spi_cmd = (cond1 or cond2 or cond3 or cond4)
            
            assert current_start_spi_cmd == exp_start_spi_cmd, \
                f"start_spi_cmd assertion failed! {current_start_spi_cmd} != {exp_start_spi_cmd}"

            # check n_cs_high_time_latched
            if prev_resetn == 0:
                exp_n_cs_high_time_latched = 0
            elif prev_state == self.STATE_ENCODING['S_RESET']:
                exp_n_cs_high_time_latched = prev_n_cs_high_time
            else:
                exp_n_cs_high_time_latched = prev_n_cs_high_time_latched
                
            assert current_n_cs_high_time_latched == exp_n_cs_high_time_latched, \
                f"n_cs_high_time_latched assertion failed! {current_n_cs_high_time_latched} != {exp_n_cs_high_time_latched}"

            # check n_cs_timer
            if prev_resetn == 0 or prev_state == self.STATE_ENCODING['S_ERROR']:
                exp_n_cs_timer = 0
            elif prev_start_spi_cmd:
                exp_n_cs_timer = prev_n_cs_high_time_latched
            elif prev_n_cs_timer > 0:
                exp_n_cs_timer = prev_n_cs_timer - 1
            else:
                exp_n_cs_timer = prev_n_cs_timer
                
            assert current_n_cs_timer == exp_n_cs_timer, \
                f"n_cs_timer assertion failed! Prev: {prev_n_cs_timer}, Exp: {exp_n_cs_timer}"

            # check running_n_cs_timer
            # Logic: running_n_cs_timer <= (n_cs_timer > 0);
            exp_running_n_cs_timer = (prev_n_cs_timer > 0)
            assert current_running_n_cs_timer == exp_running_n_cs_timer, \
                f"running_n_cs_timer assertion failed! {current_running_n_cs_timer} != {exp_running_n_cs_timer} (Prev Timer: {prev_n_cs_timer})"

            # check cs_wait_done
            assert current_cs_wait_done == (current_running_n_cs_timer == 1 and current_n_cs_timer == 0), \
                f"cs_wait_done assertion failed! {current_cs_wait_done}"

            # check n_cs
            if prev_resetn == 0 or prev_state == self.STATE_ENCODING['S_ERROR']:
                exp_n_cs = 1
            elif prev_cs_wait_done:
                exp_n_cs = 0
            elif prev_dac_spi_cmd_done or prev_state == self.STATE_ENCODING['S_IDLE']:
                exp_n_cs = 1
            else:
                exp_n_cs = prev_n_cs
            
            assert current_n_cs == exp_n_cs, \
                f"n_cs assertion failed! Prev: {prev_n_cs} -> Curr: {current_n_cs}, Exp: {exp_n_cs}"

            # check spi_bit
            if prev_resetn == 0 or prev_state == self.STATE_ENCODING['S_ERROR']:
                exp_spi_bit = 0
            elif prev_spi_bit > 0:
                exp_spi_bit = prev_spi_bit - 1
            elif prev_cs_wait_done:
                exp_spi_bit = 23
            else:
                exp_spi_bit = prev_spi_bit
            
            assert current_spi_bit == exp_spi_bit, \
                f"spi_bit assertion failed! Prev: {prev_spi_bit} -> Curr: {current_spi_bit}, Exp: {exp_spi_bit}"

            # check running_spi_bit
            # Logic: running_spi_bit <= (spi_bit > 0);
            exp_running_spi_bit = (prev_spi_bit > 0)
            assert current_running_spi_bit == exp_running_spi_bit, \
                f"running_spi_bit assertion failed! {current_running_spi_bit} != {exp_running_spi_bit}"

    # --------------------------------------
    # Wrappers for simulating the DUT.
    # Does not test functionality.
    # Use it to observe waveforms and command execution.
    # Command builders and decoders can be used to create a list of commands and log them.
    # --------------------------------------

    async def simulate_dut(self, cmd_word_list, clk_cycles=1000):
        """Simulate the DUT by sending a list of commands and observing the outputs for a certain number of clock cycles after."""

        # Both cmd_buf and executing_cmd_queue are initialized in __init__ and live inside the class.
        # Reset will also clear both the cmd_buf and executing_cmd_queue to make sure we start fresh.
        await self.reset()
        
        # command_buf_model is an interface between the DUT and the fwft fifo model called cmd_buf.
        # It is connected to the DUT's command buffer interface and, 
        # it will consume commands from cmd_buf and provide the appropriate handshakes so that DUT can read commands from it.
        # The commands read by DUT are stored in self.executing_cmd_queue for reference.
        command_buf_task = cocotb.start_soon(self.command_buf_model())

        # send_commands will try to write the commands in cmd_word_list into the cmd_buf until all commands are written.
        # If cmd_buf is full, it will try to write the same command in the next DUT clock cycle.
        send_commands_task = cocotb.start_soon(self.send_commands(cmd_word_list))

        # Log the executing commands read by DUT in readable format.
        command_logger_task = cocotb.start_soon(self.log_executing_commands(len(cmd_word_list)))
        
        await send_commands_task

        # After we are done sending commands, wait for a certain number of clock cycles to observe the outputs and waveforms.
        # Waveforms are dumped to /results/dump.vcd
        for _ in range(clk_cycles):
            await RisingEdge(self.dut.clk)

        # End tasks for safety
        command_buf_task.kill() # This lives for an indefinite amount of time and should be killed. Shouldn't be awaited.
        command_logger_task.kill() # This might live for an indefinite amount of time and should be killed. Shouldn't be awaited.
        send_commands_task.kill() # kill for safety.
        return

    async def log_executing_commands(self, num_of_commands):
        if num_of_commands == 0:
            self.dut._log.info("All executing commands are logged: 0 commands expected.")
            return
        logged = 0
        while logged < num_of_commands:
            # Wait until DUT pops one
            await RisingEdge(self.dut.clk)
            await ReadOnly()

            if len(self.executing_cmd_queue) == 0:
                continue

            popped_cmd_word = self.executing_cmd_queue.popleft()
            decoded = self.decode_cmd(popped_cmd_word)
            self.dut._log.info(f"Executing command logged: 0x{popped_cmd_word:08X}, {decoded}, Logged: {logged+1}/{num_of_commands}")
            logged += 1
        return


