import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, ReadWrite, Combine
from collections import deque

from distro import info
from fwft_fifo_model import fwft_fifo_model
import random

class shim_ads816x_adc_ctrl_base:

    # ---------------------------
    # Encodings
    # ---------------------------
    
    CMD_ENCODING = {
        'NO_OP'   : 0,  # 3'd0: Delay or trigger wait
        'SET_ORD' : 1,  # 3'd1: Set sample order
        'ADC_RD'  : 2,  # 3'd2: Read ADC samples sequence
        'ADC_RD_CH': 3, # 3'd3: Read specific ADC channel
        'CANCEL'  : 7   # 3'd7: Cancel current wait/delay
    }

    STATE_ENCODING = {
        'S_RESET'    : 0,
        'S_INIT'     : 1,
        'S_TEST_WR'  : 2,
        'S_REQ_RD'   : 3,
        'S_TEST_RD'  : 4,
        'S_IDLE'     : 5,
        'S_DELAY'    : 6,
        'S_TRIG_WAIT': 7,
        'S_ADC_RD'   : 8,
        'S_ADC_RD_CH': 9,
        'S_ERROR'    : 15
    }

    DEBUG_ENCODING = {
        'DBG_MISO_DATA'       : 1,
        'DBG_STATE_TRANSITION': 2,
        'DBG_REPEAT_BIT'      : 3,
        'DBG_N_CS_TIMER'      : 4,
        'DBG_SPI_BIT'         : 5,
        'DBG_CMD_DONE'        : 6
    }

    # ---------------------------
    # Initialization
    # ---------------------------

    def __init__ (self, dut, clk_period=4, miso_sck_period=4, time_unit='ns', cmd_buf_depth=16, data_buf_depth=16):
        self.dut = dut
        self.clk_period = clk_period
        self.miso_sck_period = miso_sck_period
        self.time_unit = time_unit

        # Parameters
        # SPI command bit width
        self.OTF_CMD_BITS = 16
        self.N_CS_MISO_START_TIME = 2
        # SPI command opcodes and register addresses
        self.SPI_CMD_REG_WRITE = 0b00001
        self.SPI_CMD_REG_READ  = 0b00010
        self.ADDR_OTF_CFG      = 0x2A
        self.SET_OTF_CFG_DATA  = 0x01
        # Command bit positions
        self.TRIG_BIT = 28
        self.CONT_BIT = 27
        self.REPEAT_BIT = 26

        # Initialize clocks
        cocotb.start_soon(Clock(dut.clk, clk_period, time_unit).start(start_high=False))
        cocotb.start_soon(Clock(dut.miso_sck, miso_sck_period, time_unit).start(start_high=False))

        # Initialize input signals
        self.dut.boot_test_skip.value = 1 # Skip boot test by default
        self.dut.debug.value = 0 # No debug by default
        self.dut.n_cs_high_time.value = 8 # Default n_cs high time
        self.dut.cmd_buf_word.value = 0
        self.dut.cmd_buf_empty.value = 1
        self.dut.data_buf_full.value = 0
        self.dut.trigger.value = 0
        self.dut.miso.value = 0

        # Interface FIFOs
        self.cmd_buf_depth = cmd_buf_depth
        self.data_buf_depth = data_buf_depth
        self.cmd_buf = fwft_fifo_model(dut, "CMD_FIFO_MODEL", DEPTH=cmd_buf_depth)
        self.data_buf = fwft_fifo_model(dut, "DATA_BUF_MODEL", DEPTH=data_buf_depth)

        # Queue to keep track of current executing command in the DUT
        self.executing_cmd_queue = deque()

        # Repeating global flags
        self.adc_rd_ch_repeating_ch = None

        # Delay and trig global flags
        self.delay_count = None
        self.trig_count = None

    # ---------------------------
    # Helpers
    # ---------------------------

    def get_cmd_name(self, cmd_value):
        for cmd_name, cmd_num in self.CMD_ENCODING.items():
            if cmd_num == cmd_value:
                return cmd_name
        return f"UNKNOWN_CMD_{cmd_value}"

    def get_state_name(self, state_value):
        for state_name, state_num in self.STATE_ENCODING.items():
            if state_num == state_value:
                return state_name
        return f"UNKNOWN_STATE_{state_value}"

    def get_debug_name(self, debug_value):
        for debug_name, debug_num in self.DEBUG_ENCODING.items():
            if debug_num == debug_value:
                return debug_name
        return f"UNKNOWN_DEBUG_{debug_value}"

    # ---------------------------
    # Resets
    # ---------------------------
    async def reset(self):
        """Reset the DUT and hold it for two clock cycles."""
        await RisingEdge(self.dut.clk)
        self.dut.resetn.value = 0
        self.dut._log.info("STARTING DUT RESET")
        self.cmd_buf.reset()
        self.data_buf.reset()
        self.executing_cmd_queue.clear()
        self.adc_rd_ch_repeating_ch = None
        self.delay_count = None
        self.trig_count = None
        await RisingEdge(self.dut.clk)
        await RisingEdge(self.dut.clk)
        self.dut.resetn.value = 1
        self.dut._log.info("DUT RESET COMPLETE")

    async def miso_reset(self):
        """Reset the MISO interface and hold it for two miso_sck cycles."""
        await RisingEdge(self.dut.miso_sck)
        self.dut.miso_resetn.value = 0
        self.dut._log.info("STARTING MISO INTERFACE RESET")
        await RisingEdge(self.dut.miso_sck)
        await RisingEdge(self.dut.miso_sck)
        self.dut.miso_resetn.value = 1
        self.dut._log.info("MISO INTERFACE RESET COMPLETE")

    async def reset_both_domains(self):
        """Reset both the DUT and MISO interface domains."""
        forked = []
        forked.append(cocotb.start_soon(self.reset()))
        forked.append(cocotb.start_soon(self.miso_reset()))
        await Combine(*forked)
        
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
    # Data buffer model
    # ---------------------------

    async def data_buf_model(self):
        """
        Model of the data FIFO. Connected to DUT's data_buf_wr_en, data_word, data_buf_full.
        The DUT will write ADC data to this FIFO.
        """
        while True:
            await RisingEdge(self.dut.clk)
            await ReadWrite() # Wait for combinational logic to settle

            # Buffer writes
            if self.dut.data_buf_wr_en.value == 1:
                data = int(self.dut.data_word.value)
                self.data_buf.write_item(data)

            # Update Buffer status signals
            self.dut.data_buf_full.value = 1 if self.data_buf.is_full() else 0

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
                # If buffer not full, write and move to the next command word.
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

    def build_noop(self, *, trig_wait: int, cont: int, value: int) -> int:
        """
        NO_OP: [31:29]=0
        [28]=TRIG_WAIT, [27]=CONT, [24:0]=Value (Delay or Trigger Count)
        """
        cmd_word = (self.CMD_ENCODING['NO_OP'] & 0x7) << 29
        cmd_word |= (1 if trig_wait else 0) << self.TRIG_BIT
        cmd_word |= (1 if cont else 0) << self.CONT_BIT
        cmd_word |= (value & 0x1FFFFFF)
        return cmd_word
    
    def build_set_ord(self, channels: list) -> int:
        """
        SET_ORD: [31:29]=1
        [23:21]=Ch7 ... [2:0]=Ch0
        Expects a list of 8 integers (0-7).
        """
        if len(channels) != 8:
            raise ValueError("SET_ORD requires exactly 8 channel indices.")
        
        cmd_word = (self.CMD_ENCODING['SET_ORD'] & 0x7) << 29
        for i, ch in enumerate(channels):
            cmd_word |= ((ch & 0x7) << (i * 3))
        return cmd_word
    
    def build_adc_rd(self, *, trig_wait: int, cont: int, repeat: int, value: int) -> int:
        """
        ADC_RD: [31:29]=2
        [28]=TRIG_WAIT, [27]=CONT, [26]=REPEAT, [24:0]=Value (Delay or Trigger Count)
        """
        cmd_word = (self.CMD_ENCODING['ADC_RD'] & 0x7) << 29
        cmd_word |= (1 if trig_wait else 0) << self.TRIG_BIT
        cmd_word |= (1 if cont else 0) << self.CONT_BIT
        cmd_word |= (1 if repeat else 0) << self.REPEAT_BIT
        cmd_word |= (value & 0x1FFFFFF)
        return cmd_word
    
    def build_adc_rd_ch(self, *, repeat: int, ch: int) -> int:
        """
        ADC_RD_CH: [31:29]=3
        [26]=REPEAT, [2:0]=Channel
        """
        cmd_word = (self.CMD_ENCODING['ADC_RD_CH'] & 0x7) << 29
        cmd_word |= (1 if repeat else 0) << self.REPEAT_BIT
        cmd_word |= (ch & 0x7)
        return cmd_word
    
    def build_cancel(self) -> int:
        """
        CANCEL: [31:29]=7
        """
        return (self.CMD_ENCODING['CANCEL'] & 0x7) << 29
    
    def build_repeat_count(self, count: int) -> int:
        """
        Build a 32-bit repeat count word for repeating commands.
        """
        return count & 0xFFFFFFFF
    
    def decode_cmd(self, cmd_word: int) -> dict:
        """Decode a raw command word into a structured dict."""
        cmd_value = (cmd_word >> 29) & 0x7
        info = {"raw": cmd_word, "cmd": cmd_value, "name": self.get_cmd_name(cmd_value)}

        if cmd_value == self.CMD_ENCODING['NO_OP']:
            info.update({
                "trig": (cmd_word >> self.TRIG_BIT) & 1,
                "cont": (cmd_word >> self.CONT_BIT) & 1,
                "value": cmd_word & 0x1FFFFFF,
            })
        elif cmd_value == self.CMD_ENCODING['SET_ORD']:
            channels = []
            for i in range(8):
                channels.append((cmd_word >> (i * 3)) & 0x7)
            info.update({"channels": channels})
        elif cmd_value == self.CMD_ENCODING['ADC_RD']:
            info.update({
                "trig": (cmd_word >> self.TRIG_BIT) & 1,
                "cont": (cmd_word >> self.CONT_BIT) & 1,
                "repeat": (cmd_word >> self.REPEAT_BIT) & 1,
                "value": cmd_word & 0x1FFFFFF,
            })
        elif cmd_value == self.CMD_ENCODING['ADC_RD_CH']:
            info.update({
                "repeat": (cmd_word >> self.REPEAT_BIT) & 1,
                "ch": cmd_word & 0x7,
            })
        elif cmd_value == self.CMD_ENCODING['CANCEL']:
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
        and run per-command scoreboards.
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
            popped_cmd_value = (popped_cmd_word >> 29) & 0x7
            
            # Verify the DUT's internal wire mirrors the popped command (sanity check)
            dut_cmd_value = int(self.dut.command.value)
            dut_cmd_word = int(self.dut.cmd_word.value)
            
            assert popped_cmd_value == dut_cmd_value, f"Cmd type mismatch: expected {self.get_cmd_name(popped_cmd_value)} got {self.get_cmd_name(dut_cmd_value)}"
            assert popped_cmd_word == dut_cmd_word, f"Cmd word mismatch: expected 0x{popped_cmd_word:08X} got 0x{dut_cmd_word:08X}"

            decoded = self.decode_cmd(popped_cmd_word)
            idx = processed
            processed += 1

            if decoded["cmd"] == self.CMD_ENCODING['NO_OP']:
                forked.append(cocotb.start_soon(self._sb_noop(decoded, idx)))

            elif decoded["cmd"] == self.CMD_ENCODING['SET_ORD']:
                forked.append(cocotb.start_soon(self._sb_set_ord(decoded, idx)))

            elif decoded["cmd"] == self.CMD_ENCODING['ADC_RD']:
                forked.append(cocotb.start_soon(self._sb_adc_rd(decoded, idx)))
                # If the command is repeating, start a separate scoreboard for the repeats with repeating count as an argument from the cmd_buf
                if decoded["repeat"] == 1:
                    while True:
                        await RisingEdge(self.dut.clk)
                        await ReadOnly()
                        if len(self.executing_cmd_queue) > 0:
                            break
                    # Sanity check
                    expected_repeat_count_word = self.executing_cmd_queue.popleft()
                    repeat_count_word = int(self.dut.cmd_buf_word.value)
                    assert repeat_count_word == expected_repeat_count_word, f"Repeat count mismatch: expected {expected_repeat_count_word} got {repeat_count_word}"
                    # Fork the repeating scoreboard
                    processed += 1
                    forked.append(cocotb.start_soon(self._sb_adc_rd_repeating(repeat_count_word)))

            elif decoded["cmd"] == self.CMD_ENCODING['ADC_RD_CH']:
                forked.append(cocotb.start_soon(self._sb_adc_rd_ch(decoded, idx)))
                # If the command is repeating, start a separate scoreboard for the repeats with repeating count as an argument from the cmd_buf
                if decoded["repeat"] == 1:
                    while True:
                        await RisingEdge(self.dut.clk)
                        await ReadOnly()
                        if len(self.executing_cmd_queue) > 0:
                            break
                    # Sanity check
                    expected_repeat_count_word = self.executing_cmd_queue.popleft()
                    repeat_count_word = int(self.dut.cmd_buf_word.value)
                    assert repeat_count_word == expected_repeat_count_word, f"Repeat count mismatch: expected {expected_repeat_count_word} got {repeat_count_word}"
                    # Fork the repeating scoreboard
                    processed += 1
                    forked.append(cocotb.start_soon(self._sb_adc_rd_ch_repeating(repeat_count_word)))

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
        self.dut._log.info(f"[{i}] NO_OP: trig={info['trig']} cont={info['cont']} val={info['value']}")

        # do_next_cmd should be asserted
        assert int(self.dut.do_next_cmd.value) == 1, \
            f"[{i}] for NO_OP command do_next_cmd was not asserted"
        # Depending on the TRIG_BIT, next_cmd_state should be either S_TRIG_WAIT or S_DELAY
        expected_next_state = self.STATE_ENCODING['S_TRIG_WAIT'] if info['trig'] == 1 else self.STATE_ENCODING['S_DELAY']
        assert int(self.dut.next_cmd_state.value) == expected_next_state, \
            f"[{i}] NO_OP command did not transition to expected state, got {self.get_state_name(int(self.dut.next_cmd_state.value))}"
        
        await RisingEdge(self.dut.clk)
        await ReadOnly()

        # Return if cancel_wait was issued
        if int(self.dut.cancel_wait.value) == 1:
            self.dut._log.info(f"[{i}] NO_OP command was cancelled via CANCEL command, exiting NO_OP scoreboard.")
            return
        
        # Expected wait_for_trig and expect_next values should be set according to command bits
        expected_wait_for_trig = info['trig']
        expected_expect_next = info['cont']

        assert int(self.dut.wait_for_trig.value) == expected_wait_for_trig, \
            f"[{i}] NO_OP command wait_for_trig mismatch: expected {expected_wait_for_trig} got {int(self.dut.wait_for_trig.value)}"

        assert int(self.dut.expect_next.value) == expected_expect_next, \
            f"[{i}] NO_OP command expect_next mismatch: expected {expected_expect_next} got {int(self.dut.expect_next.value)}"
        
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
                f"[{i}] NO_OP: Expected state S_DELAY, got {self.get_state_name(int(self.dut.state.value))}"
            assert int(self.dut.cmd_done.value) == 1, \
                f"[{i}] NO_OP: cmd_done should be asserted after completing delay"
            return

    async def _sb_set_ord(self, info: dict, i: int):
        """Verify SET_ORD command execution."""
        self.dut._log.info(f"[{i}] SET_ORD: channels={info['channels']}")

        # next_cmd_state should be S_IDLE after this command
        assert int(self.dut.next_cmd_state.value) == self.STATE_ENCODING['S_IDLE'], \
            f"[{i}] SET_ORD command did not transition to S_IDLE, got {self.get_state_name(int(self.dut.next_cmd_state.value))}"
        # do_next_cmd should be asserted
        assert int(self.dut.do_next_cmd.value) == 1, \
            f"[{i}] for SET_ORD command do_next_cmd was not asserted"

        await RisingEdge(self.dut.clk)
        await ReadOnly()
        # expected ch indexes are inside info['channels'], check it against sample_order
        for idx in range (8):
            expected_ch = info['channels'][idx]
            dut_ch = int(self.dut.sample_order[idx].value)
            self.dut._log.info(f"Channel {idx}: expected {expected_ch}, DUT sample_order={dut_ch}")
            assert expected_ch == dut_ch, \
                f"SET_ORD command channel mismatch at index {idx}: expected {expected_ch} got {dut_ch}"
        return

    async def _sb_adc_rd(self, info: dict, i: int):
        """Verify ADC_RD command execution."""
        self.dut._log.info(f"[{i}] ADC_RD: trig={info['trig']} cont={info['cont']} repeat={info['repeat']} val={info['value']}")
        forked = []

        if info['trig'] == 1:
            self.trig_count = info['value']
            self.delay_count = None
        else:
            self.delay_count = info['value']
            self.trig_count = None

        num_channels = 9 # 8 channels + 1 dummy
        num_read_backs = num_channels - 1 # don't read back dummy
        expected_spi_cmd = []
        expected_adc_samples = []

        # Build expected SPI commands
        for ch in range(8):
            # spi_req_otf_sample_cmd = {2'b10, ch, 11'd0};
            cmd_word = (0b10 << 14) | (int(self.dut.sample_order[ch].value) << 11)
            expected_spi_cmd.append(cmd_word)
        expected_spi_cmd.append((0b10 << 14) | (0 << 11)) # dummy channel 0

        # Generate random expected ADC samples for read backs
        for _ in range(num_read_backs):
            expected_adc_samples.append(random.randint(0, 0xFFFF))

        forked.append(cocotb.start_soon(self._sb_mosi_data(expected_spi_cmd, num_channels=num_channels)))
        forked.append(cocotb.start_soon(self._sb_miso_data_miso_clk(num_read_backs=num_read_backs, expected_adc_samples=expected_adc_samples)))  
        forked.append(cocotb.start_soon(self._sb_miso_data_mosi_clk(num_read_backs=num_read_backs, expected_adc_samples=expected_adc_samples, single_channel_read=False)))
        if info['trig'] == 1:
            forked.append(cocotb.start_soon(self._sb_adc_rd_trig(trig_count=self.trig_count)))
        else:
            forked.append(cocotb.start_soon(self._sb_adc_rd_delay(delay_count=self.delay_count)))

        if forked:
            await Combine(*forked)

    async def _sb_adc_rd_repeating(self, repeat_count: int):
        """Verify ADC_RD command execution for repeating commands. Initial one is handled separately."""
        self.dut._log.info(f"ADC_RD Repeating command: repeat_count={repeat_count}")
        # Wait for initial command to complete
        while True:
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            if int(self.dut.cmd_done.value) == 1:
                self.dut._log.info("ADC_RD initial command completed, starting repeating scoreboard.")
                break
        
        ## If cancel_repeat is issued, exit the scoreboard
        #if int(self.dut.cancel_repeat.value) == 1:
        #    self.dut._log.info("ADC_RD repeating command was cancelled via CANCEL (cancel repeat) command, exiting repeating scoreboard.")
        #    return
        
        # Handle repeating commands
        for repeat_idx in range(repeat_count):
            self.dut._log.info(f"ADC_RD Repeating command iteration {repeat_idx + 1} of {repeat_count}")
            forked = []

            num_channels = 9 # 8 channels + 1 dummy
            num_read_backs = num_channels - 1 # don't read back dummy
            expected_spi_cmd = []
            expected_adc_samples = []

            # Build expected SPI commands
            for ch in range(8):
                # spi_req_otf_sample_cmd = {2'b10, ch, 11'd0};
                cmd_word = (0b10 << 14) | (int(self.dut.sample_order[ch].value) << 11)
                expected_spi_cmd.append(cmd_word)
            expected_spi_cmd.append((0b10 << 14) | (0 << 11)) # dummy channel 0

            # Generate random expected ADC samples for read backs
            for _ in range(num_read_backs):
                expected_adc_samples.append(random.randint(0, 0xFFFF))

            forked.append(cocotb.start_soon(self._sb_mosi_data(expected_spi_cmd, num_channels=num_channels)))
            forked.append(cocotb.start_soon(self._sb_miso_data_miso_clk(num_read_backs=num_read_backs, expected_adc_samples=expected_adc_samples)))  
            forked.append(cocotb.start_soon(self._sb_miso_data_mosi_clk(num_read_backs=num_read_backs, expected_adc_samples=expected_adc_samples, single_channel_read=False)))
            if self.trig_count is not None:
                forked.append(cocotb.start_soon(self._sb_adc_rd_trig(trig_count=self.trig_count)))
            elif self.delay_count is not None:
                forked.append(cocotb.start_soon(self._sb_adc_rd_delay(delay_count=self.delay_count)))

            if forked:
                await Combine(*forked)

    async def _sb_adc_rd_ch(self, info: dict, i: int):
        """Verify ADC_RD_CH command execution."""
        self.dut._log.info(f"[{i}] ADC_RD_CH: ch={info['ch']} repeat={info['repeat']}")
        forked = []

        # Set the global repeating flag for repeating commands
        if info['repeat'] == 1:
            self.adc_rd_ch_repeating_ch = info['ch']
        else:
            self.adc_rd_ch_repeating_ch = None

        # ADC_RD_CH reads one channel followed by a dummy read of channel 0
        num_channels = 2 

        # SPI command to request on-the-fly sample of channel `ch` is:
        # [15:0] spi_req_otf_sample_cmd(input [2:0] ch)
        # where spi_req_otf_sample_cmd = {2'b10, ch, 11'd0};
        expected_spi_cmd = []

        # spi_req_otf_sample_cmd(cmd_word[2:0])
        expected_single_channel_cmd = (0b10 << 14) | (info['ch'] << 11)
        expected_spi_cmd.append(expected_single_channel_cmd)

        # dummy is channel 0
        expected_dummy_cmd = (0b10 << 14) | (0 << 11)
        expected_spi_cmd.append(expected_dummy_cmd)

        # Don't read back the dummy and generate random expected read backs
        expected_adc_samples = []
        num_read_backs = num_channels - 1
        for _ in range(num_read_backs):
            expected_adc_samples.append(random.randint(0, 0xFFFF))  

        forked.append(cocotb.start_soon(self._sb_mosi_data(expected_spi_cmd, num_channels=num_channels)))
        forked.append(cocotb.start_soon(self._sb_miso_data_miso_clk(num_read_backs=num_read_backs, expected_adc_samples=expected_adc_samples)))  
        forked.append(cocotb.start_soon(self._sb_miso_data_mosi_clk(num_read_backs=num_read_backs, expected_adc_samples=expected_adc_samples, single_channel_read=True)))

        if forked:
            await Combine(*forked)

    async def _sb_adc_rd_ch_repeating(self, repeat_count: int):
        """Verify ADC_RD_CH command execution for repeating commands. Initial one is handled separately."""
        self.dut._log.info(f"ADC_RD_CH Repeating command: repeat_count={repeat_count}")
        # Wait for initial command to complete
        while True:
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            if int(self.dut.cmd_done.value) == 1:
                break
        
        # If cancel_repeat is issued, exit the scoreboard
        #if int(self.dut.cancel_repeat.value) == 1:
        #    self.dut._log.info("ADC_RD_CH repeating command was cancelled via CANCEL (cancel repeat) command, exiting repeating scoreboard.")
        #    return
        
        # Handle repeating commands
        for repeat_idx in range(repeat_count):
            self.dut._log.info(f"ADC_RD_CH Repeating command iteration {repeat_idx + 1} of {repeat_count}")
            forked = []

            # ADC_RD_CH reads one channel followed by a dummy read of channel 0
            num_channels = 2 
            expected_spi_cmd = []

            expected_single_channel_cmd = (0b10 << 14) | (self.adc_rd_ch_repeating_ch << 11)
            expected_spi_cmd.append(expected_single_channel_cmd)
            expected_dummy_cmd = (0b10 << 14) | (0 << 11)
            expected_spi_cmd.append(expected_dummy_cmd)

            # Don't read back the dummy and generate random expected read backs
            expected_adc_samples = []
            num_read_backs = num_channels - 1
            for _ in range(num_read_backs):
                expected_adc_samples.append(random.randint(0, 0xFFFF))  

            forked.append(cocotb.start_soon(self._sb_mosi_data(expected_spi_cmd, num_channels=num_channels)))
            forked.append(cocotb.start_soon(self._sb_miso_data_miso_clk(num_read_backs=num_read_backs, expected_adc_samples=expected_adc_samples)))  
            forked.append(cocotb.start_soon(self._sb_miso_data_mosi_clk(num_read_backs=num_read_backs, expected_adc_samples=expected_adc_samples, single_channel_read=True)))

            if forked:
                await Combine(*forked)

    async def _sb_mosi_data(self, expected_spi_cmd, num_channels=2):
        """Verify MOSI data correctness during SPI transactions."""
        for idx in range(num_channels):
            spi_bit_counter = 16
            spi_cmd = 0

            # Wait until cs_wait_done is asserted
            while True:
                await RisingEdge(self.dut.clk)
                await ReadOnly()
                if int(self.dut.cs_wait_done.value) == 1:
                    break

            # Sample the SPI cmd being sent via MOSI
            while spi_bit_counter > 0:
                await RisingEdge(self.dut.clk)
                await ReadOnly()
                mosi = int(self.dut.mosi.value)
                spi_cmd = (spi_cmd << 1) | mosi
                spi_bit_counter -= 1

            self.dut._log.info(f"[{idx}] SPI Command sent via MOSI: 0x{spi_cmd:04X}, Expected: 0x{expected_spi_cmd[idx]:04X}")
            assert spi_cmd == expected_spi_cmd[idx], \
                f"[{idx}] MOSI SPI command mismatch: expected 0x{expected_spi_cmd[idx]:04X} got 0x{spi_cmd:04X}"
            assert int(self.dut.adc_spi_cmd_done.value) == 1, \
                f"[{idx}] adc_spi_cmd_done should be asserted after sending SPI command."
        
        await RisingEdge(self.dut.clk)
        await ReadOnly()
        assert int(self.dut.adc_rd_done.value) == 1, \
            f"[{idx}] adc_rd_done should be asserted after sending SPI commands."
        # cmd_done should be asserted after all SPI commands are sent if trig_wait_done or delay_wait_done is also asserted
        if (int(self.dut.trig_wait_done.value) == 1 and int(self.dut.wait_for_trig.value) == 1) or (int(self.dut.delay_wait_done.value) == 1 and int(self.dut.wait_for_trig.value) == 0):
            assert int(self.dut.cmd_done.value) == 1, \
                f"[{idx}] cmd_done should be asserted after sending SPI commands."
        return
    
    async def _sb_miso_data_miso_clk(self, num_read_backs, expected_adc_samples):
        """Verify MISO data correctness during SPI transactions sent via MISO clock."""
        for idx in range(num_read_backs):
            spi_bit_counter = 16
            adc_sample = 0
            # 16-bit ADC sample data to be sent via MISO
            expected_adc_sample = expected_adc_samples[idx]
    
            # Wait until start_miso is asserted
            while True:
                await RisingEdge(self.dut.miso_sck)
                await ReadWrite()
                if int(self.dut.start_miso.value) == 1:
                    break

            # Send the ADC sample bit by bit via MISO
            while spi_bit_counter > 0:
                bit_to_send = (expected_adc_sample >> (spi_bit_counter - 1)) & 1
                self.dut.miso.value = bit_to_send
                spi_bit_counter -= 1
                await RisingEdge(self.dut.miso_sck)

            await ReadOnly()
            adc_sample = int(self.dut.miso_data.value)
            self.dut._log.info(f"[{idx}] MISO ADC random sample sent: 0x{expected_adc_sample:04X}, Received in DUT: 0x{adc_sample:04X}")
            assert adc_sample == expected_adc_sample, \
                f"[{idx}] MISO ADC sample mismatch: expected 0x{expected_adc_sample:04X} got 0x{adc_sample:04X}"
            
            # miso_buf_wr_en should be asserted after receiving ADC sample
            assert int(self.dut.miso_buf_wr_en.value) == 1, \
                f"[{idx}] miso_buf_wr_en should be asserted after receiving ADC sample via MISO."
        return
    
    async def _sb_miso_data_mosi_clk(self, num_read_backs, expected_adc_samples, single_channel_read=True):
        """Verify data buffer contents after ADC reads via MOSI clock domain. data_buf_model should be running."""
        num_data_buf_reads = num_read_backs if single_channel_read else num_read_backs // 2

        for idx in range(num_data_buf_reads):
            # Wait until data_buf_wr_en is asserted
            while True:
                await RisingEdge(self.dut.clk)
                await ReadOnly()
                if int(self.dut.data_buf_wr_en.value) == 1:
                    break

            # for single channel read, data_word is 32-bit {16'b0, expected_adc_samples[idx]}
            if single_channel_read:
                expected_data_word = 0x0000FFFF & expected_adc_samples[idx]
                data_word = int(self.data_buf.pop_item())
                self.dut._log.info(f"[{idx}] Data buffer word received: 0x{data_word:08X}, Expected: 0x{expected_data_word:08X}")
                assert data_word == expected_data_word, \
                    f"[{idx}] Data buffer word mismatch: expected 0x{expected_data_word:08X} got 0x{data_word:08X}"
            # for multi-channel read, data_word is 32-bit {expected_adc_samples[idx + 1], expected_adc_samples[idx]
            else:
                expected_data_word = ((expected_adc_samples[2*idx + 1] & 0xFFFF) << 16) | (expected_adc_samples[2*idx] & 0xFFFF)
                data_word = int(self.data_buf.pop_item())
                self.dut._log.info(f"[{idx}] Data buffer word received: 0x{data_word:08X}, Expected: 0x{expected_data_word:08X}")
                assert data_word == expected_data_word, \
                    f"[{idx}] Data buffer word mismatch: expected 0x{expected_data_word:08X} got 0x{data_word:08X}"
        return
    
    async def _sb_adc_rd_delay(self, delay_count: int):
        await RisingEdge(self.dut.clk)
        await ReadOnly()

        # Return if cancel_wait was issued
        if int(self.dut.cancel_wait.value) == 1:
            self.dut._log.info(f"ADC_RD delay was cancelled via CANCEL command, exiting delay scoreboard.")
            return
        
        assert int(self.dut.delay_timer.value) == delay_count, \
            f"ADC_RD delay_timer should be {delay_count}, got {int(self.dut.delay_timer.value)}"
            
        while delay_count > 0:
            await RisingEdge(self.dut.clk)
            await ReadOnly()

            # Return if cancel was issued
            if int(self.dut.cancel_wait.value) == 1:
                self.dut._log.info(f"ADC_RD delay: Cancel detected immediately after command fetch, exiting delay scoreboard.")
                return

            delay_count -= 1

            assert int(self.dut.delay_timer.value) == delay_count, \
                f"ADC_RD delay_timer should be {delay_count}, got {int(self.dut.delay_timer.value)}"
            
        # When we exit the loop delay_timer should be 0
        self.dut._log.info(f"ADC_RD delay completed.")
        assert int(self.dut.delay_timer.value) == 0, \
            f"ADC_RD delay_timer should be 0 after completing delay"
        
        return

    async def _sb_adc_rd_trig(self, trig_count: int):
        await RisingEdge(self.dut.clk)
        await ReadOnly()

        # Return if cancel_wait was issued
        if int(self.dut.cancel_wait.value) == 1:
            self.dut._log.info(f"ADC_RD trigger wait was cancelled via CANCEL command, exiting trigger wait scoreboard.")
            return
        
        assert int(self.dut.trigger_counter.value) == trig_count, \
            f"ADC_RD: trigger_counter should be {trig_count}, got {int(self.dut.trigger_counter.value)}"
            
        # If trig_count 0 to begin with, trig_wait_done should be asserted immediately
        if trig_count == 0:
            assert int(self.dut.trig_wait_done.value) == 1, \
            f"ADC_RD: trig_wait_done should be asserted when trigger_counter is 0"
        
        while trig_count > 0:
            await RisingEdge(self.dut.clk)
            previous_external_trigger = int(self.dut.trigger.value)
            await ReadOnly()
            current_external_trigger = int(self.dut.trigger.value)

            # Return if cancel was issued
            if int(self.dut.cancel_wait.value) == 1:
                self.dut._log.info(f"ADC_RD: Cancel detected immediately after command fetch, exiting ADC_RD scoreboard.")
                return

            # When final trigger is received trig_wait_done should be asserted and cmd_done should be asserted
            if current_external_trigger == 1 and trig_count == 1:
                self.dut._log.info(f"ADC_RD: Final trigger received")
                assert int(self.dut.trig_wait_done.value) == 1, \
                    f"ADC_RD: trig_wait_done should be asserted when final trigger is received"
                assert int(self.dut.cmd_done.value) == 1, \
                    f"ADC_RD: cmd_done should be asserted when final trigger is received"
                
            # When an external trigger is received, the trigger_counter should decrement
            if previous_external_trigger == 1:
                trig_count -= 1
            assert int(self.dut.trigger_counter.value) == trig_count, \
                f"ADC_RD: trigger_counter should be {trig_count}, got {int(self.dut.trigger_counter.value)}"
            
        # When we exit the loop trigger_counter should be 0
        assert int(self.dut.trigger_counter.value) == 0, \
            f"ADC_RD: trigger_counter should be 0 after completing trigger wait"
        return

    async def _sb_cancel(self, i: int):
        """Verify CANCEL command execution."""
        self.dut._log.info(f"[{i}] CANCEL command received.")
        return

    async def _sb_bad_cmd(self, i: int):
        """Verify behavior for invalid commands."""
        self.dut._log.info(f"[{i}] BAD_CMD detected.")
        # next_cmd_state should be S_ERROR
        assert int(self.dut.next_cmd_state.value) == self.STATE_ENCODING['S_ERROR'], \
              f"[{i}] BAD_CMD: Expected next_cmd_state S_ERROR , got {self.get_state_name(int(self.dut.next_cmd_state.value))}"
        # error flag should be asserted
        assert int(self.dut.error.value) == 1, \
              f"[{i}] BAD_CMD: error flag should be asserted"
        
        await RisingEdge(self.dut.clk)
        await ReadOnly()
        # state should be S_ERROR
        assert int(self.dut.state.value) == self.STATE_ENCODING['S_ERROR'], \
              f"[{i}] BAD_CMD: Expected state S_ERROR , got {self.get_state_name(int(self.dut.state.value))}"
        return

    # --------------------------------------
    # Transition Monitors
    # --------------------------------------
    async def transition_monitor(self):
        while True:
            await RisingEdge(self.dut.clk)
            # Sample previous values
            prev_cmd_buf_rd_en = int(self.dut.cmd_buf_rd_en.value)
            prev_cmd_buf_word = int(self.dut.cmd_buf_word.value)
            prev_resetn = int(self.dut.resetn.value)
            prev_state = int(self.dut.state.value)
            prev_do_next_cmd = int(self.dut.do_next_cmd.value)
            prev_command_val = int(self.dut.command.value)
            prev_cmd_word_val = int(self.dut.cmd_word.value)
            prev_wait_for_trig = int(self.dut.wait_for_trig.value)
            prev_expect_next = int(self.dut.expect_next.value)
            prev_error = int(self.dut.error.value)
            prev_boot_test_skip = int(self.dut.boot_test_skip.value)
            prev_adc_spi_cmd_done = int(self.dut.adc_spi_cmd_done.value)
            prev_n_miso_data_ready_mosi_clk = int(self.dut.n_miso_data_ready_mosi_clk.value)
            prev_adc_rd_done = int(self.dut.adc_rd_done.value)
            prev_cancel_wait = int(self.dut.cancel_wait.value)
            prev_cmd_done = int(self.dut.cmd_done.value)
            prev_next_cmd_state = int(self.dut.next_cmd_state.value)
            prev_start_repeat = int(self.dut.start_repeat.value)
            #prev_cancel_repeat = int(self.dut.cancel_repeat.value)
            prev_repeat_cmd_word_reg = int(self.dut.repeat_cmd_word.value)
            prev_prev_cmd_buf_word_reg = int(self.dut.prev_cmd_buf_word.value) # The register 'prev_cmd_buf_word'
            prev_repeating = int(self.dut.repeating.value)
            prev_repeat_counter = int(self.dut.repeat_counter.value)
            prev_delay_timer = int(self.dut.delay_timer.value)
            prev_trigger_counter = int(self.dut.trigger_counter.value)
            prev_trigger = int(self.dut.trigger.value)
            prev_try_data_write = int(self.dut.try_data_write.value)
            prev_data_buf_full = int(self.dut.data_buf_full.value)
            prev_next_cmd_ready = int(self.dut.next_cmd_ready.value)
            prev_cmd_buf_empty = int(self.dut.cmd_buf_empty.value)
            prev_boot_fail = int(self.dut.boot_fail.value)
            prev_unexp_trig = int(self.dut.unexp_trig.value)
            prev_delay_too_short = int(self.dut.delay_too_short.value)
            prev_bad_cmd = int(self.dut.bad_cmd.value)
            prev_cmd_buf_underflow = int(self.dut.cmd_buf_underflow.value)
            prev_data_buf_overflow = int(self.dut.data_buf_overflow.value)
            prev_boot_readback_match = int(self.dut.boot_readback_match.value)
            prev_delay_wait_done = int(self.dut.delay_wait_done.value)
            prev_last_adc_word = int(self.dut.last_adc_word.value)
            prev_adc_word_idx = int(self.dut.adc_word_idx.value)
            prev_n_cs_high_time = int(self.dut.n_cs_high_time.value)
            prev_n_cs_high_time_latched = int(self.dut.n_cs_high_time_latched.value)
            prev_start_spi_cmd = int(self.dut.start_spi_cmd.value)
            prev_n_cs_timer = int(self.dut.n_cs_timer.value)
            prev_cs_wait_done = int(self.dut.cs_wait_done.value)
            prev_n_cs = int(self.dut.n_cs.value)
            prev_spi_bit = int(self.dut.spi_bit.value)
            prev_single_reads = int(self.dut.single_reads.value)
            prev_miso_stored = int(self.dut.miso_stored.value)
            prev_miso_data_storage = int(self.dut.miso_data_storage.value)
            prev_miso_data_mosi_clk = int(self.dut.miso_data_mosi_clk.value)
            prev_adc_ch_data_ready = int(self.dut.adc_ch_data_ready.value)
            await ReadOnly()
            # Sample current values
            curr_prev_cmd_buf_word = int(self.dut.prev_cmd_buf_word.value)
            curr_repeating = int(self.dut.repeating.value)
            curr_cmd_buf_empty = int(self.dut.cmd_buf_empty.value)
            curr_repeat_cmd_word = int(self.dut.repeat_cmd_word.value)
            curr_cmd_buf_word = int(self.dut.cmd_buf_word.value)
            curr_start_repeat = int(self.dut.start_repeat.value)
            curr_repeat_counter = int(self.dut.repeat_counter.value)
            curr_state = int(self.dut.state.value)
            curr_cmd_done = int(self.dut.cmd_done.value)
            curr_cancel_wait = int(self.dut.cancel_wait.value)
            curr_cmd_word = int(self.dut.cmd_word.value)
            curr_command = int(self.dut.command.value)
            curr_next_cmd_ready = int(self.dut.next_cmd_ready.value)
            #curr_cancel_repeat = int(self.dut.cancel_repeat.value)
            curr_cmd_buf_rd_en = int(self.dut.cmd_buf_rd_en.value)
            curr_wait_for_trig = int(self.dut.wait_for_trig.value)
            curr_expect_next = int(self.dut.expect_next.value)
            curr_adc_rd_done = int(self.dut.adc_rd_done.value)
            curr_trigger = int(self.dut.trigger.value)
            curr_trigger_counter = int(self.dut.trigger_counter.value)
            curr_delay_timer = int(self.dut.delay_timer.value)
            curr_trig_wait_done = int(self.dut.trig_wait_done.value)
            curr_delay_wait_done = int(self.dut.delay_wait_done.value)
            curr_do_next_cmd = int(self.dut.do_next_cmd.value)
            curr_next_cmd_state = int(self.dut.next_cmd_state.value)
            curr_waiting_for_trig = int(self.dut.waiting_for_trig.value)
            curr_error = int(self.dut.error.value)
            curr_n_miso_data_ready_mosi_clk = int(self.dut.n_miso_data_ready_mosi_clk.value)
            curr_boot_readback_match = int(self.dut.boot_readback_match.value)
            curr_try_data_write = int(self.dut.try_data_write.value)
            curr_data_buf_full = int(self.dut.data_buf_full.value)
            curr_miso_data_mosi_clk = int(self.dut.miso_data_mosi_clk.value)
            curr_boot_fail = int(self.dut.boot_fail.value)
            curr_unexp_trig = int(self.dut.unexp_trig.value)
            curr_delay_too_short = int(self.dut.delay_too_short.value)
            curr_bad_cmd = int(self.dut.bad_cmd.value)
            curr_cmd_buf_underflow = int(self.dut.cmd_buf_underflow.value)
            curr_data_buf_overflow = int(self.dut.data_buf_overflow.value)
            curr_last_adc_word = int(self.dut.last_adc_word.value)
            curr_adc_word_idx = int(self.dut.adc_word_idx.value)
            curr_adc_spi_cmd_done = int(self.dut.adc_spi_cmd_done.value)
            curr_n_cs = int(self.dut.n_cs.value)
            curr_running_n_cs_timer = int(self.dut.running_n_cs_timer.value)
            curr_spi_bit = int(self.dut.spi_bit.value)
            curr_start_spi_cmd = int(self.dut.start_spi_cmd.value)
            curr_n_cs_high_time_latched = int(self.dut.n_cs_high_time_latched.value)
            curr_n_cs_timer = int(self.dut.n_cs_timer.value)
            curr_cs_wait_done = int(self.dut.cs_wait_done.value)
            curr_running_spi_bit = int(self.dut.running_spi_bit.value)
            curr_start_miso_mosi_clk = int(self.dut.start_miso_mosi_clk.value)
            curr_debug_miso_data = int(self.dut.debug_miso_data.value)
            curr_debug_state_transition = int(self.dut.debug_state_transition.value)
            curr_debug_repeat_bit = int(self.dut.debug_repeat_bit.value)
            curr_debug_n_cs_timer = int(self.dut.debug_n_cs_timer.value)
            curr_debug_spi_bit = int(self.dut.debug_spi_bit.value)
            curr_debug_cmd_done = int(self.dut.debug_cmd_done.value)
            curr_single_reads = int(self.dut.single_reads.value)
            curr_setup_done = int(self.dut.setup_done.value)
            curr_miso_stored = int(self.dut.miso_stored.value)
            curr_miso_data_storage = int(self.dut.miso_data_storage.value)
            curr_adc_pair_data_ready = int(self.dut.adc_pair_data_ready.value)
            curr_adc_ch_data_ready = int(self.dut.adc_ch_data_ready.value)
            curr_data_buf_wr_en = int(self.dut.data_buf_wr_en.value)

            # check prev_cmd_buf_word
            if prev_cmd_buf_rd_en:
                assert curr_prev_cmd_buf_word == prev_cmd_buf_word, \
                    f"Sequential Error: prev_cmd_buf_word expected {hex(prev_cmd_buf_word)}, got {hex(curr_prev_cmd_buf_word)}"

            # check cmd_word
            if curr_repeating:
                exp_cmd_word = curr_repeat_cmd_word
            elif curr_cmd_buf_empty:
                exp_cmd_word = 0
            else:
                exp_cmd_word = curr_cmd_buf_word
            
            assert curr_cmd_word == exp_cmd_word, \
                f"Comb Error: cmd_word expected {hex(exp_cmd_word)}, got {hex(curr_cmd_word)}"
            
            # check next_cmd_ready
            if curr_start_repeat:
                exp_next_cmd_ready = 0
            elif curr_repeating:
                exp_next_cmd_ready = 1
            else:
                exp_next_cmd_ready = 0 if curr_cmd_buf_empty else 1
            
            assert curr_next_cmd_ready == exp_next_cmd_ready, \
                f"Comb Error: next_cmd_ready expected {exp_next_cmd_ready}, got {curr_next_cmd_ready}"
            
            # check cancel_repeat
            #cmd_buf_opcode = (curr_cmd_buf_word >> 29) & 0x7
            #exp_cancel_repeat = 1 if (curr_repeat_counter > 0 and 
            #                          not curr_cmd_buf_empty and 
            #                          cmd_buf_opcode == self.CMD_ENCODING['CANCEL']) else 0
            #
            #assert curr_cancel_repeat == exp_cancel_repeat, \
            #    f"Comb Error: cancel_repeat expected {exp_cancel_repeat}, got {curr_cancel_repeat}"
            
            # check cmd_buf_rd_en
            cond_state = (curr_state != self.STATE_ENCODING['S_ERROR'])
            cond_empty = (not curr_cmd_buf_empty)
            cond_repeat = (not curr_repeating)
            cond_triggers = (curr_cmd_done or curr_cancel_wait or curr_start_repeat)
            
            exp_cmd_buf_rd_en = 1 if (cond_state and cond_empty and cond_repeat and cond_triggers) else 0
            
            assert curr_cmd_buf_rd_en == exp_cmd_buf_rd_en, \
                f"Comb Error: cmd_buf_rd_en expected {exp_cmd_buf_rd_en}, got {curr_cmd_buf_rd_en}"
            
            # check wait_for_trig and expect_next
            exp_wait_for_trig = prev_wait_for_trig
            exp_expect_next = prev_expect_next

            if prev_resetn == 0 or prev_state == self.STATE_ENCODING['S_ERROR']:
                exp_wait_for_trig = 0
                exp_expect_next = 0
            elif prev_do_next_cmd:
                if prev_command_val == self.CMD_ENCODING['NO_OP'] or prev_command_val == self.CMD_ENCODING['ADC_RD']:
                    exp_wait_for_trig = (prev_cmd_word_val >> self.TRIG_BIT) & 1
                    exp_expect_next = (prev_cmd_word_val >> self.CONT_BIT) & 1
                elif prev_command_val == self.CMD_ENCODING['ADC_RD_CH']:
                    exp_wait_for_trig = 1
                    exp_expect_next = 0
                else:
                    exp_wait_for_trig = 0
                    exp_expect_next = 0
            
            assert curr_wait_for_trig == exp_wait_for_trig, \
                f"Sequential Error: wait_for_trig expected {exp_wait_for_trig}, got {curr_wait_for_trig}"
            
            assert curr_expect_next == exp_expect_next, \
                f"Sequential Error: expect_next expected {exp_expect_next}, got {curr_expect_next}"
            
            # check cancel_wait
            cond_wait_state = (curr_state == self.STATE_ENCODING['S_DELAY'] or curr_state == self.STATE_ENCODING['S_TRIG_WAIT'] or (curr_state == self.STATE_ENCODING['S_ADC_RD'] and curr_adc_rd_done))
            exp_cancel_wait = 1 if (cond_wait_state and curr_next_cmd_ready and curr_command == self.CMD_ENCODING['CANCEL']) else 0
            
            assert curr_cancel_wait == exp_cancel_wait, \
                f"Comb Error: cancel_wait expected {exp_cancel_wait}, got {curr_cancel_wait}"
            
            # check trig_wait_done
            exp_trig_wait_done = 1 if ((curr_trigger and curr_trigger_counter == 1) or curr_trigger_counter == 0) else 0
            
            assert curr_trig_wait_done == exp_trig_wait_done, \
                f"Comb Error: trig_wait_done expected {exp_trig_wait_done}, got {curr_trig_wait_done}"

            # check delay_wait_done
            exp_delay_wait_done = 1 if (curr_delay_timer == 0) else 0

            assert curr_delay_wait_done == exp_delay_wait_done, \
                f"Comb Error: delay_wait_done expected {exp_delay_wait_done}, got {curr_delay_wait_done}"

            # check cmd_done
            cond_idle = (curr_state == self.STATE_ENCODING['S_IDLE'] and curr_next_cmd_ready)
            cond_delay = (curr_state == self.STATE_ENCODING['S_DELAY'] and curr_delay_wait_done)
            cond_trig = (curr_state == self.STATE_ENCODING['S_TRIG_WAIT'] and curr_trig_wait_done)
            
            sub_condition = curr_trig_wait_done if curr_wait_for_trig else curr_delay_wait_done
            cond_adc_rd = (curr_state == self.STATE_ENCODING['S_ADC_RD'] and curr_adc_rd_done and sub_condition)
            
            cond_adc_rd_ch = (curr_state == self.STATE_ENCODING['S_ADC_RD_CH'] and curr_adc_rd_done)
            
            exp_cmd_done = 1 if (cond_idle or cond_delay or cond_trig or cond_adc_rd or cond_adc_rd_ch) else 0
            
            assert curr_cmd_done == exp_cmd_done, \
                f"Comb Error: cmd_done expected {exp_cmd_done}, got {curr_cmd_done}"

            # check do_next_cmd
            exp_do_next_cmd = 1 if (curr_cmd_done and curr_next_cmd_ready) else 0
            
            assert curr_do_next_cmd == exp_do_next_cmd, \
                f"Comb Error: do_next_cmd expected {exp_do_next_cmd}, got {curr_do_next_cmd}"
            
            # check next_cmd_state
            if not curr_next_cmd_ready:
                # If buffer is empty, error if expecting next command, otherwise IDLE
                exp_next_cmd_state = self.STATE_ENCODING['S_ERROR'] if curr_expect_next else self.STATE_ENCODING['S_IDLE']
            elif curr_command == self.CMD_ENCODING['NO_OP']:
                # If command is NO_OP, either wait for trigger or delay depending on TRIG_BIT
                is_trig = (curr_cmd_word >> self.TRIG_BIT) & 1
                exp_next_cmd_state = self.STATE_ENCODING['S_TRIG_WAIT'] if is_trig else self.STATE_ENCODING['S_DELAY']
            elif curr_command == self.CMD_ENCODING['SET_ORD']:
                # If command is SET_ORD, go to IDLE
                exp_next_cmd_state = self.STATE_ENCODING['S_IDLE']
            elif curr_command == self.CMD_ENCODING['ADC_RD']:
                # If command is ADC read, go to ADC read state
                exp_next_cmd_state = self.STATE_ENCODING['S_ADC_RD']
            elif curr_command == self.CMD_ENCODING['ADC_RD_CH']:
                # If command is single-channel ADC read, go to ADC read state
                exp_next_cmd_state = self.STATE_ENCODING['S_ADC_RD_CH']
            elif curr_command == self.CMD_ENCODING['CANCEL']:
                # If command is CANCEL, go to IDLE
                exp_next_cmd_state = self.STATE_ENCODING['S_IDLE']
            else:
                # If command is unrecognized, go to ERROR state
                exp_next_cmd_state = self.STATE_ENCODING['S_ERROR']
                
            assert curr_next_cmd_state == exp_next_cmd_state, \
                f"Comb Error: next_cmd_state expected {exp_next_cmd_state}, got {curr_next_cmd_state}"

            # check waiting_for_trig
            exp_waiting_for_trig = 1 if (curr_state == self.STATE_ENCODING['S_TRIG_WAIT']) else 0
            assert curr_waiting_for_trig == exp_waiting_for_trig, \
                f"Comb Error: waiting_for_trig expected {exp_waiting_for_trig}, got {curr_waiting_for_trig}"
            
            # check state
            exp_state = prev_state

            if prev_resetn == 0:
                exp_state = self.STATE_ENCODING['S_RESET']
            elif prev_error:
                exp_state = self.STATE_ENCODING['S_ERROR']
            elif prev_state == self.STATE_ENCODING['S_RESET']:
                exp_state = self.STATE_ENCODING['S_IDLE'] if prev_boot_test_skip else self.STATE_ENCODING['S_INIT']
            elif prev_state == self.STATE_ENCODING['S_INIT']:
                exp_state = self.STATE_ENCODING['S_TEST_WR']
            elif prev_state == self.STATE_ENCODING['S_TEST_WR'] and prev_adc_spi_cmd_done:
                exp_state = self.STATE_ENCODING['S_REQ_RD']
            elif prev_state == self.STATE_ENCODING['S_REQ_RD'] and prev_adc_spi_cmd_done:
                exp_state = self.STATE_ENCODING['S_TEST_RD']
            elif prev_state == self.STATE_ENCODING['S_TEST_RD'] and (not prev_n_miso_data_ready_mosi_clk):
                exp_state = self.STATE_ENCODING['S_IDLE']
            elif prev_cancel_wait:
                exp_state = self.STATE_ENCODING['S_IDLE']
            elif prev_cmd_done:
                exp_state = prev_next_cmd_state
            elif prev_state == self.STATE_ENCODING['S_ADC_RD'] and prev_adc_rd_done:
                exp_state = self.STATE_ENCODING['S_TRIG_WAIT'] if prev_wait_for_trig else self.STATE_ENCODING['S_DELAY']
            
            assert curr_state == exp_state, \
                f"Sequential Error: state expected {hex(exp_state)}, got {hex(curr_state)}"
            
            # check repeating
            exp_repeating = 1 if curr_repeat_counter > 0 else 0
            assert curr_repeating == exp_repeating, \
                f"Comb Error: repeating expected {exp_repeating}, got {curr_repeating}"
            
            # check start_repeat
            exp_start_repeat = prev_start_repeat
            
            if prev_resetn == 0 or prev_state == self.STATE_ENCODING['S_ERROR']:
                exp_start_repeat = 0
            elif prev_start_repeat:
                # Clear start_repeat after using it
                exp_start_repeat = 0
            elif prev_do_next_cmd and (prev_command_val == self.CMD_ENCODING['ADC_RD'] or prev_command_val == self.CMD_ENCODING['ADC_RD_CH']):
                # (cancel_repeat || start_repeat) ? 1'b0 : cmd_word[REPEAT_BIT]
                # Note: prev_start_repeat is known 0 here due to previous elif
                exp_start_repeat = (prev_cmd_word_val >> self.REPEAT_BIT) & 1
            
            assert curr_start_repeat == exp_start_repeat, \
                f"Sequential Error: start_repeat expected {exp_start_repeat}, got {curr_start_repeat}"
            
            # check repeat_cmd_word
            exp_repeat_cmd_word = prev_repeat_cmd_word_reg 

            if prev_resetn == 0 or prev_state == self.STATE_ENCODING['S_ERROR']:
                exp_repeat_cmd_word = 0
            elif prev_start_repeat:
                # prev_cmd_buf_word & ~(32'd1 << REPEAT_BIT)
                mask = ~(1 << self.REPEAT_BIT) & 0xFFFFFFFF
                exp_repeat_cmd_word = prev_prev_cmd_buf_word_reg & mask
            
            assert curr_repeat_cmd_word == exp_repeat_cmd_word, \
                f"Sequential Error: repeat_cmd_word expected {hex(exp_repeat_cmd_word)}, got {hex(curr_repeat_cmd_word)}"
            
            # check repeat_counter
            exp_repeat_counter = prev_repeat_counter

            if prev_resetn == 0 or prev_state == self.STATE_ENCODING['S_ERROR']:
                exp_repeat_counter = 0
            elif prev_repeating and prev_cmd_done:
                exp_repeat_counter = prev_repeat_counter - 1
            elif prev_start_repeat:
                # repeat_counter <= cmd_buf_word[31:0]
                exp_repeat_counter = prev_cmd_buf_word
            
            assert curr_repeat_counter == exp_repeat_counter, \
                f"Sequential Error: repeat_counter expected {hex(exp_repeat_counter)}, got {hex(curr_repeat_counter)}"
            
            # check delay_timer 
            if prev_resetn == 0 or prev_state == self.STATE_ENCODING['S_ERROR'] or prev_cancel_wait:
                exp_delay_timer = 0
            elif prev_do_next_cmd and \
                 (prev_command_val == self.CMD_ENCODING['ADC_RD'] or prev_command_val == self.CMD_ENCODING['NO_OP']) and \
                 not ((prev_cmd_word_val >> self.TRIG_BIT) & 1):
                # load the delay timer from command word (lower 25 bits)
                exp_delay_timer = prev_cmd_word_val & 0x1FFFFFF
            elif prev_delay_timer > 0:
                exp_delay_timer = prev_delay_timer - 1
            else:
                exp_delay_timer = prev_delay_timer

            assert curr_delay_timer == exp_delay_timer, \
                f"Sequential Error: delay_timer expected {hex(exp_delay_timer)}, got {hex(curr_delay_timer)}"
            
            # check trigger_counter
            if prev_resetn == 0 or prev_state == self.STATE_ENCODING['S_ERROR'] or prev_cancel_wait:
                exp_trigger_counter = 0
            elif prev_do_next_cmd and \
                 (prev_command_val == self.CMD_ENCODING['ADC_RD'] or prev_command_val == self.CMD_ENCODING['NO_OP']) and \
                 ((prev_cmd_word_val >> self.TRIG_BIT) & 1):
                # load the trigger counter from command word (lower 25 bits)
                exp_trigger_counter = prev_cmd_word_val & 0x1FFFFFF
            elif prev_do_next_cmd and prev_command_val == self.CMD_ENCODING['ADC_RD_CH']:
                exp_trigger_counter = 0
            elif prev_trigger_counter > 0 and prev_trigger:
                exp_trigger_counter = prev_trigger_counter - 1
            else:
                exp_trigger_counter = prev_trigger_counter
            
            assert curr_trigger_counter == exp_trigger_counter, \
                f"Sequential Error: trigger_counter expected {hex(exp_trigger_counter)}, got {hex(curr_trigger_counter)}"
            
             # check error
            err_boot = (curr_state == self.STATE_ENCODING['S_TEST_RD'] and not curr_n_miso_data_ready_mosi_clk and not curr_boot_readback_match)
            err_trig = (curr_state != self.STATE_ENCODING['S_TRIG_WAIT'] and curr_trigger and curr_trigger_counter <= 1)
            err_delay = (curr_state == self.STATE_ENCODING['S_ADC_RD'] and not curr_adc_rd_done and not curr_wait_for_trig and curr_delay_wait_done)
            err_cmd = (curr_do_next_cmd and curr_next_cmd_state == self.STATE_ENCODING['S_ERROR'])
            err_underflow = (curr_cmd_done and curr_expect_next and not curr_next_cmd_ready)
            err_repeat = (curr_start_repeat and curr_cmd_buf_empty)
            err_data_full = (curr_try_data_write and curr_data_buf_full)
            
            exp_error = 1 if (err_boot or err_trig or err_delay or err_cmd or err_underflow or err_repeat or err_data_full) else 0
            
            assert curr_error == exp_error, \
                f"Comb Error: error expected {exp_error}, got {curr_error}"
            
            # If we have an error, log the details
            if curr_error:
                self.dut._log.warning(f"Error detected at state {self.get_state_name(curr_state)}:")
                if err_boot:
                    self.dut._log.warning(f"  Boot readback mismatch: miso_data_mosi_clk[15:8]={hex((curr_miso_data_mosi_clk >> 8) & 0xFF)}, expected {hex(self.SET_OTF_CFG_DATA)}")
                if err_trig:
                    self.dut._log.warning(f"  Unexpected trigger: trigger={curr_trigger}, trigger_counter={curr_trigger_counter}")
                if err_delay:
                    self.dut._log.warning(f"  Delay too short: adc_rd_done={curr_adc_rd_done}, wait_for_trig={curr_wait_for_trig}, delay_wait_done={curr_delay_wait_done}")
                if err_cmd:
                    self.dut._log.warning(f"  Bad command encountered: next_cmd_state={self.get_state_name(curr_next_cmd_state)}")
                if err_underflow:
                    self.dut._log.warning(f"  Command buffer underflow: cmd_done={curr_cmd_done}, expect_next={curr_expect_next}, next_cmd_ready={curr_next_cmd_ready}")
                if err_repeat:
                    self.dut._log.warning(f"  Command buffer underflow on repeat start: start_repeat={curr_start_repeat}, cmd_buf_empty={curr_cmd_buf_empty}")
                if err_data_full:
                    self.dut._log.warning(f"  Data buffer overflow: try_data_write={curr_try_data_write}, data_buf_full={curr_data_buf_full}")
            
            # check boot_readback_match
            # (miso_data_mosi_clk[15:8] == SET_OTF_CFG_DATA)
            exp_boot_readback_match = 1 if ((curr_miso_data_mosi_clk >> 8) & 0xFF) == self.SET_OTF_CFG_DATA else 0
            assert curr_boot_readback_match == exp_boot_readback_match, \
                f"Comb Error: boot_readback_match expected {exp_boot_readback_match}, got {curr_boot_readback_match}"
            
            # check boot_fail 
            exp_boot_fail = prev_boot_fail
            
            if prev_resetn == 0:
                exp_boot_fail = 0
            elif prev_state == self.STATE_ENCODING['S_TEST_RD'] and not prev_n_miso_data_ready_mosi_clk:
                exp_boot_fail = 0 if prev_boot_readback_match else 1
            
            assert curr_boot_fail == exp_boot_fail, \
                f"Sequential Error: boot_fail expected {exp_boot_fail}, got {curr_boot_fail}"
            
            # check unexp_trig
            exp_unexp_trig = prev_unexp_trig
            if prev_resetn == 0:
                exp_unexp_trig = 0
            elif prev_state != self.STATE_ENCODING['S_TRIG_WAIT'] and prev_trigger and prev_trigger_counter <= 1:
                exp_unexp_trig = 1
            
            assert curr_unexp_trig == exp_unexp_trig, \
                f"Sequential Error: unexp_trig expected {exp_unexp_trig}, got {curr_unexp_trig}"
            
            # Log the unexpected trigger event
            if curr_unexp_trig:
                self.dut._log.warning(f"Unexpected trigger event: trigger={curr_trigger}, trigger_counter={curr_trigger_counter}")
            
            # check delay_too_short
            exp_delay_too_short = prev_delay_too_short
            
            if prev_resetn == 0:
                exp_delay_too_short = 0
            elif prev_state == self.STATE_ENCODING['S_ADC_RD'] and not prev_adc_rd_done and not prev_wait_for_trig and prev_delay_wait_done:
                exp_delay_too_short = 1
            
            assert curr_delay_too_short == exp_delay_too_short, \
                f"Sequential Error: delay_too_short expected {exp_delay_too_short}, got {curr_delay_too_short}"
            
            # Log the delay too short event
            if curr_delay_too_short:
                self.dut._log.warning(f"Delay too short event: state={curr_state}, adc_rd_done={curr_adc_rd_done}, wait_for_trig={curr_wait_for_trig}, delay_wait_done={curr_delay_wait_done}")

            # check bad_cmd
            exp_bad_cmd = prev_bad_cmd
            if prev_resetn == 0:
                exp_bad_cmd = 0
            elif prev_do_next_cmd and prev_next_cmd_state == self.STATE_ENCODING['S_ERROR']:
                exp_bad_cmd = 1
            
            assert curr_bad_cmd == exp_bad_cmd, \
                f"Sequential Error: bad_cmd expected {exp_bad_cmd}, got {curr_bad_cmd}"
            
            # check cmd_buf_underflow
            exp_cmd_buf_underflow = prev_cmd_buf_underflow
            if prev_resetn == 0:
                exp_cmd_buf_underflow = 0
            elif (prev_cmd_done and prev_expect_next and not prev_next_cmd_ready) or (prev_start_repeat and prev_cmd_buf_empty):
                exp_cmd_buf_underflow = 1
            
            assert curr_cmd_buf_underflow == exp_cmd_buf_underflow, \
                f"Sequential Error: cmd_buf_underflow expected {exp_cmd_buf_underflow}, got {curr_cmd_buf_underflow}"
            
            # check data_buf_overflow
            exp_data_buf_overflow = prev_data_buf_overflow
            if prev_resetn == 0:
                exp_data_buf_overflow = 0
            elif prev_try_data_write and prev_data_buf_full:
                exp_data_buf_overflow = 1
            
            assert curr_data_buf_overflow == exp_data_buf_overflow, \
                f"Sequential Error: data_buf_overflow expected {exp_data_buf_overflow}, got {curr_data_buf_overflow}"
            
            # check last_adc_word
            exp_last_adc_word = (curr_state == self.STATE_ENCODING['S_ADC_RD'] and curr_adc_word_idx == 8) or (curr_state == self.STATE_ENCODING['S_ADC_RD_CH'] and curr_adc_word_idx == 1)
            assert curr_last_adc_word == exp_last_adc_word, \
                f"Comb Error: last_adc_word expected {exp_last_adc_word}, got {curr_last_adc_word}"
            
            # check adc_spi_cmd_done
            cond_spi_state = (curr_state == self.STATE_ENCODING['S_ADC_RD'] or
                              curr_state == self.STATE_ENCODING['S_ADC_RD_CH'] or
                              curr_state == self.STATE_ENCODING['S_TEST_WR'] or
                              curr_state == self.STATE_ENCODING['S_REQ_RD'] or
                              curr_state == self.STATE_ENCODING['S_TEST_RD'])
            
            exp_adc_spi_cmd_done = 1 if (cond_spi_state and 
                                         not curr_n_cs and 
                                         not curr_running_n_cs_timer and 
                                         curr_spi_bit == 0) else 0

            assert curr_adc_spi_cmd_done == exp_adc_spi_cmd_done, \
                f"Comb Error: adc_spi_cmd_done expected {exp_adc_spi_cmd_done}, got {curr_adc_spi_cmd_done}"
            
            # check adc_rd_done
            if prev_resetn == 0 or prev_state == self.STATE_ENCODING['S_ERROR']:
                exp_adc_rd_done = 0
            elif prev_last_adc_word and prev_adc_spi_cmd_done:
                exp_adc_rd_done = 1
            else:
                exp_adc_rd_done = 0
            
            assert curr_adc_rd_done == exp_adc_rd_done, \
                f"Sequential Error: adc_rd_done expected {exp_adc_rd_done}, got {curr_adc_rd_done}"
            
            # check adc_word_idx
            if prev_resetn == 0 or prev_state == self.STATE_ENCODING['S_ERROR']:
                exp_adc_word_idx = 0
            elif prev_do_next_cmd and (prev_command_val == self.CMD_ENCODING['ADC_RD'] or prev_command_val == self.CMD_ENCODING['ADC_RD_CH']):
                exp_adc_word_idx = 0
            elif (prev_state == self.STATE_ENCODING['S_ADC_RD'] or prev_state == self.STATE_ENCODING['S_ADC_RD_CH']) and prev_adc_spi_cmd_done:
                if not prev_last_adc_word:
                    exp_adc_word_idx = (prev_adc_word_idx + 1) & 0xF # 4-bit wrap
                else:
                    exp_adc_word_idx = 0
            else:
                exp_adc_word_idx = prev_adc_word_idx
            
            assert curr_adc_word_idx == exp_adc_word_idx, \
                f"Sequential Error: adc_word_idx expected {exp_adc_word_idx}, got {curr_adc_word_idx}"
            
            # check start_spi_cmd
            cond_cmd_rd = (curr_do_next_cmd and (curr_command == self.CMD_ENCODING['ADC_RD'] or curr_command == self.CMD_ENCODING['ADC_RD_CH']))
            cond_init = (curr_state == self.STATE_ENCODING['S_INIT'])
            cond_test_wr = (curr_state == self.STATE_ENCODING['S_TEST_WR'] and curr_adc_spi_cmd_done)
            cond_req_rd = (curr_state == self.STATE_ENCODING['S_REQ_RD'] and curr_adc_spi_cmd_done)
            cond_adc_rd_cont = ((curr_state == self.STATE_ENCODING['S_ADC_RD'] or curr_state == self.STATE_ENCODING['S_ADC_RD_CH']) and 
                                curr_adc_spi_cmd_done and not curr_last_adc_word)
            
            exp_start_spi_cmd = 1 if (cond_cmd_rd or cond_init or cond_test_wr or cond_req_rd or cond_adc_rd_cont) else 0
            
            assert curr_start_spi_cmd == exp_start_spi_cmd, \
                f"Comb Error: start_spi_cmd expected {exp_start_spi_cmd}, got {curr_start_spi_cmd}"
            
            # check n_cs_high_time_latched
            if prev_resetn == 0:
                exp_n_cs_high_time_latched = 8
            elif prev_state == self.STATE_ENCODING['S_RESET']:
                exp_n_cs_high_time_latched = prev_n_cs_high_time
            else:
                exp_n_cs_high_time_latched = prev_n_cs_high_time_latched
            
            assert curr_n_cs_high_time_latched == exp_n_cs_high_time_latched, \
                f"Sequential Error: n_cs_high_time_latched expected {exp_n_cs_high_time_latched}, got {curr_n_cs_high_time_latched}"

            # check n_cs_timer
            if prev_resetn == 0 or prev_state == self.STATE_ENCODING['S_ERROR']:
                exp_n_cs_timer = 0
            elif prev_start_spi_cmd:
                exp_n_cs_timer = prev_n_cs_high_time_latched
            elif prev_n_cs_timer > 0:
                exp_n_cs_timer = prev_n_cs_timer - 1
            else:
                exp_n_cs_timer = prev_n_cs_timer
                
            assert curr_n_cs_timer == exp_n_cs_timer, \
                f"Sequential Error: n_cs_timer expected {exp_n_cs_timer}, got {curr_n_cs_timer}"

            # check running_n_cs_timer
            exp_running_n_cs_timer = 1 if prev_n_cs_timer > 0 else 0
            assert curr_running_n_cs_timer == exp_running_n_cs_timer, \
                f"Sequential Error: running_n_cs_timer expected {exp_running_n_cs_timer}, got {curr_running_n_cs_timer}"

            # check cs_wait_done 
            exp_cs_wait_done = 1 if (curr_running_n_cs_timer and curr_n_cs_timer == 0) else 0
            assert curr_cs_wait_done == exp_cs_wait_done, \
                f"Comb Error: cs_wait_done expected {exp_cs_wait_done}, got {curr_cs_wait_done}"

            # check n_cs
            if prev_resetn == 0 or prev_state == self.STATE_ENCODING['S_ERROR']:
                exp_n_cs = 1
            elif prev_cs_wait_done:
                exp_n_cs = 0
            elif prev_adc_spi_cmd_done or prev_state == self.STATE_ENCODING['S_IDLE']:
                exp_n_cs = 1
            else:
                exp_n_cs = prev_n_cs
            
            assert curr_n_cs == exp_n_cs, \
                f"Sequential Error: n_cs expected {exp_n_cs}, got {curr_n_cs}"
            
            # check spi_bit
            if prev_resetn == 0 or prev_state == self.STATE_ENCODING['S_ERROR']:
                exp_spi_bit = 0
            elif prev_spi_bit > 0:
                exp_spi_bit = prev_spi_bit - 1
            elif prev_cs_wait_done:
                if prev_state == self.STATE_ENCODING['S_ADC_RD'] or prev_state == self.STATE_ENCODING['S_TEST_RD'] or prev_state == self.STATE_ENCODING['S_ADC_RD_CH']:
                    exp_spi_bit = 15
                else:
                    exp_spi_bit = 23
            else:
                exp_spi_bit = prev_spi_bit
            
            assert curr_spi_bit == exp_spi_bit, \
                f"Sequential Error: spi_bit expected {exp_spi_bit}, got {curr_spi_bit}"

            # check running_spi_bit
            exp_running_spi_bit = 1 if prev_spi_bit > 0 else 0
            
            assert curr_running_spi_bit == exp_running_spi_bit, \
                f"Sequential Error: running_spi_bit expected {exp_running_spi_bit}, got {curr_running_spi_bit}"
            
            # check start_miso_mosi_clk
            if prev_resetn == 0 or prev_state == self.STATE_ENCODING['S_ERROR']:
                exp_start_miso_mosi_clk = 0
            elif (
                (prev_state == self.STATE_ENCODING['S_TEST_RD'] or 
                 ((prev_state == self.STATE_ENCODING['S_ADC_RD'] or prev_state == self.STATE_ENCODING['S_ADC_RD_CH']) and prev_adc_word_idx > 0))
                and prev_n_cs_timer == self.N_CS_MISO_START_TIME
            ):
                exp_start_miso_mosi_clk = 1
            else:
                exp_start_miso_mosi_clk = 0
            
            assert curr_start_miso_mosi_clk == exp_start_miso_mosi_clk, \
                f"Sequential Error: start_miso_mosi_clk expected {exp_start_miso_mosi_clk}, got {curr_start_miso_mosi_clk}"

            # conditions
            cond_pair_ready = (curr_state != self.STATE_ENCODING['S_TEST_RD'] and curr_setup_done and 
                               not curr_n_miso_data_ready_mosi_clk and curr_miso_stored and curr_single_reads == 0)
            exp_adc_pair_data_ready = 1 if cond_pair_ready else 0
            
            cond_ch_ready = (curr_single_reads > 0 and not curr_n_miso_data_ready_mosi_clk)
            exp_adc_ch_data_ready = 1 if cond_ch_ready else 0

            # check adc_pair_data_ready
            assert curr_adc_pair_data_ready == exp_adc_pair_data_ready, \
                f"Comb Error: adc_pair_data_ready expected {exp_adc_pair_data_ready}, got {curr_adc_pair_data_ready}"

            # check adc_ch_data_ready
            assert curr_adc_ch_data_ready == exp_adc_ch_data_ready, \
                f"Comb Error: adc_ch_data_ready expected {exp_adc_ch_data_ready}, got {curr_adc_ch_data_ready}"

            # check try_data_write
            cond_try_write = (curr_adc_pair_data_ready or 
                              curr_adc_ch_data_ready or 
                              curr_debug_miso_data or 
                              curr_debug_state_transition or 
                              curr_debug_repeat_bit or 
                              curr_debug_n_cs_timer or 
                              curr_debug_spi_bit or 
                              curr_debug_cmd_done)
            exp_try_data_write = 1 if cond_try_write else 0
            
            assert curr_try_data_write == exp_try_data_write, \
                f"Comb Error: try_data_write expected {exp_try_data_write}, got {curr_try_data_write}"

            # check single_reads
            if prev_resetn == 0 or prev_state == self.STATE_ENCODING['S_ERROR']:
                exp_single_reads = 0
            elif prev_do_next_cmd and prev_command_val == self.CMD_ENCODING['ADC_RD_CH']:
                if prev_adc_ch_data_ready:
                    exp_single_reads = prev_single_reads # Increment and decrement cancel out
                else:
                    exp_single_reads = (prev_single_reads + 1) & 0x7 # 3-bit
            elif prev_adc_ch_data_ready:
                exp_single_reads = (prev_single_reads - 1) & 0x7
            else:
                exp_single_reads = prev_single_reads
            
            assert curr_single_reads == exp_single_reads, \
                f"Sequential Error: single_reads expected {exp_single_reads}, got {curr_single_reads}"

            # check data_buf_wr_en
            if prev_resetn == 0:
                exp_data_buf_wr_en = 0
            elif prev_try_data_write and not prev_data_buf_full:
                exp_data_buf_wr_en = 1
            else:
                exp_data_buf_wr_en = 0
            
            assert curr_data_buf_wr_en == exp_data_buf_wr_en, \
                f"Sequential Error: data_buf_wr_en expected {exp_data_buf_wr_en}, got {curr_data_buf_wr_en}"

            # check miso_stored
            if prev_resetn == 0 or prev_state == self.STATE_ENCODING['S_ERROR']:
                exp_miso_stored = 0
            elif prev_state != self.STATE_ENCODING['S_TEST_RD'] and not prev_n_miso_data_ready_mosi_clk and prev_single_reads == 0:
                exp_miso_stored = 0 if prev_miso_stored else 1 # Toggle
            else:
                exp_miso_stored = prev_miso_stored
            
            assert curr_miso_stored == exp_miso_stored, \
                f"Sequential Error: miso_stored expected {exp_miso_stored}, got {curr_miso_stored}"

            # check miso_data_storage
            if prev_resetn == 0 or prev_state == self.STATE_ENCODING['S_ERROR']:
                exp_miso_data_storage = 0
            elif not prev_n_miso_data_ready_mosi_clk and prev_single_reads == 0:
                exp_miso_data_storage = prev_miso_data_mosi_clk
            else:
                exp_miso_data_storage = prev_miso_data_storage
            
            assert curr_miso_data_storage == exp_miso_data_storage, \
                f"Sequential Error: miso_data_storage expected {hex(exp_miso_data_storage)}, got {hex(curr_miso_data_storage)}"

    async def miso_transition_monitor(self):
        while True:
            await RisingEdge(self.dut.miso_sck)
            # Sample previous values
            prev_miso_resetn = int(self.dut.miso_resetn.value)
            prev_miso_bit = int(self.dut.miso_bit.value)
            prev_start_miso = int(self.dut.start_miso.value)
            prev_miso_shift_reg = int(self.dut.miso_shift_reg.value)
            prev_miso = int(self.dut.miso.value)
            await ReadOnly()
            # Sample current values
            curr_miso_bit = int(self.dut.miso_bit.value)
            curr_miso_shift_reg = int(self.dut.miso_shift_reg.value)
            curr_miso_data = int(self.dut.miso_data.value)
            curr_miso = int(self.dut.miso.value)
            curr_miso_buf_wr_en = int(self.dut.miso_buf_wr_en.value)

            # check miso_bit
            if prev_miso_resetn == 0:
                 exp_miso_bit = 0
            elif prev_miso_bit > 0:
                 exp_miso_bit = prev_miso_bit - 1
            elif prev_start_miso:
                 exp_miso_bit = 15
            else:
                 exp_miso_bit = prev_miso_bit
            
            assert curr_miso_bit == exp_miso_bit, \
                f"Sequential Error: miso_bit expected {exp_miso_bit}, got {curr_miso_bit}"

            # check miso_shift_reg
            if prev_miso_resetn == 0:
                exp_miso_shift_reg = 0
            elif prev_miso_bit > 1:
                # {miso_shift_reg[13:0], miso}
                exp_miso_shift_reg = ((prev_miso_shift_reg & 0x3FFF) << 1) | prev_miso
            elif prev_start_miso:
                # {14'd0, miso}
                exp_miso_shift_reg = prev_miso
            else:
                exp_miso_shift_reg = prev_miso_shift_reg
            
            assert curr_miso_shift_reg == exp_miso_shift_reg, \
                f"Sequential Error: miso_shift_reg expected {hex(exp_miso_shift_reg)}, got {hex(curr_miso_shift_reg)}"

            # check miso_data
            # {miso_shift_reg, miso}
            exp_miso_data = (curr_miso_shift_reg << 1) | curr_miso
            assert curr_miso_data == exp_miso_data, \
                f"Comb Error: miso_data expected {hex(exp_miso_data)}, got {hex(curr_miso_data)}"

            # check miso_buf_wr_en
            if prev_miso_resetn == 0:
                exp_miso_buf_wr_en = 0
            elif prev_miso_bit == 1:
                exp_miso_buf_wr_en = 1
            else:
                exp_miso_buf_wr_en = 0
            
            assert curr_miso_buf_wr_en == exp_miso_buf_wr_en, \
                f"Sequential Error: miso_buf_wr_en expected {exp_miso_buf_wr_en}, got {curr_miso_buf_wr_en}"
            
    # --------------------------------------
    # Wrappers for simulating the DUT.
    # Does not test functionality.
    # Use it to observe waveforms and command execution.
    # Command builders and decoders can be used to create a list of commands and log them.
    # --------------------------------------

    async def simulate_dut(self, cmd_list, clk_cycles=1000):
        """Simulate the DUT by sending a list of commands and observing the outputs for a certain number of clock cycles after."""

        # Both cmd_buf and executing_cmd_queue are initialized in __init__ and live inside the class.
        # Reset will also clear both the cmd_buf and executing_cmd_queue to make sure we start fresh.
        await self.reset()
        
        # command_buf_model is an interface between the DUT and the fwft fifo model called cmd_buf.
        # It is connected to the DUT's command buffer interface and, 
        # it will consume commands from cmd_buf and provide the appropriate handshakes so that DUT can read commands from it.
        # The commands read by DUT are stored in self.executing_cmd_queue for reference.
        command_buf_task = cocotb.start_soon(self.command_buf_model())

        # send_commands will try to write the commands in cmd_list into the cmd_buf until all commands are written.
        # If cmd_buf is full, it will try to write the same command in the next DUT clock cycle.
        send_commands_task = cocotb.start_soon(self.send_commands(cmd_list))

        # Log the executing commands read by DUT in readable format.
        command_logger_task = cocotb.start_soon(self.log_executing_commands(len(cmd_list)))
        
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

            popped = self.executing_cmd_queue.popleft()
            decoded = self.decode_cmd(popped)
            self.dut._log.info(f"Executing command logged: {decoded}, Logged: {logged+1}/{num_of_commands}")
            logged += 1
        return
