import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, ReadWrite, Combine
from collections import deque
from fwft_fifo_model import fwft_fifo_model
import random


class trigger_core_base:

    CMD_ENCODING = {
        1: 'CMD_SYNC_CH',
        2: 'CMD_SET_LOCKOUT',
        3: 'CMD_EXPECT_EXT_TRIG',
        4: 'CMD_DELAY',
        5: 'CMD_FORCE_TRIG',
        7: 'CMD_CANCEL'
    }

    STATE_ENCODING = {
        0: 'S_RESET',
        1: 'S_IDLE',
        2: 'S_SYNC_CH',
        3: 'S_EXPECT_TRIG',
        4: 'S_DELAY',
        5: 'S_ERROR'
    }
    
    def __init__ (self, dut, clk_period=4, time_unit='ns'):
        self.dut = dut
        self.clk_period = clk_period
        self.time_unit = time_unit

        # Parameters
        self.TRIGGER_LOCKOUT_DEFAULT = int(self.dut.TRIGGER_LOCKOUT_DEFAULT.value)
        self.TRIGGER_LOCKOUT_MIN = int(self.dut.TRIGGER_LOCKOUT_MIN.value)
        self.MAX_CMD_VALUE = 0x1FFFFFFF  # 29 bits for cmd_value

        # Initialize clock
        cocotb.start_soon(Clock(dut.clk, clk_period, time_unit).start(start_high=False))

        self.dut._log.info(f"TRIGGER_LOCKOUT_DEFAULT set to {self.TRIGGER_LOCKOUT_DEFAULT}")

        # Initialize Input Signals
        self.dut.cmd_word.value = 0
        self.dut.cmd_buf_empty.value = 1
        self.dut.data_buf_full.value = 0
        self.dut.data_buf_almost_full.value = 0
        self.dut.ext_trig.value = 0
        self.dut.dac_waiting_for_trig.value = 0
        self.dut.adc_waiting_for_trig.value = 0

        # Interface FIFOs
        self.cmd_buf = fwft_fifo_model(dut, "CMD_FIFO_MODEL", DEPTH=16)
        self.data_buf = fwft_fifo_model(dut, "DATA_FIFO_MODEL", DEPTH=16)

        # Queue to keep track of current executing command in the DUT
        self.executing_cmd_queue = deque()

        # For tracking expected trigger timing data
        self.expected_trig_timer_list = []

    def get_state_name(self, state_value):
        """Get the state name from the state value."""
        state_int = int(state_value)
        return self.STATE_ENCODING.get(state_int, f'STATE_{state_int}')


    def get_cmd_name(self, cmd_value):
        """Get the command name from the command value."""
        cmd_int = int(cmd_value)
        return self.CMD_ENCODING.get(cmd_int, f'CMD_{cmd_int}')
    
    async def reset(self):
        """Reset the DUT, hold reset for two clock cycles."""
        await RisingEdge(self.dut.clk)
        self.dut.resetn.value = 0
        self.dut._log.info("STARTING RESET")
        self.cmd_buf.reset()
        self.data_buf.reset()
        self.executing_cmd_queue.clear()
        self.expected_trig_timer_list.clear()

        await RisingEdge(self.dut.clk)

        await RisingEdge(self.dut.clk)
        self.dut.resetn.value = 1
        self.dut._log.info("RESET COMPLETE")

        # Check the expected output after reset
        assert int(self.dut.state.value) == 0, "DUT did not reset to initial state"
        assert int(self.dut.trig_lockout.value) == self.TRIGGER_LOCKOUT_DEFAULT, "Trigger lockout not set to default value"
        assert int(self.dut.trig_counter.value) == 0, "Trigger counter not reset to zero"
        assert int(self.dut.delay_counter.value) == 0, "Delay counter not reset to zero"
        assert int(self.dut.lockout_counter.value) == 0, "Lockout counter not reset to zero"
        assert int(self.dut.trig_out.value) == 0, "Trigger output not reset to zero"
        assert int(self.dut.bad_cmd.value) == 0, "Bad command not reset to zero"
        assert int(self.dut.data_buf_overflow.value) == 0, "Data buffer overflow not reset to zero"
        assert int(self.dut.data_word_wr_en.value) == 0, "Data word read enable not reset to zero"
        assert int(self.dut.data_word.value) == 0, "Data word not reset to zero"

    async def monitor_cmd_done(self):
        """Check if cmd_done signal is asserted when it should be."""
        while True:
            await RisingEdge(self.dut.clk)
            await ReadOnly()

            # S_IDLE
            if int(self.dut.state.value) == 1 and not int(self.dut.cmd_buf_empty.value):
                assert int(self.dut.cmd_done.value) == 1, \
                    f"cmd_done should be asserted when state is S_IDLE and cmd_buf_empty is 0, but got {int(self.dut.cmd_done.value)}"
            
            # S_SYNC_CH
            if int(self.dut.state.value) == 2 and int(self.dut.all_waiting.value) == 1:
                assert int(self.dut.cmd_done.value) == 1, \
                    f"cmd_done should be asserted when state is S_SYNC_CH and all_waiting is 1, but got {int(self.dut.cmd_done.value)}"
                
            # S_EXPECT_TRIG
            if int(self.dut.state.value) == 3 and int(self.dut.trig_counter.value) == 0:
                assert int(self.dut.cmd_done.value) == 1, \
                    f"cmd_done should be asserted when state is S_EXPECT_TRIG and trig_counter is 0, but got {int(self.dut.cmd_done.value)}"
                
            # S_DELAY
            if int(self.dut.state.value) == 4 and int(self.dut.delay_counter.value) == 0:
                assert int(self.dut.cmd_done.value) == 1, \
                    f"cmd_done should be asserted when state is S_DELAY and delay_counter is 0, but got {int(self.dut.cmd_done.value)}"
                
            # S_ERROR
            if int(self.dut.state.value) != 5 and int(self.dut.cancel.value) == 1:
                assert int(self.dut.cmd_done.value) == 1, \
                    f"cmd_done should be asserted when state is not S_ERROR and cancel is 1, but got {int(self.dut.cmd_done.value)}"
                
    # Can add here data buf overflow , trig_out and bad command monitoring if needed
    async def monitor_state_transitions(self):
        """Check if state transitions are done correctly."""
        prev_cmd_done = 0
        prev_next_cmd_state = 0
        while True:
            await RisingEdge(self.dut.clk)
            # Sample previous values
            prev_cmd_done = int(self.dut.cmd_done.value)
            prev_next_cmd_state = int(self.dut.next_cmd_state.value)

            await ReadOnly()
            # next_cmd_state to state
            if prev_cmd_done == 1:
                assert int(self.dut.state.value) == prev_next_cmd_state, \
                    f"State transition error: state should be {self.get_state_name(prev_next_cmd_state)} but got {self.get_state_name(int(self.dut.state.value))}"
                
            # next_cmd_state transitions
            # next_cmd_state should be S_IDLE if cmd_buf_empty is 1
            if int(self.dut.cmd_buf_empty.value) == 1: 
                assert int(self.dut.next_cmd_state.value) == 1, \
                    f"next_cmd_state should be S_IDLE when cmd_buf_empty is 1, but got {self.get_state_name(int(self.dut.next_cmd_state.value))}"
                
            # next_cmd_state should be S_IDLE when cmd_type is CMD_CANCEL or CMD_FORCE_TRIG
            elif int(self.dut.cmd_type.value) == 7 or int(self.dut.cmd_type.value) == 5:
                assert int(self.dut.next_cmd_state.value) == 1, \
                    f"next_cmd_state should be S_IDLE when cmd_type is CMD_CANCEL or CMD_FORCE_TRIG, but got {self.get_state_name(int(self.dut.next_cmd_state.value))}"
                
            # next_cmd_state should be S_IDLE or S_ERROR if cmd_type is CMD_SET_LOCKOUT and cmd_val is above or equal to TRIGGER_LOCKOUT_MIN or below it respectively
            elif int(self.dut.cmd_type.value) == 2:
                if int(self.dut.cmd_val.value) >= self.TRIGGER_LOCKOUT_MIN:
                    assert int(self.dut.next_cmd_state.value) == 1, \
                        f"next_cmd_state should be S_IDLE when cmd_type is CMD_SET_LOCKOUT and cmd_val is above or equal to TRIGGER_LOCKOUT_MIN, but got {self.get_state_name(int(self.dut.next_cmd_state.value))}"
                else:
                    assert int(self.dut.next_cmd_state.value) == 5, \
                        f"next_cmd_state should be S_ERROR when cmd_type is CMD_SET_LOCKOUT and cmd_val is below TRIGGER_LOCKOUT_MIN, but got {self.get_state_name(int(self.dut.next_cmd_state.value))}"
            
            # next_cmd_state should be S_IDLE or S_SYNC_CH if cmd_type is CMD_SYNC_CH and all_waiting is 1 or 0 respectively
            elif int(self.dut.cmd_type.value) == 1:
                if int(self.dut.all_waiting.value) == 1:
                    assert int(self.dut.next_cmd_state.value) == 1, \
                        f"next_cmd_state should be S_IDLE when cmd_type is CMD_SYNC_CH and all_waiting is 1, but got {self.get_state_name(int(self.dut.next_cmd_state.value))}"
                else:
                    assert int(self.dut.next_cmd_state.value) == 2, \
                        f"next_cmd_state should be S_SYNC_CH when cmd_type is CMD_SYNC_CH and all_waiting is 0, but got {self.get_state_name(int(self.dut.next_cmd_state.value))}"
                    
            # next_cmd_state should be S_EXPECT_TRIG or S_IDLE if cmd_type is CMD_EXPECT_EXT_TRIG and cmd_val is not 0 or 0 respectively
            elif int(self.dut.cmd_type.value) == 3:
                if int(self.dut.cmd_val.value) != 0:
                    assert int(self.dut.next_cmd_state.value) == 3, \
                        f"next_cmd_state should be S_EXPECT_TRIG when cmd_type is CMD_EXPECT_EXT_TRIG and cmd_val is not 0, but got {self.get_state_name(int(self.dut.next_cmd_state.value))}"
                else:
                    assert int(self.dut.next_cmd_state.value) == 1, \
                        f"next_cmd_state should be S_IDLE when cmd_type is CMD_EXPECT_EXT_TRIG and cmd_val is 0, but got {self.get_state_name(int(self.dut.next_cmd_state.value))}"
            
            # next_cmd_state should be S_DELAY or S_IDLE if cmd_type is CMD_DELAY and cmd_val is not 0 or 0 respectively
            elif int(self.dut.cmd_type.value) == 4:
                if int(self.dut.cmd_val.value) != 0:
                    assert int(self.dut.next_cmd_state.value) == 4, \
                        f"next_cmd_state should be S_DELAY when cmd_type is CMD_DELAY and cmd_val is not 0, but got {self.get_state_name(int(self.dut.next_cmd_state.value))}"
                else:
                    assert int(self.dut.next_cmd_state.value) == 1, \
                        f"next_cmd_state should be S_IDLE when cmd_type is CMD_DELAY and cmd_val is 0, but got {self.get_state_name(int(self.dut.next_cmd_state.value))}"
            
            # next_cmd_state should be S_ERROR otherwise
            else:
                assert int(self.dut.next_cmd_state.value) == 5, \
                    f"next_cmd_state should be S_ERROR for invalid cmd_type, but got {self.get_state_name(int(self.dut.next_cmd_state.value))}"
    
    async def send_commands(self, cmd_list):
        """
        Send a list of commands to the DUT via the command buffer model.
        If the command buffer is full, retry writing the same command on the next clock cycle.
        """
        self.dut._log.info(f"Sending {len(cmd_list)} commands to command buffer")

        for cmd in cmd_list:
            written = False
            while not written:
                await RisingEdge(self.dut.clk)
                # If buffer not full, write and move to next command
                if not self.cmd_buf.is_full():
                    self.cmd_buf.write_item(cmd)
                    written = True
                else:
                    # Buffer full: wait and retry on next cycle
                    self.dut._log.debug("cmd_buf full, retrying write on next cycle")

        self.dut._log.info(f"All {len(cmd_list)} commands sent to command buffer")

    async def command_buf_model(self):
        """
        Model of the command buffer. Connected to DUT's cmd_word_rd_en, cmd_word, and cmd_buf_empty.
        The DUT will read commands from this buffer.
        """
        while True:
            await RisingEdge(self.dut.clk)

            # Update Buffer status signals
            self.dut.cmd_buf_empty.value = 1 if self.cmd_buf.is_empty() else 0

            # FWFT behavior: always present the next item on cmd_word
            fwft_data = self.cmd_buf.peek_item() if not self.cmd_buf.is_empty() else None
            self.dut.cmd_word.value = fwft_data if fwft_data is not None else 0

            await ReadOnly() # Wait for combinational logic to settle
            # Buffer reads
            if self.dut.cmd_word_rd_en.value == 1 and not self.cmd_buf.is_empty():
                self.executing_cmd_queue.append(self.cmd_buf.pop_item())

    def command_word_generator(self, cmd_type, cmd_value):
        """
        Generate a command word from command type and value.
        """
        cmd_word = (cmd_type << 29) | (cmd_value & 0x1FFFFFFF)
        return cmd_word
    
    def command_word_decoder(self, cmd_word):
        """
        Decode a command word into command type and value.
        """
        cmd_type = (cmd_word >> 29) & 0x7
        cmd_value = cmd_word & 0x1FFFFFFF
        return cmd_type, cmd_value
    
    def random_command_word_generator(self, n):
        """
        Generates up to n random command words and returns them as a command list.

        - If an unexpected command is generated or CMD_SET_LOCKOUT is generated with
          an invalid value, that command is appended and generation stops (it is the
          literal last command in the returned list).
        - Chance of unexpected commands are ~5%.
        """
        cmd_list = []
        if n == 0:
            return cmd_list

        unexpected_prob = 0.05  # ~5% chance to produce an unexpected command (types 0 or 6)

        while len(cmd_list) < n:
            # Pick whether to generate an unexpected command
            if random.random() < unexpected_prob:
                cmd_type = random.choice([0, 6])  # unexpected types
            else:
                cmd_type = random.choice([1, 2, 3, 4, 5, 7])  # expected types

            # Generate cmd_value based on type
            if cmd_type == 1:  # CMD_SYNC_CH
                cmd_value = random.randint(0, self.MAX_CMD_VALUE)
            elif cmd_type == 2:  # CMD_SET_LOCKOUT
                cmd_value = random.randint(0, 50)
            elif cmd_type == 3:  # CMD_EXPECT_EXT_TRIG
                cmd_value = random.randint(0, 50)
            elif cmd_type == 4:  # CMD_DELAY
                cmd_value = random.randint(0, 50)
            elif cmd_type == 5:  # CMD_FORCE_TRIG
                cmd_value = random.randint(0, self.MAX_CMD_VALUE)
            elif cmd_type == 7:  # CMD_CANCEL
                cmd_value = random.randint(0, self.MAX_CMD_VALUE)
            else:
                # unexpected command types
                cmd_value = random.randint(0, self.MAX_CMD_VALUE)

            cmd_word = self.command_word_generator(cmd_type, cmd_value)
            cmd_list.append(cmd_word)

            # If this command is unexpected or an invalid SET_LOCKOUT, stop generation
            is_unexpected = cmd_type not in self.CMD_ENCODING
            is_invalid_lockout = (cmd_type == 2 and cmd_value < self.TRIGGER_LOCKOUT_MIN)
            if is_unexpected or is_invalid_lockout:
                # This command must be the literal last command in cmd_list
                break

        return cmd_list

    
    async def random_waiting_for_trig_driver(self):
        """Randomly drive adc_waiting_for_trig and dac_waiting_for_trig signals to random values or all 1."""
        while True:
            await RisingEdge(self.dut.clk)
            # Randomly decide to set all waiting or random waiting
            if random.random() < 0.3:  # 30% chance to set all waiting
                self.dut.adc_waiting_for_trig.value = 0xFF
                self.dut.dac_waiting_for_trig.value = 0xFF
            else:
                self.dut.adc_waiting_for_trig.value = random.randint(0, 0xFF)
                self.dut.dac_waiting_for_trig.value = random.randint(0, 0xFF)

    async def random_ext_trig_driver(self):
        """Randomly drive ext_trig signal high for one clock cycle."""
        while True:
            await RisingEdge(self.dut.clk)
            if random.random() < 0.3:  # 30% chance to trigger
                self.dut.ext_trig.value = 1
                await RisingEdge(self.dut.clk)
                self.dut.ext_trig.value = 0

    async def executing_command_scoreboard(self, num_of_commands=0):
        """
        Scoreboard to keep track of the currently executing command in the DUT.
        This is used to verify that the DUT is executing the correct command.
        """
        if num_of_commands == 0:
            self.dut._log.info("Executing command scoreboard finished: 0 commands expected.")
            return

        forked_tasks = []
        commands_processed = 0
        self.dut._log.info(f"Starting executing command scoreboard, expecting to process {num_of_commands} commands.")

        # Loop until the expected number of commands have been processed
        while commands_processed < num_of_commands:
            await RisingEdge(self.dut.clk)
            await ReadOnly()

            if len(self.executing_cmd_queue) > 0:
                commands_processed += 1

                executing_cmd = self.executing_cmd_queue.popleft()
                cmd_type, cmd_value = self.command_word_decoder(executing_cmd)

                dut_cmd_type = int(self.dut.cmd_type.value)
                dut_cmd_val = int(self.dut.cmd_val.value)

                assert cmd_type == dut_cmd_type, \
                    f"Executing command type mismatch: expected {self.get_cmd_name(cmd_type)} but got {self.get_cmd_name(dut_cmd_type)}"
                assert cmd_value == dut_cmd_val, \
                    f"Executing command value mismatch: expected {cmd_value} but got {dut_cmd_val}"
                
                # Fork scoreboard for the specific command type and save the task
                task = None
                command_i = commands_processed - 1

                if cmd_type == 1:
                    task = cocotb.start_soon(self.cmd_sync_ch_scoreboard(cmd_value, command_i))
                elif cmd_type == 2:
                    task = cocotb.start_soon(self.cmd_set_lockout_scoreboard(cmd_value, command_i))
                elif cmd_type == 3:
                    task = cocotb.start_soon(self.cmd_expect_ext_trig_scoreboard(cmd_value, command_i))
                elif cmd_type == 4:
                    task = cocotb.start_soon(self.cmd_delay_scoreboard(cmd_value, command_i))
                elif cmd_type == 5:
                    task = cocotb.start_soon(self.cmd_force_trig_scoreboard(cmd_value, command_i))
                elif cmd_type == 7:
                    task = cocotb.start_soon(self.cmd_cancel_scoreboard(cmd_value, command_i))
                else:
                    task = cocotb.start_soon(self.cmd_unexpected_scoreboard(cmd_type, cmd_value, command_i))
                
                if task:
                    forked_tasks.append(task)
        
        # Wait for all forked scoreboard tasks to complete their work.
        self.dut._log.info(f"All {num_of_commands} commands have been forked. Waiting for {len(forked_tasks)} scoreboards to complete.")
        if forked_tasks:
            await Combine(*forked_tasks)
        
        self.dut._log.info("All forked scoreboards have completed. Scoreboard finished.")

    async def cmd_sync_ch_scoreboard(self, cmd_value, command_i):
        """
        Scoreboard to verify SYNC_CH command.
        """
        self.dut._log.info(f"Verifying SYNC_CH command for command index:{command_i} with value {cmd_value}")
        # For SYNC_CH command, we expect the state to transition to S_IDLE if all_waiting is 1
        cycles_waiting = 0
        adc_waiting = int(self.dut.adc_waiting_for_trig.value) & 0xFF
        dac_waiting = int(self.dut.dac_waiting_for_trig.value) & 0xFF
        expected_all_waiting = 1 if (adc_waiting == 0xFF and dac_waiting == 0xFF) else 0

        if expected_all_waiting == 1:
            assert int(self.dut.all_waiting.value) == 1, \
                f"For command index:{command_i} all_waiting should be 1, but got {int(self.dut.all_waiting.value)}"
            assert int(self.dut.next_cmd_state.value) == 1, \
                f"For command index:{command_i} next_cmd_state should be S_IDLE, but got {self.get_state_name(int(self.dut.next_cmd_state.value))}"
            assert int(self.dut.do_trig.value) == 1, \
                f"For command index:{command_i} when all waiting do_trig should be 1, but got {int(self.dut.do_trig.value)}"
        else:
            while True:
                await RisingEdge(self.dut.clk)
                await ReadOnly()

                # Cancel exit condition
                if int(self.dut.cancel.value) == 1:
                    self.dut._log.info(f"For command index:{command_i} Command was cancelled during SYNC_CH waiting.")
                    return
                
                cycles_waiting += 1
                adc_waiting = int(self.dut.adc_waiting_for_trig.value) & 0xFF
                dac_waiting = int(self.dut.dac_waiting_for_trig.value) & 0xFF
                expected_all_waiting = 1 if (adc_waiting == 0xFF and dac_waiting == 0xFF) else 0

                if expected_all_waiting == 1:
                    self.dut._log.info(f"For command index:{command_i} all_waiting should now be 1 after {cycles_waiting} cycles")
                    self.dut._log.info(f"For command index:{command_i} adc_waiting: {adc_waiting:08b}, dac_waiting: {dac_waiting:08b}")
                    assert int(self.dut.all_waiting.value) == 1, \
                        f"For command index:{command_i} all_waiting should be 1, but got {int(self.dut.all_waiting.value)}"
                    assert int(self.dut.do_trig.value) == 1, \
                        f"For command index:{command_i} when all waiting do_trig should be 1, but got {int(self.dut.do_trig.value)}"
                    break

    async def cmd_set_lockout_scoreboard(self, cmd_value, command_i):
        """
        Scoreboard to verify SET_LOCKOUT command.
        """
        self.dut._log.info(f"Verifying SET_LOCKOUT command for command index:{command_i} with value {cmd_value}")
        self.dut._log.info(f"TRIGGER_LOCKOUT_MIN is {self.TRIGGER_LOCKOUT_MIN}, TRIGGER_LOCKOUT_DEFAULT is {self.TRIGGER_LOCKOUT_DEFAULT}")

        expected_lockout = cmd_value if cmd_value >= self.TRIGGER_LOCKOUT_MIN else int(self.dut.trig_lockout.value)

        await RisingEdge(self.dut.clk)  # Wait one clock cycle for the lockout to be updated
        await ReadOnly()
        self.dut._log.info(f"For command index:{command_i} Expected lockout: {expected_lockout}, DUT lockout: {int(self.dut.trig_lockout.value)}")
        assert int(self.dut.trig_lockout.value) == expected_lockout, \
            f"For command index:{command_i} Trigger lockout mismatch: expected {expected_lockout} but got {int(self.dut.trig_lockout.value)}"
        
        if cmd_value < self.TRIGGER_LOCKOUT_MIN:
            assert int(self.dut.state.value) == 5, \
                f"For command index:{command_i} State should be S_ERROR for invalid lockout value, but got {self.get_state_name(int(self.dut.state.value))}"
            assert int(self.dut.bad_cmd.value) == 1, \
                f"For command index:{command_i} bad_cmd should be asserted for invalid lockout value, but got {int(self.dut.bad_cmd.value)}"

    async def cmd_expect_ext_trig_scoreboard(self, cmd_value, command_i):
        """
        Scoreboard to verify EXPECT_EXT_TRIG command.
        """
        self.dut._log.info(f"Verifying EXPECT_EXT_TRIG command for command index:{command_i} with value {cmd_value}")

        expected_trig_counter = cmd_value 
        num_of_trigs_done = 0
        num_of_expected_trigs = expected_trig_counter

        # Wait one clock cycle for the trig_counter to be updated
        await RisingEdge(self.dut.clk)
        await ReadOnly()
        self.dut._log.info(f"For command index:{command_i} Expected trig_counter: {expected_trig_counter}, DUT trig_counter: {int(self.dut.trig_counter.value)}")
        assert int(self.dut.trig_counter.value) == expected_trig_counter, \
            f"For command index:{command_i} Trigger counter mismatch: expected {expected_trig_counter} but got {int(self.dut.trig_counter.value)}"
        
        # Cancel exit condition
        if int(self.dut.cancel.value) == 1:
            self.dut._log.info(f"For command index:{command_i} Command was cancelled before starting trigger countdown.")
            return
        elif expected_trig_counter == 0:
            self.dut._log.info(f"For command index:{command_i} Command value is 0, no triggers expected.")
            assert int(self.dut.state.value) == 1, \
                f"For command index:{command_i} State should be S_IDLE for 0 trigger count, but got {self.get_state_name(int(self.dut.state.value))}"
            return
        
        while True:
            await RisingEdge(self.dut.clk)
            previous_do_trig = int(self.dut.do_trig.value)
            previous_lockout_counter = int(self.dut.lockout_counter.value)
            await ReadOnly()

            # Cancel exit condition
            if int(self.dut.cancel.value) == 1:
                self.dut._log.info(f"For command index:{command_i} Command was cancelled during trigger countdown.")
                return

            # Assert that we are still in EXPECT_TRIG state
            assert int(self.dut.state.value) == 3, \
                f"For command index:{command_i} State should be S_EXPECT_TRIG, but got {self.get_state_name(int(self.dut.state.value))}"
            
            # If there is an ext_trig and lockout_counter is 0, do_trig should be asserted
            if int(self.dut.ext_trig.value) == 1 and int(self.dut.lockout_counter.value) == 0:
                assert int(self.dut.do_trig.value) == 1, \
                    f"For command index:{command_i} do_trig should be asserted when ext_trig is 1 and lockout_counter is 0, but got {int(self.dut.do_trig.value)}"
            
            # When do_trig is asserted, trig_counter should decrement by 1 in the next cycle if it is greater than 0
            if previous_do_trig == 1 and expected_trig_counter > 0:
                expected_trig_counter -= 1
                num_of_trigs_done += 1
                self.dut._log.info(f"For command index:{command_i} Expected trig_counter decremented to: {expected_trig_counter}")
                assert int(self.dut.trig_counter.value) == expected_trig_counter, \
                    f"For command index:{command_i} Trigger counter mismatch after do_trig: expected {expected_trig_counter} but got {int(self.dut.trig_counter.value)}"
            
            # When do trig is asserted, lockout_counter should be set to trig_lockout in the next cycle
            if previous_do_trig == 1:
                assert self.dut.lockout_counter.value == self.dut.trig_lockout.value , \
                    f"For command index:{command_i} Lockout counter mismatch after do_trig: expected {int(self.dut.trig_lockout.value)} but got {int(self.dut.lockout_counter.value)}"
            elif previous_lockout_counter > 0:
                # If lockout_counter is greater than 0, it should decrement by 1 in the next cycle
                assert int(self.dut.lockout_counter.value) == previous_lockout_counter - 1, \
                    f"For command index:{command_i} Lockout counter mismatch: expected {previous_lockout_counter - 1} but got {int(self.dut.lockout_counter.value)}"
                
            if num_of_trigs_done == num_of_expected_trigs:
                assert int(self.dut.trig_counter.value) == 0, \
                    f"For command index:{command_i} Trigger counter should be 0 after all expected triggers are done, but got {int(self.dut.trig_counter.value)}"
                self.dut._log.info(f"For command index:{command_i} All expected triggers are done.")
                self.dut._log.info(f"num_of_trigs_done: {num_of_trigs_done}, num_of_expected_trigs: {num_of_expected_trigs}")
                break


    async def cmd_delay_scoreboard(self, cmd_value, command_i):
        """ Scoreboard to verify DELAY command."""
        self.dut._log.info(f"Verifying DELAY command for command index:{command_i} with value {cmd_value}")

        expected_delay_counter = cmd_value

        # Wait one clock cycle for the delay_counter to be updated
        await RisingEdge(self.dut.clk)
        await ReadOnly()
        self.dut._log.info(f"For command index:{command_i} Expected delay_counter: {expected_delay_counter}, DUT delay_counter: {int(self.dut.delay_counter.value)}")
        assert int(self.dut.delay_counter.value) == expected_delay_counter, \
            f"For command index:{command_i} Delay counter mismatch: expected {expected_delay_counter} but got {int(self.dut.delay_counter.value)}"
        
        # Cancel exit condition
        if int(self.dut.cancel.value) == 1:
            self.dut._log.info(f"For command index:{command_i} Command was cancelled before starting delay countdown.")
            return
        elif expected_delay_counter == 0:
            self.dut._log.info(f"For command index:{command_i} Command value is 0, no delay expected.")
            assert int(self.dut.state.value) == 1, \
                f"For command index:{command_i} State should be S_IDLE for 0 delay, but got {self.get_state_name(int(self.dut.state.value))}"
            return
        
        while True:
            await RisingEdge(self.dut.clk)
            await ReadOnly()

            # Cancel exit condition
            if int(self.dut.cancel.value) == 1:
                self.dut._log.info(f"For command index:{command_i} Command was cancelled during delay countdown.")
                return

            # Assert that we are still in DELAY state
            assert int(self.dut.state.value) == 4, \
                f"For command index:{command_i} State should be S_DELAY, but got {self.get_state_name(int(self.dut.state.value))}"
            
            if expected_delay_counter > 0:
                expected_delay_counter -= 1
                self.dut._log.info(f"For command index:{command_i} Expected delay_counter decremented to: {expected_delay_counter}")
                assert int(self.dut.delay_counter.value) == expected_delay_counter, \
                    f"For command index:{command_i} Delay counter mismatch: expected {expected_delay_counter} but got {int(self.dut.delay_counter.value)}"
            
            if expected_delay_counter == 0:
                assert int(self.dut.delay_counter.value) == 0, \
                    f"For command index:{command_i} Delay counter should be 0 after countdown, but got {int(self.dut.delay_counter.value)}"
                self.dut._log.info(f"For command index:{command_i} Delay countdown complete.")
                break

    async def cmd_force_trig_scoreboard(self, cmd_value, command_i):
        """ Scoreboard to verify FORCE_TRIG command."""
        self.dut._log.info(f"Verifying FORCE_TRIG command for command index:{command_i} with value {cmd_value}")
        assert int(self.dut.do_trig.value) == 1, "do_trig should be asserted immediately for FORCE_TRIG command"

        await RisingEdge(self.dut.clk)
        await ReadOnly()
        assert int(self.dut.state.value) == 1, \
            f"For command index:{command_i} State should be S_IDLE after FORCE_TRIG command, but got {self.get_state_name(int(self.dut.state.value))}"

    async def cmd_cancel_scoreboard(self, cmd_value, command_i):
        """ Scoreboard to verify CANCEL command."""
        self.dut._log.info(f"Verifying CANCEL command for command index:{command_i} with value {cmd_value}")
        assert int(self.dut.cancel.value) == 1, "cancel should be asserted immediately for CANCEL command"

        await RisingEdge(self.dut.clk)
        await ReadOnly()
        assert int(self.dut.trig_counter.value) == 0, \
            f"For command index:{command_i} trig_counter should be 0 after CANCEL command, but got {int(self.dut.trig_counter.value)}"
        
        assert int(self.dut.delay_counter.value) == 0, \
            f"For command index:{command_i} delay_counter should be 0 after CANCEL command, but got {int(self.dut.delay_counter.value)}"
        
        assert int(self.dut.trig_out.value) == 0, \
            f"For command index:{command_i} trig_out should be 0 after CANCEL command, but got {int(self.dut.trig_out.value)}"
        
        assert int(self.dut.state.value) == 1, \
            f"For command index:{command_i} State should be S_IDLE after CANCEL command, but got {self.get_state_name(int(self.dut.state.value))}"
        
    async def cmd_unexpected_scoreboard(self, cmd_type, cmd_value, command_i):
        """ Scoreboard to catch any unexpected commands being executed."""
        self.dut._log.info(f"Verifying UNEXPECTED command for command index:{command_i} with type {cmd_type} and value {cmd_value}")
        assert cmd_type not in self.CMD_ENCODING, \
            f"For command index:{command_i} Unexpected command type detected: {cmd_type}"
        
        await RisingEdge(self.dut.clk)
        await ReadOnly()
        assert int(self.dut.state.value) == 5, \
            f"For command index:{command_i} State should be S_ERROR for unexpected command, but got {self.get_state_name(int(self.dut.state.value))}"
        assert int(self.dut.bad_cmd.value) == 1, \
            f"For command index:{command_i} bad_cmd should be asserted for unexpected command, but got {int(self.dut.bad_cmd.value)}"

    async def data_buf_model(self):
        """
        Model of the data FIFO. Connected to DUT's data_word_wr_en, data_word, data_buf_full, and data_buf_almost_full.
        The DUT will write Trigger timing data to this FIFO.
        """
        while True:
            await RisingEdge(self.dut.clk)
            await ReadWrite() # Wait for combinational logic to settle

            # Buffer writes
            if self.dut.data_word_wr_en.value == 1:
                data = int(self.dut.data_word.value)
                self.data_buf.write_item(data)

            # Update Buffer status signals
            self.dut.data_buf_full.value = 1 if self.data_buf.is_full() else 0
            self.dut.data_buf_almost_full.value = 1 if self.data_buf.is_almost_full() else 0

    async def trig_timer_tracker(self):
        """Generate expected trigger timing data based on do_trig signal and store in a list for later comparison."""
        expected_trig_timer = 0

        while True:
            await RisingEdge(self.dut.clk)
            prev_do_trig = int(self.dut.do_trig.value)

            await ReadOnly()
            sample_cond = int(self.dut.do_trig.value) == 1 and int(self.dut.data_buf_full.value) == 0 and int(self.dut.data_buf_almost_full.value) == 0

            if prev_do_trig == 1 and expected_trig_timer == 0:
                expected_trig_timer = 1
            elif expected_trig_timer > 0:
                expected_trig_timer += 1

            if sample_cond:
                first_data_word = expected_trig_timer & 0xFFFFFFFF
                second_data_word = (expected_trig_timer >> 32) & 0xFFFFFFFF
                await RisingEdge(self.dut.clk)
                expected_trig_timer += 1
                await RisingEdge(self.dut.clk)
                expected_trig_timer += 1
                self.dut._log.info(f"Expected Trigger Timing Data Generated: First Word:0x{first_data_word:08X}, Second Word:0x{second_data_word:08X}")
                self.expected_trig_timer_list.append((first_data_word, second_data_word))


    async def data_buf_scoreboard(self):
        """
        Each cycle read from the data buffer model and compare with expected trigger timing data.
        Only should be started after all commands have been sent and processed.
        """
        self.dut._log.info("Starting data buffer scoreboard for trig_timer.")
        num_of_data_words_checked = 0

        while 2*len(self.expected_trig_timer_list) > 0 or not self.data_buf.is_empty():
            await RisingEdge(self.dut.clk)
            await ReadOnly()

            read_data = int(self.data_buf.pop_item())

            if num_of_data_words_checked % 2 == 0:
                expected_first_word, expected_second_word = self.expected_trig_timer_list.pop(0)

            if num_of_data_words_checked % 2 == 0:
                # First word
                assert read_data == expected_first_word, \
                    f"Data buffer mismatch on first word: expected 0x{expected_first_word:08X} but got 0x{read_data:08X}"
                self.dut._log.info(f"Data buffer match on first word: 0x{read_data:08X}")
            else:
                # Second word
                assert read_data == expected_second_word, \
                    f"Data buffer mismatch on second word: expected 0x{expected_second_word:08X} but got 0x{read_data:08X}"
                self.dut._log.info(f"Data buffer match on second word: 0x{read_data:08X}")

            num_of_data_words_checked += 1
        
        self.dut._log.info("Data buffer scoreboard for trig_timer finished.")
        

            
