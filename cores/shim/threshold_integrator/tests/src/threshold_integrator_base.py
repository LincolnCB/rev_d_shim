import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, ReadWrite
from cocotb.binary import BinaryValue
import random
from collections import deque

class threshold_integrator_base:
    
    # State encoding dictionary
    STATES = {
        0: "IDLE",
        1: "SETUP",
        2: "WAIT",
        3: "RUNNING",
        4: "OUT_OF_BOUNDS",
        5: "ERROR"
    }

    def __init__(self, dut, clk_period=4, time_unit="ns"):
        self.dut = dut
        self.clk_period = clk_period
        self.time_unit = time_unit

        # Initialize clock
        cocotb.start_soon(Clock(dut.clk, clk_period, time_unit).start())

        # Initialize input signals
        self.dut.enable.value = 0
        self.dut.window.value = 0
        self.dut.threshold_average.value = 0
        self.dut.sample_core_done.value = 0
        self.dut.abs_sample_concat.value = 0

        # Inputs to drive
        self.driven_window_value = 0
        self.driven_threshold_average_value = 0

        # Channel Queues
        self.TOTAL_FIFO_DEPTH = 1024
        self.channel_queues = [deque() for _ in range(8)]

        # Expected values
        self.expected_chunk_width = 0
        self.expected_chunk_size = 0
        self.expected_number_of_elements_in_fifo = 0


    def get_state_name(self, state_value):
        """Get the name of the state based on its integer value from STATES dictionary."""
        state_int = int(state_value)
        return self.STATES.get(state_int, f"UNKNOWN_STATE({state_int})")
        
    async def reset(self):
        """Reset the DUT, hold reset for two clk cycles."""
        await RisingEdge(self.dut.clk)
        self.dut.resetn.value = 0
        self.driven_window_value = 0
        self.driven_threshold_average_value = 0
        self.expected_chunk_size = 0
        self.expected_chunk_width = 0
        self.expected_number_of_elements_in_fifo = 0

        for q in self.channel_queues:
            q.clear()

        self.dut._log.info("STARTING RESET")
        await RisingEdge(self.dut.clk)
        await RisingEdge(self.dut.clk)
        self.dut.resetn.value = 1
        self.dut._log.info("RESET COMPLETE")

    async def state_transition_monitor_and_scoreboard(self):
        """Monitor the state transitions and score them against expected values."""
        previous_state_value = 0
        previous_reset_value = 0
        previous_fifo_full_value = 0
        previous_wr_en_value = 0
        previous_fifo_empty_value = 0
        previous_rd_en_value = 0
        previous_channel_over_thresh_value = 0

        while True:
            await RisingEdge(self.dut.clk)
            previous_reset_value = int(self.dut.resetn.value)
            previous_state_value = int(self.dut.state.value)
            previous_fifo_full_value = int(self.dut.fifo_full.value)
            previous_wr_en_value = int(self.dut.wr_en.value)
            previous_fifo_empty_value = int(self.dut.fifo_empty.value)
            previous_rd_en_value = int(self.dut.rd_en.value)
            previous_channel_over_thresh_value = int(self.dut.channel_over_thresh.value)

            await ReadOnly()

            # reset to IDLE
            if previous_reset_value == 0 and int(self.dut.resetn.value) == 1:
                assert int(self.dut.state.value) == 0,\
                    f"Expected state after reset: 0 (IDLE), got: {int(self.dut.state.value)} ({self.get_state_name(self.dut.state.value)})"

            # IDLE to OUT_OF_BOUNDS
            if previous_state_value == 0 and int(self.dut.over_thresh) == 1:
                assert int(self.dut.state.value) == 4,\
                    f"Expected state after over_thresh in IDLE: 4 (OUT_OF_BOUNDS), got: {int(self.dut.state.value)} ({self.get_state_name(self.dut.state.value)})"
                
                assert int(self.dut.window.value) < 2**11,\
                    f"Expected window value in OUT_OF_BOUNDS: < 2**11, got: {int(self.dut.window.value)}"

            # RUNNING to OUT_OF_BOUNDS
            if bool(previous_channel_over_thresh_value) and previous_state_value == 3:

                self.dut._log.info(f"Current State: {int(self.dut.state.value)} ({self.get_state_name(self.dut.state.value)})")
                self.dut._log.info(f"channel_over_thresh value: {previous_channel_over_thresh_value}")

                assert int(self.dut.state.value) == 4, \
                    f"Expected state after channel_over_thresh: 4 (OUT_OF_BOUNDS), got: {int(self.dut.state.value)} ({self.get_state_name(self.dut.state.value)})"
                
                assert self.dut.over_thresh.value == 1, \
                    f"Expected over_thresh: 1, got: {int(self.dut.over_thresh.value)}"

            # RUNNING to ERROR
            # FIFO overflow
            if (previous_fifo_full_value and previous_wr_en_value and previous_state_value == 3):

                self.dut._log.info(f"Current State: {int(self.dut.state.value)} ({self.get_state_name(self.dut.state.value)})")
                self.dut._log.info(f"fifo_full value: {previous_fifo_full_value}")
                self.dut._log.info(f"wr_en value: {previous_wr_en_value}")
                self.dut._log.info(f"Expected number of elements in MODEL FIFO: {self.expected_number_of_elements_in_fifo}")

                assert self.dut.err_overflow.value == 1, \
                    f"Expected err_overflow: 1, got: {int(self.dut.err_overflow.value)}"
                
                assert int(self.dut.state.value) == 5, \
                    f"Expected state after overflow: 5 (ERROR), got: {int(self.dut.state.value)} ({self.get_state_name(self.dut.state.value)})"
                
                assert self.expected_number_of_elements_in_fifo >=1024, \
                    f"Expected number of elements in the MODEL FIFO to be >= 1024, got: {self.expected_number_of_elements_in_fifo}"
            
            # FIFO underflow
            if (previous_fifo_empty_value and previous_rd_en_value and previous_state_value == 3):

                self.dut._log.info(f"Current State: {int(self.dut.state.value)} ({self.get_state_name(self.dut.state.value)})")
                self.dut._log.info(f"fifo_empty value: {previous_fifo_empty_value}")
                self.dut._log.info(f"rd_en value: {previous_rd_en_value}")

                assert self.dut.err_underflow.value == 1, \
                    f"Expected err_underflow: 1, got: {int(self.dut.err_underflow.value)}"
                
                assert int(self.dut.state.value) == 5, \
                    f"Expected state after underflow: 5 (ERROR), got: {int(self.dut.state.value)} ({self.get_state_name(self.dut.state.value)})"

    async def idle_to_running_state(self, window_value=(2**16)-1, threshold_average_value=(2**10)-1):
        """
        Transition from IDLE to RUNNING state, will leave the DUT at the first cycle of RUNNING state.
        Takes window_value and threshold_average_value as parameters.
        Window value should not be less than 2**11, otherwise this function will work unexpectedly.
        """

        await RisingEdge(self.dut.clk)
        await ReadWrite()

        # After reset, DUT should be in IDLE state
        assert int(self.dut.state.value) == 0, f"Expected state after reset: 0 (IDLE), got: {int(self.dut.state.value)} ({self.get_state_name(self.dut.state.value)})"

        # Enable the DUT
        self.dut.enable.value = 1 
        
        # Drive the window_value and threshold_average_value and store them
        self.driven_window_value = window_value
        self.dut.window.value = self.driven_window_value

        self.driven_threshold_average_value = threshold_average_value
        self.dut.threshold_average.value = self.driven_threshold_average_value

        # DUT should be in SETUP state
        await RisingEdge(self.dut.clk)
        await ReadWrite()
        self.dut.enable.value = 0

        # Calculate the expected chunk width and size
        over_thresh_expected = False
        for i in range(31, -1, -1):
            if (self.driven_window_value >> i) & 1:
                if i >= 11:
                    self.expected_chunk_width = i - 10
                    break
                else:
                    over_thresh_expected = True
                    break
        
        if over_thresh_expected:
            self.dut._log.info("Disallowed size of window! This function will not work as expected.")
            self.expected_chunk_size = 0
            self.expected_chunk_width = 0
        else:
            self.expected_chunk_size = (2**self.expected_chunk_width) - 1


        # Assertions
        assert int(self.dut.window.value) == self.driven_window_value, \
            f"Expected window value: {self.driven_window_value}, got: {int(self.dut.window.value)}"
        
        assert int(self.dut.threshold_average.value) == self.driven_threshold_average_value, \
            f"Expected threshold_average value: {self.driven_threshold_average_value}, got: {int(self.dut.threshold_average.value)}"
        
        self.dut._log.info(f"Window: {self.driven_window_value}")
        self.dut._log.info(f"Threshold Average: {self.driven_threshold_average_value}")
        self.dut._log.info(f"Expected Chunk Width: {self.expected_chunk_width}")
        self.dut._log.info(f"Current Chunk Width: {int(self.dut.chunk_width.value)}")
 
        assert int(self.dut.state.value) == 1, \
            f"Expected state after enabling: 1 (SETUP), got: {int(self.dut.state.value)} ({self.get_state_name(self.dut.state.value)})"

        assert int(self.dut.window_reg.value) == self.driven_window_value >> 4, \
            f"Expected window_reg: {self.driven_window_value >> 4}, got: {int(self.dut.window_reg.value)}"

        assert int(self.dut.threshold_average_shift.value) == self.driven_threshold_average_value, \
            f"Expected threshold_average_shift: {self.driven_threshold_average_value}, got: {int(self.dut.threshold_average_shift.value)}"
        
        assert int(self.dut.chunk_width.value) == self.expected_chunk_width, \
            f"Expected chunk_width: {self.expected_chunk_width}, got: {int(self.dut.chunk_width.value)}"

        # Wait for the DUT to transition to WAIT state
        while True:
            await RisingEdge(self.dut.clk)
            await ReadWrite()
            if int(self.dut.state.value) == 2:
                break
        
        self.dut._log.info(f"Expected Chunk Size: {self.expected_chunk_size}")
        self.dut._log.info(f"Current Chunk Size: {int(self.dut.chunk_size.value)}")

        # Assertions
        assert int(self.dut.state.value) == 2, \
            f"Expected state after setup: 2 (WAIT), got: {int(self.dut.state.value)} ({self.get_state_name(self.dut.state.value)})"

        assert int(self.dut.max_value.value) == self.driven_threshold_average_value * (self.driven_window_value >> 4), \
            f"Expected max_value: {self.driven_threshold_average_value * (self.driven_window_value >> 4)}, got: {int(self.dut.max_value.value)}"
        
        assert int(self.dut.chunk_size.value) == self.expected_chunk_size, \
            f"Expected chunk_size: {self.expected_chunk_size}, got: {int(self.dut.chunk_size.value)}"
        
        # Go to RUNNING state
        await RisingEdge(self.dut.clk)
        self.dut.sample_core_done.value = 1

        # Now in first cycle of RUNNING state
        await RisingEdge(self.dut.clk)
        await ReadWrite()
        self.dut.sample_core_done.value = 0

        # Assertions
        assert int(self.dut.state.value) == 3, \
            f"Expected state after sample_core_done: 3 (RUNNING), got: {int(self.dut.state.value)} ({self.get_state_name(self.dut.state.value)})"
        
        assert int(self.dut.inflow_chunk_timer.value) == int(self.dut.chunk_size.value) << 4, \
            f"Expected inflow_chunk_timer: {int(self.dut.chunk_size.value) << 4}, got: {int(self.dut.inflow_chunk_timer.value)}"
        
        assert int(self.dut.outflow_timer.value) == int(self.dut.window.value) - 1, \
            f"Expected outflow_timer: {int(self.dut.window.value) - 1}, got: {int(self.dut.outflow_timer.value)}"
        
        assert self.dut.setup_done.value == 1, \
            f"Expected setup_done: 1, got: {int(self.dut.setup_done.value)}"
        
        self.dut._log.info("Transitioned to RUNNING state successfully")
        self.dut._log.info(f"Window: {int(self.dut.window.value)}")
        self.dut._log.info(f"Threshold Average: {int(self.dut.threshold_average.value)}")

        self.dut._log.info(f"Current Chunk Size: {int(self.dut.chunk_size.value)}")
        self.dut._log.info(f"Expected Chunk Size: {self.expected_chunk_size}")

        self.dut._log.info(f"Current Chunk Width: {int(self.dut.chunk_width.value)}")
        self.dut._log.info(f"Expected Chunk Width: {self.expected_chunk_width}")

        self.dut._log.info(f"Current Max Value: {int(self.dut.max_value.value)}")
        self.dut._log.info(f"Expected Max Value: {self.driven_threshold_average_value * (self.driven_window_value >> 4)}")

        self.dut._log.info(f"INITIAL inflow_chunk_timer: {int(self.dut.inflow_chunk_timer.value)}")
        self.dut._log.info(f"INITIAL outflow_timer: {int(self.dut.outflow_timer.value)}")



    async def running_state_model_and_scoreboard(self, mode="random_abs_sample_concat_values"):
        """
        Model the RUNNING state and score the DUT against it.
        Expects the DUT to be in the first cycle of RUNNING state otherwise it will work unexpectedly.

        The mode can be: 
        "random_abs_sample_concat_values"
        "max_abs_sample_concat_values"
        "high_abs_sample_concat_values"
        "low_abs_sample_concat_values"
        "random_abs_sample_concat_values" (default)

        The default mode is "random_abs_sample_concat_values" which will generate random values for abs_sample_concat.
        max_abs_sample_concat_values mode will drive max 15 bit values for abs_sample_concat.
        high_abs_sample_concat_values mode will drive values above threshold_average.
        low_abs_sample_concat_values mode will drive values below threshold_average.
        """

        # 8 element inflow_value array to hold 15 bit values
        inflow_value = [0] * 8

        # Initialize 120-bit constructed abs_sample_concat value
        constructed_abs_sample_concat = 0

        # Initiliaze previous values for scoreboard
        previous_inflow_chunk_timer_value = int(self.dut.chunk_size.value) << 4
        previous_outflow_timer_value = int(self.dut.window.value) - 1

        previous_inflow_value = [0] * 8

        previous_expected_outflow_value = [0] * 8
        previous_expected_outflow_value_plus_one = [0] * 8
        previous_expected_outflow_remainder = [0] * 8
        previous_expected_float_outflow_value = [0] * 8

        # Initialize expected values for scoreboard
        expected_inflow_chunk_sum = [0] * 8

        expected_outflow_value = [0] * 8
        expected_outflow_value_plus_one = [0] * 8
        expected_outflow_remainder = [0] * 8
        expected_float_outflow_value = [0] * 8

        expected_fifo_out_chunk_sum= [0] * 8

        expected_sum_delta = [0] * 8
        expected_float_sum_delta = [0] * 8

        expected_total_sum = [0] * 8
        expected_float_total_sum = [0] * 8

        for _ in range((int(self.dut.window.value)-1)*2+100):  # Run 2*(window-1) cycles to see full iterations of inflow and outflow logic
            
            # If there is an over threshold condition, break the loop.
            overthresh_expected = False
            for i in range(8):
                if expected_total_sum[i] > (int(self.dut.max_value.value)):
                    overthresh_expected = True
                    self.dut._log.info(f"CHANNEL {i} OVER THRESHOLD:")
                    self.dut._log.info(f"Max Value: {int(self.dut.max_value.value)}")
                    self.dut._log.info(f"Expected total_sum[{i}]: {expected_total_sum[i]}")
                    self.dut._log.info(f"Current total_sum[{i}]: {int(self.dut.total_sum[i].value.signed_integer)}")
                    self.dut._log.info(f"Expected float_total_sum[{i}]: {expected_float_total_sum[i]}")
                    self.dut._log.info(f"Total Drift from expected sum: {expected_float_total_sum[i] - int(self.dut.total_sum[i].value.signed_integer)}")

            if overthresh_expected:
                self.dut._log.info("OVER THRESHOLD DETECTED, EXPECTING DUT TO GO TO OUT_OF_BOUNDS STATE")
                break

            # If there is an overflow condition, break the loop.
            if self.expected_number_of_elements_in_fifo > 1024:
                self.dut._log.info("MODEL FIFO OVERFLOW DETECTED, EXPECTING DUT TO GO TO ERROR STATE")
                break
                
            # Construct abs_sample_concat values every cycle
            constructed_abs_sample_concat = 0
            for i in range(8):
                if mode == "random_abs_sample_concat_values":
                    random_15_bit_value = random.randint(0, 2**15-1)
                elif mode == "max_abs_sample_concat_values":
                    random_15_bit_value = 2**15 - 1
                elif mode == "high_abs_sample_concat_values":
                    random_15_bit_value = random.randint(int(self.dut.threshold_average.value), 2**15-1)
                elif mode == "low_abs_sample_concat_values":
                    random_15_bit_value = random.randint(0, int(self.dut.threshold_average.value) - 1)

                inflow_value[i] = random_15_bit_value
                constructed_abs_sample_concat |= random_15_bit_value << (i * 15)

            # Drive the abs_sample_concat value every cycle
            self.dut.abs_sample_concat.value = constructed_abs_sample_concat

            # Log the timers here for debugging
            if previous_inflow_chunk_timer_value & 0xF == 0:
                self.dut._log.info(f"PREVIOUS inflow_chunk_timer: {previous_inflow_chunk_timer_value}")
                self.dut._log.info(f"CURRENT inflow_chunk_timer: {int(self.dut.inflow_chunk_timer.value)}")
            if previous_outflow_timer_value & 0xF == 0:
                self.dut._log.info(f"PREVIOUS outflow_timer: {previous_outflow_timer_value}")
                self.dut._log.info(f"CURRENT outflow_timer: {int(self.dut.outflow_timer.value)}")
            
            ## Inflow Logic Scoreboard
            if previous_inflow_chunk_timer_value & 0xF == 0:

                if previous_inflow_chunk_timer_value != 0:
                    # Score the inflow chunk sum
                    for i in range(8):
                        expected_inflow_chunk_sum[i] += previous_inflow_value[i]

                        self.dut._log.info(f"FOR CHANNEL {i}:")
                        self.dut._log.info(f"Previous inflow_value[{i}] that should reflect to this cycle: {previous_inflow_value[i]}")
                        self.dut._log.info(f"Expected inflow_chunk_sum[{i}] this cycle: {expected_inflow_chunk_sum[i]}")
                        self.dut._log.info(f"Current inflow_chunk_sum[{i}] this cycle: {int(self.dut.inflow_chunk_sum[i].value)}")

                        # Check if the inflow chunk sum matches the expected value
                        assert int(self.dut.inflow_chunk_sum[i].value) == expected_inflow_chunk_sum[i], \
                            f"Expected inflow_chunk_sum[{i}]: {expected_inflow_chunk_sum[i]}, got: {int(self.dut.inflow_chunk_sum[i].value)}"

                # This is when chunks sums will start getting in to the FIFO  
                else:
                    for i in range(8):
                        expected_inflow_chunk_sum[i] += previous_inflow_value[i]
                        self.channel_queues[i].append(expected_inflow_chunk_sum[i])
                        self.expected_number_of_elements_in_fifo += 1

                        self.dut._log.info(f"Number of elements in the MODEL FIFO: {self.expected_number_of_elements_in_fifo}")

                        self.dut._log.info(f"FOR CHANNEL {i}:")
                        self.dut._log.info(f"Previous inflow_value[{i}] that should reflect to this cycle: {previous_inflow_value[i]}")
                        self.dut._log.info(f"Expected queued_fifo_in_chunk_sum[{i}] this cycle: {expected_inflow_chunk_sum[i]}")
                        self.dut._log.info(f"Current queued_fifo_in_chunk_sum[{i}] this cycle: {int(self.dut.queued_fifo_in_chunk_sum[i].value)}")

                        # Check if the inflow chunk sum in FIFO matches the expected value
                        assert int(self.dut.queued_fifo_in_chunk_sum[i].value) == expected_inflow_chunk_sum[i], \
                            f"Expected queued_fifo_in_chunk_sum[{i}]: {expected_inflow_chunk_sum[i]}, got: {int(self.dut.queued_fifo_in_chunk_sum[i].value)}"
                        
                        # Inflow chunk sum should reset to 0
                        expected_inflow_chunk_sum[i] = 0
                        
                        self.dut._log.info(f"Expected inflow_chunk_sum[{i}] this cycle: {expected_inflow_chunk_sum[i]}")
                        self.dut._log.info(f"Current inflow_chunk_sum[{i}] this cycle: {int(self.dut.inflow_chunk_sum[i].value)}")

                        # Check if the inflow_chunk_sum is reset to 0
                        assert int(self.dut.inflow_chunk_sum[i].value) == 0, \
                            f"Expected inflow_chunk_sum[{i}] to be reset to 0, got: {int(self.dut.inflow_chunk_sum[i].value)}"
                    
                    # When previous_inflow_chunk_timer_value is 0, it means we are at the start of a new chunk so:
                    # Inflow chunk timer should be set to chunk_size << 4
                    assert int(self.dut.inflow_chunk_timer.value) == self.expected_chunk_size << 4, \
                        f"Expected inflow_chunk_timer: {self.expected_chunk_size << 4}, got: {int(self.dut.inflow_chunk_timer.value)}"
                    # FIFO in queue count should be set to 8
                    assert int(self.dut.fifo_in_queue_count.value) == 8, \
                        f"Expected fifo_in_queue_count: 8, got: {int(self.dut.fifo_in_queue_count.value)}"

            ## Outflow Logic Scoreboard

            # When previous_outflow_timer_value is 0, outflow_timer should be set to chunk_size << 4
            # And outflow_value[i], outflow_value_plus_one[i] and outflow_remainder[i] should be available with correct values
            # This is when chunk sums will have exited the FIFO   
            if previous_outflow_timer_value == 0:
                assert int(self.dut.outflow_timer.value) == self.expected_chunk_size << 4, \
                    f"Expected outflow_timer: {self.expected_chunk_size << 4}, got: {int(self.dut.outflow_timer.value)}"
                
                for i in range(8):
                    # Model the outflow values based on DUT
                    expected_fifo_out_chunk_sum[i] = self.channel_queues[i].popleft()
                    self.expected_number_of_elements_in_fifo -= 1
                    expected_outflow_value[i] = expected_fifo_out_chunk_sum[i] >> self.expected_chunk_width
                    expected_outflow_value_plus_one[i] = (expected_fifo_out_chunk_sum[i] >> self.expected_chunk_width) + 1
                    expected_outflow_remainder[i] = expected_fifo_out_chunk_sum[i] & self.expected_chunk_size

                    # Real average of outflow value
                    expected_float_outflow_value[i] = expected_fifo_out_chunk_sum[i] / (2**self.expected_chunk_width)

                    self.dut._log.info(f"Number of elements in the MODEL FIFO: {self.expected_number_of_elements_in_fifo}")

                    self.dut._log.info(f"FOR CHANNEL {i}:")
                    self.dut._log.info(f"Expected fifo_out_chunk_sum[{i}]: {expected_fifo_out_chunk_sum[i]}")
                    self.dut._log.info(f"Current fifo_out_chunk_sum[{i}]: {int(self.dut.queued_fifo_out_chunk_sum[i].value)}")

                    self.dut._log.info(f"Expected float_outflow_value[{i}]: {expected_float_outflow_value[i]}")

                    self.dut._log.info(f"Expected outflow_value[{i}]: {expected_outflow_value[i]}")
                    self.dut._log.info(f"Current outflow_value[{i}]: {int(self.dut.outflow_value[i].value)}")

                    self.dut._log.info(f"Expected outflow_value_plus_one[{i}]: {expected_outflow_value_plus_one[i]}")
                    self.dut._log.info(f"Current outflow_value_plus_one[{i}]: {int(self.dut.outflow_value_plus_one[i].value)}")

                    self.dut._log.info(f"Expected outflow_remainder[{i}]: {expected_outflow_remainder[i]}")
                    self.dut._log.info(f"Current outflow_remainder[{i}]: {int(self.dut.outflow_remainder[i].value)}")

                    # Check if outflow_value, outflow_value_plus_one and outflow_remainder matches the expected values
                    assert int(self.dut.outflow_value[i].value) == expected_outflow_value[i], \
                        f"Expected outflow_value[{i}]: {expected_outflow_value[i]}, got: {int(self.dut.outflow_value[i].value)}"
                    
                    assert int(self.dut.outflow_value_plus_one[i].value) == expected_outflow_value_plus_one[i], \
                        f"Expected outflow_value_plus_one[{i}]: {expected_outflow_value_plus_one[i]}, got: {int(self.dut.outflow_value_plus_one[i].value)}"

                    assert int(self.dut.outflow_remainder[i].value) == expected_outflow_remainder[i], \
                        f"Expected outflow_remainder[{i}]: {expected_outflow_remainder[i]}, got: {int(self.dut.outflow_remainder[i].value)}"

                    # Check if queued_fifo_out_chunk_sum matches the expected_fifo_out_chunk_sum[i], they should be available when outflow_timer is 0
                    assert int(self.dut.queued_fifo_out_chunk_sum[i].value) == expected_fifo_out_chunk_sum[i], \
                        f"Expected queued_fifo_out_chunk_sum[{i}]: {expected_fifo_out_chunk_sum[i]}, got: {int(self.dut.queued_fifo_out_chunk_sum[i].value)}"
                    
            # Sum Logic Scoreboard
            if previous_outflow_timer_value & 0xF == 0:

                for i in range(8):
                    if (int(previous_outflow_timer_value) >> 4) < previous_expected_outflow_remainder[i]:
                        expected_sum_delta[i] = previous_inflow_value[i] - previous_expected_outflow_value_plus_one[i]
                    else:
                        expected_sum_delta[i] = previous_inflow_value[i] - previous_expected_outflow_value[i]

                    expected_float_sum_delta[i] = previous_inflow_value[i] - previous_expected_float_outflow_value[i]

                    self.dut._log.info(f"FOR CHANNEL {i}:")
                    self.dut._log.info(f"Previous inflow_value[{i}] that should reflect to this cycle: {previous_inflow_value[i]}")
                    self.dut._log.info(f"Expected sum_delta[{i}] this cycle: {expected_sum_delta[i]}")
                    self.dut._log.info(f"Current sum_delta[{i}] this cycle: {int(self.dut.sum_delta[i].value.signed_integer)}")
                    self.dut._log.info(f"Expected float_sum_delta[{i}] this cycle: {expected_float_sum_delta[i]}")

                    # Check if sum_delta matches the expected value
                    assert int(self.dut.sum_delta[i].value.signed_integer) == expected_sum_delta[i], \
                        f"Expected sum_delta[{i}]: {expected_sum_delta[i]}, got: {int(self.dut.sum_delta[i].value.signed_integer)}"
                    
            elif (previous_outflow_timer_value & 0xF) == 15:
                for i in range(8):
                    expected_total_sum[i] = expected_total_sum[i] + expected_sum_delta[i]
                    expected_float_total_sum[i] = expected_float_total_sum[i] + expected_float_sum_delta[i]

                    self.dut._log.info(f"FOR CHANNEL {i}:")
                    self.dut._log.info(f"Expected total_sum[{i}] this cycle: {expected_total_sum[i]}")
                    self.dut._log.info(f"Current total_sum[{i}] this cycle: {int(self.dut.total_sum[i].value.signed_integer)}")
                    self.dut._log.info(f"Expected float_total_sum[{i}] this cycle: {expected_float_total_sum[i]}")

                    # Check if total_sum matches the expected value
                    assert int(self.dut.total_sum[i].value.signed_integer) == expected_total_sum[i], \
                        f"Expected total_sum[{i}]: {expected_total_sum[i]}, got: {int(self.dut.total_sum[i].value.signed_integer)}"

            # When previous_outflow_timer_value is 16, fifo_out_queue_count should be set to 8
            if previous_outflow_timer_value == 16:
                assert int(self.dut.fifo_out_queue_count.value) == 8, \
                    f"Expected fifo_out_queue_count: 8, got: {int(self.dut.fifo_out_queue_count.value)}"     

            await RisingEdge(self.dut.clk)
            previous_inflow_chunk_timer_value = int(self.dut.inflow_chunk_timer.value)
            previous_outflow_timer_value = int(self.dut.outflow_timer.value)

            previous_inflow_value = inflow_value.copy()

            previous_expected_outflow_value = expected_outflow_value.copy()
            previous_expected_outflow_value_plus_one = expected_outflow_value_plus_one.copy()
            previous_expected_outflow_remainder = expected_outflow_remainder.copy()
            previous_expected_float_outflow_value = expected_float_outflow_value.copy()
            await ReadWrite()

        # Wait for the DUT to transition to OUT_OF_BOUNDS or ERROR state if loop breaks
        for i in range(20):
            await RisingEdge(self.dut.clk)

    async def fifo_will_overflow(self, window):
        """
        Check if the FIFO will overflow based on the window value.
        """
        chunk_width = 0
        initial_outflow_timer = window -1 
        for i in range(31, -1, -1):
            if (window >> i) & 1:
                if i >= 11:
                    chunk_width = i - 10
                    break
                else:
                    return False

        chunk_size = (2**chunk_width) - 1
        inflow_chunk_timer = chunk_size << 4

        self.dut._log.info(f"Window: {window}")
        self.dut._log.info(f"Chunk Width: {chunk_width}")
        self.dut._log.info(f"Chunk Size: {chunk_size}")

        ratio = initial_outflow_timer / inflow_chunk_timer

        if ratio <= 128:
            self.dut._log.info(f"For window = {window} initial_outflow_timer / inflow_chunk_timer = {ratio}, Not expecting FIFO overflow")
            return False
        else: 
            self.dut._log.info(f"For window = {window} initial_outflow_timer / inflow_chunk_timer = {ratio}, Expecting FIFO overflow")
            return True

