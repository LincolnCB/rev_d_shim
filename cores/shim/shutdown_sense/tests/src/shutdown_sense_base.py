import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, ReadWrite
import random

class shutdown_sense_base:
    def __init__(self, dut, connected=0b11111111, clk_period=4, time_unit='ns'):
        self.dut = dut
        self.clk_period = clk_period
        self.time_unit = time_unit

        # Start the clock
        cocotb.start_soon(Clock(self.dut.clk, clk_period, units=time_unit).start())

        # Initialize input signals
        self.dut.shutdown_sense_pin.value = 0
        self.dut.shutdown_sense_connected.value = connected

    async def shutdown_sense_en_drive_low(self):
        """Drive the shutdown sense en low to initialize `shutdown_sense_sel` to `000` and clear the `shutdown_sense` register."""
        await RisingEdge(self.dut.clk)
        self.dut.shutdown_sense_en.value = 0
        await RisingEdge(self.dut.clk)
        await RisingEdge(self.dut.clk)
        self.dut._log.info("DUT now at a known STATE.")

    async def monitor_and_scoreboard(self):
        """Monitor the shim shutdown sense signals and score them against expected values."""
        shutdown_sense_expected_next = [0, 0, 0, 0, 0, 0, 0, 0]
        shutdown_sense_sel_expected_next = 0
        while True:
            await RisingEdge(self.dut.clk)
            await ReadOnly()

            # Monitor the current state of the DUT compared to expected values
            assert self.dut.shutdown_sense_sel.value == shutdown_sense_sel_expected_next, \
                f"Expected shutdown_sense_sel: {int(shutdown_sense_sel_expected_next)}, got: {int(self.dut.shutdown_sense_sel.value)}"
            
            for i in range(8):
                assert self.dut.shutdown_sense[i].value == shutdown_sense_expected_next[i], \
                    f"Expected shutdown_sense[{i}]: {int(shutdown_sense_expected_next[i])}, got: {int(self.dut.shutdown_sense[i].value)}"
            
            self.dut._log.info(f"MONITOR THIS CYCLE:")
            self.dut._log.info(f"DUT shutdown_sense_en: {int(self.dut.shutdown_sense_en.value)}")
            self.dut._log.info(f"DUT shutdown_sense_pin: {int(self.dut.shutdown_sense_pin.value)}")
            self.dut._log.info(f"DUT shutdown_sense_sel: {int(self.dut.shutdown_sense_sel.value)}, "
                               f"EXPECTED shutdown_sense_sel: {shutdown_sense_sel_expected_next}")
            self.dut._log.info(f"DUT shutdown_sense: {[int(sense.value) for sense in self.dut.shutdown_sense]}, "
                               f"EXPECTED shutdown_sense: {list(reversed(shutdown_sense_expected_next))}")
            
            # Update the next cycle's expected values based on the current state of the DUT
            if self.dut.shutdown_sense_en.value == 0:
                shutdown_sense_expected_next = [0, 0, 0, 0, 0, 0, 0, 0]
                shutdown_sense_sel_expected_next = 0
            else:
                if self.dut.shutdown_sense_pin.value == 1:
                    shutdown_sense_expected_next[shutdown_sense_sel_expected_next] = 1

                shutdown_sense_sel_expected_next = (shutdown_sense_sel_expected_next + 1) % 8




