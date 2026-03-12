import cocotb
from cocotb.triggers import RisingEdge, ReadOnly
from cocotb_coverage.coverage import *
import atexit
import os

# Coverage points for shutdown_sense, shutdown_sense_sel, shutdown_sense_en, and shutdown_sense_pin signals
@CoverPoint("shutdown_sense.shutdown_sense_en",
            xf=lambda dut: int(dut.shutdown_sense_en.value),
            bins=[0, 1],
            at_least=1)

@CoverPoint("shutdown_sense.shutdown_sense_pin",
            xf=lambda dut: int(dut.shutdown_sense_pin.value),
            bins=[0, 1],
            at_least=1)

@CoverPoint("shutdown_sense.shutdown_sense_sel",
            xf=lambda dut: int(dut.shutdown_sense_sel.value),
            bins=range(0, 8),
            at_least=1)

@CoverPoint("shutdown_sense.shutdown_sense",
            xf=lambda dut: int(dut.shutdown_sense.value),
            bins=[0, 1, 2, 4, 8, 16, 32, 64, 128],
            at_least=1)
def sample_coverage(dut):
    pass


async def _coverage_monitor(dut):
    await RisingEdge(dut.clk)
    while True:
        await RisingEdge(dut.clk)
        await ReadOnly()
        sample_coverage(dut)



def start_coverage_monitor(dut):
    cocotb.start_soon(_coverage_monitor(dut))

def write_report():
    results_dir = os.getenv("RESULTS_DIR", ".") # Default to current dir if not set

    original_cwd = os.getcwd() # Store the original working directory

    try:
        # Change the current working directory to the results directory
        os.makedirs(results_dir, exist_ok=True) # Ensure the directory exists
        os.chdir(results_dir)
        cocotb.log.info(f"Changing CWD to: {os.getcwd()}") # Log the change

        # Now, export the coverage reports.
        # These functions will write to the new current working directory.
        coverage_db.export_to_xml("fifo_async_coverage.xml")
        coverage_db.export_to_yaml("fifo_async_coverage.yaml")

    finally:
        # Change back to the original working directory to avoid affecting other parts
        # of the cocotb environment or subsequent processes, although for atexit
        # this might be less critical as the process is exiting.
        os.chdir(original_cwd)
        cocotb.log.info(f"Restored CWD to: {os.getcwd()}")

atexit.register(write_report)
