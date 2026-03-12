import cocotb
from cocotb.triggers import RisingEdge, ReadOnly
from cocotb_coverage.coverage import *
import atexit
import os

# FIFO_DEPTH = 2**ADDR_WIDTH, and should be changed according to the DUT parameters for correct coverage.
FIFO_DEPTH = 16

# Coverage points for full, empty, almost full, and almost empty outputs
@CoverPoint("fifo_async.empty",
            xf=lambda dut: int(dut.empty.value),
            bins=[0, 1],
            at_least=1)

@CoverPoint("fifo_async.almost_empty",
            xf=lambda dut: int(dut.almost_empty.value),
            bins=[0, 1],
            at_least=1)

@CoverPoint("fifo_async.fifo_count_rd_clk",
            xf=lambda dut: int(dut.fifo_count_rd_clk.value),
            bins=range(0, FIFO_DEPTH),
            at_least=1)
def sample_rd_domain_fifo_status_outputs(dut):
    pass

@CoverPoint("fifo_async.fifo_count_wr_clk",
            xf=lambda dut: int(dut.fifo_count_wr_clk.value),
            bins=range(0, FIFO_DEPTH),
            at_least=1)
@CoverPoint("fifo_async.full",
            xf=lambda dut: int(dut.full.value),
            bins=[0, 1],
            at_least=1)
@CoverPoint("fifo_async.almost_full",
            xf=lambda dut: int(dut.almost_full.value),
            bins=[0, 1],
            at_least=1)
def sample_wr_domain_fifo_status_outputs(dut):
    pass

# Coverage points for rd_en and wr_en signals
@CoverPoint("fifo_async.rd_en",
            xf=lambda dut: int(dut.rd_en.value),
            bins=[0, 1],
            at_least=1)
def sample_rd_enable(dut):
    pass

@CoverPoint("fifo_async.wr_en",
            xf=lambda dut: int(dut.wr_en.value),
            bins=[0, 1],
            at_least=1)
def sample_wr_enable(dut):
    pass

# Coverage points for read and write pointers
@CoverPoint("fifo_async.rd_ptr_bin",
            xf=lambda dut: int(dut.rd_ptr_bin.value),
            bins=range(0, FIFO_DEPTH),
            at_least=1)
@CoverPoint("fifo_async.rd_ptr_bin_next",
            xf=lambda dut: int(dut.rd_ptr_bin_next.value),
            bins=range(0, FIFO_DEPTH),
            at_least=1)
@CoverPoint("fifo_async.wr_ptr_bin_rd_clk",
            xf=lambda dut: int(dut.wr_ptr_bin_rd_clk.value),
            bins=range(0, FIFO_DEPTH),
            at_least=1)
def sample_read_pointers(dut):
    pass

@CoverPoint("fifo_async.wr_ptr_bin",
            xf=lambda dut: int(dut.wr_ptr_bin.value),
            bins=range(0, FIFO_DEPTH),
            at_least=1)
@CoverPoint("fifo_async.rd_ptr_bin_wr_clk",
            xf=lambda dut: int(dut.rd_ptr_bin_wr_clk.value),
            bins=range(0, FIFO_DEPTH),
            at_least=1)
def sample_wr_pointers(dut):
    pass

async def _coverage_monitor_rd_domain(dut):
    await RisingEdge(dut.rd_clk)
    while True:
        await RisingEdge(dut.rd_clk)
        await ReadOnly()
        sample_rd_domain_fifo_status_outputs(dut)
        sample_rd_enable(dut)
        sample_read_pointers(dut)

async def _coverage_monitor_wr_domain(dut):
    await RisingEdge(dut.wr_clk)
    while True:
        await RisingEdge(dut.wr_clk)
        await ReadOnly()
        sample_wr_domain_fifo_status_outputs(dut)
        sample_wr_enable(dut)
        sample_wr_pointers(dut)


def start_coverage_monitor(dut):
    cocotb.start_soon(_coverage_monitor_rd_domain(dut))
    cocotb.start_soon(_coverage_monitor_wr_domain(dut))

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
