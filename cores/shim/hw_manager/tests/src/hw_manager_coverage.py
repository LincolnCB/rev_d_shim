import cocotb
from cocotb.triggers import RisingEdge, ReadOnly
from cocotb_coverage.coverage import *
import atexit
import os

STATE_BIN_LABELS = {
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

STATUS_BIN_LABELS = {
    0x000: "STS_EMPTY",
    0x001: "STS_OK",
    0x002: "STS_PS_SHUTDOWN",
    0x100: "STS_SPI_START_TIMEOUT",
    0x101: "STS_SPI_INIT_TIMEOUT",
    0x200: "STS_INTEG_THRESH_AVG_OOB",
    0x201: "STS_INTEG_WINDOW_OOB",
    0x202: "STS_INTEG_EN_OOB",
    0x203: "STS_SYS_EN_OOB",
    0x204: "STS_LOCK_VIOL",
    0x300: "STS_SHUTDOWN_SENSE",
    0x301: "STS_EXT_SHUTDOWN",
    0x400: "STS_OVER_THRESH",
    0x401: "STS_THRESH_UNDERFLOW",
    0x402: "STS_THRESH_OVERFLOW",
    0x500: "STS_BAD_TRIG_CMD",
    0x501: "STS_TRIG_CMD_BUF_OVERFLOW",
    0x502: "STS_TRIG_DATA_BUF_UNDERFLOW",
    0x503: "STS_TRIG_DATA_BUF_OVERFLOW",
    0x600: "STS_DAC_BOOT_FAIL",
    0x601: "STS_BAD_DAC_CMD",
    0x602: "STS_DAC_CAL_OOB",
    0x603: "STS_DAC_VAL_OOB",
    0x604: "STS_DAC_BUF_UNDERFLOW",
    0x605: "STS_DAC_BUF_OVERFLOW",
    0x606: "STS_UNEXP_DAC_TRIG",
    0x700: "STS_ADC_BOOT_FAIL",
    0x701: "STS_BAD_ADC_CMD",
    0x702: "STS_ADC_CMD_BUF_UNDERFLOW",
    0x703: "STS_ADC_CMD_BUF_OVERFLOW",
    0x704: "STS_ADC_DATA_BUF_UNDERFLOW",
    0x705: "STS_ADC_DATA_BUF_OVERFLOW",
    0x706: "STS_UNEXP_ADC_TRIG",
}

# Coverage points for states and status codes
@CoverPoint("hw_manager.state",
            xf = lambda dut: int(dut.state.value),
            bins = list(STATE_BIN_LABELS.keys()),
            bins_labels = list(STATE_BIN_LABELS.values()),
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.status_code",
            xf = lambda dut: dut.status_code.value,
            bins = list(STATUS_BIN_LABELS.keys()),
            bins_labels = list(STATUS_BIN_LABELS.values()),
            rel = lambda x, y: x == y,
            at_least = 1)
def sample_state_and_status_coverage(dut):
    pass

# Coverage points for system control signals
@CoverPoint("hw_manager.sys_en",
            xf = lambda dut: int(dut.sys_en.value),
            bins = [True, False],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.spi_off",
            xf = lambda dut: int(dut.spi_off.value),
            bins = [True, False],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.ext_shutdown",
            xf = lambda dut: int(dut.ext_shutdown.value),
            bins = [True, False],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.unlock_cfg",
            xf = lambda dut: int(dut.unlock_cfg.value),
            bins = [True, False],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.spi_clk_gate",
            xf = lambda dut: int(dut.spi_clk_gate.value),
            bins = [True, False],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.spi_en",
            xf = lambda dut: int(dut.spi_en.value),
            bins = [True, False],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.shutdown_sense_en",
            xf = lambda dut: int(dut.shutdown_sense_en.value),
            bins = [True, False],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.block_bufs",
            xf = lambda dut: int(dut.block_bufs.value),
            bins = [True, False],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.n_shutdown_force",
            xf = lambda dut: int(dut.n_shutdown_force.value),
            bins = [True, False],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.shutdown_rst",
            xf = lambda dut: int(dut.shutdown_rst.value),
            bins = [True, False],
            rel = lambda x, y: x == y,
            at_least = 1)
def sample_system_control_coverage(dut):
    pass

# Coverage points for per-board signals
@CoverPoint("hw_manager.shutdown_sense",
            xf = lambda dut: int(dut.shutdown_sense.value),
            bins = [0, 1, 2, 4, 8, 16, 32, 64, 128],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.over_thresh",
            xf = lambda dut: int(dut.over_thresh.value),
            bins = [0, 1, 2, 4, 8, 16, 32, 64, 128],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.thresh_underflow",
            xf = lambda dut: int(dut.thresh_underflow.value),
            bins = [0, 1, 2, 4, 8, 16, 32, 64, 128],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.thresh_overflow",
            xf = lambda dut: int(dut.thresh_overflow.value),
            bins = [0, 1, 2, 4, 8, 16, 32, 64, 128],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.dac_boot_fail",
            xf = lambda dut: int(dut.dac_boot_fail.value),
            bins = [0, 1, 2, 4, 8, 16, 32, 64, 128],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.bad_dac_cmd",
            xf = lambda dut: int(dut.bad_dac_cmd.value),
            bins = [0, 1, 2, 4, 8, 16, 32, 64, 128],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.dac_cal_oob",
            xf = lambda dut: int(dut.dac_cal_oob.value),
            bins = [0, 1, 2, 4, 8, 16, 32, 64, 128],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.dac_val_oob",
            xf = lambda dut: int(dut.dac_val_oob.value),
            bins = [0, 1, 2, 4, 8, 16, 32, 64, 128],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.dac_cmd_buf_underflow",
            xf = lambda dut: int(dut.dac_cmd_buf_underflow.value),
            bins = [0, 1, 2, 4, 8, 16, 32, 64, 128],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.dac_cmd_buf_overflow",
            xf = lambda dut: int(dut.dac_cmd_buf_overflow.value),
            bins = [0, 1, 2, 4, 8, 16, 32, 64, 128],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.unexp_dac_trig",
            xf = lambda dut: int(dut.unexp_dac_trig.value),
            bins = [0, 1, 2, 4, 8, 16, 32, 64, 128],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.adc_boot_fail",
            xf = lambda dut: int(dut.adc_boot_fail.value),
            bins = [0, 1, 2, 4, 8, 16, 32, 64, 128],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.bad_adc_cmd",
            xf = lambda dut: int(dut.bad_adc_cmd.value),
            bins = [0, 1, 2, 4, 8, 16, 32, 64, 128],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.adc_cmd_buf_underflow",
            xf = lambda dut: int(dut.adc_cmd_buf_underflow.value),
            bins = [0, 1, 2, 4, 8, 16, 32, 64, 128],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.adc_cmd_buf_overflow",
            xf = lambda dut: int(dut.adc_cmd_buf_overflow.value),
            bins = [0, 1, 2, 4, 8, 16, 32, 64, 128],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.adc_data_buf_underflow",
            xf = lambda dut: int(dut.adc_data_buf_underflow.value),
            bins = [0, 1, 2, 4, 8, 16, 32, 64, 128],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.adc_data_buf_overflow",
            xf = lambda dut: int(dut.adc_data_buf_overflow.value),
            bins = [0, 1, 2, 4, 8, 16, 32, 64, 128],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.unexp_adc_trig",
            xf = lambda dut: int(dut.unexp_adc_trig.value),
            bins = [0, 1, 2, 4, 8, 16, 32, 64, 128],
            rel = lambda x, y: x == y,
            at_least = 1)
def sample_per_board_signals_coverage(dut):
    pass

# Coverage points for configuration and status signals
@CoverPoint("hw_manager.integ_thresh_avg_oob",
            xf = lambda dut: int(dut.integ_thresh_avg_oob.value),
            bins = [0, 1],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.integ_window_oob",
            xf = lambda dut: int(dut.integ_window_oob.value),
            bins = [0, 1],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.integ_en_oob",
            xf = lambda dut: int(dut.integ_en_oob.value),
            bins = [0, 1],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.sys_en_oob",
            xf = lambda dut: int(dut.sys_en_oob.value),
            bins = [0, 1],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.lock_viol",
            xf = lambda dut: int(dut.lock_viol.value),
            bins = [0, 1],
            rel = lambda x, y: x == y,
            at_least = 1)
def sample_configuration_status_signals_coverage(dut):
    pass

# Coverage points for trigger buffer and command signals
@CoverPoint("hw_manager.bad_trig_cmd",
            xf = lambda dut: int(dut.bad_trig_cmd.value),
            bins = [True, False],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.trig_cmd_buf_overflow",
            xf = lambda dut: int(dut.trig_cmd_buf_overflow.value),
            bins = [True, False],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.trig_data_buf_underflow",
            xf = lambda dut: int(dut.trig_data_buf_underflow.value),
            bins = [True, False],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverPoint("hw_manager.trig_data_buf_overflow",
            xf = lambda dut: int(dut.trig_data_buf_overflow.value),
            bins = [True, False],
            rel = lambda x, y: x == y,
            at_least = 1)
def sample_trigger_buf_and_cmd_coverage(dut):
    pass

# PS interrupt cross coverage
@CoverPoint("hw_manager.ps_interrupt",
            xf = lambda dut: int(dut.ps_interrupt.value),
            bins = [1, 0],
            bins_labels = ["PS Interrupt Active", "PS Interrupt Inactive"],
            rel = lambda x, y: x == y,
            at_least = 1)
@CoverCross("hw_manager.ps_interrupt_cross",
            items = ["hw_manager.state", "hw_manager.ps_interrupt"],
            ign_bins= [ ("RELEASE_SD_F", None),
                        ("PULSE_SD_RST", None),
                        ("SD_RST_DELAY", None), 
                        ("CONFIRM_SPI_INIT", None), 
                        ("CONFIRM_SPI_START", None),
                        ("IDLE", None)])
def sample_ps_interrupt_cross_coverage(dut):
    pass

async def _coverage_monitor(dut):
    await RisingEdge(dut.clk)
    while True:
        await RisingEdge(dut.clk)
        await ReadOnly()
        sample_state_and_status_coverage(dut)
        sample_system_control_coverage(dut)
        sample_per_board_signals_coverage(dut)
        sample_configuration_status_signals_coverage(dut)
        sample_trigger_buf_and_cmd_coverage(dut)
        sample_ps_interrupt_cross_coverage(dut)

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
        coverage_db.export_to_xml("hw_manager_coverage.xml")
        coverage_db.export_to_yaml("hw_manager_coverage.yaml")

    finally:
        # Change back to the original working directory to avoid affecting other parts
        # of the cocotb environment or subsequent processes, although for atexit
        # this might be less critical as the process is exiting.
        os.chdir(original_cwd)
        cocotb.log.info(f"Restored CWD to: {os.getcwd()}")

atexit.register(write_report)
