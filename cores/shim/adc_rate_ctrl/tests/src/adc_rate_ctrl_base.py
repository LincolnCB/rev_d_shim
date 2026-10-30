import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly


# Drives the adc_rate_ctrl read-delta inputs and per-board packing signals, modelling the
# backlog fill count: word_emitted beats raise a board's backlog, read_delta_en strobes lower it
# by read_delta, and accumulate asserts while backlog >= PACKET_WORDS (a fixed parameter, the
# packet size). Backlogs are read straight off backlog_out. A single aclk domain throughout.
class adc_rate_ctrl_base:

    def __init__(self, dut, clk_period=10, time_unit="ns"):
        self.dut = dut
        self.clk_period = clk_period
        self.time_unit = time_unit

        self.NUM_BOARDS     = int(dut.NUM_BOARDS.value)
        self.PACKET_WORDS   = int(dut.PACKET_WORDS.value)
        self.ALL_BOARDS_N   = (1 << self.NUM_BOARDS) - 1

        self.dut._log.info(f"DUT params: NUM_BOARDS={self.NUM_BOARDS}, PACKET_WORDS={self.PACKET_WORDS}")

        self.clk_task = None

        # Idle drive on all inputs.
        self.dut.aresetn.value       = 1
        self.dut.word_emitted.value  = 0
        self.dut.backlog_rst_n.value = self.ALL_BOARDS_N   # no board in reset
        self.dut.read_delta_en.value = 0
        self.dut.read_delta.value    = 0

    # --- Clock and reset -----------------------------------------------------

    async def start_clock(self):
        self.clk_task = cocotb.start_soon(
            Clock(self.dut.aclk, self.clk_period, units=self.time_unit).start(start_high=False))
        self.dut._log.info("Clock started.")

    def kill_clock(self):
        if self.clk_task and not self.clk_task.done():
            self.clk_task.cancel()
        self.clk_task = None

    async def reset(self):
        await RisingEdge(self.dut.aclk)
        self.dut.aresetn.value = 0
        await RisingEdge(self.dut.aclk)
        await RisingEdge(self.dut.aclk)
        self.dut.aresetn.value = 1
        await RisingEdge(self.dut.aclk)
        self.dut._log.info("Reset complete.")

    # --- Read-delta and backlog helpers --------------------------------------

    # One-cycle read_delta_en strobe for `board`, carrying `words` on read_delta.
    async def report_read(self, board, words):
        await RisingEdge(self.dut.aclk)
        self.dut.read_delta_en.value = (1 << board)
        self.dut.read_delta.value    = words & 0xFFFFFFFF
        await RisingEdge(self.dut.aclk)
        self.dut.read_delta_en.value = 0
        self.dut.read_delta.value    = 0
        await RisingEdge(self.dut.aclk)

    async def read_backlog(self, board):
        await ReadOnly()
        val = (int(self.dut.backlog_out.value) >> (32 * board)) & 0xFFFFFFFF
        await RisingEdge(self.dut.aclk)
        return val

    # --- Per-board packing signals -------------------------------------------

    # Pulse word_emitted[board] for `cycles` clocks: one backlog increment per clock.
    async def emit(self, board, cycles=1):
        for _ in range(cycles):
            self.dut.word_emitted.value = (1 << board)
            await RisingEdge(self.dut.aclk)
        self.dut.word_emitted.value = 0
        await RisingEdge(self.dut.aclk)

    # Clear one board's backlog via its active-low per-board reset.
    async def backlog_reset(self, board):
        self.dut.backlog_rst_n.value = self.ALL_BOARDS_N & ~(1 << board)
        await RisingEdge(self.dut.aclk)
        await RisingEdge(self.dut.aclk)
        self.dut.backlog_rst_n.value = self.ALL_BOARDS_N
        await RisingEdge(self.dut.aclk)

    async def read_accumulate(self, board):
        await ReadOnly()
        val = int(self.dut.accumulate.value)
        await RisingEdge(self.dut.aclk)
        return (val >> board) & 1
