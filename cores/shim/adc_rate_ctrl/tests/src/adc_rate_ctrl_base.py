import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly


# Drives the adc_rate_ctrl AXI4-Lite port and the per-board packing signals, modelling the
# backlog fill count: word_emitted beats raise a board's backlog, READ_DELTA writes lower it,
# and accumulate asserts while backlog >= PACKET_WORDS (a fixed parameter, the packet size). A
# single aclk domain throughout.
class adc_rate_ctrl_base:

    def __init__(self, dut, clk_period=10, time_unit="ns"):
        self.dut = dut
        self.clk_period = clk_period
        self.time_unit = time_unit

        self.NUM_BOARDS     = int(dut.NUM_BOARDS.value)
        self.PACKET_WORDS   = int(dut.PACKET_WORDS.value)
        self.AXI_ADDR_WIDTH = int(dut.AXI_ADDR_WIDTH.value)
        self.ALL_BOARDS_N   = (1 << self.NUM_BOARDS) - 1

        # Register map (32-bit word indices), mirroring adc_rate_ctrl.v.
        self.READ_DELTA_BASE = 0
        self.BACKLOG_BASE    = self.NUM_BOARDS

        self.dut._log.info(f"DUT params: NUM_BOARDS={self.NUM_BOARDS}, PACKET_WORDS={self.PACKET_WORDS}, AXI_ADDR_WIDTH={self.AXI_ADDR_WIDTH}")

        self.clk_task = None

        # Idle drive on all inputs.
        self.dut.aresetn.value       = 1
        self.dut.word_emitted.value  = 0
        self.dut.backlog_rst_n.value = self.ALL_BOARDS_N   # no board in reset
        self.dut.s_axi_awaddr.value  = 0
        self.dut.s_axi_awvalid.value = 0
        self.dut.s_axi_wdata.value   = 0
        self.dut.s_axi_wstrb.value   = 0xF
        self.dut.s_axi_wvalid.value  = 0
        self.dut.s_axi_bready.value  = 0
        self.dut.s_axi_araddr.value  = 0
        self.dut.s_axi_arvalid.value = 0
        self.dut.s_axi_rready.value  = 0

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

    # --- AXI4-Lite subordinate BFM -------------------------------------------

    async def axi_write(self, word_idx, value):
        dut = self.dut
        await RisingEdge(dut.aclk)
        dut.s_axi_awaddr.value  = word_idx << 2
        dut.s_axi_awvalid.value = 1
        dut.s_axi_wdata.value   = value & 0xFFFFFFFF
        dut.s_axi_wstrb.value   = 0xF
        dut.s_axi_wvalid.value  = 1
        dut.s_axi_bready.value  = 1
        while True:
            await ReadOnly()
            if int(dut.s_axi_awready.value) and int(dut.s_axi_wready.value):
                break
            await RisingEdge(dut.aclk)
        await RisingEdge(dut.aclk)
        dut.s_axi_awvalid.value = 0
        dut.s_axi_wvalid.value  = 0
        while True:
            await ReadOnly()
            if int(dut.s_axi_bvalid.value):
                break
            await RisingEdge(dut.aclk)
        await RisingEdge(dut.aclk)
        dut.s_axi_bready.value = 0

    async def axi_read(self, word_idx):
        dut = self.dut
        await RisingEdge(dut.aclk)
        dut.s_axi_araddr.value  = word_idx << 2
        dut.s_axi_arvalid.value = 1
        dut.s_axi_rready.value  = 1
        while True:
            await ReadOnly()
            if int(dut.s_axi_arready.value):
                break
            await RisingEdge(dut.aclk)
        await RisingEdge(dut.aclk)
        dut.s_axi_arvalid.value = 0
        while True:
            await ReadOnly()
            if int(dut.s_axi_rvalid.value):
                data = int(dut.s_axi_rdata.value)
                break
            await RisingEdge(dut.aclk)
        await RisingEdge(dut.aclk)
        dut.s_axi_rready.value = 0
        return data

    # --- Register helpers ----------------------------------------------------

    async def report_read(self, board, words):
        await self.axi_write(self.READ_DELTA_BASE + board, words)

    async def read_backlog(self, board):
        return await self.axi_read(self.BACKLOG_BASE + board)

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
