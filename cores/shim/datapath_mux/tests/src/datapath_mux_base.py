import cocotb
from cocotb.triggers import Timer


# Drives the (purely combinational) datapath_mux and models its expected outputs. There is
# no clock: inputs are applied, allowed to settle, and the outputs are checked against the
# same 2:1 select the RTL implements.
class datapath_mux_base:

    def __init__(self, dut, settle_ns=1):
        self.dut = dut
        self.settle_ns = settle_ns
        self.DATA_WIDTH  = int(dut.DATA_WIDTH.value)
        self.COUNT_WIDTH = int(dut.COUNT_WIDTH.value)
        self.DATA_MASK   = (1 << self.DATA_WIDTH) - 1
        self.COUNT_MASK  = (1 << self.COUNT_WIDTH) - 1

        self.dut._log.info(f"DUT params: DATA_WIDTH={self.DATA_WIDTH}, COUNT_WIDTH={self.COUNT_WIDTH}")

    # Apply one input vector and let the combinational logic settle.
    async def apply(self, v):
        d = self.dut
        d.datapath_mode.value    = v["mode"] & 1
        d.cmd_full.value         = v["cmd_full"] & 1
        d.cmd_pio_wr_data.value  = v["cmd_pio_wr_data"] & self.DATA_MASK
        d.cmd_pio_wr_en.value    = v["cmd_pio_wr_en"] & 1
        d.cmd_dma_wr_data.value  = v["cmd_dma_wr_data"] & self.DATA_MASK
        d.cmd_dma_wr_en.value    = v["cmd_dma_wr_en"] & 1
        d.data_rd_data.value     = v["data_rd_data"] & self.DATA_MASK
        d.data_empty.value       = v["data_empty"] & 1
        d.data_count.value       = v["data_count"] & self.COUNT_MASK
        d.data_pio_rd_en.value   = v["data_pio_rd_en"] & 1
        d.data_dma_rd_en.value   = v["data_dma_rd_en"] & 1
        await Timer(self.settle_ns, "ns")

    # The reference model: exactly the select the RTL should implement.
    def expected(self, v):
        mode = v["mode"] & 1
        dm = self.DATA_MASK
        cm = self.COUNT_MASK
        return {
            "cmd_wr_data":  (v["cmd_dma_wr_data"] if mode else v["cmd_pio_wr_data"]) & dm,
            "cmd_wr_en":    (v["cmd_dma_wr_en"] if mode else v["cmd_pio_wr_en"]) & 1,
            "cmd_pio_full": 1 if mode else (v["cmd_full"] & 1),
            "cmd_dma_full":  (v["cmd_full"] & 1) if mode else 1,
            "data_rd_en":     (v["data_dma_rd_en"] if mode else v["data_pio_rd_en"]) & 1,
            "data_pio_rd_data": v["data_rd_data"] & dm,
            "data_pio_empty":   1 if mode else (v["data_empty"] & 1),
            "data_dma_rd_data":  v["data_rd_data"] & dm,
            "data_dma_empty":    (v["data_empty"] & 1) if mode else 1,
            "data_dma_count":    (v["data_count"] & cm) if mode else 0,
        }

    def check(self, v):
        exp = self.expected(v)
        d = self.dut
        actual = {
            "cmd_wr_data":       int(d.cmd_wr_data.value),
            "cmd_wr_en":         int(d.cmd_wr_en.value),
            "cmd_pio_full":      int(d.cmd_pio_full.value),
            "cmd_dma_full":      int(d.cmd_dma_full.value),
            "data_rd_en":        int(d.data_rd_en.value),
            "data_pio_rd_data":  int(d.data_pio_rd_data.value),
            "data_pio_empty":    int(d.data_pio_empty.value),
            "data_dma_rd_data":  int(d.data_dma_rd_data.value),
            "data_dma_empty":    int(d.data_dma_empty.value),
            "data_dma_count":    int(d.data_dma_count.value),
        }
        for k in exp:
            assert actual[k] == exp[k], \
                f"mode={v['mode']} output {k}: got {actual[k]:#x}, expected {exp[k]:#x} (inputs {v})"
