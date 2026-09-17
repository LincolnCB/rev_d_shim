import cocotb
import random

from datapath_mux_base import datapath_mux_base


def _distinct_vector(rnd):
    # PIO and DMA sides carry different data/enables so a mis-select is visible.
    return {
        "cmd_full":         rnd.randint(0, 1),
        "cmd_pio_wr_data":  rnd.randint(0, 0xFFFFFFFF),
        "cmd_pio_wr_en":    rnd.randint(0, 1),
        "cmd_dma_wr_data":  rnd.randint(0, 0xFFFFFFFF),
        "cmd_dma_wr_en":    rnd.randint(0, 1),
        "data_rd_data":     rnd.randint(0, 0xFFFFFFFF),
        "data_empty":       rnd.randint(0, 1),
        "data_count":       rnd.randint(0, 0x3FFF),
        "data_pio_rd_en":   rnd.randint(0, 1),
        "data_dma_rd_en":   rnd.randint(0, 1),
    }


@cocotb.test()
async def test_pio_mode_selects_pio_and_gates_dma(dut):
    tb = datapath_mux_base(dut)
    tb.dut._log.info("STARTING TEST: test_pio_mode_selects_pio_and_gates_dma")

    # A vector where the two sides differ, in PIO mode: the FIFO must see the PIO owner
    # and the DMA owner must be fully gated (full=1, empty=1, count=0).
    v = {
        "mode": 0,
        "cmd_full": 0,
        "cmd_pio_wr_data": 0xAAAA5555, "cmd_pio_wr_en": 1,
        "cmd_dma_wr_data": 0x12345678,  "cmd_dma_wr_en": 1,
        "data_rd_data": 0xCAFEF00D, "data_empty": 0, "data_count": 0x2AA,
        "data_pio_rd_en": 1, "data_dma_rd_en": 1,
    }
    await tb.apply(v)
    tb.check(v)
    assert int(dut.cmd_wr_data.value) == 0xAAAA5555
    assert int(dut.cmd_dma_full.value) == 1 and int(dut.data_dma_empty.value) == 1
    assert int(dut.data_dma_count.value) == 0


@cocotb.test()
async def test_dma_mode_selects_dma_and_gates_pio(dut):
    tb = datapath_mux_base(dut)
    tb.dut._log.info("STARTING TEST: test_dma_mode_selects_dma_and_gates_pio")

    # Same vector in DMA mode: the FIFO must see the DMA owner and the PIO owner must be
    # gated (full=1, empty=1). The DMA side sees the real full/empty/count.
    v = {
        "mode": 1,
        "cmd_full": 1,
        "cmd_pio_wr_data": 0xAAAA5555, "cmd_pio_wr_en": 1,
        "cmd_dma_wr_data": 0x12345678,  "cmd_dma_wr_en": 1,
        "data_rd_data": 0xCAFEF00D, "data_empty": 0, "data_count": 0x2AA,
        "data_pio_rd_en": 1, "data_dma_rd_en": 1,
    }
    await tb.apply(v)
    tb.check(v)
    assert int(dut.cmd_wr_data.value) == 0x12345678
    assert int(dut.cmd_pio_full.value) == 1 and int(dut.data_pio_empty.value) == 1
    assert int(dut.data_dma_count.value) == 0x2AA


@cocotb.test()
async def test_wr_en_gated_to_selected_owner(dut):
    tb = datapath_mux_base(dut)
    tb.dut._log.info("STARTING TEST: test_wr_en_gated_to_selected_owner")

    # Only the selected owner's wr_en/rd_en may reach the FIFO, regardless of what the
    # idle owner is asserting. Drive the selected owner with a known value and the idle
    # owner with the opposite, so a leak from the idle side would flip the output.
    for mode in (0, 1):
        for sel_en in (0, 1):
            idle_en = 1 - sel_en
            v = {
                "mode": mode, "cmd_full": 0,
                "cmd_pio_wr_data": 0x1, "cmd_pio_wr_en": (sel_en if mode == 0 else idle_en),
                "cmd_dma_wr_data": 0x2,  "cmd_dma_wr_en": (sel_en if mode == 1 else idle_en),
                "data_rd_data": 0x3, "data_empty": 0, "data_count": 4,
                "data_pio_rd_en": (sel_en if mode == 0 else idle_en),
                "data_dma_rd_en": (sel_en if mode == 1 else idle_en),
            }
            await tb.apply(v)
            tb.check(v)
            assert int(dut.cmd_wr_en.value) == sel_en   # selected owner passes, idle owner blocked
            assert int(dut.data_rd_en.value) == sel_en


@cocotb.test()
async def test_random(dut):
    tb = datapath_mux_base(dut)
    tb.dut._log.info("STARTING TEST: test_random")
    rnd = random.Random(1234)

    for _ in range(200):
        for mode in (0, 1):
            v = _distinct_vector(rnd)
            v["mode"] = mode
            await tb.apply(v)
            tb.check(v)
