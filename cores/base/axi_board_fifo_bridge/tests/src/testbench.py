import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly

DAC_ADDR = 0x0000   # SEL_BIT (12) = 0 -> DAC half
ADC_ADDR = 0x1000   # SEL_BIT (12) = 1 -> ADC half
RESP_OKAY = 0


async def reset(dut):
    dut.s_axi_awvalid.value = 0
    dut.s_axi_wvalid.value = 0
    dut.s_axi_bready.value = 0
    dut.s_axi_arvalid.value = 0
    dut.s_axi_rready.value = 0
    dut.s_axi_awaddr.value = 0
    dut.s_axi_araddr.value = 0
    dut.s_axi_wdata.value = 0
    dut.s_axi_wstrb.value = 0xF
    # FIFO models: not full, not empty, known read data
    dut.dac_cmd_full.value = 0
    dut.adc_cmd_full.value = 0
    dut.dac_data_empty.value = 0
    dut.adc_data_empty.value = 0
    dut.dac_data_rd_data.value = 0
    dut.adc_data_rd_data.value = 0
    dut.dac_wr_mode_block.value = 0
    dut.adc_rd_mode_block.value = 0
    for s in ("aresetn", "dac_cmd_resetn", "dac_data_resetn", "adc_cmd_resetn", "adc_data_resetn"):
        getattr(dut, s).value = 0
    for _ in range(3):
        await RisingEdge(dut.aclk)
    for s in ("aresetn", "dac_cmd_resetn", "dac_data_resetn", "adc_cmd_resetn", "adc_data_resetn"):
        getattr(dut, s).value = 1
    await RisingEdge(dut.aclk)


async def axi_write(dut, addr, data):
    dut.s_axi_awaddr.value = addr
    dut.s_axi_wdata.value = data
    dut.s_axi_awvalid.value = 1
    dut.s_axi_wvalid.value = 1
    dut.s_axi_bready.value = 1
    await ReadOnly()
    dac_wr = int(dut.dac_cmd_wr_en.value)
    adc_wr = int(dut.adc_cmd_wr_en.value)
    await RisingEdge(dut.aclk)
    dut.s_axi_awvalid.value = 0
    dut.s_axi_wvalid.value = 0
    await ReadOnly()
    while not int(dut.s_axi_bvalid.value):
        await RisingEdge(dut.aclk)
        await ReadOnly()
    bresp = int(dut.s_axi_bresp.value)
    await RisingEdge(dut.aclk)
    dut.s_axi_bready.value = 0
    return bresp, dac_wr, adc_wr


async def axi_read(dut, addr):
    dut.s_axi_araddr.value = addr
    dut.s_axi_arvalid.value = 1
    dut.s_axi_rready.value = 1
    await ReadOnly()
    dac_rd = int(dut.dac_data_rd_en.value)
    adc_rd = int(dut.adc_data_rd_en.value)
    await RisingEdge(dut.aclk)
    dut.s_axi_arvalid.value = 0
    await ReadOnly()
    while not int(dut.s_axi_rvalid.value):
        await RisingEdge(dut.aclk)
        await ReadOnly()
    rdata = int(dut.s_axi_rdata.value)
    await RisingEdge(dut.aclk)
    dut.s_axi_rready.value = 0
    return rdata, dac_rd, adc_rd


async def setup(dut):
    cocotb.start_soon(Clock(dut.aclk, 10, units="ns").start())
    await reset(dut)


@cocotb.test()
async def test_write_routing(dut):
    await setup(dut)
    bresp, dac_wr, adc_wr = await axi_write(dut, DAC_ADDR, 0xCAFE0001)
    assert bresp == RESP_OKAY and dac_wr == 1 and adc_wr == 0, "DAC write misrouted"
    assert int(dut.dac_cmd_wr_data.value) == 0xCAFE0001
    bresp, dac_wr, adc_wr = await axi_write(dut, ADC_ADDR, 0xCAFE0002)
    assert bresp == RESP_OKAY and adc_wr == 1 and dac_wr == 0, "ADC write misrouted"
    assert int(dut.adc_cmd_wr_data.value) == 0xCAFE0002


@cocotb.test()
async def test_read_routing(dut):
    await setup(dut)
    dut.dac_data_rd_data.value = 0xD00D0000
    dut.adc_data_rd_data.value = 0xA11C0000
    rdata, dac_rd, adc_rd = await axi_read(dut, DAC_ADDR)
    assert dac_rd == 1 and adc_rd == 0, "DAC read misrouted"
    assert rdata == 0xD00D0000, f"DAC rdata {rdata:#x}"
    rdata, dac_rd, adc_rd = await axi_read(dut, ADC_ADDR)
    assert adc_rd == 1 and dac_rd == 0, "ADC read misrouted"
    assert rdata == 0xA11C0000, f"ADC rdata {rdata:#x}"


@cocotb.test()
async def test_mode_block(dut):
    await setup(dut)
    # DAC command write blocked when DMA owns it
    dut.dac_wr_mode_block.value = 1
    _, dac_wr, _ = await axi_write(dut, DAC_ADDR, 0x1)
    assert dac_wr == 0, "blocked DAC write still pushed"
    await ReadOnly()
    assert int(dut.mode_viol.value) == 1, "mode_viol not set on blocked DAC write"
    await RisingEdge(dut.aclk)
    dut.dac_wr_mode_block.value = 0
    # ADC command write is never blocked
    _, _, adc_wr = await axi_write(dut, ADC_ADDR, 0x2)
    assert adc_wr == 1, "ADC write wrongly blocked"
    # ADC data read blocked when DMA owns it
    dut.adc_rd_mode_block.value = 1
    _, _, adc_rd = await axi_read(dut, ADC_ADDR)
    assert adc_rd == 0, "blocked ADC read still popped"
    await ReadOnly()
    assert int(dut.mode_viol.value) == 1, "mode_viol not set on blocked ADC read"


@cocotb.test()
async def test_overflow_underflow(dut):
    await setup(dut)
    dut.dac_cmd_full.value = 1
    await axi_write(dut, DAC_ADDR, 0x1)
    await ReadOnly()
    assert int(dut.dac_cmd_overflow.value) == 1, "dac_cmd_overflow not set"
    await RisingEdge(dut.aclk)
    dut.adc_cmd_full.value = 1
    await axi_write(dut, ADC_ADDR, 0x1)
    await ReadOnly()
    assert int(dut.adc_cmd_overflow.value) == 1, "adc_cmd_overflow not set"
    await RisingEdge(dut.aclk)
    dut.dac_data_empty.value = 1
    await axi_read(dut, DAC_ADDR)
    await ReadOnly()
    assert int(dut.dac_data_underflow.value) == 1, "dac_data_underflow not set"
    await RisingEdge(dut.aclk)
    dut.adc_data_empty.value = 1
    await axi_read(dut, ADC_ADDR)
    await ReadOnly()
    assert int(dut.adc_data_underflow.value) == 1, "adc_data_underflow not set"
    # Flags clear on their own FIFO reset
    await RisingEdge(dut.aclk)
    dut.dac_cmd_resetn.value = 0
    await RisingEdge(dut.aclk)
    dut.dac_cmd_resetn.value = 1
    await ReadOnly()
    assert int(dut.dac_cmd_overflow.value) == 0, "dac_cmd_overflow did not clear on reset"
    assert int(dut.adc_cmd_overflow.value) == 1, "adc_cmd_overflow wrongly cleared"
