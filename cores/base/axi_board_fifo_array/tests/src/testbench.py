import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly

PAGE = 0x1000
LANE_DAC_CMD = 0x00
LANE_DAC_DEBUG = 0x04
LANE_DAC_DATA = 0x08
LANE_ADC_CMD = 0x10
LANE_ADC_DEBUG = 0x14
LANE_ADC_DATA = 0x18
LANE_ADC_RATE = 0x1C
LANE_UNMAPPED = 0x0C
RESP_OKAY = 0
RESETS = ("dac_cmd_resetn", "dac_data_resetn", "adc_cmd_resetn", "adc_data_resetn")
# Read lanes: (port prefix, lane offset, read-data tag)
READ_LANES = (
    ("dac_debug", LANE_DAC_DEBUG, 0xDDB),
    ("dac_data", LANE_DAC_DATA, 0xDAC),
    ("adc_debug", LANE_ADC_DEBUG, 0xADB),
    ("adc_data", LANE_ADC_DATA, 0xADC),
)


def addr(board, lane):
    return board * PAGE + lane


def data_word(board, tag):
    return (tag << 16) | (board << 8) | 0xA5


def backlog_word(board):
    return 0x5A000000 | (board << 16) | 0x123


# Record every cycle's rate-lane write strobe as (rate_wr_en, cmd_wr_data)
async def collect_rate_strobes(dut, out):
    while True:
        await RisingEdge(dut.aclk)
        await ReadOnly()
        en = int(dut.rate_wr_en.value)
        if en:
            out.append((en, int(dut.cmd_wr_data.value)))


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
    # FIFO models: not full, not empty, distinct read data per board
    dut.dac_cmd_full.value = 0
    dut.adc_cmd_full.value = 0
    dut.mode_block.value = 0
    for name, _, tag in READ_LANES:
        getattr(dut, f"{name}_empty").value = 0
        bus = 0
        for b in range(8):
            bus |= data_word(b, tag) << (32 * b)
        getattr(dut, f"{name}_rd_data").value = bus
    dut.rate_backlog.value = 0
    dut.aresetn.value = 0
    for s in RESETS:
        getattr(dut, s).value = 0
    for _ in range(3):
        await RisingEdge(dut.aclk)
    dut.aresetn.value = 1
    for s in RESETS:
        getattr(dut, s).value = 0xFF
    await RisingEdge(dut.aclk)


async def axi_write(dut, address, data):
    dut.s_axi_awaddr.value = address
    dut.s_axi_wdata.value = data
    dut.s_axi_awvalid.value = 1
    dut.s_axi_wvalid.value = 1
    dut.s_axi_bready.value = 1
    await ReadOnly()
    dac_wr = int(dut.dac_cmd_wr_en.value)
    adc_wr = int(dut.adc_cmd_wr_en.value)
    wr_data = int(dut.cmd_wr_data.value)
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
    return bresp, dac_wr, adc_wr, wr_data


async def axi_read(dut, address):
    """Returns (rresp, rdata, {lane name: rd_en mask})."""
    dut.s_axi_araddr.value = address
    dut.s_axi_arvalid.value = 1
    dut.s_axi_rready.value = 1
    await ReadOnly()
    rd_en = {name: int(getattr(dut, f"{name}_rd_en").value) for name, _, _ in READ_LANES}
    await RisingEdge(dut.aclk)
    dut.s_axi_arvalid.value = 0
    await ReadOnly()
    while not int(dut.s_axi_rvalid.value):
        await RisingEdge(dut.aclk)
        await ReadOnly()
    rdata = int(dut.s_axi_rdata.value)
    rresp = int(dut.s_axi_rresp.value)
    await RisingEdge(dut.aclk)
    dut.s_axi_rready.value = 0
    return rresp, rdata, rd_en


async def setup(dut):
    cocotb.start_soon(Clock(dut.aclk, 10, units="ns").start())
    await reset(dut)
    return int(dut.NUM_BOARDS.value)


@cocotb.test()
async def test_write_routing(dut):
    n = await setup(dut)
    for b in range(n):
        bresp, dac_wr, adc_wr, wr_data = await axi_write(dut, addr(b, LANE_DAC_CMD), 0xD0000000 | b)
        assert bresp == RESP_OKAY, f"board {b} DAC write bresp {bresp}"
        assert dac_wr == (1 << b) and adc_wr == 0, f"board {b} DAC write misrouted ({dac_wr:#x}, {adc_wr:#x})"
        assert wr_data == 0xD0000000 | b
        bresp, dac_wr, adc_wr, wr_data = await axi_write(dut, addr(b, LANE_ADC_CMD), 0xA0000000 | b)
        assert bresp == RESP_OKAY, f"board {b} ADC write bresp {bresp}"
        assert adc_wr == (1 << b) and dac_wr == 0, f"board {b} ADC write misrouted ({dac_wr:#x}, {adc_wr:#x})"
        assert wr_data == 0xA0000000 | b


@cocotb.test()
async def test_read_routing(dut):
    n = await setup(dut)
    for b in range(n):
        for name, lane, tag in READ_LANES:
            rresp, rdata, rd_en = await axi_read(dut, addr(b, lane))
            assert rresp == RESP_OKAY
            for other, mask in rd_en.items():
                exp = (1 << b) if other == name else 0
                assert mask == exp, f"board {b} {name} read: {other}_rd_en {mask:#x}, expected {exp:#x}"
            assert rdata == data_word(b, tag), f"board {b} {name} rdata {rdata:#x}"


@cocotb.test()
async def test_unmapped_and_cross_direction(dut):
    await setup(dut)
    # Writes to read lanes and to an unmapped lane push nothing
    for lane in (LANE_DAC_DEBUG, LANE_DAC_DATA, LANE_ADC_DEBUG, LANE_ADC_DATA, LANE_UNMAPPED):
        bresp, dac_wr, adc_wr, _ = await axi_write(dut, addr(1, lane), 0x1234)
        assert bresp == RESP_OKAY and dac_wr == 0 and adc_wr == 0, f"write to lane {lane:#x} pushed a FIFO"
    # Reads from write lanes and an unmapped lane pop nothing and return zero
    for lane in (LANE_DAC_CMD, LANE_ADC_CMD, LANE_UNMAPPED):
        rresp, rdata, rd_en = await axi_read(dut, addr(1, lane))
        assert rresp == RESP_OKAY and not any(rd_en.values()), f"read of lane {lane:#x} popped a FIFO"
        assert rdata == 0, f"read of lane {lane:#x} returned {rdata:#x}"
    await ReadOnly()
    assert int(dut.mode_viol.value) == 0
    assert int(dut.dac_cmd_overflow.value) == 0 and int(dut.adc_cmd_overflow.value) == 0
    assert int(dut.dac_data_underflow.value) == 0 and int(dut.adc_data_underflow.value) == 0


@cocotb.test()
async def test_absent_board(dut):
    n = await setup(dut)
    if n >= 8:
        dut._log.info("NUM_BOARDS is 8; no absent board to test")
        return
    b = n
    dut.dac_cmd_full.value = 0xFF
    dut.dac_data_empty.value = 0xFF
    bresp, dac_wr, adc_wr, _ = await axi_write(dut, addr(b, LANE_DAC_CMD), 0x1)
    assert bresp == RESP_OKAY and dac_wr == 0 and adc_wr == 0, "absent board write pushed a FIFO"
    rresp, rdata, rd_en = await axi_read(dut, addr(b, LANE_DAC_DATA))
    assert rresp == RESP_OKAY and rdata == 0 and not any(rd_en.values()), "absent board read popped a FIFO"
    await ReadOnly()
    assert int(dut.dac_cmd_overflow.value) == 0, "absent board raised an overflow flag"
    assert int(dut.dac_data_underflow.value) == 0, "absent board raised an underflow flag"


@cocotb.test()
async def test_mode_block(dut):
    await setup(dut)
    dut.mode_block.value = 1 << 2
    # Board 2's DMA-owned lanes are discarded and flag mode_viol[2]
    _, dac_wr, _, _ = await axi_write(dut, addr(2, LANE_DAC_CMD), 0x1)
    assert dac_wr == 0, "blocked DAC write still pushed"
    await ReadOnly()
    assert int(dut.mode_viol.value) == (1 << 2), f"mode_viol {int(dut.mode_viol.value):#x} after blocked DAC write"
    await RisingEdge(dut.aclk)
    _, rdata, rd_en = await axi_read(dut, addr(2, LANE_ADC_DATA))
    assert rd_en["adc_data"] == 0 and rdata == 0, "blocked ADC read still popped"
    # Board 2's PIO-only lanes still work
    _, _, adc_wr, _ = await axi_write(dut, addr(2, LANE_ADC_CMD), 0x2)
    assert adc_wr == (1 << 2), "ADC command write wrongly blocked"
    for name, lane, _ in READ_LANES[:3]:
        _, _, rd_en = await axi_read(dut, addr(2, lane))
        assert rd_en[name] == (1 << 2), f"{name} read wrongly blocked"
    # Board 3 is unaffected
    _, dac_wr, _, _ = await axi_write(dut, addr(3, LANE_DAC_CMD), 0x3)
    assert dac_wr == (1 << 3), "unblocked board's DAC write was blocked"
    await ReadOnly()
    assert int(dut.mode_viol.value) == (1 << 2), "mode_viol spread to another board"
    # mode_viol clears on the owning FIFOs' resets
    await RisingEdge(dut.aclk)
    dut.dac_cmd_resetn.value = 0xFF & ~(1 << 2)
    dut.adc_data_resetn.value = 0xFF & ~(1 << 2)
    await RisingEdge(dut.aclk)
    dut.dac_cmd_resetn.value = 0xFF
    dut.adc_data_resetn.value = 0xFF
    await ReadOnly()
    assert int(dut.mode_viol.value) == 0, "mode_viol did not clear on FIFO reset"


@cocotb.test()
async def test_rate_lane(dut):
    n = await setup(dut)
    bus = 0
    for b in range(8):
        bus |= backlog_word(b) << (32 * b)
    dut.rate_backlog.value = bus
    strobes = []
    mon = cocotb.start_soon(collect_rate_strobes(dut, strobes))
    await RisingEdge(dut.aclk)
    # Command-lane and unmapped writes never strobe the rate lane
    for lane in (LANE_DAC_CMD, LANE_ADC_CMD, LANE_UNMAPPED):
        await axi_write(dut, addr(0, lane), 0x1)
    assert strobes == [], f"non-rate write strobed the rate lane: {strobes}"
    # The rate lane ignores mode_block and the command FIFO full flags
    dut.mode_block.value = 0xFF
    dut.dac_cmd_full.value = 0xFF
    dut.adc_cmd_full.value = 0xFF
    for b in range(n):
        strobes.clear()
        bresp, dac_wr, adc_wr, _ = await axi_write(dut, addr(b, LANE_ADC_RATE), 0x100 + b)
        assert bresp == RESP_OKAY and dac_wr == 0 and adc_wr == 0, f"board {b} rate write pushed a FIFO"
        assert strobes == [(1 << b, 0x100 + b)], f"board {b} rate write strobes {strobes}"
        # A backlog read returns that board's word and pops nothing
        rresp, rdata, rd_en = await axi_read(dut, addr(b, LANE_ADC_RATE))
        assert rresp == RESP_OKAY and not any(rd_en.values()), f"board {b} rate read popped a FIFO"
        assert rdata == backlog_word(b), f"board {b} rate read {rdata:#x}"
    # An absent board's rate lane neither strobes nor reads back
    if n < 8:
        strobes.clear()
        await axi_write(dut, addr(n, LANE_ADC_RATE), 0x1)
        assert strobes == [], "absent board rate write strobed"
        _, rdata, _ = await axi_read(dut, addr(n, LANE_ADC_RATE))
        assert rdata == 0, f"absent board rate read {rdata:#x}"
    mon.cancel()
    await ReadOnly()
    assert int(dut.dac_cmd_overflow.value) == 0 and int(dut.adc_cmd_overflow.value) == 0, "rate write raised an overflow"


@cocotb.test()
async def test_overflow_underflow(dut):
    await setup(dut)
    dut.dac_cmd_full.value = 1 << 1
    _, dac_wr, _, _ = await axi_write(dut, addr(1, LANE_DAC_CMD), 0x1)
    assert dac_wr == 0, "write to a full DAC command FIFO still pushed"
    dut.adc_cmd_full.value = 1 << 4
    await axi_write(dut, addr(4, LANE_ADC_CMD), 0x1)
    dut.dac_data_empty.value = 1 << 0
    _, rdata, rd_en = await axi_read(dut, addr(0, LANE_DAC_DATA))
    assert rd_en["dac_data"] == 0 and rdata == 0, "read of an empty DAC data FIFO still popped"
    dut.adc_data_empty.value = 1 << 5
    await axi_read(dut, addr(5, LANE_ADC_DATA))
    # Empty debug lanes return zero without a flag
    dut.dac_debug_empty.value = 1 << 3
    dut.adc_debug_empty.value = 1 << 3
    for name, lane in (("dac_debug", LANE_DAC_DEBUG), ("adc_debug", LANE_ADC_DEBUG)):
        _, rdata, rd_en = await axi_read(dut, addr(3, lane))
        assert rd_en[name] == 0 and rdata == 0, f"read of an empty {name} FIFO still popped"
    await ReadOnly()
    assert int(dut.dac_cmd_overflow.value) == (1 << 1), "dac_cmd_overflow wrong"
    assert int(dut.adc_cmd_overflow.value) == (1 << 4), "adc_cmd_overflow wrong"
    assert int(dut.dac_data_underflow.value) == (1 << 0), "dac_data_underflow wrong"
    assert int(dut.adc_data_underflow.value) == (1 << 5), "adc_data_underflow wrong"
    assert int(dut.mode_viol.value) == 0, "over/underflow raised mode_viol"
    # Each flag clears only on its own FIFO's reset
    await RisingEdge(dut.aclk)
    dut.dac_cmd_resetn.value = 0xFF & ~(1 << 1)
    await RisingEdge(dut.aclk)
    dut.dac_cmd_resetn.value = 0xFF
    await ReadOnly()
    assert int(dut.dac_cmd_overflow.value) == 0, "dac_cmd_overflow did not clear on reset"
    assert int(dut.adc_cmd_overflow.value) == (1 << 4), "adc_cmd_overflow wrongly cleared"
    assert int(dut.dac_data_underflow.value) == (1 << 0), "dac_data_underflow wrongly cleared"
    assert int(dut.adc_data_underflow.value) == (1 << 5), "adc_data_underflow wrongly cleared"
