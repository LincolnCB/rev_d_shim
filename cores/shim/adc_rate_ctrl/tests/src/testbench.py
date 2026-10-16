import cocotb
from cocotb.triggers import RisingEdge, ReadOnly

from adc_rate_ctrl_base import adc_rate_ctrl_base


async def setup_testbench(dut, clk_period=10, time_unit="ns"):
    tb = adc_rate_ctrl_base(dut, clk_period, time_unit)
    return tb


@cocotb.test()
async def test_reset(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_reset")
    await tb.start_clock()
    await tb.reset()

    # Every backlog starts at zero and no board asks to accumulate (backlog 0 < PACKET_WORDS).
    for b in range(tb.NUM_BOARDS):
        assert await tb.read_backlog(b) == 0, f"board {b} backlog not zero after reset"
        assert await tb.read_accumulate(b) == 0, f"board {b} accumulate asserted at reset"

    tb.kill_clock()


@cocotb.test()
async def test_emit_increments_backlog(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_emit_increments_backlog")
    await tb.start_clock()
    await tb.reset()

    # Each emitted word raises the board's backlog by one.
    await tb.emit(1, cycles=10)
    assert await tb.read_backlog(1) == 10, "backlog did not count the emitted words"

    tb.kill_clock()


@cocotb.test()
async def test_read_delta_subtracts(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_read_delta_subtracts")
    await tb.start_clock()
    await tb.reset()

    # A reported read delta lowers the backlog by that many words.
    await tb.emit(0, cycles=20)
    await tb.report_read(0, 8)
    assert await tb.read_backlog(0) == 12, "read delta did not subtract from the backlog"

    # Over-reporting more than is resident floors at zero rather than underflowing.
    await tb.report_read(0, 100)
    assert await tb.read_backlog(0) == 0, "backlog underflowed instead of saturating at zero"

    tb.kill_clock()


@cocotb.test()
async def test_threshold_accumulate(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_threshold_accumulate")
    await tb.start_clock()
    await tb.reset()

    # The threshold is the fixed packet size: below it the board drains eagerly, at or above it
    # it packs dense.
    t = tb.PACKET_WORDS
    await tb.emit(2, cycles=t - 1)
    assert await tb.read_accumulate(2) == 0, "accumulate asserted below the packet size"
    await tb.emit(2, cycles=1)
    assert await tb.read_accumulate(2) == 1, "accumulate did not assert at the packet size"

    # Draining back below the threshold returns to eager.
    await tb.report_read(2, 1)
    assert await tb.read_accumulate(2) == 0, "accumulate stuck high after draining below the packet size"

    tb.kill_clock()


@cocotb.test()
async def test_per_board_reset(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_per_board_reset")
    await tb.start_clock()
    await tb.reset()

    # The per-board backlog reset (wired to the adc_data FIFO reset) clears just that board.
    await tb.emit(3, cycles=7)
    assert await tb.read_backlog(3) == 7, "backlog did not accumulate before reset"
    await tb.backlog_reset(3)
    assert await tb.read_backlog(3) == 0, "per-board reset did not clear the backlog"

    tb.kill_clock()


@cocotb.test()
async def test_board_isolation(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_board_isolation")
    await tb.start_clock()
    await tb.reset()

    # Activity on one board leaves the others untouched.
    await tb.emit(0, cycles=5)
    await tb.report_read(0, 1)
    assert await tb.read_backlog(0) == 4, "board 0 backlog wrong"
    for b in range(1, tb.NUM_BOARDS):
        assert await tb.read_backlog(b) == 0, f"board {b} disturbed by board 0 activity"

    tb.kill_clock()
