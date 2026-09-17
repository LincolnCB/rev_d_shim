import cocotb
from cocotb.triggers import RisingEdge, ReadOnly
import random

from adc_packetizer_base import adc_packetizer_base


async def setup_testbench(dut, clk_period=10, time_unit="ns"):
    tb = adc_packetizer_base(dut, clk_period, time_unit)
    return tb


@cocotb.test()
async def test_reset(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_reset")
    await tb.start_clock()
    await tb.reset()

    # With an empty FIFO the packetizer must not offer a beat.
    tb.dut.m_axis_tready.value = 1
    await ReadOnly()
    assert int(dut.m_axis_tvalid.value) == 0, "tvalid should be low when the FIFO is empty"
    await RisingEdge(dut.aclk)
    await RisingEdge(dut.aclk)
    tb.kill_clock()


@cocotb.test()
async def test_preloaded_cap_and_tail(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_preloaded_cap_and_tail")
    await tb.start_clock()
    await tb.reset()

    # More words resident than two packet caps, consumer always ready: the first two
    # packets hit MAX_PACKET_WORDS and the tail closes adaptively as the FIFO drains.
    n = 2 * tb.MAX_PACKET_WORDS + tb.MAX_PACKET_WORDS // 2
    tb.preload_words(n)
    model = cocotb.start_soon(tb.fifo_model_task())
    ready = cocotb.start_soon(tb.ready_driver(prob=1.0))
    await tb.axis_scoreboard(len(tb.written))

    tb.check_byte_exact()
    assert tb.packet_lengths() == [tb.MAX_PACKET_WORDS, tb.MAX_PACKET_WORDS, tb.MAX_PACKET_WORDS // 2], \
        f"unexpected packet lengths {tb.packet_lengths()}"
    assert tb.leftover == [], "stream did not end on a tlast"

    model.cancel()
    ready.cancel()
    tb.kill_clock()


@cocotb.test()
async def test_trickle_mixed_reads(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_trickle_mixed_reads")
    await tb.start_clock()
    await tb.reset()

    # Full (4-word) and single-channel (1-word) reads share the lane. Each group is fed in
    # one cycle with idle gaps, so each drains as its own packet: framing follows the data,
    # not any fixed sample-set boundary. Mixed sizes are exactly what broke 4-word atomicity.
    sizes = [4, 1, 4, 1, 1]
    tb.trickle_groups(sizes, gap=8)
    model = cocotb.start_soon(tb.fifo_model_task())
    ready = cocotb.start_soon(tb.ready_driver(prob=1.0))
    await tb.axis_scoreboard(len(tb.written))

    tb.check_byte_exact()
    assert tb.packet_lengths() == sizes, \
        f"expected one packet per read group {sizes}, got {tb.packet_lengths()}"

    model.cancel()
    ready.cancel()
    tb.kill_clock()


@cocotb.test()
async def test_drip_single_word_packets(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_drip_single_word_packets")
    await tb.start_clock()
    await tb.reset()

    # A word arriving each cycle with the reader keeping pace never lets the FIFO build a
    # backlog, so every word leaves as its own single-word packet -- the case that broke
    # the old 4-word-atomic assumption. Software recombines these downstream.
    n = 6
    tb.drip_words(n)
    model = cocotb.start_soon(tb.fifo_model_task())
    ready = cocotb.start_soon(tb.ready_driver(prob=1.0))
    await tb.axis_scoreboard(len(tb.written))

    tb.check_byte_exact()
    assert tb.packet_lengths() == [1] * n, \
        f"expected {n} single-word packets, got {tb.packet_lengths()}"

    model.cancel()
    ready.cancel()
    tb.kill_clock()


@cocotb.test()
async def test_backpressure(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_backpressure")
    random.seed(1234)
    await tb.start_clock()
    await tb.reset()

    # Random consumer stalls must not change the data or the framing: with all words
    # resident, packet boundaries are set by the count-based drain and the cap, which are
    # unaffected by when tready deasserts.
    n = 2 * tb.MAX_PACKET_WORDS + tb.MAX_PACKET_WORDS // 2
    tb.preload_words(n)
    model = cocotb.start_soon(tb.fifo_model_task())
    ready = cocotb.start_soon(tb.ready_driver(prob=0.5))
    await tb.axis_scoreboard(len(tb.written))

    tb.check_byte_exact()
    for plen in tb.packet_lengths():
        assert plen <= tb.MAX_PACKET_WORDS, f"packet {plen} exceeds cap"
    assert tb.packet_lengths() == [tb.MAX_PACKET_WORDS, tb.MAX_PACKET_WORDS, tb.MAX_PACKET_WORDS // 2], \
        f"unexpected packet lengths {tb.packet_lengths()}"

    model.cancel()
    ready.cancel()
    tb.kill_clock()
