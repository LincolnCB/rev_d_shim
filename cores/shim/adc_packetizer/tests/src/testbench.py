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

    # 10 chunks resident up front, consumer always ready. With MAX_CHUNKS=4 the first
    # two packets hit the cap and the tail closes adaptively on the drain.
    tb.preload_chunks(10)
    model = cocotb.start_soon(tb.fifo_model_task())
    ready = cocotb.start_soon(tb.ready_driver(prob=1.0))
    await tb.axis_scoreboard(len(tb.written))

    tb.check_byte_exact()
    assert tb.packet_lengths() == [tb.MAX_PACKET_WORDS, tb.MAX_PACKET_WORDS, 2 * tb.CHUNK_WORDS], \
        f"unexpected packet lengths {tb.packet_lengths()}"
    assert tb.leftover == [], "stream did not end on a tlast"

    model.cancel()
    ready.cancel()
    tb.kill_clock()


@cocotb.test()
async def test_multi_chunk_single_packet(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_multi_chunk_single_packet")
    await tb.start_clock()
    await tb.reset()

    # Three resident chunks, under the cap, all drain into one adaptive packet.
    tb.preload_chunks(3)
    model = cocotb.start_soon(tb.fifo_model_task())
    ready = cocotb.start_soon(tb.ready_driver(prob=1.0))
    await tb.axis_scoreboard(len(tb.written))

    tb.check_byte_exact()
    assert tb.packet_lengths() == [3 * tb.CHUNK_WORDS], \
        f"expected one packet of 3 chunks, got {tb.packet_lengths()}"

    model.cancel()
    ready.cancel()
    tb.kill_clock()


@cocotb.test()
async def test_single_chunk_trickle(dut):
    tb = await setup_testbench(dut)
    tb.dut._log.info("STARTING TEST: test_single_chunk_trickle")
    await tb.start_clock()
    await tb.reset()

    # One chunk at a time with idle gaps: each must leave as its own single-chunk packet,
    # and the packetizer must never start a chunk before a whole one is resident.
    n = 5
    tb.trickle_chunks(n, gap=8)
    model = cocotb.start_soon(tb.fifo_model_task())
    ready = cocotb.start_soon(tb.ready_driver(prob=1.0))
    await tb.axis_scoreboard(len(tb.written))

    tb.check_byte_exact()
    assert tb.packet_lengths() == [tb.CHUNK_WORDS] * n, \
        f"expected {n} single-chunk packets, got {tb.packet_lengths()}"

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

    # Random consumer stalls must not change the data or the framing (all data resident,
    # so packet boundaries are fixed: a capped packet then a 2-chunk tail).
    tb.preload_chunks(6)
    model = cocotb.start_soon(tb.fifo_model_task())
    ready = cocotb.start_soon(tb.ready_driver(prob=0.5))
    await tb.axis_scoreboard(len(tb.written))

    tb.check_byte_exact()
    for plen in tb.packet_lengths():
        assert plen % tb.CHUNK_WORDS == 0, f"packet {plen} not chunk-aligned"
        assert plen <= tb.MAX_PACKET_WORDS, f"packet {plen} exceeds cap"
    assert tb.packet_lengths() == [tb.MAX_PACKET_WORDS, 2 * tb.CHUNK_WORDS], \
        f"unexpected packet lengths {tb.packet_lengths()}"

    model.cancel()
    ready.cancel()
    tb.kill_clock()
