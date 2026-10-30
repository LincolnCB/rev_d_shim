import cocotb
from cocotb.triggers import Timer

HALF_PERIOD_NS = 5


async def tick(dut, cycles=1):
    # Drive the clock by hand so each rising edge is explicit (and the clock can sit stopped)
    for _ in range(cycles):
        dut.clk.value = 0
        await Timer(HALF_PERIOD_NS, "ns")
        dut.clk.value = 1
        await Timer(HALF_PERIOD_NS, "ns")


async def settle(dut):
    await Timer(1, "ns")


async def hold_reset(dut):
    dut.clk.value = 0
    dut.rst_in_n.value = 0
    await settle(dut)
    assert int(dut.rst_out_n.value) == 0, "output not low while reset is held"


async def release_fully(dut):
    depth = int(dut.DEPTH.value)
    await hold_reset(dut)
    dut.rst_in_n.value = 1
    await tick(dut, depth)
    assert int(dut.rst_out_n.value) == 1, f"output not released after {depth} edges"


@cocotb.test()
async def test_release_latency(dut):
    depth = int(dut.DEPTH.value)
    await hold_reset(dut)
    dut.rst_in_n.value = 1
    await settle(dut)
    assert int(dut.rst_out_n.value) == 0, "output released without a clock edge"
    for edge in range(1, depth + 1):
        await tick(dut)
        expected = 1 if edge >= depth else 0
        assert int(dut.rst_out_n.value) == expected, \
            f"after edge {edge} of {depth}: rst_out_n={int(dut.rst_out_n.value)}, expected {expected}"
    await tick(dut, 5)
    assert int(dut.rst_out_n.value) == 1, "output dropped while input stayed released"


@cocotb.test()
async def test_async_assert_without_clock(dut):
    await release_fully(dut)
    # Clock parked low: assertion must not wait for an edge
    dut.clk.value = 0
    await settle(dut)
    dut.rst_in_n.value = 0
    await settle(dut)
    assert int(dut.rst_out_n.value) == 0, "assert waited for a clock edge"


@cocotb.test()
async def test_held_reset_ignores_clock(dut):
    await hold_reset(dut)
    for _ in range(10):
        await tick(dut)
        assert int(dut.rst_out_n.value) == 0, "output released while input still held"


@cocotb.test()
async def test_short_pulse(dut):
    depth = int(dut.DEPTH.value)
    await release_fully(dut)
    # A pulse shorter than a clock period, landing between edges, still resets the whole chain
    dut.clk.value = 0
    await settle(dut)
    dut.rst_in_n.value = 0
    await settle(dut)
    assert int(dut.rst_out_n.value) == 0, "short pulse did not assert the output"
    dut.rst_in_n.value = 1
    await settle(dut)
    assert int(dut.rst_out_n.value) == 0, "output released at the pulse edge, not a clock edge"
    for edge in range(1, depth + 1):
        await tick(dut)
        expected = 1 if edge >= depth else 0
        assert int(dut.rst_out_n.value) == expected, \
            f"after edge {edge} of {depth} post-pulse: rst_out_n={int(dut.rst_out_n.value)}, expected {expected}"
