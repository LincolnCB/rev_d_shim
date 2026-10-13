import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly
import random


async def do_mult(dut, a, b):
    dut.multiplicand.value = a
    dut.multiplier.value = b
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0
    await ReadOnly()
    while not int(dut.done.value):
        await RisingEdge(dut.clk)
        await ReadOnly()
    r = int(dut.result.value)
    await RisingEdge(dut.clk)
    return r


@cocotb.test()
async def test_multiply(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.start.value = 0
    dut.multiplicand.value = 0
    dut.multiplier.value = 0
    dut.resetn.value = 0
    for _ in range(3):
        await RisingEdge(dut.clk)
    dut.resetn.value = 1
    await RisingEdge(dut.clk)

    aw = int(dut.MULTIPLICAND_WIDTH.value)
    bw = int(dut.MULTIPLIER_WIDTH.value)
    amax = (1 << aw) - 1
    bmax = (1 << bw) - 1

    cases = [(0, 0), (1, 1), (amax, bmax), (amax, 1), (1, bmax),
             (amax, 0), (0, bmax), (amax >> 1, bmax >> 1)]
    for _ in range(50):
        cases.append((random.randint(0, amax), random.randint(0, bmax)))

    for a, b in cases:
        r = await do_mult(dut, a, b)
        assert r == a * b, f"{a} * {b} = {a * b}, got {r}"


# Back-to-back multiplies: result must stay valid for one transaction before the next start.
@cocotb.test()
async def test_back_to_back(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.start.value = 0
    dut.resetn.value = 0
    for _ in range(3):
        await RisingEdge(dut.clk)
    dut.resetn.value = 1
    await RisingEdge(dut.clk)

    amax = (1 << int(dut.MULTIPLICAND_WIDTH.value)) - 1
    bmax = (1 << int(dut.MULTIPLIER_WIDTH.value)) - 1
    for _ in range(20):
        a = random.randint(0, amax)
        b = random.randint(0, bmax)
        r = await do_mult(dut, a, b)
        assert r == a * b, f"{a} * {b} = {a * b}, got {r}"
        await RisingEdge(dut.clk)
