import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly
from collections import deque
import random


# Drives the DUT's first-word-fall-through FIFO read port from a software model and
# scoreboards the AXIS output for chunk-atomic framing. A single aclk domain: the FIFO
# read side and the packetizer both run on it, matching fifo_async's read-side count.
class adc_packetizer_base:

    def __init__(self, dut, clk_period=10, time_unit="ns"):
        self.dut = dut
        self.clk_period = clk_period
        self.time_unit = time_unit

        # Parameters from the DUT
        self.DATA_WIDTH       = int(dut.DATA_WIDTH.value)
        self.DEST_WIDTH       = int(dut.DEST_WIDTH.value)
        self.FIFO_COUNT_WIDTH = int(dut.FIFO_COUNT_WIDTH.value)
        self.CHUNK_WORDS      = int(dut.CHUNK_WORDS.value)
        self.MAX_CHUNKS       = int(dut.MAX_CHUNKS.value)
        self.BOARD_INDEX      = int(dut.BOARD_INDEX.value)
        self.MAX_PACKET_WORDS = self.MAX_CHUNKS * self.CHUNK_WORDS
        self.DATA_MASK        = (1 << self.DATA_WIDTH) - 1

        self.dut._log.info(f"DUT params: DATA_WIDTH={self.DATA_WIDTH}, DEST_WIDTH={self.DEST_WIDTH}, "
                           f"FIFO_COUNT_WIDTH={self.FIFO_COUNT_WIDTH}, CHUNK_WORDS={self.CHUNK_WORDS}, "
                           f"MAX_CHUNKS={self.MAX_CHUNKS}, BOARD_INDEX={self.BOARD_INDEX}")

        # FIFO model state and per-cycle feed plan (each entry is the list of words pushed that cycle).
        self.fifo = deque()
        self.feed_schedule = deque()

        # What was fed in, and what the AXIS side produced (for byte-exact and framing checks).
        self.written = []
        self.received = []
        self.packets = []       # completed packets, each a list of words
        self.leftover = []      # words seen after the last tlast (should be empty at a clean end)

        self.next_val = 1       # unique, non-zero payload generator
        self.clk_task = None

        # Drive inputs to a defined idle state
        self.dut.aresetn.value = 1
        self.dut.fifo_rd_data.value = 0
        self.dut.fifo_empty.value = 1
        self.dut.fifo_count_rd_clk.value = 0
        self.dut.m_axis_tready.value = 0

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
        self.fifo.clear()
        self._present()
        await RisingEdge(self.dut.aclk)
        await RisingEdge(self.dut.aclk)
        self.dut.aresetn.value = 1
        await RisingEdge(self.dut.aclk)
        self.dut._log.info("Reset complete.")

    # --- Payload / feed helpers ---------------------------------------------

    def make_chunk(self):
        chunk = [(self.next_val + i) & self.DATA_MASK for i in range(self.CHUNK_WORDS)]
        self.next_val += self.CHUNK_WORDS
        return chunk

    # Preload n whole chunks so they are all resident before draining starts.
    def preload_chunks(self, n):
        words = []
        for _ in range(n):
            words.extend(self.make_chunk())
        self.written.extend(words)
        self.feed_schedule.append(list(words))

    # Feed n chunks one at a time, each separated by `gap` idle cycles, to exercise
    # single-chunk adaptive packets and the "start only when a whole chunk is present" gate.
    def trickle_chunks(self, n, gap):
        for _ in range(n):
            chunk = self.make_chunk()
            self.written.extend(chunk)
            self.feed_schedule.append(list(chunk))
            for _ in range(gap):
                self.feed_schedule.append([])

    # --- FIFO read-port model ------------------------------------------------

    def _present(self):
        if self.fifo:
            self.dut.fifo_rd_data.value = self.fifo[0]
            self.dut.fifo_empty.value = 0
        else:
            self.dut.fifo_rd_data.value = 0
            self.dut.fifo_empty.value = 1
        self.dut.fifo_count_rd_clk.value = len(self.fifo)

    async def fifo_model_task(self):
        while True:
            self._present()
            await ReadOnly()
            rd = int(self.dut.fifo_rd_en.value)
            await RisingEdge(self.dut.aclk)
            if rd and self.fifo:
                self.fifo.popleft()
            adds = self.feed_schedule.popleft() if self.feed_schedule else []
            self.fifo.extend(adds)

    # --- AXIS consumer -------------------------------------------------------

    async def ready_driver(self, prob=1.0):
        while True:
            self.dut.m_axis_tready.value = 1 if random.random() < prob else 0
            await RisingEdge(self.dut.aclk)

    # Collect `expected_total` beats, validating framing as packets close.
    async def axis_scoreboard(self, expected_total, max_cycles=100000):
        cur = []
        cycles = 0
        while len(self.received) < expected_total:
            await ReadOnly()
            if int(self.dut.m_axis_tvalid.value) and int(self.dut.m_axis_tready.value):
                data = int(self.dut.m_axis_tdata.value)
                last = int(self.dut.m_axis_tlast.value)
                dest = int(self.dut.m_axis_tdest.value)
                assert dest == self.BOARD_INDEX, f"tdest {dest} != BOARD_INDEX {self.BOARD_INDEX}"
                self.received.append(data)
                cur.append(data)
                if last:
                    assert len(cur) % self.CHUNK_WORDS == 0, \
                        f"tlast off a chunk boundary: packet len {len(cur)} not a multiple of {self.CHUNK_WORDS}"
                    assert len(cur) <= self.MAX_PACKET_WORDS, \
                        f"packet len {len(cur)} exceeds cap {self.MAX_PACKET_WORDS}"
                    self.packets.append(cur)
                    cur = []
            await RisingEdge(self.dut.aclk)
            cycles += 1
            assert cycles < max_cycles, \
                f"scoreboard timed out after {cycles} cycles with {len(self.received)}/{expected_total} words"
        self.leftover = cur

    # --- Checks --------------------------------------------------------------

    def check_byte_exact(self):
        assert self.received == self.written, \
            f"data mismatch: received {len(self.received)} words, expected {len(self.written)}"

    def packet_lengths(self):
        return [len(p) for p in self.packets]
