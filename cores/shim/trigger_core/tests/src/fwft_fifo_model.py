from collections import deque

class fwft_fifo_model:
    """
    Software model of a FWFT FIFO intented to work with cocotb.
    Models a FWFT FIFO with given depth.
    Timing and control of read and write operations must be handled externally.
    """

    def __init__ (self, dut, name, DEPTH):
        self.dut = dut
        self.name = name
        self.DEPTH = DEPTH
        self.fifo = deque()

    def get_depth(self):
        return self.DEPTH
    
    def get_num_items(self):
        return len(self.fifo)

    def reset(self):
        self.fifo.clear()

    def is_empty(self):
        return len(self.fifo) == 0
    
    def is_almost_empty(self):
        return len(self.fifo) <= 2
    
    def is_full(self):
        return len(self.fifo) >= self.DEPTH
    
    def is_almost_full(self):
        return len(self.fifo) >= (self.DEPTH - 2)

    def write_item(self, cmd):
        if len(self.fifo) < self.DEPTH:
            self.fifo.append(cmd)
            self.dut._log.info(f"{self.name}: Wrote item {cmd:08x}, number of items in buf: {len(self.fifo)}")
        else:
            self.dut._log.warning(f"{self.name}: Attempted to write item but FIFO is full")

    # Pop item from the FIFO. This is used to model the read acknowledgment from the DUT.
    # The actual item is "falling through" and can be accessed via peek_item
    def pop_item(self):
        if len(self.fifo) > 0:
            cmd = self.fifo.popleft()
            self.dut._log.info(f"{self.name}: Read item {cmd:08x}, number of items in buf: {len(self.fifo)}")
            return cmd
        else:
            self.dut._log.warning(f"{self.name}: Attempted to read item but FIFO is empty")
            return None

    # For FWFT behavior we need to be able to peek at the next item without removing it.
    # This is used to model the output of the FIFO.
    def peek_item(self):
        if len(self.fifo) > 0:
            cmd = self.fifo[0]
            return cmd
        else:
            self.dut._log.warning(f"{self.name}: Attempted to peek item but FIFO is empty")
            return None