# 64-bit Status word
create_bd_pin -dir O -from 64 -to 0 fifo_sts_word

cell xilinx.com:ip:xlconstant:1.1 sts_word_padding {
  CONST_VAL 0
  CONST_WIDTH 6
} {}
cell xilinx.com:ip:xlconstant:1.1 TODO_read_count {
  CONST_VAL 0
  CONST_WIDTH 24
} {}
cell xilinx.com:ip:xlconstant:1.1 TODO_write_count {
  CONST_VAL 0
  CONST_WIDTH 24
} {}
cell xilinx.com:ip:xlconstant:1.1 TODO_underflow {
  CONST_VAL 0
  CONST_WIDTH 1
} {}
cell xilinx.com:ip:xlconstant:1.1 TODO_empty {
  CONST_VAL 0
  CONST_WIDTH 1
} {}
cell xilinx.com:ip:xlconstant:1.1 TODO_overflow {
  CONST_VAL 0
  CONST_WIDTH 1
} {}
cell xilinx.com:ip:xlconstant:1.1 TODO_full {
  CONST_VAL 0
  CONST_WIDTH 1
} {}

# Concatenate the read and empty/underflow status signals into a 32-bit word
# 23:0  -- 24b Read count
# 29:24 --     Reserved
# 30    --  1b Underflow
# 31    --  1b FIFO empty
cell xilinx.com:ip:xlconcat:2.1 read_empty_sts {
  NUM_PORTS 4
} {
  In0 TODO_read_count/dout
  In1 sts_word_padding/dout
  In2 TODO_underflow/dout
  In3 TODO_empty/dout
}
# Concatenate the write and full/overflow status signals into a 32-bit word
# 23:0  -- 24b Write count
# 29:24 --     Reserved
# 30    --  1b Overflow
# 31    --  1b FIFO full
cell xilinx.com:ip:xlconcat:2.1 write_full_sts {
  NUM_PORTS 4
} {
  In0 TODO_write_count/dout
  In1 sts_word_padding/dout
  In2 TODO_overflow/dout
  In3 TODO_full/dout
}

# Concatenate all the status signals into a 64-bit word
# 31:0  -- 32b Read/empty word (31: empty; 30: underflow; 23:0: read count)
# 63:32 -- 32b Write/full word (31: full; 30: overflow; 23:0: write count)
cell xilinx.com:ip:xlconcat:2.1 sts_word {
  NUM_PORTS 2
} {
  In0 read_empty_sts/dout
  In1 write_full_sts/dout
  dout fifo_sts_word
}
