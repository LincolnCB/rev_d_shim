# filepath: projects/rev_d_shim/modules/8ch_and.tcl

# Create input pins for the eight channels
create_bd_pin -dir I Op1
create_bd_pin -dir I Op2
create_bd_pin -dir I Op3
create_bd_pin -dir I Op4
create_bd_pin -dir I Op5
create_bd_pin -dir I Op6
create_bd_pin -dir I Op7
create_bd_pin -dir I Op8

# Create output pin for the AND operation result
create_bd_pin -dir O Res

# Instantiate a chain of AND gates to combine all eight channels
cell xilinx.com:ip:util_vector_logic and_0 {
  C_SIZE 1
  C_OPERATION and
} {
  Op1 Op1
  Op2 Op2
}

cell xilinx.com:ip:util_vector_logic and_1 {
  C_SIZE 1
  C_OPERATION and
} {
  Op1 and_0/Res
  Op2 Op3
}

cell xilinx.com:ip:util_vector_logic and_2 {
  C_SIZE 1
  C_OPERATION and
} {
  Op1 and_1/Res
  Op2 Op4
}

cell xilinx.com:ip:util_vector_logic and_3 {
  C_SIZE 1
  C_OPERATION and
} {
  Op1 and_2/Res
  Op2 Op5
}

cell xilinx.com:ip:util_vector_logic and_4 {
  C_SIZE 1
  C_OPERATION and
} {
  Op1 and_3/Res
  Op2 Op6
}

cell xilinx.com:ip:util_vector_logic and_5 {
  C_SIZE 1
  C_OPERATION and
} {
  Op1 and_4/Res
  Op2 Op7
}

cell xilinx.com:ip:util_vector_logic and_6 {
  C_SIZE 1
  C_OPERATION and
} {
  Op1 and_5/Res
  Op2 Op8
  Res Res
}
