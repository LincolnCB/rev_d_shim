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

# First stage
cell xilinx.com:ip:util_vector_logic and_1_1 {
  C_SIZE 1
  C_OPERATION and
} {
  Op1 Op1
  Op2 Op2
}
cell xilinx.com:ip:util_vector_logic and_1_2 {
  C_SIZE 1
  C_OPERATION and
} {
  Op1 Op3
  Op2 Op4
}
cell xilinx.com:ip:util_vector_logic and_1_3 {
  C_SIZE 1
  C_OPERATION and
} {
  Op1 Op5
  Op2 Op6
}
cell xilinx.com:ip:util_vector_logic and_1_4 {
  C_SIZE 1
  C_OPERATION and
} {
  Op1 Op7
  Op2 Op8
}

# Second stage
cell xilinx.com:ip:util_vector_logic and_2_1 {
  C_SIZE 1
  C_OPERATION and
} {
  Op1 and_1_1/Res
  Op2 and_1_2/Res
}
cell xilinx.com:ip:util_vector_logic and_2_2 {
  C_SIZE 1
  C_OPERATION and
} {
  Op1 and_1_3/Res
  Op2 and_1_4/Res
}

# Final stage
cell xilinx.com:ip:util_vector_logic and_3_1 {
  C_SIZE 1
  C_OPERATION and
} {
  Op1 and_2_1/Res
  Op2 and_2_2/Res
  Res Res
}
