create_bd_pin -dir I -type clock sck
create_bd_pin -dir I -type reset rst

cell lcb:user:threshold_integrator:1.0 threshold_core {} {
  clk sck
  rst rst
}
