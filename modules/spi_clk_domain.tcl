create_bd_pin -dir I -type clock sck
create_bd_pin -dir I -type reset rst

## Channel 0
module dac_ch0 {
  source projects/rev_d_shim/modules/dac_channel.tcl
} {
  sck sck
  rst rst
}
module adc_ch0 {
  source projects/rev_d_shim/modules/adc_channel.tcl
} {
  sck sck
  rst rst
}

## Channel 1
module dac_ch1 {
  source projects/rev_d_shim/modules/dac_channel.tcl
} {
  sck sck
  rst rst
}
module adc_ch1 {
  source projects/rev_d_shim/modules/adc_channel.tcl
} {
  sck sck
  rst rst
}

## Channel 2
module dac_ch2 {
  source projects/rev_d_shim/modules/dac_channel.tcl
} {
  sck sck
  rst rst
}
module adc_ch2 {
  source projects/rev_d_shim/modules/adc_channel.tcl
} {
  sck sck
  rst rst
}

## Channel 3
module dac_ch3 {
  source projects/rev_d_shim/modules/dac_channel.tcl
} {
  sck sck
  rst rst
}
module adc_ch3 {
  source projects/rev_d_shim/modules/adc_channel.tcl
} {
  sck sck
  rst rst
}

## Channel 4
module dac_ch4 {
  source projects/rev_d_shim/modules/dac_channel.tcl
} {
  sck sck
  rst rst
}
module adc_ch4 {
  source projects/rev_d_shim/modules/adc_channel.tcl
} {
  sck sck
  rst rst
}

## Channel 5
module dac_ch5 {
  source projects/rev_d_shim/modules/dac_channel.tcl
} {
  sck sck
  rst rst
}
module adc_ch5 {
  source projects/rev_d_shim/modules/adc_channel.tcl
} {
  sck sck
  rst rst
}

## Channel 6
module dac_ch6 {
  source projects/rev_d_shim/modules/dac_channel.tcl
} {
  sck sck
  rst rst
}
module adc_ch6 {
  source projects/rev_d_shim/modules/adc_channel.tcl
} {
  sck sck
  rst rst
}

## Channel 7
module dac_ch7 {
  source projects/rev_d_shim/modules/dac_channel.tcl
} {
  sck sck
  rst rst
}
module adc_ch7 {
  source projects/rev_d_shim/modules/adc_channel.tcl
} {
  sck sck
  rst rst
}
