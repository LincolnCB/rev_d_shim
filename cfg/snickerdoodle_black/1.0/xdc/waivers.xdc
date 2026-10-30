############################################################
# CDC waivers
#   Crossings that are safe by design. Waived entries move to
#   the "waived" column of report_cdc and report_methodology;
#   they don't change synthesis, placement or timing.
############################################################

# SPI clock mux select: the synchronized spi_off drives BUFGMUX_CTRL S0/S1, which takes an
# asynchronous select and switches glitch-free by design
create_waiver -type CDC -id {CDC-13} -user rev_d_shim -to [get_pins -of_objects [get_cells -hierarchical -filter {NAME =~ *spi_clk/inst/clk_mux}] -filter {REF_PIN_NAME =~ S*}] -description "BUFGMUX_CTRL select is asynchronous-safe by design"

# DAC MISO: false-pathed in ports.xdc, since readback only happens on the 5 MHz boot clock
create_waiver -type CDC -id {CDC-1} -user rev_d_shim -from [get_ports {DAC_MISO_p[*]}] -description "DAC MISO readback runs only on the 5 MHz boot clock (false-pathed)"
create_waiver -type CDC -id {CDC-13} -user rev_d_shim -from [get_ports {DAC_MISO_p[*]}] -description "DAC MISO readback runs only on the 5 MHz boot clock (false-pathed)"
