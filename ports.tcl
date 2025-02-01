# Single-ended ports
create_bd_port -dir I -from 2 -to 0 Shutdown_Sense_Sel
create_bd_port -dir I Shutdown_Sense
create_bd_port -dir I Trigger_In
create_bd_port -dir I 10Mhz_In
create_bd_port -dir I Shutdown_Force
create_bd_port -dir I Eth_Clk
create_bd_port -dir I ~Shutdown_Reset

# Differential ports
create_bd_port -dir O -from 7 -to 0 ~DAC_CS_p
create_bd_port -dir O -from 7 -to 0 ~DAC_CS_n
create_bd_port -dir O -from 7 -to 0 DAC_MOSI_p
create_bd_port -dir O -from 7 -to 0 DAC_MOSI_n
create_bd_port -dir I -from 7 -to 0 DAC_MISO_p
create_bd_port -dir I -from 7 -to 0 DAC_MISO_n
create_bd_port -dir O -from 7 -to 0 ~ADC_CS_p
create_bd_port -dir O -from 7 -to 0 ~ADC_CS_n
create_bd_port -dir O -from 7 -to 0 ADC_MOSI_p
create_bd_port -dir O -from 7 -to 0 ADC_MOSI_n
create_bd_port -dir I -from 7 -to 0 ADC_MISO_p
create_bd_port -dir I -from 7 -to 0 ADC_MISO_n
create_bd_port -dir O -from 7 -to 0 SCKO_p
create_bd_port -dir O -from 7 -to 0 SCKO_n
create_bd_port -dir O -from 0 -to 0 ~SCKI_p
create_bd_port -dir O -from 0 -to 0 ~SCKI_n
create_bd_port -dir O -from 0 -to 0 LDAC_p
create_bd_port -dir O -from 0 -to 0 LDAC_n
