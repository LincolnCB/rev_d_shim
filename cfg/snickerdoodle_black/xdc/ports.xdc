###############################################################################
#
#   Single-ended ports
#
###############################################################################


#------------------------------------------------------------
# Inputs
#------------------------------------------------------------

# 10MHz clock input (10Mhz_In)
# Pin: 10MHz_In JB2.38 N18
set_property IOSTANDARD   LVCMOS25  [get_ports Scanner_10Mhz_In]
set_property PACKAGE_PIN  N18       [get_ports Scanner_10Mhz_In]
set_property PULLTYPE     PULLDOWN  [get_ports Scanner_10Mhz_In]

# Trigger in (Trigger_In)
# Pin: Trigger_In JB2.36 P19
set_property IOSTANDARD   LVCMOS25  [get_ports Trigger_In]
set_property PACKAGE_PIN  P19       [get_ports Trigger_In]
set_property PULLTYPE     PULLDOWN  [get_ports Trigger_In]

# Shutdown Sense (Shutdown_Sense)
# Pin: Shutdown_Sense JA2.4 J15
set_property IOSTANDARD   LVCMOS25  [get_ports Shutdown_Sense]
set_property PACKAGE_PIN  J15       [get_ports Shutdown_Sense]
set_property PULLTYPE     PULLDOWN  [get_ports Shutdown_Sense]

# External Shutdown Button (Shutdown_Button)
# Pin: Shutdown_Button JA1.25 F20
set_property IOSTANDARD   LVCMOS25  [get_ports Shutdown_Button]
set_property PACKAGE_PIN  F20       [get_ports Shutdown_Button]
set_property PULLTYPE     PULLDOWN  [get_ports Shutdown_Button]


#------------------------------------------------------------
# Outputs
#------------------------------------------------------------

# Shutdown Sense Select (Shutdown_Sense_Sel[0-2])
# Pins: 
#   Shutdown_Sense_Sel[0] JB2.4 R19
#   Shutdown_Sense_Sel[1] JB1.4 T19
#   Shutdown_Sense_Sel[2] JC1.4 V5
set_property IOSTANDARD   LVCMOS25  [get_ports {Shutdown_Sense_Sel[*]}]
set_property PACKAGE_PIN  R19       [get_ports {Shutdown_Sense_Sel[0]}]
set_property PACKAGE_PIN  T19       [get_ports {Shutdown_Sense_Sel[1]}]
set_property PACKAGE_PIN  V5        [get_ports {Shutdown_Sense_Sel[2]}]

# Shutdown Force (Shutdown_Force)
# Pin: Shutdown_Force JA1.23 F19
set_property IOSTANDARD   LVCMOS25  [get_ports Shutdown_Force]
set_property PACKAGE_PIN  F19       [get_ports Shutdown_Force]

# Shutdown Reset (~Shutdown_Reset)
# Pin: n_Shutdown_Reset JA1.4 G14
set_property IOSTANDARD   LVCMOS25  [get_ports n_Shutdown_Reset]
set_property PACKAGE_PIN  G14       [get_ports n_Shutdown_Reset]



###############################################################################
#
#   Differential ports
#
###############################################################################


#------------------------------------------------------------
# DAC
#------------------------------------------------------------

# LDAC (LDAC+/-)
# Pins:
#   LDAC_p JB2.26 V16 (+) / JB2.24 W16 (-)
set_property IOSTANDARD   LVDS_25  [get_ports {LDAC_p}]
set_property PACKAGE_PIN  V16      [get_ports {LDAC_p}]
set_property IOSTANDARD   LVDS_25  [get_ports {LDAC_n}]
set_property PACKAGE_PIN  W16      [get_ports {LDAC_n}]

# DAC Chip Select (~DAC_CS[+/-][0-7])
# Pins:
#   n_DAC_CS[0] JB2.5  N17 (+) / JB2.7  P18 (-)
#   n_DAC_CS[1] JB1.20 T14 (+) / JB1.18 T15 (-)
#   n_DAC_CS[2] JC1.20 V6  (+) / JC1.18 W6  (-)
#   n_DAC_CS[3] JB1.32 V15 (+) / JB1.30 W15 (-)
#   n_DAC_CS[4] JA2.11 M14 (+) / JA2.13 M15 (-)
#   n_DAC_CS[5] JA1.20 B19 (+) / JA1.18 A20 (-)
#   n_DAC_CS[6] JA2.23 L19 (+) / JA2.25 L20 (-)
#   n_DAC_CS[7] JA1.29 J20 (+) / JA1.31 H20 (-)
set_property IOSTANDARD   LVDS_25  [get_ports {n_DAC_CS_p[*]}]
set_property IOSTANDARD   LVDS_25  [get_ports {n_DAC_CS_n[*]}]
set_property PACKAGE_PIN  N17      [get_ports {n_DAC_CS_p[0]}]
set_property PACKAGE_PIN  P18      [get_ports {n_DAC_CS_n[0]}]
set_property PACKAGE_PIN  T14      [get_ports {n_DAC_CS_p[1]}]
set_property PACKAGE_PIN  T15      [get_ports {n_DAC_CS_n[1]}]
set_property PACKAGE_PIN  V6       [get_ports {n_DAC_CS_p[2]}]
set_property PACKAGE_PIN  W6       [get_ports {n_DAC_CS_n[2]}]
set_property PACKAGE_PIN  V15      [get_ports {n_DAC_CS_p[3]}]
set_property PACKAGE_PIN  W15      [get_ports {n_DAC_CS_n[3]}]
set_property PACKAGE_PIN  M14      [get_ports {n_DAC_CS_p[4]}]
set_property PACKAGE_PIN  M15      [get_ports {n_DAC_CS_n[4]}]
set_property PACKAGE_PIN  B19      [get_ports {n_DAC_CS_p[5]}]
set_property PACKAGE_PIN  A20      [get_ports {n_DAC_CS_n[5]}]
set_property PACKAGE_PIN  L19      [get_ports {n_DAC_CS_p[6]}]
set_property PACKAGE_PIN  L20      [get_ports {n_DAC_CS_n[6]}]
set_property PACKAGE_PIN  J20      [get_ports {n_DAC_CS_p[7]}]
set_property PACKAGE_PIN  H20      [get_ports {n_DAC_CS_n[7]}]

# DAC MOSI (DAC_MOSI[+/-][0-7])
# Pins:
#   DAC_MOSI[0] JB2.20 W18 (+) / JB2.18 W19 (-)
#   DAC_MOSI[1] JB1.5  T11 (+) / JB1.7  T10 (-)
#   DAC_MOSI[2] JC1.11 T5  (+) / JC1.13 U5  (-)
#   DAC_MOSI[3] JB1.23 T16 (+) / JB1.25 U17 (-)
#   DAC_MOSI[4] JA2.8  K16 (+) / JA2.6  J16 (-)
#   DAC_MOSI[5] JA1.5  E18 (+) / JA1.7  E19 (-)
#   DAC_MOSI[6] JA2.26 K19 (+) / JA2.24 J19 (-)
#   DAC_MOSI[7] JA1.26 G19 (+) / JA1.24 G20 (-)
set_property IOSTANDARD   LVDS_25  [get_ports {DAC_MOSI_p[*]}]
set_property IOSTANDARD   LVDS_25  [get_ports {DAC_MOSI_n[*]}]
set_property PACKAGE_PIN  W18      [get_ports {DAC_MOSI_p[0]}]
set_property PACKAGE_PIN  W19      [get_ports {DAC_MOSI_n[0]}]
set_property PACKAGE_PIN  T11      [get_ports {DAC_MOSI_p[1]}]
set_property PACKAGE_PIN  T10      [get_ports {DAC_MOSI_n[1]}]
set_property PACKAGE_PIN  T5       [get_ports {DAC_MOSI_p[2]}]
set_property PACKAGE_PIN  U5       [get_ports {DAC_MOSI_n[2]}]
set_property PACKAGE_PIN  T16      [get_ports {DAC_MOSI_p[3]}]
set_property PACKAGE_PIN  U17      [get_ports {DAC_MOSI_n[3]}]
set_property PACKAGE_PIN  K16      [get_ports {DAC_MOSI_p[4]}]
set_property PACKAGE_PIN  J16      [get_ports {DAC_MOSI_n[4]}]
set_property PACKAGE_PIN  E18      [get_ports {DAC_MOSI_p[5]}]
set_property PACKAGE_PIN  E19      [get_ports {DAC_MOSI_n[5]}]
set_property PACKAGE_PIN  K19      [get_ports {DAC_MOSI_p[6]}]
set_property PACKAGE_PIN  J19      [get_ports {DAC_MOSI_n[6]}]
set_property PACKAGE_PIN  G19      [get_ports {DAC_MOSI_p[7]}]
set_property PACKAGE_PIN  G20      [get_ports {DAC_MOSI_n[7]}]

# DAC MISO (DAC_MISO[+/-][0-7])
# Pins:
#   DAC_MISO[0] JB2.14 R16 (+) / JB2.12 R17 (-)
#   DAC_MISO[1] JB1.11 P14 (+) / JB1.13 R14 (-)
#   DAC_MISO[2] JC1.17 V11 (+) / JC1.19 V10 (-)
#   DAC_MISO[3] JB1.29 W14 (+) / JB1.31 Y14 (-)
#   DAC_MISO[4] JA2.20 K14 (+) / JA2.18 J14 (-)
#   DAC_MISO[5] JA1.11 F16 (+) / JA1.13 F17 (-)
#   DAC_MISO[6] JA2.29 M17 (+) / JA2.31 M18 (-)
#   DAC_MISO[7] JA1.32 G17 (+) / JA1.30 G18 (-)
set_property IOSTANDARD   LVDS_25  [get_ports {DAC_MISO_p[*]}]
set_property DIFF_TERM    TRUE     [get_ports {DAC_MISO_p[*]}]
set_property IOSTANDARD   LVDS_25  [get_ports {DAC_MISO_n[*]}]
set_property DIFF_TERM    TRUE     [get_ports {DAC_MISO_n[*]}]
set_property PACKAGE_PIN  R16      [get_ports {DAC_MISO_p[0]}]
set_property PACKAGE_PIN  R17      [get_ports {DAC_MISO_n[0]}]
set_property PACKAGE_PIN  P14      [get_ports {DAC_MISO_p[1]}]
set_property PACKAGE_PIN  R14      [get_ports {DAC_MISO_n[1]}]
set_property PACKAGE_PIN  V11      [get_ports {DAC_MISO_p[2]}]
set_property PACKAGE_PIN  V10      [get_ports {DAC_MISO_n[2]}]
set_property PACKAGE_PIN  W14      [get_ports {DAC_MISO_p[3]}]
set_property PACKAGE_PIN  Y14      [get_ports {DAC_MISO_n[3]}]
set_property PACKAGE_PIN  K14      [get_ports {DAC_MISO_p[4]}]
set_property PACKAGE_PIN  J14      [get_ports {DAC_MISO_n[4]}]
set_property PACKAGE_PIN  F16      [get_ports {DAC_MISO_p[5]}]
set_property PACKAGE_PIN  F17      [get_ports {DAC_MISO_n[5]}]
set_property PACKAGE_PIN  M17      [get_ports {DAC_MISO_p[6]}]
set_property PACKAGE_PIN  M18      [get_ports {DAC_MISO_n[6]}]
set_property PACKAGE_PIN  G17      [get_ports {DAC_MISO_p[7]}]
set_property PACKAGE_PIN  G18      [get_ports {DAC_MISO_n[7]}]


#------------------------------------------------------------
# ADC
#------------------------------------------------------------

# ADC Chip Select (~ADC_CS[+/-][0-7])
# Pins:
#   n_ADC_CS[0] JB2.17 V17 (+) / JB2.19 V18 (-)
#   n_ADC_CS[1] JB1.8  T12 (+) / JB1.6  U12 (-)
#   n_ADC_CS[2] JC1.29 U9  (+) / JC1.31 U8  (-)
#   n_ADC_CS[3] JB2.32 Y18 (+) / JB2.30 Y19 (-)
#   n_ADC_CS[4] JC1.26 W11 (+) / JC1.24 Y11 (-)
#   n_ADC_CS[5] JA1.8  D19 (+) / JA1.6  D20 (-)
#   n_ADC_CS[6] JA2.5  L14 (+) / JA2.7  L15 (-)
#   n_ADC_CS[7] JA2.17 N15 (+) / JA2.19 N16 (-)
set_property IOSTANDARD   LVDS_25  [get_ports {n_ADC_CS_p[*]}]
set_property IOSTANDARD   LVDS_25  [get_ports {n_ADC_CS_n[*]}]
set_property PACKAGE_PIN  V17      [get_ports {n_ADC_CS_p[0]}]
set_property PACKAGE_PIN  V18      [get_ports {n_ADC_CS_n[0]}]
set_property PACKAGE_PIN  T12      [get_ports {n_ADC_CS_p[1]}]
set_property PACKAGE_PIN  U12      [get_ports {n_ADC_CS_n[1]}]
set_property PACKAGE_PIN  U9       [get_ports {n_ADC_CS_p[2]}]
set_property PACKAGE_PIN  U8       [get_ports {n_ADC_CS_n[2]}]
set_property PACKAGE_PIN  Y18      [get_ports {n_ADC_CS_p[3]}]
set_property PACKAGE_PIN  Y19      [get_ports {n_ADC_CS_n[3]}]
set_property PACKAGE_PIN  W11      [get_ports {n_ADC_CS_p[4]}]
set_property PACKAGE_PIN  Y11      [get_ports {n_ADC_CS_n[4]}]
set_property PACKAGE_PIN  D19      [get_ports {n_ADC_CS_p[5]}]
set_property PACKAGE_PIN  D20      [get_ports {n_ADC_CS_n[5]}]
set_property PACKAGE_PIN  L14      [get_ports {n_ADC_CS_p[6]}]
set_property PACKAGE_PIN  L15      [get_ports {n_ADC_CS_n[6]}]
set_property PACKAGE_PIN  N15      [get_ports {n_ADC_CS_p[7]}]
set_property PACKAGE_PIN  N16      [get_ports {n_ADC_CS_n[7]}]

# ADC MOSI (ADC_MOSI[+/-][0-7])
# Pins:
#   ADC_MOSI[0] JB2.8  P15 (+) / JB2.6  P16 (-)
#   ADC_MOSI[1] JB1.17 U13 (+) / JB1.19 V13 (-)
#   ADC_MOSI[2] JC1.23 V8  (+) / JC1.25 W8  (-)
#   ADC_MOSI[3] JB2.29 V20 (+) / JB2.31 W20 (-)
#   ADC_MOSI[4] JC1.5  U7  (+) / JC1.7  V7  (-)
#   ADC_MOSI[5] JA1.17 E17 (+) / JA1.19 D18 (-)
#   ADC_MOSI[6] JA2.14 H15 (+) / JA2.12 G15 (-)
#   ADC_MOSI[7] JA2.32 M19 (+) / JA2.30 M20 (-)
set_property IOSTANDARD   LVDS_25  [get_ports {ADC_MOSI_p[*]}]
set_property IOSTANDARD   LVDS_25  [get_ports {ADC_MOSI_n[*]}]
set_property PACKAGE_PIN  P15      [get_ports {ADC_MOSI_p[0]}]
set_property PACKAGE_PIN  P16      [get_ports {ADC_MOSI_n[0]}]
set_property PACKAGE_PIN  U13      [get_ports {ADC_MOSI_p[1]}]
set_property PACKAGE_PIN  V13      [get_ports {ADC_MOSI_n[1]}]
set_property PACKAGE_PIN  V8       [get_ports {ADC_MOSI_p[2]}]
set_property PACKAGE_PIN  W8       [get_ports {ADC_MOSI_n[2]}]
set_property PACKAGE_PIN  V20      [get_ports {ADC_MOSI_p[3]}]
set_property PACKAGE_PIN  W20      [get_ports {ADC_MOSI_n[3]}]
set_property PACKAGE_PIN  U7       [get_ports {ADC_MOSI_p[4]}]
set_property PACKAGE_PIN  V7       [get_ports {ADC_MOSI_n[4]}]
set_property PACKAGE_PIN  E17      [get_ports {ADC_MOSI_p[5]}]
set_property PACKAGE_PIN  D18      [get_ports {ADC_MOSI_n[5]}]
set_property PACKAGE_PIN  H15      [get_ports {ADC_MOSI_p[6]}]
set_property PACKAGE_PIN  G15      [get_ports {ADC_MOSI_n[6]}]
set_property PACKAGE_PIN  M19      [get_ports {ADC_MOSI_p[7]}]
set_property PACKAGE_PIN  M20      [get_ports {ADC_MOSI_n[7]}]

# ADC MISO (ADC_MISO[+/-][0-7])
# Pins:
#   ADC_MISO[0] JB2.11 T17 (+) / JB2.13 R18 (-)
#   ADC_MISO[1] JB1.14 V12 (+) / JB1.12 W13 (-)
#   ADC_MISO[2] JC1.14 Y12 (+) / JC1.12 Y13 (-)
#   ADC_MISO[3] JB1.26 Y16 (+) / JB1.24 Y17 (-)
#   ADC_MISO[4] JC1.32 Y7  (+) / JC1.30 Y6  (-)
#   ADC_MISO[5] JA1.14 C20 (+) / JA1.12 B20 (-)
#   ADC_MISO[6] JA2.29 M17 (+) / JA2.31 M18 (-)
#   ADC_MISO[7] JA1.35 J18 (+) / JA1.37 H18 (-)
set_property IOSTANDARD   LVDS_25  [get_ports {ADC_MISO_p[*]}]
set_property DIFF_TERM    TRUE     [get_ports {ADC_MISO_p[*]}]
set_property IOSTANDARD   LVDS_25  [get_ports {ADC_MISO_n[*]}]
set_property DIFF_TERM    TRUE     [get_ports {ADC_MISO_n[*]}]
set_property PACKAGE_PIN  T17      [get_ports {ADC_MISO_p[0]}]
set_property PACKAGE_PIN  R18      [get_ports {ADC_MISO_n[0]}]
set_property PACKAGE_PIN  V12      [get_ports {ADC_MISO_p[1]}]
set_property PACKAGE_PIN  W13      [get_ports {ADC_MISO_n[1]}]
set_property PACKAGE_PIN  Y12      [get_ports {ADC_MISO_p[2]}]
set_property PACKAGE_PIN  Y13      [get_ports {ADC_MISO_n[2]}]
set_property PACKAGE_PIN  Y16      [get_ports {ADC_MISO_p[3]}]
set_property PACKAGE_PIN  Y17      [get_ports {ADC_MISO_n[3]}]
set_property PACKAGE_PIN  Y7       [get_ports {ADC_MISO_p[4]}]
set_property PACKAGE_PIN  Y6       [get_ports {ADC_MISO_n[4]}]
set_property PACKAGE_PIN  C20      [get_ports {ADC_MISO_p[5]}]
set_property PACKAGE_PIN  B20      [get_ports {ADC_MISO_n[5]}]
set_property PACKAGE_PIN  M17      [get_ports {ADC_MISO_p[6]}]
set_property PACKAGE_PIN  M18      [get_ports {ADC_MISO_n[6]}]
set_property PACKAGE_PIN  J18      [get_ports {ADC_MISO_p[7]}]
set_property PACKAGE_PIN  H18      [get_ports {ADC_MISO_n[7]}]


#------------------------------------------------------------
# Clocks
#------------------------------------------------------------

# MISO SCK (SCKO[+/-][0-7])
# Pins:
#   MISO_SCK[0] JB2.35 N20 (+) / JB2.37 P20 (-)
#   MISO_SCK[1] JB1.38 U18 (+) / JB1.36 U19 (-)
#   MISO_SCK[2] JC1.8  T9  (+) / JC1.6  U10 (-)
#   MISO_SCK[3] JB1.35 U14 (+) / JB1.37 U15 (-)
#   MISO_SCK[4] JC1.35 Y9  (+) / JC1.37 Y8  (-)
#   MISO_SCK[5] JA1.36 H17 (+) / JA1.38 H16 (-)
#   MISO_SCK[6] JA2.38 K17 (+) / JA2.36 K18 (-)
#   MISO_SCK[7] JA2.35 L16 (+) / JA2.37 L17 (-)
set_property IOSTANDARD   LVDS_25  [get_ports {MISO_SCK_p[*]}]
set_property DIFF_TERM    TRUE     [get_ports {MISO_SCK_p[*]}]
set_property IOSTANDARD   LVDS_25  [get_ports {MISO_SCK_n[*]}]
set_property DIFF_TERM    TRUE     [get_ports {MISO_SCK_n[*]}]
set_property PACKAGE_PIN  N20      [get_ports {MISO_SCK_p[0]}]
set_property PACKAGE_PIN  P20      [get_ports {MISO_SCK_n[0]}]
set_property PACKAGE_PIN  U18      [get_ports {MISO_SCK_p[1]}]
set_property PACKAGE_PIN  U19      [get_ports {MISO_SCK_n[1]}]
set_property PACKAGE_PIN  T9       [get_ports {MISO_SCK_p[2]}]
set_property PACKAGE_PIN  U10      [get_ports {MISO_SCK_n[2]}]
set_property PACKAGE_PIN  U14      [get_ports {MISO_SCK_p[3]}]
set_property PACKAGE_PIN  U15      [get_ports {MISO_SCK_n[3]}]
set_property PACKAGE_PIN  Y9       [get_ports {MISO_SCK_p[4]}]
set_property PACKAGE_PIN  Y8       [get_ports {MISO_SCK_n[4]}]
set_property PACKAGE_PIN  H17      [get_ports {MISO_SCK_p[5]}]
set_property PACKAGE_PIN  H16      [get_ports {MISO_SCK_n[5]}]
set_property PACKAGE_PIN  K17      [get_ports {MISO_SCK_p[6]}]
set_property PACKAGE_PIN  K18      [get_ports {MISO_SCK_n[6]}]
set_property PACKAGE_PIN  L16      [get_ports {MISO_SCK_p[7]}]
set_property PACKAGE_PIN  L17      [get_ports {MISO_SCK_n[7]}]

# MOSI SCK (~SCKI[+/-])
# Pins:
#   n_MOSI_SCK JB2.23 T20 (+) / JB2.25 U20 (-)
set_property IOSTANDARD   LVDS_25  [get_ports {n_MOSI_SCK_p}]
set_property IOSTANDARD   LVDS_25  [get_ports {n_MOSI_SCK_n}]
set_property PACKAGE_PIN  T20      [get_ports {n_MOSI_SCK_p}]
set_property PACKAGE_PIN  U20      [get_ports {n_MOSI_SCK_n}]
