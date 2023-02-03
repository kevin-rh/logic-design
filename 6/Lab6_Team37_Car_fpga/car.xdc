# Clock signal
set_property PACKAGE_PIN W5 [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports clk]
create_clock -period 10.000 -name sys_clk_pin -waveform {0.000 5.000} -add [get_ports clk]

# LEDs
set_property PACKAGE_PIN U16 [get_ports {led[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]
set_property PACKAGE_PIN E19 [get_ports {led[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]
set_property PACKAGE_PIN U19 [get_ports {led[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[2]}]
set_property PACKAGE_PIN V19 [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[3]}]
set_property PACKAGE_PIN W18 [get_ports {led[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[4]}]
set_property PACKAGE_PIN U15 [get_ports {led[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[5]}]
set_property PACKAGE_PIN U14 [get_ports {led[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[6]}]
set_property PACKAGE_PIN V14 [get_ports {led[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[7]}]
set_property PACKAGE_PIN V13 [get_ports {led[8]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[8]}]
set_property PACKAGE_PIN V3 [get_ports {led[9]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[9]}]
set_property PACKAGE_PIN W3 [get_ports {led[10]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[10]}]
set_property PACKAGE_PIN U3 [get_ports {led[11]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[11]}]
set_property PACKAGE_PIN P3 [get_ports {led[12]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[12]}]
set_property PACKAGE_PIN N3 [get_ports {led[13]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[13]}]
set_property PACKAGE_PIN P1 [get_ports {led[14]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[14]}]
set_property PACKAGE_PIN L1 [get_ports {led[15]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[15]}]



###7 segment display
#set_property PACKAGE_PIN W7 [get_ports {seg[0]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {seg[0]}]
#set_property PACKAGE_PIN W6 [get_ports {seg[1]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {seg[1]}]
#set_property PACKAGE_PIN U8 [get_ports {seg[2]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {seg[2]}]
#set_property PACKAGE_PIN V8 [get_ports {seg[3]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {seg[3]}]
#set_property PACKAGE_PIN U5 [get_ports {seg[4]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {seg[4]}]
#set_property PACKAGE_PIN V5 [get_ports {seg[5]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {seg[5]}]
#set_property PACKAGE_PIN U7 [get_ports {seg[6]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {seg[6]}]

#set_property PACKAGE_PIN U2 [get_ports {an[0]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {an[0]}]
#set_property PACKAGE_PIN U4 [get_ports {an[1]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {an[1]}]
#set_property PACKAGE_PIN V4 [get_ports {an[2]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {an[2]}]
#set_property PACKAGE_PIN W4 [get_ports {an[3]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {an[3]}]


##Buttons
set_property PACKAGE_PIN U18 [get_ports rst]
set_property IOSTANDARD LVCMOS33 [get_ports rst]


##Pmod Header JA
##Sch name = JA1
set_property PACKAGE_PIN J1 [get_ports {left_motor}]
set_property IOSTANDARD LVCMOS33 [get_ports {left_motor}]
#Sch name = JA2
set_property PACKAGE_PIN L2 [get_ports {left[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {left[0]}]
##Sch name = JA3
set_property PACKAGE_PIN J2 [get_ports {left[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {left[1]}]
##Sch name = JA4
set_property PACKAGE_PIN G2 [get_ports {trig}]
set_property IOSTANDARD LVCMOS33 [get_ports {trig}]
##Sch name = JA7
set_property PACKAGE_PIN H1 [get_ports {right_motor}]
set_property IOSTANDARD LVCMOS33 [get_ports {right_motor}]
##Sch name = JA8
set_property PACKAGE_PIN K2 [get_ports {right[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {right[0]}]
##Sch name = JA9
set_property PACKAGE_PIN H2 [get_ports {right[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {right[1]}]
##Sch name = JA10
set_property PACKAGE_PIN G3 [get_ports {echo}]
set_property IOSTANDARD LVCMOS33 [get_ports {echo}]


##Pmod Header JB
##Sch name = JB1
set_property PACKAGE_PIN A14 [get_ports {left_signal}]
set_property IOSTANDARD LVCMOS33 [get_ports {left_signal}]
##Sch name = JB2
set_property PACKAGE_PIN A16 [get_ports {mid_signal}]
set_property IOSTANDARD LVCMOS33 [get_ports {mid_signal}]
##Sch name = JB3
set_property PACKAGE_PIN B15 [get_ports {right_signal}]
set_property IOSTANDARD LVCMOS33 [get_ports {right_signal}]



set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]

set_property BITSTREAM.CONFIG.CONFIGRATE 33 [current_design]

set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]
