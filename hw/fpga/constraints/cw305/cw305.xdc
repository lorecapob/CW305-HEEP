
######## HARDWARE ON BOARD

#LEDs
set_property DRIVE 8 [get_ports led1]
set_property PACKAGE_PIN T2 [get_ports led1]

set_property DRIVE 8 [get_ports led2]
set_property PACKAGE_PIN T3 [get_ports led2]

set_property DRIVE 8 [get_ports led3]
set_property PACKAGE_PIN T4 [get_ports led3]

#Switch's
set_property PACKAGE_PIN J16 [get_ports j16_sel]
set_property PACKAGE_PIN K16 [get_ports k16_sel]
set_property PACKAGE_PIN L14 [get_ports l14_sel]
set_property PACKAGE_PIN K15 [get_ports k15_sel]
set_property PACKAGE_PIN R1 [get_ports pushbutton]

#PLL Connections
set_property PACKAGE_PIN N13 [get_ports pll_clk1]
#set_property PACKAGE_PIN E12 [get_ports pll_clk2]

######## 20-Pin Connector

set_property PACKAGE_PIN T14 [get_ports tio_trigger]

set_property PACKAGE_PIN T15 [get_ports IO_0]

#set_property PACKAGE_PIN M16 [get_ports tio_clkout] # UNUSED
set_property PACKAGE_PIN M16 [get_ports IO_1]

# set_property PACKAGE_PIN N14 [get_ports tio_clkin] # UNUSED

# Debug UART

set_property -dict {PACKAGE_PIN P16 IOSTANDARD LVCMOS33} [get_ports debug_heep_uart_tx]
set_property -dict {PACKAGE_PIN R16 IOSTANDARD LVCMOS33} [get_ports debug_heep_uart_rx]

####### USB Connector

set_property PACKAGE_PIN F5 [get_ports usb_clk]

set_property IOSTANDARD LVCMOS33 [get_ports *]
set_property PACKAGE_PIN A7 [get_ports {usb_data[0]}]
set_property PACKAGE_PIN B6 [get_ports {usb_data[1]}]
set_property PACKAGE_PIN D3 [get_ports {usb_data[2]}]
set_property PACKAGE_PIN E3 [get_ports {usb_data[3]}]
set_property PACKAGE_PIN F3 [get_ports {usb_data[4]}]
set_property PACKAGE_PIN B5 [get_ports {usb_data[5]}]
set_property PACKAGE_PIN K1 [get_ports {usb_data[6]}]
set_property PACKAGE_PIN K2 [get_ports {usb_data[7]}]

set_property PACKAGE_PIN F4 [get_ports {usb_addr[0]}]
set_property PACKAGE_PIN G5 [get_ports {usb_addr[1]}]
set_property PACKAGE_PIN J1 [get_ports {usb_addr[2]}]
set_property PACKAGE_PIN H1 [get_ports {usb_addr[3]}]
set_property PACKAGE_PIN H2 [get_ports {usb_addr[4]}]
set_property PACKAGE_PIN G1 [get_ports {usb_addr[5]}]
set_property PACKAGE_PIN G2 [get_ports {usb_addr[6]}]
set_property PACKAGE_PIN F2 [get_ports {usb_addr[7]}]
set_property PACKAGE_PIN E1 [get_ports {usb_addr[8]}]
set_property PACKAGE_PIN E2 [get_ports {usb_addr[9]}]
set_property PACKAGE_PIN D1 [get_ports {usb_addr[10]}]
set_property PACKAGE_PIN C1 [get_ports {usb_addr[11]}]
set_property PACKAGE_PIN K3 [get_ports {usb_addr[12]}]
set_property PACKAGE_PIN L2 [get_ports {usb_addr[13]}]
set_property PACKAGE_PIN J3 [get_ports {usb_addr[14]}]
set_property PACKAGE_PIN B2 [get_ports {usb_addr[15]}]
set_property PACKAGE_PIN C7 [get_ports {usb_addr[16]}]
set_property PACKAGE_PIN C6 [get_ports {usb_addr[17]}]
set_property PACKAGE_PIN D6 [get_ports {usb_addr[18]}]
set_property PACKAGE_PIN C4 [get_ports {usb_addr[19]}]
set_property PACKAGE_PIN D5 [get_ports {usb_addr[20]}]

set_property PACKAGE_PIN A4 [get_ports usb_rdn]
set_property PACKAGE_PIN C2 [get_ports usb_wrn]
set_property PACKAGE_PIN A3 [get_ports usb_cen]

set_property PACKAGE_PIN A5 [get_ports usb_trigger]


create_clock -period 10.000 -name usb_clk -waveform {0.000 5.000} [get_nets usb_clk]
# create_clock -period 50.000 -name tio_clkin -waveform {0.000 5.000} [get_nets tio_clkin] # UNUSED
create_clock -period 50.000 -name pll_clk1 -waveform {0.000 5.000} [get_nets pll_clk1]

# set_clock_groups -asynchronous -group [get_clocks usb_clk] -group [get_clocks {tio_clkin pll_clk1}] # UNUSED tio_clkin
set_clock_groups -asynchronous -group [get_clocks usb_clk] -group [get_clocks {pll_clk1}]

# both input clocks have same properties so there is no point in doing timing analysis for both:
set_case_analysis 1 [get_pins U_clocks/CCLK_MUX/S]

# No spec for these, seems sensible:
set_input_delay -clock usb_clk -add_delay 2.000 [get_ports usb_addr]
set_input_delay -clock usb_clk -add_delay 2.000 [get_ports usb_data]
set_input_delay -clock usb_clk -add_delay 2.000 [get_ports usb_trigger]
set_input_delay -clock usb_clk -add_delay 2.000 [get_ports usb_cen]
set_input_delay -clock usb_clk -add_delay 2.000 [get_ports usb_rdn]
set_input_delay -clock usb_clk -add_delay 2.000 [get_ports usb_wrn]

set_input_delay -clock usb_clk -add_delay 0.000 [get_ports j16_sel]
set_input_delay -clock usb_clk -add_delay 0.000 [get_ports k16_sel]
set_input_delay -clock [get_clocks usb_clk] -add_delay 0.500 [get_ports pushbutton]
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets pushbutton_IBUF]

set_output_delay -clock usb_clk 0.000 [get_ports led1]
set_output_delay -clock usb_clk 0.000 [get_ports led2]
set_output_delay -clock usb_clk 0.000 [get_ports led3]
set_output_delay -clock usb_clk 0.000 [get_ports usb_data]
set_output_delay -clock usb_clk 0.000 [get_ports tio_trigger]
set_output_delay -clock usb_clk 0.000 [get_ports IO_0]
set_output_delay -clock usb_clk 0.000 [get_ports IO_1]
set_false_path -to [get_ports led1]
set_false_path -to [get_ports led2]
set_false_path -to [get_ports led3]
set_false_path -to [get_ports usb_data]
set_false_path -to [get_ports tio_trigger]
set_false_path -to [get_ports IO_0]
set_false_path -to [get_ports IO_1]

set_input_delay -clock [get_clocks usb_clk] -add_delay 0.500 [get_ports debug_heep_uart_rx]
set_output_delay -clock [get_clocks usb_clk] -add_delay 0.500 [get_ports debug_heep_uart_tx]

set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

set_property BITSTREAM.CONFIG.USR_ACCESS TIMESTAMP [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]

set_property MARK_DEBUG true [get_nets heep_clk]
set_property MARK_DEBUG true [get_nets gnt_o]
set_property MARK_DEBUG true [get_nets reset]
set_property MARK_DEBUG true [get_nets {wdata_i[13]}]
set_property MARK_DEBUG true [get_nets {wdata_i[5]}]
set_property MARK_DEBUG true [get_nets {wdata_i[0]}]
set_property MARK_DEBUG true [get_nets {wdata_i[1]}]
set_property MARK_DEBUG true [get_nets {wdata_i[2]}]
set_property MARK_DEBUG true [get_nets {wdata_i[3]}]
set_property MARK_DEBUG true [get_nets {wdata_i[4]}]
set_property MARK_DEBUG true [get_nets {wdata_i[6]}]
set_property MARK_DEBUG true [get_nets {wdata_i[7]}]
set_property MARK_DEBUG true [get_nets {wdata_i[14]}]
set_property MARK_DEBUG true [get_nets {wdata_i[15]}]
set_property MARK_DEBUG true [get_nets {wdata_i[12]}]
set_property MARK_DEBUG true [get_nets {wdata_i[18]}]
set_property MARK_DEBUG true [get_nets {wdata_i[19]}]
set_property MARK_DEBUG true [get_nets {wdata_i[16]}]
set_property MARK_DEBUG true [get_nets {wdata_i[22]}]
set_property MARK_DEBUG true [get_nets {wdata_i[23]}]
set_property MARK_DEBUG true [get_nets {wdata_i[20]}]
set_property MARK_DEBUG true [get_nets {wdata_i[26]}]
set_property MARK_DEBUG true [get_nets {wdata_i[27]}]
set_property MARK_DEBUG true [get_nets {wdata_i[24]}]
set_property MARK_DEBUG true [get_nets {wdata_i[30]}]
set_property MARK_DEBUG true [get_nets {wdata_i[31]}]
set_property MARK_DEBUG true [get_nets {wdata_i[28]}]
set_property MARK_DEBUG true [get_nets {wdata_i[29]}]
set_property MARK_DEBUG true [get_nets {wdata_i[25]}]
set_property MARK_DEBUG true [get_nets {wdata_i[21]}]
set_property MARK_DEBUG true [get_nets {wdata_i[17]}]
set_property MARK_DEBUG true [get_nets {wdata_i[8]}]
set_property MARK_DEBUG true [get_nets {wdata_i[9]}]
set_property MARK_DEBUG true [get_nets {wdata_i[10]}]
set_property MARK_DEBUG true [get_nets {wdata_i[11]}]
set_property MARK_DEBUG true [get_nets {addr_i[27]}]
set_property MARK_DEBUG true [get_nets {addr_i[28]}]
set_property MARK_DEBUG true [get_nets {addr_i[29]}]
set_property MARK_DEBUG true [get_nets {addr_i[30]}]
set_property MARK_DEBUG true [get_nets {addr_i[31]}]
set_property MARK_DEBUG true [get_nets {addr_i[4]}]
set_property MARK_DEBUG true [get_nets {addr_i[7]}]
set_property MARK_DEBUG true [get_nets {addr_i[1]}]
set_property MARK_DEBUG true [get_nets {addr_i[3]}]
set_property MARK_DEBUG true [get_nets {addr_i[5]}]
set_property MARK_DEBUG true [get_nets {addr_i[6]}]
set_property MARK_DEBUG true [get_nets {addr_i[8]}]
set_property MARK_DEBUG true [get_nets {addr_i[9]}]
set_property MARK_DEBUG true [get_nets {addr_i[10]}]
set_property MARK_DEBUG true [get_nets {addr_i[25]}]
set_property MARK_DEBUG true [get_nets {addr_i[26]}]
set_property MARK_DEBUG true [get_nets {addr_i[23]}]
set_property MARK_DEBUG true [get_nets {addr_i[24]}]
set_property MARK_DEBUG true [get_nets {addr_i[21]}]
set_property MARK_DEBUG true [get_nets {addr_i[22]}]
set_property MARK_DEBUG true [get_nets {addr_i[19]}]
set_property MARK_DEBUG true [get_nets {addr_i[20]}]
set_property MARK_DEBUG true [get_nets {addr_i[11]}]
set_property MARK_DEBUG true [get_nets {addr_i[12]}]
set_property MARK_DEBUG true [get_nets {addr_i[13]}]
set_property MARK_DEBUG true [get_nets {addr_i[14]}]
set_property MARK_DEBUG true [get_nets {addr_i[15]}]
set_property MARK_DEBUG true [get_nets {addr_i[16]}]
set_property MARK_DEBUG true [get_nets {addr_i[17]}]
set_property MARK_DEBUG true [get_nets {addr_i[18]}]

set_property MARK_DEBUG true [get_nets we_i]
set_property MARK_DEBUG true [get_nets req_i]
set_property MARK_DEBUG true [get_nets bridge_rst_instr_valid]
set_property MARK_DEBUG true [get_nets bridge_rst_new_address_valid]
set_property MARK_DEBUG true [get_nets {be_i[0]}]
set_property MARK_DEBUG true [get_nets {be_i[1]}]
set_property MARK_DEBUG true [get_nets {be_i[2]}]
set_property MARK_DEBUG true [get_nets {be_i[3]}]
create_debug_core u_ila_0 ila
set_property ALL_PROBE_SAME_MU true [get_debug_cores u_ila_0]
set_property ALL_PROBE_SAME_MU_CNT 1 [get_debug_cores u_ila_0]
set_property C_ADV_TRIGGER false [get_debug_cores u_ila_0]
set_property C_DATA_DEPTH 1024 [get_debug_cores u_ila_0]
set_property C_EN_STRG_QUAL false [get_debug_cores u_ila_0]
set_property C_INPUT_PIPE_STAGES 0 [get_debug_cores u_ila_0]
set_property C_TRIGIN_EN false [get_debug_cores u_ila_0]
set_property C_TRIGOUT_EN false [get_debug_cores u_ila_0]
set_property port_width 1 [get_debug_ports u_ila_0/clk]
connect_debug_port u_ila_0/clk [get_nets [list U_clocks/O_cryptoclk]]
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe0]
set_property port_width 32 [get_debug_ports u_ila_0/probe0]
connect_debug_port u_ila_0/probe0 [get_nets [list {wdata_i[0]} {wdata_i[1]} {wdata_i[2]} {wdata_i[3]} {wdata_i[4]} {wdata_i[5]} {wdata_i[6]} {wdata_i[7]} {wdata_i[8]} {wdata_i[9]} {wdata_i[10]} {wdata_i[11]} {wdata_i[12]} {wdata_i[13]} {wdata_i[14]} {wdata_i[15]} {wdata_i[16]} {wdata_i[17]} {wdata_i[18]} {wdata_i[19]} {wdata_i[20]} {wdata_i[21]} {wdata_i[22]} {wdata_i[23]} {wdata_i[24]} {wdata_i[25]} {wdata_i[26]} {wdata_i[27]} {wdata_i[28]} {wdata_i[29]} {wdata_i[30]} {wdata_i[31]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe1]
set_property port_width 32 [get_debug_ports u_ila_0/probe1]
connect_debug_port u_ila_0/probe1 [get_nets [list {addr_i[0]} {addr_i[1]} {addr_i[2]} {addr_i[3]} {addr_i[4]} {addr_i[5]} {addr_i[6]} {addr_i[7]} {addr_i[8]} {addr_i[9]} {addr_i[10]} {addr_i[11]} {addr_i[12]} {addr_i[13]} {addr_i[14]} {addr_i[15]} {addr_i[16]} {addr_i[17]} {addr_i[18]} {addr_i[19]} {addr_i[20]} {addr_i[21]} {addr_i[22]} {addr_i[23]} {addr_i[24]} {addr_i[25]} {addr_i[26]} {addr_i[27]} {addr_i[28]} {addr_i[29]} {addr_i[30]} {addr_i[31]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe2]
set_property port_width 4 [get_debug_ports u_ila_0/probe2]
connect_debug_port u_ila_0/probe2 [get_nets [list {be_i[0]} {be_i[1]} {be_i[2]} {be_i[3]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe3]
set_property port_width 1 [get_debug_ports u_ila_0/probe3]
connect_debug_port u_ila_0/probe3 [get_nets [list bridge_rst_instr_valid]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe4]
set_property port_width 1 [get_debug_ports u_ila_0/probe4]
connect_debug_port u_ila_0/probe4 [get_nets [list bridge_rst_new_address_valid]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe5]
set_property port_width 1 [get_debug_ports u_ila_0/probe5]
connect_debug_port u_ila_0/probe5 [get_nets [list gnt_o]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe6]
set_property port_width 1 [get_debug_ports u_ila_0/probe6]
connect_debug_port u_ila_0/probe6 [get_nets [list heep_clk]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe7]
set_property port_width 1 [get_debug_ports u_ila_0/probe7]
connect_debug_port u_ila_0/probe7 [get_nets [list req_i]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe8]
set_property port_width 1 [get_debug_ports u_ila_0/probe8]
connect_debug_port u_ila_0/probe8 [get_nets [list we_i]]
set_property C_CLK_INPUT_FREQ_HZ 300000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]
set_property C_USER_SCAN_CHAIN 1 [get_debug_cores dbg_hub]
connect_debug_port dbg_hub/clk [get_nets heep_clk]
