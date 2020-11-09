create_clock -period 5.000 -name mcb_clk_ref [get_ports clk_ref_p]


# Bank: 38 - Byte 
set_property VCCAUX_IO DONTCARE [get_ports clk_ref_p]
set_property IOSTANDARD DIFF_SSTL15 [get_ports clk_ref_p]

# Bank: 38 - Byte 
set_property VCCAUX_IO DONTCARE [get_ports clk_ref_n]
set_property IOSTANDARD DIFF_SSTL15 [get_ports clk_ref_n]
set_property PACKAGE_PIN H19 [get_ports clk_ref_p]
set_property PACKAGE_PIN G18 [get_ports clk_ref_n]

create_clock -period 6.400 -name xgemac_clk_156 [get_ports xphy_refclk_p]

##GT Ref clk
set_property PACKAGE_PIN E10 [get_ports xphy_refclk_p]
set_property PACKAGE_PIN E9 [get_ports xphy_refclk_n]


create_generated_clock -name clk50 -source [get_ports clk_ref_p] -divide_by 4 [get_pins n10g_interface_inst/clk_divide_reg[1]/Q]
#set_clock_sense -positive n10g_interface_inst/clk_divide_reg[1]_i_1/O


#button, @yang, Netfpga-sume currently only has two button
# @yang, we only retain east button for now
set_property PACKAGE_PIN AR13 [get_ports button_east]
set_property IOSTANDARD LVCMOS15 [get_ports button_east]
# set_property PACKAGE_PIN AR13 [get_ports button_west]
# set_property IOSTANDARD LVCMOS15 [get_ports button_west]

# set_property PACKAGE_PIN BB12 [get_ports button_north]
# set_property IOSTANDARD LVCMOS15 [get_ports button_north]
# set_property PACKAGE_PIN BB12 [get_ports button_south]
# set_property IOSTANDARD LVCMOS15 [get_ports button_south]
# set_property PACKAGE_PIN BB12 [get_ports button_center]
# set_property IOSTANDARD LVCMOS15 [get_ports button_center]


#UART
#set_property PACKAGE_PIN AY19 [get_ports TxD]
#set_property IOSTANDARD LVCMOS15 [get_ports TxD]

#set_property PACKAGE_PIN BA19 [get_ports RxD]
#set_property IOSTANDARD LVCMOS15 [get_ports RxD]

## bram locations @yang, not sure if sume has these
#set_property LOC RAMB36_X11Y69 [get_cells configIp/trmx/imx/ram[0].RAMB36_inst]
#set_property LOC RAMB36_X10Y68 [get_cells configIp/trmx/dmx/ram[0].RAMB36_inst]

# Needed by 10GBASE-R IP XDC
create_clock -name clk156 -period 6.400 [get_pins n10g_interface_inst/xgbaser_gt_wrapper_inst/clk156_bufg_inst/O]
create_clock -name dclk -period 12.800 [get_pins n10g_interface_inst/xgbaser_gt_wrapper_inst/dclk_bufg_inst/O]
create_clock -name refclk -period 6.400 [get_pins n10g_interface_inst/xgphy_refclk_ibuf/O]

# SFP TX Disable for 10G PHY
set_property LOC M18  [get_ports {sfp_tx_disable[0]}]
set_property IOSTANDARD LVCMOS15 [get_ports {sfp_tx_disable[0]}]
set_property LOC B31  [get_ports {sfp_tx_disable[1]}]
set_property IOSTANDARD LVCMOS15 [get_ports {sfp_tx_disable[1]}]
set_property LOC J38  [get_ports {sfp_tx_disable[2]}]
set_property IOSTANDARD LVCMOS15 [get_ports {sfp_tx_disable[2]}]
set_property LOC L21  [get_ports {sfp_tx_disable[3]}]
set_property IOSTANDARD LVCMOS15 [get_ports {sfp_tx_disable[3]}]

#10G
set_property PACKAGE_PIN B4 [get_ports xphy0_txp]
set_property PACKAGE_PIN B3 [get_ports xphy0_txn]
set_property PACKAGE_PIN A6 [get_ports xphy0_rxp]
set_property PACKAGE_PIN A5 [get_ports xphy0_rxn]

set_property PACKAGE_PIN C2 [get_ports xphy1_txp]
set_property PACKAGE_PIN C1 [get_ports xphy1_txn]
set_property PACKAGE_PIN B8 [get_ports xphy1_rxp]
set_property PACKAGE_PIN B7 [get_ports xphy1_rxn]

#set_property PACKAGE_PIN D4 [get_ports xphy2_txp]
#set_property PACKAGE_PIN D3 [get_ports xphy2_txn]
#set_property PACKAGE_PIN C6 [get_ports xphy2_rxp]
#set_property PACKAGE_PIN C5 [get_ports xphy2_rxn]

#set_property PACKAGE_PIN E2 [get_ports xphy3_txp]
#set_property PACKAGE_PIN E1 [get_ports xphy3_txn]
#set_property PACKAGE_PIN D8 [get_ports xphy3_rxp]
#set_property PACKAGE_PIN D7 [get_ports xphy3_rxn]

#create_clock -name xphy_rxusrclkout0 -period 3.103 [get_pins n10g_interface_inst/network_inst_0/ten_gig_eth_pcs_pma_inst/inst/gt0_gtwizard_gth_10gbaser_i/gthe2_i/RXOUTCLK]
#create_clock -name xphy_txusrclkout0 -period 3.103 [get_pins n10g_interface_inst/network_inst_0/ten_gig_eth_pcs_pma_inst/inst/gt0_gtwizard_gth_10gbaser_i/gthe2_i/TXOUTCLK]
#create_clock -name xphy_rxusrclkout1 -period 3.103 [get_pins network_inst_1/ten_gig_eth_pcs_pma_inst/inst/gt0_gtwizard_gth_10gbaser_i/gthe2_i/RXOUTCLK]
#create_clock -name xphy_txusrclkout1 -period 3.103 [get_pins network_inst_1/ten_gig_eth_pcs_pma_inst/inst/gt0_gtwizard_gth_10gbaser_i/gthe2_i/TXOUTCLK]
#create_clock -name xphy_rxusrclkout2 -period 3.103 [get_pins network_inst_2/ten_gig_eth_pcs_pma_inst/inst/gt0_gtwizard_gth_10gbaser_i/gthe2_i/RXOUTCLK]
#create_clock -name xphy_txusrclkout2 -period 3.103 [get_pins network_inst_2/ten_gig_eth_pcs_pma_inst/inst/gt0_gtwizard_gth_10gbaser_i/gthe2_i/TXOUTCLK]
#create_clock -name xphy_rxusrclkout3 -period 3.103 [get_pins network_inst_3/ten_gig_eth_pcs_pma_inst/inst/gt0_gtwizard_gth_10gbaser_i/gthe2_i/RXOUTCLK]
#create_clock -name xphy_txusrclkout3 -period 3.103 [get_pins network_inst_3/ten_gig_eth_pcs_pma_inst/inst/gt0_gtwizard_gth_10gbaser_i/gthe2_i/TXOUTCLK]

#########################################################
# PCIe
#########################################################
set_property PACKAGE_PIN AY35 [get_ports perst_n]
set_property IOSTANDARD LVCMOS18 [get_ports perst_n]
set_property PULLUP true [get_ports perst_n]

set_property PACKAGE_PIN AB7 [get_ports pcie_clk_n]
set_property PACKAGE_PIN AB8 [get_ports pcie_clk_p]

#set_false_path -from [get_ports perst_n]


# @yang, GTHE2_CHANNEL_XxYyy is based on corundum's xdc
# @yang, the others are based on sume master file at digilent
#set_property LOC GTHE2_CHANNEL_X1Y23 [get_cells {dma_inst/inst/pcie3_ip_i/inst/gt_top_i/pipe_wrapper_i/pipe_lane[0].gt_wrapper_i/gth_channel.gthe2_channel_i}]
set_property LOC GTHE2_CHANNEL_X1Y23 [get_cells {dma_driver_inst/dma_inst/inst/pcie3_ip_i/inst/gt_top_i/pipe_wrapper_i/pipe_lane[0].gt_wrapper_i/gth_channel.gthe2_channel_i}]
set_property PACKAGE_PIN Y3 [get_ports {pcie_rx_n[0]}]
set_property PACKAGE_PIN Y4 [get_ports {pcie_rx_p[0]}]
set_property PACKAGE_PIN W1 [get_ports {pcie_tx_n[0]}]
set_property PACKAGE_PIN W2 [get_ports {pcie_tx_p[0]}]
#set_property LOC GTHE2_CHANNEL_X1Y22 [get_cells {dma_inst/inst/pcie3_ip_i/inst/gt_top_i/pipe_wrapper_i/pipe_lane[1].gt_wrapper_i/gth_channel.gthe2_channel_i}]
set_property LOC GTHE2_CHANNEL_X1Y22 [get_cells {dma_driver_inst/dma_inst/inst/pcie3_ip_i/inst/gt_top_i/pipe_wrapper_i/pipe_lane[1].gt_wrapper_i/gth_channel.gthe2_channel_i}]
set_property PACKAGE_PIN AA5 [get_ports {pcie_rx_n[1]}]
set_property PACKAGE_PIN AA6 [get_ports {pcie_rx_p[1]}]
set_property PACKAGE_PIN AA1 [get_ports {pcie_tx_n[1]}]
set_property PACKAGE_PIN AA2 [get_ports {pcie_tx_p[1]}]
#set_property LOC GTHE2_CHANNEL_X1Y21 [get_cells {dma_inst/inst/pcie3_ip_i/inst/gt_top_i/pipe_wrapper_i/pipe_lane[2].gt_wrapper_i/gth_channel.gthe2_channel_i}]
set_property LOC GTHE2_CHANNEL_X1Y21 [get_cells {dma_driver_inst/dma_inst/inst/pcie3_ip_i/inst/gt_top_i/pipe_wrapper_i/pipe_lane[2].gt_wrapper_i/gth_channel.gthe2_channel_i}]
set_property PACKAGE_PIN AB3 [get_ports {pcie_rx_n[2]}]
set_property PACKAGE_PIN AB4 [get_ports {pcie_rx_p[2]}]
set_property PACKAGE_PIN AC1 [get_ports {pcie_tx_n[2]}]
set_property PACKAGE_PIN AC2 [get_ports {pcie_tx_p[2]}]
#set_property LOC GTHE2_CHANNEL_X1Y20 [get_cells {dma_inst/inst/pcie3_ip_i/inst/gt_top_i/pipe_wrapper_i/pipe_lane[3].gt_wrapper_i/gth_channel.gthe2_channel_i}]
set_property LOC GTHE2_CHANNEL_X1Y20 [get_cells {dma_driver_inst/dma_inst/inst/pcie3_ip_i/inst/gt_top_i/pipe_wrapper_i/pipe_lane[3].gt_wrapper_i/gth_channel.gthe2_channel_i}]
set_property PACKAGE_PIN AC5 [get_ports {pcie_rx_n[3]}]
set_property PACKAGE_PIN AC6 [get_ports {pcie_rx_p[3]}]
set_property PACKAGE_PIN AE1 [get_ports {pcie_tx_n[3]}]
set_property PACKAGE_PIN AE2 [get_ports {pcie_tx_p[3]}]
#set_property LOC GTHE2_CHANNEL_X1Y19 [get_cells {dma_inst/inst/pcie3_ip_i/inst/gt_top_i/pipe_wrapper_i/pipe_lane[4].gt_wrapper_i/gth_channel.gthe2_channel_i}]
set_property LOC GTHE2_CHANNEL_X1Y19 [get_cells {dma_driver_inst/dma_inst/inst/pcie3_ip_i/inst/gt_top_i/pipe_wrapper_i/pipe_lane[4].gt_wrapper_i/gth_channel.gthe2_channel_i}]
set_property PACKAGE_PIN AD3 [get_ports {pcie_rx_n[4]}]
set_property PACKAGE_PIN AD4 [get_ports {pcie_rx_p[4]}]
set_property PACKAGE_PIN AG1 [get_ports {pcie_tx_n[4]}]
set_property PACKAGE_PIN AG2 [get_ports {pcie_tx_p[4]}]
#set_property LOC GTHE2_CHANNEL_X1Y18 [get_cells {dma_inst/inst/pcie3_ip_i/inst/gt_top_i/pipe_wrapper_i/pipe_lane[5].gt_wrapper_i/gth_channel.gthe2_channel_i}]
set_property LOC GTHE2_CHANNEL_X1Y18 [get_cells {dma_driver_inst/dma_inst/inst/pcie3_ip_i/inst/gt_top_i/pipe_wrapper_i/pipe_lane[5].gt_wrapper_i/gth_channel.gthe2_channel_i}]
set_property PACKAGE_PIN AE5 [get_ports {pcie_rx_n[5]}]
set_property PACKAGE_PIN AE6 [get_ports {pcie_rx_p[5]}]
set_property PACKAGE_PIN AH3 [get_ports {pcie_tx_n[5]}]
set_property PACKAGE_PIN AH4 [get_ports {pcie_tx_p[5]}]
#set_property LOC GTHE2_CHANNEL_X1Y17 [get_cells {dma_inst/inst/pcie3_ip_i/inst/gt_top_i/pipe_wrapper_i/pipe_lane[6].gt_wrapper_i/gth_channel.gthe2_channel_i}]
set_property LOC GTHE2_CHANNEL_X1Y17 [get_cells {dma_driver_inst/dma_inst/inst/pcie3_ip_i/inst/gt_top_i/pipe_wrapper_i/pipe_lane[6].gt_wrapper_i/gth_channel.gthe2_channel_i}]
set_property PACKAGE_PIN AF3 [get_ports {pcie_rx_n[6]}]
set_property PACKAGE_PIN AF4 [get_ports {pcie_rx_p[6]}]
set_property PACKAGE_PIN AJ1 [get_ports {pcie_tx_n[6]}]
set_property PACKAGE_PIN AJ2 [get_ports {pcie_tx_p[6]}]
#set_property LOC GTHE2_CHANNEL_X1Y16 [get_cells {dma_inst/inst/pcie3_ip_i/inst/gt_top_i/pipe_wrapper_i/pipe_lane[7].gt_wrapper_i/gth_channel.gthe2_channel_i}]
set_property LOC GTHE2_CHANNEL_X1Y16 [get_cells {dma_driver_inst/dma_inst/inst/pcie3_ip_i/inst/gt_top_i/pipe_wrapper_i/pipe_lane[7].gt_wrapper_i/gth_channel.gthe2_channel_i}]
set_property PACKAGE_PIN AG5 [get_ports {pcie_rx_n[7]}]
set_property PACKAGE_PIN AG6 [get_ports {pcie_rx_p[7]}]
set_property PACKAGE_PIN AK3 [get_ports {pcie_tx_n[7]}]
set_property PACKAGE_PIN AK4 [get_ports {pcie_tx_p[7]}]


#########################################################
# LEDS
#########################################################

# set_property -dict {LOC G13  IOSTANDARD LVCMOS15 SLEW SLOW DRIVE 4} [get_ports {led[0]}]
# set_property -dict {LOC L15  IOSTANDARD LVCMOS15 SLEW SLOW DRIVE 4} [get_ports {led[1]}]
# set_property -dict {LOC AL22  IOSTANDARD LVCMOS15 SLEW SLOW DRIVE 4} [get_ports {led[2]}]
# set_property -dict {LOC BA20  IOSTANDARD LVCMOS15 SLEW SLOW DRIVE 4} [get_ports {led[3]}]
# set_property -dict {LOC AY18  IOSTANDARD LVCMOS15 SLEW SLOW DRIVE 4} [get_ports {led[4]}]
# set_property -dict {LOC AY17  IOSTANDARD LVCMOS15 SLEW SLOW DRIVE 4} [get_ports {led[5]}]
# set_property -dict {LOC P31  IOSTANDARD LVCMOS15 SLEW SLOW DRIVE 4} [get_ports {led[6]}]
# set_property -dict {LOC K32  IOSTANDARD LVCMOS15 SLEW SLOW DRIVE 4} [get_ports {led[7]}]

# @yang, temporary cut it to six to save more ports for ddr. 
set_property -dict {LOC L15  IOSTANDARD LVCMOS15 SLEW SLOW DRIVE 12} [get_ports {led[0]}]
set_property -dict {LOC BA20  IOSTANDARD LVCMOS15 SLEW SLOW DRIVE 12} [get_ports {led[1]}]
set_property -dict {LOC AY18  IOSTANDARD LVCMOS15 SLEW SLOW DRIVE 12} [get_ports {led[2]}]
set_property -dict {LOC AY17  IOSTANDARD LVCMOS15 SLEW SLOW DRIVE 12} [get_ports {led[3]}]
set_property -dict {LOC P31  IOSTANDARD LVCMOS15 SLEW SLOW DRIVE 12} [get_ports {led[4]}]
set_property -dict {LOC K32  IOSTANDARD LVCMOS15 SLEW SLOW DRIVE 12} [get_ports {led[5]}]



# @yang, the above are sfp_led, below is normal led
# set_property -dict {LOC AR22 IOSTANDARD LVCMOS15 SLEW SLOW DRIVE 12} [get_ports {led[0]}]
# set_property -dict {LOC AR23 IOSTANDARD LVCMOS15 SLEW SLOW DRIVE 12} [get_ports {led[1]}]

##
## Switches
## @yang, seems that sume does not have 8 gpio switch, it only has two led and btn. 
# set_property PACKAGE_PIN AR22 [get_ports {gpio_switch[0]}]
# set_property PACKAGE_PIN AR22 [get_ports {gpio_switch[1]}]
# set_property PACKAGE_PIN AR22 [get_ports {gpio_switch[2]}]
# set_property PACKAGE_PIN AR22 [get_ports {gpio_switch[3]}]
# set_property PACKAGE_PIN AR23 [get_ports {gpio_switch[4]}]
# set_property PACKAGE_PIN AR23 [get_ports {gpio_switch[5]}]
# set_property PACKAGE_PIN AR23 [get_ports {gpio_switch[6]}]
# set_property PACKAGE_PIN AR23 [get_ports {gpio_switch[7]}]

# set_property IOSTANDARD LVCMOS15 [get_ports {gpio_switch[0]}]
# set_property IOSTANDARD LVCMOS15 [get_ports {gpio_switch[1]}]
# set_property IOSTANDARD LVCMOS15 [get_ports {gpio_switch[2]}]
# set_property IOSTANDARD LVCMOS15 [get_ports {gpio_switch[3]}]
# set_property IOSTANDARD LVCMOS15 [get_ports {gpio_switch[4]}]
# set_property IOSTANDARD LVCMOS15 [get_ports {gpio_switch[5]}]
# set_property IOSTANDARD LVCMOS15 [get_ports {gpio_switch[6]}]
# set_property IOSTANDARD LVCMOS15 [get_ports {gpio_switch[7]}]

#i2c clk & stuff
set_property IOSTANDARD LVCMOS18 [get_ports i2c_clk]
set_property SLEW SLOW [get_ports i2c_clk]
set_property DRIVE 16 [get_ports i2c_clk]
set_property PULLUP TRUE [get_ports i2c_clk]
set_property LOC AK24 [get_ports i2c_clk]

set_property IOSTANDARD LVCMOS18 [get_ports i2c_data]
set_property SLEW SLOW [get_ports i2c_data]
set_property DRIVE 16 [get_ports i2c_data]
set_property PULLUP TRUE [get_ports i2c_data]
set_property LOC AK25 [get_ports i2c_data]

set_property IOSTANDARD LVCMOS15 [get_ports i2c_mux_rst_n]
set_property SLEW SLOW [get_ports i2c_mux_rst_n]
set_property DRIVE 16 [get_ports i2c_mux_rst_n]
set_property LOC AM39 [get_ports i2c_mux_rst_n]

set_property IOSTANDARD LVCMOS18 [get_ports si5324_rst_n]
set_property SLEW SLOW [get_ports si5324_rst_n]
set_property DRIVE 16 [get_ports si5324_rst_n]
set_property LOC BA29 [get_ports si5324_rst_n]


#Domain crossing constraints
set_clock_groups -name async_mcb_xgemac -asynchronous \
  -group [get_clocks  mcb_clk_ref] \
  -group [get_clocks  clk156]



set_clock_groups -name async_mig_ref_clk50 -asynchronous \
   -group [get_clocks mcb_clk_ref] \
   -group [get_clocks clk50]


#set_clock_groups -name async_rxusrclk_xgemac -asynchronous \
#  -group [get_clocks  xphy_rxusrclkout?] \
#  -group [get_clocks  clk156]

#set_clock_groups -name async_txusrclk_xgemac -asynchronous \
#  -group [get_clocks  xphy_txusrclkout?] \
#  -group [get_clocks  clk156]

#  set_clock_groups -name async_txusrclk_refclk -asynchronous \
#    -group [get_clocks  xphy_txusrclkout?] \
#    -group [get_clocks  -include_generated_clocks refclk]


set_clock_groups -name async_xgemac_drpclk -asynchronous \
   -group [get_clocks -include_generated_clocks clk156] \
   -group [get_clocks -include_generated_clocks dclk]
   
set_clock_groups -name async_xgemac_clk50 -asynchronous \
   -group [get_clocks -include_generated_clocks clk156] \
   -group [get_clocks clk50]
   
####contraints from DRAM MEM inf
create_clock -period 4.708 -name sys_clk [get_ports sys_clk_p]

# PadFunction: IO_L13P_T2_MRCC_32
set_property VCCAUX_IO DONTCARE [get_ports {sys_clk_p}]
set_property IOSTANDARD DIFF_SSTL15 [get_ports {sys_clk_p}]
set_property PACKAGE_PIN E34 [get_ports {sys_clk_p}]

# PadFunction: IO_L13N_T2_MRCC_32
set_property VCCAUX_IO DONTCARE [get_ports {sys_clk_n}]
set_property IOSTANDARD DIFF_SSTL15 [get_ports {sys_clk_n}]
set_property PACKAGE_PIN E35 [get_ports {sys_clk_n}]

# Reset @yang, I did not find this dram rst in sume, set it to CPLD rst
# PadFunction: IO_L13P_T2_MRCC_15 
set_property VCCAUX_IO DONTCARE [get_ports {sys_rst_i}]
set_property IOSTANDARD LVCMOS18 [get_ports {sys_rst_i}]
set_property PACKAGE_PIN AR32 [get_ports {sys_rst_i}]


set_property CLOCK_DEDICATED_ROUTE BACKBONE [get_nets clk233]

set_clock_groups -name clk156_pll_i -asynchronous -group [get_clocks clk_pll_i] -group [get_clocks clk156]
set_clock_groups -name clk156_pll_i_1 -asynchronous -group [get_clocks clk_pll_i_1] -group [get_clocks clk156]