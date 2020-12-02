# BIFILE/BITSTERAM compress options
# Flash type constraints. These should be modified to match the target board.
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 8 [current_design]
set_property BITSTREAM.CONFIG.EXTMASTERCCLK_EN div-1 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.SPI_FALL_EDGE YES [current_design]


# set_property IOSTANDARD LVDS [get_ports dclk_p]
# set_property IOSTANDARD LVDS [get_ports dclk_n]

# set_property PACKAGE_PIN AY24 [get_ports dclk_p]
# set_property PACKAGE_PIN AY23 [get_ports dclk_n]

# replace the above 125 MHz clock with the following 156.25 MHz clock; changing cmac_usplus GT_DRP_CLK setting accordingly
set_property	PACKAGE_PIN	AV19		        [get_ports 	dclk_n] ; 
set_property	IOSTANDARD		LVDS	        [get_ports 	dclk_n] ; 
set_property	PACKAGE_PIN	AU19		        [get_ports 	dclk_p] ; 
set_property	IOSTANDARD		LVDS	        [get_ports 	dclk_p] ; 


### These are sample constraints, please use correct constraints for your device
### update the gt_refclk pin location accordingly and un-comment the below two lines
# 161MHz
set_property PACKAGE_PIN K10 [get_ports gt_refclk_n]
set_property PACKAGE_PIN K11 [get_ports gt_refclk_p]

#QSPF28 Connector1
# set_property	PACKAGE_PIN	N3		[get_ports 	{gt_rxn_in[0]}	] ; 
# set_property	PACKAGE_PIN	M1		[get_ports 	{gt_rxn_in[1]}	] ; 
# set_property	PACKAGE_PIN	L3		[get_ports 	{gt_rxn_in[2]}	] ; 
# set_property	PACKAGE_PIN	K1		[get_ports 	{gt_rxn_in[3]}	] ; 
# set_property	PACKAGE_PIN	N4		[get_ports 	{gt_rxp_in[0]}	] ; 
# set_property	PACKAGE_PIN	M2		[get_ports 	{gt_rxp_in[1]}	] ; 
# set_property	PACKAGE_PIN	L4		[get_ports 	{gt_rxp_in[2]}	] ; 
# set_property	PACKAGE_PIN	K2		[get_ports 	{gt_rxp_in[3]}	] ; 
# set_property	PACKAGE_PIN	N8		[get_ports 	{gt_txn_out[0]}	] ; 
# set_property	PACKAGE_PIN	M6		[get_ports 	{gt_txn_out[1]}	] ; 
# set_property	PACKAGE_PIN	L8		[get_ports 	{gt_txn_out[2]}	] ; 
# set_property	PACKAGE_PIN	K6		[get_ports 	{gt_txn_out[3]}	] ; 
# set_property	PACKAGE_PIN	N9		[get_ports 	{gt_txp_out[0]}	] ; 
# set_property	PACKAGE_PIN	M7		[get_ports 	{gt_txp_out[1]}	] ; 
# set_property	PACKAGE_PIN	L9		[get_ports 	{gt_txp_out[2]}	] ; 
# set_property	PACKAGE_PIN	K7		[get_ports 	{gt_txp_out[3]}	] ; 

#QSPF28 Connector2
set_property	PACKAGE_PIN	U3		[get_ports 	{gt_rxn_in[0]}	] ; 
set_property	PACKAGE_PIN	T1		[get_ports 	{gt_rxn_in[1]}	] ; 
set_property	PACKAGE_PIN	R3		[get_ports 	{gt_rxn_in[2]}	] ; 
set_property	PACKAGE_PIN	P1		[get_ports 	{gt_rxn_in[3]}	] ; 
set_property	PACKAGE_PIN	U4		[get_ports 	{gt_rxp_in[0]}	] ; 
set_property	PACKAGE_PIN	T2		[get_ports 	{gt_rxp_in[1]}	] ; 
set_property	PACKAGE_PIN	R4		[get_ports 	{gt_rxp_in[2]}	] ; 
set_property	PACKAGE_PIN	P2		[get_ports 	{gt_rxp_in[3]}	] ; 
set_property	PACKAGE_PIN	U8		[get_ports 	{gt_txn_out[0]}	] ; 
set_property	PACKAGE_PIN	T6		[get_ports 	{gt_txn_out[1]}	] ; 
set_property	PACKAGE_PIN	R8		[get_ports 	{gt_txn_out[2]}	] ; 
set_property	PACKAGE_PIN	P6		[get_ports 	{gt_txn_out[3]}	] ; 
set_property	PACKAGE_PIN	U9		[get_ports 	{gt_txp_out[0]}	] ; 
set_property	PACKAGE_PIN	T7		[get_ports 	{gt_txp_out[1]}	] ; 
set_property	PACKAGE_PIN	R9		[get_ports 	{gt_txp_out[2]}	] ; 
set_property	PACKAGE_PIN	P7		[get_ports 	{gt_txp_out[3]}	] ; 

# CPU_RESET
set_property -dict {LOC AL20 IOSTANDARD LVCMOS12} [get_ports sys_reset]

###Board constraints to be added here
### Below XDC constraints are for VCU108 board with xcvu095-ffva2104-2-e-es2 device
### Change these constraints as per your board and device

# LED
set_property -dict {LOC BC21 IOSTANDARD LVCMOS12 SLEW SLOW DRIVE 8} [get_ports {led[0]}]
set_property -dict {LOC BB21 IOSTANDARD LVCMOS12 SLEW SLOW DRIVE 8} [get_ports {led[1]}]
set_property -dict {LOC BA20 IOSTANDARD LVCMOS12 SLEW SLOW DRIVE 8} [get_ports {led[2]}]


create_clock -period 6.400 -name dclk_clk [get_pins dclk_BUFG_inst/O]



#set_property IOSTANDARD LVDS [get_ports uclk_p]
#set_property IOSTANDARD LVDS [get_ports uclk_n]

#set_property PACKAGE_PIN AW22 [get_ports uclk_n]
#set_property PACKAGE_PIN AW23 [get_ports uclk_p]

#create_clock -period 6.400 -name uclk_clk [get_pins uclk_BUFG_inst/O]



set_max_delay -datapath_only -from [get_clocks -of_objects [get_pins -hierarchical -filter {NAME =~ */channel_inst/*_CHANNEL_PRIM_INST/RXOUTCLK}]] -to [get_clocks -of_objects [get_pins -hierarchical -filter {NAME =~ */channel_inst/*_CHANNEL_PRIM_INST/TXOUTCLK}]] 6.206
set_max_delay -datapath_only -from [get_clocks -of_objects [get_pins -hierarchical -filter {NAME =~ */channel_inst/*_CHANNEL_PRIM_INST/TXOUTCLK}]] -to [get_clocks -of_objects [get_pins -hierarchical -filter {NAME =~ */channel_inst/*_CHANNEL_PRIM_INST/RXOUTCLK}]] 6.206


set_max_delay -datapath_only -from [get_clocks dclk_clk] -to [get_clocks -of_objects [get_pins -hierarchical -filter {NAME =~ */channel_inst/*_CHANNEL_PRIM_INST/TXOUTCLK}]] 6.400
set_max_delay -datapath_only -from [get_clocks dclk_clk] -to [get_clocks -of_objects [get_pins -hierarchical -filter {NAME =~ */channel_inst/*_CHANNEL_PRIM_INST/RXOUTCLK}]] 6.400


set_max_delay -datapath_only -from [get_clocks -of_objects [get_pins -hierarchical -filter {NAME =~ */channel_inst/*_CHANNEL_PRIM_INST/RXOUTCLK}]] -to [get_clocks dclk_clk] 6.206
set_max_delay -datapath_only -from [get_clocks -of_objects [get_pins -hierarchical -filter {NAME =~ */channel_inst/*_CHANNEL_PRIM_INST/TXOUTCLK}]] -to [get_clocks dclk_clk] 6.206


###
# DDR 0
###
# 250 MHZ clk
set_property	PACKAGE_PIN	AY38		        [get_ports 	c0_sys_clk_n] ; 
set_property	IOSTANDARD		DIFF_POD12_DCI	[get_ports 	c0_sys_clk_n] ; 
set_property	PACKAGE_PIN	AY37		        [get_ports 	c0_sys_clk_p] ; 
set_property	IOSTANDARD		DIFF_POD12_DCI	[get_ports 	c0_sys_clk_p] ; 

set_property -dict {PACKAGE_PIN AR36 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_adr[16]  ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_ADR16"   - IO_L23N_T3U_N9_42
set_property -dict {PACKAGE_PIN AP36 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_adr[15]  ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_ADR15"   - IO_L23P_T3U_N8_42
#set_property -dict {PACKAGE_PIN AN34 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_odt[1]   ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_ODT1"    - IO_L22N_T3U_N7_DBC_AD0N_42
#set_property -dict {PACKAGE_PIN AM34 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_cs_n[3]  ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_CS_B3"   - IO_L22P_T3U_N6_DBC_AD0P_42
set_property -dict {PACKAGE_PIN AR33 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_cs_n[0]  ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_CS_B0"   - IO_T3U_N12_42
set_property -dict {PACKAGE_PIN AN36 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_adr[13]  ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_ADR13"   - IO_L24N_T3U_N11_42
#set_property -dict {PACKAGE_PIN AN35 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_adr[17]  ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_ADR17"   - IO_L24P_T3U_N10_42
set_property -dict {PACKAGE_PIN AP35 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_adr[14]  ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_ADR14"   - IO_L21N_T3L_N5_AD8N_42
set_property -dict {PACKAGE_PIN AP34 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_odt[0]   ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_ODT0"    - IO_L21P_T3L_N4_AD8P_42
#set_property -dict {PACKAGE_PIN AP33 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_cs_n[1]  ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_CS_B1"   - IO_L20N_T3L_N3_AD1N_42
#set_property -dict {PACKAGE_PIN AN33 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_cs_n[2]  ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_CS_B2"   - IO_L20P_T3L_N2_AD1P_42
set_property -dict {PACKAGE_PIN AT35 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_ba[0]    ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_BA0"     - IO_L19N_T3L_N1_DBC_AD9N_42
set_property -dict {PACKAGE_PIN AR35 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_adr[10]  ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_ADR10"   - IO_L19P_T3L_N0_DBC_AD9P_42
set_property -dict {PACKAGE_PIN AW38 IOSTANDARD DIFF_SSTL12_DCI} [get_ports c0_ddr4_ck_c[0]  ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_CK_C0"   - IO_L17N_T2U_N9_AD10N_42
set_property -dict {PACKAGE_PIN AV38 IOSTANDARD DIFF_SSTL12_DCI} [get_ports c0_ddr4_ck_t[0]  ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_CK_T0"   - IO_L17P_T2U_N8_AD10P_42
#set_property -dict {PACKAGE_PIN AU35 IOSTANDARD DIFF_SSTL12_DCI} [get_ports c0_ddr4_ck_c[1]  ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_CK_C1"   - IO_L16N_T2U_N7_QBC_AD3N_42
#set_property -dict {PACKAGE_PIN AU34 IOSTANDARD DIFF_SSTL12_DCI} [get_ports c0_ddr4_ck_t[1]  ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_CK_T1"   - IO_L16P_T2U_N6_QBC_AD3P_42
set_property -dict {PACKAGE_PIN AT34 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_ba[1]    ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_BA1"     - IO_T2U_N12_42
set_property -dict {PACKAGE_PIN AU36 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_parity   ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_PAR"     - IO_L18N_T2U_N11_AD2N_42
set_property -dict {PACKAGE_PIN AT36 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_adr[0]   ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_ADR0"    - IO_L18P_T2U_N10_AD2P_42
set_property -dict {PACKAGE_PIN AV37 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_adr[2]   ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_ADR2"    - IO_L15N_T2L_N5_AD11N_42
set_property -dict {PACKAGE_PIN AV36 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_adr[1]   ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_ADR1"    - IO_L15P_T2L_N4_AD11P_42
set_property -dict {PACKAGE_PIN AW36 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_adr[4]   ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_ADR4"    - IO_L14N_T2L_N3_GC_42
set_property -dict {PACKAGE_PIN AW35 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_adr[3]   ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_ADR3"    - IO_L14P_T2L_N2_GC_42
set_property -dict {PACKAGE_PIN BA38 IOSTANDARD LVCMOS12       } [get_ports c0_ddr4_alert_n  ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_ALERT_B" - IO_L11N_T1U_N9_GC_42
set_property -dict {PACKAGE_PIN BA37 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_adr[8]   ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_ADR8"    - IO_L11P_T1U_N8_GC_42
set_property -dict {PACKAGE_PIN BA40 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_adr[7]   ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_ADR7"    - IO_L10N_T1U_N7_QBC_AD4N_42
set_property -dict {PACKAGE_PIN BA39 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_adr[11]  ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_ADR11"   - IO_L10P_T1U_N6_QBC_AD4P_42
set_property -dict {PACKAGE_PIN BB37 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_adr[9]   ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_ADR9"    - IO_T1U_N12_42
set_property -dict {PACKAGE_PIN AY36 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_adr[5]   ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_ADR5"    - IO_L12N_T1U_N11_GC_42
set_property -dict {PACKAGE_PIN AY35 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_adr[6]   ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_ADR6"    - IO_L12P_T1U_N10_GC_42
#set_property -dict {PACKAGE_PIN BC40 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_cke[1]   ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_CKE1"    - IO_L9N_T1L_N5_AD12N_42
set_property -dict {PACKAGE_PIN BC39 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_bg[1]    ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_BG1"     - IO_L9P_T1L_N4_AD12P_42
set_property -dict {PACKAGE_PIN BB40 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_adr[12]  ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_ADR12"   - IO_L8N_T1L_N3_AD5N_42
set_property -dict {PACKAGE_PIN BB39 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_act_n    ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_ACT_B"   - IO_L8P_T1L_N2_AD5P_42
set_property -dict {PACKAGE_PIN BC38 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_cke[0]   ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_CKE0"    - IO_L7N_T1L_N1_QBC_AD13N_42
set_property -dict {PACKAGE_PIN BC37 IOSTANDARD SSTL12_DCI     } [get_ports c0_ddr4_bg[0]    ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_BG0"     - IO_L7P_T1L_N0_QBC_AD13P_42
set_property -dict {PACKAGE_PIN BF43 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[66]   ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_DQ66"    - IO_L5N_T0U_N9_AD14N_42
set_property -dict {PACKAGE_PIN BF42 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[67]   ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_DQ67"    - IO_L5P_T0U_N8_AD14P_42
set_property -dict {PACKAGE_PIN BF38 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_c[16]]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_DQS_C8"  - IO_L4N_T0U_N7_DBC_AD7N_42
set_property -dict {PACKAGE_PIN BE38 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_t[16]]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_DQS_T8"  - IO_L4P_T0U_N6_DBC_AD7P_42
set_property -dict {PACKAGE_PIN BD40 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[64]   ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_DQ64"    - IO_L6N_T0U_N11_AD6N_42
set_property -dict {PACKAGE_PIN BD39 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[65]   ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_DQ65"    - IO_L6P_T0U_N10_AD6P_42
set_property -dict {PACKAGE_PIN BF41 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[71]   ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_DQ71"    - IO_L3N_T0L_N5_AD15N_42
set_property -dict {PACKAGE_PIN BE40 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[70]   ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_DQ70"    - IO_L3P_T0L_N4_AD15P_42
set_property -dict {PACKAGE_PIN BF37 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[68]   ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_DQ68"    - IO_L2N_T0L_N3_42
set_property -dict {PACKAGE_PIN BE37 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[69]   ]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_DQ69"    - IO_L2P_T0L_N2_42
set_property -dict {PACKAGE_PIN BF40 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_c[17]]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_DQS_C17" - IO_L1N_T0L_N1_DBC_42
set_property -dict {PACKAGE_PIN BF39 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_t[17]]; # Bank 42 VCCO - VCC1V2 Net "DDR4_C0_DQS_T17" - IO_L1P_T0L_N0_DBC_42
set_property -dict {PACKAGE_PIN AU32 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[34]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ34"    - IO_L23N_T3U_N9_41
set_property -dict {PACKAGE_PIN AT32 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[35]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ35"    - IO_L23P_T3U_N8_41
set_property -dict {PACKAGE_PIN AM32 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_c[8] ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQS_C4"  - IO_L22N_T3U_N7_DBC_AD0N_41
set_property -dict {PACKAGE_PIN AM31 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_t[8] ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQS_T4"  - IO_L22P_T3U_N6_DBC_AD0P_41
#set_property -dict {PACKAGE_PIN AT33 IOSTANDARD LVCMOS12       } [get_ports c0_ddr4_event_n  ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_EVENT_B" - IO_T3U_N12_41
set_property -dict {PACKAGE_PIN AM30 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[33]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ33"    - IO_L24N_T3U_N11_41
set_property -dict {PACKAGE_PIN AL30 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[32]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ32"    - IO_L24P_T3U_N10_41
set_property -dict {PACKAGE_PIN AR32 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[38]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ38"    - IO_L21N_T3L_N5_AD8N_41
set_property -dict {PACKAGE_PIN AR31 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[39]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ39"    - IO_L21P_T3L_N4_AD8P_41
set_property -dict {PACKAGE_PIN AN32 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[37]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ37"    - IO_L20N_T3L_N3_AD1N_41
set_property -dict {PACKAGE_PIN AN31 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[36]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ36"    - IO_L20P_T3L_N2_AD1P_41
set_property -dict {PACKAGE_PIN AP31 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_c[9] ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQS_C13" - IO_L19N_T3L_N1_DBC_AD9N_41
set_property -dict {PACKAGE_PIN AP30 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_t[9] ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQS_T13" - IO_L19P_T3L_N0_DBC_AD9P_41
set_property -dict {PACKAGE_PIN AV32 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[25]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ25"    - IO_L17N_T2U_N9_AD10N_41
set_property -dict {PACKAGE_PIN AV31 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[24]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ24"    - IO_L17P_T2U_N8_AD10P_41
set_property -dict {PACKAGE_PIN AW33 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_c[6] ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQS_C3"  - IO_L16N_T2U_N7_QBC_AD3N_41
set_property -dict {PACKAGE_PIN AV33 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_t[6] ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQS_T3"  - IO_L16P_T2U_N6_QBC_AD3P_41
set_property -dict {PACKAGE_PIN AU31 IOSTANDARD LVCMOS12       } [get_ports c0_ddr4_reset_n  ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_RESET_N" - IO_T2U_N12_41
set_property -dict {PACKAGE_PIN AW34 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[27]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ27"    - IO_L18N_T2U_N11_AD2N_41
set_property -dict {PACKAGE_PIN AV34 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[26]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ26"    - IO_L18P_T2U_N10_AD2P_41
set_property -dict {PACKAGE_PIN AY31 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[29]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ29"    - IO_L15N_T2L_N5_AD11N_41
set_property -dict {PACKAGE_PIN AW31 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[28]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ28"    - IO_L15P_T2L_N4_AD11P_41
set_property -dict {PACKAGE_PIN BA35 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[30]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ30"    - IO_L14N_T2L_N3_GC_41
set_property -dict {PACKAGE_PIN BA34 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[31]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ31"    - IO_L14P_T2L_N2_GC_41
set_property -dict {PACKAGE_PIN BA33 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_c[7] ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQS_C12" - IO_L13N_T2L_N1_GC_QBC_41
set_property -dict {PACKAGE_PIN BA32 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_t[7] ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQS_T12" - IO_L13P_T2L_N0_GC_QBC_41
set_property -dict {PACKAGE_PIN BB32 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[17]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ17"    - IO_L11N_T1U_N9_GC_41
set_property -dict {PACKAGE_PIN BB31 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[16]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ16"    - IO_L11P_T1U_N8_GC_41
set_property -dict {PACKAGE_PIN BB36 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_c[4] ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQS_C2"  - IO_L10N_T1U_N7_QBC_AD4N_41
set_property -dict {PACKAGE_PIN BB35 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_t[4] ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQS_T2"  - IO_L10P_T1U_N6_QBC_AD4P_41
set_property -dict {PACKAGE_PIN AY33 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[19]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ19"    - IO_L12N_T1U_N11_GC_41
set_property -dict {PACKAGE_PIN AY32 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[18]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ18"    - IO_L12P_T1U_N10_GC_41
set_property -dict {PACKAGE_PIN BC33 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[21]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ21"    - IO_L9N_T1L_N5_AD12N_41
set_property -dict {PACKAGE_PIN BC32 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[20]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ20"    - IO_L9P_T1L_N4_AD12P_41
set_property -dict {PACKAGE_PIN BC34 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[23]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ23"    - IO_L8N_T1L_N3_AD5N_41
set_property -dict {PACKAGE_PIN BB34 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[22]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ22"    - IO_L8P_T1L_N2_AD5P_41
set_property -dict {PACKAGE_PIN BD31 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_c[5] ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQS_C11" - IO_L7N_T1L_N1_QBC_AD13N_41
set_property -dict {PACKAGE_PIN BC31 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_t[5] ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQS_T11" - IO_L7P_T1L_N0_QBC_AD13P_41
set_property -dict {PACKAGE_PIN BE33 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[58]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ58"    - IO_L5N_T0U_N9_AD14N_41
set_property -dict {PACKAGE_PIN BD33 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[57]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ57"    - IO_L5P_T0U_N8_AD14P_41
set_property -dict {PACKAGE_PIN BE36 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_c[14]]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQS_C7"  - IO_L4N_T0U_N7_DBC_AD7N_41
set_property -dict {PACKAGE_PIN BE35 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_t[14]]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQS_T7"  - IO_L4P_T0U_N6_DBC_AD7P_41
set_property -dict {PACKAGE_PIN BD35 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[59]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ59"    - IO_L6N_T0U_N11_AD6N_41
set_property -dict {PACKAGE_PIN BD34 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[56]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ56"    - IO_L6P_T0U_N10_AD6P_41
set_property -dict {PACKAGE_PIN BF33 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[61]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ61"    - IO_L3N_T0L_N5_AD15N_41
set_property -dict {PACKAGE_PIN BF32 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[60]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ60"    - IO_L3P_T0L_N4_AD15P_41
set_property -dict {PACKAGE_PIN BF35 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[63]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ63"    - IO_L2N_T0L_N3_41
set_property -dict {PACKAGE_PIN BF34 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[62]   ]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQ62"    - IO_L2P_T0L_N2_41
set_property -dict {PACKAGE_PIN BE32 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_c[15]]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQS_C16" - IO_L1N_T0L_N1_DBC_41
set_property -dict {PACKAGE_PIN BE31 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_t[15]]; # Bank 41 VCCO - VCC1V2 Net "DDR4_C0_DQS_T16" - IO_L1P_T0L_N0_DBC_41
set_property -dict {PACKAGE_PIN AP29 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[40]   ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ40"    - IO_L23N_T3U_N9_40
set_property -dict {PACKAGE_PIN AP28 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[41]   ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ41"    - IO_L23P_T3U_N8_40
set_property -dict {PACKAGE_PIN AL29 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_c[10]]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQS_C5"  - IO_L22N_T3U_N7_DBC_AD0N_40
set_property -dict {PACKAGE_PIN AL28 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_t[10]]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQS_T5"  - IO_L22P_T3U_N6_DBC_AD0P_40
set_property -dict {PACKAGE_PIN AN27 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[42]   ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ42"    - IO_L24N_T3U_N11_40
set_property -dict {PACKAGE_PIN AM27 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[43]   ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ43"    - IO_L24P_T3U_N10_40
set_property -dict {PACKAGE_PIN AR28 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[47]   ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ47"    - IO_L21N_T3L_N5_AD8N_40
set_property -dict {PACKAGE_PIN AR27 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[46]   ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ46"    - IO_L21P_T3L_N4_AD8P_40
set_property -dict {PACKAGE_PIN AN29 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[44]   ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ44"    - IO_L20N_T3L_N3_AD1N_40
set_property -dict {PACKAGE_PIN AM29 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[45]   ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ45"    - IO_L20P_T3L_N2_AD1P_40
set_property -dict {PACKAGE_PIN AT30 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_c[11]]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQS_C14" - IO_L19N_T3L_N1_DBC_AD9N_40
set_property -dict {PACKAGE_PIN AR30 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_t[11]]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQS_T14" - IO_L19P_T3L_N0_DBC_AD9P_40
set_property -dict {PACKAGE_PIN AV27 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[49]   ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ49"    - IO_L17N_T2U_N9_AD10N_40
set_property -dict {PACKAGE_PIN AU27 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[50]   ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ50"    - IO_L17P_T2U_N8_AD10P_40
set_property -dict {PACKAGE_PIN AU30 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_c[12]]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQS_C6"  - IO_L16N_T2U_N7_QBC_AD3N_40
set_property -dict {PACKAGE_PIN AU29 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_t[12]]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQS_T6"  - IO_L16P_T2U_N6_QBC_AD3P_40
set_property -dict {PACKAGE_PIN AT28 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[48]   ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ48"    - IO_L18N_T2U_N11_AD2N_40
set_property -dict {PACKAGE_PIN AT27 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[51]   ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ51"    - IO_L18P_T2U_N10_AD2P_40
set_property -dict {PACKAGE_PIN AV29 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[52]   ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ52"    - IO_L15N_T2L_N5_AD11N_40
set_property -dict {PACKAGE_PIN AV28 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[55]   ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ55"    - IO_L15P_T2L_N4_AD11P_40
set_property -dict {PACKAGE_PIN AY30 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[53]   ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ53"    - IO_L14N_T2L_N3_GC_40
set_property -dict {PACKAGE_PIN AW30 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[54]   ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ54"    - IO_L14P_T2L_N2_GC_40
set_property -dict {PACKAGE_PIN AY28 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_c[13]]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQS_C15" - IO_L13N_T2L_N1_GC_QBC_40
set_property -dict {PACKAGE_PIN AY27 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_t[13]]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQS_T15" - IO_L13P_T2L_N0_GC_QBC_40
set_property -dict {PACKAGE_PIN BA28 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[2]    ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ2"     - IO_L11N_T1U_N9_GC_40
set_property -dict {PACKAGE_PIN BA27 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[3]    ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ3"     - IO_L11P_T1U_N8_GC_40
set_property -dict {PACKAGE_PIN BB30 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_c[0] ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQS_C0"  - IO_L10N_T1U_N7_QBC_AD4N_40
set_property -dict {PACKAGE_PIN BA30 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_t[0] ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQS_T0"  - IO_L10P_T1U_N6_QBC_AD4P_40
set_property -dict {PACKAGE_PIN AW29 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[1]    ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ1"     - IO_L12N_T1U_N11_GC_40
set_property -dict {PACKAGE_PIN AW28 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[0]    ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ0"     - IO_L12P_T1U_N10_GC_40
set_property -dict {PACKAGE_PIN BC27 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[6]    ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ6"     - IO_L9N_T1L_N5_AD12N_40
set_property -dict {PACKAGE_PIN BB27 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[7]    ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ7"     - IO_L9P_T1L_N4_AD12P_40
set_property -dict {PACKAGE_PIN BB29 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[4]    ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ4"     - IO_L8N_T1L_N3_AD5N_40
set_property -dict {PACKAGE_PIN BA29 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[5]    ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ5"     - IO_L8P_T1L_N2_AD5P_40
set_property -dict {PACKAGE_PIN BC26 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_c[1] ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQS_C9"  - IO_L7N_T1L_N1_QBC_AD13N_40
set_property -dict {PACKAGE_PIN BB26 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_t[1] ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQS_T9"  - IO_L7P_T1L_N0_QBC_AD13P_40
set_property -dict {PACKAGE_PIN BF28 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[9]    ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ9"     - IO_L5N_T0U_N9_AD14N_40
set_property -dict {PACKAGE_PIN BE28 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[8]    ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ8"     - IO_L5P_T0U_N8_AD14P_40
set_property -dict {PACKAGE_PIN BD29 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_c[2] ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQS_C1"  - IO_L4N_T0U_N7_DBC_AD7N_40
set_property -dict {PACKAGE_PIN BD28 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_t[2] ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQS_T1"  - IO_L4P_T0U_N6_DBC_AD7P_40
set_property -dict {PACKAGE_PIN BE30 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[10]   ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ10"    - IO_L6N_T0U_N11_AD6N_40
set_property -dict {PACKAGE_PIN BD30 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[11]   ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ11"    - IO_L6P_T0U_N10_AD6P_40
set_property -dict {PACKAGE_PIN BF27 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[12]   ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ12"    - IO_L3N_T0L_N5_AD15N_40
set_property -dict {PACKAGE_PIN BE27 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[13]   ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ13"    - IO_L3P_T0L_N4_AD15P_40
set_property -dict {PACKAGE_PIN BF30 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[14]   ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ14"    - IO_L2N_T0L_N3_40
set_property -dict {PACKAGE_PIN BF29 IOSTANDARD POD12_DCI      } [get_ports c0_ddr4_dq[15]   ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQ15"    - IO_L2P_T0L_N2_40
set_property -dict {PACKAGE_PIN BE26 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_c[3] ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQS_C10" - IO_L1N_T0L_N1_DBC_40
set_property -dict {PACKAGE_PIN BD26 IOSTANDARD DIFF_POD12_DCI } [get_ports c0_ddr4_dqs_t[3] ]; # Bank 40 VCCO - VCC1V2 Net "DDR4_C0_DQS_T10" - IO_L1P_T0L_N0_DBC_40

###
# DDR 1
###
# 250 MHZ clk
set_property	PACKAGE_PIN	AW19		        [get_ports 	c1_sys_clk_n] ; 
set_property	IOSTANDARD		LVDS	        [get_ports 	c1_sys_clk_n] ; 
set_property	PACKAGE_PIN	AW20		        [get_ports 	c1_sys_clk_p] ; 
set_property	IOSTANDARD		LVDS	        [get_ports 	c1_sys_clk_p] ; 

set_property -dict {PACKAGE_PIN AN13 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[24]   ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ24"     - IO_L23N_T3U_N9_67
set_property -dict {PACKAGE_PIN AM13 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[26]   ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ26"     - IO_L23P_T3U_N8_67
set_property -dict {PACKAGE_PIN AT13 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_c[6] ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQS_C3"   - IO_L22N_T3U_N7_DBC_AD0N_67
set_property -dict {PACKAGE_PIN AT14 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_t[6] ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQS_T3"   - IO_L22P_T3U_N6_DBC_AD0P_67
set_property -dict {PACKAGE_PIN AR13 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[25]   ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ25"     - IO_L24N_T3U_N11_67
set_property -dict {PACKAGE_PIN AP13 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[27]   ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ27"     - IO_L24P_T3U_N10_67
set_property -dict {PACKAGE_PIN AM14 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[28]   ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ28"     - IO_L21N_T3L_N5_AD8N_67
set_property -dict {PACKAGE_PIN AL14 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[30]   ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ30"     - IO_L21P_T3L_N4_AD8P_67
set_property -dict {PACKAGE_PIN AT15 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[31]   ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ31"     - IO_L20N_T3L_N3_AD1N_67
set_property -dict {PACKAGE_PIN AR15 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[29]   ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ29"     - IO_L20P_T3L_N2_AD1P_67
set_property -dict {PACKAGE_PIN AP14 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_c[7] ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQS_C12"  - IO_L19N_T3L_N1_DBC_AD9N_67
set_property -dict {PACKAGE_PIN AN14 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_t[7] ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQS_T12"  - IO_L19P_T3L_N0_DBC_AD9P_67
set_property -dict {PACKAGE_PIN AV13 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[9]    ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ9"      - IO_L17N_T2U_N9_AD10N_67
set_property -dict {PACKAGE_PIN AU13 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[8]    ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ8"      - IO_L17P_T2U_N8_AD10P_67
set_property -dict {PACKAGE_PIN AY15 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_c[2] ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQS_C1"   - IO_L16N_T2U_N7_QBC_AD3N_67
set_property -dict {PACKAGE_PIN AW15 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_t[2] ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQS_T1"   - IO_L16P_T2U_N6_QBC_AD3P_67
set_property -dict {PACKAGE_PIN AW13 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[10]   ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ10"     - IO_L18N_T2U_N11_AD2N_67
set_property -dict {PACKAGE_PIN AW14 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[11]   ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ11"     - IO_L18P_T2U_N10_AD2P_67
set_property -dict {PACKAGE_PIN AV14 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[14]   ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ14"     - IO_L15N_T2L_N5_AD11N_67
set_property -dict {PACKAGE_PIN AU14 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[12]   ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ12"     - IO_L15P_T2L_N4_AD11P_67
set_property -dict {PACKAGE_PIN BA11 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[15]   ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ15"     - IO_L14N_T2L_N3_GC_67
set_property -dict {PACKAGE_PIN AY11 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[13]   ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ13"     - IO_L14P_T2L_N2_GC_67
set_property -dict {PACKAGE_PIN AY12 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_c[3] ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQS_C10"  - IO_L13N_T2L_N1_GC_QBC_67
set_property -dict {PACKAGE_PIN AY13 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_t[3] ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQS_T10"  - IO_L13P_T2L_N0_GC_QBC_67
set_property -dict {PACKAGE_PIN BA13 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[18]   ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ18"     - IO_L11N_T1U_N9_GC_67
set_property -dict {PACKAGE_PIN BA14 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[19]   ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ19"     - IO_L11P_T1U_N8_GC_67
set_property -dict {PACKAGE_PIN BB10 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_c[4] ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQS_C2"   - IO_L10N_T1U_N7_QBC_AD4N_67
set_property -dict {PACKAGE_PIN BB11 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_t[4] ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQS_T2"   - IO_L10P_T1U_N6_QBC_AD4P_67
set_property -dict {PACKAGE_PIN BB12 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[17]   ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ17"     - IO_L12N_T1U_N11_GC_67
set_property -dict {PACKAGE_PIN BA12 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[16]   ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ16"     - IO_L12P_T1U_N10_GC_67
set_property -dict {PACKAGE_PIN BA7  IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[22]   ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ22"     - IO_L9N_T1L_N5_AD12N_67
set_property -dict {PACKAGE_PIN BA8  IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[23]   ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ23"     - IO_L9P_T1L_N4_AD12P_67
set_property -dict {PACKAGE_PIN BC9  IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[20]   ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ20"     - IO_L8N_T1L_N3_AD5N_67
set_property -dict {PACKAGE_PIN BB9  IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[21]   ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ21"     - IO_L8P_T1L_N2_AD5P_67
set_property -dict {PACKAGE_PIN BA9  IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_c[5] ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQS_C11"  - IO_L7N_T1L_N1_QBC_AD13N_67
set_property -dict {PACKAGE_PIN BA10 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_t[5] ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQS_T11"  - IO_L7P_T1L_N0_QBC_AD13P_67
set_property -dict {PACKAGE_PIN BD7  IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[1]    ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ1"      - IO_L5N_T0U_N9_AD14N_67
set_property -dict {PACKAGE_PIN BC7  IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[2]    ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ2"      - IO_L5P_T0U_N8_AD14P_67
set_property -dict {PACKAGE_PIN BF9  IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_c[0] ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQS_C0"   - IO_L4N_T0U_N7_DBC_AD7N_67
set_property -dict {PACKAGE_PIN BF10 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_t[0] ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQS_T0"   - IO_L4P_T0U_N6_DBC_AD7P_67
set_property -dict {PACKAGE_PIN BD8  IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[3]    ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ3"      - IO_L6N_T0U_N11_AD6N_67
set_property -dict {PACKAGE_PIN BD9  IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[0]    ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ0"      - IO_L6P_T0U_N10_AD6P_67
set_property -dict {PACKAGE_PIN BF7  IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[7]    ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ7"      - IO_L3N_T0L_N5_AD15N_67
set_property -dict {PACKAGE_PIN BE7  IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[6]    ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ6"      - IO_L3P_T0L_N4_AD15P_67
set_property -dict {PACKAGE_PIN BE10 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[5]    ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ5"      - IO_L2N_T0L_N3_67
set_property -dict {PACKAGE_PIN BD10 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[4]    ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQ4"      - IO_L2P_T0L_N2_67
set_property -dict {PACKAGE_PIN BF8  IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_c[1] ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQS_C9"   - IO_L1N_T0L_N1_DBC_67
set_property -dict {PACKAGE_PIN BE8  IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_t[1] ]; # Bank 67 VCCO - VCC1V2 Net "DDR4_C1_DQS_T9"   - IO_L1P_T0L_N0_DBC_67
set_property -dict {PACKAGE_PIN AM15 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[56]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ56"     - IO_L23N_T3U_N9_66
set_property -dict {PACKAGE_PIN AL15 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[57]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ57"     - IO_L23P_T3U_N8_66
set_property -dict {PACKAGE_PIN AR16 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_c[14]]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQS_C7"   - IO_L22N_T3U_N7_DBC_AD0N_66
set_property -dict {PACKAGE_PIN AP16 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_t[14]]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQS_T7"   - IO_L22P_T3U_N6_DBC_AD0P_66
#set_property -dict {PACKAGE_PIN AN18 IOSTANDARD LVCMOS12       } [get_ports c1_ddr4_event_n  ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_EVENT_B"  - IO_T3U_N12_66
set_property -dict {PACKAGE_PIN AN16 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[59]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ59"     - IO_L24N_T3U_N11_66
set_property -dict {PACKAGE_PIN AN17 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[58]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ58"     - IO_L24P_T3U_N10_66
set_property -dict {PACKAGE_PIN AL16 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[63]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ63"     - IO_L21N_T3L_N5_AD8N_66
set_property -dict {PACKAGE_PIN AL17 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[62]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ62"     - IO_L21P_T3L_N4_AD8P_66
set_property -dict {PACKAGE_PIN AR18 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[60]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ60"     - IO_L20N_T3L_N3_AD1N_66
set_property -dict {PACKAGE_PIN AP18 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[61]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ61"     - IO_L20P_T3L_N2_AD1P_66
set_property -dict {PACKAGE_PIN AM16 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_c[15]]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQS_C16"  - IO_L19N_T3L_N1_DBC_AD9N_66
set_property -dict {PACKAGE_PIN AM17 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_t[15]]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQS_T16"  - IO_L19P_T3L_N0_DBC_AD9P_66
set_property -dict {PACKAGE_PIN AU16 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[50]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ50"     - IO_L17N_T2U_N9_AD10N_66
set_property -dict {PACKAGE_PIN AU17 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[51]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ51"     - IO_L17P_T2U_N8_AD10P_66
set_property -dict {PACKAGE_PIN AW18 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_c[12]]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQS_C6"   - IO_L16N_T2U_N7_QBC_AD3N_66
set_property -dict {PACKAGE_PIN AV18 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_t[12]]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQS_T6"   - IO_L16P_T2U_N6_QBC_AD3P_66
set_property -dict {PACKAGE_PIN AR17 IOSTANDARD LVCMOS12       } [get_ports c1_ddr4_reset_n  ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_RESET_N"  - IO_T2U_N12_66
set_property -dict {PACKAGE_PIN AV16 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[48]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ48"     - IO_L18N_T2U_N11_AD2N_66
set_property -dict {PACKAGE_PIN AV17 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[49]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ49"     - IO_L18P_T2U_N10_AD2P_66
set_property -dict {PACKAGE_PIN AT17 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[55]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ55"     - IO_L15N_T2L_N5_AD11N_66
set_property -dict {PACKAGE_PIN AT18 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[54]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ54"     - IO_L15P_T2L_N4_AD11P_66
set_property -dict {PACKAGE_PIN BB16 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[53]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ53"     - IO_L14N_T2L_N3_GC_66
set_property -dict {PACKAGE_PIN BB17 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[52]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ52"     - IO_L14P_T2L_N2_GC_66
set_property -dict {PACKAGE_PIN AY16 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_c[13]]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQS_C15"  - IO_L13N_T2L_N1_GC_QBC_66
set_property -dict {PACKAGE_PIN AW16 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_t[13]]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQS_T15"  - IO_L13P_T2L_N0_GC_QBC_66
set_property -dict {PACKAGE_PIN AY17 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[40]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ40"     - IO_L11N_T1U_N9_GC_66
set_property -dict {PACKAGE_PIN AY18 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[42]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ42"     - IO_L11P_T1U_N8_GC_66
set_property -dict {PACKAGE_PIN BC12 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_c[10]]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQS_C5"   - IO_L10N_T1U_N7_QBC_AD4N_66
set_property -dict {PACKAGE_PIN BC13 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_t[10]]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQS_T5"   - IO_L10P_T1U_N6_QBC_AD4P_66
set_property -dict {PACKAGE_PIN BA17 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[41]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ41"     - IO_L12N_T1U_N11_GC_66
set_property -dict {PACKAGE_PIN BA18 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[43]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ43"     - IO_L12P_T1U_N10_GC_66
set_property -dict {PACKAGE_PIN BB15 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[45]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ45"     - IO_L9N_T1L_N5_AD12N_66
set_property -dict {PACKAGE_PIN BA15 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[44]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ44"     - IO_L9P_T1L_N4_AD12P_66
set_property -dict {PACKAGE_PIN BD11 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[47]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ47"     - IO_L8N_T1L_N3_AD5N_66
set_property -dict {PACKAGE_PIN BC11 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[46]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ46"     - IO_L8P_T1L_N2_AD5P_66
set_property -dict {PACKAGE_PIN BC14 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_c[11]]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQS_C14"  - IO_L7N_T1L_N1_QBC_AD13N_66
set_property -dict {PACKAGE_PIN BB14 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_t[11]]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQS_T14"  - IO_L7P_T1L_N0_QBC_AD13P_66
set_property -dict {PACKAGE_PIN BD13 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[35]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ35"     - IO_L5N_T0U_N9_AD14N_66
set_property -dict {PACKAGE_PIN BD14 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[33]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ33"     - IO_L5P_T0U_N8_AD14P_66
set_property -dict {PACKAGE_PIN BE11 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_c[8] ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQS_C4"   - IO_L4N_T0U_N7_DBC_AD7N_66
set_property -dict {PACKAGE_PIN BE12 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_t[8] ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQS_T4"   - IO_L4P_T0U_N6_DBC_AD7P_66
set_property -dict {PACKAGE_PIN BF12 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[34]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ34"     - IO_L6N_T0U_N11_AD6N_66
set_property -dict {PACKAGE_PIN BE13 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[32]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ32"     - IO_L6P_T0U_N10_AD6P_66
set_property -dict {PACKAGE_PIN BD15 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[36]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ36"     - IO_L3N_T0L_N5_AD15N_66
set_property -dict {PACKAGE_PIN BD16 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[37]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ37"     - IO_L3P_T0L_N4_AD15P_66
set_property -dict {PACKAGE_PIN BF13 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[39]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ39"     - IO_L2N_T0L_N3_66
set_property -dict {PACKAGE_PIN BF14 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[38]   ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQ38"     - IO_L2P_T0L_N2_66
set_property -dict {PACKAGE_PIN BF15 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_c[9] ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQS_C13"  - IO_L1N_T0L_N1_DBC_66
set_property -dict {PACKAGE_PIN BE15 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_t[9] ]; # Bank 66 VCCO - VCC1V2 Net "DDR4_C1_DQS_T13"  - IO_L1P_T0L_N0_DBC_66
set_property -dict {PACKAGE_PIN AM25 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_adr[15]  ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_ADR15"    - IO_L22N_T3U_N7_DBC_AD0N_D05_65
set_property -dict {PACKAGE_PIN AL25 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_adr[14]  ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_ADR14"    - IO_L22P_T3U_N6_DBC_AD0P_D04_65
set_property -dict {PACKAGE_PIN AP26 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_ba[1]    ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_BA1"      - IO_T3U_N12_PERSTN0_65
set_property -dict {PACKAGE_PIN AN26 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_adr[3]   ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_ADR3"     - IO_L24N_T3U_N11_DOUT_CSO_B_65
set_property -dict {PACKAGE_PIN AM26 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_adr[10]  ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_ADR10"    - IO_L24P_T3U_N10_EMCCLK_65
#set_property -dict {PACKAGE_PIN AP24 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_odt[1]   ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_ODT1"     - IO_L21N_T3L_N5_AD8N_D07_65
#set_property -dict {PACKAGE_PIN AP23 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_cs_n[3]  ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_CS_B3"    - IO_L21P_T3L_N4_AD8P_D06_65
#set_property -dict {PACKAGE_PIN AM24 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_adr[17]  ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_ADR17"    - IO_L20N_T3L_N3_AD1N_D09_65
set_property -dict {PACKAGE_PIN AL24 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_adr[13]  ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_ADR13"    - IO_L20P_T3L_N2_AD1P_D08_65
set_property -dict {PACKAGE_PIN AN24 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_adr[0]   ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_ADR0"     - IO_L19N_T3L_N1_DBC_AD9N_D11_65
set_property -dict {PACKAGE_PIN AN23 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_adr[16]  ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_ADR16"    - IO_L19P_T3L_N0_DBC_AD9P_D10_65
#set_property -dict {PACKAGE_PIN AV26 IOSTANDARD DIFF_SSTL12_DCI} [get_ports c1_ddr4_ck_c[1]  ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_CK_C1"    - IO_L17N_T2U_N9_AD10N_D15_65
#set_property -dict {PACKAGE_PIN AU26 IOSTANDARD DIFF_SSTL12_DCI} [get_ports c1_ddr4_ck_t[1]  ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_CK_T1"    - IO_L17P_T2U_N8_AD10P_D14_65
set_property -dict {PACKAGE_PIN AT23 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_parity   ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_PAR"      - IO_L16N_T2U_N7_QBC_AD3N_A01_D17_65
#set_property -dict {PACKAGE_PIN AR23 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_cs_n[2]  ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_CS_B2"    - IO_L16P_T2U_N6_QBC_AD3P_A00_D16_65
#set_property -dict {PACKAGE_PIN AP25 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_cs_n[1]  ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_CS_B1"    - IO_T2U_N12_CSI_ADV_B_65
set_property -dict {PACKAGE_PIN AU24 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_ba[0]    ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_BA0"      - IO_L18N_T2U_N11_AD2N_D13_65
set_property -dict {PACKAGE_PIN AT24 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_adr[1]   ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_ADR1"     - IO_L18P_T2U_N10_AD2P_D12_65
set_property -dict {PACKAGE_PIN AU25 IOSTANDARD DIFF_SSTL12_DCI} [get_ports c1_ddr4_ck_c[0]  ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_CK_C0"    - IO_L15N_T2L_N5_AD11N_A03_D19_65
set_property -dict {PACKAGE_PIN AT25 IOSTANDARD DIFF_SSTL12_DCI} [get_ports c1_ddr4_ck_t[0]  ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_CK_T0"    - IO_L15P_T2L_N4_AD11P_A02_D18_65
set_property -dict {PACKAGE_PIN AV24 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_adr[6]   ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_ADR6"     - IO_L14N_T2L_N3_GC_A05_D21_65
set_property -dict {PACKAGE_PIN AV23 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_cs_n[0]  ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_CS_B0"    - IO_L14P_T2L_N2_GC_A04_D20_65
set_property -dict {PACKAGE_PIN AW26 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_bg[1]    ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_BG1"      - IO_L13N_T2L_N1_GC_QBC_A07_D23_65
set_property -dict {PACKAGE_PIN AW25 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_act_n    ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_ACT_B"    - IO_L13P_T2L_N0_GC_QBC_A06_D22_65
set_property -dict {PACKAGE_PIN AY26 IOSTANDARD LVCMOS12       } [get_ports c1_ddr4_alert_n  ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_ALERT_B"  - IO_L11N_T1U_N9_GC_A11_D27_65
set_property -dict {PACKAGE_PIN AY25 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_adr[8]   ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_ADR8"     - IO_L11P_T1U_N8_GC_A10_D26_65
set_property -dict {PACKAGE_PIN AY23 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_adr[5]   ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_ADR5"     - IO_L10N_T1U_N7_QBC_AD4N_A13_D29_65
set_property -dict {PACKAGE_PIN AY22 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_adr[4]   ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_ADR4"     - IO_L10P_T1U_N6_QBC_AD4P_A12_D28_65
set_property -dict {PACKAGE_PIN BA25 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_adr[11]  ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_ADR11"    - IO_T1U_N12_SMBALERT_65
set_property -dict {PACKAGE_PIN AW24 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_adr[2]   ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_ADR2"     - IO_L12N_T1U_N11_GC_A09_D25_65
set_property -dict {PACKAGE_PIN AW23 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_odt[0]   ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_ODT0"     - IO_L12P_T1U_N10_GC_A08_D24_65
set_property -dict {PACKAGE_PIN BB25 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_cke[0]   ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_CKE0"     - IO_L9N_T1L_N5_AD12N_A15_D31_65
#set_property -dict {PACKAGE_PIN BB24 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_cke[1]   ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_CKE1"     - IO_L9P_T1L_N4_AD12P_A14_D30_65
set_property -dict {PACKAGE_PIN BA23 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_adr[9]   ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_ADR9"     - IO_L8N_T1L_N3_AD5N_A17_65
set_property -dict {PACKAGE_PIN BA22 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_adr[7]   ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_ADR7"     - IO_L8P_T1L_N2_AD5P_A16_65
set_property -dict {PACKAGE_PIN BC22 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_bg[0]    ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_BG0"      - IO_L7N_T1L_N1_QBC_AD13N_A19_65
set_property -dict {PACKAGE_PIN BB22 IOSTANDARD SSTL12_DCI     } [get_ports c1_ddr4_adr[12]  ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_ADR12"    - IO_L7P_T1L_N0_QBC_AD13P_A18_65
set_property -dict {PACKAGE_PIN BF25 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[64]   ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_DQ64"     - IO_L5N_T0U_N9_AD14N_A23_65
set_property -dict {PACKAGE_PIN BF24 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[65]   ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_DQ65"     - IO_L5P_T0U_N8_AD14P_A22_65
set_property -dict {PACKAGE_PIN BD24 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_c[16]]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_DQS_C8"   - IO_L4N_T0U_N7_DBC_AD7N_A25_65
set_property -dict {PACKAGE_PIN BC24 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_t[16]]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_DQS_T8"   - IO_L4P_T0U_N6_DBC_AD7P_A24_65
set_property -dict {PACKAGE_PIN BE25 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[67]   ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_DQ67"     - IO_L6N_T0U_N11_AD6N_A21_65
set_property -dict {PACKAGE_PIN BD25 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[66]   ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_DQ66"     - IO_L6P_T0U_N10_AD6P_A20_65
set_property -dict {PACKAGE_PIN BF23 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[70]   ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_DQ70"     - IO_L3N_T0L_N5_AD15N_A27_65
set_property -dict {PACKAGE_PIN BE23 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[71]   ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_DQ71"     - IO_L3P_T0L_N4_AD15P_A26_65
set_property -dict {PACKAGE_PIN BD23 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[68]   ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_DQ68"     - IO_L2N_T0L_N3_FWE_FCS2_B_65
set_property -dict {PACKAGE_PIN BC23 IOSTANDARD POD12_DCI      } [get_ports c1_ddr4_dq[69]   ]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_DQ69"     - IO_L2P_T0L_N2_FOE_B_65
set_property -dict {PACKAGE_PIN BF22 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_c[17]]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_DQS_C17"  - IO_L1N_T0L_N1_DBC_RS1_65
set_property -dict {PACKAGE_PIN BE22 IOSTANDARD DIFF_POD12_DCI } [get_ports c1_ddr4_dqs_t[17]]; # Bank 65 VCCO - VCC1V2 Net "DDR4_C1_DQS_T17"  - IO_L1P_T0L_N0_DBC_RS0_65

#########################################################
# PCIe
#########################################################

# set_property PACKAGE_PIN AM17 [get_ports perst_n]
# set_property IOSTANDARD LVCMOS18 [get_ports perst_n]
# set_property PULLUP true [get_ports perst_n]

set_property	PACKAGE_PIN	BD21		    [get_ports 	perst_n	] ; 
set_property	IOSTANDARD		LVCMOS12	[get_ports 	perst_n	] ; 

set_property	PACKAGE_PIN	AM10	        [get_ports 	{pcie_clk_n}] ; 
set_property	PACKAGE_PIN	AM11            [get_ports 	{pcie_clk_p}] ; 


#set_false_path -from [get_ports perst_n]



#set_property LOC GTHE2_CHANNEL_X1Y35 [get_cells {dma_inst/inst/pcie3_ip_i/inst/gt_top_i/pipe_wrapper_i/pipe_lane[0].gt_wrapper_i/gth_channel.gthe2_channel_i}]
set_property PACKAGE_PIN AF2 [get_ports {pcie_rx_p[0]}]
set_property PACKAGE_PIN AF1 [get_ports {pcie_rx_n[0]}]
set_property PACKAGE_PIN AF7  [get_ports {pcie_tx_p[0]}]
set_property PACKAGE_PIN AF6  [get_ports {pcie_tx_n[0]}]
#set_property LOC GTHE2_CHANNEL_X1Y34 [get_cells {dma_inst/inst/pcie3_ip_i/inst/gt_top_i/pipe_wrapper_i/pipe_lane[1].gt_wrapper_i/gth_channel.gthe2_channel_i}]
set_property PACKAGE_PIN AG4 [get_ports {pcie_rx_p[1]}]
set_property PACKAGE_PIN AG3 [get_ports {pcie_rx_n[1]}]
set_property PACKAGE_PIN AG9 [get_ports {pcie_tx_p[1]}]
set_property PACKAGE_PIN AG8 [get_ports {pcie_tx_n[1]}]
#set_property LOC GTHE2_CHANNEL_X1Y33 [get_cells {dma_inst/inst/pcie3_ip_i/inst/gt_top_i/pipe_wrapper_i/pipe_lane[2].gt_wrapper_i/gth_channel.gthe2_channel_i}]
set_property PACKAGE_PIN AH2 [get_ports {pcie_rx_p[2]}]
set_property PACKAGE_PIN AH1 [get_ports {pcie_rx_n[2]}]
set_property PACKAGE_PIN AH7 [get_ports {pcie_tx_p[2]}]
set_property PACKAGE_PIN AH6 [get_ports {pcie_tx_n[2]}]
#set_property LOC GTHE2_CHANNEL_X1Y32 [get_cells {dma_inst/inst/pcie3_ip_i/inst/gt_top_i/pipe_wrapper_i/pipe_lane[3].gt_wrapper_i/gth_channel.gthe2_channel_i}]
set_property PACKAGE_PIN AJ4 [get_ports {pcie_rx_p[3]}]
set_property PACKAGE_PIN AJ3 [get_ports {pcie_rx_n[3]}]
set_property PACKAGE_PIN AJ9 [get_ports {pcie_tx_p[3]}]
set_property PACKAGE_PIN AJ8 [get_ports {pcie_tx_n[3]}]
#set_property LOC GTHE2_CHANNEL_X1Y31 [get_cells {dma_inst/inst/pcie3_ip_i/inst/gt_top_i/pipe_wrapper_i/pipe_lane[4].gt_wrapper_i/gth_channel.gthe2_channel_i}]
set_property PACKAGE_PIN AK2 [get_ports {pcie_rx_p[4]}]
set_property PACKAGE_PIN AK1 [get_ports {pcie_rx_n[4]}]
set_property PACKAGE_PIN AK7 [get_ports {pcie_tx_p[4]}]
set_property PACKAGE_PIN AK6 [get_ports {pcie_tx_n[4]}]
#set_property LOC GTHE2_CHANNEL_X1Y30 [get_cells {dma_inst/inst/pcie3_ip_i/inst/gt_top_i/pipe_wrapper_i/pipe_lane[5].gt_wrapper_i/gth_channel.gthe2_channel_i}]
set_property PACKAGE_PIN AL4 [get_ports {pcie_rx_p[5]}]
set_property PACKAGE_PIN AL3 [get_ports {pcie_rx_n[5]}]
set_property PACKAGE_PIN AL9 [get_ports {pcie_tx_p[5]}]
set_property PACKAGE_PIN AL8 [get_ports {pcie_tx_n[5]}]
#set_property LOC GTHE2_CHANNEL_X1Y29 [get_cells {dma_inst/inst/pcie3_ip_i/inst/gt_top_i/pipe_wrapper_i/pipe_lane[6].gt_wrapper_i/gth_channel.gthe2_channel_i}]
set_property PACKAGE_PIN AM2 [get_ports {pcie_rx_p[6]}]
set_property PACKAGE_PIN AM1 [get_ports {pcie_rx_n[6]}]
set_property PACKAGE_PIN AM7 [get_ports {pcie_tx_p[6]}]
set_property PACKAGE_PIN AM6 [get_ports {pcie_tx_n[6]}]
#set_property LOC GTHE2_CHANNEL_X1Y28 [get_cells {dma_inst/inst/pcie3_ip_i/inst/gt_top_i/pipe_wrapper_i/pipe_lane[7].gt_wrapper_i/gth_channel.gthe2_channel_i}]
set_property PACKAGE_PIN AN4 [get_ports {pcie_rx_p[7]}]
set_property PACKAGE_PIN AN3 [get_ports {pcie_rx_n[7]}]
set_property PACKAGE_PIN AN9 [get_ports {pcie_tx_p[7]}]
set_property PACKAGE_PIN AN8 [get_ports {pcie_tx_n[7]}]


