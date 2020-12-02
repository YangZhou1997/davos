#DMA
create_ip -name xdma -vendor xilinx.com -library ip -version 4.1 -module_name xdma_ip -dir $device_ip_dir
set_property -dict [list CONFIG.Component_Name {xdma_ip} CONFIG.pcie_blk_locn {X0Y1} CONFIG.pl_link_cap_max_link_width {X16} CONFIG.pl_link_cap_max_link_speed {8.0_GT/s} CONFIG.axi_data_width {512_bit} CONFIG.axisten_freq {250} CONFIG.pf0_device_id {903F} CONFIG.pf0_base_class_menu {Memory_controller} CONFIG.pf0_class_code_base {05} CONFIG.pf0_sub_class_interface_menu {Other_memory_controller} CONFIG.pf0_class_code_sub {80} CONFIG.pf0_class_code_interface {00} CONFIG.pf0_class_code {058000} CONFIG.axilite_master_en {true} CONFIG.xdma_rnum_rids {64} CONFIG.xdma_wnum_rids {32} CONFIG.plltype {QPLL1} CONFIG.xdma_axi_intf_mm {AXI_Stream} CONFIG.xdma_pcie_64bit_en {true} CONFIG.dsc_bypass_rd {0001} CONFIG.dsc_bypass_wr {0001} CONFIG.xdma_sts_ports {true} CONFIG.pf0_msix_cap_table_bir {BAR_3:2} CONFIG.pf0_msix_cap_pba_bir {BAR_3:2} CONFIG.cfg_mgmt_if {false} CONFIG.PF0_DEVICE_ID_mqdma {903F} CONFIG.PF2_DEVICE_ID_mqdma {903F} CONFIG.PF3_DEVICE_ID_mqdma {903F} CONFIG.axist_bypass_en {true}] [get_ips xdma_ip]
# set_property -dict \[ list \
#     CONFIG.axi_bypass_64bit_en {true} \
#     CONFIG.axi_bypass_prefetchable {true} \
#     CONFIG.axi_data_width {512_bit} \
#     CONFIG.axi_id_width {4} \
#     CONFIG.axist_bypass_en {true} \
#     CONFIG.axist_bypass_scale {Gigabytes} \
#     CONFIG.axist_bypass_size {1} \
#     CONFIG.axisten_freq {250} \
#     CONFIG.cfg_mgmt_if {false} \
#     CONFIG.dsc_bypass_rd {0001} \
#     CONFIG.dsc_bypass_wr {0001} \
#     CONFIG.pciebar2axibar_axil_master {0x00000000} \
#     CONFIG.pf0_msi_cap_multimsgcap {32_vectors} \
#     CONFIG.pf0_msix_cap_pba_offset {00008FE0} \
#     CONFIG.pf0_msix_cap_table_offset {00008000} \
#     CONFIG.pf0_msix_cap_table_size {01F} \
#     CONFIG.pf0_msix_enabled {true} \
#     CONFIG.pl_link_cap_max_link_speed {8.0_GT/s} \
#     CONFIG.pl_link_cap_max_link_width {X16} \
#     CONFIG.xdma_axi_intf_mm {AXI_Stream} \
#     CONFIG.xdma_num_usr_irq {1} \
#     CONFIG.xdma_rnum_chnl {1} \
#     CONFIG.xdma_sts_ports {true} \
#     CONFIG.xdma_wnum_chnl {1} \
#     CONFIG.xdma_wnum_rids {16} \
#     CONFIG.xdma_rnum_rids {16} \
# ]  [get_ips xdma_ip]
generate_target {instantiation_template} [get_files $device_ip_dir/xdma_ip/xdma_ip.xci]
update_compile_order -fileset sources_1



#HLS DMA IPs
create_ip -name tlb -vendor ethz.systems.fpga -library hls -version 0.09 -module_name tlb_ip -dir $device_ip_dir
generate_target {instantiation_template} [get_files $device_ip_dir/tlb_ip/tlb_ip.xci]
update_compile_order -fileset sources_1

create_ip -name mem_write_cmd_page_boundary_check_512 -vendor ethz.systems.fpga -library hls -version 0.3 -module_name mem_write_cmd_page_boundary_check_512_ip -dir $device_ip_dir
generate_target {instantiation_template} [get_files $device_ip_dir/mem_write_cmd_page_boundary_check_512_ip/mem_write_cmd_page_boundary_check_512_ip.xci]
update_compile_order -fileset sources_1
