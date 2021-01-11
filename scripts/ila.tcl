# NETWORK STACK

create_ip -name ila -vendor xilinx.com -library ip -version 6.2 -module_name ila_mixed -dir $device_ip_dir
set_property -dict [list \
    CONFIG.C_NUM_OF_PROBES {16} \
    CONFIG.C_PROBE0_WIDTH {1} \
    CONFIG.C_PROBE1_WIDTH {1} \
    CONFIG.C_PROBE2_WIDTH {1} \
    CONFIG.C_PROBE3_WIDTH {1} \
    CONFIG.C_PROBE4_WIDTH {1} \
    CONFIG.C_PROBE5_WIDTH {1} \
    CONFIG.C_PROBE6_WIDTH {1} \
    CONFIG.C_PROBE7_WIDTH {1} \
    CONFIG.C_PROBE8_WIDTH {52} \
    CONFIG.C_PROBE9_WIDTH {32} \
    CONFIG.C_PROBE10_WIDTH {32} \
    CONFIG.C_PROBE11_WIDTH {32} \
    CONFIG.C_PROBE12_WIDTH {32} \
    CONFIG.C_PROBE13_WIDTH {20} \
    CONFIG.C_PROBE14_WIDTH {7} \
    CONFIG.C_PROBE15_WIDTH {6} 
] [get_ips ila_mixed]
generate_target {instantiation_template} [get_files $device_ip_dir/ila_mixed/ila_mixed.xci]
update_compile_order -fileset sources_1

create_ip -name ila -vendor xilinx.com -library ip -version 6.2 -module_name ila_mixed2 -dir $device_ip_dir
set_property -dict [list \
    CONFIG.C_NUM_OF_PROBES {16} \
    CONFIG.C_PROBE0_WIDTH {1} \
    CONFIG.C_PROBE1_WIDTH {1} \
    CONFIG.C_PROBE2_WIDTH {1} \
    CONFIG.C_PROBE3_WIDTH {1} \
    CONFIG.C_PROBE4_WIDTH {1} \
    CONFIG.C_PROBE5_WIDTH {1} \
    CONFIG.C_PROBE6_WIDTH {1} \
    CONFIG.C_PROBE7_WIDTH {1} \
    CONFIG.C_PROBE8_WIDTH {52} \
    CONFIG.C_PROBE9_WIDTH {32} \
    CONFIG.C_PROBE10_WIDTH {32} \
    CONFIG.C_PROBE11_WIDTH {32} \
    CONFIG.C_PROBE12_WIDTH {32} \
    CONFIG.C_PROBE13_WIDTH {32} \
    CONFIG.C_PROBE14_WIDTH {16} \
    CONFIG.C_PROBE15_WIDTH {16} 
] [get_ips ila_mixed2]
generate_target {instantiation_template} [get_files $device_ip_dir/ila_mixed2/ila_mixed2.xci]
update_compile_order -fileset sources_1

create_ip -name ila -vendor xilinx.com -library ip -version 6.2 -module_name ila_32_mixed -dir $device_ip_dir
set_property -dict [list \
    CONFIG.C_NUM_OF_PROBES {32} \
    CONFIG.C_PROBE0_WIDTH {1} \
    CONFIG.C_PROBE1_WIDTH {1} \
    CONFIG.C_PROBE2_WIDTH {1} \
    CONFIG.C_PROBE3_WIDTH {1} \
    CONFIG.C_PROBE4_WIDTH {1} \
    CONFIG.C_PROBE5_WIDTH {1} \
    CONFIG.C_PROBE6_WIDTH {1} \
    CONFIG.C_PROBE7_WIDTH {1} \
    CONFIG.C_PROBE8_WIDTH {1} \
    CONFIG.C_PROBE9_WIDTH {1} \
    CONFIG.C_PROBE10_WIDTH {96} \
    CONFIG.C_PROBE11_WIDTH {1} \
    CONFIG.C_PROBE12_WIDTH {1} \
    CONFIG.C_PROBE13_WIDTH {1} \
    CONFIG.C_PROBE14_WIDTH {1} \
    CONFIG.C_PROBE15_WIDTH {1} \
    CONFIG.C_PROBE16_WIDTH {16} \
    CONFIG.C_PROBE17_WIDTH {64} \
    CONFIG.C_PROBE18_WIDTH {16} \
    CONFIG.C_PROBE19_WIDTH {16} \
    CONFIG.C_PROBE20_WIDTH {16} \
    CONFIG.C_PROBE21_WIDTH {16} \
    CONFIG.C_PROBE22_WIDTH {16} \
    CONFIG.C_PROBE23_WIDTH {16} \
    CONFIG.C_PROBE24_WIDTH {16} \
    CONFIG.C_PROBE25_WIDTH {32} \
    CONFIG.C_PROBE26_WIDTH {16} \
    CONFIG.C_PROBE27_WIDTH {16} \
    CONFIG.C_PROBE28_WIDTH {16} \
    CONFIG.C_PROBE29_WIDTH {16} \
    CONFIG.C_PROBE30_WIDTH {16} \
    CONFIG.C_PROBE31_WIDTH {16} 
] [get_ips ila_32_mixed]
generate_target {instantiation_template} [get_files $device_ip_dir/ila_32_mixed/ila_32_mixed.xci]
update_compile_order -fileset sources_1

create_ip -name ila -vendor xilinx.com -library ip -version 6.2 -module_name ila_mixed3 -dir $device_ip_dir
set_property -dict [list \
    CONFIG.C_NUM_OF_PROBES {16} \
    CONFIG.C_PROBE0_WIDTH {32} \
    CONFIG.C_PROBE1_WIDTH {32} \
    CONFIG.C_PROBE2_WIDTH {32} \
    CONFIG.C_PROBE3_WIDTH {32} \
    CONFIG.C_PROBE4_WIDTH {32} \
    CONFIG.C_PROBE5_WIDTH {32} \
    CONFIG.C_PROBE6_WIDTH {32} \
    CONFIG.C_PROBE7_WIDTH {32} \
    CONFIG.C_PROBE8_WIDTH {32} \
    CONFIG.C_PROBE9_WIDTH {32} \
    CONFIG.C_PROBE10_WIDTH {32} \
    CONFIG.C_PROBE11_WIDTH {32} \
    CONFIG.C_PROBE12_WIDTH {32} \
    CONFIG.C_PROBE13_WIDTH {32} \
    CONFIG.C_PROBE14_WIDTH {32} \
    CONFIG.C_PROBE15_WIDTH {32} 
] [get_ips ila_mixed3]
generate_target {instantiation_template} [get_files $device_ip_dir/ila_mixed3/ila_mixed3.xci]
update_compile_order -fileset sources_1