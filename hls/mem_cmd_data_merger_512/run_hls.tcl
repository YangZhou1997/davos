
open_project mem_cmd_data_merger_512_prj

set_top mem_cmd_data_merger_512

add_files mem_cmd_data_merger_512.hpp
add_files mem_cmd_data_merger_512.cpp


add_files -tb test_mem_cmd_data_merger_512.cpp

open_solution "solution1"
set_part {xc7vx690tffg1761-3}
create_clock -period 6.4 -name default

csynth_design
export_design -format ip_catalog -display_name "Mem cmd & data merger (512bit)" -description "" -vendor "ethz.systems.fpga" -version "0.2"

exit
