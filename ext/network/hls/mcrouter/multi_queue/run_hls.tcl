
open_project multi_queue_prj

set_top multi_queue

add_files multi_queue.hpp
add_files multi_queue.cpp


add_files -tb test_multi_queue.cpp

open_solution "solution1"
set_part {xcu250-figd2104-2L-e}
create_clock -period 3.103 -name default


csim_design
csynth_design
export_design -format ip_catalog -display_name "Multi Queue Data Structure" -description "" -vendor "ethz.systems.fpga" -version "0.1"

exit

# vivado_hls -f make.tcl