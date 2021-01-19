# Author:  David Sidler (david.sidler@inf.ethz.ch)

cmake_minimum_required(VERSION 3.0)

set (PROJECT_NAME mcrouter)
project(${PROJECT_NAME})

# Include custom Find<Module>.cmake scripts to enable searching for Vivado HLS
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake)

# Without this variable set, CMake will build tests when running install
#set(CMAKE_SKIP_INSTALL_ALL_DEPENDENCY ON)

# Generate Doxygen if available
#find_package(Doxygen)
#if(Doxygen_FOUND)
#  configure_file(${CMAKE_SOURCE_DIR}/Doxyfile.in Doxyfile)
#  add_custom_target(doxygen ALL
#      COMMAND ${DOXYGEN_EXECUTABLE} Doxyfile 
#      WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
#endif()


# @yang, using 100G U250
set(FPGA_PART xcu250-figd2104-2L-e CACHE STRING "FPGA part")
set(DATA_WIDTH 64 CACHE STRING "Width of data path in bytes")
set(CLOCK_PERIOD 6.206 CACHE STRING "Target clock period in nanoseconds")

# TCP parameters
set(TCP_STACK_MAX_SESSIONS 1000 CACHE STRING "Maximum number of sessions the TCP/IP stack can support")
set(TCP_STACK_MSS 1460 CACHE STRING "Maximum Segment Size (MSS)")
set(TCP_STACK_NODELAY_EN 1 CACHE BOOL "Enabling TCP_NODELAY option")
# set(TCP_STACK_RX_DDR_BYPASS_EN 1 CACHE BOOL "Enabling DDR bypass on the RX path")
set(TCP_STACK_RX_DDR_BYPASS_EN 0 CACHE BOOL "Enabling DDR bypass on the RX path")
set(TCP_STACK_FAST_RETRANSMIT_EN 1 CACHE BOOL "Enabling TCP fast retransmit")
set(TCP_STACK_WINDOW_SCALING_EN 1 CACHE BOOL "Enabling TCP window scaling option")

# Find Xilinx Vivado HLS
find_package(VivadoHLS REQUIRED)
if (NOT VIVADO_HLS_FOUND)
  message(FATAL_ERROR "Vivado HLS not found.")
endif()

# Installation
if (DEFINED ENV{IPREPO_DIR})
   set(IPREPO_DIR $ENV{IPREPO_DIR})
elseif(NOT  IPREPO_DIR)
   set(IPREPO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/iprepo/)
endif()

include_directories(${CMAKE_CURRENT_BINARY_DIR})

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../toe/toe_config.hpp.in toe_config.hpp)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/make.tcl.in make.tcl)


set(EXAMPLE_HLS_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/mcrouter.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/mcrouter.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../toe/toe_config.hpp.in
    ${CMAKE_CURRENT_SOURCE_DIR}/hash_table_16_1024/hash_table_16_1024.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/hash_table_16_1024/hash_table_16_1024.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/parser_stateman/parser_stateman.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/parser_stateman/parser_stateman.hpp
#    ${CMAKE_CURRENT_SOURCE_DIR}/parser/parser.cpp 
#    ${CMAKE_CURRENT_SOURCE_DIR}/parser/parser.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/parser/parser_v1_9cycles.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/parser/parser_v1_9cycles.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/hash_table_32_32/hash_table_32_32.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/hash_table_32_32/hash_table_32_32.hpp)
#    ${CMAKE_CURRENT_SOURCE_DIR}/multi_queue/multi_queue.cpp 
#    ${CMAKE_CURRENT_SOURCE_DIR}/multi_queue/multi_queue.hpp)
#    ${CMAKE_CURRENT_SOURCE_DIR}/mcrouter_config.hpp.in)


#Setup HLS custom targets
set(HLS_TARGETS synthesis csim ip installip)

foreach (target ${HLS_TARGETS})
   if (NOT TARGET ${target})
      add_custom_target(${target})
   endif()

   add_custom_target(${target}.${PROJECT_NAME}
      COMMAND ${VIVADO_HLS_BINARY} -f make.tcl -tclargs ${target}
      DEPENDS ${EXAMPLE_HLS_DEPENDS})
   add_dependencies(${target} ${target}.${PROJECT_NAME})
endforeach()

#target dependencies
add_dependencies(ip.${PROJECT_NAME} synthesis.${PROJECT_NAME})
add_dependencies(installip.${PROJECT_NAME} ip.${PROJECT_NAME})
                       
