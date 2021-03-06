# !/bin/bash

export IPREPO_DIR=/home/yangz/davos/iprepo

cd build
# rm CMakeCache.txt
# cmake .. -DDATA_WIDTH=64 -DCLOCK_PERIOD=3.1 \
#     -DFPGA_PART=xcu250-figd2104-2L-e \
#     -DFPGA_FAMILY=ultraplus -DDEVICE_NAME=vcu118 \
#     -DTCP_STACK_EN=1 \
#     -DVIVADO_ROOT_DIR=/opt/xilinx/Vivado/2019.1/bin \
#     -DVIVADO_HLS_ROOT_DIR=/opt/xilinx/Vivado/2019.1/bin

# make installip
# sleep 10s
make project
sleep 10s
make synthesize
sleep 10s
make implementation
sleep 10s
make bitstream
