# mcrouter

This project is developed using Vivado HLS 2019.1

### Preparing project
```
mkdir build && cd build
cmake .. -DVIVADO_ROOT_DIR=/opt/xilinx/Vivado/2019.1/bin -DVIVADO_HLS_ROOT_DIR=/opt/xilinx/Vivado/2019.1/bin
```

### Run c-simulation
```
make csim
```

### Run c-synthesis
```
make synthesis
```

### Check II, resource usage, clock by opening project in Vivado HLS 

