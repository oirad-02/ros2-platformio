# ros2-platformi

```cpp
# Install dependencies:
sudo apt install libserial-dev python3-serial

# Compile:
colcon build --packages-select esp32_comm_cpp esp32_comm_python

# If successful:
source install/setup.bash

# start

# C++ Version
ros2 run esp32_comm_cpp esp32_communicator --ros-args -p serial_port:=/dev/ttyUSB0 

# Python Version
ros2 run esp32_comm_python esp32_communicator --ros-args -p serial_port:=/dev/ttyUSB0 
```
