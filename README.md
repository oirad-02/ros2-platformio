# ros2-platformi

This project demonstrates serial communication between ROS 2 and an ESP32 for measuring round-trip time (RTT). It includes two ROS 2 nodes – one written in C++ and one in Python – which communicate with the ESP32 over a serial interface.

## 🔧 Requirements

- ROS 2 Jazzy (or any compatible ROS 2 distribution)
- ESP32 (any model with USB-serial communication)
- Dependencies:

```bash
sudo apt install libserial-dev python3-serial
```

## 📦 Build Instructions

The project contains two ROS 2 packages:

    esp32_comm_cpp: C++ implementation

    esp32_comm_python: Python implementation

Build with Colcon

colcon build --packages-select esp32_comm_cpp esp32_comm_python
source install/setup.bash

## 🚀 Running the Nodes

C++ Node

```bash
ros2 run esp32_comm_cpp esp32_communicator --ros-args -p serial_port:=/dev/ttyUSB0
```

Python Node
```bash
ros2 run esp32_comm_python esp32_communicator --ros-args -p serial_port:=/dev/ttyUSB0
```

Adjust the serial_port parameter to match your system (e.g. /dev/ttyUSB0, /dev/ttyACM0, etc.)

## 📡 Functionality

Each node periodically sends a message to the ESP32, which immediately echoes it back. The time between sending and receiving the message is measured as the round-trip time (RTT). This allows for basic latency measurements and communication testing.

## 🧠 ESP32 Firmware

The ESP32 firmware is located in:

Common/ESP32_ROS2_Serial/

It includes:

    src/main.cpp: main firmware code

    platformio.ini: PlatformIO build configuration

    include/, lib/, test/: placeholders for headers, libraries, and test logic

Use PlatformIO to build and flash the firmware.

```bash
cd Common/ESP32_ROS2_Serial
pio run --target upload
```

## 📁 Project Structure

ros2-platformi/  
├── Common/  
│   └── ESP32_ROS2_Serial/  
│       ├── include/            # Header files (optional)  
│       ├── lib/                # Optional libraries  
│       ├── src/  
│       │   └── main.cpp        # ESP32 firmware  
│       ├── test/               # Unit tests  
│       └── platformio.ini      # PlatformIO config  
├── src/  
│   ├── esp32_comm_cpp/  
│   │   ├── CMakeLists.txt  
│   │   ├── package.xml  
│   │   ├── include/esp32_comm_cpp/  
│   │   └── src/esp32_comm.cpp  
│   └── esp32_comm_python/  
│       ├── package.xml  
│       ├── setup.py, setup.cfg  
│       ├── esp32_comm_python/  
│       │   ├── __init__.py  
│       │   └── esp32_comm_python.py  
│       └── test/  
│           ├── test_copyright.py  
│           ├── test_flake8.py  
│           └── test_pep257.py  
├── LICENSE  
└── README.md  

## 📌 TODO / Future Work

🔁 Refactor both ROS 2 nodes to use a shared interface for common serial logic (e.g. base class or shared library)

🔧 Add launch files and parameter YAML support

✅ Add unit tests for communication and RTT measurement logic

📚 Extend documentation with usage examples and ESP32 config

## 👨‍💻 Author

 OIRAD_02

## 📄 License

This project is licensed under the MIT License – see [LICENSE](LICENSE) for details.
