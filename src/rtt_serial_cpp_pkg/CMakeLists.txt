cmake_minimum_required(VERSION 3.8)
project(rtt_serial_cpp_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_path(LIBSERIAL_INCLUDE_DIR
  NAMES libserial/SerialStream.h
  PATHS /usr/include /usr/local/include
)

if(NOT LIBSERIAL_INCLUDE_DIR)
  message(FATAL_ERROR "LibSerial include directory not found!")
endif()

include_directories(${LIBSERIAL_INCLUDE_DIR})

add_executable(rtt_serial_node src/rtt_serial_node.cpp)

ament_target_dependencies(rtt_serial_node rclcpp)

target_link_libraries(rtt_serial_node serial)

install(TARGETS
  rtt_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
