cmake_minimum_required(VERSION 3.8)
project(esp32_comm_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBSERIAL REQUIRED libserial)

add_executable(esp32_communicator src/esp32_comm.cpp)
ament_target_dependencies(esp32_communicator rclcpp std_msgs)

target_link_libraries(esp32_communicator ${LIBSERIAL_LIBRARIES})
target_include_directories(esp32_communicator PRIVATE ${LIBSERIAL_INCLUDE_DIRS})
target_compile_options(esp32_communicator PRIVATE ${LIBSERIAL_CFLAGS_OTHER})

install(TARGETS
  esp32_communicator
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
