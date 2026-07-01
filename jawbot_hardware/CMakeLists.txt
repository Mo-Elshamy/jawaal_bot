cmake_minimum_required(VERSION 3.8)
project(jawbot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# 2. BUILD THE SHARED LIBRARY
add_library(jawbot_hardware SHARED
  src/serial_comms.cpp
  src/jawbot_hardware_interface.cpp
)
target_include_directories(jawbot_hardware PRIVATE include)

# 3. LINK THE DEPENDENCIES TO THE LIBRARY
ament_target_dependencies(jawbot_hardware
  rclcpp
  hardware_interface
  pluginlib
)

# 4. EXPORT AND INSTALL
pluginlib_export_plugin_description_file(hardware_interface jawbot_hardware.xml)

install(TARGETS
  jawbot_hardware
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)
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
