cmake_minimum_required(VERSION 3.8)
project(jawbot_description)

# Enable compiler warnings for clean code tracking
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Core dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  urdf
  meshes
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Testing configuration (Keep this clean boilerplate)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()