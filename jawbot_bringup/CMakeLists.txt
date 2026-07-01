cmake_minimum_required(VERSION 3.8)
project(jawbot_bringup)

find_package(ament_cmake REQUIRED)

# This is the critical block you were missing!
# It copies your launch and config folders to the ROS 2 install directory.
install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()