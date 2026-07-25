cmake_minimum_required(VERSION 3.0.2)
project(vehicle_control)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
    VehicleCommand.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
