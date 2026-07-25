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
  INCLUDE_DIRS include
  LIBRARIES vehicle_control_core
  CATKIN_DEPENDS message_runtime roscpp sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(vehicle_control_core
  src/command_watchdog.cpp
  src/joy_mapper.cpp
  src/morai_ctrl_packet.cpp
  src/udp_sender.cpp
)
target_compile_features(vehicle_control_core PUBLIC cxx_std_14)
target_link_libraries(vehicle_control_core
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_joy_mapper
    test/test_joy_mapper.cpp
  )
  if(TARGET test_joy_mapper)
    target_compile_features(test_joy_mapper PUBLIC cxx_std_14)
    target_link_libraries(test_joy_mapper
      vehicle_control_core
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_command_watchdog
    test/test_command_watchdog.cpp
  )
  if(TARGET test_command_watchdog)
    target_compile_features(test_command_watchdog PUBLIC cxx_std_14)
    target_link_libraries(test_command_watchdog
      vehicle_control_core
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_morai_ctrl_packet
    test/test_morai_ctrl_packet.cpp
  )
  if(TARGET test_morai_ctrl_packet)
    target_compile_features(test_morai_ctrl_packet PUBLIC cxx_std_14)
    target_link_libraries(test_morai_ctrl_packet
      vehicle_control_core
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_udp_sender
    test/test_udp_sender.cpp
  )
  if(TARGET test_udp_sender)
    target_compile_features(test_udp_sender PUBLIC cxx_std_14)
    target_link_libraries(test_udp_sender
      vehicle_control_core
      ${catkin_LIBRARIES}
    )
  endif()
endif()
