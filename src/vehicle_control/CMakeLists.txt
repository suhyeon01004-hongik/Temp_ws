cmake_minimum_required(VERSION 3.0.2)
project(vehicle_control)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
    VehicleCommand.msg
    VehicleStatus.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vehicle_control_core
  CATKIN_DEPENDS message_runtime nav_msgs roscpp sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(vehicle_control_core
  src/button_edge.cpp
  src/command_watchdog.cpp
  src/gear_selector.cpp
  src/joy_mapper.cpp
  src/morai_ctrl_packet.cpp
  src/morai_vehicle_status_packet.cpp
  src/reset_command.cpp
  src/udp_receiver.cpp
  src/udp_sender.cpp
)
target_compile_features(vehicle_control_core PUBLIC cxx_std_14)
target_link_libraries(vehicle_control_core
  ${catkin_LIBRARIES}
)

add_executable(joystick_teleop_node
  src/joystick_teleop_node.cpp
)
target_compile_features(joystick_teleop_node PUBLIC cxx_std_14)
target_link_libraries(joystick_teleop_node
  vehicle_control_core
  ${catkin_LIBRARIES}
)
add_dependencies(joystick_teleop_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(morai_udp_sender_node
  src/morai_udp_sender_node.cpp
)
target_compile_features(morai_udp_sender_node PUBLIC cxx_std_14)
target_link_libraries(morai_udp_sender_node
  vehicle_control_core
  ${catkin_LIBRARIES}
)
add_dependencies(morai_udp_sender_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(morai_vehicle_status_udp_node
  src/morai_vehicle_status_udp_node.cpp
)
target_compile_features(
  morai_vehicle_status_udp_node PUBLIC cxx_std_14
)
target_link_libraries(morai_vehicle_status_udp_node
  vehicle_control_core
  ${catkin_LIBRARIES}
)
add_dependencies(morai_vehicle_status_udp_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(morai_sim_reset_node
  src/morai_sim_reset_node.cpp
)
target_compile_features(morai_sim_reset_node PUBLIC cxx_std_14)
target_link_libraries(morai_sim_reset_node
  vehicle_control_core
  ${catkin_LIBRARIES}
)
add_dependencies(morai_sim_reset_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS
    vehicle_control_core
    joystick_teleop_node
    morai_udp_sender_node
    morai_vehicle_status_udp_node
    morai_sim_reset_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest(test/cyvox_mapping.test)
  add_rostest(test/speed_source_fallback.test)
  add_rostest(test/udp_sender_reset_pause.test)

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

  catkin_add_gtest(test_gear_selector
    test/test_gear_selector.cpp
  )
  if(TARGET test_gear_selector)
    target_compile_features(test_gear_selector PUBLIC cxx_std_14)
    target_link_libraries(test_gear_selector
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

  catkin_add_gtest(test_morai_vehicle_status_packet
    test/test_morai_vehicle_status_packet.cpp
  )
  if(TARGET test_morai_vehicle_status_packet)
    target_compile_features(
      test_morai_vehicle_status_packet PUBLIC cxx_std_14
    )
    target_link_libraries(test_morai_vehicle_status_packet
      vehicle_control_core
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_udp_receiver
    test/test_udp_receiver.cpp
  )
  if(TARGET test_udp_receiver)
    target_compile_features(test_udp_receiver PUBLIC cxx_std_14)
    target_link_libraries(test_udp_receiver
      vehicle_control_core
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_button_edge
    test/test_button_edge.cpp
  )
  if(TARGET test_button_edge)
    target_compile_features(test_button_edge PUBLIC cxx_std_14)
    target_link_libraries(test_button_edge
      vehicle_control_core
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_reset_command
    test/test_reset_command.cpp
  )
  if(TARGET test_reset_command)
    target_compile_features(test_reset_command PUBLIC cxx_std_14)
    target_link_libraries(test_reset_command
      vehicle_control_core
      ${catkin_LIBRARIES}
    )
  endif()
endif()
