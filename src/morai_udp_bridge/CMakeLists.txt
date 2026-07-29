cmake_minimum_required(VERSION 3.0.2)
project(morai_udp_bridge)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)
find_package(Threads REQUIRED)

add_message_files(FILES ActuatorCommand.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES morai_udp_bridge_protocol morai_udp_bridge_runtime
            morai_udp_bridge_control_protocol morai_udp_bridge_udp_sender
  CATKIN_DEPENDS diagnostic_msgs message_runtime roscpp sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(morai_udp_bridge_protocol
  src/protocol.cpp
)
target_compile_features(morai_udp_bridge_protocol PUBLIC cxx_std_14)

add_library(morai_udp_bridge_runtime
  src/streams.cpp
  src/transport.cpp
)
target_compile_features(morai_udp_bridge_runtime PUBLIC cxx_std_14)
target_link_libraries(morai_udp_bridge_runtime
  morai_udp_bridge_protocol
  ${catkin_LIBRARIES}
  Threads::Threads
)
add_dependencies(morai_udp_bridge_runtime ${catkin_EXPORTED_TARGETS})

add_library(morai_udp_bridge_control_protocol
  src/control_protocol.cpp
)
target_compile_features(morai_udp_bridge_control_protocol PUBLIC cxx_std_14)

add_library(morai_udp_bridge_udp_sender
  src/udp_sender.cpp
)
target_compile_features(morai_udp_bridge_udp_sender PUBLIC cxx_std_14)

add_executable(morai_udp_bridge_node
  src/bridge_node.cpp
)
target_compile_features(morai_udp_bridge_node PUBLIC cxx_std_14)
target_link_libraries(morai_udp_bridge_node
  morai_udp_bridge_runtime
  morai_udp_bridge_protocol
  ${catkin_LIBRARIES}
  Threads::Threads
)
add_dependencies(morai_udp_bridge_node ${catkin_EXPORTED_TARGETS})

install(
  TARGETS
    morai_udp_bridge_protocol
    morai_udp_bridge_runtime
    morai_udp_bridge_control_protocol
    morai_udp_bridge_udp_sender
    morai_udp_bridge_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY config docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_protocols_cpp test/test_protocols_cpp.cpp)
  if(TARGET test_protocols_cpp)
    target_compile_features(test_protocols_cpp PUBLIC cxx_std_14)
    target_link_libraries(test_protocols_cpp morai_udp_bridge_protocol)
  endif()

  catkin_add_gtest(test_control_protocol test/test_control_protocol.cpp)
  if(TARGET test_control_protocol)
    target_compile_features(test_control_protocol PUBLIC cxx_std_14)
    target_link_libraries(test_control_protocol morai_udp_bridge_control_protocol)
  endif()

  # CMake test targets are global across this workspace; vehicle_control
  # already owns test_udp_sender.
  catkin_add_gtest(test_morai_udp_bridge_udp_sender test/test_udp_sender.cpp)
  if(TARGET test_morai_udp_bridge_udp_sender)
    target_compile_features(test_morai_udp_bridge_udp_sender PUBLIC cxx_std_14)
    target_link_libraries(test_morai_udp_bridge_udp_sender morai_udp_bridge_udp_sender)
    add_custom_target(run_tests_morai_udp_bridge_gtest_test_udp_sender
      DEPENDS run_tests_morai_udp_bridge_gtest_test_morai_udp_bridge_udp_sender)
  endif()
endif()
