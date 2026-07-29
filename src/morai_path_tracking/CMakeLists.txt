cmake_minimum_required(VERSION 3.0.2)
project(morai_path_tracking)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  morai_udp_bridge
  nav_msgs
  roscpp
  tf2
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES morai_pure_pursuit morai_longitudinal_pid
  CATKIN_DEPENDS geometry_msgs morai_udp_bridge nav_msgs roscpp tf2 tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(morai_pure_pursuit
  src/pure_pursuit.cpp
)
target_compile_features(morai_pure_pursuit PUBLIC cxx_std_14)

add_library(morai_longitudinal_pid
  src/pid_controller.cpp
)
target_compile_features(morai_longitudinal_pid PUBLIC cxx_std_14)

add_executable(pure_pursuit_controller_node
  src/pure_pursuit_controller_node.cpp
)
target_compile_features(pure_pursuit_controller_node PUBLIC cxx_std_14)
target_link_libraries(pure_pursuit_controller_node
  morai_pure_pursuit
  morai_longitudinal_pid
  ${catkin_LIBRARIES}
)
add_dependencies(pure_pursuit_controller_node ${catkin_EXPORTED_TARGETS})

install(
  TARGETS morai_pure_pursuit morai_longitudinal_pid pure_pursuit_controller_node
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

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_pure_pursuit test/test_pure_pursuit.cpp)
  if(TARGET test_pure_pursuit)
    target_compile_features(test_pure_pursuit PUBLIC cxx_std_14)
    target_link_libraries(test_pure_pursuit morai_pure_pursuit)
  endif()

  catkin_add_gtest(test_pid_controller test/test_pid_controller.cpp)
  if(TARGET test_pid_controller)
    target_compile_features(test_pid_controller PUBLIC cxx_std_14)
    target_link_libraries(test_pid_controller morai_longitudinal_pid)
  endif()

  find_package(rostest REQUIRED)
  add_rostest(test/pure_pursuit_controller.test
    DEPENDENCIES pure_pursuit_controller_node
  )
  catkin_install_python(
    PROGRAMS test/test_pure_pursuit_controller.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()
