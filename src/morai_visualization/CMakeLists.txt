cmake_minimum_required(VERSION 3.0.2)
project(morai_visualization)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(path_visualizer_node
  src/path_visualizer_node.cpp
)
target_compile_features(path_visualizer_node PUBLIC cxx_std_14)
target_link_libraries(path_visualizer_node
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_visualization_assets.py)
  add_rostest(test/path_visualizer.test)
endif()

install(
  TARGETS path_visualizer_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
