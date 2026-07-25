cmake_minimum_required(VERSION 3.0.2)
project(morai_bringup)

find_package(catkin REQUIRED)

catkin_package()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_launch_composition.py)
endif()

install(
  DIRECTORY config docs launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
