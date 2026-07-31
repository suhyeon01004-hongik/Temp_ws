cmake_minimum_required(VERSION 3.0.2)
project(morai_path_tracking)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  message_generation
  morai_udp_bridge
  nav_msgs
  roscpp
  std_msgs
  tf2
  tf2_geometry_msgs
)

add_message_files(FILES ControllerStatus.msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES morai_pure_pursuit morai_longitudinal_pid morai_control_timing
            morai_curvature_speed_planner morai_stanley_controller
            morai_imm_two_model_filter morai_hybrid_controller
  CATKIN_DEPENDS geometry_msgs message_filters message_runtime
                 morai_udp_bridge nav_msgs roscpp std_msgs tf2
                 tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(morai_pure_pursuit
  src/controllers/lateral/pure_pursuit.cpp
)
target_compile_features(morai_pure_pursuit PUBLIC cxx_std_14)

add_library(morai_longitudinal_pid
  src/controllers/longitudinal/pid_controller.cpp
)
target_compile_features(morai_longitudinal_pid PUBLIC cxx_std_14)

add_library(morai_control_timing
  src/common/control_timing_bounds.cpp
)
target_compile_features(morai_control_timing PUBLIC cxx_std_14)

add_library(morai_curvature_speed_planner
  src/planning/curvature_speed_planner.cpp
)
target_compile_features(morai_curvature_speed_planner PUBLIC cxx_std_14)

add_library(morai_stanley_controller
  src/controllers/lateral/stanley_controller.cpp
)
target_compile_features(morai_stanley_controller PUBLIC cxx_std_14)

add_library(morai_imm_two_model_filter
  src/controllers/lateral/imm_two_model_filter.cpp
)
target_compile_features(morai_imm_two_model_filter PUBLIC cxx_std_14)

add_library(morai_hybrid_controller
  src/controllers/lateral/hybrid_controller.cpp
)
target_compile_features(morai_hybrid_controller PUBLIC cxx_std_14)
target_link_libraries(morai_hybrid_controller
  morai_pure_pursuit
  morai_stanley_controller
  morai_imm_two_model_filter
)

add_executable(path_tracking_controller_node
  src/nodes/path_tracking_controller_node.cpp
)
target_compile_features(path_tracking_controller_node PUBLIC cxx_std_14)
target_link_libraries(path_tracking_controller_node
  morai_pure_pursuit
  morai_longitudinal_pid
  morai_control_timing
  morai_curvature_speed_planner
  morai_stanley_controller
  morai_hybrid_controller
  ${catkin_LIBRARIES}
)
add_dependencies(path_tracking_controller_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS morai_pure_pursuit morai_longitudinal_pid morai_control_timing
          morai_curvature_speed_planner morai_stanley_controller
          morai_imm_two_model_filter morai_hybrid_controller
          path_tracking_controller_node
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

catkin_install_python(
  PROGRAMS scripts/analyze_tracking_bag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_config_contract.py)
  catkin_add_nosetests(test/test_tracking_bag_analysis.py)

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

  catkin_add_gtest(test_stanley_controller
    test/test_stanley_controller.cpp
  )
  if(TARGET test_stanley_controller)
    target_compile_features(test_stanley_controller PUBLIC cxx_std_14)
    target_link_libraries(test_stanley_controller morai_stanley_controller)
  endif()

  catkin_add_gtest(test_imm_two_model_filter
    test/test_imm_two_model_filter.cpp
  )
  if(TARGET test_imm_two_model_filter)
    target_compile_features(test_imm_two_model_filter PUBLIC cxx_std_14)
    target_link_libraries(test_imm_two_model_filter
      morai_imm_two_model_filter
    )
  endif()

  catkin_add_gtest(test_hybrid_controller
    test/test_hybrid_controller.cpp
  )
  if(TARGET test_hybrid_controller)
    target_compile_features(test_hybrid_controller PUBLIC cxx_std_14)
    target_link_libraries(test_hybrid_controller
      morai_hybrid_controller
    )
  endif()

  catkin_add_gtest(test_curvature_speed_planner
    test/test_curvature_speed_planner.cpp
  )
  if(TARGET test_curvature_speed_planner)
    target_compile_features(test_curvature_speed_planner PUBLIC cxx_std_14)
    target_link_libraries(test_curvature_speed_planner
      morai_curvature_speed_planner
    )
  endif()

  catkin_add_gtest(test_control_timing_bounds
    test/test_control_timing_bounds.cpp
  )
  if(TARGET test_control_timing_bounds)
    target_compile_features(test_control_timing_bounds PUBLIC cxx_std_14)
    target_link_libraries(test_control_timing_bounds morai_control_timing)
  endif()

  catkin_add_gtest(test_input_synchronization
    test/test_input_synchronization.cpp
  )
  if(TARGET test_input_synchronization)
    target_compile_features(test_input_synchronization PUBLIC cxx_std_14)
    target_link_libraries(test_input_synchronization ${catkin_LIBRARIES})
  endif()

  find_package(rostest REQUIRED)
  add_rostest(test/pure_pursuit_controller.test
    DEPENDENCIES path_tracking_controller_node
  )
  add_rostest(test/stanley_controller.test
    DEPENDENCIES path_tracking_controller_node
  )
  add_rostest(test/hybrid_controller.test
    DEPENDENCIES path_tracking_controller_node
  )
  add_rostest(test/localization_generation_safety.test
    DEPENDENCIES path_tracking_controller_node localization_fusion_node
                 route_path_publisher_node
  )
  catkin_install_python(
    PROGRAMS
      test/test_pure_pursuit_controller.py
      test/test_stanley_controller_node.py
      test/test_hybrid_controller_node.py
      test/test_localization_generation_safety.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()
