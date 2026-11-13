cmake_minimum_required(VERSION 3.0.2)
project(png_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  detection_msgs
  geometry_msgs
  kcf_msgs
  mavros_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2
)

find_path(GEOGRAPHICLIB_INCLUDE_DIR
  NAMES GeographicLib/LocalCartesian.hpp
  PATHS /usr/include /usr/local/include
)
find_library(GEOGRAPHICLIB_LIBRARY
  NAMES Geographic GeographicLib
  PATHS /usr/lib /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/local/lib
)
if(NOT GEOGRAPHICLIB_INCLUDE_DIR OR NOT GEOGRAPHICLIB_LIBRARY)
  message(FATAL_ERROR "GeographicLib not found; install libgeographic-dev")
endif()

catkin_package(CATKIN_DEPENDS
  detection_msgs geometry_msgs kcf_msgs mavros_msgs nav_msgs roscpp sensor_msgs
  std_msgs std_srvs tf tf2
)

include_directories(
  src
  ${GEOGRAPHICLIB_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(atlink_vendor STATIC
  vendor/atlink/src/frame.c
  vendor/atlink/src/crc16.c
  vendor/atlink/src/payload.c
  vendor/atlink/src/session.c
  vendor/atlink/src/bridge.c
  vendor/atlink/src/telem.c
)
target_include_directories(atlink_vendor PUBLIC
  vendor/atlink/include
  vendor/atlink/src
)
target_compile_definitions(atlink_vendor PRIVATE ATLINK_BUILD)
set_target_properties(atlink_vendor PROPERTIES
  C_STANDARD 11
  C_STANDARD_REQUIRED ON
  POSITION_INDEPENDENT_CODE ON
)

add_executable(fixed_camera_ibvs src/fixed_camera_ibvs.cpp)
add_dependencies(fixed_camera_ibvs ${catkin_EXPORTED_TARGETS})
target_link_libraries(fixed_camera_ibvs ${catkin_LIBRARIES})

add_executable(radar_approach_handover src/radar_approach_handover.cpp)
add_dependencies(radar_approach_handover ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  radar_approach_handover ${catkin_LIBRARIES} ${GEOGRAPHICLIB_LIBRARY}
)

add_executable(camera_pitch_servo_open_loop
  src/camera_pitch_servo_open_loop.cpp)
add_dependencies(camera_pitch_servo_open_loop ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_pitch_servo_open_loop ${catkin_LIBRARIES})

add_executable(camera_pitch_servo_sysfs_pwm
  src/camera_pitch_servo_sysfs_pwm.cpp)
add_dependencies(camera_pitch_servo_sysfs_pwm ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_pitch_servo_sysfs_pwm ${catkin_LIBRARIES})

add_executable(circle_trajectory_test src/circle_trajectory_test.cpp)
add_dependencies(circle_trajectory_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(circle_trajectory_test ${catkin_LIBRARIES})

add_executable(atlink_ros_node src/atlink_ros_node.cpp)
add_dependencies(atlink_ros_node ${catkin_EXPORTED_TARGETS})
target_include_directories(atlink_ros_node PRIVATE vendor/atlink/include)
target_link_libraries(atlink_ros_node atlink_vendor ${catkin_LIBRARIES})

install(TARGETS fixed_camera_ibvs radar_approach_handover
  camera_pitch_servo_open_loop camera_pitch_servo_sysfs_pwm
  circle_trajectory_test atlink_ros_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
