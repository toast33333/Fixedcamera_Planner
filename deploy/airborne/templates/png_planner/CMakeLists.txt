cmake_minimum_required(VERSION 3.0.2)
project(png_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  detection_msgs
  geometry_msgs
  kcf_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
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
  detection_msgs geometry_msgs kcf_msgs roscpp sensor_msgs std_msgs tf
)

include_directories(
  src
  ${GEOGRAPHICLIB_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(fixed_camera_ibvs src/fixed_camera_ibvs.cpp)
add_dependencies(fixed_camera_ibvs ${catkin_EXPORTED_TARGETS})
target_link_libraries(fixed_camera_ibvs ${catkin_LIBRARIES})

add_executable(radar_approach_handover src/radar_approach_handover.cpp)
add_dependencies(radar_approach_handover ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  radar_approach_handover ${catkin_LIBRARIES} ${GEOGRAPHICLIB_LIBRARY}
)

install(TARGETS fixed_camera_ibvs radar_approach_handover
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
