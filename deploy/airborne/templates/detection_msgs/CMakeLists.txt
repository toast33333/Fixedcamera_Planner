cmake_minimum_required(VERSION 3.0.2)
project(detection_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)

add_message_files(FILES Detection.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime rospy std_msgs)

catkin_install_python(PROGRAMS
  scripts/rknnpool.py
  scripts/ros1Func.py
  scripts/ros1Main_aegis_stream.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rknnModel
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
