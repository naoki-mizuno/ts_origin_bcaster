cmake_minimum_required(VERSION 2.8.3)
project(ts_origin_bcaster)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    geometry_msgs
)

add_message_files(
  FILES
  GroundRef.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

catkin_install_python(PROGRAMS
    nodes/ts_origin_bcaster_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
