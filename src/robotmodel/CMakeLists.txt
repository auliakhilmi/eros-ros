cmake_minimum_required(VERSION 2.8.3)
project(robotmodel)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  robot_state_publisher
  roscpp
  rospy
  std_msgs  
  sensor_msgs
  tf
  urdf
  xacro  
  dynamixel_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(state_publisher src/state_publisher.cpp)
target_link_libraries(state_publisher ${catkin_LIBRARIES})
add_dependencies(state_publisher robotmodel_generate_messages_cpp)

add_executable(model_publisher src/model_publisher.cpp)
target_link_libraries(model_publisher ${catkin_LIBRARIES})
add_dependencies(model_publisher robotmodel_generate_messages_cpp)

install(TARGETS state_publisher model_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  FILES_MATCHING PATTERN "*"
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  FILES_MATCHING PATTERN "*"
)
