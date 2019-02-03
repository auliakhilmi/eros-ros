cmake_minimum_required(VERSION 2.8.3)
project(servocontroller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamixel_controllers roscpp rospy std_msgs sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES servocontroller
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(servo_publisher src/servo_publisher.cpp)
target_link_libraries(servo_publisher ${catkin_LIBRARIES})
install(TARGETS servo_publisher
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