cmake_minimum_required(VERSION 2.8.3)
project(motion)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs  
  sensor_msgs
  geometry_msgs
  dynamixel_sdk
  dynamixel_workbench_toolbox
  robotcontrol
)

set(PROJECT_DEPS
  roscpp
  robotcontrol
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs dynamixel_sdk dynamixel_workbench_toolbox

  INCLUDE_DIRS include
  LIBRARIES serial robotcontrol

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(servo_control src/servo_control.cpp)
#target_link_libraries(servo_control ${catkin_LIBRARIES})
#add_dependencies(servo_control motion_generate_messages_cpp)

add_executable(motion_control src/motion_control.cpp)
target_link_libraries(motion_control ${catkin_LIBRARIES})
add_dependencies(motion_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(motion_dummy src/motion_dummy.cpp)
target_link_libraries(motion_dummy ${catkin_LIBRARIES})
add_dependencies(motion_dummy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS motion_control motion_dummy
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(dynamixel_controller src/dynamixel_controller.cpp)
add_dependencies(dynamixel_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_controller ${catkin_LIBRARIES})

install(TARGETS dynamixel_controller
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
  FILES_MATCHING PATTERN "*"
)
