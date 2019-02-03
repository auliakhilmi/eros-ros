cmake_minimum_required(VERSION 2.8.3)
project(robotcontrol)

add_compile_options(-std=c++11)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake) # For finding custom .cmake files
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files( FILES Razor.msg Trajectory.msg Motion.msg)

generate_messages(
  DEPENDENCIES std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robotcontrol
)

add_library(robotcontrol
  src/robotcontrol.cpp
)
target_link_libraries(robotcontrol
  ${catkin_LIBRARIES}
)
add_dependencies(robotcontrol robotcontrol_gencpp)

install(TARGETS robotcontrol
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

add_executable(razor_stream src/razor_stream.cpp)
target_link_libraries(razor_stream ${catkin_LIBRARIES})
add_dependencies(razor_stream ${catkin_EXPORTED_TARGETS})

install(TARGETS razor_stream
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)
