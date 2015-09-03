cmake_minimum_required(VERSION 2.8.3)
project(roboteq)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    nav_msgs
    roscpp
    serial
    serial_utils
    tf
)

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES roboteq
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp serial serial_utils tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(hroboteq src/roboteq.cc)
target_link_libraries(roboteq ${catkin_LIBRARIES})

install(TARGETS roboteq
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
