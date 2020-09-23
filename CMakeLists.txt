cmake_minimum_required(VERSION 3.0.2)
project(rrt)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  tf
)

################################################
## Declare ROS messages, services and actions ##
################################################
# add_message_files(
#   FILES
# )

generate_messages(
  DEPENDENCIES
  nav_msgs
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rrt
  CATKIN_DEPENDS message_runtime nav_msgs roscpp rospy std_msgs geometry_msgs tf
  DEPENDS system_lib 
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(rrt src/rrt.cpp)

# Declaring executables that have to be built
# add_executable(server src/mapserver.cpp)
# target_link_libraries(server ${catkin_LIBRARIES})
# add_dependencies(server rrt_generate_messages_cpp)

add_executable(planner src/planner.cpp)
target_link_libraries(planner ${catkin_LIBRARIES} rrt)
add_dependencies(planner rrt_generate_messages_cpp)

target_link_libraries(rrt ${catkin_LIBRARIES})

#############
## Install ##
#############
install(
  TARGETS rrt
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(
  DIRECTORY include/rrt/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
