cmake_minimum_required(VERSION 2.8.3)
project(tmr_driver)

add_definitions(-DUSE_BOOST)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  trajectory_msgs
  tf
  tmr_msgs
  tmrlib
  hardware_interface
  controller_manager
)

#set(BOOST_FLAG TRUE)
#if(BOOST_FLAG)
#add_definitions( -DUSE_BOOST )
#find_package(Boost REQUIRED COMPONENTS system thread chrono)
#endif()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    actionlib
    control_msgs
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs
    trajectory_msgs
    tf
    tmr_msgs
    tmrlib
    hardware_interface
    controller_manager
)

###########
## Build ##
###########

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_FLAGS "-std=c++14")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support.")
endif()

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

# Hardware Interface
#add_library(tmrobothw
#  src/tmr_ros_robot_hw.cpp
#)
#target_link_libraries(tmrobothw
#  ${catkin_LIBRARIES}
#  tmrdriver
#)

## Add cmake target dependencies of the library
# add_dependencies(tmr_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(tmr_driver
  src/tmr_ros_node.cpp
  src/tmr_ros_traj_as.cpp
  src/tmr_ros_svr_pub.cpp
  src/tmr_ros_svr_srv.cpp
  src/tmr_ros_sct_pub.cpp
  src/tmr_ros_sct_srv.cpp
  src/tmr_ros_robot_hw.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(tmr_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(tmr_driver
  ${catkin_LIBRARIES}
  #tmrdriver
  #tmrobothw
)

#############
## Install ##
#############

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Mark executables and/or libraries for installation
#install(TARGETS tmdriver #tmrobothw
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
install(TARGETS tmr_driver
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############

