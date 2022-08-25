cmake_minimum_required(VERSION 3.0.2)
project(screen_demo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED 
  geometry_msgs
  roscpp
  rospy
  serial
  std_msgs
  sensor_msgs
  nav_msgs
)

find_package(Boost REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES scree_demo
 CATKIN_DEPENDS geometry_msgs roscpp rospy serial std_msgs
 DEPENDS system_lib
 DEPENDS Boost 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)



add_executable(screen_demo src/serial_port_demo.cpp)
target_link_libraries(screen_demo ${catkin_LIBRARIES})
add_dependencies(screen_demo ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(serial_port_demo_write_read src/serial_port_demo_write_read.cpp)
target_link_libraries(serial_port_demo_write_read ${catkin_LIBRARIES})
add_dependencies(serial_port_demo_write_read ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

#ts_ros_commu
# add_executable(ts_ros_commu src/ts_ros_commu.cpp)
# target_link_libraries(ts_ros_commu ${catkin_LIBRARIES})
# add_dependencies(ts_ros_commu ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )

#声明 头文件
add_library(ts_ros_commu_library
include/scree_demo/ts_ros_commu.h
src/ts_ros_commu.cpp
)   
add_dependencies(ts_ros_commu_library ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ts_ros_commu_library ${catkin_LIBRARIES})


#ts_ros_commu_node
add_executable(ts_ros_commu_node src/ts_ros_commu_node.cpp )
target_link_libraries(ts_ros_commu_node ts_ros_commu_library ${catkin_LIBRARIES})
add_dependencies(ts_ros_commu_node 
${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS} 
)
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/serial_port_demo.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

install(TARGETS screen_demo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
