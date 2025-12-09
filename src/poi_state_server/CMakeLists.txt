cmake_minimum_required(VERSION 3.0.2)
project(poi_state_server)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
  roscpp
  rospy
  std_msgs
)
find_package(yaml-cpp REQUIRED)

add_service_files(
   FILES
   GetPOIInfo.srv
   ListPOIs.srv
)
add_message_files(
  FILES
  NamedPoint.msg
  POIInfo.msg
)
# 生成消息和服务的依赖
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


add_executable(poi_state_server_node
  src/poi_state_server_node.cpp
)
target_link_libraries(poi_state_server_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES poi_state_server
#  CATKIN_DEPENDS message_generation nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## 确保消息/服务先生成
add_dependencies(poi_state_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
