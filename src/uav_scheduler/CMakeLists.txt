cmake_minimum_required(VERSION 3.0.2)
project(uav_scheduler)

## 编译选项
set(CMAKE_CXX_STANDARD 14)

## 依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

## srv 文件
add_service_files(
  FILES
  ComputeAssignment.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 调度算法库
add_library(hungarian_solver
  src/hungarian_solver.cpp
)
target_link_libraries(hungarian_solver ${catkin_LIBRARIES})

## 服务节点
add_executable(scheduler_node src/scheduler_node.cpp)
add_dependencies(scheduler_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scheduler_node
  hungarian_solver
  ${catkin_LIBRARIES}
)

