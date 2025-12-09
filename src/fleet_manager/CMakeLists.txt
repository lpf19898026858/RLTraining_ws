# ROS包的CMake最低版本要求
cmake_minimum_required(VERSION 3.0.2)
# ROS包的名称
project(fleet_manager)

## 查找编译所需的Catkin包和系统依赖
## add_compile_options(-std=c++11) # 如果需要，可取消注释以启用C++11/14/17
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  # [重要] 添加对 nlp_drone_control 包的依赖，因为我们使用了它的消息
  nlp_drone_control 
  uav_scheduler
  # [重要] 添加消息/服务生成器
  message_generation
)

################################################
## 添加自定义消息和服务的生成规则
################################################

## (1) 列出你所有的 .msg 文件
add_message_files(
  FILES
  DroneStatus.msg
  TaskStatus.msg
)

## (2) 列出你所有的 .srv 文件
add_service_files(
  FILES
  QueryFleetAndTasks.srv
  InjectTasks.srv
)

## (3) 生成所有消息和服务的代码
generate_messages(
  # [重要] 确保依赖了所有用到的消息包
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nlp_drone_control 
  uav_scheduler
)

################################################
## Catkin包元信息
################################################

catkin_package(
#  INCLUDE_DIRS include # 如果你有单独的头文件目录，取消此行注释
#  LIBRARIES ${PROJECT_NAME} # 如果你编译的是一个库，取消此行注释
  CATKIN_DEPENDS 
    roscpp 
    std_msgs
    geometry_msgs
    nlp_drone_control
    message_runtime # [重要] 运行时依赖
#  DEPENDS system_lib
)

################################################
## 编译可执行文件（你的节点）
################################################

# 添加 include 目录，通常用于存放 .h 头文件
include_directories(
  # 如果你的头文件和源文件都在 src/ 目录下，可以省略
  # include 
  ${catkin_INCLUDE_DIRS}
)

## (1) 定义你的节点可执行文件
## 节点名称: fleet_manager_node
## 源文件:   src/fleet_manager_node.cpp
add_executable(fleet_manager_node src/fleet_manager_node.cpp)

## (2) 添加生成的消息/服务作为编译依赖
## 这确保了在编译你的节点之前，消息/服务的代码已经被生成
add_dependencies(fleet_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

## (3) 链接你的节点需要的库
target_link_libraries(fleet_manager_node
  ${catkin_LIBRARIES}
)

################################################
## (可选) 安装规则
################################################

# 如果你想将你的节点、launch文件等安装到ROS环境中，可以添加安装规则
# install(TARGETS fleet_manager_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )
