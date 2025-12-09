cmake_minimum_required(VERSION 3.0.2)
project(nlp_drone_control)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找所有需要的 catkin 包
find_package( OpenCV REQUIRED )
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  vlm_service
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation
  poi_state_server   # 加上这一行
   cv_bridge
   uav_scheduler
)

add_service_files(
   FILES
   ExecuteDroneAction.srv
)
add_message_files(
  FILES
    Action.msg
    Plan.msg
)
# 生成消息和服务的依赖
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# ==============================================================================
#  catkin 包配置 (通常在消息/服务生成之后)
# ==============================================================================
catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    vlm_service
    tf
    message_runtime 
    poi_state_server   # 加上这一行
)
# ==============================================================================
#  头文件包含路径
# ==============================================================================
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ${PROJECT_SOURCE_DIR}/lib/json/include
  ${OpenCV_INCLUDE_DIRS} # 包含OpenCV的头文件目录
)


# ==============================================================================
#  编译第三方库 (CPR)
# ==============================================================================
option(CPR_USE_SYSTEM_CURL "Use system curl" ON)
set(CPR_INSTALL OFF CACHE BOOL "Disable installation of CPR")
option(CPR_BUILD_TESTS "Build cpr tests" OFF)
option(CPR_GENERATE_COVERAGE "Generate coverage reports" OFF)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/cpr)

# ==============================================================================
#  构建最终的可执行节点 (后定义可执行文件)
# ==============================================================================

# --- Planner Node ---
add_executable(llm_planner_node
  src/llm_planner_node.cpp
  src/llm_planner_main.cpp
)
target_link_libraries(llm_planner_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  cpr::cpr
)

# --- Executor Node ---
add_executable(task_executor_node
  src/task_executor_node.cpp
  src/task_executor_main.cpp
)
target_link_libraries(task_executor_node
  ${catkin_LIBRARIES}
  cpr::cpr
)

add_dependencies(task_executor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
