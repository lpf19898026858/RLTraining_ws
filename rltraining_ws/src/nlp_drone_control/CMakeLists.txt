cmake_minimum_required(VERSION 3.0.2)
project(nlp_drone_control)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找所有需要的 catkin 包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  vlm_service
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation
)

add_service_files(
   FILES
   ExecuteDroneAction.srv
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
)
# ==============================================================================
#  头文件包含路径
# ==============================================================================
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ${PROJECT_SOURCE_DIR}/lib/json/include
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

# 定义 llm_processor_node 可执行文件
add_executable(llm_processor_node
  src/llm_processor_node.cpp
)

# 将 llm_processor_node 链接到它需要的库
target_link_libraries(llm_processor_node
    ${catkin_LIBRARIES}
    cpr::cpr
)

