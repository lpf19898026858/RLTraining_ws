cmake_minimum_required(VERSION 3.5)
project(ui_control)
add_compile_options(-std=c++11)

##############################################################################
# Find Packages
##############################################################################
find_package(catkin REQUIRED COMPONENTS 
  roscpp nodelet sensor_msgs cv_bridge)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)
find_package(OpenCV REQUIRED)

##############################################################################
# Catkin Package Information
##############################################################################
catkin_package(INCLUDE_DIRS include)

##############################################################################
# Qt Configuration & Code Generation
##############################################################################

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# --- 关键修改：告诉 AUTOUIC 新的UI文件搜索路径 ---
# 现在我们告诉它要去 src/ 和 ui/ 这两个目录里寻找 .ui 文件
list(APPEND CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/src)
list(APPEND CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

# --- 定义文件列表，路径必须与您的文件结构完全匹配 ---
set(QT_HEADER_FILES
  src/log_in.h
  src/v_uav_local_network.h
  include/ui_control/main_window.hpp
  include/ui_control/qnode.hpp
)

set(QT_UI_FILES
  src/log_in.ui
  src/v_uav_local_network.ui
  ui/main_window.ui # <<< 已修正为正确的 ui/ 目录
)

set(SOURCE_FILES
  src/log_in.cpp
  src/main.cpp
  src/main_window.cpp
  src/qnode.cpp
  src/v_uav_local_network.cpp
)

##############################################################################
# Includes, Binaries, and Linking
##############################################################################

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

add_executable(${PROJECT_NAME}_node
  ${SOURCE_FILES}
  ${QT_HEADER_FILES} 
  ${QT_UI_FILES}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Qt5::Widgets
)

install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
