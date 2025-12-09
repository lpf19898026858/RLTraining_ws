cmake_minimum_required(VERSION 3.0.2)
project(map_generator)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
)
find_package(yaml-cpp REQUIRED)

# catkin_package() 应该在这里！
# 它必须在 find_package(catkin) 之后，
# 并且在 add_executable/add_library 之前。
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(yaml_to_map_publisher_node src/yaml_to_map_publisher.cpp)

target_link_libraries(yaml_to_map_publisher_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
