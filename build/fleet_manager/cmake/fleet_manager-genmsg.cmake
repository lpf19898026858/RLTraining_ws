# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "fleet_manager: 2 messages, 2 services")

set(MSG_I_FLAGS "-Ifleet_manager:/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Inlp_drone_control:/home/lpf/docker_shared/rltraining_ws/src/nlp_drone_control/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(fleet_manager_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/DroneStatus.msg" NAME_WE)
add_custom_target(_fleet_manager_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fleet_manager" "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/DroneStatus.msg" "geometry_msgs/Quaternion:geometry_msgs/Pose:geometry_msgs/Point"
)

get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/TaskStatus.msg" NAME_WE)
add_custom_target(_fleet_manager_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fleet_manager" "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/TaskStatus.msg" "nlp_drone_control/Action"
)

get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/srv/QueryFleetAndTasks.srv" NAME_WE)
add_custom_target(_fleet_manager_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fleet_manager" "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/srv/QueryFleetAndTasks.srv" "geometry_msgs/Quaternion:fleet_manager/TaskStatus:nlp_drone_control/Action:geometry_msgs/Pose:geometry_msgs/Point:fleet_manager/DroneStatus"
)

get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/srv/InjectTasks.srv" NAME_WE)
add_custom_target(_fleet_manager_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fleet_manager" "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/srv/InjectTasks.srv" "nlp_drone_control/Action:fleet_manager/TaskStatus"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(fleet_manager
  "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/DroneStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fleet_manager
)
_generate_msg_cpp(fleet_manager
  "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/TaskStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/lpf/docker_shared/rltraining_ws/src/nlp_drone_control/msg/Action.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fleet_manager
)

### Generating Services
_generate_srv_cpp(fleet_manager
  "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/srv/QueryFleetAndTasks.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/TaskStatus.msg;/home/lpf/docker_shared/rltraining_ws/src/nlp_drone_control/msg/Action.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/DroneStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fleet_manager
)
_generate_srv_cpp(fleet_manager
  "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/srv/InjectTasks.srv"
  "${MSG_I_FLAGS}"
  "/home/lpf/docker_shared/rltraining_ws/src/nlp_drone_control/msg/Action.msg;/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/TaskStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fleet_manager
)

### Generating Module File
_generate_module_cpp(fleet_manager
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fleet_manager
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(fleet_manager_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(fleet_manager_generate_messages fleet_manager_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/DroneStatus.msg" NAME_WE)
add_dependencies(fleet_manager_generate_messages_cpp _fleet_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/TaskStatus.msg" NAME_WE)
add_dependencies(fleet_manager_generate_messages_cpp _fleet_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/srv/QueryFleetAndTasks.srv" NAME_WE)
add_dependencies(fleet_manager_generate_messages_cpp _fleet_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/srv/InjectTasks.srv" NAME_WE)
add_dependencies(fleet_manager_generate_messages_cpp _fleet_manager_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fleet_manager_gencpp)
add_dependencies(fleet_manager_gencpp fleet_manager_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fleet_manager_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(fleet_manager
  "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/DroneStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fleet_manager
)
_generate_msg_eus(fleet_manager
  "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/TaskStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/lpf/docker_shared/rltraining_ws/src/nlp_drone_control/msg/Action.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fleet_manager
)

### Generating Services
_generate_srv_eus(fleet_manager
  "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/srv/QueryFleetAndTasks.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/TaskStatus.msg;/home/lpf/docker_shared/rltraining_ws/src/nlp_drone_control/msg/Action.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/DroneStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fleet_manager
)
_generate_srv_eus(fleet_manager
  "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/srv/InjectTasks.srv"
  "${MSG_I_FLAGS}"
  "/home/lpf/docker_shared/rltraining_ws/src/nlp_drone_control/msg/Action.msg;/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/TaskStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fleet_manager
)

### Generating Module File
_generate_module_eus(fleet_manager
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fleet_manager
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(fleet_manager_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(fleet_manager_generate_messages fleet_manager_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/DroneStatus.msg" NAME_WE)
add_dependencies(fleet_manager_generate_messages_eus _fleet_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/TaskStatus.msg" NAME_WE)
add_dependencies(fleet_manager_generate_messages_eus _fleet_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/srv/QueryFleetAndTasks.srv" NAME_WE)
add_dependencies(fleet_manager_generate_messages_eus _fleet_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/srv/InjectTasks.srv" NAME_WE)
add_dependencies(fleet_manager_generate_messages_eus _fleet_manager_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fleet_manager_geneus)
add_dependencies(fleet_manager_geneus fleet_manager_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fleet_manager_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(fleet_manager
  "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/DroneStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fleet_manager
)
_generate_msg_lisp(fleet_manager
  "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/TaskStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/lpf/docker_shared/rltraining_ws/src/nlp_drone_control/msg/Action.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fleet_manager
)

### Generating Services
_generate_srv_lisp(fleet_manager
  "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/srv/QueryFleetAndTasks.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/TaskStatus.msg;/home/lpf/docker_shared/rltraining_ws/src/nlp_drone_control/msg/Action.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/DroneStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fleet_manager
)
_generate_srv_lisp(fleet_manager
  "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/srv/InjectTasks.srv"
  "${MSG_I_FLAGS}"
  "/home/lpf/docker_shared/rltraining_ws/src/nlp_drone_control/msg/Action.msg;/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/TaskStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fleet_manager
)

### Generating Module File
_generate_module_lisp(fleet_manager
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fleet_manager
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(fleet_manager_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(fleet_manager_generate_messages fleet_manager_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/DroneStatus.msg" NAME_WE)
add_dependencies(fleet_manager_generate_messages_lisp _fleet_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/TaskStatus.msg" NAME_WE)
add_dependencies(fleet_manager_generate_messages_lisp _fleet_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/srv/QueryFleetAndTasks.srv" NAME_WE)
add_dependencies(fleet_manager_generate_messages_lisp _fleet_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/srv/InjectTasks.srv" NAME_WE)
add_dependencies(fleet_manager_generate_messages_lisp _fleet_manager_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fleet_manager_genlisp)
add_dependencies(fleet_manager_genlisp fleet_manager_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fleet_manager_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(fleet_manager
  "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/DroneStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fleet_manager
)
_generate_msg_nodejs(fleet_manager
  "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/TaskStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/lpf/docker_shared/rltraining_ws/src/nlp_drone_control/msg/Action.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fleet_manager
)

### Generating Services
_generate_srv_nodejs(fleet_manager
  "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/srv/QueryFleetAndTasks.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/TaskStatus.msg;/home/lpf/docker_shared/rltraining_ws/src/nlp_drone_control/msg/Action.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/DroneStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fleet_manager
)
_generate_srv_nodejs(fleet_manager
  "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/srv/InjectTasks.srv"
  "${MSG_I_FLAGS}"
  "/home/lpf/docker_shared/rltraining_ws/src/nlp_drone_control/msg/Action.msg;/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/TaskStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fleet_manager
)

### Generating Module File
_generate_module_nodejs(fleet_manager
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fleet_manager
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(fleet_manager_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(fleet_manager_generate_messages fleet_manager_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/DroneStatus.msg" NAME_WE)
add_dependencies(fleet_manager_generate_messages_nodejs _fleet_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/TaskStatus.msg" NAME_WE)
add_dependencies(fleet_manager_generate_messages_nodejs _fleet_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/srv/QueryFleetAndTasks.srv" NAME_WE)
add_dependencies(fleet_manager_generate_messages_nodejs _fleet_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/srv/InjectTasks.srv" NAME_WE)
add_dependencies(fleet_manager_generate_messages_nodejs _fleet_manager_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fleet_manager_gennodejs)
add_dependencies(fleet_manager_gennodejs fleet_manager_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fleet_manager_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(fleet_manager
  "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/DroneStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fleet_manager
)
_generate_msg_py(fleet_manager
  "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/TaskStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/lpf/docker_shared/rltraining_ws/src/nlp_drone_control/msg/Action.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fleet_manager
)

### Generating Services
_generate_srv_py(fleet_manager
  "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/srv/QueryFleetAndTasks.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/TaskStatus.msg;/home/lpf/docker_shared/rltraining_ws/src/nlp_drone_control/msg/Action.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/DroneStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fleet_manager
)
_generate_srv_py(fleet_manager
  "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/srv/InjectTasks.srv"
  "${MSG_I_FLAGS}"
  "/home/lpf/docker_shared/rltraining_ws/src/nlp_drone_control/msg/Action.msg;/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/TaskStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fleet_manager
)

### Generating Module File
_generate_module_py(fleet_manager
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fleet_manager
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(fleet_manager_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(fleet_manager_generate_messages fleet_manager_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/DroneStatus.msg" NAME_WE)
add_dependencies(fleet_manager_generate_messages_py _fleet_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/msg/TaskStatus.msg" NAME_WE)
add_dependencies(fleet_manager_generate_messages_py _fleet_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/srv/QueryFleetAndTasks.srv" NAME_WE)
add_dependencies(fleet_manager_generate_messages_py _fleet_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/fleet_manager/srv/InjectTasks.srv" NAME_WE)
add_dependencies(fleet_manager_generate_messages_py _fleet_manager_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fleet_manager_genpy)
add_dependencies(fleet_manager_genpy fleet_manager_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fleet_manager_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fleet_manager)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fleet_manager
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(fleet_manager_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(fleet_manager_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nlp_drone_control_generate_messages_cpp)
  add_dependencies(fleet_manager_generate_messages_cpp nlp_drone_control_generate_messages_cpp)
endif()
if(TARGET uav_scheduler_generate_messages_cpp)
  add_dependencies(fleet_manager_generate_messages_cpp uav_scheduler_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fleet_manager)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fleet_manager
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(fleet_manager_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(fleet_manager_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nlp_drone_control_generate_messages_eus)
  add_dependencies(fleet_manager_generate_messages_eus nlp_drone_control_generate_messages_eus)
endif()
if(TARGET uav_scheduler_generate_messages_eus)
  add_dependencies(fleet_manager_generate_messages_eus uav_scheduler_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fleet_manager)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fleet_manager
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(fleet_manager_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(fleet_manager_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nlp_drone_control_generate_messages_lisp)
  add_dependencies(fleet_manager_generate_messages_lisp nlp_drone_control_generate_messages_lisp)
endif()
if(TARGET uav_scheduler_generate_messages_lisp)
  add_dependencies(fleet_manager_generate_messages_lisp uav_scheduler_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fleet_manager)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fleet_manager
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(fleet_manager_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(fleet_manager_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nlp_drone_control_generate_messages_nodejs)
  add_dependencies(fleet_manager_generate_messages_nodejs nlp_drone_control_generate_messages_nodejs)
endif()
if(TARGET uav_scheduler_generate_messages_nodejs)
  add_dependencies(fleet_manager_generate_messages_nodejs uav_scheduler_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fleet_manager)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fleet_manager\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fleet_manager
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(fleet_manager_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(fleet_manager_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nlp_drone_control_generate_messages_py)
  add_dependencies(fleet_manager_generate_messages_py nlp_drone_control_generate_messages_py)
endif()
if(TARGET uav_scheduler_generate_messages_py)
  add_dependencies(fleet_manager_generate_messages_py uav_scheduler_generate_messages_py)
endif()
