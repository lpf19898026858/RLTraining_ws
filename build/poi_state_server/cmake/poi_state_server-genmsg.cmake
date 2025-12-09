# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "poi_state_server: 2 messages, 2 services")

set(MSG_I_FLAGS "-Ipoi_state_server:/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(poi_state_server_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg" NAME_WE)
add_custom_target(_poi_state_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "poi_state_server" "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/POIInfo.msg" NAME_WE)
add_custom_target(_poi_state_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "poi_state_server" "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/POIInfo.msg" "poi_state_server/NamedPoint:geometry_msgs/Point"
)

get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/srv/GetPOIInfo.srv" NAME_WE)
add_custom_target(_poi_state_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "poi_state_server" "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/srv/GetPOIInfo.srv" "poi_state_server/POIInfo:poi_state_server/NamedPoint:geometry_msgs/Point"
)

get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/srv/ListPOIs.srv" NAME_WE)
add_custom_target(_poi_state_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "poi_state_server" "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/srv/ListPOIs.srv" "poi_state_server/POIInfo:poi_state_server/NamedPoint:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(poi_state_server
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/poi_state_server
)
_generate_msg_cpp(poi_state_server
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/POIInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/poi_state_server
)

### Generating Services
_generate_srv_cpp(poi_state_server
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/srv/GetPOIInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/POIInfo.msg;/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/poi_state_server
)
_generate_srv_cpp(poi_state_server
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/srv/ListPOIs.srv"
  "${MSG_I_FLAGS}"
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/POIInfo.msg;/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/poi_state_server
)

### Generating Module File
_generate_module_cpp(poi_state_server
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/poi_state_server
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(poi_state_server_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(poi_state_server_generate_messages poi_state_server_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg" NAME_WE)
add_dependencies(poi_state_server_generate_messages_cpp _poi_state_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/POIInfo.msg" NAME_WE)
add_dependencies(poi_state_server_generate_messages_cpp _poi_state_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/srv/GetPOIInfo.srv" NAME_WE)
add_dependencies(poi_state_server_generate_messages_cpp _poi_state_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/srv/ListPOIs.srv" NAME_WE)
add_dependencies(poi_state_server_generate_messages_cpp _poi_state_server_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(poi_state_server_gencpp)
add_dependencies(poi_state_server_gencpp poi_state_server_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS poi_state_server_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(poi_state_server
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/poi_state_server
)
_generate_msg_eus(poi_state_server
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/POIInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/poi_state_server
)

### Generating Services
_generate_srv_eus(poi_state_server
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/srv/GetPOIInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/POIInfo.msg;/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/poi_state_server
)
_generate_srv_eus(poi_state_server
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/srv/ListPOIs.srv"
  "${MSG_I_FLAGS}"
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/POIInfo.msg;/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/poi_state_server
)

### Generating Module File
_generate_module_eus(poi_state_server
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/poi_state_server
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(poi_state_server_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(poi_state_server_generate_messages poi_state_server_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg" NAME_WE)
add_dependencies(poi_state_server_generate_messages_eus _poi_state_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/POIInfo.msg" NAME_WE)
add_dependencies(poi_state_server_generate_messages_eus _poi_state_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/srv/GetPOIInfo.srv" NAME_WE)
add_dependencies(poi_state_server_generate_messages_eus _poi_state_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/srv/ListPOIs.srv" NAME_WE)
add_dependencies(poi_state_server_generate_messages_eus _poi_state_server_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(poi_state_server_geneus)
add_dependencies(poi_state_server_geneus poi_state_server_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS poi_state_server_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(poi_state_server
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/poi_state_server
)
_generate_msg_lisp(poi_state_server
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/POIInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/poi_state_server
)

### Generating Services
_generate_srv_lisp(poi_state_server
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/srv/GetPOIInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/POIInfo.msg;/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/poi_state_server
)
_generate_srv_lisp(poi_state_server
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/srv/ListPOIs.srv"
  "${MSG_I_FLAGS}"
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/POIInfo.msg;/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/poi_state_server
)

### Generating Module File
_generate_module_lisp(poi_state_server
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/poi_state_server
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(poi_state_server_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(poi_state_server_generate_messages poi_state_server_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg" NAME_WE)
add_dependencies(poi_state_server_generate_messages_lisp _poi_state_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/POIInfo.msg" NAME_WE)
add_dependencies(poi_state_server_generate_messages_lisp _poi_state_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/srv/GetPOIInfo.srv" NAME_WE)
add_dependencies(poi_state_server_generate_messages_lisp _poi_state_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/srv/ListPOIs.srv" NAME_WE)
add_dependencies(poi_state_server_generate_messages_lisp _poi_state_server_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(poi_state_server_genlisp)
add_dependencies(poi_state_server_genlisp poi_state_server_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS poi_state_server_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(poi_state_server
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/poi_state_server
)
_generate_msg_nodejs(poi_state_server
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/POIInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/poi_state_server
)

### Generating Services
_generate_srv_nodejs(poi_state_server
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/srv/GetPOIInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/POIInfo.msg;/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/poi_state_server
)
_generate_srv_nodejs(poi_state_server
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/srv/ListPOIs.srv"
  "${MSG_I_FLAGS}"
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/POIInfo.msg;/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/poi_state_server
)

### Generating Module File
_generate_module_nodejs(poi_state_server
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/poi_state_server
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(poi_state_server_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(poi_state_server_generate_messages poi_state_server_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg" NAME_WE)
add_dependencies(poi_state_server_generate_messages_nodejs _poi_state_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/POIInfo.msg" NAME_WE)
add_dependencies(poi_state_server_generate_messages_nodejs _poi_state_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/srv/GetPOIInfo.srv" NAME_WE)
add_dependencies(poi_state_server_generate_messages_nodejs _poi_state_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/srv/ListPOIs.srv" NAME_WE)
add_dependencies(poi_state_server_generate_messages_nodejs _poi_state_server_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(poi_state_server_gennodejs)
add_dependencies(poi_state_server_gennodejs poi_state_server_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS poi_state_server_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(poi_state_server
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/poi_state_server
)
_generate_msg_py(poi_state_server
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/POIInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/poi_state_server
)

### Generating Services
_generate_srv_py(poi_state_server
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/srv/GetPOIInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/POIInfo.msg;/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/poi_state_server
)
_generate_srv_py(poi_state_server
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/srv/ListPOIs.srv"
  "${MSG_I_FLAGS}"
  "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/POIInfo.msg;/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/poi_state_server
)

### Generating Module File
_generate_module_py(poi_state_server
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/poi_state_server
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(poi_state_server_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(poi_state_server_generate_messages poi_state_server_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/NamedPoint.msg" NAME_WE)
add_dependencies(poi_state_server_generate_messages_py _poi_state_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/msg/POIInfo.msg" NAME_WE)
add_dependencies(poi_state_server_generate_messages_py _poi_state_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/srv/GetPOIInfo.srv" NAME_WE)
add_dependencies(poi_state_server_generate_messages_py _poi_state_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lpf/docker_shared/rltraining_ws/src/poi_state_server/srv/ListPOIs.srv" NAME_WE)
add_dependencies(poi_state_server_generate_messages_py _poi_state_server_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(poi_state_server_genpy)
add_dependencies(poi_state_server_genpy poi_state_server_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS poi_state_server_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/poi_state_server)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/poi_state_server
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(poi_state_server_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(poi_state_server_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/poi_state_server)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/poi_state_server
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(poi_state_server_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(poi_state_server_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/poi_state_server)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/poi_state_server
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(poi_state_server_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(poi_state_server_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/poi_state_server)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/poi_state_server
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(poi_state_server_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(poi_state_server_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/poi_state_server)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/poi_state_server\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/poi_state_server
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(poi_state_server_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(poi_state_server_generate_messages_py geometry_msgs_generate_messages_py)
endif()
