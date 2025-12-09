execute_process(COMMAND "/home/lpf/docker_shared/rltraining_ws/build/ros_tcp_endpoint/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/lpf/docker_shared/rltraining_ws/build/ros_tcp_endpoint/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
