execute_process(COMMAND "/home/pi/RCR/2018/ros_catkin_ws/build_isolated/rosmake/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/pi/RCR/2018/ros_catkin_ws/build_isolated/rosmake/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
