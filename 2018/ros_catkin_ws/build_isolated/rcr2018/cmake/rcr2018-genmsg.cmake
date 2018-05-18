# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rcr2018: 4 messages, 0 services")

set(MSG_I_FLAGS "-Ircr2018:/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rcr2018_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/TofSide.msg" NAME_WE)
add_custom_target(_rcr2018_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rcr2018" "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/TofSide.msg" ""
)

get_filename_component(_filename "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/TofFront.msg" NAME_WE)
add_custom_target(_rcr2018_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rcr2018" "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/TofFront.msg" ""
)

get_filename_component(_filename "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/AngVel.msg" NAME_WE)
add_custom_target(_rcr2018_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rcr2018" "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/AngVel.msg" ""
)

get_filename_component(_filename "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/LineCount.msg" NAME_WE)
add_custom_target(_rcr2018_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rcr2018" "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/LineCount.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(rcr2018
  "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/TofSide.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rcr2018
)
_generate_msg_cpp(rcr2018
  "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/TofFront.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rcr2018
)
_generate_msg_cpp(rcr2018
  "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/AngVel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rcr2018
)
_generate_msg_cpp(rcr2018
  "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/LineCount.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rcr2018
)

### Generating Services

### Generating Module File
_generate_module_cpp(rcr2018
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rcr2018
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rcr2018_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rcr2018_generate_messages rcr2018_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/TofSide.msg" NAME_WE)
add_dependencies(rcr2018_generate_messages_cpp _rcr2018_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/TofFront.msg" NAME_WE)
add_dependencies(rcr2018_generate_messages_cpp _rcr2018_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/AngVel.msg" NAME_WE)
add_dependencies(rcr2018_generate_messages_cpp _rcr2018_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/LineCount.msg" NAME_WE)
add_dependencies(rcr2018_generate_messages_cpp _rcr2018_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rcr2018_gencpp)
add_dependencies(rcr2018_gencpp rcr2018_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rcr2018_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(rcr2018
  "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/TofSide.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rcr2018
)
_generate_msg_eus(rcr2018
  "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/TofFront.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rcr2018
)
_generate_msg_eus(rcr2018
  "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/AngVel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rcr2018
)
_generate_msg_eus(rcr2018
  "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/LineCount.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rcr2018
)

### Generating Services

### Generating Module File
_generate_module_eus(rcr2018
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rcr2018
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(rcr2018_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(rcr2018_generate_messages rcr2018_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/TofSide.msg" NAME_WE)
add_dependencies(rcr2018_generate_messages_eus _rcr2018_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/TofFront.msg" NAME_WE)
add_dependencies(rcr2018_generate_messages_eus _rcr2018_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/AngVel.msg" NAME_WE)
add_dependencies(rcr2018_generate_messages_eus _rcr2018_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/LineCount.msg" NAME_WE)
add_dependencies(rcr2018_generate_messages_eus _rcr2018_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rcr2018_geneus)
add_dependencies(rcr2018_geneus rcr2018_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rcr2018_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(rcr2018
  "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/TofSide.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rcr2018
)
_generate_msg_lisp(rcr2018
  "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/TofFront.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rcr2018
)
_generate_msg_lisp(rcr2018
  "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/AngVel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rcr2018
)
_generate_msg_lisp(rcr2018
  "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/LineCount.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rcr2018
)

### Generating Services

### Generating Module File
_generate_module_lisp(rcr2018
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rcr2018
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rcr2018_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rcr2018_generate_messages rcr2018_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/TofSide.msg" NAME_WE)
add_dependencies(rcr2018_generate_messages_lisp _rcr2018_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/TofFront.msg" NAME_WE)
add_dependencies(rcr2018_generate_messages_lisp _rcr2018_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/AngVel.msg" NAME_WE)
add_dependencies(rcr2018_generate_messages_lisp _rcr2018_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/LineCount.msg" NAME_WE)
add_dependencies(rcr2018_generate_messages_lisp _rcr2018_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rcr2018_genlisp)
add_dependencies(rcr2018_genlisp rcr2018_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rcr2018_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(rcr2018
  "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/TofSide.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rcr2018
)
_generate_msg_nodejs(rcr2018
  "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/TofFront.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rcr2018
)
_generate_msg_nodejs(rcr2018
  "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/AngVel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rcr2018
)
_generate_msg_nodejs(rcr2018
  "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/LineCount.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rcr2018
)

### Generating Services

### Generating Module File
_generate_module_nodejs(rcr2018
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rcr2018
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(rcr2018_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(rcr2018_generate_messages rcr2018_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/TofSide.msg" NAME_WE)
add_dependencies(rcr2018_generate_messages_nodejs _rcr2018_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/TofFront.msg" NAME_WE)
add_dependencies(rcr2018_generate_messages_nodejs _rcr2018_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/AngVel.msg" NAME_WE)
add_dependencies(rcr2018_generate_messages_nodejs _rcr2018_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/LineCount.msg" NAME_WE)
add_dependencies(rcr2018_generate_messages_nodejs _rcr2018_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rcr2018_gennodejs)
add_dependencies(rcr2018_gennodejs rcr2018_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rcr2018_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(rcr2018
  "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/TofSide.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rcr2018
)
_generate_msg_py(rcr2018
  "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/TofFront.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rcr2018
)
_generate_msg_py(rcr2018
  "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/AngVel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rcr2018
)
_generate_msg_py(rcr2018
  "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/LineCount.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rcr2018
)

### Generating Services

### Generating Module File
_generate_module_py(rcr2018
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rcr2018
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rcr2018_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rcr2018_generate_messages rcr2018_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/TofSide.msg" NAME_WE)
add_dependencies(rcr2018_generate_messages_py _rcr2018_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/TofFront.msg" NAME_WE)
add_dependencies(rcr2018_generate_messages_py _rcr2018_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/AngVel.msg" NAME_WE)
add_dependencies(rcr2018_generate_messages_py _rcr2018_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/RCR/2018/ros_catkin_ws/src/rcr2018/msg/LineCount.msg" NAME_WE)
add_dependencies(rcr2018_generate_messages_py _rcr2018_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rcr2018_genpy)
add_dependencies(rcr2018_genpy rcr2018_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rcr2018_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rcr2018)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rcr2018
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(rcr2018_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rcr2018)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rcr2018
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(rcr2018_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rcr2018)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rcr2018
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(rcr2018_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rcr2018)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rcr2018
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(rcr2018_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rcr2018)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rcr2018\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rcr2018
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(rcr2018_generate_messages_py std_msgs_generate_messages_py)
endif()
