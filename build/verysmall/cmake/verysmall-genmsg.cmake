# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "verysmall: 6 messages, 0 services")

set(MSG_I_FLAGS "-Iverysmall:/home/marquesman/ararabots/src/verysmall/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(verysmall_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/game_topic.msg" NAME_WE)
add_custom_target(_verysmall_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "verysmall" "/home/marquesman/ararabots/src/verysmall/msg/game_topic.msg" ""
)

get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/motor_speed.msg" NAME_WE)
add_custom_target(_verysmall_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "verysmall" "/home/marquesman/ararabots/src/verysmall/msg/motor_speed.msg" ""
)

get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/comunication_topic.msg" NAME_WE)
add_custom_target(_verysmall_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "verysmall" "/home/marquesman/ararabots/src/verysmall/msg/comunication_topic.msg" "verysmall/motor_speed"
)

get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/five_robot_vector.msg" NAME_WE)
add_custom_target(_verysmall_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "verysmall" "/home/marquesman/ararabots/src/verysmall/msg/five_robot_vector.msg" ""
)

get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/five_robot_pos.msg" NAME_WE)
add_custom_target(_verysmall_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "verysmall" "/home/marquesman/ararabots/src/verysmall/msg/five_robot_pos.msg" ""
)

get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/things_position.msg" NAME_WE)
add_custom_target(_verysmall_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "verysmall" "/home/marquesman/ararabots/src/verysmall/msg/things_position.msg" "verysmall/five_robot_pos:verysmall/five_robot_vector"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/game_topic.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/verysmall
)
_generate_msg_cpp(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/motor_speed.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/verysmall
)
_generate_msg_cpp(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/comunication_topic.msg"
  "${MSG_I_FLAGS}"
  "/home/marquesman/ararabots/src/verysmall/msg/motor_speed.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/verysmall
)
_generate_msg_cpp(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/five_robot_vector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/verysmall
)
_generate_msg_cpp(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/five_robot_pos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/verysmall
)
_generate_msg_cpp(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/things_position.msg"
  "${MSG_I_FLAGS}"
  "/home/marquesman/ararabots/src/verysmall/msg/five_robot_pos.msg;/home/marquesman/ararabots/src/verysmall/msg/five_robot_vector.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/verysmall
)

### Generating Services

### Generating Module File
_generate_module_cpp(verysmall
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/verysmall
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(verysmall_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(verysmall_generate_messages verysmall_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/game_topic.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_cpp _verysmall_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/motor_speed.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_cpp _verysmall_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/comunication_topic.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_cpp _verysmall_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/five_robot_vector.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_cpp _verysmall_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/five_robot_pos.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_cpp _verysmall_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/things_position.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_cpp _verysmall_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(verysmall_gencpp)
add_dependencies(verysmall_gencpp verysmall_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS verysmall_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/game_topic.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/verysmall
)
_generate_msg_eus(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/motor_speed.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/verysmall
)
_generate_msg_eus(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/comunication_topic.msg"
  "${MSG_I_FLAGS}"
  "/home/marquesman/ararabots/src/verysmall/msg/motor_speed.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/verysmall
)
_generate_msg_eus(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/five_robot_vector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/verysmall
)
_generate_msg_eus(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/five_robot_pos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/verysmall
)
_generate_msg_eus(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/things_position.msg"
  "${MSG_I_FLAGS}"
  "/home/marquesman/ararabots/src/verysmall/msg/five_robot_pos.msg;/home/marquesman/ararabots/src/verysmall/msg/five_robot_vector.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/verysmall
)

### Generating Services

### Generating Module File
_generate_module_eus(verysmall
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/verysmall
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(verysmall_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(verysmall_generate_messages verysmall_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/game_topic.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_eus _verysmall_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/motor_speed.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_eus _verysmall_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/comunication_topic.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_eus _verysmall_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/five_robot_vector.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_eus _verysmall_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/five_robot_pos.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_eus _verysmall_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/things_position.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_eus _verysmall_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(verysmall_geneus)
add_dependencies(verysmall_geneus verysmall_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS verysmall_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/game_topic.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/verysmall
)
_generate_msg_lisp(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/motor_speed.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/verysmall
)
_generate_msg_lisp(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/comunication_topic.msg"
  "${MSG_I_FLAGS}"
  "/home/marquesman/ararabots/src/verysmall/msg/motor_speed.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/verysmall
)
_generate_msg_lisp(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/five_robot_vector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/verysmall
)
_generate_msg_lisp(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/five_robot_pos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/verysmall
)
_generate_msg_lisp(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/things_position.msg"
  "${MSG_I_FLAGS}"
  "/home/marquesman/ararabots/src/verysmall/msg/five_robot_pos.msg;/home/marquesman/ararabots/src/verysmall/msg/five_robot_vector.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/verysmall
)

### Generating Services

### Generating Module File
_generate_module_lisp(verysmall
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/verysmall
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(verysmall_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(verysmall_generate_messages verysmall_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/game_topic.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_lisp _verysmall_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/motor_speed.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_lisp _verysmall_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/comunication_topic.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_lisp _verysmall_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/five_robot_vector.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_lisp _verysmall_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/five_robot_pos.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_lisp _verysmall_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/things_position.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_lisp _verysmall_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(verysmall_genlisp)
add_dependencies(verysmall_genlisp verysmall_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS verysmall_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/game_topic.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/verysmall
)
_generate_msg_nodejs(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/motor_speed.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/verysmall
)
_generate_msg_nodejs(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/comunication_topic.msg"
  "${MSG_I_FLAGS}"
  "/home/marquesman/ararabots/src/verysmall/msg/motor_speed.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/verysmall
)
_generate_msg_nodejs(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/five_robot_vector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/verysmall
)
_generate_msg_nodejs(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/five_robot_pos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/verysmall
)
_generate_msg_nodejs(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/things_position.msg"
  "${MSG_I_FLAGS}"
  "/home/marquesman/ararabots/src/verysmall/msg/five_robot_pos.msg;/home/marquesman/ararabots/src/verysmall/msg/five_robot_vector.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/verysmall
)

### Generating Services

### Generating Module File
_generate_module_nodejs(verysmall
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/verysmall
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(verysmall_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(verysmall_generate_messages verysmall_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/game_topic.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_nodejs _verysmall_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/motor_speed.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_nodejs _verysmall_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/comunication_topic.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_nodejs _verysmall_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/five_robot_vector.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_nodejs _verysmall_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/five_robot_pos.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_nodejs _verysmall_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/things_position.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_nodejs _verysmall_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(verysmall_gennodejs)
add_dependencies(verysmall_gennodejs verysmall_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS verysmall_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/game_topic.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/verysmall
)
_generate_msg_py(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/motor_speed.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/verysmall
)
_generate_msg_py(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/comunication_topic.msg"
  "${MSG_I_FLAGS}"
  "/home/marquesman/ararabots/src/verysmall/msg/motor_speed.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/verysmall
)
_generate_msg_py(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/five_robot_vector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/verysmall
)
_generate_msg_py(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/five_robot_pos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/verysmall
)
_generate_msg_py(verysmall
  "/home/marquesman/ararabots/src/verysmall/msg/things_position.msg"
  "${MSG_I_FLAGS}"
  "/home/marquesman/ararabots/src/verysmall/msg/five_robot_pos.msg;/home/marquesman/ararabots/src/verysmall/msg/five_robot_vector.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/verysmall
)

### Generating Services

### Generating Module File
_generate_module_py(verysmall
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/verysmall
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(verysmall_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(verysmall_generate_messages verysmall_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/game_topic.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_py _verysmall_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/motor_speed.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_py _verysmall_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/comunication_topic.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_py _verysmall_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/five_robot_vector.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_py _verysmall_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/five_robot_pos.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_py _verysmall_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marquesman/ararabots/src/verysmall/msg/things_position.msg" NAME_WE)
add_dependencies(verysmall_generate_messages_py _verysmall_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(verysmall_genpy)
add_dependencies(verysmall_genpy verysmall_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS verysmall_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/verysmall)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/verysmall
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(verysmall_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/verysmall)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/verysmall
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(verysmall_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/verysmall)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/verysmall
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(verysmall_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/verysmall)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/verysmall
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(verysmall_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/verysmall)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/verysmall\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/verysmall
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(verysmall_generate_messages_py std_msgs_generate_messages_py)
endif()
