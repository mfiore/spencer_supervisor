# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "appl: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/groovy/share/std_msgs/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

#better way to handle this?
set (ALL_GEN_OUTPUT_FILES_cpp "")

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(appl
  /home/mfiore/catkin_ws/src/appl/srv/appl_request.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/appl
)

### Generating Module File
_generate_module_cpp(appl
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/appl
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(appl_gencpp ALL
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(appl
  /home/mfiore/catkin_ws/src/appl/srv/appl_request.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/appl
)

### Generating Module File
_generate_module_lisp(appl
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/appl
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(appl_genlisp ALL
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(appl
  /home/mfiore/catkin_ws/src/appl/srv/appl_request.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/appl
)

### Generating Module File
_generate_module_py(appl
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/appl
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(appl_genpy ALL
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)


debug_message(2 "appl: Iflags=${MSG_I_FLAGS}")


if(gencpp_INSTALL_DIR)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/appl
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(appl_gencpp std_msgs_gencpp)

if(genlisp_INSTALL_DIR)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/appl
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(appl_genlisp std_msgs_genlisp)

if(genpy_INSTALL_DIR)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/appl\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/appl
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(appl_genpy std_msgs_genpy)
