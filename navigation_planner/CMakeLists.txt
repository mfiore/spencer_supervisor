cmake_minimum_required(VERSION 2.8.3)
project(navigation_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs situation_assessment_msgs supervision_msgs message_runtime )



include_directories(${CMAKE_CURRENT_BINARY_DIR})
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
#add_message_files(
#   FILES
 #  Edge.msg
  # Graph.msg
   #Node.msg
   #Property.msg
  
 #)

## Generate services in the 'srv' folder
# add_service_files(
 #  FILES
  # ModifyEdge.srv
   #RequestMap.srv
   
 #)

## Generate added messages and services with any dependencies listed here


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 #  INCLUDE_DIRS include 
#  LIBRARIES beginner_tutorials
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime supervision_msgs situation_assessment_msgs
#  DEPENDS system_lib

)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

MESSAGE("${catkin_INCLUDE_DIRS}")
include_directories(
#include
  ${catkin_INCLUDE_DIRS} 
)


## Declare a cpp library
# add_library(planner
#   "/home/mfiore/catkin_ws/src/DrawGraph/Graph.cpp"
 #)

## Declare a cpp executable
 add_executable(navigation_planner src/navigation_planner.cpp)

 


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

add_dependencies(navigation_planner supervision_msgs_generate_messages_cpp situation_assessment_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(navigation_planner
   ${catkin_LIBRARIES}  
 )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS beginner_tutorials beginner_tutorials_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_beginner_tutorials.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)



