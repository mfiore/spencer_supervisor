cmake_minimum_required(VERSION 2.8.3)
project(appl)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation message_runtime )

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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   GetAction.srv
   
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #INCLUDE_DIRS include
#  LIBRARIES appl
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime message_generation
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
   "/home/mfiore/openrobots/include/openprs/"
    # src
    src/Core
    src/Models/MOMDP
    src/Bounds
    src/Algorithms
    src/Algorithms/SARSOP
    src/MathLib
    src/Parser
    src/Parser/Cassandra
    src/Parser/POMDPX
     src/Utils
    src/OfflineSolver
    src/Evaluator
    src/Simulator
    src/Controller
)

# Declare a cpp library
add_library(appl_lib
    src/PolicyGraph/PolicyGraph.cpp
    src/PolicyGraph/PolicyGraphGenerator.cpp
    src/Evaluator/EvaluationEngine.cpp
    src/Evaluator/EvaluatorSampleEngine.cpp
    src/Simulator/SimulationEngine.cpp
    src/MathLib/DenseVector.cpp
    src/MathLib/MathLib.cpp
    src/MathLib/SparseMatrix.cpp
    src/MathLib/SparseVector.cpp
    src/Algorithms/SARSOP/BinManager.cpp
    src/Algorithms/SARSOP/BinManagerSet.cpp
    src/Algorithms/SARSOP/Sample.cpp
    src/Algorithms/SARSOP/SampleBP.cpp
    src/Algorithms/SARSOP/SARSOP.cpp
    src/Algorithms/SARSOP/SARSOPPrune.cpp
    src/Models/MOMDP/StateTransitionX.cpp
    src/Models/MOMDP/IVariable.cpp
    src/Models/MOMDP/BooleanVariable.cpp
    src/Models/MOMDP/BeliefTransitionMOMDPLite.cpp
    src/Models/MOMDP/Variable.cpp
    src/Models/MOMDP/IVariableValue.cpp
    src/Models/MOMDP/StateTransitionXY.cpp
    src/Models/MOMDP/MOMDP.cpp
    src/Models/MOMDP/StateTransitionXXpY.cpp
    src/Models/MOMDP/VariableRelation.cpp
    src/Models/MOMDP/StateTransitionY.cpp
    src/Models/MOMDP/VariableValue.cpp
    src/Models/MOMDP/VariableCombined.cpp
    src/Models/MOMDP/MOMDPLite.cpp
    src/Models/MOMDP/Rewards.cpp
    src/Models/MOMDP/ObservationProbabilities.cpp
    src/Models/MOMDP/BeliefTransitionMOMDP.cpp
    src/OfflineSolver/GlobalResource.cpp
    src/OfflineSolver/solverUtils.cpp
    src/Bounds/AlphaPlane.cpp
    src/Bounds/AlphaPlanePool.cpp
    src/Bounds/AlphaPlanePoolSet.cpp
    src/Bounds/AlphaVectorPolicy.cpp
    src/Bounds/BackupAlphaPlaneMOMDP.cpp
    src/Bounds/BackupAlphaPlaneMOMDPLite.cpp
    src/Bounds/BackupBeliefValuePairMOMDP.cpp
    src/Bounds/BackupBeliefValuePairMOMDPLite.cpp
    src/Bounds/BeliefValuePair.cpp
    src/Bounds/BeliefValuePairPool.cpp
    src/Bounds/BeliefValuePairPoolSet.cpp
    src/Bounds/BlindLBInitializer.cpp
    src/Bounds/FastInfUBInitializer.cpp
    src/Bounds/FullObsUBInitializer.cpp
    src/Bounds/PruneAlphaPlane.cpp
    src/Bounds/PruneBeliefValuePair.cpp
    src/Bounds/xml_parse_lib.c
    src/Core/Actions.cpp
    src/Core/VariableContainer.cpp
    src/Core/Belief.cpp
    src/Core/BeliefCache.cpp
    src/Core/BeliefException.cpp
    src/Core/BeliefForest.cpp
    src/Core/BeliefTreeNode.cpp
    src/Core/BeliefWithState.cpp
    src/Core/MObject.cpp
    src/Core/Observations.cpp
    src/Core/States.cpp
    src/Core/UniqueBeliefHeap.cpp
    src/Parser/Cassandra/Parser.cpp
    src/Parser/Cassandra/POMDP.cpp
    src/Parser/Cassandra/pomdpCassandraWrapper.cpp
    src/Parser/ParserSelector.cpp
    src/Parser/POMDPX/FactoredPomdp.cpp
    src/Parser/POMDPX/Function.cpp
    src/Parser/POMDPX/ObsAct.cpp
    src/Parser/POMDPX/PreCEntry.cpp
    src/Parser/POMDPX/PreCMatrix.cpp
    src/Parser/POMDPX/SparseEntry.cpp
    src/Parser/POMDPX/SparseTable.cpp
    src/Parser/POMDPX/State.cpp
    src/Parser/POMDPX/StateObsAct.cpp
    src/Parser/POMDPX/tinystr.cpp
    src/Parser/POMDPX/tinyxml.cpp
    src/Parser/POMDPX/tinyxmlerror.cpp
    src/Parser/POMDPX/tinyxmlparser.cpp
    src/Utils/InfoLog.cpp
    src/Utils/md5.cpp
    src/Utils/md5wrapper.cpp
    src/Utils/CPMemUtils.cpp
    src/Utils/StatsCollector.cpp
    src/Utils/SimulationRewardCollector.cpp
    src/Parser/Cassandra/include/pomdp_spec.tab.cc
    src/Parser/Cassandra/include/pomdp_spec.yy.cc
    src/miniposix/getopt.c
    src/miniposix/getopt1.c
    src/miniposix/getopt_init.c
    src/Parser/Cassandra/decision-tree.c
    src/Parser/Cassandra/imm-reward.c
    src/Parser/Cassandra/mdpCassandra.c
    src/Parser/Cassandra/parse_err.c
    src/Parser/Cassandra/parse_hash.c
    src/Parser/Cassandra/sparse-matrix.c
    src/Controller/Controller.cpp
)
## Declare a cpp executable
add_executable(controlpipe src/Controller/controlpipe.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
 add_dependencies(controlpipe appl_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(appl_node
#   ${catkin_LIBRARIES}
# )
 target_link_libraries(controlpipe
   ${catkin_LIBRARIES} 
   appl_lib

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
# install(TARGETS appl appl_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_appl.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
