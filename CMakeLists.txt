cmake_minimum_required(VERSION 2.8.3)
project(autonomous_mobile_robot)
set(CMAKE_BUILD_TYPE Debug)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_generation
  cv_bridge
  pcl_ros
  dynamic_reconfigure
  beego_control
  dynamixel_sdk
)
find_package(OpenCV REQUIRED)
#find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
#include_directories(/usr/local/include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ClassificationData.msg
  ClassificationElement.msg
  ClassificationState.msg
  ClassificationVelocityData.msg
  ImageFlow2D.msg
  ImageMatchingData.msg
  MaskImageData.msg
  SensorMapData.msg
  SensorMapDataMultiLayer.msg
  synchronizedImage.msg
  recordData.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# configファイルを追加
generate_dynamic_reconfigure_options(
  cfg/classification.cfg
  cfg/imageMatching.cfg
  cfg/velocityEstimation.cfg
  cfg/measurementVelocity.cfg
  cfg/recordExData.cfg
  cfg/obstacleAvoidance.cfg
  cfg/simulater.cfg
  cfg/trackingAvoidance.cfg
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jet_gpdr
   CATKIN_DEPENDS roscpp sensor_msgs message_runtime 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/jet_gpdr.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_cc # classification
  src/classification/constructor.cpp
  src/classification/methods.cpp
  src/classification/debug_main.cpp
  src/classification/debug_methods.cpp
  src/classification/property.cpp
)
add_executable(${PROJECT_NAME}_sc # stereo camera 
  src/convCamData/constructor.cpp
  src/convCamData/methods.cpp
  src/convCamData/debug_main.cpp
  src/convCamData/property.cpp
)
add_executable(${PROJECT_NAME}_lrf # laser range finder
  src/convLRFData/constructor.cpp
  src/convLRFData/methods.cpp
  src/convLRFData/debug_main.cpp
  src/convLRFData/property.cpp
)
add_executable(${PROJECT_NAME}_im # image matching
   src/imageMatching/constructor.cpp
   src/imageMatching/methods.cpp
   src/imageMatching/debug_main.cpp
   src/imageMatching/debug_methods.cpp
   src/imageMatching/property.cpp
 )
 add_executable(${PROJECT_NAME}_mv # measure Velocity
 src/measureVelocity/constructor.cpp
 src/measureVelocity/methods.cpp
 src/measureVelocity/debug_main.cpp
 src/measureVelocity/debug_methods.cpp
 src/measureVelocity/property.cpp
)
add_executable(${PROJECT_NAME}_si # point cloud
  src/synchroImage/constructor.cpp
  src/synchroImage/methods.cpp
  src/synchroImage/debug_main.cpp
  src/synchroImage/debug_methods.cpp
  src/synchroImage/property.cpp
)
add_executable(${PROJECT_NAME}_frm # manage
  src/frameRateManagement/constructor.cpp
  src/frameRateManagement/methods.cpp
  src/frameRateManagement/debug_main.cpp
  src/frameRateManagement/debug_methods.cpp
  src/frameRateManagement/property.cpp
)
add_executable(${PROJECT_NAME}_estimate # manage
  src/estimationVelocity/constructor.cpp
  src/estimationVelocity/methods.cpp
  src/estimationVelocity/debug_main.cpp
  src/estimationVelocity/property.cpp
  src/estimationVelocity/debug_methods.cpp
)
add_executable(${PROJECT_NAME}_red # record experiment data
  src/recordExData/constructor.cpp
  src/recordExData/methods.cpp
  src/recordExData/debug_main.cpp
  src/recordExData/property.cpp
  src/recordExData/debug_methods.cpp
)
add_executable(${PROJECT_NAME}_cp_vfh # obstacle_avoidance
  src/obstacleAvoidance/constructor.cpp
  src/obstacleAvoidance/methods.cpp
  src/obstacleAvoidance/debug_main.cpp
  src/obstacleAvoidance/debug_methods.cpp
  src/obstacleAvoidance/property.cpp
)
add_executable(${PROJECT_NAME}_simulater #simulater
  src/obstacleAvoidance/simulater_run.cpp
)
add_executable(${PROJECT_NAME}_apf 
  src/apf/apf.cpp 
  src/apf/gradient_map.cpp 
  src/apf/path_planning.cpp 
  src/apf/coordinate_transform.cpp 
  src/apf/grid_map.cpp 
  src/apf/potential_map.cpp 
  src/apf/set_param.cpp 
  src/apf/debug.cpp
  src/apf/apf_test.cpp 
)
add_executable(${PROJECT_NAME}_apf_mpc 
src/apf_mpc/apf_mpc_test.cpp
src/apf_mpc/apf_mpc.cpp 
src/apf_mpc/set_data.cpp 
src/apf_mpc/virtual_fnc.cpp 
src/apf_mpc/mv_pot_map.cpp  
src/apf_mpc/mpc_func.cpp 
src/apf_mpc/debug.cpp
src/apf/apf.cpp 
src/apf/gradient_map.cpp 
src/apf/path_planning.cpp 
src/apf/coordinate_transform.cpp 
src/apf/grid_map.cpp 
src/apf/potential_map.cpp 
src/apf/set_param.cpp 
src/apf/debug.cpp)

add_executable(${PROJECT_NAME}_aa 
src/trackingAvoidance/trackingAvoidance_analysis.cpp
src/trackingAvoidance/trackingAvoidance_experiment.cpp
src/trackingAvoidance/trackingAvoidance_cpvfh.cpp
src/trackingAvoidance/trackingAvoidance_ptapf.cpp)
add_executable(${PROJECT_NAME}_aa_sim 
src/trackingAvoidance/trackingAvoidance_simulation.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
## Add cmake target dependencies of the executable
## same as for the library above

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_cc ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_sc ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_lrf ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_im ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_mv ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_si ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_frm ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_estimate ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_red ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_cp_vfh ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_simulater ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(${PROJECT_NAME}_aa ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_aa_sim ${PROJECT_NAME}_generate_messages_cpp)

# c++ ROSノードのDynamic Reconfigureの依存の追加
add_dependencies(${PROJECT_NAME}_cc ${PROJECT_NAME}_gencfg)
add_dependencies(${PROJECT_NAME}_estimate ${PROJECT_NAME}_gencfg)
add_dependencies(${PROJECT_NAME}_im ${PROJECT_NAME}_gencfg)
add_dependencies(${PROJECT_NAME}_mv ${PROJECT_NAME}_gencfg)
add_dependencies(${PROJECT_NAME}_red ${PROJECT_NAME}_gencfg)
add_dependencies(${PROJECT_NAME}_cp_vfh ${PROJECT_NAME}_gencfg)
add_dependencies(${PROJECT_NAME}_simulater ${PROJECT_NAME}_gencfg)

add_dependencies(${PROJECT_NAME}_aa ${PROJECT_NAME}_gencfg)
add_dependencies(${PROJECT_NAME}_aa_sim ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_cc ${catkin_LIBRARIES}) # classification
target_link_libraries(${PROJECT_NAME}_sc ${catkin_LIBRARIES}) # stereo camera 
target_link_libraries(${PROJECT_NAME}_lrf ${catkin_LIBRARIES}) # laser range finder
target_link_libraries(${PROJECT_NAME}_im ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}) # image matching
target_link_libraries(${PROJECT_NAME}_mv ${catkin_LIBRARIES} ) # measurement velocity
target_link_libraries(${PROJECT_NAME}_si ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}) # synchro image
target_link_libraries(${PROJECT_NAME}_frm ${catkin_LIBRARIES} ) # manege 
target_link_libraries(${PROJECT_NAME}_estimate ${catkin_LIBRARIES} ) # estimate 
target_link_libraries(${PROJECT_NAME}_red ${catkin_LIBRARIES} ) # record experiment dat
target_link_libraries(${PROJECT_NAME}_cp_vfh ${catkin_LIBRARIES}) # obstacle_avoidance
target_link_libraries(${PROJECT_NAME}_simulater ${catkin_LIBRARIES}) #simulater
target_link_libraries(${PROJECT_NAME}_apf ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_apf_mpc ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_aa ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_aa_sim ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_jet_gpdr.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_definitions(-std=c++11)
