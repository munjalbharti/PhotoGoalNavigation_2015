cmake_minimum_required(VERSION 2.8.3)
project(photo_goal_navigation_new)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs std_msgs cv_bridge image_transport image_geometry pcl_conversions pcl_ros message_filters tf cmake_modules keyboard )

FIND_PACKAGE(Eigen REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})



message("Overriding defaults as unable to find correct opencv version")
set(OpenCV_INCLUDE_DIRS "/usr/prakt/w043/Downloads/opencv_new/opencv_installed/include/")


set(OpenCV_LIBS ${OpenCV_LIBS} "/usr/prakt/w043/Downloads/opencv_new/opencv_installed/lib/libopencv_core.so.3.1.0")
set(OpenCV_LIBS ${OpenCV_LIBS} "/usr/prakt/w043/Downloads/opencv_new/opencv_installed/lib/libopencv_imgcodecs.so.3.1.0")
set(OpenCV_LIBS ${OpenCV_LIBS} "/usr/prakt/w043/Downloads/opencv_new/opencv_installed/lib/libopencv_highgui.so.3.1.0")
set(OpenCV_LIBS ${OpenCV_LIBS} "/usr/prakt/w043/Downloads/opencv_new/opencv_installed/lib/libopencv_features2d.so.3.1.0")
set(OpenCV_LIBS ${OpenCV_LIBS} "/usr/prakt/w043/Downloads/opencv_new/opencv_installed/lib/libopencv_xfeatures2d.so.3.1.0")
set(OpenCV_LIBS ${OpenCV_LIBS} "/usr/prakt/w043/Downloads/opencv_new/opencv_installed/lib/libopencv_imgproc.so.3.1.0")
set(OpenCV_LIBS ${OpenCV_LIBS} "/usr/prakt/w043/Downloads/opencv_new/opencv_installed/lib/libopencv_calib3d.so.3.1.0")


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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

add_message_files(FILES filter_state.msg)
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
#   keyboard
   std_msgs  # Or other packages containing msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## Declare things to be passed to dependent projects
## The catkin_package macro generates cmake config files for your package
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES exercise1
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
   DEPENDS opencv	
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)


## Declare a C++ library
# add_library(exercise1
#   src/${PROJECT_NAME}/exercise1.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(exercise1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(exercise1_node src/exercise1_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(exercise1_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(exercise1_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS exercise1 exercise1_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_exercise1.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


#add_executable(undistort_image src/undistort_image.cpp)
#target_link_libraries(undistort_image ${catkin_LIBRARIES})

#add_executable(rgb_to_cloud src/rgb_to_point_cloud.cpp)
#target_link_libraries(rgb_to_cloud ${catkin_LIBRARIES})

#add_executable(point_cloud_world_cordinates src/point_cloud_world_cordinates.cpp)
#target_link_libraries(point_cloud_world_cordinates ${catkin_LIBRARIES})

message("CMAKE binary direc ${CMAKE_BINARY_DIR}")

include_directories(
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/TooN/include 
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/libcvd/include 
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/gvars3/include 
)

link_directories(
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/libcvd/lib 
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/gvars3/lib 
)



set(SRCS src/photo_goal_nav.cpp )
add_executable(photo_goal ${SRCS})
target_link_libraries(photo_goal ${OpenCV_LIBS}  ${catkin_LIBRARIES})
include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
#add_dependencies(photo_goal keyboard_gencpp)
find_package(PCL REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

set (CMAKE_CXX_FLAGS "-Wl,-rpath=/usr/prakt/w043/Downloads/opencv_new/opencv_installed/lib/")
