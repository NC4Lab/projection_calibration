cmake_minimum_required(VERSION 3.0.2)
project(projection_calibration)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  genmsg
  roslib
  cv_bridge
)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)
# find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED)

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
	DIRECTORY
	msg
	FILES
  # Message1.msg
   #Message2.msg
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
   std_msgs  # Or other packages containing msgs
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
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
#  INCLUDE_DIRS glfw-3.3.8/include
  INCLUDE_DIRS lodepng/
  INCLUDE_DIRS nanosvg-master/
#  LIBRARIES projection_calibration OpenGL
  CATKIN_DEPENDS roscpp roslib std_msgs message_runtime
#  DEPENDS system_lib
)


###########
## Build ##
###########
# TODO - README.md should ask user to install these libraries and create environment variables pointing to them.
# set(PROJECTION_DIR "C:/ws/src/projection_calibration")
# set(glfw3_DIR "C:/Program Files (x86)/GLFW/lib/cmake/glfw3/")
# set(IL_INCLUDE_DIR "C:/Program Files (x86)/IL/include")
# set(IL_LIB_DIR "C:/Program Files (x86)/IL/lib/x64/Release/")
# set(glm_DIR "C:/Program Files (x86)/glm/cmake/glm/")


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  #${PROJECTION_DIR}/glfw-3.3.8/include
 # ${PROJECTION_DIR}/glfw-3.3.8/deps/glad
  # ${glfw3_dir}/../../../include
  ${glfw3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  $ENV{IL_INCLUDE_DIR}
  ${PROJECTION_DIR}/src/glm
  ${PROJECTION_DIR}/src/pugixml
 # "C:/Program Files (x86)/IL/include"
 # ${PROJECTION_DIR}/dll

)

find_library(IL_LIBRARY NAMES DevIL ILU ILUT.lib PATHS $ENV{IL_LIB_DIR})
find_library(ILU_LIBRARY NAMES ILU PATHS $ENV{IL_LIB_DIR})
find_library(ILUT_LIBRARY NAMES ILUT PATHS $ENV{IL_LIB_DIR})
#find_library(ILUT_LIBRARY NAMES ILUT.lib PATH "C:/Program Files (x86)/IL/lib/x64/Release/ILUT.lib")
#find_library(IL_LIBRARY NAMES DevIL ILU ILUT PATHS ${IL_LIB_DIR})
#find_library(DEVIL_LIBRARIES NAMES DevIL ILU ILUT PATHS ${IL_LIBRARY})

if(IL_LIBRARY)
  message(STATUS "Found IL library: ${IL_LIBRARY}")
else()
  message(FATAL_ERROR "IL library not found")
endif()
if(ILU_LIBRARY)
  message(STATUS "Found ILU library: ${ILU_LIBRARY}")
#else()
#  message(FATAL_ERROR "ILU library not found")
endif()

#find_package(IL REQUIRED)
#add_subdirectory(glfw-3.3.8)
#add_subdirectory(glfw-3.3.8/deps/glad)
#add_subdirectory(nanosvg-master)

add_library(GLAD src/glad.c)


add_executable(${PROJECT_NAME}_node 
	src/projection.cpp
	src/projection.h
#	lodepng/lodepng.cpp
#	lodepng/lodepng.h
	# nanosvg-master/src/nanosvg.h
	# nanosvg-master/src/nanosvgrast.h
	src/glad.c
	src/glad/glad.h
	src/glm/glm.hpp
  # src/Eigen/Eigen
  src/Eigen/Dense
	src/learnopengl/camera.h
	src/learnopengl/shader_m.h
  src/pugixml/pugixml.cpp
  src/pugixml/pugixml.hpp
	)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
target_link_libraries(${PROJECT_NAME}_node 
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	${IL_LIBRARY} ${ILU_LIBRARY} ${ILUT_LIBRARY}	
	glfw GLAD # glm
  ${OpenCV_LIBRARIES}
)
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
#target_link_libraries(GLEW GLU glut)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
install(TARGETS projection_calibration_node 
  DESTINATION ${PROJECT_BINARY_DIR}/lib
)


## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_projection_calibration.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
