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
find_package(OpenCV REQUIRED)
# find_package(OpenGL REQUIRED)

<<<<<<< HEAD
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

=======
>>>>>>> fa00cfd333a24f2b1e6d605079da70bfbb8cfeb8
## Generate messages in the 'msg' folder
add_message_files(
	DIRECTORY
	msg
	FILES
  # Message1.msg
   #Message2.msg
<<<<<<< HEAD
 )
=======
)
>>>>>>> fa00cfd333a24f2b1e6d605079da70bfbb8cfeb8

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
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
#  INCLUDE_DIRS glfw-3.3.8/include
  INCLUDE_DIRS lodepng/
  INCLUDE_DIRS nanosvg-master/
#  LIBRARIES projection_calibration OpenGL
  CATKIN_DEPENDS roscpp roslib std_msgs message_runtime
#  DEPENDS system_lib
)

<<<<<<< HEAD
###########
## Build ##
###########
# TODO - README.md should ask user to install these libraries and create environment variables pointing to them.
set(glfw3_DIR "C:/Program Files (x86)/GLFW/lib/cmake/glfw3/")
set(IL_INCLUDE_DIR "C:/Program Files (x86)/IL/include")
set(IL_LIB_DIR "C:/Program Files (x86)/IL/lib/x64/Release/")
set(glm_DIR "C:/Program Files (x86)/glm/cmake/glm/")

# Find GLFW
find_package(glfw3 REQUIRED)

# Include directories for GLFW and DevIL
include_directories(${glfw3_INCLUDE_DIRS})
include_directories(${IL_INCLUDE_DIR})

# # Link GLFW and specify DevIL libraries manually
# target_link_libraries(your_target ${glfw3_LIBRARIES})
# target_link_libraries(your_target "${IL_LIB_DIR}/DevIL.lib")
# target_link_libraries(your_target "${IL_LIB_DIR}/ILU.lib")
# target_link_libraries(your_target "${IL_LIB_DIR}/ILUT.lib")
=======
>>>>>>> fa00cfd333a24f2b1e6d605079da70bfbb8cfeb8

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  #${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/glfw-3.3.8/include
 # ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/glfw-3.3.8/deps/glad
  # ${glfw3_dir}/../../../include
  ${glfw3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  $ENV{IL_INCLUDE_DIR}
  ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/src/glm
  ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/src/pugixml
 # "C:/Program Files (x86)/IL/include"
 # ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/dll

)

# find_library(IL_LIBRARY NAMES DevIL ILU ILUT.lib PATHS $ENV{IL_LIB_DIR})
# find_library(ILU_LIBRARY NAMES ILU PATHS $ENV{IL_LIB_DIR})
# find_library(ILUT_LIBRARY NAMES ILUT PATHS $ENV{IL_LIB_DIR})

#find_library(ILUT_LIBRARY NAMES ILUT.lib PATH "C:/Program Files (x86)/IL/lib/x64/Release/ILUT.lib")
#find_library(IL_LIBRARY NAMES DevIL ILU ILUT PATHS ${IL_LIB_DIR})
#find_library(DEVIL_LIBRARIES NAMES DevIL ILU ILUT PATHS ${IL_LIBRARY})

# find_library(IL_LIBRARY NAMES DevIL ILU ILUT.lib PATHS "C:/ws/src/projection_calibration/dll")
# find_library(ILU_LIBRARY NAMES ILU PATHS "C:/ws/src/projection_calibration/dll")
# find_library(ILUT_LIBRARY NAMES ILUT PATHS "C:/ws/src/projection_calibration/dll")

# set(IL_LIBRARY "C:/ws/src/projection_calibration/dll/DevIL.dll")
# set(ILU_LIBRARY "C:/ws/src/projection_calibration/dll/ILU.dll")
# set(ILUT_LIBRARY "C:/ws/src/projection_calibration/dll/ILUT.dll")

# find_library(ILUT_LIBRARY NAMES ILUT.lib PATHS "C:/Program Files (x86)/IL/lib/x64/Release/")
# find_library(IL_LIBRARY NAMES DevIL ILU ILUT.lib PATHS "C:/Program Files (x86)/IL/lib/x64/Release/")
# find_library(ILU_LIBRARY NAMES ILU PATHS "C:/Program Files (x86)/IL/lib/x64/Release/")

# # To these lines (Assuming your project root is the workspace root)
find_library(ILUT_LIBRARY NAMES ILUT.lib PATHS "${CMAKE_SOURCE_DIR}/projection_calibration/dll")
find_library(IL_LIBRARY NAMES DevIL ILU ILUT.lib PATHS "${CMAKE_SOURCE_DIR}/projection_calibration/dll")
find_library(ILU_LIBRARY NAMES ILU PATHS "${CMAKE_SOURCE_DIR}/projection_calibration/dll")

if(IL_LIBRARY)
  message(STATUS "Found IL library: ${IL_LIBRARY}")
else()
  message(FATAL_ERROR "IL library not found")
endif()
if(ILU_LIBRARY)
  message(STATUS "Found ILU library: ${ILU_LIBRARY}")
else()
  message(FATAL_ERROR "ILU library not found")
endif()

<<<<<<< HEAD

=======
>>>>>>> fa00cfd333a24f2b1e6d605079da70bfbb8cfeb8
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
<<<<<<< HEAD
	# src/learnopengl/camera.h
	# src/learnopengl/shader_m.h
=======
	src/learnopengl/camera.h
	src/learnopengl/shader_m.h
>>>>>>> fa00cfd333a24f2b1e6d605079da70bfbb8cfeb8
  src/pugixml/pugixml.cpp
  src/pugixml/pugixml.hpp
)

# Copy DevIL.dll to the executable's directory post-build
add_custom_command(TARGET ${PROJECT_NAME}_node POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different
<<<<<<< HEAD
                   ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/dll/DevIL.dll
=======
                   ${CMAKE_SOURCE_DIR}/projection_calibration/dll/DevIL.dll
>>>>>>> fa00cfd333a24f2b1e6d605079da70bfbb8cfeb8
                   $<TARGET_FILE_DIR:${PROJECT_NAME}_node>)

# Copy ILU.dll to the executable's directory post-build
add_custom_command(TARGET ${PROJECT_NAME}_node POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different
<<<<<<< HEAD
                   ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/dll/ILU.dll
=======
                   ${CMAKE_SOURCE_DIR}/projection_calibration/dll/ILU.dll
>>>>>>> fa00cfd333a24f2b1e6d605079da70bfbb8cfeb8
                   $<TARGET_FILE_DIR:${PROJECT_NAME}_node>)

# Copy ILUT.dll to the executable's directory post-build
add_custom_command(TARGET ${PROJECT_NAME}_node POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different
<<<<<<< HEAD
                   ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/dll/ILUT.dll
                   $<TARGET_FILE_DIR:${PROJECT_NAME}_node>)

# set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
# set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
# set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
=======
                   ${CMAKE_SOURCE_DIR}/projection_calibration/dll/ILUT.dll
                   $<TARGET_FILE_DIR:${PROJECT_NAME}_node>)

>>>>>>> fa00cfd333a24f2b1e6d605079da70bfbb8cfeb8

target_link_libraries(${PROJECT_NAME}_node 
	${catkin_LIBRARIES}
	${IL_LIBRARY} ${ILU_LIBRARY} ${ILUT_LIBRARY}	
	glfw GLAD # glm
  ${OpenCV_LIBRARIES}
)
<<<<<<< HEAD

## Print out the values of the environment variables
message("[ENV]: PROJECT_NAME set to: ${PROJECT_NAME}")
message("[ENV]: catkin_LIBRARIES set to: ${catkin_LIBRARIES}")
message("[ENV]: CMAKE_SOURCE_DIR set to: ${CMAKE_SOURCE_DIR}")
message("[ENV]: glfw3_DIR set to: ${glfw3_DIR}")
message("[ENV]: IL_INCLUDE_DIR set to: $ENV{IL_INCLUDE_DIR}") 
message("[ENV]: IL_LIB_DIR set to: $ENV{IL_LIB_DIR}")
message("[ENV]: glm_DIR set to: ${glm_DIR}")
message("[ENV]: IL_LIBRARY set to: ${IL_LIBRARY}")
message("[ENV]: ILU_LIBRARY set to: ${ILU_LIBRARY}")
message("[ENV]: ILUT_LIBRARY set to: ${ILUT_LIBRARY}")
message("[ENV]: OpenCV_INCLUDE_DIRS set to: ${OpenCV_INCLUDE_DIRS}")
message("[ENV]: OpenCV_LIBRARIES set to: ${OpenCV_LIBRARIES}")
message("[ENV]: OpenCV_LIBS set to: ${OpenCV_LIBS}")
message("[ENV]: OpenCV_DIR set to: ${OpenCV_DIR}")

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
=======
>>>>>>> fa00cfd333a24f2b1e6d605079da70bfbb8cfeb8


install(TARGETS projection_calibration_node 
  DESTINATION ${PROJECT_BINARY_DIR}/lib
)

