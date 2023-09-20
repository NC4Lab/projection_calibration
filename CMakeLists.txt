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

## Generate messages in the 'msg' folder
add_message_files(
	DIRECTORY
	msg
	FILES
  # Message1.msg
   #Message2.msg
)

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
	src/learnopengl/camera.h
	src/learnopengl/shader_m.h
  src/pugixml/pugixml.cpp
  src/pugixml/pugixml.hpp
)

# Copy DevIL.dll to the executable's directory post-build
add_custom_command(TARGET ${PROJECT_NAME}_node POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different
                   ${CMAKE_SOURCE_DIR}/projection_calibration/dll/DevIL.dll
                   $<TARGET_FILE_DIR:${PROJECT_NAME}_node>)

# Copy ILU.dll to the executable's directory post-build
add_custom_command(TARGET ${PROJECT_NAME}_node POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different
                   ${CMAKE_SOURCE_DIR}/projection_calibration/dll/ILU.dll
                   $<TARGET_FILE_DIR:${PROJECT_NAME}_node>)

# Copy ILUT.dll to the executable's directory post-build
add_custom_command(TARGET ${PROJECT_NAME}_node POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different
                   ${CMAKE_SOURCE_DIR}/projection_calibration/dll/ILUT.dll
                   $<TARGET_FILE_DIR:${PROJECT_NAME}_node>)


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


install(TARGETS projection_calibration_node 
  DESTINATION ${PROJECT_BINARY_DIR}/lib
)

