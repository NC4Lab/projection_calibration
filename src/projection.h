// ######################################

//========= projection.h ===========

// ######################################
#ifndef _projection_h
#define _projection_h

//============= INCLUDE ================

// // GLAD for OpenGL function pointers
// /// @note GLAD must be included before GLFW
// #include "glad/glad.h"

// // GLFW for window management
// #include <GLFW/glfw3.h>

// OpenGL (GLAD and GLFW) for graphics and windowing
#define GLAPIENTRY APIENTRY
#include "glad/glad.h"
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

// DevIL for image loading and manipulation
#include <IL/il.h>
#include <IL/ilu.h>
#include <IL/ilut.h>
#include <IL/devil_cpp_wrapper.hpp>

// ROS for robot operating system functionalities
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <XmlRpcValue.h>

// Standard Library for various utilities
#include <fstream>
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>

// PugiXML for XML parsing
#include "pugixml.hpp"

// OpenCV for computer vision tasks
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/hal/interface.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

// Namespace declarations
using namespace std;

// ============= METHODS =============

std::vector<cv::Point2f> createRectPoints(float, float, float, float, float);

// Function to load coordinates from an XML file
void loadCoordinates();

// Function to save coordinates to an XML file
void saveCoordinates();

// Function to compute homography matrix
void computeHomography();

// Callback function for handling keyboard input
void callbackKeyBinding(GLFWwindow *, int, int, int, int);

// Callback function for handling window resize events
void callbackFrameBufferSize(GLFWwindow *, int, int);

// Callback function for handling GLFW errors
static void callbackError(int, const char *);

// Function to draw a target
void drawTarget(float, float, float, float);

// Function to draw a rectangle with given corners
void drawRect(vector<cv::Point2f>, int);

// Function to draw multiple wall images
void drawWalls();

// The main function of the program
int main(int, char **);


#endif


