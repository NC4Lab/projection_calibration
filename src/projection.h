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

// ============= VARIABLES =============

// Constants
const int MAZE_SIZE = 3;

// Variables related to square positions and transformation
int imageNumber = 0;
float squarePositions[4][5] = {
    {-0.8f, 0.8f, 0.02f, 0.02f, 0.0f}, // top-left square
    {0.8f, 0.8f, 0.02f, 0.02f, 0.0f},  // top-right square
    {0.8f, -0.8f, 0.02f, 0.02f, 0.0f}, // bottom-right square
    {-0.8f, -0.8f, 0.02f, 0.02f, 0.0f} // bottom-left square
};
float shearValues[MAZE_SIZE][MAZE_SIZE];
float sizeValues[MAZE_SIZE][MAZE_SIZE];
float configurationValues[3][3][3];
cv::Mat H = cv::Mat::eye(3, 3, CV_32F);
int selectedSquare = 0;

// Variables related to wall properties
float wallWidth = 0.02f;
float wallHeight = 0.02f;
float wallSep = 0.05f;
string changeMode = "pos";
float shearAmount = 0.0f;
vector<cv::Point2f> wallCorners = createRectPoints(0.0f, 0.0f, wallWidth, wallHeight, 0);

// Variables related to image and file paths
string packagePath = ros::package::getPath("projection_calibration");
string configPath;
string windowName;

// List of image file paths
std::vector<std::string> imagePaths = {
    packagePath + "/src/tj.bmp",
    packagePath + "/src/mmCarribean.png",
    // Add more image file paths as needed
};

// Container to hold the loaded images
std::vector<ILuint> imageIDs;

// Variables related to window and OpenGL
int winWidth = 3840;
int winHeight = 2160;
GLFWwindow *window;
GLuint fbo;
GLuint fboTexture;
GLFWmonitor *monitor = NULL;
int monitorNumber = 0;
GLFWmonitor **monitors;
int monitor_count;

ILint texWidth;
ILint texHeight;

// ============= METHODS =============

std::vector<cv::Point2f> createRectPoints(float, float, float, float, float);

void saveCoordinates();
int main(int, char**);

#endif


