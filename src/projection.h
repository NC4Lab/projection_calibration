
//#define BOOST_BIND_NO_PLACEHOLDERS
//#define APIENTRY __stdcall

//#define BOOST_BIND_GLOBAL_PLACEHOLDERS
//#include <boost/bind/bind.hpp>
#include <boost/bind/bind.hpp>
using namespace boost::placeholders;

// Rest of your code

#define GLAPIENTRY APIENTRY
#include "glad/glad.h"
#define GLFW_INCLUDE_NONE 
#include <GLFW/glfw3.h>

//DevIL libraries
#include<IL/il.h>
#include<IL/devil_cpp_wrapper.hpp>
#include<IL/ilu.h>
#include<IL/ilut.h>

// #include <GL/gl.h>
//#include <GL/glcorearb.h>
//#include <GL/glu.h>  // Include GLU library for gluErrorString()
//#include <glfw-3.3.8/deps/glad/gl.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <fstream>
#include <string>
//#include <windows.h>

//#include <gl/GL.h>
//#include <gl/GLU.h>
#include <cstdio>
#include <cstdlib>

#include <iostream>     // std::cout
#include <algorithm>    // std::find
#include <vector>  

//#include "../lodepng/lodepng.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <iostream>
#include <sstream>
#include <regex>
#include <vector>
#include <cmath>
#include "glm/glm.hpp"
#include "glm/ext.hpp"
#include "glm/gtx/string_cast.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "learnopengl/shader_m.h"
#include "learnopengl/camera.h" 
using namespace glm;
using namespace std;
#include <iostream>
#include <bitset>


#include "../nanosvg-master/src/nanosvg.h"
#include "../nanosvg-master/src/nanosvgrast.h"
//#include <GL/gl.h>

// typedef XmlRpc::XmlRpcValue XmlRpcValue;
// typedef std::vector<XmlRpc::XmlRpcValue> Row;
// typedef std::vector<std::vector<XmlRpc::XmlRpcValue>> XmlRpcArray;

float r = 140.0f / 255.0f;
float g = 72.0f / 255.0f;
float b = 159.0f / 255.0f;

//#include "C://GL/GLAD/src/"
//#include "../glfw-3.3.8/deps/glad/gl.h"
//#include "../glfw-3.3.8/deps/glad/vk_platform.h"
//#include "../glfw-3.3.8/deps/glad/vulkan.h"
//#include <GL/glut.h>


//void reshape(int, int);
//void quitVisualScene();
////void toggleFullscreen();
//void keyPressed(unsigned char, int, int);
//void init();
//void display();
//void err_callback();
void saveCoordinates();
int main(int, char**);


//void loadPNG(const char* filename);


