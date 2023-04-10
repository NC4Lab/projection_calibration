#include <ros/ros.h>
#include "projection.h"

using namespace std;


// Callback function for handling window resize events
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

//void err_callback() {
//
//    GLenum err;
//    err = glGetError();
//    ROS_ERROR(err);
//}

void checkGLError()
{
    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR) {
        ROS_ERROR("OpenGL error: %d", static_cast<int>(err));
    }
}
//void err_callback() {
//    GLenum err = glGetError();
//    if (err != GL_NO_ERROR) {
//        const char* errorString = reinterpret_cast<const char*>(gluErrorString(err));
//        ROS_ERROR("OpenGL error: %s", errorString);
//    }
//}
//void APIENTRY err_callback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* userParam)
//{
//    // Your error handling code here
//
//    GLenum err = glGetError();
//    if (err != GL_NO_ERROR) {
//    }
//}



int main(int argc, char** argv) {

    ros::init(argc, argv, "Projection", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
	ROS_ERROR("main ran");
    checkGLError();
    //const GLubyte* version = glGetString(GL_VERSION);
    //checkGLError();
    //if (version)
    //{
    //    ROS_ERROR("OpenGL version: %s", reinterpret_cast<const char*>(version));
    //    std::cout << "OpenGL version: " << version << std::endl;
    //}
    //else
    //{
    //    ROS_ERROR("GL TOH NAHI HAI");
    //    //std::cerr << "Failed to get OpenGL version string" << std::endl;
    //}

    //glDebugMessageCallback(err_callback, nullptr);
    // Initialize GLFW

    if (!glfwInit()) {
        ROS_ERROR("glfw init issue");
        return -1;
    }
    // Create a window with a 4K resolution (3840x2160)
    GLFWwindow* window = glfwCreateWindow(3840, 2160, "GLFW 4K Window", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    // Set the window as the current OpenGL context
    glfwMakeContextCurrent(window);
	ROS_ERROR("window ran");

    // Set the window resize callback
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    while (!glfwWindowShouldClose(window))
    {
        //glfwPollEvents();
        // Clear the screen
		ROS_ERROR("while ran1");
        glClear(GL_COLOR_BUFFER_BIT);

		ROS_ERROR("while ran2");
        // Draw 80x80 pixel boxes using OpenGL commands
        glBegin(GL_QUADS);
        glColor3f(1.0f, 0.0f, 0.0f);  // Red color
        glVertex2f(-1.0f, 1.0f);
        glVertex2f(-0.6f, 1.0f);
        glVertex2f(-0.6f, 0.6f);
        glVertex2f(-1.0f, 0.6f);

        glColor3f(0.0f, 1.0f, 0.0f);  // Green color
        glVertex2f(-0.4f, 1.0f);
        glVertex2f(0.0f, 1.0f);
        glVertex2f(0.0f, 0.6f);
        glVertex2f(-0.4f, 0.6f);

        glColor3f(0.0f, 0.0f, 1.0f);  // Blue color
        glVertex2f(0.2f, 1.0f);
        glVertex2f(0.6f, 1.0f);
        glVertex2f(0.6f, 0.6f);
        glVertex2f(0.2f, 0.6f);
        //glEnd();
        // Swap the buffers
        glfwSwapBuffers(window);

        // Wait for events or a short delay (in milliseconds)
        glfwWaitEventsTimeout(0.010);

        // Exit the loop when the window is closed or escape key is pressed
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS || glfwWindowShouldClose(window))
            break;
    }

    // Terminate GLFW
    glfwTerminate();

    return 0;
}

