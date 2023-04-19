#include <ros/ros.h>
#include "projection.h"

using namespace std;
float squarePositions[4][2] = {
    {0.0f, 0.0f},   // top-left square
    {0.1f, 0.1f},   // top-right square
    {-0.1f, 0.1f},  // bottom-left square
    {-0.1f, -0.1f}  // bottom-right square
};
int selectedSquare = 0;

// Callback function for handling window resize events
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

void checkGLError()
{
    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR) {
        ROS_ERROR("OpenGL error: %d", static_cast<int>(err));
    }
}

static void error_callback(int error, const char* description)
{
    ROS_ERROR("Error: %s\n", description);
}

void drawSquare(float x, float y, float size) {
    glRectf(x, y, x + size, y + size);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "Projection", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
	ROS_ERROR("main ran");
     glfwSetErrorCallback(error_callback);

    if (!glfwInit()) {
        ROS_ERROR("glfw init issue");
        return -1;
    }
    // Create a window with a 4K resolution (3840x2160)
    int width = 3840; 
    int height = 2160;
    GLFWwindow* window = glfwCreateWindow(width,height, "GLFW 4K Window", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    // Set the window as the current OpenGL context
    glfwMakeContextCurrent(window);
	ROS_ERROR("window ran");

    gladLoadGL();
    glfwSwapInterval(1);

    // Set the window resize callback
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    while (!glfwWindowShouldClose(window))
    {
        //glfwPollEvents();
        // Clear the screen
		//ROS_ERROR("while ran1")

        // Listen for arrow key input to move selected square
        if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
            squarePositions[selectedSquare][0] -= 0.01f;
        }
        if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
            squarePositions[selectedSquare][0] += 0.01f;
        }
        if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
            squarePositions[selectedSquare][1] += 0.01f;
        }
        if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
            squarePositions[selectedSquare][1] -= 0.01f;
        }

        // Draw squares with updated positions
        glClear(GL_COLOR_BUFFER_BIT);

        for (int i = 0; i < 4; i++) {
            //int pixelX = (int)((squarePositions[i][0] + 1.0f) / 2.0f * width);
            //int pixelY = (int)((1.0f - squarePositions[i][1]) / 2.0f * height);

            // Draw square with pixel coordinates
            //glColor3f(1.0f, 0.0f, 0.0f);
           
            if (i == selectedSquare) {
                glColor3f(1.0f, 1.0f, 1.0f);  // selected square is white
            }
            else {
                glColor3f(r,g,b);  // other squares are red
            }
            //drawSquare(pixelX, pixelY, 20.0f);
            drawSquare(squarePositions[i][0], squarePositions[i][1], 0.1f);
        }

        // Swap the buffers
        glfwSwapBuffers(window);
        glfwPollEvents();


        // Exit the loop when the window is closed or escape key is pressed
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS || glfwWindowShouldClose(window))
            break;
    }

    glfwDestroyWindow(window);
    // Terminate GLFW
    glfwTerminate();

    return 0;
}

