#include <ros/ros.h>
#include "projection.h"
//#include "../dll/*"
//#include"../dll/ILU.dll"
//#include"../dll/DevIL.dll"
//#include"../dll/ILUT.dll"



using namespace std;
float squarePositions[4][2] = {
    {0.0f, 0.0f},   // bottom-right square
    {0.1f, 0.1f},   // top-right square
    {-0.1f, 0.1f},  // top-left square
    {-0.1f, -0.1f}  // bottom-left square
};
int selectedSquare = 0;
int width = 3840; 
int height = 2160;


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

void saveCoordinates() {
	for (int i = 0; i < 4; i++) {
		int pixelX = (int)((squarePositions[i][0] + 1.0f) / 2.0f * width);
		int pixelY = (int)((1.0f - squarePositions[i][1]) / 2.0f * height);
		ROS_ERROR("Square %d position: (%d, %d)\n", i, pixelX, pixelY);
	}

}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {

        if (key == GLFW_KEY_1 && action == GLFW_RELEASE) {
            selectedSquare = 0;
        }
        if (key == GLFW_KEY_2 && action == GLFW_RELEASE) {
            selectedSquare = 1;
        }
        if (key == GLFW_KEY_3 && action == GLFW_RELEASE) {
            selectedSquare = 2;
        }
        if (key == GLFW_KEY_4 && action == GLFW_RELEASE) {
            selectedSquare = 3;
        }


        // Listen for arrow key input to move selected square
        if (key == GLFW_KEY_LEFT && action == GLFW_RELEASE) {
            squarePositions[selectedSquare][0] -= 0.01f;
        }

        //@rony: fix the stuff below
        if (key == GLFW_KEY_RIGHT && action == GLFW_RELEASE) {
            squarePositions[selectedSquare][0] += 0.01f;
        }
        if (key == GLFW_KEY_UP && action == GLFW_RELEASE) {
            squarePositions[selectedSquare][1] += 0.01f;
        }
        if (key == GLFW_KEY_DOWN && action == GLFW_RELEASE) {
            squarePositions[selectedSquare][1] -= 0.01f;
        }

        if (glfwGetKey(window, GLFW_KEY_ENTER) == GLFW_PRESS) {
            saveCoordinates();
        }



}

void drawSquare(float x, float y, float size) {
    // Bind the texture
    glBindTexture(GL_TEXTURE_2D, textureID);

    // Draw a quad with the texture
    glBegin(GL_QUADS);
    glTexCoord2f(x, y);
    glVertex2f(-1, -1);
    glTexCoord2f(x+size, y);
    glVertex2f(1, -1);
    glTexCoord2f(x+size, y+size);
    glVertex2f(1, 1);
    glTexCoord2f(x, y+size);
    glVertex2f(-1, 1);
    glEnd();
}

int main(int argc, char** argv) {

	//NSVGimage *svg = NULL;
	//NSVGrasterizer *rast = NULL;
    ros::init(argc, argv, "Projection", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
	ROS_ERROR("main ran");
    //ILuint image;
    ilInit();
    iluInit();
    ilutInit();

    //ilInit();

    ilutRenderer(ILUT_OPENGL);

    //ilGenImages(1, &image);
    //ilBindImage(image);
    ilLoadImage("tj.png");
    glfwSetErrorCallback(error_callback);
    //svg = nsvgParseFromFile("the_box.svg", "px", 96.0f);

    // Create a rasterizer and set its parameters
    //NSVGrasterizer* rast = nsvgCreateRasterizer();
    //nsvgRasterizerScanline(rast, NULL, NULL, 800, 600);

    if (!glfwInit()) {
        ROS_ERROR("glfw init issue");
        return -1;
    }
    // Create a window with a 4K resolution (3840x2160)
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
    glfwSetKeyCallback(window, keyCallback);


    // Set the window resize callback
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    while (!glfwWindowShouldClose(window))
    {
        //glfwPollEvents();
        // Clear the screen
		//ROS_ERROR("while ran1")


        // Draw squares with updated positions
        glClear(GL_COLOR_BUFFER_BIT);
        //nsvgRasterize(rast, svg, 0, 0, 1.0f, NULL, 0, 0, 800, 600);


        for (int i = 0; i < 4; i++) {
            //int pixelX = (int)((squarePositions[i][0] + 1.0f) / 2.0f * width);
            //int pixelY = (int)((1.0f - squarePositions[i][1]) / 2.0f * height);

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

    //nsvgDelete(svg);
    //nsvgDeleteRasterizer(rast);
    glfwDestroyWindow(window);
    // Terminate GLFW
    glfwTerminate();

    return 0;
}

