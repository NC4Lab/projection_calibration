#include <ros/ros.h>
#include <ros/package.h>

#include "projection.h"

using namespace std;
float squarePositions[4][2] = {
    {0.0f, 0.0f},   // bottom-right square
    {0.1f, 0.1f},   // top-right square
    {-0.1f, 0.1f},  // top-left square
    {-0.1f, -0.1f}  // bottom-left square
};
int selectedSquare = 0;
int winWidth = 3840; 
int winHeight = 2160;

ILint texWidth, texHeight;


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
		int pixelX = (int)((squarePositions[i][0] + 1.0f) / 2.0f * winWidth);
		int pixelY = (int)((1.0f - squarePositions[i][1]) / 2.0f * winHeight);
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

void drawRect(float x, float y, float width, float height) {
    glBegin(GL_QUADS);

    //glTexCoord2f(x, y);
    glTexCoord2f(0.0f, 0.0f);
    glVertex2f(x, y);

    //glTexCoord2f(x+texWidth, y);
    glTexCoord2f(1.0f, 0.0f);
    glVertex2f(x+width, y);

    //glTexCoord2f(x+texWidth, y+texHeight);
    glTexCoord2f(1.0f, 1.0f);
    glVertex2f(x+width, y+height);

    //glTexCoord2f(x, y+texHeight);
    glTexCoord2f(0.0f, 1.0f);
    glVertex2f(x, y+height);
    glEnd();

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "Projection", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
	ROS_ERROR("main ran");

    //ILuint image;
    //ilInit();
    //iluInit();
    //ilutInit();

    //ilInit();

    //ilutRenderer(ILUT_OPENGL);

    ILuint ImgId = 0;
    ilGenImages(1, &ImgId);
    ilBindImage(ImgId);


    string packagePath = ros::package::getPath("projection_calibration");

    string texFileName = packagePath + "/src/tj.bmp";

    ROS_ERROR(texFileName.c_str());

    ilLoad(IL_BMP, texFileName.c_str());

    ROS_ERROR("Loading image: %s", iluErrorString(ilGetError()));

    ilConvertImage(IL_RGB, IL_UNSIGNED_BYTE);

    ROS_ERROR("Converting image: %s", iluErrorString(ilGetError()));

    texWidth = ilGetInteger(IL_IMAGE_WIDTH);
    texHeight = ilGetInteger(IL_IMAGE_HEIGHT);

    ROS_ERROR("%d", texWidth);
    ROS_ERROR("%d", texHeight);

    //ILubyte* imageData = ilGetData();

    glfwSetErrorCallback(error_callback);

    if (!glfwInit()) {
        ROS_ERROR("glfw init issue");
        return -1;
    }
    // Create a window with a 4K resolution (3840x2160)
    GLFWwindow* window = glfwCreateWindow(winWidth,winHeight, "GLFW 4K Window", NULL, NULL);
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
    GLuint textureID;

    // Set the window resize callback
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);


    //glMatrixMode(GL_MODELVIEW);
    //glLoadIdentity();
    //glTranslatef(0, 0, 0);


    while (!glfwWindowShouldClose(window))
    {
        // Draw squares with updated positions
        glClear(GL_COLOR_BUFFER_BIT);
		glGenTextures(1, &textureID);
		glBindTexture(GL_TEXTURE_2D, textureID);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

		// Load image data into texture
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ilGetInteger(IL_IMAGE_WIDTH),
            ilGetInteger(IL_IMAGE_HEIGHT), 0, GL_RGB,
            GL_UNSIGNED_BYTE, ilGetData());

		// Enable texture mapping
		glEnable(GL_TEXTURE_2D);



			

        for (int i = 0; i < 4; i++) {
   //         //int pixelX = (int)((squarePositions[i][0] + 1.0f) / 2.0f * width);
   //         //int pixelY = (int)((1.0f - squarePositions[i][1]) / 2.0f * height);

   //         //if (i == selectedSquare) {
   //         //    glColor3f(1.0f, 1.0f, 1.0f);  // selected square is white
   //         //}
   //         //else {
                //glColor3f(r,g,b);  // other squares are red
   //         //}
   //         //drawSquare(pixelX, pixelY, 20.0f);
   //         float x = squarePositions[i][0];
   //         float y = squarePositions[i][1];
   //         float size = 0.1f;
            drawRect(squarePositions[i][0], squarePositions[i][1], 0.1f, 0.2f);
        }

        // Swap the buffers
        glfwSwapBuffers(window);
        glfwPollEvents();


        // Exit the loop when the window is closed or escape key is pressed
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS || glfwWindowShouldClose(window))
            break;
    }

    glfwDestroyWindow(window);
    ilDeleteImages(1, &ImgId);
    // Terminate GLFW
    glfwTerminate();

    return 0;
}

