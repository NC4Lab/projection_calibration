#include <ros/ros.h>
#include <ros/package.h>

#include "projection.h"

using namespace std;
float squarePositions[4][2] = {
    {-0.1f, 0.1f}, // top-left square
    {0.1f, 0.1f},  // top-right square
    {0.0f, 0.0f},  // bottom-right square
    {-0.1f, -0.1f} // bottom-left square
};
int selectedSquare = 0;
int winWidth = 3840;
int winHeight = 2160;
GLFWwindow *window;
GLuint fbo;
GLuint fboTexture;
GLFWmonitor *monitor = NULL;
int monitorNumber = 0;
GLFWmonitor **monitors;
int monitor_count;

// int array1[8] = {13,19,20,25,26,27,32,33};
// int array2[8] = {191,32,123,32,119,2,183,2};

/**NOTES: 0,0 number refers to the bottom left oct. 6,6 would refer to top right in the number system.
 * However, when the coordinates are caluclated, they're corrected to represent Rebecca's code.
 * So, theoretically, it should correct things automatically.
 */

vector<int> array1{0,6,42};
vector<int> array2{255,1,1};

Camera camera(glm::vec3(0.0f, 0.0f, 200.0f));
float mouseOffset = 1.0f;
float zoomOffset = 1.0f;

// timing
float deltaTime = 0.0f; // time between current frame and last frame
float lastFrame = 0.0f;

float rollOffset = 0.5f;
float upOffset = 0.05f;

// z component of the wall
int Z = 0;

// Define the number of octagons in a row and column
const int ROWS = 7;
const int COLUMNS = 7;
const int NUMBER_OCT_VERTICES = 8;
const int NUMBER_WALL_VERTICES = 4;
const int NUMBER_WALLS = 8;

// last one is x,y,z
GLfloat wall_vertices[ROWS][COLUMNS][NUMBER_WALLS][NUMBER_WALL_VERTICES][3];
bool show_wall[ROWS][COLUMNS][NUMBER_WALLS];
bool show_octagon[ROWS][COLUMNS];
GLfloat octagon_vertices[ROWS][COLUMNS][NUMBER_OCT_VERTICES][3];

// coordinate correction for the maze
float scalingFactor = 30.0f;     // each floating point value in the coordinate system refers 1cm in real world, hopefully.
const float WALL_HEIGHT = 15.0f; // wall height in cm

const float OCTAGON_SIZE = 0.5f * scalingFactor;

// frustum related variables
string cameraMode = "";
int frustumLeft = -2 * scalingFactor;
int frustumRight = 2 * scalingFactor;
int frustumBottom = -2 * scalingFactor;
int frustumTop = 2 * scalingFactor;
int frustumNearVal = 2 * scalingFactor;
int frustumFarVal = 12 * scalingFactor;

// glFrustum(frustumLeft, frustumRight, frustumBottom, frustumTop, frustumNearVal, frustumFarVal);

ILint texWidth, texHeight;

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{

    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (cameraMode == "Frustrum Left and Right")
    {
        ROS_ERROR(cameraMode.c_str());
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            frustumLeft++;
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            frustumLeft--;
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            frustumRight--;
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            frustumRight++;
    }

    if (cameraMode == "Frustum Top and Bottom")
    {
        ROS_ERROR(cameraMode.c_str());
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            frustumTop++;
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            frustumTop--;
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            frustumBottom--;
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            frustumBottom++;
    }

    if (cameraMode == "Frustum Far and Near")
    {
        ROS_ERROR(cameraMode.c_str());
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            frustumFarVal++;
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        {
            if ((frustumFarVal--) == 0)
            {
                ROS_ERROR("Far Val minimum Val");
            }
            else
                frustumFarVal--;
        }
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            if ((frustumNearVal--) == 0)
            {
                ROS_ERROR("Far Val minimum Val");
            }
            else
                frustumNearVal--;
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            frustumNearVal++;
    }

    if (cameraMode == "Camera")
    {
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            camera.ProcessKeyboard(FORWARD, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            camera.ProcessKeyboard(BACKWARD, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            camera.ProcessKeyboard(LEFT, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            camera.ProcessKeyboard(RIGHT, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
            camera.ProcessUp(FORWARD, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
            camera.ProcessUp(BACKWARD, deltaTime);
    }

    if (cameraMode == "Roll")
    {
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        {
            camera.ProcessRoll(rollOffset);
            ROS_ERROR("ROLL INCREASES");
        }
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            camera.ProcessRoll(-rollOffset);

        // if (cameraMode == "Up")
        // {

        // }
        // if (glfwGetKey(window, GLFW_KEY_ENTER) == GLFW_PRESS)
        //     ROS_ERROR("processing input");

        if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
        {
            camera.ProcessMouseMovement(0, mouseOffset);
        }

        if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
        {
            camera.ProcessMouseMovement(0, -mouseOffset);
        }

        if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
        {
            camera.ProcessMouseMovement(mouseOffset, 0);
        }

        if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
        {
            camera.ProcessMouseMovement(-mouseOffset, 0);
        }
    }

    if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_RELEASE)
    {
        camera.ProcessMouseScroll(mouseOffset);
    }

    if (glfwGetKey(window, GLFW_KEY_X) == GLFW_RELEASE)
    {
        camera.ProcessMouseScroll(-mouseOffset);
    }

    // float xpos = static_cast<float>(xposIn);
    // float ypos = static_cast<float>(yposIn);

    // if (firstMouse)
    //{
    //     lastX = xpos;
    //     lastY = ypos;
    //     firstMouse = false;
    // }

    // float xoffset = xpos - lastX;
    // float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    // lastX = xpos;
    // lastY = ypos;

    // camera.ProcessMouseMovement(xoffset, yoffset);
}

void drawWall(int i, int j, int k)
{

    glBegin(GL_QUADS);

    // right now the number of vertices for each wall is hard coded (4 atm)

    // glTexCoord2f(x, y);
    glTexCoord2f(0.0f, 0.0f);
    glVertex3fv(wall_vertices[i][j][k][0]);

    // glTexCoord2f(x+texWidth, y);
    glTexCoord2f(1.0f, 0.0f);
    glVertex3fv(wall_vertices[i][j][k][1]);

    // glTexCoord2f(x+texWidth, y+texHeight);
    glTexCoord2f(1.0f, 1.0f);
    glVertex3fv(wall_vertices[i][j][k][2]);

    // glTexCoord2f(x, y+texHeight);
    glTexCoord2f(0.0f, 1.0f);
    glVertex3fv(wall_vertices[i][j][k][3]);
    glEnd();
}
void drawOctagon(int i, int j)
{
    // for some reason Abhishek thought calling million different functions for the same thing would
    // a brilliant idea

    for (int k = 0; k < NUMBER_WALLS; k++)
    {
        if (show_wall[i][j][k])
        {
            glBindTexture(GL_TEXTURE_2D, fboTexture);
            drawWall(i, j, k);
        }

        // drawWall(i, j, k);
    }
}
#define INT_BITS 8
void drawMaze()
{
    for (int i = 0; i < ROWS; i++)
    {

        for (int j = 0; j < COLUMNS; j++)
        {
            if (show_octagon[i][j])
                drawOctagon(i, j);
        }
    }
}

// Callback function for handling window resize events
void framebuffer_size_callback(GLFWwindow *window, int width, int height)
{
    glViewport(0, 0, width, height);
}

void checkGLError()
{
    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR)
    {
        ROS_ERROR("OpenGL error: %d", static_cast<int>(err));
    }
}

static void error_callback(int error, const char *description)
{
    ROS_ERROR("Error: %s\n", description);
}

void saveCoordinates()
{

    std::ofstream file("calibration_variables.csv", std::ios::app); // Open the file in append mode

    if (file.is_open())
    {
        // Write the variable name and value to the file
        file << "frustumLeft"
             << "," << frustumLeft << "\n"
             << "frustumRight"
             << "," << frustumRight << "\n"
             << "frustumTop"
             << "," << frustumTop << "\n"
             << "frustumBottom"
             << "," << frustumBottom << "\n"
             << "frustumNearVal"
             << "," << frustumNearVal << "\n"
             << "frustumFarVal"
             << "," << frustumFarVal << "\n"
             << "frustumRight"
             << "," << frustumRight << "\n"
             << "Camera Matrix"
             << "," << glm::to_string(camera.GetViewMatrix());
        // << "view matrix"<<","<< camera.GetViewMatrix().x<<camera.GetViewMatrix().y<<camera.GetViewMatrix().z;

        file.close();
        std::cout << "Variable saved to "
                  << "calibration_variables.csv"
                  << " successfully.\n";
    }
    else
    {
        std::cerr << "Unable to open the file "
                  << "calibration_variables.csv"
                  << "\n";
    }
    // returns the pixel position of the bottom left corner of rect
    for (int i = 0; i < 4; i++)
    {
        int pixelX = (int)((squarePositions[i][0] + 1.0f) / 2.0f * winWidth);
        int pixelY = (int)((1.0f - squarePositions[i][1]) / 2.0f * winHeight);
        ROS_ERROR("Square %d position: (%d, %d)\n", i, pixelX, pixelY);
    }
    //       int pixelWidth = (int)(0.1f * winWidth);
    //       int pixelHeight = (int)(0.1f * winHeight);

    // ROS_ERROR("Pixel dimensions, width is: %d and height is %d", pixelWidth, pixelHeight);
}

void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods)
{

    if (key == GLFW_KEY_1 && action == GLFW_RELEASE)
    {
        cameraMode = "Frustum Left and Right";
    }
    if (key == GLFW_KEY_2 && action == GLFW_RELEASE)
    {
        cameraMode = "Frustum Top and Bottom";
    }
    if (key == GLFW_KEY_3 && action == GLFW_RELEASE)
    {
        cameraMode = "Frustum Far and Near";
    }
    if (key == GLFW_KEY_4 && action == GLFW_RELEASE)
    {
        cameraMode = "Camera";
        ROS_ERROR(cameraMode.c_str());
    }

    if (key == GLFW_KEY_5 && action == GLFW_RELEASE)
    {
        cameraMode = "Roll";
        ROS_ERROR(cameraMode.c_str());
    }

    if (key == GLFW_KEY_6 && action == GLFW_RELEASE)
    {
        cameraMode = "Up";
        ROS_ERROR(cameraMode.c_str());
    }

    if (key == GLFW_KEY_ENTER && action == GLFW_RELEASE)
    {
        saveCoordinates();
    }

    if (key == GLFW_KEY_F && action == GLFW_RELEASE)
    {
        monitors = glfwGetMonitors(&monitor_count);

        // find the second monitor (index 1) by checking its position
        for (int i = 0; i < monitor_count; i++)
        {
            const GLFWvidmode *mode = glfwGetVideoMode(monitors[i]);
            int monitor_x, monitor_y;
            glfwGetMonitorPos(monitors[i], &monitor_x, &monitor_y);
            if (monitor_x != 0 || monitor_y != 0)
            {
                monitorNumber = i;
                monitor = monitors[i];
                break;
            }
        }

        // make the window full screen on the second monitor
        if (monitor)
        {
            const GLFWvidmode *mode = glfwGetVideoMode(monitor);
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
        }
    }

    if (key == GLFW_KEY_M && action == GLFW_RELEASE)
    {
        monitors = glfwGetMonitors(&monitor_count);
        monitorNumber++;
        monitor = monitors[monitorNumber % monitor_count];
        if (monitor)
        {
            const GLFWvidmode *mode = glfwGetVideoMode(monitor);
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
        }
    }
}

void drawRect(float x, float y, float width, float height)
{
    glBegin(GL_QUADS);

    glTexCoord2f(0.0f, 0.0f);
    glVertex2f(x, y);

    glTexCoord2f(1.0f, 0.0f);
    glVertex2f(x + width, y);

    glTexCoord2f(1.0f, 1.0f);
    glVertex2f(x + width, y + height);

    glTexCoord2f(0.0f, 1.0f);
    glVertex2f(x, y + height);
    glEnd();
}


unsigned int leftRotate(unsigned int number, unsigned int rotation) {
    rotation = rotation % (sizeof(unsigned int) * 8);  // Ensure rotation is within valid range

    return (number << rotation) | (number >> (INT_BITS - rotation));
}


void fillCoordinates()
{
    for (int i = 0; i < ROWS; i++)
    {

        for (int j = 0; j < COLUMNS; j++)
        {

            // indices being converted into the coordinate system

            int x = (j) * scalingFactor; // correction to be coherent with rebecca's coordinate system.
            int y = (6 - i) * scalingFactor;

            // check if the octagon is in the path array, if it is fill the octagon spot with truth,
            //  else skip the iteration and save time and memory
            int octagonNumber = (i * 6) + (j) + i;
            ROS_ERROR("octagon Number %d", octagonNumber);
            show_octagon[i][j] = false;
            std::vector<int>::iterator it;
            it = std::find(array1.begin(), array1.end(), octagonNumber);
            int wallIndex;
            if (it != end(array1))
            {
                wallIndex = it - array1.begin();
                show_octagon[i][j] = true;
            }
            else
                continue;

            // gonna check for which walls are up and set the truth value for them


            unsigned int leftRotatedNumber = leftRotate(array2[wallIndex],(unsigned int) 3);

            std::bitset<8> binary(leftRotatedNumber);
            std::string binaryString = binary.to_string();

            


            ROS_ERROR("WallIndex Number %d", wallIndex);
            ROS_ERROR("array Number %d", array2[wallIndex]);
            ROS_ERROR("Binary Number ----- ");
            ROS_ERROR(binaryString.c_str());
            for (int k = binary.size() - 1; k >= 0; --k)
            {
                bool bit = binary[k];
                show_wall[i][j][k] = false;

                if (bit)
                {

                    ROS_ERROR("boi is tru %d", k);
                    show_wall[i][j][k] = true;

                    float angle1 = (k + 0.5) * 2 * M_PI / 8;
                    float angle2 = (k + 1.5) * 2 * M_PI / 8;

                    // z and a few other variables should be initialized at the very top

                    wall_vertices[i][j][k][0][0] = (float)x + OCTAGON_SIZE * cos(angle1);
                    wall_vertices[i][j][k][0][1] = (float)y + OCTAGON_SIZE * sin(angle1);
                    wall_vertices[i][j][k][0][2] = (float)Z;

                    wall_vertices[i][j][k][1][0] = (float)x + OCTAGON_SIZE * cos(angle2);
                    wall_vertices[i][j][k][1][1] = (float)y + OCTAGON_SIZE * sin(angle2);
                    wall_vertices[i][j][k][1][2] = (float)Z;

                    wall_vertices[i][j][k][2][0] = (float)x + OCTAGON_SIZE * cos(angle2);
                    wall_vertices[i][j][k][2][1] = (float)y + OCTAGON_SIZE * sin(angle2);
                    wall_vertices[i][j][k][2][2] = (float)Z + WALL_HEIGHT;

                    wall_vertices[i][j][k][3][0] = (float)x + OCTAGON_SIZE * cos(angle1);
                    wall_vertices[i][j][k][3][1] = (float)y + OCTAGON_SIZE * sin(angle1);
                    wall_vertices[i][j][k][3][2] = (float)Z + WALL_HEIGHT;
                }
                else
                    continue;
            }
        }
    }
}

int main(int argc, char **argv)
{

    int startValue = 0;

    // for (int i = 0; i < 20  ; ++i)
    // {
    //     array1.push_back(startValue + i);
    //     array2.push_back(255);
    // }

    fillCoordinates();

    ros::init(argc, argv, "Projection", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    ROS_ERROR("main ran");

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

    glfwSetErrorCallback(error_callback);

    if (!glfwInit())
    {
        ROS_ERROR("glfw init issue");
        return -1;
    }
    // Create a window with a 4K resolution (3840x2160)
    window = glfwCreateWindow(winWidth, winHeight, "GLFW 4K Window", NULL, NULL);
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
    // GLuint textureID;
    // Set the window resize callback
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // Create an FBO and attach the texture to it
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    glGenTextures(1, &fboTexture);
    glBindTexture(GL_TEXTURE_2D, fboTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, winWidth, winHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fboTexture, 0);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    while (!glfwWindowShouldClose(window))
    {

        // for keyboard delta-time
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        processInput(window);

        glClearColor(0.8f, 0.8f, 0.8f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        // glFrustum(-2, 2, -2, 2, 1, 10);
        glFrustum(frustumLeft, frustumRight, frustumBottom, frustumTop, frustumNearVal, frustumFarVal);
        // glFrustum(-50,50,-50,1,1,50);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glLoadMatrixf(glm::value_ptr(camera.GetViewMatrix()));

        glTranslatef(0, 0, -1);

        //// Load image data into texture
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ilGetInteger(IL_IMAGE_WIDTH),
                     ilGetInteger(IL_IMAGE_HEIGHT), 0, GL_RGB,
                     GL_UNSIGNED_BYTE, ilGetData());

        // Enable texture mapping
        glEnable(GL_TEXTURE_2D);

        drawMaze();

        // for (int i = 0; i < 4; i++) {
        //     glBindTexture(GL_TEXTURE_2D, fboTexture);
        //     drawRect(squarePositions[i][0], squarePositions[i][1], 0.1f, 0.1f);
        // }

        // Swap the buffers
        glfwSwapBuffers(window);
        glfwPollEvents();
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

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
