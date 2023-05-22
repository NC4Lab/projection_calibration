#include <ros/ros.h>
#include <ros/package.h>
#include "projection.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/hal/interface.h>
// #include "opencv2/highgui.hpp"
// #include <opencv2/imgproc/hal/hal.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

vector<cv::Point2f> createRectPoints(float x0, float y0, float width, float height)
{
    vector<cv::Point2f> rectPoints;
    rectPoints.push_back(cv::Point2f(x0, y0));
    rectPoints.push_back(cv::Point2f(x0, y0 + height));
    rectPoints.push_back(cv::Point2f(x0 + width, y0 + height));
    rectPoints.push_back(cv::Point2f(x0 + width, y0));

    return rectPoints;
}

float wallWidth = 0.02f;
float wallSep = 0.05f;
string changeMode = "pos";
float shearAmount = 0.0f; // Calculate the shear amount based on your requirements
vector<cv::Point2f> wallCorners = createRectPoints(0.0f, 0.0f, wallWidth, wallWidth);

// x,y,width,height
float squarePositions[4][4] = {
    {-0.1f, 0.1f, 0.1f, 0.1f}, // top-left square
    {0.1f, 0.1f, 0.1f, 0.1f},  // top-right square
    {0.1f, -0.1f, 0.1f, 0.1f}, // bottom-right square
    {-0.1f, -0.1f, 0.1f, 0.1f} // bottom-left square
};

cv::Mat H = cv::Mat::eye(3, 3, CV_32F);

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

ILint texWidth, texHeight;

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

void computeHomography()
{
    vector<cv::Point2f> targetCorners;
    vector<cv::Point2f> imageCorners;

    // for (int i=0; i<4; i++) {
    //     targetCorners.push_back(cv::Point2f(squarePositions[i][0], squarePositions[i][1]));

    // }

    // hard coding the specific corners for each of the squares.
    targetCorners.push_back(cv::Point2f(squarePositions[0][0], squarePositions[0][1] + squarePositions[0][3]));
    targetCorners.push_back(cv::Point2f(squarePositions[1][0] + squarePositions[1][2] - wallWidth, squarePositions[1][1] + squarePositions[1][3]));
    targetCorners.push_back(cv::Point2f(squarePositions[2][0] + squarePositions[2][2] - wallWidth, squarePositions[2][1] + wallWidth));
    targetCorners.push_back(cv::Point2f(squarePositions[3][0], squarePositions[3][1] + wallWidth));

    imageCorners = createRectPoints(0.0f, 0.0f, 6.0 * wallSep, 6.0 * wallSep);

    cerr << "target corners " << targetCorners << "\n"
         << "square poisitons " << squarePositions << "\n"
         << "imageCorners " << imageCorners << "\n";

    H = findHomography(imageCorners, targetCorners);
    // H = findHomography(targetCorners, imageCorners);

    // cerr << H;

    // float value = ptMat.at<float>(0, 2);
    // ROS_ERROR("The value is: %f", value);
}

// void changeCameraMode(int dir) {
//     if changeMod
// }

void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods)
{

    if (action == GLFW_RELEASE)
    {
        if (key == GLFW_KEY_1)
        {
            selectedSquare = 0;
        }
        else if (key == GLFW_KEY_2)
        {
            selectedSquare = 1;
        }
        else if (key == GLFW_KEY_3)
        {
            selectedSquare = 2;
        }
        else if (key == GLFW_KEY_4)
        {
            selectedSquare = 3;
        }
        else if (key == GLFW_KEY_ENTER)
        {
            saveCoordinates();
        }
        else if (key == GLFW_KEY_P)
        {
            changeMode = "pos";
        }
        else if (key == GLFW_KEY_D)
        {
            changeMode = "dimensions";
        }
         else if (key == GLFW_KEY_S)
        {
            changeMode = "shear";
        }
        else if (key == GLFW_KEY_F)
        {
            monitors = glfwGetMonitors(&monitor_count);
            // GLFWmonitor* monitor = glfwGetWindowMonitor(window);
            // const GLFWvidmode* mode = glfwGetVideoMode(monitor);

            //// Create a window
            ////GLFWwindow* window = glfwCreateWindow(mode->width, mode->height, "My Window", monitor, NULL);

            //// Make the window fullscreen
            ////int count;
            ////GLFWmonitor** monitors = glfwGetMonitors(&count);
            // glfwSetWindowMonitor(window, NULL, 0, 0, mode->width, mode->height, mode->refreshRate);

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
        else if (key == GLFW_KEY_M)
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
        else if (key == GLFW_KEY_H)
        {
            computeHomography();
        }
    }
    else if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {

        if (changeMode == "pos")
        {

            // Listen for arrow key input to move selected square
            if (key == GLFW_KEY_LEFT)
            {
                squarePositions[selectedSquare][0] -= 0.05f;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                squarePositions[selectedSquare][0] += 0.05f;
            }
            else if (key == GLFW_KEY_UP)
            {
                squarePositions[selectedSquare][1] += 0.05f;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                squarePositions[selectedSquare][1] -= 0.05f;
            }
        }

        if (changeMode == "dimensions")
        {
            if (key == GLFW_KEY_LEFT)
            {
                squarePositions[selectedSquare][2] -= 0.05f;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                squarePositions[selectedSquare][2] += 0.05f;
            }
            else if (key == GLFW_KEY_UP)
            {
                squarePositions[selectedSquare][3] += 0.05f;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                squarePositions[selectedSquare][3] -= 0.05f;
            }
        }

        if (changeMode == "shear"){
            if (key == GLFW_KEY_UP)
            {
                shearAmount += 0.05f;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                shearAmount -= 0.05f;
            }
        }

    }
}

void drawTarget(float x, float y, float targetWidth, float targetHeight)
{
    glBegin(GL_QUADS);

    // in clockwise direction, starting from left bottom
    glVertex2f(x, y);
    glVertex2f(x, y + targetHeight);

    glVertex2f(x + targetWidth, y + targetHeight);
    glVertex2f(x + targetWidth, y);

    glEnd();
}

void drawRect(vector<cv::Point2f> corners)
{
    glBegin(GL_QUADS);

    // glTexCoord2f(x, y);
    glTexCoord2f(0.0f, 0.0f);
    glVertex2f(corners[0].x, corners[0].y);

    // glTexCoord2f(x+texWidth, y);
    glTexCoord2f(1.0f, 0.0f);
    glVertex2f(corners[1].x, corners[1].y);
    // glVertex2f(x+width, y);

    // glTexCoord2f(x+texWidth, y+texHeight);
    glTexCoord2f(1.0f, 1.0f);
    glVertex2f(corners[2].x, corners[2].y);
    // glVertex2f(x+width, y+height);

    // glTexCoord2f(x, y+texHeight);
    glTexCoord2f(0.0f, 1.0f);
    glVertex2f(corners[3].x, corners[3].y);
    // glVertex2f(x, y+height);
    glEnd();
}

void drawWalls()
{

    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            glBindTexture(GL_TEXTURE_2D, fboTexture);
            vector<cv::Point2f> c = wallCorners;

            for (auto it = c.begin(); it != c.end(); it++)
            {
                cv::Point2f p = *it;

                p.x += i * wallSep;
                p.y += j * wallSep;

                // p.x += shearAmount * p.y;
                float data[] = {p.x, p.y, 1}; // apparently that's how it is

                cv::Mat ptMat(3, 1, CV_32F, data);
                cv::Mat ptMat2(3, 1, CV_32F, data);

                H.convertTo(H, ptMat.type());

                // warpPerspective(ptMat2, ptMat, H, ptMat.size());

                ptMat = H * ptMat;

                ptMat /= ptMat.at<float>(2);

                // cerr << "\n" << ptMat;

                it->x = ptMat.at<float>(0, 0);
                it->y = ptMat.at<float>(0, 1);
            }

        for (auto it = c.begin(); it != c.end(); it++) {
            cv::Point2f p = *it;
            p.x += shearAmount * p.y;
            it->x = p.x;
            it->y = p.y;
        }

            drawRect(c);
        }
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Projection", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    ROS_ERROR("main ran");

    // ILuint image;
    // ilInit();
    // iluInit();
    // ilutInit();

    // ilInit();

    // ilutRenderer(ILUT_OPENGL);

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

    // ILubyte* imageData = ilGetData();

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
    GLuint textureID;
    // Set the window resize callback
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // glMatrixMode(GL_MODELVIEW);
    // glLoadIdentity();
    // glTranslatef(0, 0, 0);

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

        // glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        ////glViewport(0, 0, winWidth, winHeight);

        //// Clear the FBO
        // glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        // glClear(GL_COLOR_BUFFER_BIT);

        //      // Draw squares with updated positions
        glClear(GL_COLOR_BUFFER_BIT);
        // glGenTextures(1, &textureID);
        // glBindTexture(GL_TEXTURE_2D, textureID);
        // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

        //// Load image data into texture
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ilGetInteger(IL_IMAGE_WIDTH),
                     ilGetInteger(IL_IMAGE_HEIGHT), 0, GL_RGB,
                     GL_UNSIGNED_BYTE, ilGetData());

        // Enable texture mapping
        glEnable(GL_TEXTURE_2D);

        drawWalls();

        for (int i = 0; i < 4; i++)
        {
            drawTarget(squarePositions[i][0], squarePositions[i][1], squarePositions[i][2], squarePositions[i][3]);
        }

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