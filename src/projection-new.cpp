#include <ros/ros.h>
#include <ros/package.h>

#include "projection.h"
//
 //Define window size
const int WIDTH = 3840;
const int HEIGHT = 2160;


Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
float mouseOffset = 1.0f;
float zoomOffset = 1.0f;


GLuint fbo;
GLuint fboTexture;

//Monitor related
GLFWwindow* window;
GLFWmonitor* monitor = NULL;
int monitorNumber = 0;
GLFWmonitor** monitors;
int monitor_count;

//camera stuff
glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f, 3.0f);
glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);

// timing
float deltaTime = 0.0f;	// time between current frame and last frame
float lastFrame = 0.0f;


// Define octagon radius and height
const float RADIUS = 0.5f;
//z component of the wall
int Z = 0;

// Define the number of octagons in a row and column
const int ROWS = 1;
const int COLUMNS = 1;
const int NUMBER_OCT_VERTICES = 8;
const int NUMBER_WALL_VERTICES = 4;
const int NUMBER_WALLS = 1;
const float WALL_HEIGHT = 0.5f;
const float OCTAGON_SIZE = 0.5f;

//last one is x,y,z
GLfloat wall_vertices[ROWS][COLUMNS][NUMBER_WALLS][NUMBER_WALL_VERTICES][3];
GLfloat octagon_vertices[ROWS][COLUMNS][NUMBER_OCT_VERTICES][3];



const float ANGLE_STEP = 360.0f / NUMBER_OCT_VERTICES;
// Define the vertices for the vertical walls of an octagon
const int NUM_VERTICES_WALL = 4;





//make things efficient using indices to reduce duplicate rendering
// Define the indices for an octagon
const int NUM_INDICES_OCTAGON = 24;
GLuint indices_octagon[NUM_INDICES_OCTAGON] = {
    0, 1, 2,
    0, 2, 3,
    0, 3, 4,
    0, 4, 5,
    0, 5, 6,
    0, 6, 7,
    0, 7, 8,
    0, 8, 1
};

// Define the shader program source code
const char* vertexShaderSource = R"glsl(
    #version 330 core

    layout (location = 0) in vec3 aPos;
	layout (location = 1) in vec2 aTexCoord;

	out vec2 TexCoord;

    uniform mat4 model;
    uniform mat4 view;
    uniform mat4 projection;

    void main()
    {
        gl_Position = projection * view * model * vec4(aPos, 1.0);
		TexCoord = aTexCoord;
    }
)glsl";

const char* fragmentShaderSource = R"glsl(
    #version 330 core

    out vec4 FragColor;

    uniform vec3 color;

	in vec2 TexCoord;

	uniform sampler2D texture1;

    void main()
    {
         FragColor = texture(texture1, TexCoord);
    }
)glsl";



// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow* window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_ENTER) == GLFW_PRESS)
        ROS_ERROR("processing input");


    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
		camera.ProcessMouseMovement(0,mouseOffset);
    }


    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
		camera.ProcessMouseMovement(0,-mouseOffset);
    }

    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
		camera.ProcessMouseMovement(mouseOffset, 0);
    }


    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
		camera.ProcessMouseMovement(-mouseOffset, 0);
    }


    if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_RELEASE) {
		camera.ProcessMouseScroll(mouseOffset);
    }

    if (glfwGetKey(window, GLFW_KEY_X) == GLFW_RELEASE) {
		camera.ProcessMouseScroll(-mouseOffset);
    }


    //float xpos = static_cast<float>(xposIn);
    //float ypos = static_cast<float>(yposIn);

    //if (firstMouse)
    //{
    //    lastX = xpos;
    //    lastY = ypos;
    //    firstMouse = false;
    //}

    //float xoffset = xpos - lastX;
    //float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    //lastX = xpos;
    //lastY = ypos;

    //camera.ProcessMouseMovement(xoffset, yoffset);

}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(static_cast<float>(yoffset));
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {

    //if (key == GLFW_KEY_1 && action == GLFW_RELEASE) {
    //    selectedSquare = 0;
    //}
    //if (key == GLFW_KEY_2 && action == GLFW_RELEASE) {
    //    selectedSquare = 1;
    //}
    //if (key == GLFW_KEY_3 && action == GLFW_RELEASE) {
    //    selectedSquare = 2;
    //}
    //if (key == GLFW_KEY_4 && action == GLFW_RELEASE) {
    //    selectedSquare = 3;
    //}


    //// Listen for arrow key input to move selected square
    //if (key == GLFW_KEY_LEFT && action == GLFW_RELEASE) {
    //    squarePositions[selectedSquare][0] -= 0.01f;
    //}

    ////@rony: fix the stuff below
    //if (key == GLFW_KEY_RIGHT && action == GLFW_RELEASE) {
    //    squarePositions[selectedSquare][0] += 0.01f;
    //}
    //if (key == GLFW_KEY_UP && action == GLFW_RELEASE) {
    //    squarePositions[selectedSquare][1] += 0.01f;
    //}
    //if (key == GLFW_KEY_DOWN && action == GLFW_RELEASE) {
    //    squarePositions[selectedSquare][1] -= 0.01f;
    //}

    //if (key == GLFW_KEY_ENTER && action == GLFW_RELEASE) {
    //    saveCoordinates();
    //}


    if (key == GLFW_KEY_F && action == GLFW_RELEASE) {
        monitors = glfwGetMonitors(&monitor_count);
        //GLFWmonitor* monitor = glfwGetWindowMonitor(window);
        //const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        //// Create a window
        ////GLFWwindow* window = glfwCreateWindow(mode->width, mode->height, "My Window", monitor, NULL);

        //// Make the window fullscreen
        ////int count;
        ////GLFWmonitor** monitors = glfwGetMonitors(&count);
        //glfwSetWindowMonitor(window, NULL, 0, 0, mode->width, mode->height, mode->refreshRate);



        // find the second monitor (index 1) by checking its position
        for (int i = 0; i < monitor_count; i++) {
            const GLFWvidmode* mode = glfwGetVideoMode(monitors[i]);
            int monitor_x, monitor_y;
            glfwGetMonitorPos(monitors[i], &monitor_x, &monitor_y);
            if (monitor_x != 0 || monitor_y != 0) {
                monitorNumber = i;
                monitor = monitors[i];
                break;
            }
        }

        // make the window full screen on the second monitor
        if (monitor) {
            const GLFWvidmode* mode = glfwGetVideoMode(monitor);
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
        }

    }

    if (key == GLFW_KEY_M && action == GLFW_RELEASE) {
        monitors = glfwGetMonitors(&monitor_count);
        monitorNumber++;
        monitor = monitors[monitorNumber % monitor_count];
        if (monitor) {
            const GLFWvidmode* mode = glfwGetVideoMode(monitor);
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
        }

    }
}

void drawWall(int i,int j,int k) {

    glBegin(GL_QUADS);

    //right now the number of vertices for each wall is hard coded (4 atm)

    //glTexCoord2f(x, y);
    glTexCoord2f(0.0f, 0.0f);
    glVertex3fv(wall_vertices[i][j][k][0]);

    //glTexCoord2f(x+texWidth, y);
    glTexCoord2f(1.0f, 0.0f);
    glVertex3fv(wall_vertices[i][j][k][1]);

    //glTexCoord2f(x+texWidth, y+texHeight);
    glTexCoord2f(1.0f, 1.0f);
    glVertex3fv(wall_vertices[i][j][k][2]);

    //glTexCoord2f(x, y+texHeight);
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
			drawWall(i, j, k);

    }

    //glBegin(GL_QUADS);
    //// draw the bottom face
    ////for (int i = 0; i < 8; i++)
    ////{
    ////    float angle = i * 2 * M_PI / 8;
    ////    glVertex3f(x + size * cos(angle), y + size * sin(angle), z);
    ////}
    //// draw the walls
    //for (int i = 0; i < 8; i++)
    //{
    //    float angle1 = (i + 0.5) * 2 * M_PI / 8;
    //    float angle2 = (i + 1.5) * 2 * M_PI / 8;
    //    glVertex3f(x + size * cos(angle1), y + size * sin(angle1), z);
    //    glVertex3f(x + size * cos(angle2), y + size * sin(angle2), z);
    //    glVertex3f(x + size * cos(angle2), y + size * sin(angle2), z + height);
    //    glVertex3f(x + size * cos(angle1), y + size * sin(angle1), z + height);
    //}
    //// draw the top face
    ////for (int i = 0; i < 8; i++)
    ////{
    ////    float angle = i * 2 * M_PI / 8;
    ////    glVertex3f(x + size * cos(angle), y + size * sin(angle), z + height);
    ////}
    //glEnd();
}

void drawMaze() {
    for (int i = 0; i < ROWS; i++)
    {

        for (int j = 0; j < COLUMNS; j++)
        {
            drawOctagon(i, j);
        }
    }
}

int main(int argc, char** argv)
{
    const float offset_angle = 22.5f;
    // for future stuff
   for (int i = 0; i < ROWS; i++) {

       for (int j = 0; j < COLUMNS; j++) {

			for (int k = 0; k < NUMBER_WALLS; k++) {

				//for (int l = 0; l < NUMBER_WALL_VERTICES; l++) {
                       
     //                for (int m = 0; m < 3) {

     //                   /* wall_vertices[i][j][k][l][m]*/
					//	vertices_octagon[index + k * 3] = RADIUS * std::cos(angle * M_PI / 180.0) + x_offset;
					//vertices_octagon[index + k * 3 + 1] = RADIUS * std::sin(angle * M_PI / 180.0f) + y_offset;
					//	vertices_octagon[index + k * 3 + 2] = HEIGHT_WALL;
					//	{

                            // calculating all the vertices on the wall
							float angle1 = (k + 0.5) * 2 * M_PI / 8;
							float angle2 = (k + 1.5) * 2 * M_PI / 8;
                            
                            //z and a few other variables should be initialized at the very top

                            wall_vertices[i][j][k][0][0] = (float) i + OCTAGON_SIZE * cos(angle1);
                            wall_vertices[i][j][k][0][1] = (float) j + OCTAGON_SIZE * sin(angle1);
                            wall_vertices[i][j][k][0][2] = (float) Z;

                            wall_vertices[i][j][k][1][0] =(float) i + OCTAGON_SIZE * cos(angle2);
                            wall_vertices[i][j][k][1][1] =(float) j + OCTAGON_SIZE * sin(angle2);
                            wall_vertices[i][j][k][1][2] =(float) Z;

                            wall_vertices[i][j][k][2][0] =(float) i + OCTAGON_SIZE * cos(angle2);
                            wall_vertices[i][j][k][2][1] =(float) j + OCTAGON_SIZE * sin(angle2);
                            wall_vertices[i][j][k][2][2] =(float) Z + (float) WALL_HEIGHT;

                            wall_vertices[i][j][k][3][0] =(float) i + OCTAGON_SIZE * cos(angle1);
                            wall_vertices[i][j][k][3][1] =(float) j + OCTAGON_SIZE * sin(angle1);
                            wall_vertices[i][j][k][3][2] =(float) Z + (float) WALL_HEIGHT;


							//glVertex3f(x + size * cos(angle2), y + size * sin(angle2), z);
							//glVertex3f(x + size * cos(angle2), y + size * sin(angle2), z + height);
							//glVertex3f(x + size * cos(angle1), y + size * sin(angle1), z + height);
						//}


                     }
				}
			}

 //   wall_vertices[0][0][0][0][0] = 0.0f;
 //   wall_vertices[0][0][0][0][1] = 0.0f;
 //   wall_vertices[0][0][0][0][2] = 0.0f;

 //   
 //   wall_vertices[0][0][0][1][0] = 0.1f;
	//wall_vertices[0][0][0][1][1] = 0.0f;
	//wall_vertices[0][0][0][1][2] = 0.0f;

 //   wall_vertices[0][0][0][2][0] = 0.1f;
	//wall_vertices[0][0][0][2][1] = 0.1f;
	//wall_vertices[0][0][0][2][2] = 0.0f;

 //   wall_vertices[0][0][0][3][0] = 0.0f;
	//wall_vertices[0][0][0][3][1] = 0.1f;
	//wall_vertices[0][0][0][3][2] = 0.0f;

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



    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    // Create a windowed mode window and its OpenGL context
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window = glfwCreateWindow(WIDTH, HEIGHT, "GLFW 4K Window", NULL, NULL);

    // Set the window as the current OpenGL context
    glfwMakeContextCurrent(window);
    ROS_ERROR("window ran");

    gladLoadGL();
    glfwSwapInterval(1);
    glfwSetKeyCallback(window, keyCallback);

    //image textures



    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    glGenTextures(1, &fboTexture);
    glBindTexture(GL_TEXTURE_2D, fboTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, WIDTH, HEIGHT, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fboTexture, 0);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);



    glEnable(GL_DEPTH_TEST);
    Shader ourShader(vertexShaderSource, fragmentShaderSource);
    glfwMakeContextCurrent(window);
    


    // Set up the vertex buffer objects (VBOs) and element buffer objects (EBOs)
    GLuint VBO_wall, VAO_wall;  //, EBO_octagon, VBO_octagon, VAO_octagon;
    glGenBuffers(1, &VBO_wall);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_wall);
    glBufferData(GL_ARRAY_BUFFER, sizeof(wall_vertices), wall_vertices, GL_STATIC_DRAW);


    glGenVertexArrays(1, &VAO_wall);
    glBindVertexArray(VAO_wall);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_wall);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (void*)0);
    glEnableVertexAttribArray(0);
    //glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), (void*)(3 * sizeof(GLfloat)));
    //glEnableVertexAttribArray(1);


    //GLuint texture;
    //glGenTextures(1, &texture);
    //glBindTexture(GL_TEXTURE_2D, texture); // All upcoming GL_TEXTURE_2D operations now have effect on this texture object
    //// Set the texture wrapping parameters
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    //// Set texture filtering parameters
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    //// Load image, create texture and generate mipmaps
    //glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ilGetInteger(IL_IMAGE_WIDTH),
    //        ilGetInteger(IL_IMAGE_HEIGHT), 0, GL_RGB,
    //        GL_UNSIGNED_BYTE, ilGetData());
    //glGenerateMipmap(GL_TEXTURE_2D);

    //glDeleteShader(vertexShader);
    //glDeleteShader(fragmentShader);

    ////// Set up the model, view, and projection matrices  - do we need these (till line 287?)
    //glm::mat4 model = glm::mat4(1.0f);
    //glm::mat4 view = glm::lookAt(glm::vec3(0.0f, 0.0f, 10.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    //glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)WIDTH / (float)HEIGHT, 0.1f, 100.0f);

    //////// Set the uniform variables for the shader program
    //glUseProgram(shaderProgram);
    //glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
    //glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
    //glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

    //// Define the color for the walls of the octagons
    GLfloat wallColor[] = { 1, 0, 0 };

    // Activate the shader program
    ourShader.use();
    //ourShader.setInt("texture1", 0); // set the texture unit to 0
    glBindTexture(GL_TEXTURE_2D, fboTexture);
    glBindVertexArray(VAO_wall);
    //ourShader.setVec3("colour", 1,0,0);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    ////// Set up the rendering loop
    while (!glfwWindowShouldClose(window)) {

        //for keyboard delta-time
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        processInput(window);
        //    // Clear the screen to a light gray color
        glClearColor(0.8f, 0.8f, 0.8f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


        //@AD: work on integrating those million different shading related things.


        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)WIDTH / (float)HEIGHT, 0.1f, 100.0f);
        ourShader.setMat4("projection", projection);

        // camera/view transformation
        //glm::mat4 view = camera.GetViewMatrix();
        //ourShader.setMat4("view", view);

		glm::mat4 model = glm::mat4(1.0f); 
        ourShader.setMat4("model", model);

		//// Load image data into texture
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ilGetInteger(IL_IMAGE_WIDTH),
            ilGetInteger(IL_IMAGE_HEIGHT), 0, GL_RGB, GL_UNSIGNED_BYTE, ilGetData());

		// Enable texture mapping
		glEnable(GL_TEXTURE_2D);



        //    // Set the uniform variables for the shader program
        //glUseProgram(shaderProgram);
 /*     glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
*/
        //    // Draw the octagons
        //glBindVertexArray(VAO_octagon);
        //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_octagon);
        //glDrawElements(GL_TRIANGLES, sizeof(indices_octagon), GL_UNSIGNED_INT, 0);


        //bind wall image texture, not sure if it's placed correctly here
        //glBindTexture(GL_TEXTURE_2D, texture);

        // Draw the walls of the octagons
        //glColor(0.5, 0.5, 0.5);
        ourShader.use();
        //ourShader.setInt("texture1", 0); // set the texture unit to 0
        glBindTexture(GL_TEXTURE_2D,fboTexture);
        glBindVertexArray(VAO_wall);

        //glUseProgram(shaderProgram);
     //   /*glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
   //     glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
 //       glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));*/
        //glUniform3fv(glGetUniformLocation(ourShader, "color"), 1, wallColor);

        //ourShader.use();
        //ourShader.setInt("texture1", 0); // Set the texture uniform to 0 (GL_TEXTURE0)
        glm::mat4 view = camera.GetViewMatrix(); //glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);

        //ourShader.setMat4("view", view);

        float angle = 20.0f;
        view = glm::rotate(view, glm::radians(angle), glm::vec3(0.0f, 1.0f, 0.0f));

        // pass the view matrix to the shader
        ourShader.setMat4("view", view);



        //glDrawArrays(GL_TRIANGLES, 0, 3);
        //drawWall(0, 0, 0);
   //     for (int i = 0; i < 8; i++) {
   //         for (int j = 0; j < 8; j++)
   //         {
   //            
			//drawOctagon((float) i, (float) j, 1, 0.5, 0.5);
   //         }
   //     }

        drawMaze();

        //    // Swap the front and back buffers
        glfwSwapBuffers(window);

        //    // Poll for and process events
        glfwPollEvents();
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    ////// Clean up
    //glDeleteBuffers(1, &VBO_octagon);
    glDeleteBuffers(1, &VBO_wall);
    //glDeleteVertexArrays(1, &VAO_octagon);
    glDeleteVertexArrays(1, &VAO_wall);
    //glDeleteBuffers(1, &EBO_octagon);
    //glDeleteProgram(shaderProgram);

    ////// Terminate GLFW
    glfwTerminate();
    return 0; 
}

// Define the model, view, and projection matrices //glm::mat4 model = glm::mat4(1.0f);
//  //glm::mat4 view = glm::lookAt(glm::vec3(0.0f, 0.0f, 10.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f)); 
// //glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)WIDTH / (float)HEIGHT, 0.1f, 100.0f); // 
// //// Set the shader program uniforms //GLint modelLoc = glGetUniformLocation(shaderProgram, "model"); //GLint viewLoc = glGetUniformLocation(shaderProgram, "view"); //GLint projectionLoc = glGetUniformLocation(shaderProgram, "projection"); //GLint colorLoc = glGetUniformLocation(shaderProgram, "color");
//
//glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
//glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
//glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projection));
//
//// Set the color uniform for the walls
//glUniform3f(colorLoc, 0.5f, 0.5f, 0.5f);
//
//// Render the vertical walls
//glBindVertexArray(VAO_wall);
//glDrawArrays(GL_TRIANGLE_STRIP, 0, NUM_VERTICES_WALL);
//
//// Set the color uniform for the octagons
//glUniform3f(colorLoc, 1.0f, 0.5f, 0.0f);
//
//// Render the octagons
//glBindVertexArray(VAO_octagon);
//glDrawElementsInstanced(GL_TRIANGLES, NUM_INDICES_OCTAGON, GL_UNSIGNED_INT, 0, ROWS* COLUMNS);

// Clean up the buffers and exit
//glDeleteVertexArrays(1, &VAO_octagon);
//glDeleteBuffers(1, &VBO_octagon);
//glDeleteBuffers(1, &EBO_octagon);
//
//glDeleteVertexArrays(1, &VAO_wall);
//glDeleteBuffers(1, &VBO_wall);
//
//glfwTerminate();
//return 0;
//}


//GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
//glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
//glCompileShader(vertexShader);
//int success;
//char infoLog[512];
//glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
//if (!success) {
//    glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
//    std::cerr << "Error compiling vertex shader:\n" << infoLog << std::endl;
//}
//
//GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
//glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
//glCompileShader(fragmentShader);
//
//glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
//if (!success) {
//    glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
//    std::cerr << "Error compiling fragment shader:\n" << infoLog << std::endl;
//}
//
//GLuint shaderProgram = glCreateProgram();
//glAttachShader(shaderProgram, vertexShader);
//glAttachShader(shaderProgram, fragmentShader);
//glLinkProgram(shaderProgram);
//
//glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
//if (!success) {
//    glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
//    std::cerr << "Error linking shader program:\n" << infoLog << std::endl;
//}
//
//glDeleteShader(vertexShader);
//glDeleteShader(fragmentShader);
//
//// Set up the camera projection matrix
//float aspect_ratio = (float)WIDTH / HEIGHT;
//glm::mat4 projection_matrix = glm::perspective(glm::radians(45.0f), aspect_ratio, 0.1f, 100.0f);
//
//// Set up the camera view matrix
//glm::mat4 view_matrix = glm::lookAt(
//    glm::vec3(0.0f, 0.0f, 3.0f),
//    glm::vec3(0.0f, 0.0f, 0.0f),
//    glm::vec3(0.0f, 1.0f, 0.0f)
//);
//
//// Set up the render loop
//while (!glfwWindowShouldClose(window)) {
//    // Process inputs
//    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
//        glfwSetWindowShouldClose(window, true);
//    }
//
//    // Set the background color
//    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//
//    // Activate the shader program
//    glUseProgram(shaderProgram);
//
//    // Draw the octagons
//    glBindVertexArray(VAO_octagon);
//    glm::mat4 model_matrix_octagon = glm::mat4(1.0f);
//
//    for (int i = 0; i < ROWS; ++i) {
//        for (int j = 0; j < COLUMNS; ++j) {
//            // Calculate the position of the octagon
//            float x = (2.0f * j - (COLUMNS - 1)) * 1.2f;
//            float y = (2.0f * i - (ROWS - 1)) * 1.2f;
//            glm::vec3 position(x, y, 0.0f);
//
//            // Calculate the model matrix for the octagon
//            model_matrix_octagon = glm::translate(glm::mat4(1.0f), position);
//
//            // Set the model, view, and projection matrices as uniforms in the shader program
//            glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model_matrix"), 1, GL_FALSE, glm::value_ptr(model_matrix_octagon));
//            glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view_matrix"), 1, GL_FALSE, glm::value_ptr(view_matrix));
//            glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection_matrix"), 1, GL_FALSE, glm::value_ptr(projection_matrix));
//            glDrawArrays(GL_TRIANGLE_FAN, 0, 8);
//        }
//    }
//
//    // Swap the front and back buffers
//    glfwSwapBuffers(window);
//
//    // Poll for and process events
//    glfwPollEvents();
//    // Clean up
//    glDeleteVertexArrays(1, &VAO_octagon);
//    glDeleteBuffers(1, &VBO_octagon);
//    glDeleteProgram(shaderProgram);
//
//    // Terminate GLFW
//    glfwTerminate();
//    return 0;
//}

//const int WIDTH = 800;
//const int HEIGHT = 600;
//
//void drawOctagon(float x, float y, float z, float size, float height)
//{
//    glBegin(GL_QUADS);
//    // draw the bottom face
//    for (int i = 0; i < 8; i++)
//    {
//        float angle = i * 2 * M_PI / 8;
//        glVertex3f(x + size * cos(angle), y + size * sin(angle), z);
//    }
//    // draw the walls
//    for (int i = 0; i < 8; i++)
//    {
//        float angle1 = i * 2 * M_PI / 8;
//        float angle2 = (i + 1) * 2 * M_PI / 8;
//        glVertex3f(x + size * cos(angle1), y + size * sin(angle1), z);
//        glVertex3f(x + size * cos(angle2), y + size * sin(angle2), z);
//        glVertex3f(x + size * cos(angle2), y + size * sin(angle2), z + height);
//        glVertex3f(x + size * cos(angle1), y + size * sin(angle1), z + height);
//    }
//    // draw the top face
//    for (int i = 0; i < 8; i++)
//    {
//        float angle = i * 2 * M_PI / 8;
//        glVertex3f(x + size * cos(angle), y + size * sin(angle), z + height);
//    }
//    glEnd();
//}
//
//void drawFlatSurface(int rows, int cols, float size, float height)
//{
//    float xOffset = -(cols - 1) * size / 2;
//    float yOffset = -(rows - 1) * size / 2;
//    for (int i = 0; i < rows; i++)
//    {
//        for (int j = 0; j < cols; j++)
//        {
//            float x = j * size + xOffset;
//            float y = i * size + yOffset;
//            drawOctagon(x, y, 0, size, height);
//        }
//    }
//}
//
//int main(int argc, char** argv)
//{
//        ros::init(argc, argv, "Projection", ros::init_options::AnonymousName);
//    ros::NodeHandle n;
//    ros::NodeHandle nh("~");
//    ROS_ERROR("main ran");
//
//
//    GLFWwindow* window;
//
//    if (!glfwInit())
//        return -1;
//
//    window = glfwCreateWindow(WIDTH, HEIGHT, "OpenGL Without GLEW", NULL, NULL);
//    if (!window)
//    {
//        glfwTerminate();
//        return -1;
//    }
//    gladLoadGL();
//
//    glfwMakeContextCurrent(window);
//
//    glMatrixMode(GL_PROJECTION);
//    glLoadIdentity();
//    glOrtho(-WIDTH / 2, WIDTH / 2, -HEIGHT / 2, HEIGHT / 2, -1000, 1000);
//
//    glMatrixMode(GL_MODELVIEW);
//    glLoadIdentity();
//    glTranslatef(0, 0, -500);
//
//    glEnable(GL_DEPTH_TEST);
//
//    while (!glfwWindowShouldClose(window))
//    {
//        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//
//        drawFlatSurface(7, 7, 50, 20);
//
//        glfwSwapBuffers(window);
//        glfwPollEvents();
//    }
//
//    glfwTerminate();
//    return 0;
//}
//
