#include <ros/ros.h>
#include <ros/package.h>

#include "projection.h"
//
 //Define window size
const int WIDTH = 3840;
const int HEIGHT = 2160;


Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
float lastX = WIDTH / 2.0f;
float lastY = HEIGHT / 2.0f;
bool firstMouse = true;

// timing
float deltaTime = 0.0f;	// time between current frame and last frame
float lastFrame = 0.0f;


// Define octagon radius and height
const float RADIUS = 0.5f;
const float HEIGHT_WALL = 0.2f;

// Define the number of octagons in a row and column
const int ROWS = 7;
const int COLUMNS = 7;

// Define the vertices for an octagon
const int NUM_VERTICES_OCTAGON = 8;
GLfloat vertices_octagon[NUM_VERTICES_OCTAGON * 3];
const float ANGLE_STEP = 360.0f / NUM_VERTICES_OCTAGON;
// Define the vertices for the vertical walls of an octagon
const int NUM_VERTICES_WALL = 4;
GLfloat vertices_wall[NUM_VERTICES_WALL * 3] = {
    -RADIUS, -RADIUS, 0.0f,
    -RADIUS, -RADIUS, HEIGHT_WALL,
    -RADIUS,  RADIUS, 0.0f,
    -RADIUS,  RADIUS, HEIGHT_WALL
};

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

    uniform mat4 model;
    uniform mat4 view;
    uniform mat4 projection;

    void main()
    {
        gl_Position = projection * view * model * vec4(aPos, 1.0);
    }
)glsl";

const char* fragmentShaderSource = R"glsl(
    #version 330 core

    out vec4 FragColor;

    uniform vec3 color;

    void main()
    {
        FragColor = vec4(color, 1.0f);
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
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}


// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
{
    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);

    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(static_cast<float>(yoffset));
}


int main(int argc, char** argv)
{
    const float offset_angle = 22.5f;
    for (int i = 0; i < NUM_VERTICES_OCTAGON; ++i) {
        float angle = ANGLE_STEP * i;
        vertices_octagon[i * 3] = RADIUS * std::cos(angle * M_PI / 180.0f);
        vertices_octagon[i * 3 + 1] = RADIUS * std::sin(angle * M_PI / 180.0f);
        vertices_octagon[i * 3 + 2] = 0.0f;
    }
    //weird formation.
    //for (int i = 0; i < ROWS; ++i) {
    //    for (int j = 0; j < COLUMNS; ++j) {
    //        int index = (i * COLUMNS + j) * NUM_VERTICES_OCTAGON * 3;
    //        for (int k = 0; k < NUM_VERTICES_OCTAGON; ++k) {
    //            float angle = ANGLE_STEP * k;
    //            vertices_octagon[index + k * 3] = RADIUS * std::cos(angle * M_PI / 180.0) * std::cos((k * 45.0 + offset_angle) * M_PI / 180.0);
    //            vertices_octagon[index + k * 3 + 1] = RADIUS * std::sin(angle * M_PI / 180.0f);
    //            vertices_octagon[index + k * 3 + 2] = HEIGHT_WALL * (i + j);
    //        }
    //    }
    //}

    //attempt for 7x7 formation, 2.5 is the offset between octagons
    for (int i = 0; i < ROWS; ++i) {
        for (int j = 0; j < COLUMNS; ++j) {
            int index = ((i * COLUMNS) + j) * NUM_VERTICES_OCTAGON * 3;
            float x_offset = 0;//(j - 3.0f) * 2.5f * RADIUS;
            float y_offset = 0;// (i - 3.0f) * 2.5f * RADIUS;
            for (int k = 0; k < NUM_VERTICES_OCTAGON; ++k) {
                float angle = ANGLE_STEP * k;
                vertices_octagon[index + k * 3] = RADIUS * std::cos(angle * M_PI / 180.0) + x_offset;
                vertices_octagon[index + k * 3 + 1] = RADIUS * std::sin(angle * M_PI / 180.0f) + y_offset;
                vertices_octagon[index + k * 3 + 2] = HEIGHT_WALL;
            }
        }
    }


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

    GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "GLFW 4K Window", NULL, NULL);

    // Set the window as the current OpenGL context
    glfwMakeContextCurrent(window);
    ROS_ERROR("window ran");

    gladLoadGL();
    glfwSwapInterval(1);
    //glfwSetKeyCallback(window, keyCallback);
    // Make the window's context current

    glEnable(GL_DEPTH_TEST);
    Shader ourShader(vertexShaderSource, fragmentShaderSource);
    glfwMakeContextCurrent(window);
    

    unsigned int texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, WIDTH, HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glGenerateMipmap(GL_TEXTURE_2D);




    // Set up the vertex buffer objects (VBOs) and element buffer objects (EBOs)
    GLuint VBO_octagon, VBO_wall, VAO_octagon, VAO_wall, EBO_octagon;

    glGenBuffers(1, &VBO_octagon);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_octagon);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices_octagon), vertices_octagon, GL_STATIC_DRAW);

    glGenBuffers(1, &VBO_wall);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_wall);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices_wall), vertices_wall, GL_STATIC_DRAW);


    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), (void*)(3 * sizeof(GLfloat)));
    glEnableVertexAttribArray(1);

    glGenVertexArrays(1, &VAO_octagon);
    glBindVertexArray(VAO_octagon);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_octagon);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (void*)0);
    glEnableVertexAttribArray(0);

    glGenVertexArrays(1, &VAO_wall);
    glBindVertexArray(VAO_wall);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_wall);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (void*)0);
    glEnableVertexAttribArray(0);

    glGenBuffers(1, &EBO_octagon);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_octagon);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices_octagon), indices_octagon, GL_STATIC_DRAW);

    //// Set up the shader program
    //GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    //glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    //glCompileShader(vertexShader);

    //GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    //glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    //glCompileShader(fragmentShader);

    //GLuint shaderProgram = glCreateProgram();
    //glAttachShader(shaderProgram, vertexShader);
    //glAttachShader(shaderProgram, fragmentShader);
    //glLinkProgram(shaderProgram);

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
    //ourShader.setMat4("view", view);
    //ourShader.setMat4("projection", projection);

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
        glm::mat4 view = camera.GetViewMatrix();
        ourShader.setMat4("view", view);

		glm:mat4 model = glm::mat4(1.0f); 
        ourShader.setMat4("model", model);

        //ourShader.setFloat("colour", wallColor);


        //    // Set the uniform variables for the shader program
        //glUseProgram(shaderProgram);
 /*       glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
*/
        //    // Draw the octagons
        glBindVertexArray(VAO_octagon);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_octagon);
        glDrawElements(GL_TRIANGLES, sizeof(indices_octagon), GL_UNSIGNED_INT, 0);


        //bind wall image texture, not sure if it's placed correctly here
        glBindTexture(GL_TEXTURE_2D, texture);

        // Draw the walls of the octagons
        glBindVertexArray(VAO_wall);
        //glUseProgram(shaderProgram);
        /*glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));*/
        //glUniform3fv(glGetUniformLocation(ourShader, "color"), 1, wallColor);

        ourShader.use();
        ourShader.setInt("texture1", 0); // Set the texture uniform to 0 (GL_TEXTURE0)



        glDrawArrays(GL_LINES, 0, sizeof(vertices_wall) / sizeof(vertices_wall[0]));

        //    // Swap the front and back buffers
        glfwSwapBuffers(window);

        //    // Poll for and process events
        glfwPollEvents();
    }

    ////// Clean up
    glDeleteBuffers(1, &VBO_octagon);
    glDeleteBuffers(1, &VBO_wall);
    glDeleteVertexArrays(1, &VAO_octagon);
    glDeleteVertexArrays(1, &VAO_wall);
    glDeleteBuffers(1, &EBO_octagon);
    //glDeleteProgram(shaderProgram);

    ////// Terminate GLFW
    glfwTerminate();
    return 0; 
}

// Define the model, view, and projection matrices //glm::mat4 model = glm::mat4(1.0f); //glm::mat4 view = glm::lookAt(glm::vec3(0.0f, 0.0f, 10.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f)); //glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)WIDTH / (float)HEIGHT, 0.1f, 100.0f); // //// Set the shader program uniforms //GLint modelLoc = glGetUniformLocation(shaderProgram, "model"); //GLint viewLoc = glGetUniformLocation(shaderProgram, "view"); //GLint projectionLoc = glGetUniformLocation(shaderProgram, "projection"); //GLint colorLoc = glGetUniformLocation(shaderProgram, "color");
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
