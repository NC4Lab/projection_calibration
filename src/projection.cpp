#include <ros/ros.h>
#include "projection.h"

using namespace std;
GLFWwindow* window;

//int width, height;
//unsigned int width2 = 820 , height2 = 560;
//unsigned char* image;


//bool window_fullScreen = false;
//int pos1[3] = { 0,0,0 };
//int pos2[3] = { 0,0,0 };
//int pos3[3] = { 0,0,0 };
//int pos4[3] = { 0,0,0 };
//
//void reshape(int w, int h) {
//    if (h == 0) h = 1;
//    GLdouble aspect = (GLdouble)w / (GLdouble)h;
//    glMatrixMode(GL_PROJECTION);
//    glLoadIdentity();
//    glMatrixMode(GL_MODELVIEW);
//    glViewport(0, 0, w, h);
//    // display();
//}
//void quitVisualScene(void) {
//
//    //glutDestroyWindow(glutGetWindow());
//    exit(0);
//}

//void toggleFullscreen(void)
////{
////    if (window_fullScreen) {
////        glutReshapeWindow(glutGet(GLUT_INIT_WINDOW_WIDTH), glutGet(GLUT_INIT_WINDOW_HEIGHT));
////        glutPositionWindow(glutGet(GLUT_INIT_WINDOW_X), glutGet(GLUT_INIT_WINDOW_Y));
////        window_fullScreen = false;
////    }
////    else {
////        glutFullScreen();
////        window_fullScreen = true;
//    //}
//}
//void keyPressed(unsigned char key, int x, int y) {
//    switch (key) {
//    case 'q':
//        quitVisualScene();
//        break;
//    case 'p':
//        //toggleFullscreen();
//        break;
//    default:
//        ;
//    }
//}
//void init() {
//    //glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_ALPHA | GLUT_MULTISAMPLE); //|GLUT_ACCUM);
//    //glutInitWindowPosition(0, 0);
//    //glutInitWindowSize(1920, 1080);
//    //glutCreateWindow("Maze Calibration");
//  }


//void display()
//{
//    glClearColor(1.0, 1.0, 1.0, 0.0);
//    glClear(GL_COLOR_BUFFER_BIT);
//    glDrawPixels(width, height, GL_RGBA, GL_UNSIGNED_BYTE, image);
//    // gluOrtho2D(0.0,100.0,0,100.0);
//    //glColor3f(0.0, 1.0, 0.0);
//    //// square 1
//    //glBegin(GL_POLYGON);
//    //glVertex3f(2.0, 4.0, 0.0);
//    //glVertex3f(8.0, 4.0, 0.0);
//    //glVertex3f(8.0, 6.0, 0.0);
//    //glVertex3f(2.0, 6.0, 0.0);
//    //glEnd();
//    //glFlush();
//	 glfwSwapBuffers(window);
//}


//void loadPNG(const char* filename) {
//    unsigned error = lodepng_decode_file(&image, &width2, &height2, filename, LCT_RGB, 24U);
//    if (error) {
//        printf("error %u: %s\n", error, lodepng_error_text(error));
//        exit(EXIT_FAILURE);
//    }
//    //return;
//}

int main(int argc, char** argv) {

    ros::init(argc, argv, "Projection",ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    //string filename = "the_box.svg";

    //loading the image using NSVG library
    NSVGimage* image = nsvgParseFromFile("src/the_box.svg", "px", 96.0f);
    if (!image) {
        std::cerr << "Error: could not parse SVG file: " << std::endl;
        return 0;
    }



    if (!glfwInit()) {
        std::cerr << "Error: could not initialize GLFW" << std::endl;
        return 0;
    }

    window = glfwCreateWindow(image->width, image->height, "SVG Viewer", NULL, NULL);

    if (!window) {
        std::cerr << "Error: could not create GLFW window" << std::endl;
        glfwTerminate();
        return 0;
    }

    glfwMakeContextCurrent(window);
    glfwInit();
    //unsigned char* dst;
    //NSVGrasterizer* rast = nsvgCreateRasterizer();
    //nsvgRasterize(rast, image, 0.0f, 0.0f, 1.0f, 1.0f, dst, image->width, image->height, NULL);
    //glfwClear(GLFW_COLOR_BUFFER_BIT);

    ////nsvgRasterize(rast, image, 0, 0, 1.0f, NULL);
    //glfwDrawPixels(rast->width, rast->height, GL_RGBA, GL_UNSIGNED_BYTE, rast->image);

    //glfwSwapBuffers(window);


    std::vector<float> vertices;
    std::vector<float> colors;

    for (NSVGshape* shape = image->shapes; shape != NULL; shape = shape->next) {
        for (NSVGpath* path = shape->paths; path != NULL; path = path->next) {
            for (int i = 0; i < path->npts - 1; i++) {
                vertices.push_back(path->pts[i * 2]);
                vertices.push_back(path->pts[i * 2 + 1]);
                colors.push_back(shape->fill.color / 255.0f);
                colors.push_back(shape->fill.color / 255.0f);
                colors.push_back(shape->fill.color / 255.0f);
                //colors.push_back(shape->fill.opacity);
            }
        }
    }

    unsigned int vao, vbo, cbo;

    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(0);

    glGenBuffers(1, &cbo);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(GLfloat), colors.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(1);

    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT);

        glDrawArrays(GL_TRIANGLE_STRIP, 0, vertices.size() / 2);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}


    //glutInit(&argc, argv);
 //   if (!glfwInit())
 //   {
 //       // Initialization failed
 //   }

 //   glEnable(GL_BLEND);
 //   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

 //   //init();
 //   // Display functions
 //   //glutDisplayFunc(display);
 //   //glutIdleFunc(glutPostRedisplay);
 //   //glutReshapeFunc(reshape);
 //   //glutKeyboardFunc(keyPressed);

 //   // glewInit();
 //   // setShaders();

 //   //glutMainLoop();

	//window = glfwCreateWindow(640, 480, "My Title", NULL, NULL);
 //   if (!window)
 //   {
 //       glfwTerminate();
 //       exit(EXIT_FAILURE);
 //   }

 //   glfwMakeContextCurrent(window);
 //   glfwSwapInterval(1);

 //   while (!glfwWindowShouldClose(window))
 //   {
 //       // Keep running
 //   }

 //   int width, height;
 //   glfwGetFramebufferSize(window, &width, &height);
 //   glViewport(0, 0, width, height);

 //   glfwDestroyWindow(window);
 //   glfwTerminate();

 //   return 0;

    //if (!glfwInit())
        //return -1;

//    /* Create a windowed mode window and its OpenGL context */
//    window = glfwCreateWindow(820, 560, "PNG Viewer", NULL, NULL);
//    if (!window)
//    {
//        glfwTerminate();
//        return -1;
//    }
//
//    /* Make the window's context current */
//    glfwMakeContextCurrent(window);
//    loadPNG("src/projection_calibration/src/clipart.png");
//
//    /* Loop until the user closes the window */
//    while (!glfwWindowShouldClose(window))
//    {
//        /* Render here */
//        //glClear(GL_COLOR_BUFFER_BIT);
//
//        ///* Swap front and back buffers */
//        //glfwSwapBuffers(window);
//        display();
//
//        /* Poll for and process events */
//        glfwPollEvents();
//    }
//
//    glfwDestroyWindow(window);
//    glfwTerminate();
//
//    //free(image);
//    //exit(EXIT_SUCCESS);
//    return 0;
//
//
//}
