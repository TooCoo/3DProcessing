#include <iostream>
#include <fstream>

#include "viewer.h"

using Eigen::MatrixXd;

extern Viewer *myViewer;

// ----------------------------------------------------------------------------
// DISPLAY FUNCTIONS
// ----------------------------------------------------------------------------

Viewer::Viewer(int nEVecsToUse) {
    this->nEVecsToCalculate = nEVecsToUse;

    buttonList.push_back(Button(5, 5, 100, 25, "Shading", [](){ SetDisplayMode(Smooth); }));
    buttonList.push_back(Button(105, 5, 100, 25, "Show H", [](){ SetColoringMode(MeanCurvature); }));
    buttonList.push_back(Button(205, 5, 100, 25, "Wireframe", [](){ SetDisplayMode(WireFrame); }));
    buttonList.push_back(Button(305, 5, 100, 25, "Pointcloud", [](){ SetDisplayMode(PointCloud); }));
    buttonList.push_back(Button(405, 5, 100, 25, "Show E weight", [](){ SetColoringMode(SpectralWeight); }));
    buttonList.push_back(Button(505, 5, 100, 25, "Show K1 K2", [](){ SetColoringMode(PrincipalCurvature); }));

    textBoxList.push_back(TextBox{5, 50, 50, 25, "Eigen Decomposition"});

    buttonList.push_back(Button{ 25, 75, 50, 25, "+N Evec", [](){ IncreaseCalculatedEVecs(5, 1000); }});
    buttonList.push_back(Button{25, 100, 50, 25, "Find", RecomputeEigenvectors});
    buttonList.push_back(Button{ 25, 125, 50, 25, "-N Evec", [](){ IncreaseCalculatedEVecs(-5, 1000); }});

    //Eigen reconstruction
    textBoxList.push_back(TextBox{ 5, 160, 50, 25, "Eigen Reconstruction" });

    buttonList.push_back(Button{ 25, 185, 50, 25, "+N Evec", [](){ IncreaseNShownEigenVectors(5); } });
    buttonList.push_back(Button{25, 210, 50, 25, "Find", ReconstructMesh});
    buttonList.push_back(Button{ 25, 235, 50, 25, "-N Evec", [](){ IncreaseNShownEigenVectors(-5); } });

    //Edit the spectral coeffs:
    textBoxList.push_back(TextBox{ 5, 270, 50, 25, "Edit Spectral Coeffs" });


    buttonList.push_back(Button{ 5, 295, 50, 25, "Next", [](){ IncreaseSelectedEigenVector(1); }});
    buttonList.push_back(Button{ 55, 295, 50, 25, "Increase", [](){ IncreaseSpectralCoeff(1); }});
    buttonList.push_back(Button{ 5, 320, 100, 25, "Show", [](){ SetColoringMode(SpectralWeight); }}); //
    buttonList.push_back(Button{ 5, 345, 50, 25, "Prev", [](){ IncreaseSelectedEigenVector(-1); }}); //
    buttonList.push_back(Button{ 55, 345, 50, 25, "Decrease", [](){ IncreaseSpectralCoeff(-1); }}); //

    buttonList.push_back(Button{5, 375, 100, 25, "ResetMeshes", ResetMeshes}); //

    primary_mesh = 0;
    secondary_mesh = 1;

    globalScale += 5;
}


void Viewer::SetDisplayMode(DisplayMode mode) {
    if (mode == Smooth)
        std::cout << "Render mode: toggled flat/smooth shading... ";
    else if (mode == WireFrame)
        std::cout << "Render mode: toggled wireframe... ";
    else
        std::cout << "Render mode: pointcloud... ";

    myViewer->displayMode = mode;
    std::cout << "Done." << std::endl;
}


void Viewer::SetColoringMode(ColorMode mode) {
    if (mode == PrincipalCurvature) {
        std::cout << "Show principle curvature" << std::endl;
        std::cout << "\t colour codes:" << std::endl;
        std::cout << "\t \t Red:\t elliptic surface" << std::endl;
        std::cout << "\t \t Green:\t parabolic" << std::endl;
        std::cout << "\t \t Blue:\t hyperbolic" << std::endl;
    }
    else if (mode == MeanCurvature) {
        std::cout << "Toggled show mean curvature" << std::endl;
        std::cout << "\t colour codes: Red: low values, Blue: High value" << std::endl;
    }
    else if (mode == GaussianCurvature) {
        std::cout << "Toggled show Gaussian curvature" << std::endl;
        std::cout << "\t colour codes: Red: low values, Blue: High value" << std::endl;
    }
    else if (mode == SpectralWeight) {
        std::cout << "Showing weight of " << myViewer->selectedEVec << "th eigen vector" << std::endl;
    }
    std::cout << "Toggling... " << std::endl;
    myViewer->colorMode = mode;
    std::cout << "Done." << std::endl;
}


void Viewer::DrawButton(Button *b) {
    //deal with font if I want text

    if (b) {
        if (b->hoverState == Hover) {
            glColor3f(0.7f, 0.7f, 0.8f);
        }
        else {
            glColor3f(0.6f, 0.6f, 0.6f);
        }
    }

    //draw button background
    glBegin(GL_QUADS);
    glVertex2i(b->x_top_left, b->y_top_left);
    glVertex2i(b->x_top_left, b->y_top_left + b->h);
    glVertex2i(b->x_top_left + b->w, b->y_top_left + b->h);
    glVertex2i(b->x_top_left + b->w, b->y_top_left);
    glEnd();

    glTranslatef(0.0f, 0.0f, 0.1f);

    //Button Outline
    glLineWidth(3);
    //Change colour when clicked
    if (b->pressState == Pressed && b->hoverState == Hover)
        glColor3f(0.4f, 0.4f, 0.4f);
    else
        glColor3f(0.8f, 0.8f, 0.8f);

    glBegin(GL_LINE_STRIP);
    glVertex2i(b->x_top_left + b->w, b->y_top_left);
    glVertex2i(b->x_top_left, b->y_top_left);
    glVertex2i(b->x_top_left, b->y_top_left + b->h);
    glEnd();

    if (b->pressState == Pressed)
        glColor3f(0.8f, 0.8f, 0.8f);
    else
        glColor3f(0.4f, 0.4f, 0.4f);

    glBegin(GL_LINE_STRIP);
    glVertex2i(b->x_top_left, b->y_top_left + b->h);
    glVertex2i(b->x_top_left + b->w, b->y_top_left + b->h);
    glVertex2i(b->x_top_left + b->w, b->y_top_left);
    glEnd();

    glLineWidth(1);

    //Draw text

    int fontx;
    int fonty;

    //set middle of the text
    //fontx = b->x_top_left + (b->w - glutBitmapLength(GLUT_BITMAP_HELVETICA_10, b->slabel)) / 2;
    fontx = b->x_top_left + (b->w)/2 - 20;
    fonty = b->y_top_left + (b->h + 10) / 2;

    //if button pressed move string
    if (b->pressState == Pressed) {
        fontx += 2;
        fonty += 2;
    }

    glTranslatef(0.0f, 0.0f, 0.1f);

    //when highlighted use a different colour
    if (b->hoverState == Hover)
    {
        glColor3f(0, 0, 0);
        DrawText(GLUT_BITMAP_HELVETICA_10, b->label, fontx, fonty);
        fontx--;
        fonty--;
    }

    glColor3f(1, 1, 1);
    DrawText(GLUT_BITMAP_HELVETICA_10, b->label, fontx, fonty);
    glTranslatef(0.0f, 0.0f, -0.20f);

}

void Viewer::DrawText(void *font, const char *text, int x, int y) {
    glRasterPos2i(x, y);

    while (*text != '\0')
    {
        glutBitmapCharacter(font, *text);
        ++text;
    }
}

void Viewer::DrawTextBoxes(TextBox *tb) {
    int fontx;
    int fonty;


    fontx = tb->x_top_left + (tb->w) / 2 - 20;
    fonty = tb->y_top_left + (tb->h + 10) / 2;
    glTranslatef(0.0f, 0.0f, 0.1f);
    glColor3f(1, 1, 1);
    DrawText(GLUT_BITMAP_HELVETICA_10, tb->label, fontx, fonty);
    glTranslatef(0.0f, 0.0f, -0.1f);
}

void Viewer::display() {
    glfwSetErrorCallback(error_callback);
    GLFWwindow* window;

    if (!glfwInit())
        exit(EXIT_FAILURE);

    if (!glfwInit())
        exit(EXIT_FAILURE);

    window = glfwCreateWindow(640, 480, "Seb's Coursework 3", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    glfwSetKeyCallback(window, key_callback);
    glfwSetMouseButtonCallback(window, mouse_callback);
    glfwSetCursorPosCallback(window, mouse_moved_callback);

    while (!glfwWindowShouldClose(window))
    {
        display(window);
    }

    glfwDestroyWindow(window);

    glfwTerminate();
    exit(EXIT_SUCCESS);
}

void Viewer::display(GLFWwindow *window) {
    float ratio;
    int width, height;

    glfwGetFramebufferSize(window, &width, &height);
    ratio = width / (float)height;

    glViewport(0, 0, width, height);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-ratio, ratio, -1.f, 1.f, -1.f, 1.f);
    glMatrixMode(GL_MODELVIEW);

    glLoadIdentity();

    for (int i = 0; i < mesh_list.size(); i++) {
        //pop
        primary_mesh = i;

        glPushMatrix();

        //rotate
        glRotatef(global_rotation, 0.f, 1.f, 0.f);

        //translate so in centre
        glTranslatef(global_translation[0], global_translation[1], global_translation[2]);

        //scale here will act as a zoom
        glScalef(globalScale, globalScale, globalScale);

        mesh_list[primary_mesh].display(displayMode, colorMode, selectedEVec);
    }

    // Set the orthographic viewing transformation
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, window_w, window_h, 0, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    //This function will be called within the display function and draw the GUI for my mesh viewer
    for (int i = 0; i < buttonList.size(); i++) {
        DrawButton(&buttonList[i]);
    }

    for (int i = 0; i < textBoxList.size(); i++) {
        DrawTextBoxes(&textBoxList[i]);
    }

    glfwSwapBuffers(window);
    glfwPollEvents();
}