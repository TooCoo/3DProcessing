//
// Created by Ben Eisner on 5/24/16.
//

#include "viewer.h"

// GLFW
#include <GLFW/glfw3.h>

// GLUT
#include <include/GL/freeglut.h>

#include <iostream>

using Eigen::MatrixXd;

extern Viewer *myViewer;

// ----------------------------------------------------------------------------
// TOGGLING FUNCTIONS
// ----------------------------------------------------------------------------

void Viewer::TogglePointCloud() {
    myViewer->displayMode = PointCloud;
    std::cout << "Render mode: pointcloud\n";
}

void Viewer::ToggleShading() {
    myViewer->displayMode = Smooth;
    std::cout << "Render mode: toggled flat/smooth shading\n";
}

void Viewer::ToggleWireFrame() {
    std::cout << "Render mode: toggled wireframe\n";
    myViewer->displayMode = WireFrame;
}

void Viewer::ToggleShowPrincipleCurvature() {
    std::cout << "Toggled show principle curvature\n";
    std::cout << "\t colour codes:\t Red:\t elliptic surface\n";
    std::cout << "\t \t Green:\t parabolic\n";
    std::cout << "\t \t Blue:\t hyperbolic\n";
    myViewer->colorMode = PrincipalCurvature;
}

void Viewer::ToggleShowMeanCurvature() {
    std::cout << "Toggled show mean curvature\n";
    std::cout << "\t colour codes: Red: low values, Blue: High value\n";
    myViewer->colorMode = MeanCurvature;
}

void Viewer::ToggleShowGaussCurvature() {
    std::cout << "Toggled show Gaussian curvature\n";
    std::cout << "\t colour codes: Red: low values, Blue: High value\n";
    myViewer->colorMode = GaussianCurvature;
}

void Viewer::ToggleShowSpectralWeighting() {
    if (myViewer->colorMode == SpectralWeight) {
        myViewer->whichEigToDraw++;

        std::cout << myViewer->whichEigToDraw << "\n";

        if (myViewer->whichEigToDraw == myViewer->nEVecsToUse) {
            myViewer->whichEigToDraw = 0;
        }

        std::cout << myViewer->whichEigToDraw << "\n";

    }
    std::cout << "Showing weight of " << myViewer->whichEigToDraw << "th eigen vector\n";
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
        Font(GLUT_BITMAP_HELVETICA_10, b->label, fontx, fonty);
        fontx--;
        fonty--;
    }

    glColor3f(1, 1, 1);
    Font(GLUT_BITMAP_HELVETICA_10, b->label, fontx, fonty);
    glTranslatef(0.0f, 0.0f, -0.20f);

}

void Viewer::Font(void *font, const char *text, int x, int y) {
    glRasterPos2i(x, y);

    while (*text != '\0')
    {
        glutBitmapCharacter(font, *text);
        ++text;
    }
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
        current_mesh = i;

        glPushMatrix();

        //rotate
        glRotatef(global_rotation, 0.f, 1.f, 0.f);

        //translate so in centre
        glTranslatef(global_translation[0], global_translation[1], global_translation[2]);

        //scale here will act as a zoom
        glScalef(globalScale, globalScale, globalScale);

        mesh_list[current_mesh].display(displayMode, colorMode, whichEigToDraw);
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

    glfwSwapBuffers(window);
    glfwPollEvents();
}

void Viewer::display() {
    for (int i = 0; i < mesh_list.size(); i++) {
        auto a = mesh_list[i].n_faces();
        auto b = mesh_list[i].n_verts();
        auto c = 5;
    }
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

    // auto k_call = static_cast<GLFWkeyfun>([this](GLFWwindow* win, int a, int b, int c, int d) { this->key_callback(win, a, b, c, d); });

    //auto k_call = boost::bind(&Viewer::key_callback, this, _1, _2, _3, _4, _5);
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

void Viewer::colourByNormals() {
    for (int i = 0; i < mesh_list.size(); i++) {
        current_mesh = i;
        mesh_list[current_mesh].colourByNormals();
    }
}

void Viewer::colourPhong() {
    lightDir.normalize();
    for (int i = 0; i < mesh_list.size(); i++) {
        current_mesh = i;
        mesh_list[current_mesh].colourPhong(material, lightDir, ambientLight, specular, diffuse, virtual_camera_location);
    }
}

Viewer::Viewer(int nEVecsToUse) {
    this->nEVecsToUse = nEVecsToUse;

    buttonList.push_back(Button(5, 5, 100, 25, "Shading", ToggleShading));
    buttonList.push_back(Button(105, 5, 100, 25, "Show H", ToggleShowMeanCurvature));
    buttonList.push_back(Button(205, 5, 100, 25, "Wireframe", ToggleWireFrame));
    buttonList.push_back(Button(305, 5, 100, 25, "Pointcloud", TogglePointCloud));
    buttonList.push_back(Button(405, 5, 100, 25, "Show E weight", ToggleShowSpectralWeighting));
    buttonList.push_back(Button(505, 5, 100, 25, "Show K1 K2", ToggleShowPrincipleCurvature));

    current_mesh = 0;


    globalScale += 5;

}

void Viewer::addMesh(TriMesh triMesh) {
    // Add mesh curvatures as mesh properties
    mesh_list.push_back(triMesh);


    // update the global translation
    if (mesh_list.size() == 1) {
        global_translation = getCentreOfMesh(); //get centre
        global_translation *= -6.0f;
    }

    // Initialise various functions so I can view and draw the meshes

    findFaceNormals();
    findVertexNormalsFromFaces();
    //findGaussianCurvature2();
    for (int i = 0; i < mesh_list.size(); i++) {
        auto a = mesh_list[i].n_faces();
        auto b = mesh_list[i].n_verts();
        auto c = 5;
    }
}

void Viewer::colourRandomly() {
    for (int i = 0; i < mesh_list.size(); i++) {
        current_mesh = i;
        mesh_list[current_mesh].colourRandomly();
    }
}

void Viewer::findVertexNormalsFromFaces() {
    current_mesh = 0;
    mesh_list[current_mesh].findVertexNormalsFromFaces();
}

void Viewer::findFaceNormals() {
    current_mesh = 0;
    mesh_list[current_mesh].findFaceNormals();
}

void Viewer::editModeTranslate(float _dir) {
    current_mesh = 1;
    mesh_list[current_mesh].translate(_dir, move_amount, current_axis);
}

void Viewer::editModeRotate(float _dir) {
    current_mesh = 1;
    mesh_list[current_mesh].rotate(_dir, current_axis);
}

void Viewer::uniformLaplaceDiscretization() {
    current_mesh = 0;
    mesh_list[current_mesh].uniformLaplaceDiscretization();
}

void Viewer::discreteLaplaceDiscretization() {
    current_mesh = 0;
    mesh_list[current_mesh].discreteLaplaceDiscretization();
}

void Viewer::addNoise(float _sigma) {
    current_mesh = 0;
    mesh_list[current_mesh].addNoise(_sigma);
}

void Viewer::findGaussianCurvature2() {
    current_mesh = 0;
    mesh_list[current_mesh].findGaussianCurvature2();
}

void Viewer::applyDiffusionFlow(double lambda) {
    current_mesh = 0;
    mesh_list[current_mesh].applyDiffusionFlow(lambda);
}

void Viewer::implicitLaplacianMeshSmoothing(double lambda) {
    current_mesh = 0;
    mesh_list[current_mesh].implicitLaplacianMeshSmoothing(lambda);
}

void Viewer::findGaussianCurvature() {
    current_mesh = 0;
    mesh_list[current_mesh].findGaussianCurvature();
}

OpenMesh::Vec3f Viewer::getCentreOfMesh() {
    return mesh_list[current_mesh].getCentreOfMesh();
}

void Viewer::error_callback(int error, const char *description) {
    fputs(description, stderr);
}

void Viewer::key_callback(GLFWwindow *window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);

    if (key == GLFW_KEY_H && action == GLFW_PRESS) {
        std::cout << "H: HELP:\n\n";

        std::cout << "To toggle Edit mode press \tE\n";
        std::cout << "To toggle ICP mode press \tI\n";
        std::cout << "Current mode: ";

        if (myViewer->editmode) {
            std::cout << "Edit mode\n";
        }
        else {
            std::cout << "ICP mode\n";
        }

        std::cout << "\nEdit mode commands:\n";
        std::cout << "\t" << "R:" << "\t" << "Select Rotation mode\n";
        std::cout << "\t" << "T:" << "\t" << "Select Translation mode\n";
        std::cout << "\t" << "X:" << "\t" << "Select X axis\n";
        std::cout << "\t" << "Y:" << "\t" << "Select Y axis\n";
        std::cout << "\t" << "Z:" << "\t" << "Select Z axis\n";
        std::cout << "\t" << "LEFT:" << "\t" << "translate/rotate\n";
        std::cout << "\t" << "RIGHT:" << "\t" << "translate/rotate\n";

        std::cout << "\nICP mode commands:\n";
        std::cout << "\t" << "LEFT:" << "\t" << "rotate scene\n";
        std::cout << "\t" << "RIGHT:" << "\t" << "rotate scene\n";
        std::cout << "\t" << "UP:" << "\t" << "zoom in\n";
        std::cout << "\t" << "DOWN:" << "\t" << "zoom out\n";
        std::cout << "\t" << "SPACE:" << "\t" << "centre Scene\n";
        std::cout << "\t" << "O:" << "\t" << "Draw Overlap between two meshes\n";
        std::cout << "\t" << "I:" << "\t" << "Apply single diffusion iterations\n";
        std::cout << "\t" << "M:" << "\t" << "Colour by mesh id\n";
        std::cout << "\t" << "N:" << "\t" << "Find normals\n";
        std::cout << "\t" << "K:" << "\t" << "Colour by normals (depreciated use P)\n";
        //std::cout << "\t" << "L:" << "\t" << "Align Normals - not perfectly implemented\n";
        std::cout << "\t" << "L:" << "\t" << "Implicis Laplacian smoothing\n";
        std::cout << "\t" << "S:" << "\t" << "Save second mesh\n";
        std::cout << "\t" << "P:" << "\t" << "Colour by normals - Phong shading\n";
        std::cout << "\t" << "C:" << "\t" << "Move meshes to common centre\n";
        std::cout << "\t" << "R:" << "\t" << "Move meshes to the origin - similar to SPACE\n";
        std::cout << "\t" << "G:" << "\t" << "Add Gaussian noise to second mesh\n";

    }


    //in help
    if (key == GLFW_KEY_E && action == GLFW_PRESS) {
        myViewer->editmode = true;
        std::cout << "E: Edit mode enabled\n";
    }


    if (!myViewer->editmode) {

        //in help
        if (key == GLFW_KEY_LEFT && action == GLFW_PRESS) {
            myViewer->global_rotation -= 5.0f;
            // std::cout << "Left\n";
        }
        //in help
        if (key == GLFW_KEY_RIGHT && action == GLFW_PRESS) {
            myViewer->global_rotation += 5.0f;
            // std::cout << "right\n";
        }
        //in help
        if (key == GLFW_KEY_UP && action == GLFW_PRESS) {
            myViewer->globalScale += 1.0f;
            // std::cout << "UP: Zoom in\n";
        }
        //in help
        if (key == GLFW_KEY_DOWN && action == GLFW_PRESS) {
            myViewer->globalScale -= 1.0f;
            // std::cout << "DOWN: Zoom out\n";
        }
        //in help
        if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
            myViewer->global_translation = myViewer->getCentreOfMesh(); //get centre
            myViewer->global_translation *= -4.0f;
            std::cout << "SPACE: Mesh Centred\n";
        }
        //in help
        if (key == GLFW_KEY_O && action == GLFW_PRESS) {
            myViewer->mesh_list[0].colourOverlap(myViewer->mesh_list[1]);
            std::cout << "O: Overlap drawn\n";
        }
        //in help
        if (key == GLFW_KEY_M && action == GLFW_PRESS) {
            myViewer->mesh_list[myViewer->current_mesh].colourRandomly();
            std::cout << "M: Mesh Coloured\n";
        }
        //in help
        if (key == GLFW_KEY_N && action == GLFW_PRESS) {
            std::cout << "N: Generating normals...\t";
            myViewer->findFaceNormals();
            myViewer->findVertexNormalsFromFaces();
            myViewer->findGaussianCurvature2();
            std::cout << "Done\n";
        }
        //in help
        if (key == GLFW_KEY_K && action == GLFW_PRESS) {
            //double lambda = -1.0;
            int n_largest_eigs = 50;
            myViewer->nEVecsToUse += 3;
            //eigenReconstruction(lambda, n_largest_eigs);

            if (myViewer->nEVecsToUse == 1) {
                //findEigenVectors(n_largest_eigs);
            }
            else {
                myViewer->findEigenVectors(myViewer->nEVecsToUse);
                myViewer->remakeFromEVecs(myViewer->nEVecsToUse);
            }
        }

        if (key == GLFW_KEY_KP_ADD && action == GLFW_PRESS) {
            myViewer->mesh_list[myViewer->current_mesh].evecs_coeffs(myViewer->whichEigToDraw) += myViewer->eig_inc;
            myViewer->remakeFromEVecs(myViewer->nEVecsToUse);
        }

        if (key == GLFW_KEY_KP_SUBTRACT && action == GLFW_PRESS) {
            myViewer->mesh_list[myViewer->current_mesh].evecs_coeffs(myViewer->whichEigToDraw) -= myViewer->eig_inc;
            myViewer->remakeFromEVecs(myViewer->nEVecsToUse);
        }

        //in help
        if (key == GLFW_KEY_L && action == GLFW_PRESS) {
            //alignNormals();
            std::cout << "L: implicit laplacian smoothing iteration... ";
            double lambda = -1.0;
            myViewer->implicitLaplacianMeshSmoothing(lambda);
            myViewer->findFaceNormals();
            myViewer->findVertexNormalsFromFaces();
            myViewer->findGaussianCurvature2();


            std::cout << "Done\n";
        }
        //in help
        if (key == GLFW_KEY_S && action == GLFW_PRESS) {
            myViewer->mesh_list[myViewer->current_mesh].save();
            std::cout << "S: mesh Saved\n";
        }

        //in help
        if (key == GLFW_KEY_P && action == GLFW_PRESS) {
            myViewer->colourPhong();
            std::cout << "P: Phong shading\n";
        }
        // in help
        if (key == GLFW_KEY_C && action == GLFW_PRESS) {
            myViewer->moveMeshesToCommonCentre();
            std::cout << "C: move meshes to Common Centre\n";
        }

        //help
        if (key == GLFW_KEY_R && action == GLFW_PRESS) {
            myViewer->moveMeshesToOrigin();
            std::cout << "R: Meshes moved to Origin\n";
        }
        //help
        if (key == GLFW_KEY_I && action == GLFW_PRESS) {
            for (int i = 0; i < 10; i++) {
                double lambda = -0.0000001;
                myViewer->applyDiffusionFlow(lambda);
                myViewer->findFaceNormals();
                myViewer->findVertexNormalsFromFaces();
                myViewer->findGaussianCurvature2();
            }
            std::cout << "I: apply diffusion smoothing\n";
        }

        //help
        if (key == GLFW_KEY_G && action == GLFW_PRESS) {
            double sigma = 0.0002;
            myViewer->addNoise(sigma);
            std::cout << "G: Gaussian noise added, var: " << sigma << "\n";
        }

    }
    else {
        //in help
        if (key == GLFW_KEY_R && action == GLFW_PRESS) {
            myViewer->editType = RotationMode;
            std::cout << "R: Rotation mode toggled\n";
        }
        //in help
        if (key == GLFW_KEY_T && action == GLFW_PRESS) {
            myViewer->editType = TranslationMode;
            std::cout << "R: Translation mode toggled\n";
        }
        //in help
        if (myViewer->editType == TranslationMode) {
            if (key == GLFW_KEY_LEFT && action == GLFW_PRESS) {
                myViewer->editModeTranslate(-1.0);
                std::cout << "Left: Mesh moved left\n";
            }
            //in help
            if (key == GLFW_KEY_RIGHT && action == GLFW_PRESS) {
                myViewer->editModeTranslate(1.0);
                std::cout << "Right: Mesh moved right\n";
            }
        }

        if (myViewer->editType == RotationMode) {
            //in help
            if (key == GLFW_KEY_LEFT && action == GLFW_PRESS) {
                myViewer->editModeRotate(-1.0);
                std::cout << "Left: Mesh rotated left\n";
            }
            //in help
            if (key == GLFW_KEY_RIGHT && action == GLFW_PRESS) {
                myViewer->editModeRotate(1.0);
                std::cout << "Right: Mesh rotated right\n";
            }
        }

        //in help
        if (key == GLFW_KEY_X && action == GLFW_PRESS) {
            myViewer->current_axis = 0;
            std::cout << "X: X axis selected\n";
        }
        //in help
        if (key == GLFW_KEY_Y && action == GLFW_PRESS) {
            myViewer->current_axis = 1;
            std::cout << "Y: Y axis selected\n";
        }
        //in help
        if (key == GLFW_KEY_Z && action == GLFW_PRESS) {
            myViewer->current_axis = 2;
            std::cout << "Z: Z axis selected\n";
        }

    }
}

void Viewer::mouse_callback(GLFWwindow *window, int button, int action, int mods) {
    auto mouse = &myViewer->TheMouse;

    if (button == GLFW_MOUSE_BUTTON_1) {
        //mouse button 1 - Left mouse Button
        mouse->lmb = action;
        for (int i = 0; i < myViewer->buttonList.size(); i++) {
            myViewer->buttonList[i].updateClickState(mouse->x, mouse->y, action == 1);
        }

    }
    else if(button == GLFW_MOUSE_BUTTON_2) {
        //mouse button 2 - Right mouse Button
        mouse->rmb = action;
    }
}

void Viewer::mouse_moved_callback(GLFWwindow *window, double x_pos, double y_pos) {

    auto mouse = &myViewer->TheMouse;

    mouse->dx = mouse->x - x_pos;
    mouse->dy = mouse->y - y_pos;

    mouse->x = (int)x_pos;
    mouse->y = (int)y_pos;

    bool anyButton = false;

    for (int i = 0; i < myViewer->buttonList.size(); i++) {
        if (myViewer->buttonList[i].updateHoverState(mouse->x, mouse->y)) {
            anyButton = true;
        }
    }

    if (!anyButton) {
        if (mouse->lmb) myViewer->global_rotation -= float(mouse->dx);// / 1.0f;
    }
}

void Viewer::eigenReconstruction(double lambda, int nLargestEigs) {
    current_mesh = 0;
    mesh_list[current_mesh].eigenReconstruction(lambda, nLargestEigs);
}

void Viewer::findEigenVectors(int nLargestEigs) {
    current_mesh = 0;
    mesh_list[current_mesh].findEigenVectors(nLargestEigs);
}

void Viewer::remakeFromEVecs(int nLargestEigs) {
    current_mesh = 0;
    mesh_list[current_mesh].remakeFromEVecs(nLargestEigs, unchangedMesh);
}

void Viewer::moveMeshesToCommonCentre() {

    std::vector<OpenMesh::Vec3f> mesh_centre;
    //find average point
    for (int i = 0; i < mesh_list.size(); i++) {

        mesh_centre.push_back(OpenMesh::Vec3f()*0.0f);
        current_mesh = i;
        mesh_centre[current_mesh] = mesh_list[current_mesh].getCentreOfMesh();
    }

    //move points to mesh 1 location
    for (int i = 1; i < mesh_list.size(); i++) {

        current_mesh = i;
        OpenMesh::Vec3f translation = mesh_centre[0] - mesh_centre[current_mesh];
        mesh_list[current_mesh].translate(translation);
    }
}

void Viewer::moveMeshesToOrigin() {

    std::vector<OpenMesh::Vec3f> mesh_centre;
    //find average point
    for (int i = 0; i < mesh_list.size(); i++) {

        mesh_centre.push_back(OpenMesh::Vec3f()*0.0f);
        current_mesh = i;
        mesh_centre[current_mesh] = mesh_list[current_mesh].getCentreOfMesh();
    }

    //move points to mesh 1 location
    for (int i = 0; i < mesh_list.size(); i++) {

        current_mesh = i;
        OpenMesh::Vec3f translation = -1.0 * mesh_centre[0];
        mesh_list[current_mesh].translate(translation);
    }
}

void Viewer::setUnchangedMesh(TriMesh *triMesh) {
    unchangedMesh = triMesh;
}
