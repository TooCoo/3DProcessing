//
// Created by Ben Eisner on 5/24/16.
//

#include <iostream>
#include <fstream>

#include "viewer.h"

using Eigen::MatrixXd;

extern Viewer *myViewer;

void Viewer::key_callback(GLFWwindow *window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS) {
        // Edit mode keys
        if (myViewer->editMode) {
            switch (key) {
                // enable icp mode
                case GLFW_KEY_E:
                    std::cout << "E: Enabling icp mode... ";
                    myViewer->editMode = false;
                    std::cout << "Done." << std::endl;

                    // enable rotation
                case GLFW_KEY_R:
                    std::cout << "R: Toggle rotation mode... ";
                    myViewer->editType = RotationMode;
                    std::cout << "Done." << std::endl;
                    break;

                    // enable translation
                case GLFW_KEY_T:
                    std::cout << "R: Toggle translation mode... ";
                    myViewer->editType = TranslationMode;
                    std::cout << "Done." << std::endl;
                    break;

                    // perform some sort of negative transformation
                case GLFW_KEY_LEFT:
                    if (myViewer->editType == TranslationMode) {
                        std::cout << "Left: Move mesh left... ";
                        EditModeTranslate(-transformDelta);
                        std::cout << "Done." << std::endl;
                    }
                    else {
                        std::cout << "Left: Rotate mesh left... ";
                        EditModeRotate(-transformDelta);
                        std::cout << "Done." << std::endl;
                    }
                    break;

                    // perform some sort of positive transformation
                case GLFW_KEY_RIGHT:
                    if (myViewer->editType == TranslationMode) {
                        std::cout << "Right: Move mesh right... ";
                        EditModeTranslate(transformDelta);
                        std::cout << "Done." << std::endl;
                    }
                    else {
                        std::cout << "Right: Rotate mesh right... ";
                        EditModeRotate(transformDelta);
                        std::cout << "Done." << std::endl;
                    }
                    break;

                    // select x axis
                case GLFW_KEY_X:
                    std::cout << "X: select X axis... ";
                    myViewer->current_axis = 0;
                    std::cout << "Done." << std::endl;
                    break;

                    // select y axis
                case GLFW_KEY_Y:
                    std::cout << "Y: select Y axis... ";
                    myViewer->current_axis = 1;
                    std::cout << "Done." << std::endl;
                    break;

                    // select z axis
                case GLFW_KEY_Z:
                    std::cout << "Z: select z axis... ";
                    myViewer->current_axis = 2;
                    std::cout << "Done." << std::endl;
                    break;
            }
        }

        // non-edit-mode
        switch (key) {
            // quit
            case GLFW_KEY_ESCAPE:
                std::cout << "Closing window... ";
                glfwSetWindowShouldClose(window, GL_TRUE);
                std::cout << "Done." << std::endl;
                break;

            // enable edit mode
            case GLFW_KEY_E:
                std::cout << "E: Enabling edit mode... ";
                myViewer->editMode = true;
                std::cout << "Done." << std::endl;
                break;

            // rotate global scene left
            case GLFW_KEY_LEFT:
                std::cout << "Left: Rotate left... ";
                myViewer->global_rotation -= 5.0f;
                std::cout << "Done." << std::endl;
                break;

            // rotate global scene right
            case GLFW_KEY_RIGHT:
                std::cout << "Right: Rotate right... ";
                myViewer->global_rotation += rotationDelta;
                std::cout << "Done." << std::endl;
                break;

            // zoom in
            case GLFW_KEY_UP:
                std::cout << "Up: Zoom in... ";
                myViewer->globalScale += zoomDelta;
                std::cout << "Done." << std::endl;
                break;

            // zoom out
            case GLFW_KEY_DOWN:
                std::cout << "Up: Zoom out... ";
                myViewer->globalScale -= zoomDelta;
                std::cout << "Done." << std::endl;
                break;

            // center meshes
            case GLFW_KEY_SPACE:
                std::cout << "SPACE: Centering mesh... ";
                myViewer->global_translation = GetCentreOfMesh() * centerScalar;
                std::cout << "Done." << std::endl;
                break;

            // colour overlap
            case GLFW_KEY_O:
                std::cout << "O: Drawing overlap... ";
                ColourOverlap();
                std::cout << "Done." << std::endl;
                break;

            // coulour randomly
            case GLFW_KEY_M:
                std::cout << "M: Coloring mesh randomly... ";
                ColourRandomly();
                std::cout << "Done." << std::endl;
                break;

            // find normals and curvature
            case GLFW_KEY_N:
                std::cout << "N: Generating normals and curvature... ";
                FindNormalsAndCurvature();
                std::cout << "Done." << std::endl;
                break;

            // Exact computation of eigenvectors, apply with smallest eigenvectors
            case GLFW_KEY_J:
                std::cout << "J: [Exact, Eigenvalues] Computing eigenvectors... " << std::flush;
                ReconstructMesh(Uniform, Exact, SmallestEigenvalues);
                std::cout << "Done." << std::endl;
                break;

            // Exact computation of eigenvectors, apply with largest spectral coefficients
            case GLFW_KEY_K:
                std::cout << "K: [Exact, Coeff] Computing eigenvectors... " << std::flush;
                ReconstructMesh(Uniform, Exact, LargestCoefficients);
                std::cout << "Done." << std::endl;
                break;

            // Approximate computation of eigenvectors, apply with smallest eigenvectors
            case GLFW_KEY_L:
                std::cout << "L: [Approx, Eigenvalues] Computing eigenvectors... " << std::flush;
                ReconstructMesh(Uniform, Approximate, SmallestEigenvalues);
                std::cout << "Done." << std::endl;
                break;

            // Approximate computation of eigenvectors, apply largest spectral coefficients
            case GLFW_KEY_SEMICOLON:
                std::cout << ";: [Approx, Coeff] Computing eigenvectors... " << std::flush;
                ReconstructMesh(Uniform, Approximate, LargestCoefficients);
                std::cout << "Done." << std::endl;
                break;

            // implicit laplacian smoothing
            case GLFW_KEY_D:
                std::cout << "D: implicit laplacian smoothing iteration... ";
                ImplicitLaplacianMeshSmoothing(implicitLambda);
                FindNormalsAndCurvature();
                std::cout << "Done." << std::endl;
                break;

            // save mesh
            case GLFW_KEY_S:
                std::cout << "S: saving mesh... ";
                SaveMesh();
                std::cout << "Done." << std::endl;
                break;

            // color by Phong shading
            case GLFW_KEY_P:
                std::cout << "P: Phong shading... ";
                ColourByPhong();
                std::cout << "Done." << std::endl;
                break;

            // move meshes to common centre
            case GLFW_KEY_C:
                std::cout << "C: moving meshes to Common Centre...";
                MoveMeshesToCommonCentre();
                std::cout << "Done." << std::endl;
                break;

            // move meshes to origin
            case GLFW_KEY_R:
                std::cout << "R: Meshes moved to Origin... ";
                MoveMeshesToOrigin();
                std::cout << "Done." << std::endl;
                break;

            // diffusion smoothing
            case GLFW_KEY_I:
                std::cout << "I: apply diffusion smoothing... ";
                for (int i = 0; i < 10; i++) {
                    ApplyDiffusionFlow(diffusionLambda);
                    FindNormalsAndCurvature();
                }
                std::cout << "Done." << std::endl;
                break;

            // add noise
            case GLFW_KEY_G:
                std::cout << "G: Gaussian noise added, var: " << noiseSigma << "... ";
                myViewer->AddNoise(noiseSigma);
                std::cout << "Done." << std::endl;
                break;

            // perform spectral testing
            case GLFW_KEY_T:
                std::cout << "Performing tests with n_eigs = " << myViewer->nEVecsToCalculate << std::endl;
                myViewer->PerformSpectralTests();
                std::cout << "Done." << std::endl;
                break;

            // help
            case GLFW_KEY_H:
                std::cout << "H: HELP:\n" << std::endl;

                std::cout << "To toggle Edit mode press \tE" << std::endl;
                std::cout << "To toggle ICP mode press \tI" << std::endl;
                std::cout << "Current mode: " << (myViewer->editMode ? "Edit mode\n" : "ICP mode") << std::endl;

                std::cout << "\nEdit mode commands:" << std::endl;
                std::cout << "\t" << "R:" << "\t" << "Select Rotation mode" << std::endl;
                std::cout << "\t" << "T:" << "\t" << "Select Translation mode" << std::endl;
                std::cout << "\t" << "X:" << "\t" << "Select X axis" << std::endl;
                std::cout << "\t" << "Y:" << "\t" << "Select Y axis" << std::endl;
                std::cout << "\t" << "Z:" << "\t" << "Select Z axis" << std::endl;
                std::cout << "\t" << "LEFT:" << "\t" << "translate/rotate" << std::endl;
                std::cout << "\t" << "RIGHT:" << "\t" << "translate/rotate" << std::endl;

                std::cout << "\nICP mode commands:" << std::endl;
                std::cout << "\t" << "LEFT:" << "\t" << "rotate scene" << std::endl;
                std::cout << "\t" << "RIGHT:" << "\t" << "rotate scene" << std::endl;
                std::cout << "\t" << "UP:" << "\t" << "zoom in" << std::endl;
                std::cout << "\t" << "DOWN:" << "\t" << "zoom out" << std::endl;
                std::cout << "\t" << "SPACE:" << "\t" << "centre Scene" << std::endl;
                std::cout << "\t" << "O:" << "\t" << "Draw Overlap between two meshes" << std::endl;
                std::cout << "\t" << "I:" << "\t" << "Apply single diffusion iterations" << std::endl;
                std::cout << "\t" << "M:" << "\t" << "Colour by mesh id" << std::endl;
                std::cout << "\t" << "N:" << "\t" << "Find normals" << std::endl;
                std::cout << "\t" << "D:" << "\t" << "Implicit Laplacian smoothing" << std::endl;
                std::cout << "\t" << "S:" << "\t" << "Save second mesh" << std::endl;
                std::cout << "\t" << "P:" << "\t" << "Colour by normals - Phong shading" << std::endl;
                std::cout << "\t" << "C:" << "\t" << "Move meshes to common centre" << std::endl;
                std::cout << "\t" << "R:" << "\t" << "Move meshes to the origin - similar to SPACE" << std::endl;
                std::cout << "\t" << "G:" << "\t" << "Add Gaussian noise to second mesh" << std::endl;

                std::cout << "\nSpectral commands:" << std::endl;
                std::cout << "\t" << "J:" << "\t" << "Exact computation of eigenvectors, apply with smallest eigenvectors" << std::endl;
                std::cout << "\t" << "K:" << "\t" << "Exact computation of eigenvectors, apply with largest spectral coefficients" << std::endl;
                std::cout << "\t" << "L:" << "\t" << "Approximate computation of eigenvectors, apply with smallest eigenvalues" << std::endl;
                std::cout << "\t" << ";:" << "\t" << "Approximate computation of eigenvectors, apply with largest spectral coefficients" << std::endl;
                std::cout << "\t" << "T:" << "\t" << "Perform tests " << std::endl;
                break;
        }
    }
}

void Viewer::error_callback(int error, const char *description) {
    fputs(description, stderr);
}

void Viewer::mouse_callback(GLFWwindow *window, int button, int action, int mods) {
    auto mouse = &myViewer->mouse;

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

    auto mouse = &myViewer->mouse;

    mouse->dx = (int)(mouse->x - x_pos);
    mouse->dy = (int)(mouse->y - y_pos);

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

void Viewer::ColourByPhong() {
    myViewer->lightDir.normalize();
    myViewer->mesh_list[myViewer->primary_mesh].colourPhong(myViewer->material, myViewer->lightDir,
                                                            myViewer->ambientLight, myViewer->specular,
                                                            myViewer->diffuse, myViewer->virtualCameraLocation);
}

void Viewer::ColourByNormals() { myViewer->mesh_list[myViewer->primary_mesh].colourByNormals(); }

void Viewer::ColourRandomly() { myViewer->mesh_list[myViewer->primary_mesh].colourRandomly(); }

void Viewer::FindVertexNormalsFromFaces() { myViewer->mesh_list[myViewer->primary_mesh].findVertexNormalsFromFaces(); }

void Viewer::FindFaceNormals() { myViewer->mesh_list[myViewer->primary_mesh].findFaceNormals(); }

void Viewer::EditModeTranslate(float _dir) { myViewer->mesh_list[myViewer->primary_mesh].translate(_dir, myViewer->move_amount, myViewer->current_axis); }

void Viewer::EditModeRotate(float _dir) { myViewer->mesh_list[myViewer->primary_mesh].rotate(_dir, myViewer->current_axis); }

void Viewer::AddNoise(float sigma) { myViewer->mesh_list[myViewer->primary_mesh].addNoise(sigma); }

void Viewer::FindGaussianCurvature() { myViewer->mesh_list[myViewer->primary_mesh].findGaussianCurvature(); }

void Viewer::ApplyDiffusionFlow(double lambda) { myViewer->mesh_list[myViewer->primary_mesh].applyDiffusionFlow(lambda); }

void Viewer::ImplicitLaplacianMeshSmoothing(double lambda) { myViewer->mesh_list[myViewer->primary_mesh].implicitLaplacianMeshSmoothing(lambda); }

OpenMesh::Vec3f Viewer::GetCentreOfMesh() { return myViewer->mesh_list[myViewer->primary_mesh].getCentreOfMesh(); }

void Viewer::ColourOverlap() { myViewer->mesh_list[myViewer->primary_mesh].colourOverlap(myViewer->mesh_list[1]); }

void Viewer::SaveMesh() { myViewer->mesh_list[myViewer->primary_mesh].save(); }

void Viewer::MoveMeshesToCommonCentre() {

    std::vector<OpenMesh::Vec3f> mesh_centre;
    //find average point
    for (int i = 0; i < myViewer->mesh_list.size(); i++) {

        mesh_centre.push_back(OpenMesh::Vec3f()*0.0f);
        myViewer->primary_mesh = i;
        mesh_centre[myViewer->primary_mesh] = myViewer->mesh_list[myViewer->primary_mesh].getCentreOfMesh();
    }

    //move points to mesh 1 location
    for (int i = 1; i < myViewer->mesh_list.size(); i++) {

        myViewer->primary_mesh = i;
        OpenMesh::Vec3f translation = mesh_centre[0] - mesh_centre[myViewer->primary_mesh];
        myViewer->mesh_list[myViewer->primary_mesh].translate(translation);
    }
}

void Viewer::MoveMeshesToOrigin() {

    std::vector<OpenMesh::Vec3f> mesh_centre;
    //find average point
    for (int i = 0; i < myViewer->mesh_list.size(); i++) {

        mesh_centre.push_back(OpenMesh::Vec3f()*0.0f);
        myViewer->primary_mesh = i;
        mesh_centre[myViewer->primary_mesh] = myViewer->mesh_list[myViewer->primary_mesh].getCentreOfMesh();
    }

    //move points to mesh 1 location
    for (int i = 0; i < myViewer->mesh_list.size(); i++) {

        myViewer->primary_mesh = i;
        OpenMesh::Vec3f translation = -1.0 * mesh_centre[0];
        myViewer->mesh_list[myViewer->primary_mesh].translate(translation);
    }
}

void Viewer::FindNormalsAndCurvature() {
    FindFaceNormals();
    FindVertexNormalsFromFaces();
    FindGaussianCurvature();
}

void Viewer::IncreaseSpectralCoeff(int delta) {
    std::cout << "Increasing eigenvector " << myViewer->selectedEVec << " by " << delta << " and reconstructing... ";
    myViewer->mesh_list[myViewer->primary_mesh].increaseEVecCoeff(myViewer->selectedEVec, delta);
    myViewer->mesh_list[myViewer->primary_mesh].reconstructMesh(myViewer->nEVecsToDraw);
    std::cout << "Done." << std::endl;
}

void Viewer::IncreaseSelectedEigenVector(int delta) {
    int eig = (myViewer->selectedEVec + delta) % myViewer->nEVecsToDraw;
    std::cout << "Selecting eigenvector " << eig << "... ";
    myViewer->selectedEVec = eig;
    std::cout << "Done." << std::endl;
}

void Viewer::IncreaseCalculatedEVecs(int delta, int maxNumEigs) {
    int eig = (myViewer->nEVecsToCalculate + delta) % maxNumEigs;
    std::cout << "Changing calculated eigenvectors to " << eig << "... ";
    myViewer->nEVecsToCalculate = eig;
    std::cout << "Done." <<std::endl;
}

void Viewer::IncreaseNShownEigenVectors(int delta) {
    int eig = (myViewer->nEVecsToDraw + delta) % myViewer->nEVecsToCalculate;
    std::cout << "Reconstructing mesh with " << eig <<  " eigenvectors... ";
    myViewer->nEVecsToDraw = eig;
    myViewer->mesh_list[myViewer->primary_mesh].reconstructMesh(myViewer->nEVecsToDraw);
    std::cout << "Done." <<std::endl;
}

void Viewer::ReconstructMesh() {
    std::cout << "Reconstructing first " << myViewer->nEVecsToCalculate << " eigenvectors... ";
    myViewer->mesh_list[myViewer->primary_mesh].reconstructMesh(myViewer->nEVecsToCalculate);
    std::cout << "Done." << std::endl;
}

void Viewer::RecomputeEigenvectors() {
    std::cout << "Recomputing " << myViewer->nEVecsToCalculate << " eigenvectors... ";
    myViewer->mesh_list[myViewer->primary_mesh].recomputeEigenvectors(myViewer->nEVecsToCalculate);
    std::cout << "Done." << std::endl;
}

void Viewer::ReconstructMesh(Laplacian laplacian, EigenSolver solverType, Reconstruction reconType) {
    auto mesh = &myViewer->mesh_list[myViewer->primary_mesh];
    auto L = laplacian == Uniform ? mesh->computeUniformLaplacian() : mesh->computeCotanLaplacian();
    auto evecs = solverType == Exact ? mesh->exactlyDecomposeLaplacian(L)
                                     : mesh->approximatelyDecomposeLaplacian(myViewer->nEVecsToCalculate, L);
    reconType == SmallestEigenvalues ? mesh->reconstructBySmallestEigenvalue(evecs, myViewer->nEVecsToDraw)
                                     : mesh->reconstructByLargestSpectralCoefficient(evecs, myViewer->nEVecsToDraw);
}

void Viewer::PerformSpectralTests() {
    auto testMesh = &myViewer->mesh_list[myViewer->primary_mesh];
    auto N = testMesh->numVertices();
    MatrixXd approxEigs;
    MatrixXd exactEigs;

    // perform the tests on each reconstructed mesh

    auto Luni = testMesh->computeUniformLaplacian();
    auto Lcot = testMesh->computeCotanLaplacian();

    auto exactThreshold = 1000;

    if (N <= exactThreshold) {
        // set the laplacian for uniform
        std::cout << "Calculating exact uniform laplacian... " << std::flush;
        exactEigs = testMesh->exactlyDecomposeLaplacian(Luni);
        std::cout << "Done." << std::endl;

        std::cout << "----------------------------------------------------------------------------------" << std::endl;
        std::cout << "Exact Uniform Eigenvalue" << std::endl;
        std::cout << "----------------------------------------------------------------------------------" << std::endl;

        std::ofstream out1("results/exact_uniform_eigenvalues.txt");
        for (int i = 3; i < myViewer->nEVecsToCalculate; i += (i / 4) + 1) {

            std::cout << "Testing exact uniform eigenvalue reconstruction at eig_n = " << i << std::endl;
            // reconstruct the mesh
            testMesh->reconstructBySmallestEigenvalue(exactEigs, i);

            // check the difference
            double ssd = testMesh->ssd();

            out1 << i << "\t" << ssd << std::endl;
        }
        out1.close();

        std::cout << "----------------------------------------------------------------------------------" << std::endl;
        std::cout << "Exact Uniform Coefficient" << std::endl;
        std::cout << "----------------------------------------------------------------------------------" << std::endl;

        std::ofstream out2("results/exact_uniform_coeff.txt");
        for (int i = 3; i < myViewer->nEVecsToCalculate; i += (i / 4) + 1) {

            std::cout << "Testing exact uniform coeff reconstruction at eig_n = " << i << std::endl;
            // reconstruct the mesh
            testMesh->reconstructByLargestSpectralCoefficient(exactEigs, i);

            // check the difference
            double ssd = testMesh->ssd();

            out2 << i << "\t" << ssd << std::endl;
        }
        out2.close();

        // reset the laplacian for cotan
        std::cout << "Calculating exact cotan laplacian... " << std::flush;
        exactEigs = testMesh->exactlyDecomposeLaplacian(Lcot);
        std::cout << "Done." << std::endl;

        std::cout << "----------------------------------------------------------------------------------" << std::endl;
        std::cout << "Exact Cotan Eigenvalue" << std::endl;
        std::cout << "----------------------------------------------------------------------------------" << std::endl;

        std::ofstream out3("results/exact_cotan_eigenvalues.txt");
        for (int i = 3; i < myViewer->nEVecsToCalculate; i += (i / 4) + 1) {

            std::cout << "Testing exact cotan eigenvalue reconstruction at eig_n = " << i << std::endl;
            // reconstruct the mesh
            testMesh->reconstructBySmallestEigenvalue(exactEigs, i);

            // check the difference
            double ssd = testMesh->ssd();

            out3 << i << "\t" << ssd << std::endl;
        }
        out3.close();

        std::cout << "----------------------------------------------------------------------------------" << std::endl;
        std::cout << "Exact Cotan Coefficient" << std::endl;
        std::cout << "----------------------------------------------------------------------------------" << std::endl;

        std::ofstream out4("results/exact_cotan_coeff.txt");
        for (int i = 3; i < myViewer->nEVecsToCalculate; i += (i / 4) + 1) {

            std::cout << "Testing exact cotan coeff reconstruction at eig_n = " << i << std::endl;
            // reconstruct the mesh
            testMesh->reconstructByLargestSpectralCoefficient(exactEigs, i);

            // check the difference
            double ssd = testMesh->ssd();

            out4 << i << "\t" << ssd << std::endl;
        }
        out4.close();
    }
    else {
        std::cout << "Skipping exact test because N is too large..." << std::endl;
    }


    // set the laplacian for uniform


    std::cout << "----------------------------------------------------------------------------------" << std::endl;
    std::cout << "Approximate Uniform Eigenvalues" << std::endl;
    std::cout << "----------------------------------------------------------------------------------" << std::endl;

    std::ofstream out1("results/approx_uniform_eigenvalues.txt");
    for (int i = 3; i < myViewer->nEVecsToCalculate; i += (i / 4) + 1) {
        approxEigs = testMesh->approximatelyDecomposeLaplacian(i, Luni);

        std::cout << "Testing approx uniform eigenvalue reconstruction at eig_n = " << i << std::endl;
        // reconstruct the mesh
        testMesh->reconstructBySmallestEigenvalue(approxEigs, i);

        // check the difference
        double ssd = testMesh->ssd();

        out1 << i << "\t" << ssd << std::endl;
    }
    out1.close();

    std::cout << "----------------------------------------------------------------------------------" << std::endl;
    std::cout << "Approximate Uniform Coefficients" << std::endl;
    std::cout << "----------------------------------------------------------------------------------" << std::endl;

    std::ofstream out2("results/approx_uniform_coeff.txt");
    for (int i = 3; i < myViewer->nEVecsToCalculate; i += (i / 4) + 1) {
        approxEigs = testMesh->approximatelyDecomposeLaplacian(i, Luni);

        std::cout << "Testing approx uniform coeff reconstruction at eig_n = " << i << std::endl;
        // reconstruct the mesh
        testMesh->reconstructByLargestSpectralCoefficient(approxEigs, i);

        // check the difference
        double ssd = testMesh->ssd();

        out2 << i << "\t" << ssd << std::endl;
    }
    out2.close();

    std::cout << "----------------------------------------------------------------------------------" << std::endl;
    std::cout << "Approximate Cotan Eigenvalues" << std::endl;
    std::cout << "----------------------------------------------------------------------------------" << std::endl;

    std::ofstream out3("results/approx_cotan_eigenvalues.txt");
    for (int i = 3; i < myViewer->nEVecsToCalculate; i += (i / 4) + 1) {
        approxEigs = testMesh->approximatelyDecomposeLaplacian(i, Lcot);

        std::cout << "Testing approx cotan eigenvalue reconstruction at eig_n = " << i << std::endl;
        // reconstruct the mesh
        testMesh->reconstructBySmallestEigenvalue(approxEigs, i);

        // check the difference
        double ssd = testMesh->ssd();

        out3 << i << "\t" << ssd << std::endl;
    }
    out3.close();

    std::cout << "----------------------------------------------------------------------------------" << std::endl;
    std::cout << "Approximate Cotan Coefficients" << std::endl;
    std::cout << "----------------------------------------------------------------------------------" << std::endl;

    std::ofstream out4("results/approx_cotan_coeff.txt");
    for (int i = 3; i < myViewer->nEVecsToCalculate; i += (i / 4) + 1) {
        approxEigs = testMesh->approximatelyDecomposeLaplacian(i, Lcot);

        std::cout << "Testing approx cotan coeff reconstruction at eig_n = " << i << std::endl;
        // reconstruct the mesh
        testMesh->reconstructByLargestSpectralCoefficient(approxEigs, i);

        // check the difference
        double ssd = testMesh->ssd();

        out4 << i << "\t" << ssd << std::endl;
    }
    out4.close();



    std::cout << "----------------------------------------------------------------------------------" << std::endl;
    std::cout << "Noise Resiliency 0.002" << std::endl;
    std::cout << "----------------------------------------------------------------------------------" << std::endl;

    std::ofstream out5("results/noise_0.002.txt");
    testMesh->reset();
    testMesh->addNoise(0.002);
    for (int i = 3; i < myViewer->nEVecsToCalculate; i += (i / 4) + 1) {
        approxEigs = testMesh->approximatelyDecomposeLaplacian(i, Luni);

        std::cout << "Testing noise resiliency of 0.002 at eig_n = " << i << std::endl;
        // reconstruct the mesh
        testMesh->reconstructBySmallestEigenvalue(approxEigs, i);

        // check the difference
        double ssd = testMesh->ssd();

        out5 << i << "\t" << ssd << std::endl;
    }
    testMesh->reset();
    out5.close();

    std::cout << "----------------------------------------------------------------------------------" << std::endl;
    std::cout << "Noise Resiliency 0.005" << std::endl;
    std::cout << "----------------------------------------------------------------------------------" << std::endl;

    std::ofstream out6("results/noise_0.005.txt");
    testMesh->addNoise(0.005);
    for (int i = 3; i < myViewer->nEVecsToCalculate; i += (i / 4) + 1) {
        approxEigs = testMesh->approximatelyDecomposeLaplacian(i, Luni);

        std::cout << "Testing noise resiliency of 0.005 at eig_n = " << i << std::endl;
        // reconstruct the mesh
        testMesh->reconstructBySmallestEigenvalue(approxEigs, i);

        // check the difference
        double ssd = testMesh->ssd();

        out6 << i << "\t" << ssd << std::endl;
    }
    testMesh->reset();
    out6.close();

    std::cout << "----------------------------------------------------------------------------------" << std::endl;
    std::cout << "Noise Resiliency 0.01" << std::endl;
    std::cout << "----------------------------------------------------------------------------------" << std::endl;

    std::ofstream out7("results/noise_0.01.txt");
    testMesh->addNoise(0.01);
    for (int i = 3; i < myViewer->nEVecsToCalculate; i += (i / 4) + 1) {
        approxEigs = testMesh->approximatelyDecomposeLaplacian(i, Luni);

        std::cout << "Testing noise resiliency of 0.01 at eig_n = " << i << std::endl;
        // reconstruct the mesh
        testMesh->reconstructBySmallestEigenvalue(approxEigs, i);

        // check the difference
        double ssd = testMesh->ssd();

        out7 << i << "\t" << ssd << std::endl;
    }
    testMesh->reset();
    out7.close();

    std::cout << "----------------------------------------------------------------------------------" << std::endl;
    std::cout << "Noise Resiliency 0.1" << std::endl;
    std::cout << "----------------------------------------------------------------------------------" << std::endl;

    std::ofstream out8("results/noise_0.1.txt");
    testMesh->addNoise(0.1);
    for (int i = 3; i < myViewer->nEVecsToCalculate; i += (i / 4) + 1) {
        approxEigs = testMesh->approximatelyDecomposeLaplacian(i, Luni);

        std::cout << "Testing noise resiliency of 0.1 at eig_n = " << i << std::endl;
        // reconstruct the mesh
        testMesh->reconstructBySmallestEigenvalue(approxEigs, i);

        // check the difference
        double ssd = testMesh->ssd();

        out8 << i << "\t" << ssd << std::endl;
    }
    testMesh->reset();
    out8.close();

    std::cout << "Done." << std::endl;
}

void Viewer::ResetMeshes() { myViewer->mesh_list[myViewer->primary_mesh].reset(); }

void Viewer::addMesh(TriMesh triMesh) {
    // Add mesh curvatures as mesh properties
    mesh_list.push_back(triMesh);

    // update the global translation
    if (mesh_list.size() == 1) {
        global_translation = GetCentreOfMesh(); //get centre
        global_translation *= -6.0f;
    }

    // Initialise various functions so I can view and draw the meshes
    FindNormalsAndCurvature();
}