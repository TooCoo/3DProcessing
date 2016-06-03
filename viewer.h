//
// Created by Ben Eisner on 5/24/16.
//

#ifndef INC_3D_PROCESSING_VIEWER_H
#define INC_3D_PROCESSING_VIEWER_H

#include "enums.h"
#include "viewer_ui_components.h"
#include "trimesh.h"


#include <vector>

// GLFW
#include <GLFW/glfw3.h>

// GLUT
#include <include/GL/freeglut.h>

// OpenMesh
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/PolyConnectivity.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>

// Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SparseCore>
#include <eigen3/Eigen/Sparse>


// parameters
#define transformDelta 1.0f
#define rotationDelta 5.0f
#define zoomDelta 1.0f
#define centerScalar -4.0f
#define implicitLambda -1.0
#define diffusionLambda -0.0000001
#define noiseSigma 0.005f

class Viewer {
private:

    // --------------------------------------------------------------
    // Display
    // --------------------------------------------------------------

    // lighting and camera
    Eigen::Vector3d material = Eigen::Vector3d(0.9f, 0.9f, 0.9f);
    Eigen::Vector3d lightDir = Eigen::Vector3d(0.5f, 0.5f, 0.0f);
    Eigen::Vector3d ambientLight = Eigen::Vector3d(0.3f, 0.3f, 0.3f);
    Eigen::Vector3d diffuse = Eigen::Vector3d(0.5f, 0.5f, 0.5f);
    Eigen::Vector3d specular = Eigen::Vector3d(0.6f, 0.6f, 0.6f);
    Eigen::Vector3d virtualCameraLocation = Eigen::Vector3d(0.0f, -1.0f, 0.0f);
    double alpha_shininess = 0.5f;

    // global transformation
    float globalScale = 1.0f;
    float global_rotation = 0.0f;
    OpenMesh::Vec3f global_translation = OpenMesh::Vec3f();

    int window_w = 640;
    int window_h = 480;

    // --------------------------------------------------------------
    // UI Components
    // --------------------------------------------------------------

    // mouse
    Mouse mouse = { 0,0,0,0,0 };

    // buttons
    std::vector<Button> buttonList;
    std::vector<TextBox> textBoxList;
    void DrawButton(Button* b);
    void DrawTextBoxes(TextBox *tb);
    void DrawText(void *font, const char *text, int x, int y);

    // --------------------------------------------------------------
    // UI State and Constants
    // --------------------------------------------------------------

    // edit modes
    bool editMode = false;
    EditType editType = RotationMode;
    ColorMode colorMode = None;
    DisplayMode displayMode = Smooth;

    // edit values
    int current_axis = 0; //x = 0, y = 1, z = 2
    float move_amount = 0.0001;

    // meshes
    std::vector<TriMesh> mesh_list;
    int primary_mesh = 0;
    int secondary_mesh = 1;

    // spectral properties
    int nEVecsToCalculate = 3;
    int nEVecsToDraw = 3;
    int selectedEVec = 1;
    double eig_inc = 0.1;

    // --------------------------------------------------------------
    // UI Static Functions
    // --------------------------------------------------------------

    // display mode functions
    static void SetDisplayMode(DisplayMode mode);
    static void SetColoringMode(ColorMode mode);

    // coloring
    static void ColourByPhong();
    static void ColourByNormals();
    static void ColourRandomly();
    static void ColourOverlap();

    // move meshes
    static void MoveMeshesToCommonCentre();
    static void MoveMeshesToOrigin();
    static void EditModeTranslate(float _dir);
    static void EditModeRotate(float _dir);
    static OpenMesh::Vec3f GetCentreOfMesh();

    // normals and curvature
    static void FindNormalsAndCurvature();
    static void FindVertexNormalsFromFaces();
    static void FindFaceNormals();
    static void FindGaussianCurvature();

    // noise
    static void AddNoise(float sigma);

    // smoothing
    static void ApplyDiffusionFlow(double lambda);
    static void ImplicitLaplacianMeshSmoothing(double lambda);

    // spectral display/calculation state
    static void IncreaseCalculatedEVecs(int delta, int maxNumEigs);
    static void IncreaseNShownEigenVectors(int delta);
    static void IncreaseSelectedEigenVector(int delta);
    static void IncreaseSpectralCoeff(int delta);

    // spectral tasks
    static void RecomputeEigenvectors();
    static void ReconstructMesh();
    static void ReconstructMesh(Laplacian laplacian, EigenSolver solverType, Reconstruction reconType);
    static void PerformSpectralTests();

    // general mesh functions
    static void SaveMesh();
    static void ResetMeshes();

    // --------------------------------------------------------------
    // UI Callbacks
    // --------------------------------------------------------------

    static void error_callback(int error, const char* description);
    static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void mouse_callback(GLFWwindow* window, int button, int action, int mods);
    static void mouse_moved_callback(GLFWwindow* window, double x_pos, double y_pos);

    // --------------------------------------------------------------
    // UI Functions
    // --------------------------------------------------------------

    // display
    void display(GLFWwindow* window);

public:
    Viewer(int nEVecsToUse);
    void addMesh(TriMesh triMesh);
    void display();
};


#endif //INC_3D_PROCESSING_VIEWER_H
