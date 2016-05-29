//
// Created by Ben Eisner on 5/24/16.
//

#ifndef INC_3D_PROCESSING_VIEWER_H
#define INC_3D_PROCESSING_VIEWER_H

#include "enums.h"
#include "button.h"
#include "mouse.h"
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

typedef void(*ButtonCallback)();

class Viewer {
private:

    // unchanged mesh
    TriMesh *unchangedMesh;

    // lighting properties
    Eigen::Vector3d material = Eigen::Vector3d(0.9f, 0.9f, 0.9f);
    Eigen::Vector3d lightDir = Eigen::Vector3d(0.5f, 0.5f, 0.0f);
    Eigen::Vector3d ambientLight = Eigen::Vector3d(0.3f, 0.3f, 0.3f);
    Eigen::Vector3d diffuse = Eigen::Vector3d(0.5f, 0.5f, 0.5f);
    Eigen::Vector3d specular = Eigen::Vector3d(0.6f, 0.6f, 0.6f);
    double alpha_shininess = 0.5f;

    Eigen::Vector3d virtual_camera_location = Eigen::Vector3d(0.0f, -1.0f, 0.0f);

    // global rotation
    float globalScale = 1.0f;
    float global_rotation = 0.0f;
    OpenMesh::Vec3f global_translation = OpenMesh::Vec3f();

    //window dimensions
    int window_w = 640;
    int window_h = 480;

    // mouse
    mouse TheMouse = { 0,0,0,0,0 };

    // buttons
    std::vector<Button> buttonList;
    void DrawButton(Button* b);
    void Font(void *font, const char *text, int x, int y);

    // edit modes
    bool editmode = false;
    EditType editType = RotationMode;
    ColorMode colorMode = None;
    DisplayMode displayMode = Smooth;

    int current_axis = 0; //x = 0, y = 1, z = 2
    float move_amount = 0.0001;

    // spectral properties
    int nEVecsToUse = 10;
    double eig_inc = 0.5;


    // button functions
    static void TogglePointCloud();
    static void ToggleShading();
    static void ToggleWireFrame();
    static void ToggleShowPrincipleCurvature();
    static void ToggleShowMeanCurvature();
    static void ToggleShowGaussCurvature();
    static void ToggleShowSpectralWeighting();

    // spectral-specific code
    int whichEigToDraw = 5;

    // display
    void display(GLFWwindow* window);



    // coloring
    void colourPhong();
    void colourByNormals();
    void colourRandomly();

    // move meshes
    void moveMeshesToCommonCentre();
    void moveMeshesToOrigin();
    void editModeTranslate(float _dir);
    void editModeRotate(float _dir);
    OpenMesh::Vec3f getCentreOfMesh();

    // normals
    void findVertexNormalsFromFaces();
    void findFaceNormals();

    // noise
    void addNoise(float _sigma);

    // curvature
    void uniformLaplaceDiscretization();
    void discreteLaplaceDiscretization();
    void findGaussianCurvature();
    void findGaussianCurvature2();
    void applyDiffusionFlow(double lambda);
    void implicitLaplacianMeshSmoothing(double lambda);

    // spectral
    void eigenReconstruction(double lambda, int nLargestEigs);
    void findEigenVectors(int nLargestEigs);
    void remakeFromEVecs(int nLargestEigs);

    // UI callbacks
    static void error_callback(int error, const char* description);
    static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void mouse_callback(GLFWwindow* window, int button, int action, int mods);
    static void mouse_moved_callback(GLFWwindow* window, double x_pos, double y_pos);

public:

    // meshes
    std::vector<TriMesh> mesh_list;
    int current_mesh = 0;

    Viewer(int nEVecsToUse);
    void addMesh(TriMesh triMesh);
    void setUnchangedMesh(TriMesh *triMesh);
    void display();
};


#endif //INC_3D_PROCESSING_VIEWER_H
