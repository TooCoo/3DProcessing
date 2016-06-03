//
// Created by Ben Eisner on 5/25/16.
//

#ifndef INC_3D_PROCESSING_TRIMESH_H
#define INC_3D_PROCESSING_TRIMESH_H

// OpenMesh
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/PolyConnectivity.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SparseCore>
#include <eigen3/Eigen/Sparse>

#include "enums.h"

using Eigen::MatrixXd;

struct CurvatureProperties {

    double max_mean_curvature = -10000.0;
    double max_gauss_curvature = -10000.0;
    double min_gauss_curvature = 100000000;
    double max_k1_curvature = -10000.0;
    double min_k1_curvature = 10000.0;
    double max_k2_curvature = -10000.0;
    double min_k2_curvature = 10000.0;
    double min_mean_curvature = 10000.0;
};

struct MyTraits : public OpenMesh::DefaultTraits {
    typedef OpenMesh::Vec3f Point;
    typedef OpenMesh::Vec3f Normal;
    typedef OpenMesh::Vec4f Color;

    VertexTraits{
        public:
            const unsigned int valance() const { return valence; }
            void set_valence(const unsigned int v) { valence = v; }
        private:
            unsigned int valence;
    };

    VertexAttributes(OpenMesh::Attributes::Normal | OpenMesh::Attributes::Color);
    FaceAttributes(OpenMesh::Attributes::Normal | OpenMesh::Attributes::Color);

};

typedef OpenMesh::PolyMesh_ArrayKernelT<MyTraits> BaseMesh;

class TriMesh {
private:
    int k = 5;				// number of nearest neighbors // I'm using 5 as one of the nearest points is the point
    const int dim = 3;	    // dimension
    double eps = 0;			// error bound

    // internal representations
    BaseMesh mesh;                  // original mesh
    BaseMesh intermediateMesh;      // original mesh, but potentially with noise
    BaseMesh trulyUnchangedMesh;    // transformed mesh

    // keep track of the type of eigenvectors we've calculated and the reconstruction
    Laplacian laplacian = Uniform;
    EigenSolver solver = Approximate;
    Reconstruction reconstruction = LargestCoefficients;

    // spectral parts
    int numEigsCalculated = 0;
    Eigen::MatrixXd evecs;
    Eigen::MatrixXd evecs_coeffs;
    Eigen::VectorXcd evalues;
    Eigen::MatrixXd displacementValues;

    // laplacian helpers
    double calculateBarycentricArea(OpenMesh::PolyConnectivity::VertexIter vlt);
    std::vector<OpenMesh::VertexHandle> getNeighbors(OpenMesh::PolyConnectivity::VertexIter vlt);
    void colourVertex(OpenMesh::VertexHandle _vh, OpenMesh::Vec4f col);

    // mapping functions
    void mapOverVertices(std::function<void(OpenMesh::PolyConnectivity::VertexIter)> func);
    void mapOverVertexNeighbors(OpenMesh::PolyConnectivity::VertexIter vlt, std::function<void(OpenMesh::PolyConnectivity::ConstVertexVertexIter)> func);


public:
    // --------------------------------------------------------------
    // Basic Functionality
    // --------------------------------------------------------------
    TriMesh(BaseMesh baseMesh);

    // coloring
    void colourOverlap(TriMesh otherMesh);
    void colourByNormals();
    void colourPhong(Eigen::Vector3d material, Eigen::Vector3d lightDir, Eigen::Vector3d ambientLight,
                     Eigen::Vector3d specular, Eigen::Vector3d diffuse, Eigen::Vector3d virtual_camera_location);
    void colourRandomly();

    // transformation
    void translate(float _dir, float move_amount, int current_axis);
    void translate(OpenMesh::Vec3f);
    void rotate(float _dir, int current_axis);
    OpenMesh::Vec3f getCentreOfMesh();

    // basic operations
    void save();    // save to a file
    void display(DisplayMode displayMode, ColorMode colorMode, int whichEigToDraw);
    void reset();   // reset to the trulyUnchanged mesh

    // mesh properties
    int numVertices();
    int numVertexNeighbors(OpenMesh::PolyConnectivity::VertexIter vlt);

    // --------------------------------------------------------------
    // Curvature and distortion
    // --------------------------------------------------------------

    // normals and curvature
    void findVertexNormalsFromFaces();
    void findFaceNormals();
    void findGaussianCurvature();
    OpenMesh::Vec3f simpleColourMap(float value, float max, float min);
    OpenMesh::Vec3f principleKColourmap(double k1, double k2);

    // distortion
    void addNoise(float _sigma);
    void applyDiffusionFlow(double lambda);
    void implicitLaplacianMeshSmoothing(double lambda);

    // --------------------------------------------------------------
    // Spectral decomposition
    // --------------------------------------------------------------

    // laplacian operator construction
    Eigen::SparseMatrix<double, 0, int> computeUniformLaplacian();
    Eigen::SparseMatrix<double, 0, int> computeCotanLaplacian();

    // eigendecomposition
    Eigen::MatrixXd exactlyDecomposeLaplacian(Eigen::SparseMatrix<double> L);
    Eigen::MatrixXd approximatelyDecomposeLaplacian(int nSmallestEigs, Eigen::SparseMatrix<double> L);

    // mesh reconstruction
    void reconstructMesh(int nEigs);
    void recomputeEigenvectors(int nLargestEigs);
    void reconstructBySmallestEigenvalue(Eigen::MatrixXd eigenVecs, int nSmallestEigs);
    void reconstructByLargestSpectralCoefficient(Eigen::MatrixXd eigenVecs, int nLargestCoeffs);

    // change the coefficients for the eigenvectors
    void increaseEVecCoeff(int eig, double coeff);

    // testing and evaluation
    double ssd();
};

#endif //INC_3D_PROCESSING_TRIMESH_H
