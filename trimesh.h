//
// Created by Ben Eisner on 5/25/16.
//

#ifndef INC_3D_PROCESSING_TRIMESH_H
#define INC_3D_PROCESSING_TRIMESH_H

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/PolyConnectivity.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SparseCore>
#include <eigen3/Eigen/Sparse>
#include "enums.h"


using Eigen::MatrixXd;



struct CurvatureProperties {
    OpenMesh::VPropHandleT<double>       mean_curvature;
    OpenMesh::VPropHandleT<double>       gauss_curvature;
    OpenMesh::VPropHandleT<double>       k1;
    OpenMesh::VPropHandleT<double>       k2;
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
    //typedef float mean_curvature;
    VertexTraits{
        public:

        const unsigned int valance() const { return valence; }
        void set_valence(const unsigned int v) { valence = v; }
        //float meanCurvature() { return mean_curvature; }
        //void setMeanCurvature(float H) { mean_curvature = H; }
        //void set_normal(OpenMesh::Vec3f fn) { Normal = fn; }

        private:
        unsigned int valence;
        //float mean_curvature;
    };

    VertexAttributes(
            OpenMesh::Attributes::Normal |
    OpenMesh::Attributes::Color);


    FaceAttributes(
            OpenMesh::Attributes::Normal |
    OpenMesh::Attributes::Color);


};

typedef OpenMesh::PolyMesh_ArrayKernelT<MyTraits> BaseMesh;

class TriMesh {
private:
    void colourVertex(OpenMesh::VertexHandle _vh, OpenMesh::Vec4f col);

    int k = 5;				// number of nearest neighbors // I'm using 5 as one of the nearest points is the point
    const int dim = 3;	    // dimension
    double eps = 0;			// error bound
    int maxPts = 1000;		// maximum number of data points //this is changed lower down...
    BaseMesh baseMesh;

public:
    // spectral parts
    Eigen::MatrixXd evecs;
    Eigen::MatrixXd evecs_coeffs;
    Eigen::MatrixXd displacementValues;

    TriMesh(BaseMesh baseMesh);
    void colourOverlap(TriMesh otherMesh);
    void colourByNormals();
    void colourPhong(Eigen::Vector3d material, Eigen::Vector3d lightDir, Eigen::Vector3d ambientLight,
                     Eigen::Vector3d specular, Eigen::Vector3d diffuse, Eigen::Vector3d virtual_camera_location);
    void colourRandomly();
    void findVertexNormalsFromFaces();
    void findFaceNormals();
    void save();
    void translate(float _dir, float move_amount, int current_axis);
    void translate(OpenMesh::Vec3f);
    void rotate(float _dir, int current_axis);
    void addNoise(float _sigma);
    OpenMesh::Vec3f getCentreOfMesh();
    void applyTranslation(MatrixXd _R, MatrixXd _T);
    void applyTranslation(MatrixXd _x);

    // curvature
    void uniformLaplaceDiscretization();
    void discreteLaplaceDiscretization();
    void findGaussianCurvature();
    void findGaussianCurvature2();
    void applyDiffusionFlow(double lambda);
    void implicitLaplacianMeshSmoothing(double lambda);
    OpenMesh::Vec3f simpleColourMap(float value, float max, float min);
    OpenMesh::Vec3f principleKColourmap(double k1, double k2);

    // spectral reconstruction
    void eigenReconstruction(double lambda, int nLargestEigs);
    void findEigenVectors(int nLargestEigs);
    void remakeFromEVecs(int nLargestEigs, TriMesh *unchangedMesh);

    // display
    void display(DisplayMode displayMode, ColorMode colorMode, int whichEigToDraw);

    int n_verts();
    int n_faces();
};

#endif //INC_3D_PROCESSING_TRIMESH_H
