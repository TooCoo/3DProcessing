//
// Created by Ben Eisner on 5/25/16.
//

// standard libraries
#include <random>

// OpenMesh
#include <OpenMesh/Core/IO/MeshIO.hh>

// ANN
#include <include/ANN/ANN.h>

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/SparseCore>
#include <eigen3/Eigen/Sparse>

// GLFW
#include <GLFW/glfw3.h>

// local imports
#include "trimesh.h"

#define PI 3.14159265

// 15 degrees
#define COS15DEGREE 0.96592582628
#define SIN15DEGREE 0.2588190451

extern CurvatureProperties curveProps;
extern OpenMesh::VPropHandleT<double>       mean_curvature;
extern OpenMesh::VPropHandleT<double>       gauss_curvature;
extern OpenMesh::VPropHandleT<double>       k1;
extern OpenMesh::VPropHandleT<double>       k2;


struct PointPair {
    OpenMesh::Vec3f p;
    OpenMesh::Vec3f q;
    float d;
    int pi;
    int qi;
};

typedef struct PointPair PointPair;

void TriMesh::colourVertex(OpenMesh::VertexHandle _vh, OpenMesh::Vec4f col) {
    mesh.set_color(_vh, col);
}

void TriMesh::colourOverlap(TriMesh otherMesh) {
    //first of all lets find some nearest neighbours
    //let us use a subset picked at random of around 1% of the points in each mesh
    int initial_k = k;
    k = 1;

    //use one mesh as the data set and one as the query set

    BaseMesh::VertexIter vlt_0, vBegin_0, vEnd_0;
    vBegin_0 = mesh.vertices_begin();
    vEnd_0 = mesh.vertices_end();

    BaseMesh::VertexIter vlt_1, vBegin_1, vEnd_1;
    vBegin_1 = otherMesh.mesh.vertices_begin();
    vEnd_1 = otherMesh.mesh.vertices_end();

    //_______________________________ Set up ANN kdtree, etc...
    int maxPts = (int) mesh.n_vertices();		// maximum number of data points
    //int maxPts = 1000000;		// maximum number of data points
    int					nPts;					// actual number of data points
    ANNpointArray		dataPts;				// data points
    ANNpoint			queryPt;				// query point
    ANNidxArray			nnIdx;					// near neighbor indices
    ANNdistArray		dists;					// near neighbor distances
    ANNkd_tree*			kdTree;					// search structure

    queryPt = annAllocPt(dim);					// allocate query point
    dataPts = annAllocPts(maxPts, dim);			// allocate data points
    nnIdx = new ANNidx[k];						// allocate near neigh indices
    dists = new ANNdist[k];						// allocate near neighbor dists

    nPts = 0;									// read data points

    //_______________________________


    //now read all mesh points into the query points
    //each point - read into ANN

    //adding points from base mesh to data points set
    for (vlt_0 = vBegin_0; vlt_0 != vEnd_0; ++vlt_0) {
        for (int i = 0; i < dim; i++) {
            dataPts[nPts][i] = mesh.point(*vlt_0)[i];
        }
        nPts++;
    }

    //build the kdtree
    kdTree = new ANNkd_tree(					// build search structure
            dataPts,					// the data points
            nPts,						// number of points
            dim);						// dimension of space



    //each point //use ANN to find nearest neighbour
    std::vector<PointPair> neighbour_list;


    for (vlt_1 = vBegin_1; vlt_1 != vEnd_1; ++vlt_1) {

        //set query point
        for (int i = 0; i < dim; i++) {
            queryPt[i] = otherMesh.mesh.point(*vlt_1)[i];

        }

        //find nearest neighbour
        kdTree->annkSearch(						// search
                queryPt,						// query point
                k,								// number of near neighbors
                nnIdx,							// nearest neighbors (returned)
                dists,							// distance (returned)
                eps);							// error bound


        int pid, qid;
        pid = nnIdx[0];
        qid = vlt_1->idx();

        PointPair this_pp = PointPair{mesh.point(OpenMesh::VertexHandle(nnIdx[0])), otherMesh.mesh.point(*vlt_1), (float)dists[0], pid, qid};

        neighbour_list.push_back(this_pp);

    }

    float average_distance = 0.0f;

    for (int i = 0; i < neighbour_list.size(); i++) {
        average_distance += neighbour_list[i].d;
    }

    average_distance *= (1.0f / (float)neighbour_list.size());

    std::cout << "average_distance: " << average_distance << "\n";
    float cutoff_distance = 1.0f;
    cutoff_distance *= average_distance;
    cutoff_distance += 0.000001;

    //colour if not close
    OpenMesh::Vec4f aligned_colour = OpenMesh::Vec4f(0.0f, 1.0f, 0.0f, 1.0f);
    OpenMesh::Vec4f unaligned_colour = OpenMesh::Vec4f(1.0f, 0.0f, 0.0f, 1.0f);

    for (int neighbour_list_i = 0; neighbour_list_i < neighbour_list.size(); neighbour_list_i++) {
        if (cutoff_distance > neighbour_list[neighbour_list_i].d) {
            colourVertex(OpenMesh::VertexHandle(neighbour_list_i), aligned_colour);
        }
        else {
            colourVertex(OpenMesh::VertexHandle(neighbour_list_i), unaligned_colour);
        }
    }


    // clean things up
    delete[] nnIdx;
    delete[] dists;
    delete kdTree;
    delete queryPt;
    delete dataPts;
    annClose();


    //return k to its origonal value,
    k = initial_k;
}

void TriMesh::colourByNormals() {
    BaseMesh::VertexIter vlt, vBegin, vEnd;
    vBegin = mesh.vertices_begin();
    vEnd = mesh.vertices_end();

    for (vlt = vBegin; vlt != vEnd; ++vlt) {
        //do something here

        float r = mesh.normal(*vlt)[0];
        float g = mesh.normal(*vlt)[1];
        float b = mesh.normal(*vlt)[2];


        if (r < 0) r *= -1.0f;
        if (r < 0.1) r = 0.1f;

        OpenMesh::Vec4f newCol = OpenMesh::Vec4f(r, r, r, 1.0f);

        colourVertex(*vlt, newCol);
    }
}

void TriMesh::colourPhong(Eigen::Vector3d material, Eigen::Vector3d lightDir, Eigen::Vector3d ambientLight,
                          Eigen::Vector3d specular, Eigen::Vector3d diffuse, Eigen::Vector3d virtual_camera_location) {
    BaseMesh::VertexIter vlt, vBegin, vEnd;
    vBegin = mesh.vertices_begin();
    vEnd = mesh.vertices_end();

    //std::cout << vBegin << "\t" << vEnd << "\n";


    for (vlt = vBegin; vlt != vEnd; ++vlt) {


        Eigen::Vector3d n(mesh.normal(*vlt)[0], mesh.normal(*vlt)[1], mesh.normal(*vlt)[2]);
        n.normalize();
        //light location = lightDir
        //n = mesh_list[primary_mesh].normal(*vlt)
        Eigen::Vector3d reflection = 2.0 * (lightDir.dot(n)) * n - lightDir;
        reflection.normalize();
        float ks = material[0];
        float kd = material[1];
        float ka = material[2];

        Eigen::Vector3f this_colour(0, 0, 0);


        float ambient_component = ka * ambientLight[0];
        float specular_component = ks * (reflection.dot(virtual_camera_location) * specular[0]);
        float diffuse_component = kd * (lightDir.dot(n) * diffuse[0]);
        this_colour(0) = ambient_component + diffuse_component + specular_component;


        //OpenMesh::Vec4f newCol = OpenMesh::Vec4f(r, g, b, 1.0f);
        OpenMesh::Vec4f newCol = OpenMesh::Vec4f(this_colour(0), this_colour(0), this_colour(0), 1.0f);

        colourVertex(*vlt, newCol);
    }
}

void TriMesh::save() {
    try
    {
        if (!OpenMesh::IO::write_mesh(mesh, "output_mesh.ply"))
        {
            std::cerr << "Cannot write mesh to file 'output.off'" << std::endl;
        }
    }
    catch (std::exception& x)
    {
        std::cerr << x.what() << std::endl;
    }
}

void TriMesh::colourRandomly() {
    BaseMesh::VertexIter vlt, vBegin, vEnd;
    vBegin = mesh.vertices_begin();
    vEnd = mesh.vertices_end();

    //std::cout << vBegin << "\t" << vEnd << "\n";

    float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    float g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    float b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

    for (vlt = vBegin; vlt != vEnd; ++vlt) {
        //do something here

        OpenMesh::Vec4f newCol = OpenMesh::Vec4f(r, g, b, 1.0f);

        colourVertex(*vlt, newCol);

    }
}

void TriMesh::findVertexNormalsFromFaces() {

    BaseMesh::VertexIter vlt, vBegin, vEnd;
    vBegin = mesh.vertices_begin();
    vEnd = mesh.vertices_end();

    //itterate over all of the points
    //int n_adjacent = 0;

    for (vlt = vBegin; vlt != vEnd; ++vlt) {
        OpenMesh::VertexHandle vh = *vlt;
        //find the adjecent faces

        OpenMesh::Vec3f thisNormal = OpenMesh::Vec3f(0.0f, 0.0f, 0.0f);

        for (BaseMesh::VertexFaceIter vfi = mesh.vf_iter(vh); vfi.is_valid(); ++vfi) {

            //n_adjacent++;
            thisNormal += mesh.normal(*vfi);

        }

        //Dont need this since I normalize
        //thisNormal *= 1.0f / float(n_adjacent);

        thisNormal = thisNormal.normalize();

        mesh.set_normal(vh, thisNormal);
    }
}

void TriMesh::findFaceNormals() {
//iterate over mesh's faces and find their normal
    for (BaseMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it) {
        //MyMesh::FaceVertexIter OpenMesh::PolyConnectivity::FaceVertexIter OpenMesh::PolyConnectivity::fv_iter(FaceHandle _fh); (FaceHandle _fh);
        OpenMesh::FaceHandle fa = *f_it;

        //There should be three verts for every face.
        OpenMesh::Vec3f v1;
        OpenMesh::Vec3f v2;
        OpenMesh::Vec3f v3;

        int current_v = 0;

        for (BaseMesh::FaceVertexIter fvi = mesh.fv_iter(fa); fvi.is_valid(); ++fvi) {
            OpenMesh::Vec3f thisCoord;
            thisCoord = mesh.point(*fvi);
            current_v++;

            //std::cout << current_v << "\t" << thisCoord << "\n";

            switch (current_v) {
                case 1:
                    v1 = thisCoord;
                    break;
                case 2:
                    v2 = thisCoord;
                    break;
                case 3:
                    v3 = thisCoord;
                    break;
                default:
                    std::cout << "Error: More than three verts on a face!\n";
                    break;
            }

        }

        //system("PAUSE");


        // I should now have the face verts so I can calculate the face normal - Assuming that they are correctly ordered

        Eigen::Vector3f v1v2 = Eigen::Vector3f(v2[0]-v1[0], v2[1] - v1[1], v2[2] - v1[2]);
        Eigen::Vector3f v2v3 = Eigen::Vector3f(v3[0] - v2[0], v3[1] - v2[1], v3[2] - v2[2]);
        Eigen::Vector3f f_normal = v1v2.cross(v2v3);
        f_normal.normalize();

        OpenMesh::Vec3f this_normal = OpenMesh::Vec3f(f_normal(0), f_normal(1), f_normal(2));

        mesh.set_normal(fa, this_normal);

    }
}

void TriMesh::translate(float _dir, float move_amount, int current_axis) {
    BaseMesh::VertexIter vlt, vBegin, vEnd;
    vBegin = mesh.vertices_begin();
    vEnd = mesh.vertices_end();

    OpenMesh::Vec3f translation = OpenMesh::Vec3f()*0.0f;

    translation[current_axis] = 1.0f;
    translation *= (_dir * move_amount);


    for (vlt = vBegin; vlt != vEnd; ++vlt) {

        OpenMesh::Vec3f newCoord;
        newCoord = mesh.point(*vlt);

        newCoord += translation;

        mesh.set_point(*vlt, newCoord);

    }
}

void TriMesh::addNoise(float _sigma) {
    BaseMesh::VertexIter vlt, vBegin, vEnd;
    vBegin = mesh.vertices_begin();
    vEnd = mesh.vertices_end();

    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, _sigma);


    for (vlt = vBegin; vlt != vEnd; ++vlt) {

        OpenMesh::Vec3f newCoord;
        newCoord = mesh.point(*vlt);

        OpenMesh::Vec3f noise = OpenMesh::Vec3f(0.0f, 0.0f, 0.0f);

        for (int dim_i = 0; dim_i < 3; dim_i++) {

            double number = distribution(generator);
            noise[dim_i] = (float)number;

        }

        newCoord += noise;

        mesh.set_point(*vlt, newCoord);
        intermediateMesh.set_point(*vlt, newCoord);

    }
}

void TriMesh::rotate(float _dir, int current_axis) {
    BaseMesh::VertexIter vlt, vBegin, vEnd;
    vBegin = mesh.vertices_begin();
    vEnd = mesh.vertices_end();

    MatrixXd rotation(3, 3);

    double sine_value = SIN15DEGREE * _dir;

    if (current_axis == 0) {
        rotation(0, 0) = 1;
        rotation(0, 1) = 0;
        rotation(0, 2) = 0;

        rotation(1, 0) = 0;
        rotation(1, 1) = COS15DEGREE;
        rotation(1, 2) = sine_value;

        rotation(2, 0) = 0;
        rotation(2, 1) = -1.0 * sine_value;
        rotation(2, 2) = COS15DEGREE;
    }
    else if (current_axis == 1) {
        rotation(0, 0) = COS15DEGREE;
        rotation(0, 1) = 0;
        rotation(0, 2) = -1.0 * sine_value;

        rotation(1, 0) = 0;
        rotation(1, 1) = 1;
        rotation(1, 2) = 0;

        rotation(2, 0) = sine_value;
        rotation(2, 1) = 0;
        rotation(2, 2) = COS15DEGREE;
    }
    else if (current_axis == 2) {
        rotation(0, 0) = COS15DEGREE;
        rotation(0, 1) = sine_value;
        rotation(0, 2) = 0;

        rotation(1, 0) = -1.0 * sine_value;
        rotation(1, 1) = COS15DEGREE;
        rotation(1, 2) = 0;

        rotation(2, 0) = 0;
        rotation(2, 1) = 0;
        rotation(2, 2) = 1;
    }
    else {
        std::cout << "NO AXIS SPECIFIED" << "\n";
    }


    for (vlt = vBegin; vlt != vEnd; ++vlt) {

        OpenMesh::Vec3f newCoord;
        newCoord = mesh.point(*vlt);

        MatrixXd coord_mat(3, 1);
        coord_mat << newCoord[0], newCoord[1], newCoord[2];
        coord_mat = rotation*coord_mat;

        newCoord[0] = coord_mat(0, 0);
        newCoord[1] = coord_mat(1, 0);
        newCoord[2] = coord_mat(2, 0);

        mesh.set_point(*vlt, newCoord);

    }
}

TriMesh::TriMesh(BaseMesh baseMesh) {
    this->mesh = baseMesh;
    this->intermediateMesh = BaseMesh(baseMesh);
    this->trulyUnchangedMesh = BaseMesh(baseMesh);
}

OpenMesh::Vec3f TriMesh::getCentreOfMesh() {
    int nVerts = 0;

    OpenMesh::Vec3f averageLocation = OpenMesh::Vec3f();
    averageLocation = averageLocation * 0.0f;


    BaseMesh::VertexIter vlt, vBegin, vEnd;
    vBegin = mesh.vertices_begin();
    vEnd = mesh.vertices_end();

    //std::cout << vBegin << "\t" << vEnd << "\n";

    for (vlt = vBegin; vlt != vEnd; ++vlt) {
        nVerts++;
        //do something here
        OpenMesh::Vec3f newCoord;
        newCoord = mesh.point(*vlt);
        averageLocation = averageLocation + newCoord;

    }
    averageLocation = averageLocation / nVerts;
    // std::cout << averageLocation << "\n";

    return averageLocation;
}

void TriMesh::display(DisplayMode displayMode, ColorMode colorMode, int whichEigToDraw) {
    if (displayMode == PointCloud) {
        BaseMesh::VertexIter vlt, vBegin, vEnd;
        vBegin = mesh.vertices_begin();
        vEnd = mesh.vertices_end();
        glPointSize(1.0f);

        glBegin(GL_POINTS);

        for (vlt = vBegin; vlt != vEnd; ++vlt) {

            OpenMesh::Vec3f thisCoord;
            OpenMesh::Vec4f thisCol;
            thisCoord = mesh.point(*vlt);

            thisCol[0] = 1.0f;
            thisCol[1] = 1.0f;
            thisCol[2] = 1.0f;
            thisCol[3] = 1.0f;

            glColor3f(thisCol[0], thisCol[1], thisCol[2]);
            glVertex3f(thisCoord[0], thisCoord[1], thisCoord[2]);

        }

        glEnd();

    }
    else {

        if (displayMode == WireFrame) {
            glColor3f(1.f, 1.f, 1.f);
            glLineWidth(1.5);
            //Draw wireframe
            for (BaseMesh::EdgeIter e_it = mesh.edges_begin(); e_it != mesh.edges_end(); ++e_it) {
                BaseMesh::Point to = mesh.point(mesh.to_vertex_handle(mesh.halfedge_handle(*e_it, 0)));
                BaseMesh::Point from = mesh.point(mesh.from_vertex_handle(mesh.halfedge_handle(*e_it, 0)));
                glBegin(GL_LINE_STRIP);
                OpenMesh::Vec4f thisCol;

                glVertex3f(to[0], to[1], to[2]);
                glVertex3f(from[0], from[1], from[2]);

                glEnd();

            }
        }

        //draw faces
        glBegin(GL_TRIANGLES);
        glColor3f(0.0f, 0.0f, 0.0f);
        auto begin = mesh.faces_begin();
        auto end = mesh.faces_end();
        for (BaseMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it) {
            //MyMesh::FaceVertexIter OpenMesh::PolyConnectivity::FaceVertexIter OpenMesh::PolyConnectivity::fv_iter(FaceHandle _fh); (FaceHandle _fh);
            OpenMesh::FaceHandle fa = *f_it;
            OpenMesh::Vec3f thisCol;
            thisCol = mesh.normal(fa);
            float col_scale_factor = 1.0 / curveProps.max_mean_curvature;
            if (colorMode == MeanCurvature) {


                for (BaseMesh::FaceVertexIter fvi = mesh.fv_iter(fa); fvi.is_valid(); ++fvi) {
                    OpenMesh::Vec3f thisCoord;
                    OpenMesh::Vec3f thisCol;
                    thisCoord = mesh.point(*fvi);

                    float thisH = mesh.property(mean_curvature, OpenMesh::VertexHandle(*fvi));
                    float thisK = mesh.property(gauss_curvature, OpenMesh::VertexHandle(*fvi));
                    //float thisK1 = mesh_list[primary_mesh].property(k1, OpenMesh::VertexHandle(fvi.handle()));
                    //float thisK2 = mesh_list[primary_mesh].property(k2, OpenMesh::VertexHandle(fvi.handle()));


                    //thisCol = simpleHColourMap(thisH);
                    thisCol = this->simpleColourMap(thisH, curveProps.max_mean_curvature,
                                                    curveProps.min_mean_curvature);
                    glColor3f(thisCol[0], thisCol[1], thisCol[2]);

                    //gauss curvature
                    //glColor3f(thisK, thisK, thisK);

                    glVertex3f(thisCoord[0], thisCoord[1], thisCoord[2]);

                    //std::cout << mesh_list[primary_mesh].property(mean_curvature, OpenMesh::VertexHandle(*fvi)) << "\n";

                }
            }
            else if (colorMode == GaussianCurvature) {

                for (BaseMesh::FaceVertexIter fvi = mesh.fv_iter(fa); fvi.is_valid(); ++fvi) {
                    OpenMesh::Vec3f thisCoord;
                    OpenMesh::Vec3f thisCol;
                    thisCoord = mesh.point(*fvi);

                    //OpenMesh::Vec4f thisCol;
                    //thisCol = mesh_list[primary_mesh].color(fvi.handle());

                    float thisK = mesh.property(gauss_curvature, OpenMesh::VertexHandle(*fvi));
                    float thisK1 = mesh.property(k1, OpenMesh::VertexHandle(*fvi));
                    float thisK2 = mesh.property(k2, OpenMesh::VertexHandle(*fvi));

                    thisCol = this->simpleColourMap(thisK, curveProps.max_gauss_curvature,
                                                    curveProps.min_gauss_curvature);

                    //thisCol = simpleHColourMap(thisH);
                    glColor3f(thisCol[0], thisCol[1], thisCol[2]);

                    //gauss curvature
                    //glColor3f(thisK, thisK, thisK);

                    glVertex3f(thisCoord[0], thisCoord[1], thisCoord[2]);

                    //std::cout << mesh_list[primary_mesh].property(mean_curvature, OpenMesh::VertexHandle(*fvi)) << "\n";

                }
            }
            else if (colorMode == PrincipalCurvature) {
                for (BaseMesh::FaceVertexIter fvi = mesh.fv_iter(fa); fvi.is_valid(); ++fvi) {
                    OpenMesh::Vec3f thisCoord;
                    OpenMesh::Vec3f thisCol;
                    thisCoord = mesh.point(*fvi);

                    //OpenMesh::Vec4f thisCol;
                    //thisCol = mesh_list[primary_mesh].color(*fvi);

                    float thisK1 = mesh.property(k1, OpenMesh::VertexHandle(*fvi));
                    float thisK2 = mesh.property(k2, OpenMesh::VertexHandle(*fvi));

                    //thisCol = simpleColourMap(thisK, max_gauss_curvature, min_gauss_curvature);
                    thisCol = this->principleKColourmap(thisK1, thisK2);

                    //thisCol = simpleHColourMap(thisH);
                    glColor3f(thisCol[0], thisCol[1], thisCol[2]);

                    //gauss curvature
                    //glColor3f(thisK, thisK, thisK);

                    glVertex3f(thisCoord[0], thisCoord[1], thisCoord[2]);
                }
            }
            else if (colorMode == SpectralWeight) {

                //bool showSpectralWeight = true;
                //int whichEigToDraw = 5;
                //Eigen::MatrixXd displacementValues;

                for (BaseMesh::FaceVertexIter fvi = mesh.fv_iter(fa); fvi.is_valid(); ++fvi) {
                    OpenMesh::Vec3f thisCoord;
                    OpenMesh::Vec3f thisCol;
                    thisCoord = mesh.point(*fvi);

                    //OpenMesh::Vec4f thisCol;
                    //thisCol = mesh_list[primary_mesh].color(fvi.handle());

                    thisCol[0] = this->displacementValues(fvi->idx(), whichEigToDraw);
                    thisCol[1] = this->displacementValues(fvi->idx(), whichEigToDraw);
                    thisCol[2] = this->displacementValues(fvi->idx(), whichEigToDraw);

                    //thisCol = simpleHColourMap(thisH);
                    glColor3f(thisCol[0], thisCol[1], thisCol[2]);

                    //gauss curvature
                    //glColor3f(thisK, thisK, thisK);

                    glVertex3f(thisCoord[0], thisCoord[1], thisCoord[2]);
                }
            }
            else {
                if (displayMode != Smooth) {
                    if (displayMode != WireFrame) {
                        glColor3f(thisCol[0], thisCol[1], thisCol[2]);
                    }
                }
                for (BaseMesh::FaceVertexIter fvi = mesh.fv_iter(fa); fvi.is_valid(); ++fvi) {
                    OpenMesh::Vec3f thisCoord;
                    thisCoord = mesh.point(*fvi);
                    if (displayMode == Smooth && displayMode != WireFrame) {
                        //OpenMesh::Vec4f thisCol;
                        //thisCol = mesh_list[primary_mesh].color(fvi.handle());
                        OpenMesh::Vec3f thisCol;
                        thisCol = mesh.normal(*fvi);
                        glColor3f(thisCol[0], thisCol[1], thisCol[2]);
                    }
                    glVertex3f(thisCoord[0], thisCoord[1], thisCoord[2]);

                }

            }
        }
        glEnd();

        //pop
        glPopMatrix();
    }
}

void TriMesh::translate(OpenMesh::Vec3f translation) {
    BaseMesh::VertexIter vlt, vBegin, vEnd;
    vBegin = mesh.vertices_begin();
    vEnd = mesh.vertices_end();

    int n_points = 0;

    for (vlt = vBegin; vlt != vEnd; ++vlt) {
        //do something here
        n_points++;
        mesh.point(*vlt) += translation;

    }
}

void TriMesh::mapOverVertices(std::function<void(OpenMesh::PolyConnectivity::VertexIter)> func) {
    //iterate over all vertices
    BaseMesh::VertexIter vlt, vBegin, vEnd;
    vBegin = mesh.vertices_begin();
    vEnd = mesh.vertices_end();
    for (vlt = vBegin; vlt != vEnd; ++vlt) {
        func(vlt);
    }
}

void TriMesh::mapOverVertexNeighbors(OpenMesh::PolyConnectivity::VertexIter vlt,
                                     std::function<void(OpenMesh::PolyConnectivity::ConstVertexVertexIter)> func) {
    for (BaseMesh::ConstVertexVertexIter vvi = mesh.vv_iter(*vlt); vvi.is_valid(); ++vvi) {
        func(vvi);
    }
}

int TriMesh::numVertices() {
    int N = 0;
    auto sum = [&](OpenMesh::PolyConnectivity::VertexIter vlt) {
        N += 1;
    };
    mapOverVertices(sum);
    return N;
}

int TriMesh::numVertexNeighbors(OpenMesh::PolyConnectivity::VertexIter vlt) {
    int N = 0;
    auto sum = [&](OpenMesh::PolyConnectivity::ConstVertexVertexIter vvi) {
        N += 1;
    };
    mapOverVertexNeighbors(vlt, sum);
    return N;
}

std::vector<OpenMesh::VertexHandle> TriMesh::getNeighbors(OpenMesh::PolyConnectivity::VertexIter vlt) {
    std::vector<OpenMesh::VertexHandle> neighbors;
    auto areaFunc = [&](OpenMesh::PolyConnectivity::ConstVertexVertexIter vvi) {
        neighbors.push_back(*vvi);
    };
    mapOverVertexNeighbors(vlt, areaFunc);
    return neighbors;
}

void TriMesh::reset() {
    mesh = BaseMesh(trulyUnchangedMesh);
    intermediateMesh = BaseMesh(trulyUnchangedMesh);
    evecs_coeffs.setOnes();
}
