// Standard Library
#include <iostream>
#include <fstream>
#include <random>

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SparseCore>

// OpenMesh
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
//#include <OpenMesh/Core/Mesh/PolyConnectivity.hh>
//#include <OpenMesh/Core/Geometry/VectorT.hh>

// local imports
#include "viewer.h"

/* ALIGN NORMALS NOT USED CURRENTLY
void alignNormals() {

    int initial_k = k;
    k = 30;

    int normals_flipped = 0;

    //run over each mesh
    for (int mesh_i = 0; mesh_i < mesh_list.size(); mesh_i++) {

        current_mesh = mesh_i;

        MyMesh::VertexIter vlt, vBegin, vEnd;
        vBegin = mesh_list[current_mesh].vertices_begin();
        vEnd = mesh_list[current_mesh].vertices_end();


        //_______________________________ Set up ANN kdtree, etc...
        int maxPts = mesh_list[current_mesh].n_vertices();		// maximum number of data points
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
        for (vlt = vBegin; vlt != vEnd; ++vlt) {

            for (int i = 0; i < dim; i++) {

                dataPts[nPts][i] = mesh_list[current_mesh].point(*vlt)[i];

            }
            nPts++;
        }

        //build the kdtree
        kdTree = new ANNkd_tree(					// build search structure
            dataPts,					// the data points
            nPts,						// number of points
            dim);						// dimension of space



                                        //each point //use ANN to find nearest neighbour
        for (vlt = vBegin; vlt != vEnd; ++vlt) {
            //set query point
            for (int i = 0; i < dim; i++) {
                queryPt[i] = mesh_list[current_mesh].point(*vlt)[i];

            }

            //find 4 nearest neighbours
            kdTree->annkSearch(						// search
                queryPt,						// query point
                k,								// number of near neighbors
                nnIdx,							// nearest neighbors (returned)
                dists,							// distance (returned)
                eps);							// error bound


            OpenMesh::Vec3f average_normal = OpenMesh::Vec3f()*0.0f;

            for (int k_i = 0; k_i < k; k_i++) {
                OpenMesh::VertexHandle thisHandle = OpenMesh::VertexHandle(nnIdx[k_i]);
                for (int dim_i = 0; dim_i < dim; dim_i++) {
                    //sum normals:
                    average_normal[dim_i] += mesh_list[current_mesh].normal(thisHandle)[dim_i];
                }
            }
            average_normal *= (1.0f / (float)k);

            //if this normal is more different than 1.0 of the average normal then change it
            double difference_beteen_normals = (mesh_list[current_mesh].normal(*vlt) - average_normal).norm();
            //std::cout << difference_beteen_normals << "\n";
            if (difference_beteen_normals > 0.8) {
                OpenMesh::Vec3f new_normal = mesh_list[current_mesh].normal(*vlt);
                new_normal *= -1.0;
                mesh_list[current_mesh].set_normal(*vlt, new_normal);
                normals_flipped++;

            }


        }



        delete[] nnIdx;							// clean things up
        delete[] dists;
        delete kdTree;
        delete queryPt;					// allocate query point
        delete dataPts;
        annClose();									// done with ANN


    }

    std::cout << "Normals flipped: " << normals_flipped << "\n";

    k = initial_k;

}
*/

/* NOT USED CURRENTLY
void findNormals() {

    current_mesh = 0;
    //iterate over all vertices
    MyMesh::VertexIter vlt, vBegin, vEnd;
    vBegin = mesh_list[current_mesh].vertices_begin();
    vEnd = mesh_list[current_mesh].vertices_end();

    for (vlt = vBegin; vlt != vEnd; ++vlt) {

        OpenMesh::VertexHandle vh = *vlt;

        OpenMesh::Vec3f xi = mesh_list[current_mesh].point(vh);

        std::vector<OpenMesh::Vec3f> xj;

        OpenMesh::Vec3f unnormalisedS = OpenMesh::Vec3f(0.0f, 0.0f, 0.0f);
        float area = 0.0f;

        //now iterate over adjacent vertices
        for (MyMesh::ConstVertexVertexIter vvi = mesh_list[current_mesh].vv_iter(vh); vvi.is_valid(); ++vvi) {
            xj.push_back(mesh_list[current_mesh].point(*vvi));
        }
        //now find laplaceS
        for (int i = 0; i < xj.size(); i++) {

            //I need angles which look at the edge xi -> xj
            OpenMesh::Vec3f xjm1;
            OpenMesh::Vec3f xjp1;
            if (i == 0) {
                xjm1 = xj[xj.size() - 1];
            }
            else {
                xjm1 = xj[i - 1];
            }

            if (i == xj.size() - 1) {
                xjp1 = xj[0];
            }
            else {
                xjp1 = xj[i + 1];
            }
            //alpha angle is xj xj-1 xi
            OpenMesh::Vec3f vecAB = xi - xjm1;
            OpenMesh::Vec3f vecAC = xj[i] - xjm1;
            float alpha = acos(OpenMesh::dot(vecAB, vecAC) / (vecAB.norm() * vecAC.norm()));
            //beta angle is xj xj+1 xi
            vecAB = xi - xjp1;
            vecAC = xj[i] - xjp1;
            float beta = acos(OpenMesh::dot(vecAB, vecAC) / (vecAB.norm() * vecAC.norm()));
            //cot is cos/sin
            //w = cot alpha + cot beta
            float weight = (cos(alpha) / sin(alpha)) + (cos(beta) / sin(beta));
            area += vecAB.norm()*vecAC.norm() * sin(beta) / 6.0f;
            unnormalisedS += weight * (xj[i] - xi);

        }

        //set normal to be unnormalisedS

        mesh_list[current_mesh].set_normal(vh, unnormalisedS.normalized());


    }



}
 */

/* NOT USED

void scale(OpenMesh::VertexHandle &_vh, float _alpha) {
    OpenMesh::Vec3f newCoord;
    newCoord = mesh_list[current_mesh].point(_vh);
    mesh_list[current_mesh].set_point(_vh, newCoord * _alpha);
}

 */


Viewer* myViewer;
CurvatureProperties curveProps;

int main(void)
{


    int nEVecsToUse = 10;
    myViewer = new Viewer(nEVecsToUse);

	//_______READ MESH________________________	

	std::string filename_03 = "dragon_vrip_res4.ply";
	std::string filename_04 = "bun_zipper.ply";
	std::string filename_05 = "bun_zipper_res4.ply";
	std::string filename_06 = "smooth_test.obj";
	std::string filename_07 = "dragon_lowres.ply";	// Dragon model - worked well with the smoothing
	std::string filename_wolf = "Wolf2.obj";
	std::string filename_deer = "Deer.obj";
	std::string filename_cat = "cat.obj";
	std::string filename_goat = "Goat.obj";
	std::string filename_shark = "Shark.obj";
	std::string filename_cube = "cube.obj";
	std::string filename_ox = "ox.obj"; //low poly, blocky model I made in blender
	std::string filename_ox1 = "ox_sd1.obj"; //subsurface division 1
	std::string filename_ox2= "ox_sd2.obj"; //subsurface division 2

	std::vector<std::string> input_files;

	input_files.push_back(filename_ox1);

	std::cout << "loading meshes...";

	BaseMesh uBaseMesh;
	if (!OpenMesh::IO::read_mesh(uBaseMesh, input_files[0]))
	{
		std::cerr << "read error\n";
		exit(1);
	}
    auto unchangedMesh = TriMesh(uBaseMesh);
    myViewer->setUnchangedMesh(&unchangedMesh);

    for (int i = 0; i < input_files.size(); i++) {

        BaseMesh baseMesh;
        if (!OpenMesh::IO::read_mesh(baseMesh, input_files[i]))
        {
            std::cerr << "read error\n";
            exit(1);
        }
        auto mesh = TriMesh(baseMesh);
        myViewer->addMesh(mesh);
        for (int i = 0; i < myViewer->mesh_list.size(); i++) {
            auto a = myViewer->mesh_list[i].n_faces();
            auto b = myViewer->mesh_list[i].n_verts();
            auto c = 5;
        }
    }
    //_______READ MESH________________________
    for (int i = 0; i < myViewer->mesh_list.size(); i++) {
        auto a = myViewer->mesh_list[i].n_faces();
        auto b = myViewer->mesh_list[i].n_verts();
        auto c = 5;
    }

	std::cout << " meshes loaded\n\n";


	std::cout << "|-----------------------------|\n";
	std::cout << "|                             |\n";
	std::cout << "|        Seb's Meshviewer     |\n";
	std::cout << "|                             |\n";
	std::cout << "|-----------------------------|\n";

	std::cout << "eigenReconstruction: press k\n";


	// The help function - remind user it exists
	std::cout << "Press H for help.\n";
    for (int i = 0; i < myViewer->mesh_list.size(); i++) {
        auto a = myViewer->mesh_list[i].n_faces();
        auto b = myViewer->mesh_list[i].n_verts();
        auto c = 5;
    }

	//set a seed for the random number gen - this was primarily so the meshes would be the same colour each time for ICP
	srand(1);

    for (int i = 0; i < myViewer->mesh_list.size(); i++) {
        auto a = myViewer->mesh_list[i].n_faces();
        auto b = myViewer->mesh_list[i].n_verts();
        auto c = 5;
    }

    // display until quitting time

    myViewer->display();

}




