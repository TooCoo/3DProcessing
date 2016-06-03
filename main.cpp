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

// local imports
#include "viewer.h"


// required global variables
Viewer *myViewer;
CurvatureProperties curveProps;
OpenMesh::VPropHandleT<double>       mean_curvature;
OpenMesh::VPropHandleT<double>       gauss_curvature;
OpenMesh::VPropHandleT<double>       k1;
OpenMesh::VPropHandleT<double>       k2;

inline void print_usage( int argc, char** argv )
{
	std::cout << "Usage: " << argv[0] << " [first.obj] [n_eigs]" << std::endl;
}

int main(int argc, char* argv[])
{
	if ( argc != 3 ) {
		print_usage(argc, argv);
		exit(EXIT_FAILURE);
	}

	std::vector<std::string> input_files;

	input_files.push_back(argv[1]);
	int nEVecsToUse = std::stoi(argv[2]);

	myViewer = new Viewer(nEVecsToUse);

	std::cout << "loading meshes...";

    // load all meshes for viewing
    for (int i = 0; i < input_files.size(); i++) {

        BaseMesh baseMesh;

		// read both in
        if (!OpenMesh::IO::read_mesh(baseMesh, input_files[i]))
        {
            std::cerr << "read error\n";
            exit(1);
        }

		baseMesh.add_property(mean_curvature, "mean_curvature");
		baseMesh.property(mean_curvature).set_persistent(true);

		baseMesh.add_property(gauss_curvature, "gauss_curvature");
		baseMesh.property(gauss_curvature).set_persistent(true);

		baseMesh.add_property(k1, "k1");
		baseMesh.property(k1).set_persistent(true);

		baseMesh.add_property(k2, "k2");
		baseMesh.property(k2).set_persistent(true);

        auto mesh = TriMesh(baseMesh);
        myViewer->addMesh(mesh);
    }

	std::cout << " meshes loaded\n\n";
	std::cout << "|-----------------------------|\n";
	std::cout << "|                             |\n";
	std::cout << "|        Seb's Meshviewer     |\n";
	std::cout << "|       (modified by Ben)     |\n";
	std::cout << "|-----------------------------|\n";

	// The help function - remind user it exists
	std::cout << "Press H for help.\n";

	//set a seed for the random number gen - this was primarily so the meshes would be the same colour each time for ICP
	srand(1);

    // display until quitting time
    myViewer->display();
}