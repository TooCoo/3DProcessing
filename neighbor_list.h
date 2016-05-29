
#include <vector>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>

#include "point_pair.h"

#ifndef NEIGHBOURLIST_H
#define NEIGHBOURLIST_H

class neighbor_list {

private:

	std::vector<point_pair> neighbour_list;

public:

	neighbor_list() {
		;
	}

	void addPair(point_pair _pp) {
		neighbour_list.push_back(_pp);
	}

	point_pair getPair(int _i) {
		return neighbour_list[_i];
	}

	int GetSize() {
		return neighbour_list.size();
	}

	float getAverageSeperation() {
		float av = 0.0f;

		for (int i = 0; i < neighbour_list.size(); i++) {
			av += neighbour_list[i].getD();
		}

		av *= (1.0f / (float)neighbour_list.size());

		return av;

	}

};



#endif
