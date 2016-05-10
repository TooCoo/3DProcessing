
#include <vector>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>

#include "PointPair.h"

#ifndef NEIGHBOURLIST_H
#define NEIGHBOURLIST_H

class NeighbourList {

private:

	std::vector<PointPair> neighbour_list;

public:

	NeighbourList() {
		;
	}

	void addPair(PointPair _pp) {
		neighbour_list.push_back(_pp);
	}

	PointPair getPair(int _i) {
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
