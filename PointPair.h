#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>

#ifndef POINTPAIR_H
#define POINTPAIR_H

class PointPair {

private:
	//two points and the distance between them
	OpenMesh::Vec3f p;
	OpenMesh::Vec3f q;
	int pi;
	int qi;
	float d;


public:

	PointPair() {
		p = OpenMesh::Vec3f()*0.0f;
		q = OpenMesh::Vec3f()*0.0f;
		d = 0.0f;
		pi = 0;
		qi = 0;
	}
	
	//constructor
	PointPair(OpenMesh::Vec3f _p, OpenMesh::Vec3f _q, float _d, int _pi, int _qi) {
		p = _p;
		q = _q;
		d = _d;
		pi = _pi;
		qi = _qi;
		
	}

	OpenMesh::Vec3f getP() {
		return p;
	}

	OpenMesh::Vec3f getQ() {
		return q;
	}

	float getD() {
		return d;
	}

	int getPi() {
		return pi;
	}

	int getQi() {
		return qi;
	}
};


#endif
