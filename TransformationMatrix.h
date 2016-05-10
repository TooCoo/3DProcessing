#include <eigen3/Eigen/Dense>
#include <math.h>
#include <iostream>

#ifndef TRANSFORMATIONMATRIX_H
#define TRANSFORMATIONMATRIX_H

using Eigen::MatrixXd;

class TranformationMatrix {

private:



public:

	TranformationMatrix() { ; }

	MatrixXd getRotationMatrix(double _rx, double _ry, double _rz) {

		double cos_x = cos(_rx);
		double cos_y = cos(_ry);
		double cos_z = cos(_rz);

		double sin_x = sin(_rx);
		double sin_y = sin(_ry);
		double sin_z = sin(_rz);

		MatrixXd RX(3, 3);
		MatrixXd RY(3, 3);
		MatrixXd RZ(3, 3);

		RX(0, 0) = 1.0;
		RX(0, 1) = 0.0;
		RX(0, 2) = 0.0;

		RX(1, 0) = 0.0;
		RX(1, 1) = cos_x;
		RX(1, 2) = -1.0 * sin_x;

		RX(2, 0) = 0.0;
		RX(2, 1) = sin_x;
		RX(2, 2) = cos_x;
		
		//---------------

		RY(0, 0) = cos_y;
		RY(0, 1) = 0.0;
		RY(0, 2) = sin_y;

		RY(1, 0) = 0.0;
		RY(1, 1) = 1.0;
		RY(1, 2) = 0.0;

		RY(2, 0) = -1.0 * sin_y;
		RY(2, 1) = 0.0;
		RY(2, 2) = cos_y;

		//---------------

		RZ(0, 0) = cos_z;
		RZ(0, 1) = -1.0 * sin_z;
		RZ(0, 2) = 0.0;

		RZ(1, 0) = sin_z;
		RZ(1, 1) = cos_z;
		RZ(1, 2) = 0.0;

		RZ(2, 0) = 0.0;
		RZ(2, 1) = 0.0;
		RZ(2, 2) = 1.0;

		//---------------
		
		//std::cout << "RX:\n" << RX << "\nRY:\n" << RY << "\nRZ:\n" << RZ << "\n";

		MatrixXd R(3, 3);
		R = RZ * RY * RX;
		
		R.transposeInPlace();

		//std::cout << "R:\n" << R << "\n";

		return R;
	}



};



#endif
