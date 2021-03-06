
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <random>

#include <GLFW/glfw3.h>
#include <vector>

#include <GL\glut.h>

#include <eigen3\Eigen/Core>
#include <eigen3\Eigen/SparseCore>
#include <GenEigsSolver.h>
#include <MatOp/SparseGenMatProd.h>
#include <iostream>

#define PI 3.14159265
//cos 3 degrees
#define COS3DEGREE 0.99862953475
#define SIN3DEGREE 0.05233595624

//15 degrees
#define COS15DEGREE 0.96592582628
#define SIN15DEGREE 0.2588190451

//my data structure/classes
#include "TransformationMatrix.h"

// Import ANN so that I can find nearest neighbours
#include <ANN/ANN.h>					

// and eigen ti do some linear algebra
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/IterativeLinearSolvers>

using Eigen::MatrixXd;
//typedef Eigen::SparseMatrix<double> SpMat;

//lighting     --------
Eigen::Vector3d material(0.9f, 0.9f, 0.9f);

Eigen::Vector3d lightDir(0.5f, 0.5f, 0.0f);
Eigen::Vector3d ambientLight(0.3f, 0.3f, 0.3f);
Eigen::Vector3d diffuse(0.5f, 0.5f, 0.5f);
Eigen::Vector3d specular(0.6f, 0.6f, 0.6f);
float alpha_shininess = 0.5f;

Eigen::Vector3d  virtual_camera_location(0.0f, -1.0f, 0.0f);

//window dimensions
int window_w = 640;
int window_h = 480;
int window_w_initial = 640;
int window_h_initial = 480;

//end lighting --------

//OpenMesh
// -------------------- OpenMesh
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/PolyConnectivity.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>

#define _USE_MATH_DEFINES
#include <math.h>
// ----------------------------------------------------------------------------

float globalScale = 1.0f;
float global_rotation = 0.0f;
OpenMesh::Vec3f global_translation = OpenMesh::Vec3f();

int k = 5;				// number of nearest neighbors // I'm using 5 as one of the nearest points is the point
int dim = 3;			// dimension
double eps = 0;			// error bound
int maxPts = 1000;		// maximum number of data points //this is changed lower down...

						// ----------------------------------------------------------------------------
						//Edit mode

bool editmode = false;
bool rotation_mode = false;
bool translation_mode = false;
int current_axis = 0; //x = 0, y = 1, z = 2
float move_amount = 0.0001;

//Rendering:
bool wireframe = false;
bool pointcloud = false;
bool smoothFaces = true;
bool showMeanCurvature = false;
bool showGaussCurvature = false;
bool showPrincipleCurvature = false;
bool showSpectralWeight = false;
int whichEigToDraw = 1;
TranformationMatrix tm = TranformationMatrix();
MatrixXd globalRotationMatrix = tm.getRotationMatrix(0.0, 0.0, 0.0);

// ----------------------------------------------------------------------------

struct MyTraits : public OpenMesh::DefaultTraits {
	typedef OpenMesh::Vec3f Point;
	typedef OpenMesh::Vec3f Normal;
	typedef OpenMesh::Vec4f Color;
	//typedef float mean_curvature;
	VertexTraits{
public:

	const unsigned int valance() const { return valence; }
	void set_valence(const unsigned int v) { valance = v; }
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

typedef OpenMesh::PolyMesh_ArrayKernelT<MyTraits>  MyMesh;
OpenMesh::VPropHandleT<double>       mean_curvature;
OpenMesh::VPropHandleT<double>       gauss_curvature;
OpenMesh::VPropHandleT<double>       k1;
OpenMesh::VPropHandleT<double>       k2;
double max_mean_curvture = -10000.0;
double max_gauss_curvture = -10000.0;
double min_gauss_curvture = 100000000;
double max_k1_curvture = -10000.0;
double min_k1_curvture = 10000.0;
double max_k2_curvture = -10000.0;
double min_k2_curvture = 10000.0;
double min_mean_curvture = 10000.0;
//MyMesh mesh;
int current_mesh = 0;
std::vector<MyMesh> mesh_list;

MyMesh unchangedMesh;
MyMesh noisyMesh;
// ----------------------------------------------------------------------------

// -------------------Matrix holding the eigen vectors-------------------------

Eigen::MatrixXd evecs;
Eigen::VectorXcd evals;
Eigen::MatrixXd evecs_coeffs;
double min_coeff = 0.0;
double max_coeff = 0.0;
Eigen::MatrixXd evecs_coeffs_temp;
double eig_inc = 0.1;
Eigen::MatrixXd displacementValues;
int nEVecsToUse = 25;
int nEVecsToShow = 25;
int currentlyHeldEvects = 0;

enum Laplacian_type {uniform, beltrami, area};

// ----------------------------------------------------------------------------

// ----------------------- Mouse functionality---------------------------------
// Based on code found at: https://nccastaff.bournemouth.ac.uk/jmacey/RobTheBloke/www/opengl_programming.html
// Modifed - some copied - I assume that's fine since I'm not being assessed on my GL Buttons and Mouse clicks
struct Mouse
{
	int x;		/*	the x coordinate of the mouse cursor	*/
	int y;		/*	the y coordinate of the mouse cursor	*/
	int lmb;	/*	is the left button pressed?		*/
	int mmb;	/*	is the middle button pressed?	*/
	int rmb;	/*	is the right button pressed?	*/

	int dx;
	int dy;

	int xpress; /*	stores the x-coord of when the first button press occurred	*/
	int ypress; /*	stores the y-coord of when the first button press occurred	*/
};

typedef struct Mouse Mouse;

Mouse TheMouse = { 0,0,0,0,0 };

// ----------------------------------------------------------------------------

// ---------------------- Button interaction System -------------------------
// Based on code found at: https://nccastaff.bournemouth.ac.uk/jmacey/RobTheBloke/www/opengl_programming.html
// Modified for my use
typedef void(*ButtonCallback)();

//struct for button
struct Button
{
	int x_top_left;
	int y_top_left;
	int w;				// width
	int h;				// height
	int state;			// if 1: then pressed, else 0: not pressed
	int highlighted;		// is mouse over the button
	char* label;		// button text
	std::string slabel;
	//const unsigned char* culabel;		// button text
	ButtonCallback callbackFunction;

};

typedef struct Button Button;

struct TextBox
{
	int x_top_left;
	int y_top_left;
	int w;
	int h;
	char* label;
};

typedef struct TextBox TextBox;


std::vector<Button> buttonList;
std::vector<TextBox> textBoxList;

//function declarations for the buttons
void findEigenVectors(int nEVecsToUse, Laplacian_type lap_type);
void remakeFromEVecs(int nEVecsToUse);
void remakeFromModifiedEVecs(int nEVecsToUse);
void DoEigenDecomposition();
void findFaceNormals();
void findVertNormalsFromFaces();

//	Button functions:

void TogglePointCloud() {
	pointcloud = !pointcloud;
	std::cout << "Render mode: pointcloud\n";
}

void ToggleShading() {
	smoothFaces = !smoothFaces;
	std::cout << "Render mode: toggled flat/smooth shading\n";
}

void ToggleShowPrincipleCurvature() {
	std::cout << "Toggled show principle curvature\n";
	std::cout << "\t colour codes:\t Red:\t elliptic surface\n";
	std::cout << "\t \t Green:\t parabolic\n";
	std::cout << "\t \t Blue:\t hyperbolic\n";
	showPrincipleCurvature = !showPrincipleCurvature;
	showMeanCurvature = false;
	showGaussCurvature = false;
	showSpectralWeight = false;
}

void ToggleShowMeanCurvature() {
	std::cout << "Toggled show mean curvature\n";
	std::cout << "\t colour codes: Red: low values, Blue: High value\n";
	showMeanCurvature = !showMeanCurvature;
	showGaussCurvature = false;
	showPrincipleCurvature = false;
}

void ToggleShowGaussCurvature() {
	std::cout << "Toggled show Gaussian curvature\n";
	std::cout << "\t colour codes: Red: low values, Blue: High value\n";
	showGaussCurvature = !showGaussCurvature;
	showMeanCurvature = false;
	showPrincipleCurvature = false;
	showSpectralWeight = false;
}

void ToggleShowSpectralWeighting() {
	
	if (currentlyHeldEvects != nEVecsToUse) {
		std::cout << "Mismatch in number of eigen vectors: recalculating.\n";
		DoEigenDecomposition();
	}
	else {
		showSpectralWeight = !showSpectralWeight;

		showMeanCurvature = false;
		showPrincipleCurvature = false;
		showGaussCurvature = false;
	}	
}

void ToggleWireframe() {
	std::cout << "Render mode: toggled wireframe\n";
	wireframe = !wireframe;
}

void IncreaseNEigenVectors() {
	nEVecsToUse += 5;
	std::cout << "N Eigen Vecs: " << nEVecsToUse << "\n";
}

void DecreaseNEigenVectors() {
	nEVecsToUse -= 5;
	if (nEVecsToUse <= 5) nEVecsToUse = 5;
	std::cout << "N Eigen Vecs: " << nEVecsToUse << "\n";
}

void DoEigenDecomposition() {
	std::cout << "Computing Eigen Decomposition, finding " << nEVecsToUse << " largest eigenvectors... ";
	nEVecsToShow = nEVecsToUse;
	findEigenVectors(nEVecsToUse, uniform);
	remakeFromEVecs(nEVecsToShow);
	std::cout << "Done.\n";
}

void IncreaseNShownEigenVectors() {
	nEVecsToShow += 1;
	if (nEVecsToShow > nEVecsToUse) nEVecsToShow = nEVecsToUse;

	std::cout << "N Eigen Vecs: " << nEVecsToShow << "\n";
}

void DecreaseNShownEigenVectors() {
	nEVecsToShow -= 1;
	if (nEVecsToShow <= 1) nEVecsToShow = 1;
	std::cout << "N Eigen Vecs: " << nEVecsToShow << "\n";
}

void DoEigenReconstruciton() {
	std::cout << "Computing Eigen reconstruction, using " << nEVecsToShow << " largest eigenvectors... ";	
	remakeFromModifiedEVecs(nEVecsToShow);
	std::cout << "Done.\n";
}

void IncreaseSelectedEigenVector() {

	if ((currentlyHeldEvects != nEVecsToUse) && (showSpectralWeight)){
		std::cout << "Mismatch in number of eigen vectors: recalculating.\n";
		DoEigenDecomposition();
	}
	else if (showSpectralWeight) {
		
		whichEigToDraw++;
		if (whichEigToDraw == nEVecsToUse) whichEigToDraw = 0;

		std::cout << "Eig selected: " << whichEigToDraw << "\n";

	}
	
}

void DecreaseSelectedEigenVector() {

	if ((currentlyHeldEvects != nEVecsToUse) && (showSpectralWeight)) {
		std::cout << "Mismatch in number of eigen vectors: recalculating.\n";
		DoEigenDecomposition();
	}
	else if (showSpectralWeight) {
		whichEigToDraw--;
		if (whichEigToDraw < 0) whichEigToDraw = nEVecsToUse - 1;

		std::cout << "Eig selected: " << whichEigToDraw << "\n";
	}

}

void IncreaseSpectralCoeff() {

	if ((currentlyHeldEvects != nEVecsToUse) && (showSpectralWeight)) {
		std::cout << "Mismatch in number of eigen vectors: recalculating.\n";
		DoEigenDecomposition();
	}
	else if(showSpectralWeight){
		//evecs_coeffs(whichEigToDraw) += eig_inc;
		evecs_coeffs_temp(whichEigToDraw, 0) += eig_inc*evecs_coeffs_temp(whichEigToDraw, 0);
		evecs_coeffs_temp(whichEigToDraw, 1) += eig_inc*evecs_coeffs_temp(whichEigToDraw, 1);
		evecs_coeffs_temp(whichEigToDraw, 2) += eig_inc*evecs_coeffs_temp(whichEigToDraw, 2);
		//remakeFromEVecs(nEVecsToUse);
		remakeFromModifiedEVecs(nEVecsToShow);
	}
	
}

void DecreaseSpectralCoeff() {

	if ((currentlyHeldEvects != nEVecsToUse) && (showSpectralWeight)) {
		std::cout << "Mismatch in number of eigen vectors: recalculating.\n";
		DoEigenDecomposition();
	}
	else if(showSpectralWeight){
		//evecs_coeffs(whichEigToDraw) += eig_inc;
		evecs_coeffs_temp(whichEigToDraw, 0) -= eig_inc*evecs_coeffs_temp(whichEigToDraw, 0);
		evecs_coeffs_temp(whichEigToDraw, 1) -= eig_inc*evecs_coeffs_temp(whichEigToDraw, 1);
		evecs_coeffs_temp(whichEigToDraw, 2) -= eig_inc*evecs_coeffs_temp(whichEigToDraw, 2);
		//remakeFromEVecs(nEVecsToUse);
		remakeFromModifiedEVecs(nEVecsToShow);
	}
	
}

//copies original mesh onto current mesh
void Reset() {
	//copy from initial mesh to new mesh
	//use mesh 0
	current_mesh = 0;
	//iterate over all vertices
	MyMesh::VertexIter vlt, vBegin, vEnd;
	vBegin = unchangedMesh.vertices_begin();
	vEnd = unchangedMesh.vertices_end();
	
	int N = 0;
	for (vlt = vBegin; vlt != vEnd; ++vlt) {
		N++;
	}

	// make vectors (as a matrix) for old coords	
	Eigen::MatrixXd XnMat = Eigen::MatrixXd::Zero(N, 1);
	Eigen::MatrixXd YnMat = Eigen::MatrixXd::Zero(N, 1);
	Eigen::MatrixXd ZnMat = Eigen::MatrixXd::Zero(N, 1);

	//fill vector
	for (vlt = vBegin; vlt != vEnd; ++vlt) {
		OpenMesh::VertexHandle vh = vlt.handle();

		int i = vh.idx();
		int n_neighbours = 0;

		XnMat(i, 0) = unchangedMesh.point(vh)[0];
		YnMat(i, 0) = unchangedMesh.point(vh)[1];
		ZnMat(i, 0) = unchangedMesh.point(vh)[2];

	}

	//save new coords - update

	for (vlt = vBegin; vlt != vEnd; ++vlt) {

		OpenMesh::Vec3f newCoord;

		newCoord[0] = XnMat(vlt.handle().idx(), 0);
		newCoord[1] = YnMat(vlt.handle().idx(), 0);
		newCoord[2] = ZnMat(vlt.handle().idx(), 0);

		mesh_list[current_mesh].set_point(vlt.handle(), newCoord);

	}

	// find new normals for colours
	findFaceNormals();
	findVertNormalsFromFaces();
	//set all coeffs to one - this is done when calculating anyway
	//evecs_coeffs = MatrixXd::Ones(nEVecsToUse, 1);
	currentlyHeldEvects = 0;
	
}

//function used for testing denosing
void CopyNoisyMesh() {
	//copy from initial mesh to new mesh
	//use mesh 0
	current_mesh = 0;
	//iterate over all vertices
	MyMesh::VertexIter vlt, vBegin, vEnd;
	vBegin = noisyMesh.vertices_begin();
	vEnd = noisyMesh.vertices_end();

	int N = 0;
	for (vlt = vBegin; vlt != vEnd; ++vlt) {
		N++;
	}

	// make vectors (as a matrix) for old coords	
	Eigen::MatrixXd XnMat = Eigen::MatrixXd::Zero(N, 1);
	Eigen::MatrixXd YnMat = Eigen::MatrixXd::Zero(N, 1);
	Eigen::MatrixXd ZnMat = Eigen::MatrixXd::Zero(N, 1);

	//fill vector
	for (vlt = vBegin; vlt != vEnd; ++vlt) {
		OpenMesh::VertexHandle vh = vlt.handle();

		int i = vh.idx();
		int n_neighbours = 0;

		XnMat(i, 0) = noisyMesh.point(vh)[0];
		YnMat(i, 0) = noisyMesh.point(vh)[1];
		ZnMat(i, 0) = noisyMesh.point(vh)[2];

	}

	//save new coords - update

	for (vlt = vBegin; vlt != vEnd; ++vlt) {

		OpenMesh::Vec3f newCoord;

		newCoord[0] = XnMat(vlt.handle().idx(), 0);
		newCoord[1] = YnMat(vlt.handle().idx(), 0);
		newCoord[2] = ZnMat(vlt.handle().idx(), 0);

		mesh_list[current_mesh].set_point(vlt.handle(), newCoord);

	}

	// find new normals for colours
	findFaceNormals();
	findVertNormalsFromFaces();
	//set all coeffs to one - this is done when calculating anyway
	//evecs_coeffs = MatrixXd::Ones(nEVecsToUse, 1);
	currentlyHeldEvects = 0;

}

//returns 1 or zero if mouse is above button
int ButtonClickTest(Button* b, int x, int y)
{

	if (b)
	{
		/*
		*	If clicked within button area, then return true
		*/				
		if (x > b->x_top_left      &&
			x < b->x_top_left + b->w &&
			y > b->y_top_left      &&
			y < b->y_top_left + b->h) {
			return 1;

		}
	}

	/*
	*	otherwise false.
	*/
	return 0;
}

//button released
void ButtonRelease(Button *b, int x, int y)
{
	if (b)
	{
		/*
		*	If the mouse button was pressed within the button area
		*	as well as being released on the button.....
		*/
		if (ButtonClickTest(b, TheMouse.xpress, TheMouse.ypress) &&
			ButtonClickTest(b, x, y))
		{
			/*
			*	Then if a callback function has been set, call it.
			*/
			if (b->callbackFunction) {
				b->callbackFunction();
			}
		}

		/*
		*	Set state back to zero.
		*/
		b->state = 0;
	}
}

//button pressed
void ButtonPress(Button *b, int x, int y)
{
	if (b)
	{
		/*
		*	if the mouse click was within the buttons client area,
		*	set the state to true.
		*/
		if (ButtonClickTest(b, x, y))
		{
			b->state = 1;
		}
	}
}

bool ButtonPassive(Button *b, int x, int y)
{
	bool output = false;

	if (b)
	{
		/*
		*	if the mouse moved over the control
		*/
		if (ButtonClickTest(b, x, y))
		{
			/*
			*	If the cursor has just arrived over the control, set the highlighted flag
			*	and force a redraw. The screen will not be redrawn again until the mouse
			*	is no longer over this control
			*/
			if (b->highlighted == 0) {
				b->highlighted = 1;
				//glutPostRedisplay();

			}

			output = true;

		}
		else

			/*
			*	If the cursor is no longer over the control, then if the control
			*	is highlighted (ie, the mouse has JUST moved off the control) then
			*	we set the highlighting back to false, and force a redraw.
			*/
			if (b->highlighted == 1)
			{
				b->highlighted = 0;
				//glutPostRedisplay();
			}
	}

	return output;

}

//	End of Button Functions

// ----------------------------------------------------------------------------

//colours the vertex vh with colour col
void colourVertex(OpenMesh::VertexHandle &_vh, OpenMesh::Vec4f col) {
	OpenMesh::Vec3f newCoord;
	newCoord = mesh_list[current_mesh].point(_vh);
	mesh_list[current_mesh].set_color(_vh, col);
}

//saves mesh
void saveMesh() {

	current_mesh = 0;

	try
	{
		if (!OpenMesh::IO::write_mesh(mesh_list[current_mesh], "remade_mesh.ply"))
		{
			std::cerr << "Cannot write mesh to file 'output.off'" << std::endl;
		}
	}
	catch (std::exception& x)
	{
		std::cerr << x.what() << std::endl;
	}

}

//saves the noisy mesh - used for denoising test
void saveNoisyMesh() {

	try
	{
		if (!OpenMesh::IO::write_mesh(noisyMesh, "noisy_mesh.ply"))
		{
			std::cerr << "Cannot write mesh to file 'output.off'" << std::endl;
		}
	}
	catch (std::exception& x)
	{
		std::cerr << x.what() << std::endl;
	}

}

void moveMeshesToOrigin() {

	std::vector<OpenMesh::Vec3f> mesh_centre;
	//find average point
	for (int i = 0; i < mesh_list.size(); i++) {

		mesh_centre.push_back(OpenMesh::Vec3f()*0.0f);

		current_mesh = i;

		MyMesh::VertexIter vlt, vBegin, vEnd;
		vBegin = mesh_list[current_mesh].vertices_begin();
		vEnd = mesh_list[current_mesh].vertices_end();

		int n_points = 0;

		for (vlt = vBegin; vlt != vEnd; ++vlt) {
			//do something here	
			n_points++;
			mesh_centre[current_mesh] += mesh_list[current_mesh].point(vlt.handle());

		}
		mesh_centre[current_mesh] *= (1.0f / (float)n_points);
	}

	//move points to mesh 1 location
	for (int i = 0; i < mesh_list.size(); i++) {

		current_mesh = i;

		OpenMesh::Vec3f translation = -1.0 * mesh_centre[0];

		MyMesh::VertexIter vlt, vBegin, vEnd;
		vBegin = mesh_list[current_mesh].vertices_begin();
		vEnd = mesh_list[current_mesh].vertices_end();

		int n_points = 0;

		for (vlt = vBegin; vlt != vEnd; ++vlt) {
			//do something here	
			n_points++;
			mesh_list[current_mesh].point(vlt.handle()) += translation;

		}
	}


}

void applyTranslation(MatrixXd _R, MatrixXd _T, int target_mesh) {

	current_mesh = target_mesh;

	MatrixXd T(1, 3);

	T(0, 0) = _T(0, 0);
	T(0, 1) = _T(1, 0);
	T(0, 2) = _T(2, 0);

	MatrixXd RTrans(3, 3);

	RTrans(0, 0) = _R(0, 0);
	RTrans(0, 1) = _R(1, 0);
	RTrans(0, 2) = _R(2, 0);

	RTrans(1, 0) = _R(0, 1);
	RTrans(1, 1) = _R(1, 1);
	RTrans(1, 2) = _R(2, 1);

	RTrans(2, 0) = _R(0, 2);
	RTrans(2, 1) = _R(1, 2);
	RTrans(2, 2) = _R(2, 2);

	MyMesh::VertexIter vlt, vBegin, vEnd;
	vBegin = mesh_list[current_mesh].vertices_begin();
	vEnd = mesh_list[current_mesh].vertices_end();

	for (vlt = vBegin; vlt != vEnd; ++vlt) {

		OpenMesh::Vec3f newCoord;
		newCoord = mesh_list[current_mesh].point(vlt.handle());

		MatrixXd new_point_matrix(1, 3);
		new_point_matrix(0, 0) = newCoord[0];
		new_point_matrix(0, 1) = newCoord[1];
		new_point_matrix(0, 2) = newCoord[2];
		//new_point_matrix(0, 3) = 1.0f;
		//std::cout << "_R\n" << _R << "\n";
		//std::cout << "T\n" << T << "\n";
		//std::cout << "translation:\n" << translation << "\n";
		//std::cout << "old coord:\n" << new_point_matrix << "\n";
		//system("PAUSE");

		//new_point_matrix = new_point_matrix * _R + T;
		new_point_matrix = new_point_matrix * RTrans + T;


		newCoord[0] = new_point_matrix(0, 0);
		newCoord[1] = new_point_matrix(0, 1);
		newCoord[2] = new_point_matrix(0, 2);

		mesh_list[current_mesh].set_point(vlt.handle(), newCoord);

	}



}

void applyTranslation(MatrixXd _x, int target_mesh) {

	current_mesh = target_mesh;

	//std::cout << "x:\n" << _x << "\n";
	//system("PAUSE");

	MatrixXd T(1, 3);

	T(0, 0) = _x(3, 0);
	T(0, 1) = _x(4, 0);
	T(0, 2) = _x(5, 0);

	TranformationMatrix tm = TranformationMatrix();

	MatrixXd R(3, 3);
	R = tm.getRotationMatrix(_x(0, 0), _x(1, 0), _x(2, 0));

	MyMesh::VertexIter vlt, vBegin, vEnd;
	vBegin = mesh_list[current_mesh].vertices_begin();
	vEnd = mesh_list[current_mesh].vertices_end();

	for (vlt = vBegin; vlt != vEnd; ++vlt) {

		OpenMesh::Vec3f newCoord;
		newCoord = mesh_list[current_mesh].point(vlt.handle());

		MatrixXd new_point_matrix(1, 3);
		new_point_matrix(0, 0) = newCoord[0];
		new_point_matrix(0, 1) = newCoord[1];
		new_point_matrix(0, 2) = newCoord[2];

		//std::cout << "R\n" << R << "\n";
		//std::cout << "T\n" << T << "\n";		
		//std::cout << "old coord:\n" << new_point_matrix << "\n";


		//new_point_matrix = new_point_matrix * _R + T;
		new_point_matrix = new_point_matrix * R + T;

		newCoord[0] = new_point_matrix(0, 0);
		newCoord[1] = new_point_matrix(0, 1);
		newCoord[2] = new_point_matrix(0, 2);

		mesh_list[current_mesh].set_point(vlt.handle(), newCoord);

		//std::cout << "newcoor:\n" << newCoord << "\n";

		//system("PAUSE");

	}



}

void colourByMesh() {


	for (int i = 0; i < mesh_list.size(); i++) {

		current_mesh = i;

		MyMesh::VertexIter vlt, vBegin, vEnd;
		vBegin = mesh_list[current_mesh].vertices_begin();
		vEnd = mesh_list[current_mesh].vertices_end();

		//std::cout << vBegin << "\t" << vEnd << "\n";

		float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		float g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		float b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

		for (vlt = vBegin; vlt != vEnd; ++vlt) {
			//do something here	

			OpenMesh::Vec4f newCol = OpenMesh::Vec4f(r, g, b, 1.0f);

			colourVertex(vlt.handle(), newCol);

		}
	}
}

void randomlyColour() {

	MyMesh::VertexIter vlt, vBegin, vEnd;
	vBegin = mesh_list[current_mesh].vertices_begin();
	vEnd = mesh_list[current_mesh].vertices_end();

	//std::cout << vBegin << "\t" << vEnd << "\n";

	for (vlt = vBegin; vlt != vEnd; ++vlt) {

		//do something here

		float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		float g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		float b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

		OpenMesh::Vec4f newCol = OpenMesh::Vec4f(r, g, b, 1.0f);
		colourVertex(vlt.handle(), newCol);

	}


}

bool readPt(std::istream &in, ANNpoint p)			// read point (false on EOF)
{
	for (int i = 0; i < dim; i++) {
		if (!(in >> p[i])) return false;
	}
	return true;
}

void printPt(std::ostream &out, ANNpoint p)			// print point
{
	out << "(" << p[0];
	for (int i = 1; i < dim; i++) {
		out << ", " << p[i];
	}
	out << ")\n";
}

void findNormals() {

	current_mesh = 0;
	//iterate over all vertices
	MyMesh::VertexIter vlt, vBegin, vEnd;
	vBegin = mesh_list[current_mesh].vertices_begin();
	vEnd = mesh_list[current_mesh].vertices_end();

	for (vlt = vBegin; vlt != vEnd; ++vlt) {

		OpenMesh::VertexHandle vh = vlt.handle();

		OpenMesh::Vec3f xi = mesh_list[current_mesh].point(vh);

		std::vector<OpenMesh::Vec3f> xj;

		OpenMesh::Vec3f unnormalisedS = OpenMesh::Vec3f(0.0f, 0.0f, 0.0f);
		float area = 0.0f;

		//now iterate over adjacent vertices
		for (MyMesh::ConstVertexVertexIter vvi = mesh_list[current_mesh].vv_iter(vh); vvi; ++vvi) {
			xj.push_back(mesh_list[current_mesh].point(vvi.handle()));
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

void findVertNormalsFromFaces() {
	current_mesh = 0;

	MyMesh::VertexIter vlt, vBegin, vEnd;
	vBegin = mesh_list[current_mesh].vertices_begin();
	vEnd = mesh_list[current_mesh].vertices_end();

	//itterate over all of the points
	//int n_adjacent = 0;

	for (vlt = vBegin; vlt != vEnd; ++vlt) {
		OpenMesh::VertexHandle vh = vlt.handle();
		//find the adjecent faces

		OpenMesh::Vec3f thisNormal = OpenMesh::Vec3f(0.0f, 0.0f, 0.0f);

		for (MyMesh::VertexFaceIter vfi = mesh_list[current_mesh].vf_iter(vh); vfi; ++vfi) {

			//n_adjacent++;
			thisNormal += mesh_list[current_mesh].normal(vfi.handle());

		}

		//Dont need this since I normalize
		//thisNormal *= 1.0f / float(n_adjacent);

		thisNormal = thisNormal.normalize();

		mesh_list[current_mesh].set_normal(vh, thisNormal);
	}


}

void findFaceNormals() {

	current_mesh = 0;

	//iterate over mesh's faces and find their normal
	for (MyMesh::FaceIter f_it = mesh_list[current_mesh].faces_begin(); f_it != mesh_list[current_mesh].faces_end(); ++f_it) {
		//MyMesh::FaceVertexIter OpenMesh::PolyConnectivity::FaceVertexIter OpenMesh::PolyConnectivity::fv_iter(FaceHandle _fh); (FaceHandle _fh);
		OpenMesh::FaceHandle fa = f_it.handle();

		//There should be three verts for every face.
		OpenMesh::Vec3f v1;
		OpenMesh::Vec3f v2;
		OpenMesh::Vec3f v3;

		int current_v = 0;

		for (MyMesh::FaceVertexIter fvi = mesh_list[current_mesh].fv_iter(fa); fvi; ++fvi) {
			OpenMesh::Vec3f thisCoord;
			thisCoord = mesh_list[current_mesh].point(fvi.handle());
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

		Eigen::Vector3f v1v2 = Eigen::Vector3f(v2[0] - v1[0], v2[1] - v1[1], v2[2] - v1[2]);
		Eigen::Vector3f v2v3 = Eigen::Vector3f(v3[0] - v2[0], v3[1] - v2[1], v3[2] - v2[2]);
		Eigen::Vector3f f_normal = v1v2.cross(v2v3);
		f_normal.normalize();

		OpenMesh::Vec3f this_normal = OpenMesh::Vec3f(f_normal(0), f_normal(1), f_normal(2));

		mesh_list[current_mesh].set_normal(fa, this_normal);

	}

}

void scale(OpenMesh::VertexHandle &_vh, float _alpha) {
	OpenMesh::Vec3f newCoord;
	newCoord = mesh_list[current_mesh].point(_vh);
	mesh_list[current_mesh].set_point(_vh, newCoord * _alpha);
}

void addNoiseToCurrentMesh(float _sigma) {

	current_mesh = 0;
	MyMesh::VertexIter vlt, vBegin, vEnd;
	vBegin = mesh_list[current_mesh].vertices_begin();
	vEnd = mesh_list[current_mesh].vertices_end();

	std::default_random_engine generator;
	std::normal_distribution<double> distribution(0.0, _sigma);


	for (vlt = vBegin; vlt != vEnd; ++vlt) {

		OpenMesh::Vec3f newCoord;
		newCoord = mesh_list[current_mesh].point(vlt.handle());

		OpenMesh::Vec3f noise = OpenMesh::Vec3f(0.0f, 0.0f, 0.0f);

		for (int dim_i = 0; dim_i < 3; dim_i++) {

			double number = distribution(generator);
			noise[dim_i] = (float)number;

		}

		newCoord += noise;

		mesh_list[current_mesh].set_point(vlt.handle(), newCoord);

	}


}

//used for denoise test
void addNoiseToNoisyMesh(float _sigma) {
	
	
	MyMesh::VertexIter vlt, vBegin, vEnd;
	vBegin = noisyMesh.vertices_begin();
	vEnd = noisyMesh.vertices_end();

	std::default_random_engine generator;
	std::normal_distribution<double> distribution(0.0, _sigma);


	for (vlt = vBegin; vlt != vEnd; ++vlt) {

		OpenMesh::Vec3f newCoord;
		newCoord = noisyMesh.point(vlt.handle());

		OpenMesh::Vec3f noise = OpenMesh::Vec3f(0.0f, 0.0f, 0.0f);

		for (int dim_i = 0; dim_i < 3; dim_i++) {

			double number = distribution(generator);
			noise[dim_i] = (float)number;

		}

		newCoord += noise;

		noisyMesh.set_point(vlt.handle(), newCoord);

	}


}

//find the curvature of the mesh
void findGaussCurvature2() {

	max_mean_curvture = -10000.0;
	min_mean_curvture = 10000.0;

	current_mesh = 0;
	//iterate over all vertices
	MyMesh::VertexIter vlt, vBegin, vEnd;
	vBegin = mesh_list[current_mesh].vertices_begin();
	vEnd = mesh_list[current_mesh].vertices_end();

	for (vlt = vBegin; vlt != vEnd; ++vlt) {

		OpenMesh::VertexHandle vh = vlt.handle();

		OpenMesh::Vec3f xi = mesh_list[current_mesh].point(vh);

		std::vector<OpenMesh::Vec3f> xj;

		OpenMesh::Vec3f unnormalisedS = OpenMesh::Vec3f(0.0, 0.0, 0.0);
		double area = 0.0;

		double angle_deficit = 0.0;

		//now iterate over adjacent vertices
		for (MyMesh::ConstVertexVertexIter vvi = mesh_list[current_mesh].vv_iter(vh); vvi; ++vvi) {
			xj.push_back(mesh_list[current_mesh].point(vvi.handle()));
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
			double alpha = acos(OpenMesh::dot(vecAB, vecAC) / (vecAB.norm() * vecAC.norm()));
			//beta angle is xj xj+1 xi
			vecAB = xi - xjp1;
			vecAC = xj[i] - xjp1;
			double beta = acos(OpenMesh::dot(vecAB, vecAC) / (vecAB.norm() * vecAC.norm()));
			//cot is cos/sin
			//w = cot alpha + cot beta
			double weight = (cos(alpha) / sin(alpha)) + (cos(beta) / sin(beta));
			//area is 1/3 of the triangle
			area += vecAB.norm()*vecAC.norm() * sin(beta) / 6.0;
			unnormalisedS += weight * (xj[i] - xi);

			vecAB = xj[i] - xi;
			vecAC = xjp1 - xi;

			angle_deficit += acos(OpenMesh::dot(vecAB, vecAC) / (vecAB.norm() * vecAC.norm()));

		}

		//now normalise S by area
		//in slides area is divided by 2 and S is multiplied by 2 I have skipped these steps
		double h = unnormalisedS.norm();

		h *= (1.0f / area);

		//check for -ve h
		OpenMesh::Vec3f normalVariation = (unnormalisedS.normalized() - mesh_list[current_mesh].normal(vh));// .norm();

		if (normalVariation.norm() < 0.5f) {
			h *= -1.0f;
		}

		//std::cout << h << "\n";

		//K = (2*pi - sum(angle))/Area
		double K = (2.0f*PI - angle_deficit) / area;

		if ((area == 0.0f) || (h == INFINITY)) {
			h = 0.0f;
			K = 0.0f;
		}

		double inside_sqrt = (h*h - K);

		if ((inside_sqrt >= 0.0) && (xj.size() > 2)) {

			double thisk1 = h + sqrt(inside_sqrt);
			double thisk2 = h - sqrt(inside_sqrt);

			mesh_list[current_mesh].property(k1, vlt.handle()) = h + sqrt(inside_sqrt);
			mesh_list[current_mesh].property(k2, vlt.handle()) = h - sqrt(inside_sqrt);

		}
		else {
			std::cout << "error: in find gauss curvature2: negative sqrt or less than 3 neighbouring vertices setting k1 and k2 to zero\n";
			h = 0.0;
			K = 0.0;
			mesh_list[current_mesh].property(k1, vlt.handle()) = 0.0;
			mesh_list[current_mesh].property(k2, vlt.handle()) = 0.0;
		}

		mesh_list[current_mesh].property(mean_curvature, vh) = h;
		mesh_list[current_mesh].property(gauss_curvature, vlt.handle()) = K;

		if (h > max_mean_curvture) max_mean_curvture = h;
		if (h < min_mean_curvture) min_mean_curvture = h;
		if (K > max_gauss_curvture) max_gauss_curvture = K;
		if (K < min_gauss_curvture) min_gauss_curvture = K;
		//if (thisk1 > max_k1_curvture) max_k1_curvture = thisk1;
		//if (thisk2 < min_k2_curvture) min_k2_curvture = thisk2;
	}

	std::cout << "max H: " << max_mean_curvture << "\n";
	std::cout << "min H: " << min_mean_curvture << "\n";
	std::cout << "max K: " << max_gauss_curvture << "\n";
	std::cout << "min K: " << min_gauss_curvture << "\n";

	//max_mean_curvture = 10000.0;
	//min_gauss_curvture = -1000000.0;

}

void eigenReconstruction(double lambda, int nLargestEigs) {

	std::cout << "\nstarting...\n";

	//use mesh 0
	current_mesh = 0;
	//iterate over all vertices
	MyMesh::VertexIter vlt, vBegin, vEnd;
	vBegin = mesh_list[current_mesh].vertices_begin();
	vEnd = mesh_list[current_mesh].vertices_end();
	int N = 0;
	for (vlt = vBegin; vlt != vEnd; ++vlt) {
		N++;
	}

	//construct A matrix - discrete laplacian
	//A = I - lambda L
	
	Eigen::SparseMatrix<double> L = Eigen::SparseMatrix<double>(N, N);	

	// make vectors (as a matrix) for old coords	
	Eigen::MatrixXd XnMat = Eigen::MatrixXd::Zero(N, 1);
	Eigen::MatrixXd YnMat = Eigen::MatrixXd::Zero(N, 1);
	Eigen::MatrixXd ZnMat = Eigen::MatrixXd::Zero(N, 1);

	//fill matrix and vector
	for (vlt = vBegin; vlt != vEnd; ++vlt) {
		OpenMesh::VertexHandle vh = vlt.handle();

		int i = vh.idx();
		int n_neighbours = 0;

		XnMat(i, 0) = mesh_list[current_mesh].point(vh)[0];
		YnMat(i, 0) = mesh_list[current_mesh].point(vh)[1];
		ZnMat(i, 0) = mesh_list[current_mesh].point(vh)[2];

		//now iterate over adjacent vertices
		for (MyMesh::ConstVertexVertexIter vvi = mesh_list[current_mesh].vv_iter(vh); vvi; ++vvi) {
			OpenMesh::VertexHandle vh2 = vvi.handle();
			n_neighbours++;			
		}

		double valence_normalisation = 1.0 / double(n_neighbours);

		for (MyMesh::ConstVertexVertexIter vvi = mesh_list[current_mesh].vv_iter(vh); vvi; ++vvi) {
			OpenMesh::VertexHandle vh2 = vvi.handle();
			//n_neighbours++;
			int j = vh2.idx();
			L.insert(i, j) = valence_normalisation * lambda * 1.0;
		}
		L.insert(i, i) = 1.0 - 1.0*lambda;

	}

	std::cout << "\nLaplacian matrix made...\n";

	// now have the laplacian matrix

	//take an eigen decomposition of it:
	Spectra::SparseGenMatProd<double> op(L);

	// ncv affects rate of convergance - higher means faster convergance but also more memory usage
	// ncv must be greater than n_largest Eigs + 2 and less than n which is size of the matrix
	// I've found through trial and error that below relation returns the desired number of eigen vectors
	int ncv = nLargestEigs + 30;
	
	// Want the dominant eigen vectors which are associated with the smallest magnitude of eiven values
	Spectra::GenEigsSolver< double, Spectra::SMALLEST_MAGN, Spectra::SparseGenMatProd<double> > eigs(&op, nLargestEigs, ncv);
	// Initialize and compute
	std::cout << "\nInitialising and solving...\n";

	eigs.init();
	int nconv = eigs.compute();
		
	// Retrieve results
	Eigen::VectorXcd evalues;
	Eigen::MatrixXd evecs;
	if (eigs.info() == Spectra::SUCCESSFUL) {
		evalues = eigs.eigenvalues();
		evecs = eigs.eigenvectors().real();
		std::cout << "eval and evec assigned\n";
	}
	else {
		std::cout << "Fewer eigen vectors than requested were returned, likely due to ncv being too small.\n";
		std::cout << "will continue with retuned eigen vecs\n";
		evecs = eigs.eigenvectors().real();
	}

	// new x = sum(i) of (x'e)e where e is the eigen vec to i
	std::cout << "\nSolved, now reconstructing mesh\n";	
	
	Eigen::MatrixXd Xn1Mat = Eigen::MatrixXd::Zero(N, 1);
	Eigen::MatrixXd Yn1Mat = Eigen::MatrixXd::Zero(N, 1);
	Eigen::MatrixXd Zn1Mat = Eigen::MatrixXd::Zero(N, 1);

	std::cout << "Xn size: " << N << "\n";
	std::cout << "nEig: " << nLargestEigs << "\n";
	
	for (int i = 0; i < evecs.cols(); i++) {
	
		MatrixXd thisEigenVec = Eigen::MatrixXd::Zero(N, 1);

		for (int j = 0; j < N; j++) {
			
			thisEigenVec(j, 0) = evecs(j, i); 
		}


		// sum of E^T X E	-	where E is the eigen vector, and X is the original point coords
		// The (0, 0) refers to E^T X being a 1x1 matrix which in turn scales the eigen vecs
		Xn1Mat += (thisEigenVec.transpose() * XnMat)(0, 0) * thisEigenVec;
		Yn1Mat += (thisEigenVec.transpose() * YnMat)(0, 0) * thisEigenVec;
		Zn1Mat += (thisEigenVec.transpose() * ZnMat)(0, 0) * thisEigenVec;
		
	}


	std::cout << "saving new coords...\n";
	

	//save new coords - update
	for (vlt = vBegin; vlt != vEnd; ++vlt) {

		OpenMesh::Vec3f newCoord;

		newCoord[0] = Xn1Mat(vlt.handle().idx(), 0);
		newCoord[1] = Yn1Mat(vlt.handle().idx(), 0);
		newCoord[2] = Zn1Mat(vlt.handle().idx(), 0);

		mesh_list[current_mesh].set_point(vlt.handle(), newCoord);

	}
	
	// find new normals for colours
	findFaceNormals();
	findVertNormalsFromFaces();

	std::cout << "done.\n";

}

Eigen::SparseMatrix<double> makeUniformLaplace() {
	
	current_mesh = 0;
	double lambda = -1.0;

	//use mesh 0
	current_mesh = 0;
	//iterate over all vertices
	MyMesh::VertexIter vlt, vBegin, vEnd;
	vBegin = mesh_list[current_mesh].vertices_begin();
	vEnd = mesh_list[current_mesh].vertices_end();
	int N = 0;
	for (vlt = vBegin; vlt != vEnd; ++vlt) {
		N++;
	}

	Eigen::SparseMatrix<double> L = Eigen::SparseMatrix<double>(N, N);

	for (vlt = vBegin; vlt != vEnd; ++vlt) {
		OpenMesh::VertexHandle vh = vlt.handle();

		int i = vh.idx();
		int n_neighbours = 0;

		//now iterate over adjacent vertices
		for (MyMesh::ConstVertexVertexIter vvi = mesh_list[current_mesh].vv_iter(vh); vvi; ++vvi) {
			OpenMesh::VertexHandle vh2 = vvi.handle();
			n_neighbours++;
		}

		double valence_normalisation = 1.0 / double(n_neighbours);

		for (MyMesh::ConstVertexVertexIter vvi = mesh_list[current_mesh].vv_iter(vh); vvi; ++vvi) {
			OpenMesh::VertexHandle vh2 = vvi.handle();
			//n_neighbours++;
			int j = vh2.idx();
			L.insert(i, j) = valence_normalisation * lambda;

		}
		L.insert(i, i) = 1.0 - 1.0*lambda;
		
	}

	return L;

}

Eigen::SparseMatrix<double> makeLaplaceBeltrami() {

	current_mesh = 0;
	double lambda = -1.0;

	//iterate over all vertices

	MyMesh::VertexIter vlt, vBegin, vEnd;
	vBegin = mesh_list[current_mesh].vertices_begin();
	vEnd = mesh_list[current_mesh].vertices_end();
	int N = 0;
	for (vlt = vBegin; vlt != vEnd; ++vlt) {
		N++;
	}

	Eigen::SparseMatrix<double> L = Eigen::SparseMatrix<double>(N, N);

	//iterate over all vertices
	for (vlt = vBegin; vlt != vEnd; ++vlt) {

		OpenMesh::VertexHandle vh = vlt.handle();
		OpenMesh::Vec3f xi = mesh_list[current_mesh].point(vlt.handle());

		std::vector<OpenMesh::Vec3f> xj;
		std::vector<int> xj_index;
		std::vector<double> xj_cotangent;
				
		float area = 0.0f;

		//now iterate over adjacent vertices
		for (MyMesh::ConstVertexVertexIter vvi = mesh_list[current_mesh].vv_iter(vh); vvi; ++vvi) {
			xj.push_back(mesh_list[current_mesh].point(vvi.handle()));
			xj_index.push_back(vvi.handle().idx());
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

			xj_cotangent.push_back(weight);

			// Add a third of the area of this triangle to the area normalisation term
			area += vecAB.norm()*vecAC.norm() * sin(beta) / 6.0f;
			
		}

		//now I should have the cotangent, which vertex it belongs to, and the area so can compute the laplace beltrami matrix
		
		double area_normalisation_term = 1.0 / (area*2.0);

		double all_cotangent = 0.0;

		//fill in off diagonals
		for (int i = 0; i < xj_index.size(); i++) {
			L.insert(vlt.handle().idx(), xj_index[i]) = area_normalisation_term * xj_cotangent[i]*-1;
			all_cotangent += xj_cotangent[i];
			//std::cout << "\nweight of non-diagonal: " << area_normalisation_term * xj_cotangent[i] << "\n";
		}
		
		L.insert(vlt.handle().idx(), vlt.handle().idx()) = all_cotangent * area_normalisation_term;
		//std::cout << "weight of diagonal: " << all_cotangent * area_normalisation_term * -1.0 << "\n";
		//system("PAUSE");
		//end of global vertex iterator
	}


	return L;


}

Eigen::SparseMatrix<double> makeLaplaceAreaWeighted() {

	current_mesh = 0;
	double lambda = -1.0;

	//use mesh 0
	current_mesh = 0;
	//iterate over all vertices
	MyMesh::VertexIter vlt, vBegin, vEnd;
	vBegin = mesh_list[current_mesh].vertices_begin();
	vEnd = mesh_list[current_mesh].vertices_end();
	int N = 0;
	for (vlt = vBegin; vlt != vEnd; ++vlt) {
		N++;
	}

	Eigen::SparseMatrix<double> L = Eigen::SparseMatrix<double>(N, N);

	for (vlt = vBegin; vlt != vEnd; ++vlt) {
		OpenMesh::VertexHandle vh = vlt.handle();

		int i = vh.idx();
		int n_neighbours = 0;

		//now iterate over adjacent vertices
		for (MyMesh::ConstVertexVertexIter vvi = mesh_list[current_mesh].vv_iter(vh); vvi; ++vvi) {
			OpenMesh::VertexHandle vh2 = vvi.handle();
			n_neighbours++;
		}

		double valence_normalisation = 1.0 / double(n_neighbours);

		for (MyMesh::ConstVertexVertexIter vvi = mesh_list[current_mesh].vv_iter(vh); vvi; ++vvi) {
			OpenMesh::VertexHandle vh2 = vvi.handle();
			//n_neighbours++;
			int j = vh2.idx();
			L.insert(i, j) = valence_normalisation * lambda * -1.0;

		}
		L.insert(i, i) = 1.0 - 1.0*lambda;

	}

	return L;


}

void findEigenVectors(int nLargestEigs, Laplacian_type lap_type) {

	//this function finds k largest eigenvectors for the chosen laplacian type - only uniform was used in project although beltrami should work too

	std::cout << "Finding Eigen Vectors... ";

	//use mesh 0
	current_mesh = 0;
	
	double lambda = -1.0;
	//iterate over all vertices
	MyMesh::VertexIter vlt, vBegin, vEnd;
	vBegin = mesh_list[current_mesh].vertices_begin();
	vEnd = mesh_list[current_mesh].vertices_end();
	//count vertices
	int N = 0;
	for (vlt = vBegin; vlt != vEnd; ++vlt) {
		N++;
	}

	//construct A matrix - discrete laplacian

	//A = I - lambda L	
	Eigen::SparseMatrix<double> L = Eigen::SparseMatrix<double>(N, N);	

	if (lap_type == uniform) {
		L = makeUniformLaplace();
	}
	else if (lap_type == beltrami) {
		L = makeLaplaceBeltrami();
	}
	else if (lap_type == area) {
		L - makeLaplaceAreaWeighted();
	}else {
		std::cout << "incorrect laplacian selected using uniform\n";
		L = makeUniformLaplace();
	}
	
	// now have the laplacian matrix	

	//take an eigen decomposition of it:
	Spectra::SparseGenMatProd<double> op(L);
	
	/* ncv affects rate of convergance - higher means faster convergance but also more memory usage
	 ncv must be greater than n_largest Eigs + 2 and less than n which is size of the matrix
	 I've found through trial and error that below relation returns the desired number of eigen vectors

	 Lower values of ncv do not return the requested number of eigenvectors
	*/

	int ncv = nLargestEigs + 30;

	// Want the dominant eigen vectors which are associated with the smallest magnitude of eiven values
	Spectra::GenEigsSolver< double, Spectra::SMALLEST_MAGN, Spectra::SparseGenMatProd<double> > eigs(&op, nLargestEigs, ncv);
	
	// Initialize and compute	
	eigs.init();
	int nconv = eigs.compute();

	// Retrieve results		
	if (eigs.info() == Spectra::SUCCESSFUL) {
		//assign the eigenvectors/values
		evals = eigs.eigenvalues();
		evecs = eigs.eigenvectors().real();
		std::cout << "eval and evec assigned\n";
	}
	else {
		//as mentioned above if ncv is too small then we will not have enough eigenvectors - here I let you know
		std::cout << "Fewer eigen vectors than requested were returned, likely due to ncv being too small.\n";
		std::cout << "will continue with retuned eigen vecs\n";
		evals = eigs.eigenvalues();
		evecs = eigs.eigenvectors().real();
	}

	//assign spectral coefficients
	evecs_coeffs = MatrixXd::Ones(nLargestEigs, 1);
	//temp are actually the ones which are used
	evecs_coeffs_temp = MatrixXd::Ones(nLargestEigs, 3);
	//assign globas value of how many eigenvectors are held
	currentlyHeldEvects = nLargestEigs;
	//use all held eigenvectors for reconstruction
	nEVecsToUse = currentlyHeldEvects;
	

}

//returns 1/N * SSD of current mesh and initial mesh
double SSD(void) {
	
	double ssd = 0.0;

	//use mesh 0
	current_mesh = 0;
	//iterate over all vertices
	MyMesh::VertexIter vlt, vBegin, vEnd;
	vBegin = unchangedMesh.vertices_begin();
	vEnd = unchangedMesh.vertices_end();

	int N = 0;
	for (vlt = vBegin; vlt != vEnd; ++vlt) {
		N++;

		OpenMesh::Vec3f diff = unchangedMesh.point(vlt.handle()) - mesh_list[current_mesh].point(vlt.handle());
		ssd += diff.sqrnorm();

	}

	ssd *= (1.0/N);

	return ssd;

}

//used for testing
void DoTest(std::string out_filename, int k, Laplacian_type lap_type) {
	std::cout << "running test\n";
	// do eig rec for n

	//check ssd for each eig rec k

	//save ssd for eack k to a text file

	int nLargestEigs = k;
	nEVecsToUse = nLargestEigs;
	std::string outputFilename = out_filename;

	findEigenVectors(nLargestEigs, lap_type);
	remakeFromEVecs(nLargestEigs);
	
	//openfile	
	std::ofstream out(outputFilename);
	out.precision(15);
	std::string input_title = "k\tssd\teigenval\tCx\tCy\tCz\n";
	out << input_title;

	//this is the ssd
	for (int i = 0; i < nLargestEigs; i++) {
		//remake with k
		remakeFromModifiedEVecs(i);
		//find ssd
		double thisSSD = SSD();
		//saveline

		std::string input;
		input.append(std::to_string(i));
		input.append("\t");
		input.append(std::to_string(thisSSD));
		input.append("\t");
		input.append(std::to_string(evals(i).real()));
		input.append("\t");
		input.append(std::to_string(evecs_coeffs_temp(i, 0)));
		input.append("\t");
		input.append(std::to_string(evecs_coeffs_temp(i, 1)));
		input.append("\t");
		input.append(std::to_string(evecs_coeffs_temp(i, 2)));
		input.append("\n");
		out << input;

	}

	//close file
	out.close();

	std::cout << "test finished\n";

}

//used for testing
void DenoiseTest(std::string out_filename, int k, Laplacian_type lap_type) {
	std::cout << "running test\n";
	// do eig rec for n

	//check ssd for each eig rec k

	//save ssd for eack k to a text file


	double sigma = 0.001;
	addNoiseToNoisyMesh(sigma);
	CopyNoisyMesh();
	//addnoise

	saveNoisyMesh();
	
	int nLargestEigs = k;
	nEVecsToUse = nLargestEigs;
	std::string outputFilename = out_filename;

	findEigenVectors(nLargestEigs, lap_type);
	remakeFromEVecs(nLargestEigs);

	//openfile	
	std::ofstream out(outputFilename);
	out.precision(15);
	std::string input_title = "k\tssd\teigenval\tCx\tCy\tCz\n";
	out << input_title;

	//this is the ssd
	for (int i = 0; i < nLargestEigs; i++) {
		//remake with k
		remakeFromModifiedEVecs(i);
		//find ssd
		double thisSSD = SSD();
		//saveline

		std::string input;
		input.append(std::to_string(i));
		input.append("\t");
		input.append(std::to_string(thisSSD));
		input.append("\t");
		input.append(std::to_string(evals(i).real()));
		input.append("\t");
		input.append(std::to_string(evecs_coeffs_temp(i, 0)));
		input.append("\t");
		input.append(std::to_string(evecs_coeffs_temp(i, 1)));
		input.append("\t");
		input.append(std::to_string(evecs_coeffs_temp(i, 2)));
		input.append("\n");
		out << input;

	}

	//close file
	out.close();
	saveMesh();
	std::cout << "test finished\n";
	
}

//used the first time - assigns spectral coefficients
void remakeFromEVecs(int nLargestEigs) {
	
	//use mesh 0
	current_mesh = 0;
	//iterate over all vertices
	MyMesh::VertexIter vlt, vBegin, vEnd;
	vBegin = noisyMesh.vertices_begin();
	vEnd = noisyMesh.vertices_end();
	//count vertices
	int N = 0;
	for (vlt = vBegin; vlt != vEnd; ++vlt) {
		N++;
	}
		
	// make vectors (as a matrix) for old coords	
	Eigen::MatrixXd XnMat = Eigen::MatrixXd::Zero(N, 1);
	Eigen::MatrixXd YnMat = Eigen::MatrixXd::Zero(N, 1);
	Eigen::MatrixXd ZnMat = Eigen::MatrixXd::Zero(N, 1);

	//fill vector
	for (vlt = vBegin; vlt != vEnd; ++vlt) {
		OpenMesh::VertexHandle vh = vlt.handle();

		int i = vh.idx();
		int n_neighbours = 0;

		XnMat(i, 0) = noisyMesh.point(vh)[0];
		YnMat(i, 0) = noisyMesh.point(vh)[1];
		ZnMat(i, 0) = noisyMesh.point(vh)[2];

	}

	// new points
	Eigen::MatrixXd Xn1Mat = Eigen::MatrixXd::Zero(N, 1);
	Eigen::MatrixXd Yn1Mat = Eigen::MatrixXd::Zero(N, 1);
	Eigen::MatrixXd Zn1Mat = Eigen::MatrixXd::Zero(N, 1);

	Eigen::MatrixXd Xn1MatOutput = Eigen::MatrixXd::Zero(N, 1);
	Eigen::MatrixXd Yn1MatOutput = Eigen::MatrixXd::Zero(N, 1);
	Eigen::MatrixXd Zn1MatOutput = Eigen::MatrixXd::Zero(N, 1);
		
	std::cout << "nEig: " << nLargestEigs << "\n";

	if (nLargestEigs > evecs.cols()) {
		nLargestEigs = evecs.cols();
		std::cout << "not enough e vecs\n";
	}

	
	displacementValues = Eigen::MatrixXd::Zero(N, nLargestEigs);

	for (int i = 0; i < nLargestEigs; i++) {	
			
		MatrixXd thisEigenVec = Eigen::MatrixXd::Zero(N, 1);
		
		for (int j = 0; j < N; j++) {
			thisEigenVec(j, 0) = evecs(j, i);
		}
		
		

		// sum of E^T X E	-	where E is the eigen vector, and X is the original point coords
		// The (0, 0) refers to E^T X being a 1x1 matrix which in turn scales the eigen vecs
		Xn1Mat += (thisEigenVec.transpose() * XnMat)(0, 0) * thisEigenVec;		
		Yn1Mat += (thisEigenVec.transpose() * YnMat)(0, 0) * thisEigenVec;		
		Zn1Mat += (thisEigenVec.transpose() * ZnMat)(0, 0) * thisEigenVec;

		//std::cout << thisEigenVec.transpose() * XnMat << "\n";

		Xn1MatOutput += evecs_coeffs(i, 0) * (thisEigenVec.transpose() * XnMat)(0, 0) * thisEigenVec;
		Yn1MatOutput += evecs_coeffs(i, 0) * (thisEigenVec.transpose() * YnMat)(0, 0) * thisEigenVec;
		Zn1MatOutput += evecs_coeffs(i, 0) * (thisEigenVec.transpose() * ZnMat)(0, 0) * thisEigenVec;

		evecs_coeffs_temp(i, 0) = evecs_coeffs(i, 0) * (thisEigenVec.transpose() * XnMat)(0, 0);
		evecs_coeffs_temp(i, 1) = evecs_coeffs(i, 0) * (thisEigenVec.transpose() * YnMat)(0, 0);
		evecs_coeffs_temp(i, 2) = evecs_coeffs(i, 0) * (thisEigenVec.transpose() * ZnMat)(0, 0);

		for (int l = 0; l < 3; l++) {
			if (min_coeff > evecs_coeffs_temp(i, l)) min_coeff = evecs_coeffs_temp(i, l);
			if (max_coeff < evecs_coeffs_temp(i, l)) max_coeff = evecs_coeffs_temp(i, l);
		}

		//thisEigenVec *= evecs_coeffs(i, 0);
		thisEigenVec *= 2.0;

		//Keep track of a displacement of the mesh when an eigen vector coeff is changed
		Eigen::MatrixXd Xn1MatDisp = Xn1Mat + (thisEigenVec.transpose() * XnMat)(0, 0) * thisEigenVec;
		Eigen::MatrixXd Yn1MatDisp = Yn1Mat + (thisEigenVec.transpose() * YnMat)(0, 0) * thisEigenVec;
		Eigen::MatrixXd Zn1MatDisp = Zn1Mat + (thisEigenVec.transpose() * ZnMat)(0, 0) * thisEigenVec;
		Xn1MatDisp -= Xn1Mat;
		Yn1MatDisp -= Xn1Mat;
		Zn1MatDisp -= Xn1Mat;

		double max_disp = 0.0;

		//displacement values are used to colour mesh when displaying spectral coefficients

		for (int k = 0; k < N; k++) {
			displacementValues(k, i) = Xn1MatDisp(k, 0) * Xn1MatDisp(k, 0) + Yn1MatDisp(k, 0) * Yn1MatDisp(k, 0) + Zn1MatDisp(k, 0)* Zn1MatDisp(k, 0);
			//displacementValues(k, i) = sqrt(displacementValues(k, i));
			if (displacementValues(k, i) > max_disp) max_disp = displacementValues(k, i);
		}

		max_disp = (1.0 / max_disp); // this is now a scale factor

		for (int k = 0; k < N; k++) {
			displacementValues(k, i) *= max_disp;
			
		}
	}
			
	//save new coords - update1
	for (vlt = vBegin; vlt != vEnd; ++vlt) {

		OpenMesh::Vec3f newCoord;

		newCoord[0] = Xn1MatOutput(vlt.handle().idx(), 0);
		newCoord[1] = Yn1MatOutput(vlt.handle().idx(), 0);
		newCoord[2] = Zn1MatOutput(vlt.handle().idx(), 0);

		mesh_list[current_mesh].set_point(vlt.handle(), newCoord);

	}

	std::cout << "min co: " << min_coeff << "\nmax co: " << max_coeff << "\n";

	// find new normals for colours
	findFaceNormals();
	findVertNormalsFromFaces();
	

}

//used when editing spectral coeffs - removes requirment to recalculate eigenvectors / coefficients
void remakeFromModifiedEVecs(int nLargestEigs) {
	
	//use mesh 0
	current_mesh = 0;
	//iterate over all vertices
	MyMesh::VertexIter vlt, vBegin, vEnd;
	vBegin = noisyMesh.vertices_begin();
	vEnd = noisyMesh.vertices_end();

	int N = 0;
	for (vlt = vBegin; vlt != vEnd; ++vlt) {
		N++;
	}

	// make vectors (as a matrix) for old coords	
	Eigen::MatrixXd XnMat = Eigen::MatrixXd::Zero(N, 1);
	Eigen::MatrixXd YnMat = Eigen::MatrixXd::Zero(N, 1);
	Eigen::MatrixXd ZnMat = Eigen::MatrixXd::Zero(N, 1);

	//fill vector
	for (vlt = vBegin; vlt != vEnd; ++vlt) {
		OpenMesh::VertexHandle vh = vlt.handle();

		int i = vh.idx();
		int n_neighbours = 0;

		XnMat(i, 0) = noisyMesh.point(vh)[0];
		YnMat(i, 0) = noisyMesh.point(vh)[1];
		ZnMat(i, 0) = noisyMesh.point(vh)[2];

	}

	// new points
	Eigen::MatrixXd Xn1Mat = Eigen::MatrixXd::Zero(N, 1);
	Eigen::MatrixXd Yn1Mat = Eigen::MatrixXd::Zero(N, 1);
	Eigen::MatrixXd Zn1Mat = Eigen::MatrixXd::Zero(N, 1);

	Eigen::MatrixXd Xn1MatOutput = Eigen::MatrixXd::Zero(N, 1);
	Eigen::MatrixXd Yn1MatOutput = Eigen::MatrixXd::Zero(N, 1);
	Eigen::MatrixXd Zn1MatOutput = Eigen::MatrixXd::Zero(N, 1);

	if (nLargestEigs > evecs.cols()) {
		nLargestEigs = evecs.cols();
		std::cout << "not enough e vecs\n";
	}

	int n_start = currentlyHeldEvects - nLargestEigs;

	if (n_start < 0) n_start = 0;

	for (int i = n_start; i < currentlyHeldEvects; i++) {
			

		MatrixXd thisEigenVec = Eigen::MatrixXd::Zero(N, 1);

		for (int j = 0; j < N; j++) {
			thisEigenVec(j, 0) = evecs(j, i);
		}



		// sum of E^T X E	-	where E is the eigen vector, and X is the original point coords
		// The (0, 0) refers to E^T X being a 1x1 matrix which in turn scales the eigen vecs
		
		Xn1MatOutput += evecs_coeffs_temp(i, 0) * thisEigenVec;
		Yn1MatOutput += evecs_coeffs_temp(i, 1) * thisEigenVec;
		Zn1MatOutput += evecs_coeffs_temp(i, 2) * thisEigenVec;
		
	}

	//save new coords - update
	for (vlt = vBegin; vlt != vEnd; ++vlt) {

		OpenMesh::Vec3f newCoord;

		newCoord[0] = Xn1MatOutput(vlt.handle().idx(), 0);
		newCoord[1] = Yn1MatOutput(vlt.handle().idx(), 0);
		newCoord[2] = Zn1MatOutput(vlt.handle().idx(), 0);

		mesh_list[current_mesh].set_point(vlt.handle(), newCoord);

	}

	// find new normals for colours
	findFaceNormals();
	findVertNormalsFromFaces();	
	findGaussCurvature2();
	max_mean_curvture = max_mean_curvture*0.3;
	min_mean_curvture = min_mean_curvture*0.3;


}

void findGaussCurvature() {
	current_mesh = 0;

	int wrongCurves = 0;
	int n_curves = 0;

	MyMesh::VertexIter vlt, vBegin, vEnd;
	vBegin = mesh_list[current_mesh].vertices_begin();
	vEnd = mesh_list[current_mesh].vertices_end();

	for (vlt = vBegin; vlt != vEnd; ++vlt) {
		//over each vertex

		float max_angle = 0.0f;
		float min_angle = 100.0f;

		//need to sum areas and angles at the current vertex

		//K = (2*pi - sum(angle))/Area
		//Note area is 1/3 area of all adjacent faces

		float area = 0;
		float angle = 0;

		//iterate over faces
		int n_adjacent_faces = 0;
		for (MyMesh::ConstVertexFaceIter vfi = mesh_list[current_mesh].vf_iter(vlt.handle()); vfi; ++vfi) {
			n_adjacent_faces++;
			OpenMesh::Vec3f core_vert = mesh_list[current_mesh].point(vlt.handle());

			//Should be three verts
			OpenMesh::Vec3f v1;
			OpenMesh::Vec3f v2;
			OpenMesh::Vec3f v3;

			int vert_count = 0;

			//I need coords of opposite vertecies
			for (MyMesh::FaceVertexIter fvi = mesh_list[current_mesh].fv_iter(vfi.handle()); fvi; ++fvi) {

				vert_count++;

				switch (vert_count) {
				case 1:
					v1 = mesh_list[current_mesh].point(fvi.handle());
					break;
				case 2:
					v2 = mesh_list[current_mesh].point(fvi.handle());
					break;
				case 3:
					v3 = mesh_list[current_mesh].point(fvi.handle());
					break;
				default:
					std::cout << "Error in findGaussCurvature: More than three vertices in triangle\n";
					break;
				}
			}

			//I now have the faces verts so I can calculate the angle and area

			OpenMesh::Vec3f vecAB;
			OpenMesh::Vec3f vecAC;

			OpenMesh::Vec3f point_A;
			OpenMesh::Vec3f point_B;
			OpenMesh::Vec3f point_C;

			if (v1 == core_vert) {
				point_A = v1;
				point_B = v2;
				point_C = v3;
				//std::cout << "v1\n";
			}
			else if (v2 == core_vert) {
				point_A = v2;
				point_B = v1;
				point_C = v3;
				//std::cout << "v2\n";
			}
			else if (v3 == core_vert) {
				point_A = v3;
				point_B = v1;
				point_C = v2;
				//std::cout << "v3\n";
			}
			else {
				std::cout << "Error in findGaussCurvature: cannot find starting vertex\n";
			}

			if (point_A == point_B || point_A == point_C || point_B == point_C) {
				std::cout << "Error in find gauss curvature: points are the same\n";
			}

			vecAB = point_B - point_A;
			vecAC = point_C - point_A;

			//AB dot AC / |AB||AC| = cosThete
			//theta = acos(AB dot AC / |AB||AC|)

			float theta = acos(OpenMesh::dot(vecAB, vecAC) / (vecAB.norm() * vecAC.norm()));

			if (theta > max_angle) max_angle = theta;
			if (theta < min_angle) min_angle = theta;

			//Area = 0.5 * |AB||AC|sin theta
			float thisArea = vecAB.norm()*vecAC.norm() * sin(theta) / 6.0f;
			area += thisArea;
			angle += theta;
		}

		//if(n_adjacent_faces == )
		if (n_adjacent_faces != 1) {

			if ((PI*2.0f - angle) < 0.0f) {
				std::cout << "negative angle deficit" << "\n";
			}

			float K = (PI*2.0f - angle) / area;



			if (area == 0.0) {
				std::cout << area << "\n";
				K = 1.0;
			}

			float H = mesh_list[current_mesh].property(mean_curvature, vlt.handle());
			float inside_sqrt = H*H - K;


			n_curves++;

			if (inside_sqrt < 0.0) {
				wrongCurves++;
				//cover the thing
				mesh_list[current_mesh].property(gauss_curvature, vlt.handle()) = 0.0;
				mesh_list[current_mesh].property(k1, vlt.handle()) = 0.0;
				mesh_list[current_mesh].property(k2, vlt.handle()) = 0.0;
				inside_sqrt *= -1.0f;
			}
			//else {

			float thisk1 = H + sqrt(inside_sqrt);
			float thisk2 = H - sqrt(inside_sqrt);
			mesh_list[current_mesh].property(gauss_curvature, vlt.handle()) = K;
			mesh_list[current_mesh].property(k1, vlt.handle()) = thisk1;
			mesh_list[current_mesh].property(k2, vlt.handle()) = thisk2;

			if (K > max_gauss_curvture) max_gauss_curvture = K;
			if (K < min_gauss_curvture) min_gauss_curvture = K;
			if (thisk1 > max_k1_curvture) max_k1_curvture = thisk1;
			if (thisk2 < min_k2_curvture) min_k2_curvture = thisk2;
			//}

		}


		//std::cout << "min angle: \t" << min_angle << "\tmax angle: \t" << max_angle << "\n";

	}

	std::cout << "max k : " << max_gauss_curvture << "\n";
	std::cout << "min k : " << min_gauss_curvture << "\n";
	std::cout << "max k1: " << max_k1_curvture << "\n";
	std::cout << "min k2: " << min_k2_curvture << "\n";
	std::cout << "ratio of bad curvatures: " << wrongCurves / n_curves << "\n";

}

OpenMesh::Vec3f getCentreOfMesh() {

	int nVerts = 0;

	OpenMesh::Vec3f averageLocation = OpenMesh::Vec3f();
	averageLocation = averageLocation * 0.0f;


	MyMesh::VertexIter vlt, vBegin, vEnd;
	vBegin = mesh_list[current_mesh].vertices_begin();
	vEnd = mesh_list[current_mesh].vertices_end();

	//std::cout << vBegin << "\t" << vEnd << "\n";

	for (vlt = vBegin; vlt != vEnd; ++vlt) {
		nVerts++;
		//do something here
		OpenMesh::Vec3f newCoord;
		newCoord = mesh_list[current_mesh].point(vlt.handle());
		averageLocation = averageLocation + newCoord;

	}
	averageLocation = averageLocation / nVerts;
	std::cout << averageLocation << "\n";



	return averageLocation;

}

static void error_callback(int error, const char* description)
{
	fputs(description, stderr);
}

//	Functions for user input

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{

	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);

	if (key == GLFW_KEY_H && action == GLFW_PRESS) {
		std::cout << "H: HELP:\n\n";

		std::cout << "Arrow keys:\trotate and scale mesh.\n";
		std::cout << "Space bar:\tcentres mesh.\n";
		std::cout << "N:\tFind normals and curvature.\n";
		std::cout << "S:\tSave Mesh as .obj file.\n";
		std::cout << "R:\tMove mesh to origin.\n";
	}

	//in help
	if (key == GLFW_KEY_LEFT && action == GLFW_PRESS) {
		global_rotation -= 5.0f;
		std::cout << "Left\n";
	}
	//in help
	if (key == GLFW_KEY_RIGHT && action == GLFW_PRESS) {
		global_rotation += 5.0f;
		std::cout << "right\n";
	}
	//in help
	if (key == GLFW_KEY_UP && action == GLFW_PRESS) {
		globalScale += 1.0f;
		std::cout << "UP: Zoom in\n";
	}
	//in help
	if (key == GLFW_KEY_DOWN && action == GLFW_PRESS) {
		globalScale -= 1.0f;
		std::cout << "DOWN: Zoom out\n";
	}
	//in help
	if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
		global_translation = getCentreOfMesh(); //get centre
		global_translation *= -4.0f;
		std::cout << "SPACE: Mesh Centred\n";
	}
	
	//in help
	if (key == GLFW_KEY_N && action == GLFW_PRESS) {
		std::cout << "N: Generating normals...\t";
		findFaceNormals();
		findVertNormalsFromFaces();
		findGaussCurvature2();
		std::cout << "Done\n";
	}
	
	//in help
	if (key == GLFW_KEY_S && action == GLFW_PRESS) {
		saveMesh();
		std::cout << "S: mesh Saved\n";
	}
		
	//help
	if (key == GLFW_KEY_R && action == GLFW_PRESS) {
		moveMeshesToOrigin();
		std::cout << "R: Meshes moved to Origin\n";
	}
	
}

static void mouse_callback(GLFWwindow* window, int button, int action, int mods) {

	if (button == GLFW_MOUSE_BUTTON_1) {
		//mouse button 1 - Left Mouse Button
		TheMouse.lmb = action;

		if (action == 1) {

			for (int i = 0; i < buttonList.size(); i++) {
				ButtonPress(&buttonList[i], TheMouse.x, TheMouse.y);
			}
			TheMouse.xpress = TheMouse.x;
			TheMouse.ypress = TheMouse.y;
		}
		else if (action == 0) {
			for (int i = 0; i < buttonList.size(); i++) {
				ButtonRelease(&buttonList[i], TheMouse.x, TheMouse.y);
			}
		}

	}
	else if (button == GLFW_MOUSE_BUTTON_2) {
		//mouse button 2 - Right Mouse Button
		TheMouse.rmb = action;
	}
	else {
		//
	}

}

static void mouse_moved_callback(GLFWwindow* window, double x_pos, double y_pos) {

	TheMouse.dx = TheMouse.x - x_pos;
	TheMouse.dy = TheMouse.y - y_pos;

	TheMouse.x = (int)x_pos;
	TheMouse.y = (int)y_pos;

	bool anyButton = false;

	for (int i = 0; i < buttonList.size(); i++) {
		if (ButtonPassive(&buttonList[i], TheMouse.x, TheMouse.y)) {
			anyButton = true;
		}
	}

	if (!anyButton) {
		//MatrixXd thisRotation = tm.getRotationMatrix(TheMouse.dx, 0, TheMouse.dy);
		//globalRotationMatrix = globalRotationMatrix * thisRotation;
		if (TheMouse.lmb) global_rotation -= float(TheMouse.dx);// / 1.0f;
	}


}

//	End of functions for user input



//	UI functions

void Font(void *font, char *text, int x, int y)
{
	glRasterPos2i(x, y);

	while (*text != '\0')
	{
		glutBitmapCharacter(font, *text);
		++text;
	}
}

void DrawButton(Button *b) {

	//deal with font if I want text

	if (b) {
		if (b->highlighted)
			glColor3f(0.7f, 0.7f, 0.8f);
		else
			glColor3f(0.6f, 0.6f, 0.6f);
	}

	//draw button background
	glBegin(GL_QUADS);
	glVertex2i(b->x_top_left, b->y_top_left);
	glVertex2i(b->x_top_left, b->y_top_left + b->h);
	glVertex2i(b->x_top_left + b->w, b->y_top_left + b->h);
	glVertex2i(b->x_top_left + b->w, b->y_top_left);
	glEnd();

	glTranslatef(0.0f, 0.0f, 0.1f);

	//Button Outline
	glLineWidth(3);
	//Change colour when clicked
	if (b->state)
		glColor3f(0.4f, 0.4f, 0.4f);
	else
		glColor3f(0.8f, 0.8f, 0.8f);

	glBegin(GL_LINE_STRIP);
	glVertex2i(b->x_top_left + b->w, b->y_top_left);
	glVertex2i(b->x_top_left, b->y_top_left);
	glVertex2i(b->x_top_left, b->y_top_left + b->h);
	glEnd();

	if (b->state)
		glColor3f(0.8f, 0.8f, 0.8f);
	else
		glColor3f(0.4f, 0.4f, 0.4f);

	glBegin(GL_LINE_STRIP);
	glVertex2i(b->x_top_left, b->y_top_left + b->h);
	glVertex2i(b->x_top_left + b->w, b->y_top_left + b->h);
	glVertex2i(b->x_top_left + b->w, b->y_top_left);
	glEnd();

	glLineWidth(1);

	//Draw text

	int fontx;
	int fonty;

	//set middle of the text
	//fontx = b->x_top_left + (b->w - glutBitmapLength(GLUT_BITMAP_HELVETICA_10, b->slabel)) / 2;
	fontx = b->x_top_left + (b->w) / 2 - 20;
	fonty = b->y_top_left + (b->h + 10) / 2;

	//if button pressed move string
	if (b->state) {
		fontx += 2;
		fonty += 2;
	}

	glTranslatef(0.0f, 0.0f, 0.1f);

	//when highlighted use a different colour
	if (b->highlighted)
	{
		glColor3f(0, 0, 0);
		Font(GLUT_BITMAP_HELVETICA_10, b->label, fontx, fonty);
		fontx--;
		fonty--;
	}



	glColor3f(1, 1, 1);
	Font(GLUT_BITMAP_HELVETICA_10, b->label, fontx, fonty);
	glTranslatef(0.0f, 0.0f, -0.20f);



}

void DrawTextBoxes(TextBox *tb) {
	int fontx;
	int fonty;
	
	
	fontx = tb->x_top_left + (tb->w) / 2 - 20;
	fonty = tb->y_top_left + (tb->h + 10) / 2;
	glTranslatef(0.0f, 0.0f, 0.1f);
	glColor3f(1, 1, 1);
	Font(GLUT_BITMAP_HELVETICA_10, tb->label, fontx, fonty);
	glTranslatef(0.0f, 0.0f, -0.10f);
}

void DrawGUI(void) {

	//This function will be called within the display function and draw the GUI for my mesh viewer
	for (int i = 0; i < buttonList.size(); i++) {
		DrawButton(&buttonList[i]);
	}

	for (int i = 0; i < textBoxList.size(); i++) {
		DrawTextBoxes(&textBoxList[i]);
	}

}

//	End of UI Funcitons


void display_pointcloud(GLFWwindow* window) {
	float ratio;
	int width, height;



	glfwGetFramebufferSize(window, &width, &height);
	ratio = width / (float)height;

	glViewport(0, 0, width, height);
	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-ratio, ratio, -1.f, 1.f, -1.f, 1.f);
	//glFrustum(-1.0, 1.0, -1.0, 1.0, 0.0, 2.0);
	glMatrixMode(GL_MODELVIEW);


	glLoadIdentity();
	glPushMatrix();
	glTranslatef(global_translation[0], global_translation[1], global_translation[2]);
	if (editmode) {
		glBegin(GL_LINE);

		if (current_axis = 0) {
			glColor3f(1.0f, 0.0f, 0.0f);
			glVertex3f(-1.0f, 0.0f, 0.0f);
			glVertex3f(1.0f, 0.0f, 0.0f);
		}

		glEnd();
	}
	glPopMatrix();

	for (int i = 0; i < mesh_list.size(); i++) {
		//pop
		current_mesh = i;

		glPushMatrix();

		//rotate
		glRotatef(global_rotation, 0.f, 1.f, 0.f);

		//translate so in centre
		glTranslatef(global_translation[0], global_translation[1], global_translation[2]);

		//scale here will act as a zoom
		glScalef(globalScale, globalScale, globalScale);

		glPointSize(4.0f);

		glBegin(GL_POINTS);
		glColor3f(1.f, 1.f, 1.f);



		MyMesh::VertexIter vlt, vBegin, vEnd;
		vBegin = mesh_list[current_mesh].vertices_begin();
		vEnd = mesh_list[current_mesh].vertices_end();

		//std::cout << vBegin << "\t" << vEnd << "\n";

		for (vlt = vBegin; vlt != vEnd; ++vlt) {
			//do something here
			OpenMesh::Vec3f thisCoord;
			OpenMesh::Vec4f thisCol;
			thisCoord = mesh_list[current_mesh].point(vlt.handle());
			thisCol = mesh_list[current_mesh].color(vlt.handle());
			glColor3f(thisCol[0], thisCol[1], thisCol[2]);
			glVertex3f(thisCoord[0], thisCoord[1], thisCoord[2]);

		}

		glEnd();

		//pop
		glPopMatrix();
	}


	// Set the orthographic viewing transformation	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, window_w, window_h, 0, -1, 1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	DrawGUI();

	glfwSwapBuffers(window);
	glfwPollEvents();

}

OpenMesh::Vec3f simpleColourMap(float value, float max, float min) {
	OpenMesh::Vec3f  colour;

	float pos = (value - min) / (max - min);
	float V = 1.0f;
	if (pos > 1.0f || pos < 0.0f) {
		//std::cout << "Error in simple colour map: outside colour range\n";
		colour[0] = 1.0f;
		colour[1] = 1.0f;
		colour[2] = 1.0f;
		return colour;
	}
	else {

		//http://academe.co.uk/2012/04/arduino-cycling-through-colours-of-the-rainbow/

		//red
		if (pos > 0.5) {
			colour[0] = 0.0f;
		}
		else if (pos > 0.33333333f) {
			colour[0] = V*(-6.0f*pos + 3.0f);
		}
		else if (pos > 0.16666667f) {
			colour[0] = V;
		}
		else {
			colour[0] = V*(6.0f*pos);
		}

		//green
		if (pos > 0.8333333f) {
			colour[1] = 0.0f;
		}
		else if (pos > 0.666667f) {
			colour[1] = V * (-6.0f * pos + 5.0f);
		}
		else if (pos > 0.333333f) {
			colour[1] = V;
		}
		else if (pos > 0.1666667f) {
			colour[1] = V * (6.0f * pos - 1.0f);
		}
		else {
			colour[1] = 0.0f;
		}

		//blue
		if (pos > 0.8333333f) {
			colour[2] = V * (-6.0f * pos + 6);
		}
		else if (pos > 0.666667f) {
			colour[2] = V;
		}
		else if (pos > 0.5f) {
			colour[2] = V * (6.0f * pos - 3.0f);
		}
		else {
			colour[2] = 0.0f;
		}

		return colour;
	}

}

OpenMesh::Vec3f principleKColourmap(double k1, double k2) {

	OpenMesh::Vec3f colour = OpenMesh::Vec3f(1.0f, 1.0f, 1.0f);
	//draw curvatures, allow for a threshold so that we can see umbillic and parabollic 

	double umbillic = (k1 - k2)*(k1 - k2);
	double umbillic_threshold = 10.0;
	if (umbillic < umbillic_threshold) {
		// umbillic point
		colour[0] = 1.0f;
		colour[1] = 1.0f;
		colour[2] = 1.0f;
	}
	//else if(k1*k2 == 0.0){
	else if (k1*k1 < umbillic_threshold || k2*k2 < umbillic_threshold) {
		//parabollic
		colour[0] = 0.0f;
		colour[1] = 1.0f;
		colour[2] = 0.0f;
	}

	else if (k1*k2 > 0.0) {
		//elliptic point
		colour[0] = 1.0f;
		colour[1] = 0.0f;
		colour[2] = 0.0f;
	}
	else if (k1*k2 < 0.0) {
		//hyperbolic point
		colour[0] = 0.0f;
		colour[1] = 0.0f;
		colour[2] = 1.0f;
	}
	else {
		colour[0] = 0.0f;
		colour[1] = 0.0f;
		colour[2] = 0.0f;
	}

	return colour;

}

void display_graphs(int width, int height) {

	glDisable(GL_DEPTH_TEST);
	//can I plot a graph here?
	//glEnable(GL_DEPTH_TEST);
	//glDepthMask(GL_TRUE);

	glViewport(0, 0, width, int(height*0.2));
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, window_w, window_h, 0, -1, 1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();


	float unit_length = evals.rows() / 620.0f;

	float x_origin = 20.0;
	float y_origin = height / 2.0;

	float x_end = 620.0;
	float y_max = height - 40.0;
	float y_min = 40.0;

	float x_range = x_end - x_origin;
	float y_range = y_max - y_min;

	float x_inc = float(x_range) / float(evals.rows());

	//draw area such showing region selected
	glBegin(GL_TRIANGLE_FAN);
	glColor3f(0.4f, 0.4f, 0.4f);
	glTranslatef(0.0f, 0.0f, +0.1f);
	float x_used_verts = ((nEVecsToUse - nEVecsToShow) * x_range / nEVecsToUse);

	glVertex3f(x_origin, y_max, 0.0f);
	glVertex3f(x_used_verts + x_origin, y_max, 0.0f);
	glVertex3f(x_used_verts + x_origin, y_min, 0.0f);
	glVertex3f(x_origin, y_min, 0.0f);
	glTranslatef(0.0f, 0.0f, -0.1f);
	glEnd();
	// End of drawing region


	glColor3f(0.9f, 0.9f, 0.9f);
	glLineWidth(1.0);
	glBegin(GL_LINES);

	glVertex3f(x_origin, y_origin, 0.0f);
	glVertex3f(x_end, y_origin, 0.0f);

	glVertex3f(x_origin, y_max, 0.0f);
	glVertex3f(x_origin, y_min, 0.0f);



	y_min *= 5.0;
	//	evecs_coeffs_temp
	for (int d = 0; d < 3; d++) {

		if (d == 0)glColor3f(1.0f, 0.0f, 0.0f);
		if (d == 1)glColor3f(0.0f, 1.0f, 0.0f);
		if (d == 2)glColor3f(0.0f, 0.0f, 1.0f);

		for (int i = 1; i < evals.rows() - 1; i++) {

			//std::cout << evals[i] << "\n";

			float this_x = x_origin + (i * x_inc);
			float this_y = y_origin + evecs_coeffs_temp(i, d) * y_min; //might be minus if y points down

			glVertex3f(this_x, this_y, 0.0f);

			this_x = x_origin + ((i - 1) * x_inc);
			this_y = y_origin + evecs_coeffs_temp(i - 1, d) * y_min; //might be minus if y points down

			glVertex3f(this_x, this_y, 0.0f);

			//		std::cout << this_x << "\n";
		}
	}

	//need a marker to indicate which eigen is selected
	glColor3f(1.0f, 1.0f, 1.0f);
	float current_eig_marker_x = x_origin + (whichEigToDraw * x_inc);

	glVertex3f(current_eig_marker_x, y_max - 80, 0.0f);
	glVertex3f(current_eig_marker_x, y_min*0.2 + 80, 0.0f);


	glEnd();

	//write some numbers
	int fontx;
	int fonty;

	std::string temp_str = std::to_string(whichEigToDraw);
	char* char_type = new char[temp_str.length()];
	strcpy(char_type, temp_str.c_str());

	fontx = current_eig_marker_x - 10;
	fonty = y_origin;
	glColor3f(1, 1, 0);
	Font(GLUT_BITMAP_HELVETICA_10, char_type, fontx, fonty - 150);
	//End of selection marker

}

void display(GLFWwindow* window) {
	float ratio;
	int width, height;

	glfwGetFramebufferSize(window, &width, &height);
	ratio = width / (float)height;

	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

	glViewport(0, 0, width, height);
	//glViewport(0, int(height*0.2), width, int(height*0.8));

	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-ratio, ratio, -1.f, 1.f, -1.f, 1.f);
	glMatrixMode(GL_MODELVIEW);
	
	glLoadIdentity();

	for (int i = 0; i < mesh_list.size(); i++) {
		//pop
		current_mesh = i;

		glPushMatrix();

		//rotate
		glRotatef(global_rotation, 0.f, 1.f, 0.f);

		//translate so in centre
		glTranslatef(global_translation[0], global_translation[1], global_translation[2]);

		//scale here will act as a zoom
		glScalef(globalScale, globalScale, globalScale);



		if (pointcloud) {
			MyMesh::VertexIter vlt, vBegin, vEnd;
			vBegin = mesh_list[current_mesh].vertices_begin();
			vEnd = mesh_list[current_mesh].vertices_end();
			glPointSize(1.0f);

			glBegin(GL_POINTS);

			for (vlt = vBegin; vlt != vEnd; ++vlt) {
				
				OpenMesh::Vec3f thisCoord;
				OpenMesh::Vec4f thisCol;
				thisCoord = mesh_list[current_mesh].point(vlt.handle());
				
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

			if (wireframe) {
				glColor3f(1.f, 1.f, 1.f);
				glLineWidth(1.5);
				//Draw wireframe
				for (MyMesh::EdgeIter e_it = mesh_list[current_mesh].edges_begin(); e_it != mesh_list[current_mesh].edges_end(); ++e_it) {
					MyMesh::Point to = mesh_list[current_mesh].point(mesh_list[current_mesh].to_vertex_handle(mesh_list[current_mesh].halfedge_handle(e_it, 0)));
					MyMesh::Point from = mesh_list[current_mesh].point(mesh_list[current_mesh].from_vertex_handle(mesh_list[current_mesh].halfedge_handle(e_it, 0)));
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
			for (MyMesh::FaceIter f_it = mesh_list[current_mesh].faces_begin(); f_it != mesh_list[current_mesh].faces_end(); ++f_it) {
				//MyMesh::FaceVertexIter OpenMesh::PolyConnectivity::FaceVertexIter OpenMesh::PolyConnectivity::fv_iter(FaceHandle _fh); (FaceHandle _fh);
				OpenMesh::FaceHandle fa = f_it.handle();
				OpenMesh::Vec3f thisCol;
				thisCol = mesh_list[current_mesh].normal(fa);
				float col_scale_factor = 1.0 / max_mean_curvture;
				if (showMeanCurvature) {


					for (MyMesh::FaceVertexIter fvi = mesh_list[current_mesh].fv_iter(fa); fvi; ++fvi) {
						OpenMesh::Vec3f thisCoord;
						OpenMesh::Vec3f thisCol;
						thisCoord = mesh_list[current_mesh].point(fvi.handle());

						float thisH = mesh_list[current_mesh].property(mean_curvature, OpenMesh::VertexHandle(fvi.handle()));
						float thisK = mesh_list[current_mesh].property(gauss_curvature, OpenMesh::VertexHandle(fvi.handle()));
						//float thisK1 = mesh_list[current_mesh].property(k1, OpenMesh::VertexHandle(fvi.handle()));
						//float thisK2 = mesh_list[current_mesh].property(k2, OpenMesh::VertexHandle(fvi.handle()));


						//thisCol = simpleHColourMap(thisH);
						thisCol = simpleColourMap(thisH, max_mean_curvture, min_mean_curvture);
						glColor3f(thisCol[0], thisCol[1], thisCol[2]);

						//gauss curvature
						//glColor3f(thisK, thisK, thisK);

						glVertex3f(thisCoord[0], thisCoord[1], thisCoord[2]);

						//std::cout << mesh_list[current_mesh].property(mean_curvature, OpenMesh::VertexHandle(fvi.handle())) << "\n";

					}
				}
				else if (showGaussCurvature) {

					for (MyMesh::FaceVertexIter fvi = mesh_list[current_mesh].fv_iter(fa); fvi; ++fvi) {
						OpenMesh::Vec3f thisCoord;
						OpenMesh::Vec3f thisCol;
						thisCoord = mesh_list[current_mesh].point(fvi.handle());

						//OpenMesh::Vec4f thisCol;
						//thisCol = mesh_list[current_mesh].color(fvi.handle());

						float thisK = mesh_list[current_mesh].property(gauss_curvature, OpenMesh::VertexHandle(fvi.handle()));
						float thisK1 = mesh_list[current_mesh].property(k1, OpenMesh::VertexHandle(fvi.handle()));
						float thisK2 = mesh_list[current_mesh].property(k2, OpenMesh::VertexHandle(fvi.handle()));

						thisCol = simpleColourMap(thisK, max_gauss_curvture, min_gauss_curvture);

						//thisCol = simpleHColourMap(thisH);
						glColor3f(thisCol[0], thisCol[1], thisCol[2]);

						//gauss curvature
						//glColor3f(thisK, thisK, thisK);

						glVertex3f(thisCoord[0], thisCoord[1], thisCoord[2]);

						//std::cout << mesh_list[current_mesh].property(mean_curvature, OpenMesh::VertexHandle(fvi.handle())) << "\n";

					}
				}
				else if (showPrincipleCurvature) {
					for (MyMesh::FaceVertexIter fvi = mesh_list[current_mesh].fv_iter(fa); fvi; ++fvi) {
						OpenMesh::Vec3f thisCoord;
						OpenMesh::Vec3f thisCol;
						thisCoord = mesh_list[current_mesh].point(fvi.handle());

						//OpenMesh::Vec4f thisCol;
						//thisCol = mesh_list[current_mesh].color(fvi.handle());

						float thisK1 = mesh_list[current_mesh].property(k1, OpenMesh::VertexHandle(fvi.handle()));
						float thisK2 = mesh_list[current_mesh].property(k2, OpenMesh::VertexHandle(fvi.handle()));

						//thisCol = simpleColourMap(thisK, max_gauss_curvture, min_gauss_curvture);
						thisCol = principleKColourmap(thisK1, thisK2);

						//thisCol = simpleHColourMap(thisH);
						glColor3f(thisCol[0], thisCol[1], thisCol[2]);

						//gauss curvature
						//glColor3f(thisK, thisK, thisK);

						glVertex3f(thisCoord[0], thisCoord[1], thisCoord[2]);
					}
				}
				else if (showSpectralWeight) {

					//bool showSpectralWeight = true;
					//int whichEigToDraw = 5;
					//Eigen::MatrixXd displacementValues;

					for (MyMesh::FaceVertexIter fvi = mesh_list[current_mesh].fv_iter(fa); fvi; ++fvi) {
						OpenMesh::Vec3f thisCoord;
						OpenMesh::Vec3f thisCol;
						thisCoord = mesh_list[current_mesh].point(fvi.handle());

						//OpenMesh::Vec4f thisCol;
						//thisCol = mesh_list[current_mesh].color(fvi.handle());
						
						thisCol[0] = displacementValues(fvi.handle().idx(), whichEigToDraw);
						thisCol[1] = displacementValues(fvi.handle().idx(), whichEigToDraw);
						thisCol[2] = displacementValues(fvi.handle().idx(), whichEigToDraw);

						thisCol = simpleColourMap(displacementValues(fvi.handle().idx(), whichEigToDraw), 1.0f, 0.0f);
						glColor3f(thisCol[0], thisCol[1], thisCol[2]);

						//gauss curvature
						//glColor3f(thisK, thisK, thisK);

						glVertex3f(thisCoord[0], thisCoord[1], thisCoord[2]);
					}
				}
				else {
					if (!smoothFaces) {
						if (!wireframe) {
							glColor3f(thisCol[0], thisCol[1], thisCol[2]);
						}
					}
					for (MyMesh::FaceVertexIter fvi = mesh_list[current_mesh].fv_iter(fa); fvi; ++fvi) {
						OpenMesh::Vec3f thisCoord;
						thisCoord = mesh_list[current_mesh].point(fvi.handle());
						if (smoothFaces && !wireframe) {
							//OpenMesh::Vec4f thisCol;
							//thisCol = mesh_list[current_mesh].color(fvi.handle());
							OpenMesh::Vec3f thisCol;
							thisCol = mesh_list[current_mesh].normal(fvi.handle());
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

	// Set the orthographic viewing transformation	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, window_w, window_h, 0, -1, 1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	DrawGUI();

	display_graphs(width, height); // width and height are the dimensions of the full window
	
	

	//glFlush();
	glfwSwapBuffers(window);
	glfwPollEvents();

}

int main(void)
{


	//_______READ MESH________________________	

	std::string filename_03 = "dragon_vrip_res4.ply";
	std::string filename_04 = "bun_zipper.ply";
	std::string filename_05 = "bun_zipper_res4.ply";
	std::string filename_06 = "smooth_test.obj"; 
	std::string filename_07 = "dragon_lowres.ply";	// Dragon model - worked well with the smoothing
	std::string filename_wolf = "Wolf2.obj";
	std::string filename_deer = "Deer.obj";
	std::string filename_cat = "cat.obj";
	//std::string filename_head = "head.obj";	// Head from turbo squid - about 6k verts
	std::string filename_dragon = "dragon_1500.obj";	// Stanford dragon model - low poly - about 1.5k verts
	std::string filename_dragon4k = "dragon_4000.obj";	// Stanford dragon model - low poly - about 4.0k verts
	std::string filename_dragon12k = "dragon_12k.obj";	// Stanford dragon model - low poly - about 12.0k verts
	std::string filename_armadillo4k = "Armadillo_4000.obj";	// Stanford armadillo model - low poly - about 4.0k verts	
	std::string filename_armadillo1k = "Armadillo_1000.obj";	// Stanford armadillo model - low poly - about 1.0k verts	
	std::string filename_head = "head2.obj";	// Head from turbo squid - about 0.7k verts
	std::string filename_goat = "Goat.obj";
	std::string filename_shark = "Shark.obj";	
	std::string filename_fish = "fish.obj";
	std::string filename_cube = "cube.obj";
	std::string filename_ox = "ox.obj"; //low poly, blocky model I made in blender
	std::string filename_ox1 = "ox_sd1.obj"; //subsurface division 1
	std::string filename_ox2= "ox_sd2.obj"; //subsurface division 2
	std::string filename_arnie = "ArnoldSchwarzeneggerBust.obj"; // from turbo squid
	std::string filename_man_head = "male_head.obj"; // from turbo squid
		
	std::vector<std::string> input_files;
	
	//input_files.push_back(filename_head);
	//input_files.push_back(filename_armadillo1k);
	
	input_files.push_back(filename_dragon12k);
										
	std::cout << "loading meshes...";

	unchangedMesh = MyMesh();
	if (!OpenMesh::IO::read_mesh(unchangedMesh, input_files[0]))
	{
		std::cerr << "read error\n";
		exit(1);
	}

	noisyMesh = MyMesh();
	if (!OpenMesh::IO::read_mesh(noisyMesh, input_files[0]))
	{
		std::cerr << "read error\n";
		exit(1);
	}

	for (int i = 0; i < input_files.size(); i++) {
		current_mesh = i;
		mesh_list.push_back(MyMesh());

		if (!OpenMesh::IO::read_mesh(mesh_list[current_mesh], input_files[i]))
		{
			std::cerr << "read error\n";
			exit(1);
		}
	}
	//_______READ MESH________________________


	std::cout << " meshes loaded\n\n";

	buttonList.push_back(Button{ 5, 5, 100, 25, 0, 0, "Shading", "Shading", ToggleShading });
	buttonList.push_back(Button{ 105, 5, 100, 25, 0, 0, "Show H", "Show H", ToggleShowMeanCurvature });
	buttonList.push_back(Button{ 205, 5, 100, 25, 0, 0, "Wireframe", "Wireframe", ToggleWireframe });
	buttonList.push_back(Button{ 305, 5, 100, 25, 0, 0, "Pointcloud", "Pointcloud", TogglePointCloud });
	buttonList.push_back(Button{ 405, 5, 100, 25, 0, 0, "Show E weight", "Show E weight", ToggleShowSpectralWeighting });
	buttonList.push_back(Button{ 505, 5, 100, 25, 0, 0, "Show K1 K2", "Show K1 K2", ToggleShowPrincipleCurvature });

	//Eigen decomposition buttons:

	textBoxList.push_back(TextBox{5, 50, 50, 25, "Eigen Decomposition"});

	buttonList.push_back(Button{ 25, 75, 50, 25, 0, 0, "+N Evec", "Less", IncreaseNEigenVectors });
	buttonList.push_back(Button{ 25, 100, 50, 25, 0, 0, "Find", "Find", DoEigenDecomposition });
	buttonList.push_back(Button{ 25, 125, 50, 25, 0, 0, "-N Evec", "More", DecreaseNEigenVectors });
		
	//Eigen reconstruction
	textBoxList.push_back(TextBox{ 5, 160, 50, 25, "Eigen Reconstruction" });

	buttonList.push_back(Button{ 25, 185, 50, 25, 0, 0, "+N Evec", "Less", IncreaseNShownEigenVectors });
	buttonList.push_back(Button{ 25, 210, 50, 25, 0, 0, "Find", "Find", DoEigenReconstruciton});
	buttonList.push_back(Button{ 25, 235, 50, 25, 0, 0, "-N Evec", "More", DecreaseNShownEigenVectors });

	
	//Edit the spectral coeffs:
	textBoxList.push_back(TextBox{ 5, 270, 50, 25, "Edit Spectral Coeffs" });

	buttonList.push_back(Button{ 5, 295, 50, 25, 0, 0, "Next", "Next", IncreaseSelectedEigenVector }); //
	buttonList.push_back(Button{ 55, 295, 50, 25, 0, 0, "Increase", "Increase", IncreaseSpectralCoeff }); //
	buttonList.push_back(Button{ 5, 320, 100, 25, 0, 0, "Show", "Show", ToggleShowSpectralWeighting }); //
	buttonList.push_back(Button{ 5, 345, 50, 25, 0, 0, "Prev", "Prev", DecreaseSelectedEigenVector }); //
	buttonList.push_back(Button{ 55, 345, 50, 25, 0, 0, "Decrease", "Decrease", DecreaseSpectralCoeff }); //
	
	
	buttonList.push_back(Button{ 5, 375, 100, 25, 0, 0, "Reset", "Reset", Reset }); //
	
	
	

	std::cout << "|-----------------------------|\n";
	std::cout << "|                             |\n";
	std::cout << "|        Seb's Meshviewer     |\n";
	std::cout << "|                             |\n";
	std::cout << "|-----------------------------|\n";

	std::cout << "eigenReconstruction: press k\n";
	current_mesh = 0;

	// Add mesh curvatures as mesh properties
	mesh_list[current_mesh].add_property(mean_curvature, "mean_curvature");
	mesh_list[current_mesh].property(mean_curvature).set_persistent(true);

	mesh_list[current_mesh].add_property(gauss_curvature, "gauss_curvature");
	mesh_list[current_mesh].property(gauss_curvature).set_persistent(true);

	mesh_list[current_mesh].add_property(k1, "k1");
	mesh_list[current_mesh].property(k1).set_persistent(true);

	mesh_list[current_mesh].add_property(k2, "k2");
	mesh_list[current_mesh].property(k2).set_persistent(true);

	// Initialise various functions so I can view and draw the meshes
	findFaceNormals();
	findVertNormalsFromFaces();	
	findGaussCurvature2();		
	globalScale += 5;
	global_translation = getCentreOfMesh(); //get centre
	global_translation *= -6.0f;

	// The help function - remind user it exists
	std::cout << "Press H for help.\n";

	//set a seed for the random number gen - this was primarily so the meshes would be the same colour each time for ICP
	srand(1);

	//Create the window:
	glfwSetErrorCallback(error_callback);
	GLFWwindow* window;
	
	//DenoiseTest("z_uniform_denoise_k300_dragon_4k.txt", 300, uniform);		//		--------- TEST-------------------
	//DoTest("z_uniform_k300_armadillio_4k.txt", 300, uniform);
	min_mean_curvture = -900.0;
	max_mean_curvture = 600.0;

	if (!glfwInit())
		return 1;

	if (!glfwInit())
		exit(EXIT_FAILURE);

	//window = glfwCreateWindow(640, 520, "Seb's Coursework 3 - Uniform Laplace", NULL, NULL);
	//window = glfwCreateWindow(640, 520, "Seb's Coursework 3 - Laplace Beltrami", NULL, NULL);
	window = glfwCreateWindow(640, 520, "Seb's Coursework 3 - Uniform", NULL, NULL);

	if (!window)
	{
		glfwTerminate();
		exit(EXIT_FAILURE);
	}

	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	glfwSetKeyCallback(window, key_callback);
	glfwSetMouseButtonCallback(window, mouse_callback);
	glfwSetCursorPosCallback(window, mouse_moved_callback);


	while (!glfwWindowShouldClose(window))
	{
		display(window);
				
		glfwGetWindowSize(window, &window_w, &window_h);
		
	}

	glfwDestroyWindow(window);
	
	glfwTerminate();
	exit(EXIT_SUCCESS);
}





