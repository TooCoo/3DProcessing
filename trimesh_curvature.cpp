//
// Created by Ben Eisner on 5/25/16.
//

#include <iostream>
#include "trimesh.h"

extern CurvatureProperties curveProps;
extern OpenMesh::VPropHandleT<double>       mean_curvature;
extern OpenMesh::VPropHandleT<double>       gauss_curvature;
extern OpenMesh::VPropHandleT<double>       k1;
extern OpenMesh::VPropHandleT<double>       k2;

#define PI 3.14159265

void TriMesh::findGaussianCurvature() {
    //iterate over all vertices
    BaseMesh::VertexIter vlt, vBegin, vEnd;
    vBegin = mesh.vertices_begin();
    vEnd = mesh.vertices_end();

    for (vlt = vBegin; vlt != vEnd; ++vlt) {

        OpenMesh::VertexHandle vh = *vlt;

        OpenMesh::Vec3f xi = mesh.point(vh);

        std::vector<OpenMesh::Vec3f> xj;

        OpenMesh::Vec3f unnormalisedS = OpenMesh::Vec3f(0.0, 0.0, 0.0);
        double area = 0.0;

        double angle_deficit = 0.0;

        //now iterate over adjacent vertices
        for (BaseMesh::ConstVertexVertexIter vvi = mesh.vv_iter(vh); vvi.is_valid(); ++vvi) {
            xj.push_back(mesh.point(*vvi));
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
        OpenMesh::Vec3f normalVariation = (unnormalisedS.normalized() - mesh.normal(vh));// .norm();

        if (normalVariation.norm() < 0.5f) {
            h *= -1.0f;
        }

        //std::cout << h << "\n";

        //K = (2*pi - sum(angle))/Area
        double K = ((2.0f*PI) - angle_deficit) / area;

        if ((area == 0.0f) || (h == INFINITY)) {
            h = 0.0f;
            K = 0.0f;
        }

        double inside_sqrt = (h*h - K);

        if ((inside_sqrt >= 0.0) && (xj.size() > 2)) {

            double thisk1 = h + sqrt(inside_sqrt);
            double thisk2 = h - sqrt(inside_sqrt);

            mesh.property(k1, *vlt) = h + sqrt(inside_sqrt);
            mesh.property(k2, *vlt) = h - sqrt(inside_sqrt);

        }
        else {
            std::cout << "error: in find gauss curvature2: negative sqrt or less than 3 neighbouring vertices setting k1 and k2 to zero\n";
            h = 0.0;
            K = 0.0;
            mesh.property(k1, *vlt) = 0.0;
            mesh.property(k2, *vlt) = 0.0;
        }

        mesh.property(mean_curvature, vh) = h;
        mesh.property(gauss_curvature, *vlt) = K;

        if (h > curveProps.max_mean_curvature) curveProps.max_mean_curvature = h;
        if (h < curveProps.min_mean_curvature) curveProps.min_mean_curvature = h;
        if (K > curveProps.max_gauss_curvature) curveProps.max_gauss_curvature = K;
        if (K < curveProps.min_gauss_curvature) curveProps.min_gauss_curvature = K;
        //if (thisk1 > max_k1_curvature) max_k1_curvature = thisk1;
        //if (thisk2 < min_k2_curvature) min_k2_curvature = thisk2;
    }

    std::cout << "max H: " << curveProps.max_mean_curvature << "\n";
    std::cout << "min H: " << curveProps.min_mean_curvature << "\n";
    std::cout << "max K: " << curveProps.max_gauss_curvature << "\n";
    std::cout << "min K: " << curveProps.min_gauss_curvature << "\n";
}

void TriMesh::applyDiffusionFlow(double lambda) {
    BaseMesh::VertexIter vlt, vBegin, vEnd;
    vBegin = mesh.vertices_begin();
    vEnd = mesh.vertices_end();

    for (vlt = vBegin; vlt != vEnd; ++vlt) {

        OpenMesh::Vec3f newCoord;
        newCoord = mesh.point(*vlt);
        double h = mesh.property(mean_curvature, OpenMesh::VertexHandle(*vlt));
        newCoord += lambda * h * mesh.normal(*vlt);
        mesh.set_point(*vlt, newCoord);

    }
}

void TriMesh::implicitLaplacianMeshSmoothing(double lambda) {
    BaseMesh::VertexIter vlt, vBegin, vEnd;
    vBegin = mesh.vertices_begin();
    vEnd = mesh.vertices_end();
    int N = 0;
    for (vlt = vBegin; vlt != vEnd; ++vlt) {
        N++;
    }

    //construct A matrix
    //A = I - lambda L

    Eigen::SparseMatrix<double> L = Eigen::SparseMatrix<double>(N, N);
    //Eigen::VectorXd Xn(N);// = Eigen::VectorXd::Zero(1, N);
    Eigen::VectorXd Xn(N), Yn(N), Zn(N), Xn1(N), Yn1(N), Zn1(N);

    //fill matrix and vector

    for (vlt = vBegin; vlt != vEnd; ++vlt) {
        OpenMesh::VertexHandle vh = *vlt;

        int i = vh.idx();
        int n_neighbours = 0;

        Xn(i) = mesh.point(vh)[0];
        Yn(i) = mesh.point(vh)[1];
        Zn(i) = mesh.point(vh)[2];

        //now iterate over adjacent vertices
        for (BaseMesh::ConstVertexVertexIter vvi = mesh.vv_iter(vh); vvi.is_valid(); ++vvi) {
            OpenMesh::VertexHandle vh2 = *vvi;
            n_neighbours++;
            //int j = vh2.idx();
            //L.insert(i, j) = lambda * 1.0;
        }
        double valence_normalisation = 1.0 / double(n_neighbours);

        for (BaseMesh::ConstVertexVertexIter vvi = mesh.vv_iter(vh); vvi.is_valid(); ++vvi) {
            OpenMesh::VertexHandle vh2 = *vvi;
            //n_neighbours++;
            int j = vh2.idx();
            L.insert(i, j) = valence_normalisation * lambda * 1.0;
        }
        L.insert(i, i) = 1 - 1.0*lambda;

    }

    //I should now have my matrices
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper> cg;
    cg.compute(L);
    Xn1 = cg.solve(Xn);
    std::cout << "#iterations:     " << cg.iterations() << std::endl;
    std::cout << "estimated error: " << cg.error() << std::endl;

    Yn1 = cg.solve(Yn);
    std::cout << "#iterations:     " << cg.iterations() << std::endl;
    std::cout << "estimated error: " << cg.error() << std::endl;

    Zn1 = cg.solve(Zn);
    std::cout << "#iterations:     " << cg.iterations() << std::endl;
    std::cout << "estimated error: " << cg.error() << std::endl;

    //Should have my new locations

    for (vlt = vBegin; vlt != vEnd; ++vlt) {

        OpenMesh::Vec3f newCoord;

        newCoord[0] = Xn1(vlt->idx());
        newCoord[1] = Yn1(vlt->idx());
        newCoord[2] = Zn1(vlt->idx());

        mesh.set_point(*vlt, newCoord);

    }
}

OpenMesh::Vec3f TriMesh::simpleColourMap(float value, float max, float min) {
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

OpenMesh::Vec3f TriMesh::principleKColourmap(double k1, double k2) {

    OpenMesh::Vec3f colour = OpenMesh::Vec3f(1.0f, 1.0f, 1.0f);
    //draw curvatures, allow for a threshold so that we can see umbillic and parabollic

    double umbillic = (k1 - k2) * (k1 - k2);
    double umbillic_threshold = 10.0;
    if (umbillic < umbillic_threshold) {
        // umbillic point
        colour[0] = 1.0f;
        colour[1] = 1.0f;
        colour[2] = 1.0f;
    }
        //else if(k1*k2 == 0.0){
    else if (k1 * k1 < umbillic_threshold || k2 * k2 < umbillic_threshold) {
        //parabollic
        colour[0] = 0.0f;
        colour[1] = 1.0f;
        colour[2] = 0.0f;
    }

    else if (k1 * k2 > 0.0) {
        //elliptic point
        colour[0] = 1.0f;
        colour[1] = 0.0f;
        colour[2] = 0.0f;
    }
    else if (k1 * k2 < 0.0) {
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