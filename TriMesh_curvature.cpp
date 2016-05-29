//
// Created by Ben Eisner on 5/25/16.
//

#include <iostream>
#include "trimesh.h"

extern CurvatureProperties curveProps;

#define PI 3.14159265

void TriMesh::uniformLaplaceDiscretization() {
    BaseMesh::VertexIter vlt, vBegin, vEnd;
    vBegin = baseMesh.vertices_begin();
    vEnd = baseMesh.vertices_end();

    for (vlt = vBegin; vlt != vEnd; ++vlt) {
        OpenMesh::VertexHandle vh = *vlt;
        //find the adjecent verts

        OpenMesh::Vec3f thisNormal = baseMesh.normal(vh);
        OpenMesh::Vec3f xi = baseMesh.point(vh);
        OpenMesh::Vec3f xj = OpenMesh::Vec3f(0.0f, 0.0f, 0.0f);
        OpenMesh::Vec3f laplace = OpenMesh::Vec3f(0.0f, 0.0f, 0.0f);
        int n_adjacent = 0;

        for (BaseMesh::VertexVertexIter vvi = baseMesh.vv_iter(vh); vvi.is_valid(); ++vvi) {

            n_adjacent++;
            xj = baseMesh.point(*vvi);
            laplace += (xj - xi);
        }

        //normalise
        laplace *= 1.0f / float(n_adjacent);

        laplace *= -0.5;

        float h = laplace.norm();

        //need to check if h is negative
        laplace.normalize();
        if ((laplace + thisNormal).norm() < 1.0f) {
            h *= -1.0f;
        }

        baseMesh.property(curveProps.mean_curvature, vh) = h;

        if (h > curveProps.max_mean_curvature) curveProps.max_mean_curvature = h;
        if (h < curveProps.min_mean_curvature) curveProps.min_mean_curvature = h;

    }

    std::cout << "max H: " << curveProps.max_mean_curvature << "\n";
    std::cout << "min H: " << curveProps.min_mean_curvature << "\n";
    //system("PAUSE");
}

void TriMesh::discreteLaplaceDiscretization() {
    //iterate over all vertices
    BaseMesh::VertexIter vlt, vBegin, vEnd;
    vBegin = baseMesh.vertices_begin();
    vEnd = baseMesh.vertices_end();

    for (vlt = vBegin; vlt != vEnd; ++vlt) {

        OpenMesh::VertexHandle vh = *vlt;

        OpenMesh::Vec3f xi = baseMesh.point(vh);

        std::vector<OpenMesh::Vec3f> xj;

        OpenMesh::Vec3f unnormalisedS = OpenMesh::Vec3f(0.0f, 0.0f, 0.0f);
        float area = 0.0f;

        //now iterate over adjacent vertices
        for (BaseMesh::ConstVertexVertexIter vvi = baseMesh.vv_iter(vh); vvi.is_valid(); ++vvi) {
            xj.push_back(baseMesh.point(*vvi));
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

        //now normalise S by area
        area *= 2.0f;

        float h = unnormalisedS.norm();
        h *= (1.0f / area);

        h *= 0.5;

        baseMesh.property(curveProps.mean_curvature, vh) = h;

        if (h > curveProps.max_mean_curvature) curveProps.max_mean_curvature = h;
        if (h < curveProps.min_mean_curvature) curveProps.min_mean_curvature = h;

    }

    std::cout << "max H: " << curveProps.max_mean_curvature << "\n";
    std::cout << "min H: " << curveProps.min_mean_curvature << "\n";

}

void TriMesh::findGaussianCurvature2() {
    //iterate over all vertices
    BaseMesh::VertexIter vlt, vBegin, vEnd;
    vBegin = baseMesh.vertices_begin();
    vEnd = baseMesh.vertices_end();

    for (vlt = vBegin; vlt != vEnd; ++vlt) {

        OpenMesh::VertexHandle vh = *vlt;

        OpenMesh::Vec3f xi = baseMesh.point(vh);

        std::vector<OpenMesh::Vec3f> xj;

        OpenMesh::Vec3f unnormalisedS = OpenMesh::Vec3f(0.0, 0.0, 0.0);
        double area = 0.0;

        double angle_deficit = 0.0;

        //now iterate over adjacent vertices
        for (BaseMesh::ConstVertexVertexIter vvi = baseMesh.vv_iter(vh); vvi.is_valid(); ++vvi) {
            xj.push_back(baseMesh.point(*vvi));
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
        OpenMesh::Vec3f normalVariation = (unnormalisedS.normalized() - baseMesh.normal(vh));// .norm();

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

            baseMesh.property(curveProps.k1, *vlt) = h + sqrt(inside_sqrt);
            baseMesh.property(curveProps.k2, *vlt) = h - sqrt(inside_sqrt);

        }
        else {
            std::cout << "error: in find gauss curvature2: negative sqrt or less than 3 neighbouring vertices setting k1 and k2 to zero\n";
            h = 0.0;
            K = 0.0;
            baseMesh.property(curveProps.k1, *vlt) = 0.0;
            baseMesh.property(curveProps.k2, *vlt) = 0.0;
        }

        baseMesh.property(curveProps.mean_curvature, vh) = h;
        baseMesh.property(curveProps.gauss_curvature, *vlt) = K;

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
    vBegin = baseMesh.vertices_begin();
    vEnd = baseMesh.vertices_end();

    for (vlt = vBegin; vlt != vEnd; ++vlt) {

        OpenMesh::Vec3f newCoord;
        newCoord = baseMesh.point(*vlt);
        double h = baseMesh.property(curveProps.mean_curvature, OpenMesh::VertexHandle(*vlt));
        newCoord += lambda * h * baseMesh.normal(*vlt);
        baseMesh.set_point(*vlt, newCoord);

    }
}

void TriMesh::implicitLaplacianMeshSmoothing(double lambda) {
    BaseMesh::VertexIter vlt, vBegin, vEnd;
    vBegin = baseMesh.vertices_begin();
    vEnd = baseMesh.vertices_end();
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

        Xn(i) = baseMesh.point(vh)[0];
        Yn(i) = baseMesh.point(vh)[1];
        Zn(i) = baseMesh.point(vh)[2];

        //now iterate over adjacent vertices
        for (BaseMesh::ConstVertexVertexIter vvi = baseMesh.vv_iter(vh); vvi.is_valid(); ++vvi) {
            OpenMesh::VertexHandle vh2 = *vvi;
            n_neighbours++;
            //int j = vh2.idx();
            //L.insert(i, j) = lambda * 1.0;
        }
        double valence_normalisation = 1.0 / double(n_neighbours);

        for (BaseMesh::ConstVertexVertexIter vvi = baseMesh.vv_iter(vh); vvi.is_valid(); ++vvi) {
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

        baseMesh.set_point(*vlt, newCoord);

    }
}

void TriMesh::findGaussianCurvature() {
    int wrongCurves = 0;
    int n_curves = 0;

    BaseMesh::VertexIter vlt, vBegin, vEnd;
    vBegin = baseMesh.vertices_begin();
    vEnd = baseMesh.vertices_end();

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
        for (BaseMesh::ConstVertexFaceIter vfi = baseMesh.vf_iter(*vlt); vfi.is_valid(); ++vfi) {
            n_adjacent_faces++;
            OpenMesh::Vec3f core_vert = baseMesh.point(*vlt);

            //Should be three verts
            OpenMesh::Vec3f v1;
            OpenMesh::Vec3f v2;
            OpenMesh::Vec3f v3;

            int vert_count = 0;

            //I need coords of opposite vertecies
            for (BaseMesh::FaceVertexIter fvi = baseMesh.fv_iter(*vfi); fvi.is_valid(); ++fvi) {

                vert_count++;

                switch (vert_count) {
                    case 1:
                        v1 = baseMesh.point(*fvi);
                        break;
                    case 2:
                        v2 = baseMesh.point(*fvi);
                        break;
                    case 3:
                        v3 = baseMesh.point(*fvi);
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

            float H = baseMesh.property(curveProps.mean_curvature, *vlt);
            float inside_sqrt = H*H - K;


            n_curves++;

            if (inside_sqrt < 0.0) {
                wrongCurves++;
                //cover the thing
                baseMesh.property(curveProps.gauss_curvature, *vlt) = 0.0;
                baseMesh.property(curveProps.k1, *vlt) = 0.0;
                baseMesh.property(curveProps.k2, *vlt) = 0.0;
                inside_sqrt *= -1.0f;
            }
            //else {

            float thisk1 = H + sqrt(inside_sqrt);
            float thisk2 = H - sqrt(inside_sqrt);
            baseMesh.property(curveProps.gauss_curvature, *vlt) = K;
            baseMesh.property(curveProps.k1, *vlt) = thisk1;
            baseMesh.property(curveProps.k2, *vlt) = thisk2;

            if (K > curveProps.max_gauss_curvature) curveProps.max_gauss_curvature = K;
            if (K < curveProps.min_gauss_curvature) curveProps.min_gauss_curvature = K;
            if (thisk1 > curveProps.max_k1_curvature) curveProps.max_k1_curvature = thisk1;
            if (thisk2 < curveProps.min_k2_curvature) curveProps.min_k2_curvature = thisk2;
            //}

        }

        //std::cout << "min angle: \t" << min_angle << "\tmax angle: \t" << max_angle << "\n";

    }

    std::cout << "max k : " << curveProps.max_gauss_curvature << "\n";
    std::cout << "min k : " << curveProps.min_gauss_curvature << "\n";
    std::cout << "max k1: " << curveProps.max_k1_curvature << "\n";
    std::cout << "min k2: " << curveProps.min_k2_curvature << "\n";
    std::cout << "ratio of bad curvatures: " << wrongCurves / n_curves << "\n";
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