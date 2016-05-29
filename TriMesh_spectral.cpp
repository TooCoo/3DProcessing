//
// Created by Ben Eisner on 5/25/16.
//

#include <iostream>

// Spectra
#include <spectra/include/GenEigsSolver.h>
#include <spectra/include/MatOp/SparseGenMatProd.h>

#include "trimesh.h"

using Eigen::MatrixXd;

void TriMesh::eigenReconstruction(double lambda, int nLargestEigs) {
    //iterate over all vertices
    BaseMesh::VertexIter vlt, vBegin, vEnd;
    vBegin = baseMesh.vertices_begin();
    vEnd = baseMesh.vertices_end();
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
        OpenMesh::VertexHandle vh = *vlt;

        int i = vh.idx();
        int n_neighbours = 0;

        XnMat(i, 0) = baseMesh.point(vh)[0];
        YnMat(i, 0) = baseMesh.point(vh)[1];
        ZnMat(i, 0) = baseMesh.point(vh)[2];

        //now iterate over adjacent vertices
        for (BaseMesh::ConstVertexVertexIter vvi = baseMesh.vv_iter(vh); vvi.is_valid(); ++vvi) {
            OpenMesh::VertexHandle vh2 = *vvi;
            n_neighbours++;
        }

        double valence_normalisation = 1.0 / double(n_neighbours);

        for (BaseMesh::ConstVertexVertexIter vvi = baseMesh.vv_iter(vh); vvi.is_valid(); ++vvi) {
            OpenMesh::VertexHandle vh2 = *vvi;
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

        newCoord[0] = Xn1Mat(vlt->idx(), 0);
        newCoord[1] = Yn1Mat(vlt->idx(), 0);
        newCoord[2] = Zn1Mat(vlt->idx(), 0);

        baseMesh.set_point(*vlt, newCoord);

    }

    // find new normals for colours
    findFaceNormals();
    findVertexNormalsFromFaces();

    std::cout << "done.\n";
}

void TriMesh::findEigenVectors(int nLargestEigs) {
    std::cout << "Finding Eigen Vectors... ";

    double lambda = -1.0;

    //iterate over all vertices
    BaseMesh::VertexIter vlt, vBegin, vEnd;
    vBegin = baseMesh.vertices_begin();
    vEnd = baseMesh.vertices_end();
    int N = 0;
    for (vlt = vBegin; vlt != vEnd; ++vlt) {
        N++;
    }

    //construct A matrix - discrete laplacian
    //A = I - lambda L

    Eigen::SparseMatrix<double> L = Eigen::SparseMatrix<double>(N, N);

    //fill matrix and vector
    for (vlt = vBegin; vlt != vEnd; ++vlt) {
        OpenMesh::VertexHandle vh = *vlt;

        int i = vh.idx();
        int n_neighbours = 0;

        //now iterate over adjacent vertices
        for (BaseMesh::ConstVertexVertexIter vvi = baseMesh.vv_iter(vh); vvi.is_valid(); ++vvi) {
            OpenMesh::VertexHandle vh2 = *vvi;
            n_neighbours++;
        }

        double valence_normalisation = 1.0 / double(n_neighbours);

        for (BaseMesh::ConstVertexVertexIter vvi = baseMesh.vv_iter(vh); vvi.is_valid(); ++vvi) {
            OpenMesh::VertexHandle vh2 = *vvi;
            //n_neighbours++;
            int j = vh2.idx();
            L.insert(i, j) = valence_normalisation * lambda * 1.0;
        }
        L.insert(i, i) = 1.0 - 1.0*lambda;

    }

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

    eigs.init();
    int nconv = eigs.compute();

    // Retrieve results
    Eigen::VectorXcd evalues;
    //Eigen::MatrixXd evecs; - now initialised as a global variable
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

    evecs_coeffs = MatrixXd::Ones(nLargestEigs, 1);
}

void TriMesh::remakeFromEVecs(int nLargestEigs, TriMesh *unchangedMesh) {
    //iterate over all vertices
    BaseMesh::VertexIter vlt, vBegin, vEnd;
    vBegin = unchangedMesh->baseMesh.vertices_begin();
    vEnd = unchangedMesh->baseMesh.vertices_end();

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
        OpenMesh::VertexHandle vh = *vlt;

        int i = vh.idx();
        int n_neighbours = 0;

        XnMat(i, 0) = unchangedMesh->baseMesh.point(vh)[0];
        YnMat(i, 0) = unchangedMesh->baseMesh.point(vh)[1];
        ZnMat(i, 0) = unchangedMesh->baseMesh.point(vh)[2];

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

        Xn1MatOutput += evecs_coeffs(i, 0) * (thisEigenVec.transpose() * XnMat)(0, 0) * thisEigenVec;
        Yn1MatOutput += evecs_coeffs(i, 0) * (thisEigenVec.transpose() * YnMat)(0, 0) * thisEigenVec;
        Zn1MatOutput += evecs_coeffs(i, 0) * (thisEigenVec.transpose() * ZnMat)(0, 0) * thisEigenVec;

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

    //save new coords - update
    for (vlt = vBegin; vlt != vEnd; ++vlt) {

        OpenMesh::Vec3f newCoord;

        newCoord[0] = Xn1MatOutput(vlt->idx(), 0);
        newCoord[1] = Yn1MatOutput(vlt->idx(), 0);
        newCoord[2] = Zn1MatOutput(vlt->idx(), 0);

        baseMesh.set_point(*vlt, newCoord);

    }
    // find new normals for colours
    findFaceNormals();
    findVertexNormalsFromFaces();
}

