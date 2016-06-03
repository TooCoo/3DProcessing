/*
 * Spectral Mesh Processing
 * Algorithms in this file implemented by Ben Eisner
 */
// standard library
#include <iostream>

// Spectra
#include <spectra/include/GenEigsSolver.h>
#include <spectra/include/MatOp/SparseGenMatProd.h>
#include <queue>

// local imports
#include "trimesh.h"

using Eigen::MatrixXd;

Eigen::SparseMatrix<double> TriMesh::computeUniformLaplacian() {
    double lambda = -1.0;
    int N = numVertices();
    Eigen::SparseMatrix<double> L = Eigen::SparseMatrix<double>(N, N);

    auto buildLaplacianRow = [&](OpenMesh::PolyConnectivity::VertexIter vlt) {
        int i = vlt->idx();
        int nNeighbors = numVertexNeighbors(vlt);
        double norm = 1.0/ (double)nNeighbors;

        // build each neighbor entry
        auto insertNeighborValue = [&](OpenMesh::PolyConnectivity::ConstVertexVertexIter vvi) {
            auto j = vvi->idx();
            L.insert(i, j) = norm * lambda * 1.0;
        };
        mapOverVertexNeighbors(vlt, insertNeighborValue);

        L.insert(i, i) = 1.0 * (1 - lambda);
    };

    // build each row
    mapOverVertices(buildLaplacianRow);
    laplacian = Uniform;
    return L;
}

double TriMesh::calculateBarycentricArea(OpenMesh::PolyConnectivity::VertexIter vlt) {
    double totalArea = 0.0;
    auto neighbors = getNeighbors(vlt);

    // central point
    auto v1 = mesh.point(*vlt);

    for (int i = 0; i < neighbors.size(); i++) {
        auto v2 = mesh.point(neighbors[i]);
        auto v3 = mesh.point(neighbors[(i + 1) % neighbors.size()]);

        auto a = (v2 - v1);
        auto b = (v3 - v1);

        // cross product is %
        totalArea += 0.5 * (a % b).norm();
    }

    return totalArea / 3.0;
}

Eigen::SparseMatrix<double> TriMesh::computeCotanLaplacian() {
    int N = numVertices();
    Eigen::SparseMatrix<double> L = Eigen::SparseMatrix<double>(N, N);

    std::vector<Eigen::Triplet<double>> entries;
    entries.reserve(N * 6);

    double avgArea = 0;
    auto averageFunc = [&](OpenMesh::PolyConnectivity::VertexIter vlt) {
        avgArea += calculateBarycentricArea(vlt) / (double) N;
    };
    mapOverVertices(averageFunc);

    auto weightFunc = [&](OpenMesh::PolyConnectivity::VertexIter vlt) {
        double normalizedArea = calculateBarycentricArea(vlt) / avgArea;
        double wi = 0.0;

        auto v1 = mesh.point(*vlt);
        auto neighbors = getNeighbors(vlt);

        for (int i = 0; i < neighbors.size(); i++) {
            auto v2 = mesh.point(neighbors[i]);
            auto v3 = mesh.point(neighbors[(i + 1) % neighbors.size()]);
            auto v4 = mesh.point(neighbors[(i + 2) % neighbors.size()]);

            auto a1 = v1 - v2;
            auto a2 = v3 - v2;

            auto b1 = v1 - v4;
            auto b2 = v3 - v4;

            // | is dot product...
            double alpha = std::acos((a1 | a2)/(a1.norm() * a2.norm()));
            double beta = std::acos((b1 | b2)/(b1.norm() * b2.norm()));

            double cotSumArea = (1.0/std::tan(alpha) + 1.0/std::tan(beta))/(2 * normalizedArea);
            // std::cout << "COTSUM: " << cotSumArea << std::endl;
            wi -= cotSumArea;
            // add each individual vertex
            entries.push_back(Eigen::Triplet<double>(vlt->idx(), neighbors[(i + 1) % neighbors.size()].idx(), cotSumArea));
        }

        // add the current vertex
        entries.push_back(Eigen::Triplet<double>(vlt->idx(), vlt->idx(), wi));
    };
    mapOverVertices(weightFunc);

    L.setFromTriplets(entries.begin(), entries.end());
    laplacian = Cotan;
    return L;

}

Eigen::MatrixXd TriMesh::exactlyDecomposeLaplacian(Eigen::SparseMatrix<double> L) {
    auto N = numVertices();

    this->evecs_coeffs = MatrixXd::Ones(N, 1);
    numEigsCalculated = N;

    Eigen::MatrixXd eigenVecs = Eigen::MatrixXd::Zero(N, N);

    // the solver only takes dense representations
    auto denseRepresentation = Eigen::MatrixXd(L);
    // construct and compute the eigenvectors and eigenvalues
    Eigen::EigenSolver<Eigen::MatrixXd> es(denseRepresentation);

    // because we only want the real parts of the eigenvectors
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++)
            // just take the real part
            eigenVecs(i, j) = es.eigenvectors().row(i)[j].real();
    }

    solver = Exact;
    evecs = eigenVecs;
    return eigenVecs;
}

MatrixXd TriMesh::approximatelyDecomposeLaplacian(int nSmallestEigs, Eigen::SparseMatrix<double> L) {
    auto N = numVertices();

    this->evecs_coeffs = MatrixXd::Ones(N, 1);
    numEigsCalculated = nSmallestEigs;

    // set up Spectra; ncv is a threshold for convergence between nSmallestEigs and N

     Spectra::SparseGenMatProd<double> op(L);
    int ncv = 2 * nSmallestEigs + 1;

    Spectra::GenEigsSolver<double, Spectra::SMALLEST_MAGN, Spectra::SparseGenMatProd<double>> eigs(&op, nSmallestEigs, ncv);

    eigs.init();
    int nconv = eigs.compute();

    // results
    Eigen::MatrixXd eigenVecs = Eigen::MatrixXd::Zero(N, N);

    auto tempEigs = eigs.eigenvectors();
    // we only want the real part
    if (eigs.info() == Spectra::SUCCESSFUL) {
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < nSmallestEigs; j++) {
                // we want to reverse the order of the eigenvectors, because the smallest eigenvalues are at the back
                eigenVecs(i, j) = tempEigs.real()(i, (nSmallestEigs - 1) - j);
            }
        }
    }
    solver = Approximate;
    evecs = eigenVecs;
    return eigenVecs;
}

void TriMesh::reconstructBySmallestEigenvalue(Eigen::MatrixXd eigenVecs, int nSmallestEigs) {
    int N = numVertices();

    // dim is the dimension of the mesh, usually 3
    Eigen::MatrixXd origPoints = Eigen::MatrixXd::Zero(N, dim); // N x 3
    Eigen::MatrixXd outputVerts = Eigen::MatrixXd::Zero(N, dim);

    // make a copy of each point
    auto copyFunc = [&](OpenMesh::PolyConnectivity::VertexIter vlt) {
        int i = vlt->idx();
        for (int j = 0; j < dim; j++)
            origPoints(i, j) = intermediateMesh.point(*vlt)[j];
    };
    mapOverVertices(copyFunc);

    // construct the resulting matrix one eigenvector at a time
    // X^(k) = SUM(e_i * e_i^T * X) from 0 -> k-1

    for (int i = 0; i < nSmallestEigs; i++) {
        auto currentEVec = eigenVecs.col(i); // N x 1
        // make sure the dimensions line up
        // [N x 1] * [1 * N] -> [N x N] ------> [N x N] * [N x 3] -> [N x 3]
        outputVerts += evecs_coeffs(i, 0) * (currentEVec * currentEVec.transpose()) * origPoints;
    }

    // update the points
    auto setFunc = [&](OpenMesh::PolyConnectivity::VertexIter vlt) {
        OpenMesh::Vec3f newCoord;
        // ugly, wish there was a better constructor...
        for (int j = 0; j < dim; j++)
            newCoord[j] = outputVerts(vlt->idx(), j);

        mesh.set_point(*vlt, newCoord);
    };
    mapOverVertices(setFunc);
    reconstruction = SmallestEigenvalues;

}

void TriMesh::reconstructByLargestSpectralCoefficient(Eigen::MatrixXd eigenVecs, int nLargestCoeffs) {
    int N = numVertices();

    // dim is the dimension of the mesh, usually 3
    Eigen::MatrixXd origPoints = Eigen::MatrixXd::Zero(N, dim); // N x 3
    Eigen::MatrixXd outputVerts = Eigen::MatrixXd::Zero(N, dim);

    // make a copy of each point
    auto copyFunc = [&](OpenMesh::PolyConnectivity::VertexIter vlt) {
        int i = vlt->idx();
        for (int j = 0; j < dim; j++)
            origPoints(i, j) = intermediateMesh.point(*vlt)[j];
    };
    mapOverVertices(copyFunc);

    // construct the resulting matrix one eigenvector at a time
    // X^(k) = SUM(e_i * e_i^T * X) from 0 -> k-1

    // three different dimensions of projections
    std::priority_queue<std::pair<double, int>> x_q;
    std::priority_queue<std::pair<double, int>> y_q;
    std::priority_queue<std::pair<double, int>> z_q;
    for (int i = 0; i < numEigsCalculated; i++) {
        auto currentEVec = eigenVecs.col(i);
        Eigen::MatrixXd x_hat = currentEVec.transpose() * origPoints;

        x_q.push(std::pair<double, int>(fabs(x_hat(0, 0)), i));
        y_q.push(std::pair<double, int>(fabs(x_hat(0, 1)), i));
        z_q.push(std::pair<double, int>(fabs(x_hat(0, 2)), i));
    }

    for (int i = 0; i < nLargestCoeffs; i++) {
        int x_ix = x_q.top().second;
        int y_ix = y_q.top().second;
        int z_ix = z_q.top().second;

        auto currentEXVec = eigenVecs.col(x_ix);
        Eigen::MatrixXd x_hat = currentEXVec.transpose() * origPoints;

        auto currentEYVec = eigenVecs.col(y_ix);
        Eigen::MatrixXd y_hat = currentEYVec.transpose() * origPoints;

        auto currentEZVec = eigenVecs.col(z_ix);
        Eigen::MatrixXd z_hat = currentEZVec.transpose() * origPoints;

        Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(N, dim);
        auto x_vec = evecs_coeffs(x_ix, 0) * eigenVecs.col(x_ix) * x_hat(0, 0);
        auto y_vec = evecs_coeffs(y_ix, 0) * eigenVecs.col(y_ix) * y_hat(0, 1);
        auto z_vec = evecs_coeffs(z_ix, 0) * eigenVecs.col(z_ix) * z_hat(0, 2);
        temp << x_vec, y_vec, z_vec;

        outputVerts += temp;

        x_q.pop();
        y_q.pop();
        z_q.pop();
    }


    // update the points
    auto setFunc = [&](OpenMesh::PolyConnectivity::VertexIter vlt) {
        OpenMesh::Vec3f newCoord;
        // ugly, wish there was a better constructor...
        for (int j = 0; j < dim; j++)
            newCoord[j] = outputVerts(vlt->idx(), j);

        mesh.set_point(*vlt, newCoord);
    };
    mapOverVertices(setFunc);
    reconstruction = LargestCoefficients;
}

// recompute eigenvectors based on past state
void TriMesh::recomputeEigenvectors(int nLargestEigs) {
    auto L = laplacian == Uniform ? computeUniformLaplacian() : computeCotanLaplacian();
    solver == Exact ? exactlyDecomposeLaplacian(L)
                    : approximatelyDecomposeLaplacian(nLargestEigs, L);
}

// reconstruct mesh based on past state
void TriMesh::reconstructMesh(int nLargestEigs) {
    reconstruction == SmallestEigenvalues ? reconstructBySmallestEigenvalue(evecs, nLargestEigs)
                                          : reconstructByLargestSpectralCoefficient(evecs, nLargestEigs);
}

void TriMesh::increaseEVecCoeff(int eig, double coeff) {
    evecs_coeffs(eig, 0) += coeff;
}

// get the squared distance
double TriMesh::ssd() {
    double ssd = 0.0;

    // get the squared distance from points after reconstruction
    auto ssdFunc = [&](OpenMesh::PolyConnectivity::VertexIter vlt) {
        auto diff = trulyUnchangedMesh.point(*vlt) - mesh.point(*vlt);
        ssd += diff.sqrnorm();
    };

    mapOverVertices(ssdFunc);
    return ssd;
}

