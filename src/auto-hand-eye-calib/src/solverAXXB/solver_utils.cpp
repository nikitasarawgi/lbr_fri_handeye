#include "solver_utils.h"
#include <unsupported/Eigen/MatrixFunctions>

Eigen::Matrix3d skew(Eigen::Vector3d v)
{
   Eigen::Matrix3d S;
    S <<  0,   -v.z(),  v.y(),
         v.z(),  0,   -v.x(),
        -v.y(), v.x(),  0;
    return S;
}

// Compute the matrix logarithm
Eigen::Matrix3d logm(const Eigen::Matrix3d& M) {
    // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(M);
    // Eigen::Matrix3d D = eigensolver.eigenvalues().array().log().matrix().asDiagonal();
    // Eigen::Matrix3d V = eigensolver.eigenvectors();
    // return V * D * V.inverse();
    return M.log();
}
