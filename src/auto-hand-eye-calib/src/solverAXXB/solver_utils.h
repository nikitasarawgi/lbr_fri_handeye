#ifndef SOLVER_UTILS_H
#define SOLVER_UTILS_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

Eigen::Matrix3d skew(Eigen::Vector3d u);
Eigen::Matrix3d logm(const Eigen::Matrix3d& M);

#endif // SOLVER_UTILS_H
