#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "tsaiaxxb.h"
#include "../solver_utils.h"

Eigen::Matrix4d TsaiAXXB::SolveX()
{
    int n = A_.size();

    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(3*n, 3);
    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(3*n, 1);

    for(int i = 0; i < n; i++){
        Eigen::Matrix3d A1 = logm(A_[i].block<3,3>(0, 0));
        Eigen::Matrix3d B1 = logm(B_[i].block<3,3>(0, 0));
        Eigen::Vector3d a = A1.diagonal().normalized();
        Eigen::Vector3d b = B1.diagonal().normalized();
        S.block<3,3>(3*i, 0) = skew(a + b);
        V.block<3,1>(3*i, 0) = a - b;
    }
    Eigen::Vector3d x = S.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(V);
    double theta = 2*atan(x.norm());
    x = x / x.norm();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * cos(theta) + sin(theta) * skew(x) + (1 - cos(theta)) * x * x.transpose();

    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3*n, 3);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(3*n, 1);
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    for(int i = 0; i < n; i++){
        C.block<3,3>(3*i, 0) = I - A_[i].block<3,3>(0, 0);
        D.block<3,1>(3*i, 0) = A_[i].block<3,1>(0, 3) - R * B_[i].block<3,1>(0, 3);
    }
    Eigen::Vector3d t = C.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(D);
    Eigen::Matrix4d X = Eigen::Matrix4d::Identity();
    X.block<3,3>(0,0) = R;
    X.block<3,1>(0,3) = t;
    return X;
}

