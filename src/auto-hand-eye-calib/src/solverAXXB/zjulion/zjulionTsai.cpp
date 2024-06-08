#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <iostream>
#include "zjulionTsai.h"
#include "../solver_utils.h"

Eigen::Matrix4d ZJulionTsai::SolveX()
{
    Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
    if (A_.size() != B_.size())
    {
        std::cout << "A and B must be same size.\n";
        return H;
    }

    const int n = A_.size();

    Eigen::Matrix3d R_a, R_b;
    Eigen::Vector3d r_a, r_b;

    Eigen::MatrixXd A(n*3, 3);
    Eigen::MatrixXd b(n*3, 1);
    A.setZero();
    b.setZero(); 

    for (int i = 0; i < n; ++i)
    {
        R_a = A_[i].block<3,3>(0,0);
        R_b = B_[i].block<3,3>(0,0);
        
        Eigen::Vector3d rod_a = R_a.eulerAngles(0, 1, 2);
        Eigen::Vector3d rod_b = R_b.eulerAngles(0, 1, 2);

        double theta_a = rod_a.norm();
        double theta_b = rod_b.norm();

        rod_a /= theta_a;
        rod_b /= theta_b;

        Eigen::Vector3d P_a = 2*sin(theta_a/2)*rod_a;
        Eigen::Vector3d P_b = 2*sin(theta_b/2)*rod_b;         

        Eigen::Matrix3d rot = skew(P_b + P_a);
        Eigen::Vector3d v = P_b - P_a;

        A.block<3,3>(3*i, 0) = rot;
        b.block<3,1>(3*i, 0) = v;
    }

    Eigen::MatrixXd pinA = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Eigen::MatrixXd::Identity(n*3, n*3)); 

    Eigen::Vector3d H_ba_prime = pinA * b;
     
    Eigen::Vector3d H_ba = 2 * H_ba_prime / sqrt(1 + H_ba_prime.squaredNorm());

    Eigen::Matrix3d R_ba = (1 - H_ba.squaredNorm() / 2) * Eigen::Matrix3d::Identity() 
        + 0.5 * (H_ba * H_ba.transpose() + sqrt(4 - H_ba.squaredNorm())*skew(H_ba));
    

    A.setZero();
    b.setZero();
    for (int i = 0; i < n; ++i)
    {
        Eigen::Matrix3d AA = A_[i].block<3,3>(0,0) - Eigen::Matrix3d::Identity();
        Eigen::Vector3d bb = R_ba * B_[i].block<3,1>(0,3) - A_[i].block<3,1>(0,3);

        A.block<3,3>(3*i, 0) = AA;
        b.block<3,1>(3*i, 0) = bb;
    }
    pinA = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Eigen::MatrixXd::Identity(n*3, n*3));
    Eigen::Vector3d t_ba = pinA * b;
    H.block<3,3>(0,0) = R_ba;
    H.block<3,1>(0,3) = t_ba;

    return H;
}