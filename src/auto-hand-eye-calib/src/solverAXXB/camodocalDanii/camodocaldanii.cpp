#include "camodocaldanii.h"
#include <Eigen/Core>

Eigen::Matrix4d CamodocalDanii::SolveX()
{
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> rvecs1;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> tvecs1;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> rvecs2;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> tvecs2;

    for (const auto& T : A_) {
        // Extract the rotation matrix
        Eigen::Matrix3d R = T.block<3,3>(0, 0);

        // Extract the translation vector
        Eigen::Vector3d t = T.block<3,1>(0, 3);

        // Convert the rotation matrix to axis-angle representation
        Eigen::AngleAxisd angleAxis(R);
        Eigen::Vector3d rvec = angleAxis.axis() * angleAxis.angle();

        // Add to vectors
        rvecs1.push_back(rvec);
        tvecs1.push_back(t);
    }

    for (const auto& T : B_) {
        // Extract the rotation matrix
        Eigen::Matrix3d R = T.block<3,3>(0, 0);

        // Extract the translation vector
        Eigen::Vector3d t = T.block<3,1>(0, 3);

        // Convert the rotation matrix to axis-angle representation
        Eigen::AngleAxisd angleAxis(R);
        Eigen::Vector3d rvec = angleAxis.axis() * angleAxis.angle();

        // Add to vectors
        rvecs2.push_back(rvec);
        tvecs2.push_back(t);
    }

    Eigen::Matrix4d H_12;
    calibrator.estimateHandEyeScrew(rvecs1, tvecs1, rvecs2, tvecs2, H_12);

    return H_12;

}