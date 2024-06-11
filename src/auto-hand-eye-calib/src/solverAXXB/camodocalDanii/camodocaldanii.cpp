#include "camodocaldanii.h"
#include <Eigen/Core>
#include "rclcpp/rclcpp.hpp"

Eigen::Matrix4d CamodocalDanii::SolveX()
{
    auto logger = rclcpp::get_logger("camodocal");

    // RCLCPP_INFO(logger, "I am in this function");
    RCLCPP_INFO(logger, "A: %d, B: %d", A_.size(), B_.size());
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> rvecs1;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> tvecs1;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> rvecs2;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> tvecs2;

    for (const auto& T : A_) {
        // RCLCPP_INFO(logger, "Am i here 1");
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
        // RCLCPP_INFO(logger, "am i here 2");
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

    RCLCPP_INFO(logger, "I have successfully created these");
    RCLCPP_INFO(logger, "rvec1: %d, tvec1: %d, rvec2: %d, tvec2: %d", rvecs1.size(), tvecs1.size(), rvecs2.size(), tvecs2.size());


    Eigen::Matrix4d H_12;
    calibrator.estimateHandEyeScrew(rvecs1, tvecs1, rvecs2, tvecs2, H_12);

    return H_12;

}