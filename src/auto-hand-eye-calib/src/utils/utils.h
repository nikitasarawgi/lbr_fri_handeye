#ifndef AUTO_HAND_EYE_UTILS_H
#define AUTO_HAND_EYE_UTILS_H

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <opencv2/opencv.hpp>

int getch();
geometry_msgs::msg::TransformStamped cvMatToTransformStamped(const cv::Mat& transform, const std::string& frame_id, const std::string& child_frame_id);
geometry_msgs::msg::TransformStamped poseToTransform(geometry_msgs::msg::PoseStamped pose, const std::string& child_frame_id);
std::vector<Eigen::Matrix4d> transformVecToEigenVec(const std::vector<geometry_msgs::msg::TransformStamped> transformVec);
Eigen::Matrix4d transformToEigenMatrix(const geometry_msgs::msg::TransformStamped& tf);

#endif // AUTO_HAND_EYE_UTILS_H