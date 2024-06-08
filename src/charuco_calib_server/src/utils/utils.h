#ifndef CHARUCO_UTILS_H
#define CHARUCO_UTILS_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <opencv2/opencv.hpp>

int getch();
geometry_msgs::msg::TransformStamped cvMatToTransformStamped(const cv::Mat& transform, const std::string& frame_id, const std::string& child_frame_id);

#endif // CHARUCO_UTILS_H