#include <termios.h>
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

// function getch is from
// http://answers.ros.org/question/63491/keyboard-key-pressed/
int getch() {
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt); // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);               // disable buffering
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // apply new settings

    int c = getchar(); // read character (non-blocking)

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore old settings
    return c;
}

geometry_msgs::msg::TransformStamped cvMatToTransformStamped(const cv::Mat& transform, const std::string& frame_id, const std::string& child_frame_id){
    geometry_msgs::msg::TransformStamped transformStamped;

    // Set the frame IDs
    transformStamped.header.frame_id = frame_id;
    transformStamped.child_frame_id = child_frame_id;

    // Set the translation
    tf2::Vector3 position;

    position.setX(transform.at<double>(0, 3));
    position.setY(transform.at<double>(1, 3));
    position.setZ(transform.at<double>(2, 3));
     transformStamped.transform.translation = tf2::toMsg(position);

    // Convert rotation matrix to quaternion
    tf2::Matrix3x3 rotMatrix(
        transform.at<double>(0, 0), transform.at<double>(0, 1), transform.at<double>(0, 2),
        transform.at<double>(1, 0), transform.at<double>(1, 1), transform.at<double>(1, 2),
        transform.at<double>(2, 0), transform.at<double>(2, 1), transform.at<double>(2, 2)
    );
    tf2::Quaternion quaternion;
    rotMatrix.getRotation(quaternion);

    // Set the rotation
    transformStamped.transform.rotation = tf2::toMsg(quaternion);

    return transformStamped;
}

geometry_msgs::msg::TransformStamped poseToTransform(geometry_msgs::msg::PoseStamped p, const std::string& child_frame_id){

    // tf2::Vector3 position;  
    // tf2::fromMsg(pose.pose.position, position);
    // tf2::Quaternion orientation;
    // tf2::fromMsg(pose.pose.orientation, orientation);

    // geometry_msgs::msg::TransformStamped transform;
    // transform.child_frame_id = child_frame_id;
    // transform.header.frame_id = pose.header.frame_id;
    // transform.transform.translation = tf2::toMsg(position);
    // transform.transform.rotation = tf2::toMsg(orientation);

    auto logger = rclcpp::get_logger("trasnformLogger");
    // RCLCPP_INFO(logger, "HERE");


    geometry_msgs::msg::TransformStamped t;
    // RCLCPP_INFO(logger, "Just printing x here. x: %f", p.pose.position.x);
    // RCLCPP_INFO(logger, "Just printing y here. y: %f", p.pose.position.y);
    // RCLCPP_INFO(logger, "Just printing z here. z: %f", p.pose.position.z);
    // t.child_frame_id = child_frame_id;
    t.header.frame_id = p.header.frame_id;
    t.transform.translation.x = p.pose.position.x;
    t.transform.translation.y = p.pose.position.y;
    t.transform.translation.z = p.pose.position.z;
    t.transform.rotation = p.pose.orientation;

    // RCLCPP_INFO(logger, "Just printing x here. x: %f", t.transform.translation.x);
    // RCLCPP_INFO(logger, "Just printing y here. y: %f", t.transform.translation.y);
    // RCLCPP_INFO(logger, "Just printing z here. z: %f", t.transform.translation.z);
                        

    return t;
    // RCLCPP_INFO(logger, "returning from here");
}

std::vector<Eigen::Matrix4d> transformVecToEigenVec(const std::vector<geometry_msgs::msg::TransformStamped> transformVec){
    std::vector<Eigen::Matrix4d> eigenVec;
    for(int i = 0; i < transformVec.size(); i++){
        Eigen::Isometry3d iso = tf2::transformToEigen(transformVec[i]);
        eigenVec.push_back(iso.matrix());
    }
    return eigenVec;
}

