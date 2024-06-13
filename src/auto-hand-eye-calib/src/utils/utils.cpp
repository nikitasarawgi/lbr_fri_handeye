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

    transformStamped.header.frame_id = frame_id;
    transformStamped.child_frame_id = child_frame_id;

    tf2::Vector3 position;

    // position.setX(transform.at<double>(0, 3));
    // position.setY(transform.at<double>(1, 3));
    // position.setZ(transform.at<double>(2, 3));
    //  transformStamped.transform.translation = tf2::toMsg(position);

    // without tf2
    position.x() = transform.at<double>(0, 3);
    position.y() = transform.at<double>(1, 3);
    position.z() = transform.at<double>(2, 3);
    transformStamped.transform.translation.x = position.x();
    transformStamped.transform.translation.y = position.y();
    transformStamped.transform.translation.z = position.z();

    // tf2::Matrix3x3 rotMatrix(
    //     transform.at<double>(0, 0), transform.at<double>(0, 1), transform.at<double>(0, 2),
    //     transform.at<double>(1, 0), transform.at<double>(1, 1), transform.at<double>(1, 2),
    //     transform.at<double>(2, 0), transform.at<double>(2, 1), transform.at<double>(2, 2)
    // );
    // tf2::Quaternion quaternion;
    // rotMatrix.getRotation(quaternion);

    // transformStamped.transform.rotation = tf2::toMsg(quaternion);

    // without using tf2
    Eigen::Matrix3d rotMatrix;
    rotMatrix << transform.at<double>(0, 0), transform.at<double>(0, 1), transform.at<double>(0, 2),
                transform.at<double>(1, 0), transform.at<double>(1, 1), transform.at<double>(1, 2),
                transform.at<double>(2, 0), transform.at<double>(2, 1), transform.at<double>(2, 2);
    Eigen::Quaterniond quaternion(rotMatrix);
    transformStamped.transform.rotation.x = quaternion.x();
    transformStamped.transform.rotation.y = quaternion.y();
    transformStamped.transform.rotation.z = quaternion.z();
    transformStamped.transform.rotation.w = quaternion.w();

    return transformStamped;
}

geometry_msgs::msg::TransformStamped poseToTransform(geometry_msgs::msg::PoseStamped p, const std::string& child_frame_id){
    // without tf2

    auto logger = rclcpp::get_logger("trasnformLogger");

    geometry_msgs::msg::TransformStamped t;

    t.child_frame_id = child_frame_id;
    t.header.frame_id = p.header.frame_id;
    t.transform.translation.x = p.pose.position.x;
    t.transform.translation.y = p.pose.position.y;
    t.transform.translation.z = p.pose.position.z;
    t.transform.rotation = p.pose.orientation;

    return t;
}

Eigen::Matrix4d transformToEigenMatrix(const geometry_msgs::msg::TransformStamped& tf) {
    Eigen::Matrix4d eigenMatrix = Eigen::Matrix4d::Identity();

    // Translation
    eigenMatrix(0, 3) = tf.transform.translation.x;
    eigenMatrix(1, 3) = tf.transform.translation.y;
    eigenMatrix(2, 3) = tf.transform.translation.z;

    // Rotation
    Eigen::Quaterniond q(
        tf.transform.rotation.w,
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z
    );
    eigenMatrix.block<3,3>(0,0) = q.normalized().toRotationMatrix();

    return eigenMatrix;
}

std::vector<Eigen::Matrix4d> transformVecToEigenVec(const std::vector<geometry_msgs::msg::TransformStamped> transformVec){
    std::vector<Eigen::Matrix4d> eigenVec;
    for(int i = 0; i < transformVec.size(); i++){
        // Eigen::Isometry3d iso = tf2::transformToEigen(transformVec[i]);
        // eigenVec.push_back(iso.matrix());

        // Without using tf2
        Eigen::Matrix4d mat = transformToEigenMatrix(transformVec[i]);
        eigenVec.push_back(mat);
    }
    return eigenVec;
}

