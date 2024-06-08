#include <termios.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

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
    transformStamped.transform.translation.x = transform.at<double>(0, 3);
    transformStamped.transform.translation.y = transform.at<double>(1, 3);
    transformStamped.transform.translation.z = transform.at<double>(2, 3);

    // Convert rotation matrix to quaternion
    cv::Mat rvec;
    cv::Rodrigues(transform(cv::Rect(0, 0, 3, 3)), rvec); // Extract rotation matrix and convert to rotation vector

    tf2::Matrix3x3 rotMatrix(
        transform.at<double>(0, 0), transform.at<double>(0, 1), transform.at<double>(0, 2),
        transform.at<double>(1, 0), transform.at<double>(1, 1), transform.at<double>(1, 2),
        transform.at<double>(2, 0), transform.at<double>(2, 1), transform.at<double>(2, 2)
    );
    tf2::Quaternion quaternion;
    rotMatrix.getRotation(quaternion);

    // Set the rotation
    transformStamped.transform.rotation.x = quaternion.x();
    transformStamped.transform.rotation.y = quaternion.y();
    transformStamped.transform.rotation.z = quaternion.z();
    transformStamped.transform.rotation.w = quaternion.w();

    return transformStamped;
}

