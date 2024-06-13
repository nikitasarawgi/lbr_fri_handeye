/**
 * The charuco-calib-server is responsible for calibrating the camera using the charuco board.
 * The idea is that, once you have the camera adjusted to look at the  charuco board from a certain angle,
 * you can call this service with the live image and 
 * use that image to calculate the extrinsic parameters of the camera for that image and timestamp
 * The calculated transform will be returned as a response.
*/

#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "charuco-calibrator.h"
#include "utils/utils.h"
#include "hand_eye_msgs/srv/camera_calibrated_transform.hpp"

class CharucoCalibServer : public rclcpp::Node{
    public:
        CharucoCalibServer()
        : Node("charuco_calib_server"){
            this->declare_parameter("squaresX", 5);
            this->declare_parameter("squaresY", 6);
            this->declare_parameter("squareLength", 38.0);
            this->declare_parameter("markerLength", 19.0);
            this->declare_parameter("dictionaryId", 11);

            int squaresX = this->get_parameter("squaresX").as_int();
            int squaresY = this->get_parameter("squaresY").as_int();
            float squareLength = (float)this->get_parameter("squareLength").as_double();
            int markerLength = (float)this->get_parameter("markerLength").as_double();
            int dictionaryId = this->get_parameter("dictionaryId").as_int();

            calibrator = CharucoCalibrator(squaresX, squaresY, squareLength, markerLength, dictionaryId);

            // Create a subscriber to get the camera info, only once
            this->declare_parameter("camera_info_topic", "/camera/camera_info");
            std::string cameraInfoTopic = this->get_parameter("camera_info_topic").as_string();
            cameraInfoSub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                cameraInfoTopic.c_str(), 10, std::bind(&CharucoCalibServer::cameraInfoCallback, this, std::placeholders::_1));
        }
        void calibrateServer(const std::shared_ptr<hand_eye_msgs::srv::CameraCalibratedTransform::Request> request,
                                    const std::shared_ptr<hand_eye_msgs::srv::CameraCalibratedTransform::Response> response){
            RCLCPP_INFO(this->get_logger(), "Calibrating the incoming image...");
            cv::Mat cvImage = cv_bridge::toCvCopy((request->image), sensor_msgs::image_encodings::BGR8)->image;
            cv::Mat transform = calibrator.calibrate(cvImage);
            geometry_msgs::msg::TransformStamped rosTransform = cvMatToTransformStamped(transform, "camera", "object");
            response->transform = rosTransform;

            RCLCPP_INFO(this->get_logger(), "Sending the transform back...");
        }
    private:
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoSub;
        CharucoCalibrator calibrator;

        void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "Received camera info message ...");
            // Set the camera matrix and distortion coefficients
            calibrator.setCameraProperties(msg->k, msg->d);

            // We just need the camera info once and then we can unsubscribe
            cameraInfoSub.reset();  
        }        
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto calibServer = std::make_shared<CharucoCalibServer>();

    rclcpp::Service<hand_eye_msgs::srv::CameraCalibratedTransform>::SharedPtr service =
        calibServer->create_service<hand_eye_msgs::srv::CameraCalibratedTransform>("charuco_calib_server", 
            std::bind(&CharucoCalibServer::calibrateServer, calibServer, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(calibServer->get_logger(), "Charuco calibration server is running...");

    rclcpp::spin(calibServer);
    rclcpp::shutdown();
    return 0;
}