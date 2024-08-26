#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <chrono>
#include "solverAXXB/zhixy/conventionalaxxbsvdsolver.h"
#include "solverAXXB/zhixy/andreffextendedaxxbsolver.h"
#include "solverAXXB/zhixy/extendedaxxbelilambdasvdsolver.h"
#include "solverAXXB/tsaiAXXB/tsaiaxxb.h"
#include "solverAXXB/camodocalDanii/camodocaldanii.h"
#include "moveit_msgs/srv/get_position_fk.hpp"
#include "moveit_msgs/msg/robot_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "hand_eye_msgs/srv/camera_calibrated_transform.hpp"
#include "utils/utils.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <cv_bridge/cv_bridge.h>

class JointRecorder : public rclcpp::Node{
    public:
        JointRecorder():Node("joint_recorder"){
            RCLCPP_INFO(this->get_logger(), "JointRecorder started...");

            this->declare_parameter("image_topic", "/camera/camera/color/image_raw");
            this->declare_parameter("joint_state_topic", "/lbr/joint_states");
            this->declare_parameter("images_folder", "./cameraCalibrated-try3.txt");
            this->declare_parameter("joint_state_file", "./jointState-try3.txt");


            imageTopic_ = this->get_parameter("image_topic").as_string();
            jointStateTopic_ = this->get_parameter("joint_state_topic").as_string();
            imageDataFile_ = this->get_parameter("image_folder").as_string();
            jointStateDataFile_ = this->get_parameter("joint_state_file").as_string();

            fkCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            
            imageCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            jointStateCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            rclcpp::SubscriptionOptions imageSubOptions;
            imageSubOptions.callback_group = imageCallbackGroup_;
            imageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
                imageTopic_,
                10,
                std::bind(&JointRecorder::imageCallback, this, std::placeholders::_1),
                imageSubOptions
            );

            rclcpp::SubscriptionOptions jointStateSubOptions;
            jointStateSubOptions.callback_group = jointStateCallbackGroup_;
            jointStateSub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                jointStateTopic_,
                10,
                std::bind(&JointRecorder::jointStateCallback, this, std::placeholders::_1),
                jointStateSubOptions
            );

            fkClient_ = this->create_client<moveit_msgs::srv::GetPositionFK>("/lbr/compute_fk", rmw_qos_profile_default, fkCallbackGroup_);

            while(!fkClient_->wait_for_service(std::chrono::seconds(1))){
                if(!rclcpp::ok()){
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service. Exiting...");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Moveit compute kinematics service not available. Waiting...");
            }

            inputThread_ = std::thread(&JointRecorder::readInput, this);
            keepRunning_ = true;
        }
        ~JointRecorder(){
            keepRunning_ = false;
            if(inputThread_.joinable()){
                inputThread_.join();
            }
        }
    private:
        void imageCallback(const sensor_msgs::msg::Image::SharedPtr image){
            numCount++;
            RCLCPP_INFO(this->get_logger(), "Image data received...");
            // If you are only saving the image data
            std::string filename = imageDataFile_ + "image_rgb_" + std::to_string(numCount) + ".png";
            cv::Mat cvImage = cv_bridge::toCvCopy(image, "bgr8")->image;
            cv::imwrite(filename, cvImage);
             
        }
        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr jointState){
            RCLCPP_INFO(this->get_logger(), "Joint state data received...");

            // Using the joint state data, calculate the forward kinematics and get Transform of Flange wrt Base
            auto request = std::make_shared<moveit_msgs::srv::GetPositionFK::Request>();
            request->header.frame_id = "lbr/link_0";
            request->header.stamp = rclcpp::Time();
            request->fk_link_names.push_back("link_ee");
            request->robot_state.joint_state = *jointState;

            auto result = fkClient_->async_send_request(request);
            std::future_status status = result.wait_for(std::chrono::seconds(5));
            // if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result)
            //     == rclcpp::FutureReturnCode::SUCCESS){
            if(status == std::future_status::ready){
                    RCLCPP_INFO(this->get_logger(), "Forward kinematics data received successfully...");
                    // Convert the data to geometry_msgs and save the data to the vector
                    geometry_msgs::msg::PoseStamped p = result.get()->pose_stamped[0];
                    geometry_msgs::msg::TransformStamped t = poseToTransform(p, "link_ee");
                    
                    {
                        std::lock_guard<std::mutex> data_lock(mutex2_);
                        jointStateData_.push_back(t);
                        RCLCPP_INFO(this->get_logger(), "Transformation pushed. New size: %d", jointStateData_.size());
                    }
                    return;
                    
            }else{
                RCLCPP_ERROR(this->get_logger(), "Failed to receive moveit FK data...");
                return;
            }
              
        }
        void saveData(){
            // Save both the transform vectors into their respective files  

            std::ofstream jfile(jointStateDataFile_);
            for(const auto& transform : jointStateData_){
                tf2::Transform tf2_transform;
                tf2::fromMsg(transform.transform, tf2_transform);
                tf2::Matrix3x3 rotMatrix = tf2_transform.getBasis();
                tf2::Vector3 translation = tf2_transform.getOrigin();
                jfile << "[";
                for (int i = 0; i < 3; ++i) {
                    jfile << rotMatrix[i][0] << ", " << rotMatrix[i][1] << ", " << rotMatrix[i][2] << ", " << translation[i];
                    if (i < 2) { 
                        jfile << ", ";
                    }
                }
                jfile << ", 0, 0, 0, 1";
                jfile << "]\n";
            }
            jfile.close();
        }
        void printMatrix(Eigen::Matrix4d matrix){
            std::ostringstream oss;
            oss << "\n" << matrix;
            RCLCPP_INFO(this->get_logger(), oss.str().c_str());
        }
        void readInput(){
            while(keepRunning_){
                int key = getch();
                if(key == 's' || key == 'S'){
                    RCLCPP_INFO(this->get_logger(), "Saving the data into a file...");
                    saveData();
                }
            }
        }

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSub_;
        rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedPtr fkClient_;
        int numCount = 0;
        std::string imageTopic_;
        std::string jointStateTopic_;
        std::thread inputThread_;
        bool keepRunning_;
        std::string jointStateDataFile_;
        std::string imageDataFile_;

        std::vector<geometry_msgs::msg::TransformStamped> cameraCalibrationData_;
        std::vector<geometry_msgs::msg::TransformStamped> jointStateData_;

        rclcpp::CallbackGroup::SharedPtr calibCallbackGroup_;
        rclcpp::CallbackGroup::SharedPtr fkCallbackGroup_;
        rclcpp::CallbackGroup::SharedPtr imageCallbackGroup_;
        rclcpp::CallbackGroup::SharedPtr jointStateCallbackGroup_;
    
};



int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<JointRecorder>());
    auto node = std::make_shared<JointRecorder>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}