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
class AutoHandEyeCalib : public rclcpp::Node{
    public:
        AutoHandEyeCalib():Node("auto_hand_eye_calib"){
            RCLCPP_INFO(this->get_logger(), "AutoHandEyeCalib started...");

            this->declare_parameter("image_topic", "/camera/color/image_raw");
            this->declare_parameter("joint_state_topic", "/joint_states");
            this->declare_parameter("camera_calibration_file", "./cameraCalibrated987.txt");
            this->declare_parameter("joint_state_file", "./jointState987.txt");
            this->declare_parameter("image_data_folder", "./images/");
            this->declare_parameter("save_images", false);
            this->declare_parameter("is_calibrate_camera", true);

            imageTopic_ = this->get_parameter("image_topic").as_string();
            jointStateTopic_ = this->get_parameter("joint_state_topic").as_string();
            cameraCalibrationDataFile_ = this->get_parameter("camera_calibration_file").as_string();
            jointStateDataFile_ = this->get_parameter("joint_state_file").as_string();
            imageDataFile_ = this->get_parameter("image_data_folder").as_string();
            saveImageData_ = this->get_parameter("save_images").as_bool();
            isCalibrateCamera_ = this->get_parameter("is_calibrate_camera").as_bool();

            if(!isCalibrateCamera_){
                RCLCPP_INFO(this->get_logger(), "Not calibrating the camera...");
                saveImageData_ = true;
            }

            imageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
                imageTopic_,
                10,
                std::bind(&AutoHandEyeCalib::imageCallback, this, std::placeholders::_1)
            );

            jointStateSub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                jointStateTopic_,
                10,
                std::bind(&AutoHandEyeCalib::jointStateCallback, this, std::placeholders::_1)
            );

            numObservations_ = 0;
            readImage_ = false;
            readJointState_ = false;

            fkClient_ = this->create_client<moveit_msgs::srv::GetPositionFK>("/compute_fk");
            cameraCalibrationClient_ = this->create_client<hand_eye_msgs::srv::CameraCalibratedTransform>("/charuco_calib_server");

            while(!cameraCalibrationClient_->wait_for_service(std::chrono::seconds(1))){
                if(!rclcpp::ok()){
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service. Exiting...");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Charuco service not available. Waiting...");
            }

            while(!fkClient_->wait_for_service(std::chrono::seconds(1))){
                if(!rclcpp::ok()){
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service. Exiting...");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Moveit compute kinematics service not available. Waiting...");
            }

            inputThread_ = std::thread(&AutoHandEyeCalib::readInput, this);
            keepRunning_ = true;
        }
        ~AutoHandEyeCalib(){
            keepRunning_ = false;
            if(inputThread_.joinable()){
                inputThread_.join();
            }
        }
    private:
        void getLastImage(){
            RCLCPP_INFO(this->get_logger(), "Getting the last image data...");
            {
                std::lock_guard<std::mutex> lock(mutex_);
                readImage_ = true;
            }   
        }
        void getLastJointState(){
            RCLCPP_INFO(this->get_logger(), "Getting the last joint state data...");
            {
                std::lock_guard<std::mutex> lock(mutex_);
                readJointState_ = true; 
            }       
        }
        void imageCallback(const sensor_msgs::msg::Image::SharedPtr image){
            if(readImage_){
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if(isCalibrateCamera_){
                        readImage_ = false;
                    }
                    numObservations_++;
                }
                
                RCLCPP_INFO(this->get_logger(), "Image data received...");
                // If you are only saving the image data
                if(saveImageData_){
                    std::string filename = imageDataFile_ + "image" + std::to_string(numObservations_) + ".jpg";
                    cv::Mat cvImage = cv_bridge::toCvCopy(image, "bgr8")->image;
                    cv::imwrite(filename, cvImage);
                    
                    if(!isCalibrateCamera_){
                        return;
                    }
                }
                // Using the image data, get the camera calibration matrix and append it to the datastructure
                auto request = std::make_shared<hand_eye_msgs::srv::CameraCalibratedTransform::Request>();
                request->image = *image;
                
                auto result = cameraCalibrationClient_->async_send_request(request);
                if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result)
                    == rclcpp::FutureReturnCode::SUCCESS){
                        RCLCPP_INFO(this->get_logger(), "Camera calibration data received successfully...");
                        // Save the data to the vector
                        geometry_msgs::msg::TransformStamped t = result.get()->transform;
                        cameraCalibrationData_.push_back(t);
                }else{
                    RCLCPP_ERROR(this->get_logger(), "Failed to receive camera calibration data...");
                    return;
                }
            }     
        }
        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr jointState){
            if(readJointState_){
                if(isCalibrateCamera_){
                    std::lock_guard<std::mutex> lock(mutex_);
                    readJointState_ = false;
                }
                RCLCPP_INFO(this->get_logger(), "Joint state data received...");

                // Using the joint state data, calculate the forward kinematics and get Transform of Flange wrt Base
                auto request = std::make_shared<moveit_msgs::srv::GetPositionFK::Request>();
                request->header.frame_id = "lbr/link_0";
                request->header.stamp = rclcpp::Time();
                request->fk_link_names.push_back("link_ee");
                request->robot_state.joint_state = *jointState;

                auto result = fkClient_->async_send_request(request);
                if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result)
                    == rclcpp::FutureReturnCode::SUCCESS){
                        RCLCPP_INFO(this->get_logger(), "Forward kinematics data received successfully...");
                        // Convert the data to geometry_msgs and save the data to the vector
                        geometry_msgs::msg::PoseStamped pose = result.get()->pose_stamped[0];
                        geometry_msgs::msg::TransformStamped transform = poseToTransform(pose, "link_ee");
                        jointStateData_.push_back(transform);
                        
                }else{
                    RCLCPP_ERROR(this->get_logger(), "Failed to receive moveit FK data...");
                    return;
                }
            }    
        }
        void saveData(){
            // Save both the transform vectors into their respective files  
            std::ofstream cfile(cameraCalibrationDataFile_);
            for(const auto& transform : cameraCalibrationData_){
                tf2::Transform tf2_transform;
                tf2::fromMsg(transform.transform, tf2_transform);
                tf2::Matrix3x3 rotMatrix = tf2_transform.getBasis();
                tf2::Vector3 translation = tf2_transform.getOrigin();
                cfile << "[";
                for (int i = 0; i < 3; ++i) {
                    cfile << rotMatrix[i][0] << ", " << rotMatrix[i][1] << ", " << rotMatrix[i][2] << ", " << translation[i];
                    if (i < 2) { 
                        cfile << ", ";
                    }
                }
                cfile << ", 0, 0, 0, 1";
                cfile << "]\n";
            }
            cfile.close();

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
        void readData(){
            // Read data from the two files into the corresponding vectors
            std::ifstream cfile(cameraCalibrationDataFile_);
            std::string line;

            while(std::getline(cfile, line)){
                line = line.substr(1, line.size() - 2);

                std::istringstream iss(line);
                std::vector<double> matrixElements((std::istream_iterator<double>(iss)), std::istream_iterator<double>());

                geometry_msgs::msg::TransformStamped transform;
                tf2::Matrix3x3 rotMatrix(matrixElements[0], matrixElements[1], matrixElements[2],
                                        matrixElements[4], matrixElements[5], matrixElements[6],
                                        matrixElements[8], matrixElements[9], matrixElements[10]);
                tf2::Vector3 translation(matrixElements[3], matrixElements[7], matrixElements[11]);
                tf2::Transform tf2_transform(rotMatrix, translation);
                transform.transform = tf2::toMsg(tf2_transform);

                cameraCalibrationData_.push_back(transform);
            }

            cfile.close();

            std::ifstream jfile(jointStateDataFile_);

            while(std::getline(jfile, line)){
                line = line.substr(1, line.size() - 2);

                std::istringstream iss(line);
                std::vector<double> matrixElements((std::istream_iterator<double>(iss)), std::istream_iterator<double>());

                geometry_msgs::msg::TransformStamped transform;
                tf2::Matrix3x3 rotMatrix(matrixElements[0], matrixElements[1], matrixElements[2],
                                        matrixElements[4], matrixElements[5], matrixElements[6],
                                        matrixElements[8], matrixElements[9], matrixElements[10]);
                tf2::Vector3 translation(matrixElements[3], matrixElements[7], matrixElements[11]);
                tf2::Transform tf2_transform(rotMatrix, translation);
                transform.transform = tf2::toMsg(tf2_transform);

                jointStateData_.push_back(transform);
            }
            jfile.close();
        }
        void calculateFinalTransforms(){
            std::vector<Eigen::Matrix4d> cameraCalibrationEigen = transformVecToEigenVec(cameraCalibrationData_);
            std::vector<Eigen::Matrix4d> jointStateEigen = transformVecToEigenVec(jointStateData_);

            // Create the A and B vectors for the AXXB solvers
            std::vector<Eigen::Matrix4d> A_values;
            std::vector<Eigen::Matrix4d> B_values;

            for(int i = 0; i < jointStateEigen.size() - 1; i ++){
                Eigen::Matrix4d A = jointStateEigen[i].inverse() * jointStateEigen[i + 1];
                A_values.push_back(A);
            }
            for(int i = 0; i < cameraCalibrationEigen.size() - 1; i++){
                Eigen::Matrix4d B = cameraCalibrationEigen[i] * cameraCalibrationEigen[i + 1].inverse();
                B_values.push_back(B);
            }

             // Testing the solvers
            RCLCPP_INFO(this->get_logger(), "Testing the solvers...");

            // Zhixy's solver
            ConventionalAXXBSVDSolver zhixy_conv_solver(A_values, B_values);
            Pose result = zhixy_conv_solver.SolveX();
            RCLCPP_INFO(this->get_logger(), "Zhixy's ConventionalAXXBSVDSolver: ");
            printMatrix(result);

            ExtendedAXXBEliLambdaSVDSolver zhixy_ext_solver(A_values, B_values);
            Pose result2 = zhixy_ext_solver.SolveX();
            RCLCPP_INFO(this->get_logger(), "Zhixy's ExtendedAXXBEliLambdaSVDSolver: ");
            printMatrix(result2);

            AndreffExtendedAXXBSolver zhixy_ext2_solver(A_values, B_values);
            Pose result3 = zhixy_ext2_solver.SolveX();
            RCLCPP_INFO(this->get_logger(), "Zhixy's AndreffExtendedAXXBSolver: ");
            printMatrix(result3);

            // Tsai's solver
            TsaiAXXB tsai_solver(A_values, B_values);
            Eigen::Matrix4d result4 = tsai_solver.SolveX();
            RCLCPP_INFO(this->get_logger(), "Tsai's TsaiAXXB: ");
            printMatrix(result4);

            CamodocalDanii camodocal_solver(jointStateEigen, cameraCalibrationEigen);
             Eigen::Matrix4d result5 = camodocal_solver.SolveX();
            RCLCPP_INFO(this->get_logger(), "Camodocal's CamodocalDanii: ");
            printMatrix(result5);

        }
        void printMatrix(Eigen::Matrix4d matrix){
            std::ostringstream oss;
            oss << "\n" << matrix;
            RCLCPP_INFO(this->get_logger(), oss.str().c_str());
        }
        void readInput(){
            while(keepRunning_){
                int key = getch();
                if(key == 'c' || key == 'C'){
                    RCLCPP_INFO(this->get_logger(), "Received input from terminal...");
                    getLastImage();
                    getLastJointState();
                }
                else if(key == 's' || key == 'S'){
                    RCLCPP_INFO(this->get_logger(), "Saving the data into a file...");
                    saveData();
                }
                else if(key == 'r' || key == 'R'){
                    RCLCPP_INFO(this->get_logger(), "Reading from the files...");
                    readData();
                }
                else if(key == 'e' || key == 'E'){
                    RCLCPP_INFO(this->get_logger(), "Calculating the final transforms...");
                    calculateFinalTransforms();
                }
            }
        }

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSub_;
        rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedPtr fkClient_;
        rclcpp::Client<hand_eye_msgs::srv::CameraCalibratedTransform>::SharedPtr cameraCalibrationClient_; 
        rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr handEyePub_;
        std::string imageTopic_;
        std::string jointStateTopic_;
        bool readImage_;
        bool readJointState_;
        int numObservations_;
        std::thread inputThread_;
        bool keepRunning_;
        std::mutex mutex_;
        bool saveImageData_;
        bool isCalibrateCamera_;
        std::string cameraCalibrationDataFile_;
        std::string jointStateDataFile_;
        std::string imageDataFile_;

        std::vector<geometry_msgs::msg::TransformStamped> cameraCalibrationData_;
        std::vector<geometry_msgs::msg::TransformStamped> jointStateData_;
    
};



int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoHandEyeCalib>());
    rclcpp::shutdown();
    return 0;
}