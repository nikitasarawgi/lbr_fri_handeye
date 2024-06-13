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

            this->declare_parameter("image_topic", "/camera/camera/color/image_raw");
            this->declare_parameter("joint_state_topic", "/lbr/joint_states");
            this->declare_parameter("camera_calibration_file", "./cameraCalibrated-try3.txt");
            this->declare_parameter("joint_state_file", "./jointState-try3.txt");
            this->declare_parameter("image_data_folder", "./images/");
            this->declare_parameter("save_images", true);
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

            fkCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            calibCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            
            imageCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            jointStateCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            rclcpp::SubscriptionOptions imageSubOptions;
            imageSubOptions.callback_group = imageCallbackGroup_;
            imageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
                imageTopic_,
                10,
                std::bind(&AutoHandEyeCalib::imageCallback, this, std::placeholders::_1),
                imageSubOptions
            );

            rclcpp::SubscriptionOptions jointStateSubOptions;
            jointStateSubOptions.callback_group = jointStateCallbackGroup_;
            jointStateSub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                jointStateTopic_,
                10,
                std::bind(&AutoHandEyeCalib::jointStateCallback, this, std::placeholders::_1),
                jointStateSubOptions
            );

            numObservations_ = 0;
            readImage_ = false;
            readJointState_ = false;

            fkClient_ = this->create_client<moveit_msgs::srv::GetPositionFK>("/lbr/compute_fk", rmw_qos_profile_default, fkCallbackGroup_);
            cameraCalibrationClient_ = this->create_client<hand_eye_msgs::srv::CameraCalibratedTransform>("/charuco_calib_server", rmw_qos_profile_default, calibCallbackGroup_);

            // while(!cameraCalibrationClient_->wait_for_service(std::chrono::seconds(1))){
            //     if(!rclcpp::ok()){
            //         RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service. Exiting...");
            //         return;
            //     }
            //     RCLCPP_INFO(this->get_logger(), "Charuco service not available. Waiting...");
            // }

            // while(!fkClient_->wait_for_service(std::chrono::seconds(1))){
            //     if(!rclcpp::ok()){
            //         RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service. Exiting...");
            //         return;
            //     }
            //     RCLCPP_INFO(this->get_logger(), "Moveit compute kinematics service not available. Waiting...");
            // }

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
                std::lock_guard<std::mutex> lock(mutex1_);
                readImage_ = true;
            } 
            return;  
        }
        void getLastJointState(){
            RCLCPP_INFO(this->get_logger(), "Getting the last joint state data...");
            {
                std::lock_guard<std::mutex> lock(mutex1_);
                readJointState_ = true; 
            } 
            return;      
        }
        void imageCallback(const sensor_msgs::msg::Image::SharedPtr image){
            if(readImage_){
                {
                    std::lock_guard<std::mutex> lock(mutex1_);
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
                std::future_status status = result.wait_for(std::chrono::seconds(5));
                // if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result)
                //     == rclcpp::FutureReturnCode::SUCCESS){
                if(status == std::future_status::ready){
                        RCLCPP_INFO(this->get_logger(), "Camera calibration data received ...");
                        // Save the data to the vector
                        geometry_msgs::msg::TransformStamped t = result.get()->transform;
                        {
                            std::lock_guard<std::mutex> data_lock(mutex2_);
                            cameraCalibrationData_.push_back(t);
                        }
                }else{
                    RCLCPP_ERROR(this->get_logger(), "Failed to receive camera calibration data...");
                    return;
                }
            }     
        }
        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr jointState){
            if(readJointState_){
                if(isCalibrateCamera_){
                    {
                        std::lock_guard<std::mutex> lock(mutex1_);
                        readJointState_ = false;
                    }
                }
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
                line.erase(std::remove(line.begin(), line.end(), '['), line.end());
                line.erase(std::remove(line.begin(), line.end(), ']'), line.end());

                std::istringstream iss(line);
                std::string value;
                std::vector<double> matrixElements;

                while (std::getline(iss, value, ',')) {
                    matrixElements.push_back(std::stod(value));
                }

                geometry_msgs::msg::TransformStamped transform;

                transform.transform.translation.x = matrixElements[3];
                transform.transform.translation.y = matrixElements[7];
                transform.transform.translation.z = matrixElements[11];

                Eigen::Matrix3d rotMatrix;
                rotMatrix << matrixElements[0], matrixElements[1], matrixElements[2],
                            matrixElements[4], matrixElements[5], matrixElements[6],
                            matrixElements[8], matrixElements[9], matrixElements[10];
                Eigen::Quaterniond q(rotMatrix);
                transform.transform.rotation.x = q.x();
                transform.transform.rotation.y = q.y();
                transform.transform.rotation.z = q.z();
                transform.transform.rotation.w = q.w();

                cameraCalibrationData_.push_back(transform);
            }

            cfile.close();

            std::ifstream jfile(jointStateDataFile_);

            while(std::getline(jfile, line)){
                line.erase(std::remove(line.begin(), line.end(), '['), line.end());
                line.erase(std::remove(line.begin(), line.end(), ']'), line.end());

                std::istringstream iss(line);
                std::string value;
                std::vector<double> matrixElements;

                while (std::getline(iss, value, ',')) {
                    matrixElements.push_back(std::stod(value));
                }

                geometry_msgs::msg::TransformStamped transform;

                transform.transform.translation.x = matrixElements[3];
                transform.transform.translation.y = matrixElements[7];
                transform.transform.translation.z = matrixElements[11];

                Eigen::Matrix3d rotMatrix;
                rotMatrix << matrixElements[0], matrixElements[1], matrixElements[2],
                            matrixElements[4], matrixElements[5], matrixElements[6],
                            matrixElements[8], matrixElements[9], matrixElements[10];
                Eigen::Quaterniond q(rotMatrix);
                transform.transform.rotation.x = q.x();
                transform.transform.rotation.y = q.y();
                transform.transform.rotation.z = q.z();
                transform.transform.rotation.w = q.w();

                jointStateData_.push_back(transform);
            }
            jfile.close();
        }
        void calculateFinalTransforms(){
            RCLCPP_INFO(this->get_logger(), "Calculating final transforms...");

            std::vector<Eigen::Matrix4d> cameraCalibrationEigen = transformVecToEigenVec(cameraCalibrationData_);
            std::vector<Eigen::Matrix4d> jointStateEigen = transformVecToEigenVec(jointStateData_);

            RCLCPP_INFO(this->get_logger(), "Size of cameraCalibrationEigen: %d", cameraCalibrationEigen.size());
            RCLCPP_INFO(this->get_logger(), "Size of jointStateEigen: %d", jointStateEigen.size());

            // Create the A and B vectors for the AXXB solvers
            std::vector<Eigen::Matrix4d> A_values;
            std::vector<Eigen::Matrix4d> B_values;

            for(int i = 0; i < jointStateEigen.size() - 1; i ++){
                Eigen::Matrix4d A = jointStateEigen[i].inverse() * jointStateEigen[i + 1];
                A_values.push_back(A);

            }
            RCLCPP_INFO(this->get_logger(), "Size of A_values: %d", A_values.size());
            for(int i = 0; i < cameraCalibrationEigen.size() - 1; i++){
                Eigen::Matrix4d B = cameraCalibrationEigen[i] * cameraCalibrationEigen[i + 1].inverse();
                B_values.push_back(B);
            }
            RCLCPP_INFO(this->get_logger(), "Size of B_values: %d", B_values.size());
             // Testing the solvers
            RCLCPP_INFO(this->get_logger(), "Testing the solvers...");

            // // Zhixy's solver
            // ConventionalAXXBSVDSolver zhixy_conv_solver(A_values, B_values);
            // Pose result = zhixy_conv_solver.SolveX();
            // RCLCPP_INFO(this->get_logger(), "Zhixy's ConventionalAXXBSVDSolver: ");
            // printMatrix(result);

            // ExtendedAXXBEliLambdaSVDSolver zhixy_ext_solver(A_values, B_values);
            // Pose result2 = zhixy_ext_solver.SolveX();
            // RCLCPP_INFO(this->get_logger(), "Zhixy's ExtendedAXXBEliLambdaSVDSolver: ");
            // printMatrix(result2);

            // AndreffExtendedAXXBSolver zhixy_ext2_solver(A_values, B_values);
            // Pose result3 = zhixy_ext2_solver.SolveX();
            // RCLCPP_INFO(this->get_logger(), "Zhixy's AndreffExtendedAXXBSolver: ");
            // printMatrix(result3);

            // // Tsai's solver
            // TsaiAXXB tsai_solver(A_values, B_values);
            // Eigen::Matrix4d result4 = tsai_solver.SolveX();
            // RCLCPP_INFO(this->get_logger(), "Tsai's TsaiAXXB: ");
            // printMatrix(result4);

            CamodocalDanii camodocal_solver(jointStateEigen, cameraCalibrationEigen);
             Eigen::Matrix4d result5 = camodocal_solver.SolveX();
            RCLCPP_INFO(this->get_logger(), "Camodocal's CamodocalDanii: ");
            printMatrix(result5);

            RCLCPP_INFO(this->get_logger(), "PRINTING TFC PART 1...");
            Eigen::Matrix4d tfc = cameraCalibrationEigen[0] * result5.inverse() * jointStateEigen[0];
            printMatrix(tfc.inverse());

            RCLCPP_INFO(this->get_logger(), "PRINTING TFC PART 2...");
            Eigen::Matrix4d tfc2 = cameraCalibrationEigen[1] * result5.inverse() * jointStateEigen[1];
            printMatrix(tfc2.inverse());

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
        std::mutex mutex1_;
        std::mutex mutex2_;
        bool saveImageData_;
        bool isCalibrateCamera_;
        std::string cameraCalibrationDataFile_;
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
    // rclcpp::spin(std::make_shared<AutoHandEyeCalib>());
    auto node = std::make_shared<AutoHandEyeCalib>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}