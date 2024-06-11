#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
// #include <opencv2/aruco/dictionary.hpp>
#include <opencv2/highgui.hpp>
#include "charuco-calibrator.h"
#include "rclcpp/rclcpp.hpp"

CharucoCalibrator::CharucoCalibrator(){

}

CharucoCalibrator::CharucoCalibrator(int squaresX, int squaresY, float squareLength, float markerLength, int dictionaryId){
    this->squaresX = squaresX;
    this->squaresY = squaresY;
    this->squareLength = squareLength;
    this->markerLength = markerLength;
    this->dictionary = cv::aruco::getPredefinedDictionary(dictionaryId);
    createCharucoBoard();
}

void CharucoCalibrator::createCharucoBoard(){
    cv::Size size(this->squaresX, this->squaresY);
    // this->charucoBoard = cv::makePtr<cv::aruco::CharucoBoard>(size, this->squareLength, this->markerLength, this->dictionary);

    this->charucoBoard = cv::aruco::CharucoBoard::create(this->squaresX, this->squaresY, this->squareLength, this->markerLength, this->dictionary);
}

void CharucoCalibrator::setCameraProperties(std::array<double, 9UL> k, std::vector<double> d){
    auto logger = rclcpp::get_logger("caliblogger");
    RCLCPP_INFO(logger, "Setting Camera properties..");
    this->cameraMatrix = cv::Mat(3, 3, CV_64F, (void*)k.data()).clone();
    this->distCoeffs = cv::Mat(1, d.size(), CV_64F, (void*)d.data()).clone();
    RCLCPP_INFO(logger, "HERE HERE cameraMatrixSize: %d, dist: %d", this->cameraMatrix.size().width, this->distCoeffs.size().width);
}

cv::Mat CharucoCalibrator::calibrate(cv::Mat image){
    auto logger = rclcpp::get_logger("caliblogger");
    cv::Vec3d rvec, tvec;

    // no explanation necessary. Please refer to opencv documentations and tutorials
    cv::Mat imageCopy;
    image.copyTo(imageCopy);
    cv::Mat grayImage;
    cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::makePtr<cv::aruco::DetectorParameters>();

    cv::aruco::detectMarkers(grayImage, cv::makePtr<cv::aruco::Dictionary>(this->dictionary), markerCorners, markerIds, params);

    if(markerIds.size() > 0){
        cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
        std::vector<cv::Point2f> charucoCorners;
        std::vector<int> charucoIds;
        cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, grayImage, this->charucoBoard, charucoCorners, charucoIds, this->cameraMatrix, this->distCoeffs);

        if(charucoIds.size() > 0){
            cv::Scalar color = cv::Scalar(255, 0, 0);
            cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);
            RCLCPP_INFO(logger, "CharucoCornerSize: %d, charucoIdsSize: %d", charucoCorners.size(), charucoIds.size());
            RCLCPP_INFO(logger, "cameraMatrixSize: %d, dist: %d", this->cameraMatrix.size(), this->distCoeffs.size());
            bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, this->charucoBoard, this->cameraMatrix, this->distCoeffs, rvec, tvec);
            if(valid){
                 cv::drawFrameAxes(imageCopy, this->cameraMatrix, this->distCoeffs, rvec, tvec, 0.1f);

            }
        }
    }
    cv::imshow("out", imageCopy);

    // convert rvec and tvec into a transformation matrix
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat transform = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(transform(cv::Rect(0, 0, 3, 3)));
    transform.at<double>(0, 3) = tvec[0];
    transform.at<double>(1, 3) = tvec[1];
    transform.at<double>(2, 3) = tvec[2];

    return transform;
}