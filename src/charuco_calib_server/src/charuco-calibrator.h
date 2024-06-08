#ifndef CHARUCO_CALIBRATOR_H
#define CHARUCO_CALIBRATOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>

class CharucoCalibrator{
    public:
        int squaresX, squaresY;
        float squareLength, markerLength;
        cv::Mat cameraMatrix, distCoeffs;
        cv::aruco::Dictionary dictionary;
        cv::Ptr<cv::aruco::CharucoBoard> charucoBoard;

        CharucoCalibrator();

        CharucoCalibrator(int squaresX, int squaresY, float squareLength, float markerLength, int dictionaryId);
        
        // Create a charucoBoard. Make sure it has the same parameters as the one you have printed
        void createCharucoBoard();

        // Instantiate the camera intrinsic properties given k and d (as received from CameraInfo datatype)
        void setCameraProperties(std::array<double, 9UL> k, std::vector<double> d);

        // Given an image, calibrate it to return a transformation matrix representing the 
        // extrinsic calibration parameters of the object wrt the camera.
        cv::Mat calibrate(cv::Mat image);
};

#endif // CHARUCO_CALIBRATOR_H