#ifndef CAMODOCALDANII_H
#define CAMODOCALDANII_H

#include "camodocal/calib/HandEyeCalibration.h"

#include "../solverAXXB.h"

class CamodocalDanii: public SolverAXXB{
    public:
        //BIGGEST NOTE: Please note that the A and B here are the transformation matrices for baseEE and cameraObject
        //respectively. They are NOT the A and B matrices for AXXB solver. Be careful.
        CamodocalDanii(const std::vector<Eigen::Matrix4d> A, const std::vector<Eigen::Matrix4d> B):
            SolverAXXB(A, B)
            {
                calibrator = camodocal::HandEyeCalibration();
                calibrator.setVerbose(false);
            }
        Eigen::Matrix4d SolveX();
    private:
        std::vector<Eigen::Matrix4d> A_;
        std::vector<Eigen::Matrix4d> B_;
        camodocal::HandEyeCalibration calibrator;

};

#endif // CAMODOCALDANII_H