#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include "solverAXXB/zhixy/conventionalaxxbsvdsolver.h"
#include "solverAXXB/zhixy/andreffextendedaxxbsolver.h"
#include "solverAXXB/zhixy/extendedaxxbelilambdasvdsolver.h"
#include "solverAXXB/tsaiAXXB/tsaiaxxb.h"
#include "solverAXXB/camodocalDanii/camodocaldanii.h"
#include "utils/utils.h"

class TestAutoHandEyeCalib : public rclcpp::Node{
    public:
        TestAutoHandEyeCalib():Node("auto_hand_eye_calib"){
            RCLCPP_INFO(this->get_logger(), "TestAutoHandEyeCalib started...");
            createAndTest();
        }
    private:
        std::vector<Eigen::Matrix4d> A_values_;
        std::vector<Eigen::Matrix4d> B_values_;


        void createAndTest(){
            std::vector<std::array<double, 16>> Tbf = {
                {
                    -1, 0, 0, 651.04,
                    0, 1, 0, -117.25,
                    0, 0, -1, 306.28,
                    0, 0, 0, 1 
                },
                {
                    -0.80405571, -0.58298042, 0.11674007, 513.36,
                    -0.57628438, 0.81247811, 0.08817952, -117.04,
                    -0.14625569, 0.00362577, -0.98924018, 340.79,
                    0, 0, 0, 1
                },
                {
                    -1, 0, 0, 691.51,
                    0, 1, 0, -147.83,
                    0, 0, -1, 320.17,
                    0, 0, 0, 1 
                },
                {
                    -0.6845594699999999, -0.64604025, 0.33765415, 541.34,
                    -0.59928651, 0.76248193, 0.24387902, -162.23,
                    -0.41501085, -0.03540188, -0.90912744, 340.72,
                    0, 0, 0, 1 
                }
            };
            std::vector<std::array<double, 16>> Tco = {
                {   
                    0.9454053952158723, 0.2880060521299599, -0.1525160733602695, -85.88403638988746,
                    -0.1116087087766026, 0.725816165401498, 0.678774182012755, 11.31047930847414,
                    0.3061897039776809, -0.6246946517926818, 0.7183345023035438, 287.376469066622,
                    0, 0, 0, 1 
                },
                {
                    0.9565, -0.291, -0.0116, -101.51158,
                    0.22655, 0.7172, 0.65899, -10.0310,
                    -0.18375, -0.6329, 0.75205, 359.4884,
                    0, 0, 0, 1
                },
                {
                    0.93148074466329, 0.3241709338965756, -0.1650964201251645, -43.69416986721683,
                    -0.1330730505348938, 0.7259937965994629, 0.6747033203715803, 37.00210911767341,
                    0.3385781823196238, -0.6065032670280678, 0.7193876573457595, 288.2163948084068,
                    0, 0, 0, 1 
                },
                {
                    0.9025969253659591, -0.3503169004437544, 0.2501936441706659, 15.96686001260923,
                    0.1501574804954988, 0.800887732074004, 0.57968230235587, 7.634028532505012,
                    -0.4034495276625826, -0.4856510165508141, 0.7754815076789388, 348.9210527576388,
                    0, 0, 0, 1 
                }
            };

            // Since this is Eye in Hand calibration ...
            // Creating the vectors for A and B
            for(int i = 0; i < (int)Tbf.size() - 1; i++){
                Eigen::Matrix4d E1 = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(Tbf[i].data());
                Eigen::Matrix4d E2 = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(Tbf[i+1].data());
                Eigen::Matrix4d poseA = E1.inverse() * E2;
                A_values_.push_back(poseA);
            
                Eigen::Matrix4d S1 = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(Tco[i].data());
                Eigen::Matrix4d S2 = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(Tco[i+1].data());
                Eigen::Matrix4d poseB = S1 * S2.inverse();
                B_values_.push_back(poseB);
            }


            // Testing the solvers
            RCLCPP_INFO(this->get_logger(), "Testing the solvers...");

            // Zhixy's solver
            ConventionalAXXBSVDSolver zhixy_conv_solver(A_values_, B_values_);
            Pose result = zhixy_conv_solver.SolveX();
            RCLCPP_INFO(this->get_logger(), "Zhixy's ConventionalAXXBSVDSolver: ");
            printMatrix(result);

            ExtendedAXXBEliLambdaSVDSolver zhixy_ext_solver(A_values_, B_values_);
            Pose result2 = zhixy_ext_solver.SolveX();
            RCLCPP_INFO(this->get_logger(), "Zhixy's ExtendedAXXBEliLambdaSVDSolver: ");
            printMatrix(result2);

            AndreffExtendedAXXBSolver zhixy_ext2_solver(A_values_, B_values_);
            Pose result3 = zhixy_ext2_solver.SolveX();
            RCLCPP_INFO(this->get_logger(), "Zhixy's AndreffExtendedAXXBSolver: ");
            printMatrix(result3);

            // Tsai's solver
            TsaiAXXB tsai_solver(A_values_, B_values_);
            Eigen::Matrix4d result4 = tsai_solver.SolveX();
            RCLCPP_INFO(this->get_logger(), "Tsai's TsaiAXXB: ");
            printMatrix(result4);

            // // Daniilidis' solver
            // DaniilidisAXXB daniilidis_solver(A_values_, B_values_);
            // Eigen::Matrix4d result5 = daniilidis_solver.SolveX();
            // RCLCPP_INFO(this->get_logger(), "Daniilidis' DaniilidisAXXB: ");
            // printMatrix(result5);

            // Testing Camodocal's solver
            std::vector<Eigen::Matrix4d> A_values_camodocal;
            std::vector<Eigen::Matrix4d> B_values_camodocal;
            for(int i = 0; i < Tbf.size(); i++){
                Eigen::Matrix4d E = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(Tbf[i].data());
                Eigen::Matrix4d S = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(Tco[i].data());
                A_values_camodocal.push_back(E);
                B_values_camodocal.push_back(S);
            }
            CamodocalDanii camodocal_solver(A_values_camodocal, B_values_camodocal);
            Eigen::Matrix4d result6 = camodocal_solver.SolveX();
            RCLCPP_INFO(this->get_logger(), "Camodocal's CamodocalDanii: ");
            printMatrix(result6);


        }
        
        void printMatrix(Eigen::Matrix4d matrix){
            std::ostringstream oss;
            oss << "\n" << matrix;
            RCLCPP_INFO(this->get_logger(), oss.str().c_str());
        }
            
};