#ifndef SOLVERAXXB_H
#define SOLVERAXXB_H

#include <Eigen/Core>

class SolverAXXB{
    public:
        SolverAXXB();
        SolverAXXB(const std::vector<Eigen::Matrix4d> A, const std::vector<Eigen::Matrix4d> B):
            A_(A), B_(B){
                assert(A.size() == B.size());
                assert(A.size() >= 2);
            }
        virtual Eigen::Matrix4d SolveX()=0;

    protected:
        std::vector<Eigen::Matrix4d> A_;
        std::vector<Eigen::Matrix4d> B_;
};

#endif // SOLVERAXXB_H