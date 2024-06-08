#ifndef ZJULIONTSAI_H
#define ZJULIONTSAI_H

#include "../solverAXXB.h"

class ZJulionTsai: public SolverAXXB{
    public:
        ZJulionTsai(const std::vector<Eigen::Matrix4d> A, const std::vector<Eigen::Matrix4d> B):
            SolverAXXB(A, B){}
        Eigen::Matrix4d SolveX();
};

#endif // ZJULIONTSAI_H