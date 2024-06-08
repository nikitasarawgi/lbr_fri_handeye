#ifndef TSAIAXXB_H
#define TSAIAXXB_H

#include "../solverAXXB.h"

class TsaiAXXB: public SolverAXXB{
    public:
        TsaiAXXB(const std::vector<Eigen::Matrix4d> A, const std::vector<Eigen::Matrix4d> B):
            SolverAXXB(A, B){}
        Eigen::Matrix4d SolveX();
};

#endif // TSAIAXXB_H