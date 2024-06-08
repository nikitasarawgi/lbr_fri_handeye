#ifndef EXTENDEDAXXBELILAMBDASVDSOLVER_H
#define EXTENDEDAXXBELILAMBDASVDSOLVER_H

#include "../solverAXXB.h"
#include "type.h"
class ExtendedAXXBEliLambdaSVDSolver:public SolverAXXB
{
public:
    ExtendedAXXBEliLambdaSVDSolver(const Poses A, const Poses B):SolverAXXB(A,B){}
    Pose SolveX();
};

#endif // EXTENDEDAXXBELILAMBDASVDSOLVER_H
