#ifndef ANDREFFEXTENDEDAXXBSOLVER_H
#define ANDREFFEXTENDEDAXXBSOLVER_H

#include "../solverAXXB.h"
#include "type.h"

class AndreffExtendedAXXBSolver:public SolverAXXB
{
public:
    AndreffExtendedAXXBSolver(const Poses A, const Poses B):SolverAXXB(A,B){}
    Pose SolveX();
};

#endif // ANDREFFEXTENDEDAXXBSOLVER_H
