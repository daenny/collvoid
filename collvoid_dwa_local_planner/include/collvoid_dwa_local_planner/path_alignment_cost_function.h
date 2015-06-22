#ifndef PATHALIGMENTCOSTFUNCTION_H
#define PATHALIGMENTCOSTFUNCTION_H

#include <base_local_planner/trajectory_cost_function.h>

namespace collvoid_dwa_local_planner
{

using namespace base_local_planner;

class PathAlignmentCostFunction : public TrajectoryCostFunction
{
public:
    PathAlignmentCostFunction() {}
    ~PathAlignmentCostFunction() {}

    bool prepare() { return true; }

    double scoreTrajectory(Trajectory &traj);
};
}

#endif // PATHALIGMENTCOSTFUNCTION_H
