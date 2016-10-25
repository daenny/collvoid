#include "collvoid_dwa_local_planner/path_alignment_cost_function.h"

#include <math.h>

namespace collvoid_dwa_local_planner
{
double PathAlignmentCostFunction::scoreTrajectory(Trajectory &traj)
{
    return fabs(traj.yv_ );
}
}
