//
// Created by danielclaes on 22/06/15.
//

#ifndef COLLVOID_LOCAL_PLANNER_COLLVOID_SCORING_FUNCTION_H
#define COLLVOID_LOCAL_PLANNER_COLLVOID_SCORING_FUNCTION_H

#include <base_local_planner/trajectory_cost_function.h>
#include <geometry_msgs/PoseStamped.h>

namespace collvoid_dwa_local_planner
{

    using namespace base_local_planner;

    class CollvoidScoringFunction : public TrajectoryCostFunction
    {
    public:
        CollvoidScoringFunction() {}
        ~CollvoidScoringFunction() {}

        void setParams();

        bool prepare();
        double scoreTrajectory(Trajectory &traj);
        void setGoalPose(const geometry_msgs::PoseStamped &goalPose) { goalPose_ = goalPose; }

    private:
        geometry_msgs::PoseStamped goalPose_;

    };
}


#endif //COLLVOID_LOCAL_PLANNER_COLLVOID_SCORING_FUNCTION_H
