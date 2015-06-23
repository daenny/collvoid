//
// Created by danielclaes on 22/06/15.
//

#ifndef COLLVOID_LOCAL_PLANNER_COLLVOID_SCORING_FUNCTION_H
#define COLLVOID_LOCAL_PLANNER_COLLVOID_SCORING_FUNCTION_H
#include <ros/ros.h>
#include <tf/tf.h>
#include <base_local_planner/trajectory_cost_function.h>
#include <geometry_msgs/PoseStamped.h>
#include <collvoid_msgs/PoseTwistWithCovariance.h>
#include "collvoid_local_planner/Vector2.h"
#include "collvoid_local_planner/Utils.h"
#include "collvoid_local_planner/Agent.h"

using namespace collvoid;

namespace collvoid_scoring_function
{

    using namespace base_local_planner;

    class CollvoidScoringFunction : public TrajectoryCostFunction
    {
    public:
        CollvoidScoringFunction();
        virtual ~CollvoidScoringFunction();

        void init(ros::NodeHandle nh);
        bool prepare();
        double scoreTrajectory(Trajectory &traj);
        void setGoalPose(const geometry_msgs::PoseStamped &goal_pose) { goal_pose_ = goal_pose; }

    private:
        bool getMe();
        bool getNeighbors();
        bool compareNeighborsPositions(const AgentPtr &agent1, const AgentPtr &agent2);
        bool compareVectorPosition(const collvoid::Vector2 &v1, const collvoid::Vector2 &v2);

        AgentPtr createAgentFromMsg(collvoid_msgs::PoseTwistWithCovariance &msg);


        AgentPtr me_;
        bool use_truncation_, convex_, holo_robot_;
        double trunc_time_;
        geometry_msgs::PoseStamped goal_pose_;
        ros::ServiceClient get_me_srv_, get_neighbors_srv_;
    };
}


#endif //COLLVOID_LOCAL_PLANNER_COLLVOID_SCORING_FUNCTION_H
