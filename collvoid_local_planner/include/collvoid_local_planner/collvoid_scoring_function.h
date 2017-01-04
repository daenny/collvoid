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
#include <collvoid_srvs/GetNeighbors.h>
#include <collvoid_srvs/GetMe.h>

#include <visualization_msgs/MarkerArray.h>
#include "collvoid_local_planner/Vector2.h"
#include "collvoid_local_planner/Utils.h"
#include "collvoid_local_planner/Agent.h"
#include "collvoid_local_planner/collvoid_publishers.h"


using namespace collvoid;

namespace collvoid_scoring_function
{

    using namespace base_local_planner;

    class CollvoidScoringFunction : public TrajectoryCostFunction
    {
    public:
        CollvoidScoringFunction() {};
        ~CollvoidScoringFunction() {};

        void init(ros::NodeHandle nh);
        bool prepare();
        double scoreTrajectory(Trajectory &traj);

    private:
        bool getMe();
        bool getNeighbors();
        bool compareNeighborsPositions(const AgentPtr &agent1, const AgentPtr &agent2);
        bool compareVectorPosition(const collvoid::Vector2 &v1, const collvoid::Vector2 &v2);

        AgentPtr createAgentFromMsg(collvoid_msgs::PoseTwistWithCovariance &msg);

        AgentPtr me_;
        bool use_truncation_, use_polygon_footprint_;
        double trunc_time_, max_dist_vo_;
        ros::Publisher vo_pub_, neighbors_pub_, samples_pub_;
        ros::ServiceClient get_me_srv_, get_neighbors_srv_;

        std::vector<VelocitySample> points;
    };
}


#endif //COLLVOID_LOCAL_PLANNER_COLLVOID_SCORING_FUNCTION_H
