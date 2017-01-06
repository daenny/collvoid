/*
 * Copyright (c) 2012, Daniel Claes, Maastricht University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Maastricht University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef ROSAGENT_H
#define ROSAGENT_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/local_planner_limits.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <collvoid_msgs/PoseArrayWeighted.h>
#include <collvoid_msgs/AggregatedPoseTwist.h>

#include <geometry_msgs/PolygonStamped.h>

#include <collvoid_msgs/PoseTwistWithCovariance.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>

#include <collvoid_local_planner/Agent.h>
#include <collvoid_local_planner/GetCollvoidTwist.h>
#include <collvoid_srvs/GetNeighbors.h>
#include <collvoid_srvs/GetMe.h>

#include <base_local_planner/local_planner_util.h>
#include <collvoid_local_planner/CollvoidConfig.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/simple_trajectory_generator.h>
#include <base_local_planner/simple_scored_sampling_planner.h>
#include <base_local_planner/map_grid_cost_function.h>


namespace collvoid {
    class ROSAgent : public Agent {
    public:
        ROSAgent();

        ~ROSAgent();
        bool checkTrajectory(Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f vel_samples);

        double scoreTrajectory(Eigen::Vector3f vel_samples);

        void reconfigure(collvoid_local_planner::CollvoidConfig &cfg);
        void init(ros::NodeHandle private_nh, tf::TransformListener *tf, base_local_planner::LocalPlannerUtil *planner_util, costmap_2d::Costmap2DROS* costmap_ros);

        void computeNewVelocity(Vector2 pref_velocity, geometry_msgs::Twist &cmd_vel);
        void computeOrcaVelocity(Vector2 pref_velocity);
        void computeClearpathVelocity(Vector2 pref_velocity);
        void computeSampledVelocity(Vector2 pref_velocity);

        void addNHConstraints(double min_dist, Vector2 pref_velocity);

        void computeObstacles();
        std::vector<Obstacle> getObstacles();
        void sortObstacleLines();


        collvoid::Vector2 LineSegmentToLineSegmentIntersection(double x1, double y1, double x2, double y2, double x3,
                                                               double y3, double x4, double y4);

        double getDistToFootprint(collvoid::Vector2 &point);
        void computeObstacleLine(Vector2 &point);
        void createObstacleLine(std::vector<Vector2> &own_footprint, Vector2 &obst1, Vector2 &obst2);

        bool getMe();
        bool getNeighbors();
        AgentPtr createAgentFromMsg(collvoid_msgs::PoseTwistWithCovariance &msg);

        bool compareNeighborsPositions(const AgentPtr &agent1, const AgentPtr &agent2);
        bool compareVectorPosition(const collvoid::Vector2 &v1, const collvoid::Vector2 &v2);

        double vMaxAng();

        void updatePlanAndLocalCosts(tf::Stamped<tf::Pose> global_pose,
                                               const std::vector<geometry_msgs::PoseStamped> &new_plan, const tf::Stamped<tf::Pose>& robot_vel);
        Eigen::Vector3f createTwistFromVector(Vector2 speed, base_local_planner::LocalPlannerLimits &limits);

            //config
        int num_samples_;

        //NH stuff
        double min_error_holo_;
        double max_error_holo_;
        bool holo_robot_;
        double time_to_holo_;

        //ORCA stuff
        double min_dist_obst_;

        //obstacles
        std::vector<collvoid::Vector2> obstacle_points_;
        costmap_2d::Costmap2DROS* costmap_ros_;


        //Agent description
        std::string base_frame_, global_frame_, name_space_;
        double wheel_base_;
        double fixed_robot_radius_;

        //set automatically
        bool initialized_;
        double last_twist_ang_;

        double current_angular_speed_;

        std::vector<std::pair<collvoid::Vector2, collvoid::Vector2> > footprint_lines_;

        //COLLVOID
        boost::mutex obstacle_lock_, computing_lock_;

        //me stuff
        tf::TransformListener *tf_;

        //subscribers and publishers
        ros::Publisher lines_pub_, neighbors_pub_, vo_pub_, samples_pub_, speed_pub_, obstacles_pub_;


        // service calls
        bool getTwistServiceCB(collvoid_local_planner::GetCollvoidTwist::Request &req, collvoid_local_planner::GetCollvoidTwist::Response &res);
        geometry_msgs::Twist computeVelocityCommand(Vector2 waypoint, double goal_ang);

        ros::ServiceServer get_collvoid_twist_service_;
        ros::ServiceClient get_obstacles_srv_;
        ros::ServiceClient get_me_srv_, get_neighbors_srv_;


        bool use_dwa_score_;
        Eigen::Vector3f pos_, vel_;

        //obstacle check
        base_local_planner::LocalPlannerUtil *planner_util_;

        double pdist_scale_;

        double sim_period_;///< @brief The number of seconds to use to compute max/min vels for dwa

        double forward_point_distance_;
        double goal_heading_sq_dist_;

        std::vector<geometry_msgs::PoseStamped> global_plan_;

        boost::mutex configuration_mutex_;

        // see constructor body for explanations
        base_local_planner::SimpleTrajectoryGenerator generator_;
        base_local_planner::ObstacleCostFunction* obstacle_costs_;

        base_local_planner::MapGridCostFunction* path_costs_;
        base_local_planner::MapGridCostFunction* goal_costs_;
        base_local_planner::MapGridCostFunction* goal_front_costs_;
        base_local_planner::MapGridCostFunction* alignment_costs_;
        base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;
        std::vector<base_local_planner::TrajectoryCostFunction *> critics_;


    };

    typedef boost::shared_ptr<ROSAgent> ROSAgentPtr;


}
#endif
