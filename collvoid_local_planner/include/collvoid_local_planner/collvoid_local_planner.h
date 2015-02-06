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


#ifndef COLLVOID_LOCAL_PLANNER_H_
#define COLLVOID_LOCAL_PLANNER_H_


#include <tf/transform_listener.h>
#include <nav_core/base_local_planner.h>
#include <angles/angles.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GridCells.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include "collvoid_local_planner/ROSAgent.h"
#include "collvoid_local_planner/CollvoidConfig.h"
//#include <base_local_planner/trajectory_planner_ros.h>

using namespace collvoid;

namespace collvoid_local_planner {
  /* typedef boost::shared_ptr<ROSAgent> ROSAgentPtr; */

  class CollvoidLocalPlanner: public nav_core::BaseLocalPlanner {
  public:
    CollvoidLocalPlanner();
    CollvoidLocalPlanner(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
    ~CollvoidLocalPlanner();
    void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
    bool isGoalReached();
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  private:
    bool rotateToGoal(const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, double goal_th, geometry_msgs::Twist& cmd_vel);
    bool stopWithAccLimits(const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, geometry_msgs::Twist& cmd_vel);


    bool transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan, const costmap_2d::Costmap2DROS& costmap, const std::string& global_frame, std::vector<geometry_msgs::PoseStamped>& transformed_plan);
    void findBestWaypoint(geometry_msgs::PoseStamped& target_pose, const tf::Stamped<tf::Pose>& global_pose);

    void obstaclesCallback(const nav_msgs::GridCells::ConstPtr& msg);

    //Dyn reconfigure
    boost::recursive_mutex configuration_mutex_;
    dynamic_reconfigure::Server<collvoid_local_planner::CollvoidConfig> *dsrv_;
    void reconfigureCB(collvoid_local_planner::CollvoidConfig &config, uint32_t level);
    collvoid_local_planner::CollvoidConfig last_config_;
    collvoid_local_planner::CollvoidConfig default_config_;



    //Datatypes:
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D costmap_; ///< @brief The costmap the controller will use

    tf::TransformListener* tf_; 
    
    bool initialized_, skip_next_, setup_;

    //Agent stuff
    double sim_period_;
    double max_vel_x_, min_vel_x_;
    double max_vel_y_, min_vel_y_;
    double max_vel_th_, min_vel_th_, min_vel_th_inplace_;
    double acc_lim_x_, acc_lim_y_, acc_lim_th_;
    double wheel_base_, radius_;
    double max_vel_with_obstacles_;

    bool holo_robot_;

    double trunc_time_, left_pref_; 
    
    double xy_goal_tolerance_, yaw_goal_tolerance_;
    double rot_stopped_velocity_, trans_stopped_velocity_;

    double publish_me_period_, publish_positions_period_;
    double threshold_last_seen_;
    

    bool latch_xy_goal_tolerance_, xy_tolerance_latch_, rotating_to_goal_, ignore_goal_yaw_, delete_observations_, been_in_obstacle_;
    
    unsigned int current_waypoint_;
    //params ORCA
    double  time_horizon_obst_;
    double eps_;

    ROSAgentPtr me_;
    //boost::unordered_map<std::string,ROSAgent> neighbors_;
    //base_local_planner::TrajectoryPlannerROS collision_planner_;
     
    
    double time_to_holo_, min_error_holo_, max_error_holo_;

    std::string global_frame_; ///< @brief The frame in which the controller will run
    std::string robot_base_frame_; ///< @brief Used as the base frame id of the robot
    std::string my_id_;
    std::vector<geometry_msgs::PoseStamped> global_plan_, transformed_plan_;
    ros::Publisher g_plan_pub_, l_plan_pub_;
    ros::Subscriber obstacles_sub_;

  };//end class

  //  Vector2 rotateVectorByAngle(double x, double y, double ang);


}; //end namespace

#endif //end guard catch
