/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef DWA_LOCAL_PLANNER_DWA_PLANNER_ROS_H_
#define DWA_LOCAL_PLANNER_DWA_PLANNER_ROS_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <collvoid_dwa_local_planner/DWAPlannerConfig.h>

#include <std_srvs/Empty.h>

#include <angles/angles.h>

#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/latched_stop_rotate_controller.h>

#include <base_local_planner/odometry_helper_ros.h>

#include <collvoid_dwa_local_planner/dwa_planner.h>

namespace collvoid_dwa_local_planner
{
/** @brief ROS Wrapper for the DWAPlanner that adheres to the
 *  @class DWAPlannerROS
 *  BaseLocalPlanner interface and can be used as a plugin for move_base.
 */
class DWAPlannerROS : public nav_core::BaseLocalPlanner
{
public:
    /** @brief  Constructor for DWAPlannerROS wrapper */
    DWAPlannerROS();

    /** @brief  Constructs the ros wrapper
     *  @param name The name to give this instance of the trajectory planner
     *  @param tf A pointer to a transform listener
     *  @param costmap The cost map to use for assigning costs to trajectories
     */
    void initialize(std::string name, tf::TransformListener *tf,
                    costmap_2d::Costmap2DROS *costmap_ros);

    /** @brief  Destructor for the wrapper */
    ~DWAPlannerROS();

    /** @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
     *  @param cmd_vel Will be filled with the velocity command to be passed to the robot base
     *  @return True if a valid trajectory was found, false otherwise
     */
    bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);


    /** @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base, using dynamic window approach
     *  @param cmd_vel Will be filled with the velocity command to be passed to the robot base
     *  @return True if a valid trajectory was found, false otherwise
     */
    bool dwaComputeVelocityCommands(tf::Stamped<tf::Pose> &global_pose, geometry_msgs::Twist &cmd_vel);

    /** @brief  Set the plan that the controller is following
     *  @param orig_global_plan The plan to pass to the controller
     *  @return True if the plan was updated successfully, false otherwise
     */
    bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);

    /** @brief  Check if the goal pose has been achieved
     *  @return True if achieved, false otherwise
     */
    bool isGoalReached();

    /**
   * Check if a trajectory (normally a part of a global plan) is free of obstacles
   * within obstacle_max_distance_ meters from the robot
   * @param robot_pose Current robot pose
   * @param trajectory Trajectory to check
   * @param distance Distance from the robot to the closest obstacle, if any
   * @return True if the trajectory is free, false otherwise
   */
    bool freeOfObstacles(const tf::Stamped<tf::Pose>& robot_local_pose,
                       const std::vector<geometry_msgs::PoseStamped>& plan, double& distance);


    inline bool isInitialized() const { return initialized_; }

private:
    /** @brief Callback to update the local planner's parameters based on dynamic reconfigure */
    void reconfigureCB(DWAPlannerConfig &config, uint32_t level);

    void publishLocalPlan(std::vector<geometry_msgs::PoseStamped> &path);

    void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped> &path);

    bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
    void clearCostmaps();
    tf::TransformListener *tf_; ///< @brief Used for transforming point clouds

    // for visualisation, publishers of global and local plan
    ros::Publisher g_plan_pub_, l_plan_pub_;

    ros::ServiceServer  clear_costmaps_srv_;

    base_local_planner::LocalPlannerUtil planner_util_;

    boost::shared_ptr<DWAPlanner> dp_; ///< @brief The trajectory controller

    costmap_2d::Costmap2DROS *costmap_ros_;

    dynamic_reconfigure::Server<DWAPlannerConfig> *dsrv_;
    collvoid_dwa_local_planner::DWAPlannerConfig default_config_;
    bool setup_;
    tf::Stamped<tf::Pose> current_pose_;

    base_local_planner::LatchedStopRotateController latchedStopRotateController_;
    base_local_planner::WorldModel* world_model_;

    bool initialized_;


    base_local_planner::OdometryHelperRos odom_helper_;
    std::string odom_topic_;

    // Trajectory free parameters
    double obstacle_max_distance_ = 1.0; //was 1.6
    size_t blocked_path_count_ = 0;
    size_t max_blocked_paths_  = 10;  // blocked_path_count_ can grow until this value before considering a path blocked

    inline double distance2D(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{
  double x1 = p1.pose.position.x, x2 = p2.pose.position.x;
  double y1 = p1.pose.position.y, y2 = p2.pose.position.y;
  return std::sqrt(std::pow(x2 - x1, 2.0) + std::pow(y2 - y1, 2.0));
}

inline double distance2D(const geometry_msgs::PoseStamped& p, const std::vector<geometry_msgs::PoseStamped>& t, size_t& closest_point)
{
  double min_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < t.size(); i++)
  {
    double distance = distance2D(p, t[i]);
    if (distance < min_distance)
    {
      min_distance = distance;
      closest_point = i;
    }
  }
  return min_distance;
}



};
};
#endif
