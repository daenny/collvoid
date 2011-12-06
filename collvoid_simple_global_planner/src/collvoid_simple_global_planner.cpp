/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
* Authors: Daniel Claes
*********************************************************************/
#include "collvoid_simple_global_planner/collvoid_simple_global_planner.h"
#include <pluginlib/class_list_macros.h>


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_DECLARE_CLASS(collvoid_simple_global_planner, CollvoidSimpleGlobalPlanner, collvoid_simple_global_planner::CollvoidSimpleGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace collvoid_simple_global_planner {

  CollvoidSimpleGlobalPlanner::CollvoidSimpleGlobalPlanner()
  {}

  CollvoidSimpleGlobalPlanner::CollvoidSimpleGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
    initialize(name, costmap_ros);
  }
  
  void CollvoidSimpleGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
  }

  bool CollvoidSimpleGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

    ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();
    plan.push_back(start);
    double x, y, dir_x, dir_y;
    dir_x = goal.pose.position.x - start.pose.position.x;
    dir_y = goal.pose.position.y - start.pose.position.y;
    double length = sqrt(dir_y * dir_y + dir_x * dir_x);
    dir_x /= length;
    dir_y /= length;
    x = start.pose.position.x + 0.1 * dir_x;
    y = start.pose.position.y + 0.1 * dir_y;
    ROS_DEBUG("dir: %.2f, %.2f, cur: %.2f, %.2f", dir_x, dir_y, x, y);

    while (fabs(x-goal.pose.position.x) > 0.2 || fabs(y-goal.pose.position.y) > 0.2) {
      geometry_msgs::PoseStamped point;
      point.header = goal.header;
      point.pose.position.x = x;
      point.pose.position.y = y;
      point.pose.orientation.w = 1;
      plan.push_back(point);
      x += 0.1 * dir_x;
      y += 0.1 * dir_y;
    }
    plan.push_back(goal);

    return true;
  }

};
