/*
 *  
 *
 *
 *  Created by Daniel Claes on 14.06.11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */


#include <pluginlib/class_list_macros.h>
#include "collvoid_local_planner/collvoid_local_planner.h"

using namespace std;
using namespace costmap_2d;

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_DECLARE_CLASS(collvoid_local_planner, CollvoidLocalPlanner, collvoid_local_planner::CollvoidLocalPlanner, nav_core::BaseLocalPlanner)

namespace collvoid_local_planner {
  

  template <typename T>
  void getParam (const ros::NodeHandle nh, const string& name, T* place)
  {
    bool found = nh.getParam(name, *place);
    ROS_ASSERT_MSG (found, "Did not find parameter %s", nh.resolveName(name).c_str());
  }

  CollvoidLocalPlanner::CollvoidLocalPlanner():
    initialized_(false)
 {
 }
  
  void CollvoidLocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros){
    if (!initialized_){
      tf_ = tf;
      costmap_ros_ = costmap_ros;
      ros::NodeHandle private_nh("~/" + name);


    } 


  } // end init
 

}; //end namespace
