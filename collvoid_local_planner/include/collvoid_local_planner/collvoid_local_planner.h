
#ifndef COLLVOID_LOCAL_PLANNER_H_
#define COLLVOID_LOCAL_PLANNER_H_


#include <tf/transform_listener.h>
#include <nav_core/base_local_planner.h>

#include <nav_msgs/Odometry.h>

namespace collvoid_local_planner {

  class CollvoidLocalPlanner: public nav_core::BaseLocalPlanner {
  public:
    CollvoidLocalPlanner();
    void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
    bool isGoalReached();
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  private:



    bool rotateToGoal(const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, double goal_th, geometry_msgs::Twist& cmd_vel);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    std::vector<geometry_msgs::PoseStamped> global_plan_;

    bool initialized_;

    double max_vel_x_, min_vel_x_;
    double max_vel_y_, min_vel_y_;
    double max_vel_th_, min_vel_th_;
    double acc_lim_x_, acc_lim_y_, acc_lim_theta_;

    

    //Datatypes:
    costmap_2d::Costmap2DROS* costmap_ros_; 
    tf::TransformListener* tf_; 

  };//end class

}; //end namespace

#endif //end catch
