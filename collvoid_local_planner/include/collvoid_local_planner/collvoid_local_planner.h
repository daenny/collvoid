
#ifndef COLLVOID_LOCAL_PLANNER_H_
#define COLLVOID_LOCAL_PLANNER_H_


#include <tf/transform_listener.h>
#include <nav_core/base_local_planner.h>
#include <angles/angles.h>
#include <nav_msgs/Odometry.h>

namespace collvoid_local_planner {

  class CollvoidLocalPlanner: public nav_core::BaseLocalPlanner {
  public:
    CollvoidLocalPlanner();
    CollvoidLocalPlanner(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
    ~CollvoidLocalPlanner();
    void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
    bool isGoalReached();
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  private:



    bool rotateToGoal(const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, double goal_th, geometry_msgs::Twist& cmd_vel);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    double sign(double x){
      return x < 0.0 ? -1.0 : 1.0;
    }


    //Datatypes:
    costmap_2d::Costmap2DROS* costmap_ros_; 
    tf::TransformListener* tf_; 

    bool initialized_;

    double sim_period_;
    double max_vel_x_, min_vel_x_;
    double max_vel_y_, min_vel_y_;
    double max_vel_th_, min_vel_th_;
    double acc_lim_x_, acc_lim_y_, acc_lim_theta_;
    double xy_goal_tolerance_, yaw_goal_tolerance_;

    double rot_stopped_velocity_, trans_stopped_velocity_;
  

    double inscribed_radius_, circumscribed_radius_, inflation_radius_; 


    std::string global_frame_; ///< @brief The frame in which the controller will run
    std::string robot_base_frame_; ///< @brief Used as the base frame id of the robot


    std::vector<geometry_msgs::PoseStamped> global_plan_;

    
    bool holo_robot_;


    ros::Subscriber odom_sub_;
    nav_msgs::Odometry base_odom_; ///< @brief Used to get the velocity of the robot
    boost::recursive_mutex odom_lock_;
   

    


  };//end class

}; //end namespace

#endif //end guard catch
