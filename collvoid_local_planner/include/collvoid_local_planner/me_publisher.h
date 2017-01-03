//
// Created by danielclaes on 23/06/15.
//

#ifndef COLLVOID_LOCAL_PLANNER_ME_PUBLISHER_H
#define COLLVOID_LOCAL_PLANNER_ME_PUBLISHER_H
#include <ros/ros.h>

#include <collvoid_msgs/PoseArrayWeighted.h>
#include <collvoid_msgs/PoseTwistWithCovariance.h>

#include <collvoid_srvs/GetMe.h>

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>

#include "collvoid_local_planner/clearpath.h"
#include "collvoid_local_planner/Utils.h"
#include "collvoid_local_planner/Vector2.h"
#include "collvoid_local_planner/collvoid_publishers.h"


using namespace collvoid;

class MePublisher {
public:
    MePublisher();
    virtual ~MePublisher() { };
    void init(ros::NodeHandle private_nh, tf::TransformListener *tf);

private:
    void computeNewMinkowskiFootprint();
    void computeNewLocUncertainty();
    void amclPoseArrayWeightedCallback(const collvoid_msgs::PoseArrayWeighted::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void publishMePoseTwist();
    bool compareConvexHullPointsPosition(const ConvexHullPoint &p1, const ConvexHullPoint &p2);
    void setMinkowskiFootprintVector2(geometry_msgs::PolygonStamped minkowski_footprint);
    geometry_msgs::PolygonStamped createFootprintMsgFromVector2(const std::vector<Vector2> &footprint);
    void getFootprint(ros::NodeHandle private_nh);
    bool getMeCB(collvoid_srvs::GetMe::Request &req, collvoid_srvs::GetMe::Response &res);

    bool getGlobalPose(tf::Stamped <tf::Pose> &global_pose, std::string target_frame, const ros::Time stamp);
    bool createMeMsg(collvoid_msgs::PoseTwistWithCovariance &me_msg, std::string target_frame);


    //Agent description
    std::string my_id_;
    std::string base_frame_, global_frame_;
    bool use_polygon_footprint_, holo_robot_, controlled_;
    Vector2 holo_velocity_;

    double uninflated_robot_radius_, radius_, cur_loc_unc_radius_;

    std::vector<Vector2> minkowski_footprint_;
    geometry_msgs::PolygonStamped footprint_msg_;


    double eps_; //localization epsilon



    double publish_me_period_;

    ros::Time last_seen_, last_time_me_published_;
    ros::Publisher position_share_pub_, polygon_pub_, me_pub_;
    ros::Subscriber odom_sub_, particle_sub_;
    ros::ServiceServer server_;
    tf::TransformListener *tf_;

    geometry_msgs::Twist twist_;

    boost::mutex convex_lock_;
    std::vector<std::pair<double, geometry_msgs::Point32> > pose_array_weighted_;

};

#endif //COLLVOID_LOCAL_PLANNER_ME_PUBLISHER_H
