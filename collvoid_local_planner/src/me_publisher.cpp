//
// Created by danielclaes on 23/06/15.
//

#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <costmap_2d/array_parser.h>
#include <costmap_2d/footprint.h>
#include "collvoid_local_planner/me_publisher.h"


template<typename T>
void getParam(const ros::NodeHandle nh, const std::string &name, T *place) {
    bool found = nh.getParam(name, *place);
    ROS_ASSERT_MSG (found, "Could not find parameter %s", name.c_str());
    ROS_DEBUG_STREAM_NAMED ("init", std::string("Initialized ") << name << " to " << *place);
}

template<class T>
T getParamDef(const ros::NodeHandle nh, const std::string &name, const T &default_val) {
    T val;
    nh.param(name, val, default_val);
    ROS_DEBUG_STREAM_NAMED ("init", std::string("Initialized ") << name << " to " << val <<
                                                   "(default was " << default_val << ")");
    return val;
}

using namespace collvoid;

MePublisher::MePublisher() {
}

void MePublisher::init(ros::NodeHandle nh, tf::TransformListener *tf) {
    tf_ = tf;
    ros::NodeHandle ns_nh("move_base/local_costmap");
    ros::NodeHandle private_nh("collvoid");

    //set my id
    my_id_ = nh.getNamespace();
    if (strcmp(my_id_.c_str(), "/") == 0) {
        char hostname[1024];
        hostname[1023] = '\0';
        gethostname(hostname, 1023);
        my_id_ = std::string(hostname);
    }
    // remove funky "/" to get uniform name in python and here
    my_id_.erase(std::remove(my_id_.begin(), my_id_.end(), '/'), my_id_.end());

    // agent params
    my_id_ = getParamDef<std::string>(private_nh, "name", my_id_);


    private_nh.param<std::string>("base_frame", base_frame_, nh.getNamespace() + "/base_link");
    private_nh.param<std::string>("global_frame", global_frame_, "map");


    eps_ = getParamDef(private_nh, "eps", 0.1);
    ROS_INFO("My name is: %s, Eps: %f", my_id_.c_str(), eps_);
    getParam(private_nh, "use_polygon_footprint", &use_polygon_footprint_);
    getParam(private_nh, "holo_robot", &holo_robot_);
    controlled_ = getParamDef(private_nh, "controlled", true);

    publish_me_period_ = getParamDef(private_nh, "publish_me_frequency", 10.0);
    publish_me_period_ = 1.0 / publish_me_period_;
    cur_loc_unc_radius_ = 0.;
    getFootprint(ns_nh);

    //Publishers
    me_pub_ = nh.advertise<visualization_msgs::MarkerArray>("me", 1, true);
    polygon_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("convex_hull", 1, true);
    position_share_pub_ = nh.advertise<collvoid_msgs::PoseTwistWithCovariance>("/position_share", 1);


    //Subscribers
    particle_sub_= nh.subscribe("particlecloud_weighted", 1, &MePublisher::amclPoseArrayWeightedCallback, this);
    odom_sub_ = nh.subscribe("odom", 1, &MePublisher::odomCallback, this);

    //Service provider
    server_ = nh.advertiseService("get_me", &MePublisher::getMeCB, this);

}

bool MePublisher::getMeCB(collvoid_srvs::GetMe::Request &req, collvoid_srvs::GetMe::Response &res){
    boost::mutex::scoped_lock(me_lock_);
    collvoid_msgs::PoseTwistWithCovariance me_msg;
    if (createMeMsg(me_msg, global_frame_)) {
        res.me = me_msg;
        return true;
    }
    else {
        return false;
    }
}


void MePublisher::amclPoseArrayWeightedCallback(const collvoid_msgs::PoseArrayWeighted::ConstPtr &msg) {
    boost::mutex::scoped_lock lock(convex_lock_);

    pose_array_weighted_.clear();
    sensor_msgs::PointCloud result;
    //in.header = msg->header;
    sensor_msgs::PointCloud pc;
    pc.header = msg->header;
    pc.header.stamp = ros::Time(0);
    for (int i = 0; i < (int) msg->poses.size(); i++) {
        geometry_msgs::Point32 p;
        p.x = msg->poses[i].position.x;
        p.y = msg->poses[i].position.y;
        pc.points.push_back(p);
    }
    try {
        tf_->waitForTransform(global_frame_, base_frame_, pc.header.stamp, ros::Duration(0.2));
        tf_->transformPointCloud(base_frame_, pc, result);

    }
    catch (tf::TransformException ex) {
        ROS_WARN("%s", ex.what());
        ROS_WARN("Me Publisher: AMCL callback point transform failed");
        return;
    };
    for (int i = 0; i < (int) msg->poses.size(); i++) {
        pose_array_weighted_.push_back(std::make_pair(msg->weights[i], result.points[i]));
    }
    //    if (!use_polygon_footprint_ || orca_) {
    if (!use_polygon_footprint_) {
        computeNewLocUncertainty();
    }
    else {
        computeNewMinkowskiFootprint();
    }

}

void MePublisher::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    //we assume that the odometry is published in the frame of the base
    boost::mutex::scoped_lock(me_lock_);
    twist_.linear.x = msg->twist.twist.linear.x;
    twist_.linear.y = msg->twist.twist.linear.y;
    twist_.angular.z = msg->twist.twist.angular.z;

    last_seen_ = msg->header.stamp;


    if ((ros::Time::now() - last_time_me_published_).toSec() > publish_me_period_) {
        last_time_me_published_ = ros::Time::now();
        publishMePoseTwist();

        tf::Stamped <tf::Pose> global_pose;
        if (getGlobalPose(global_pose, global_frame_, msg->header.stamp)) {
            collvoid::publishMePosition(radius_, global_pose, global_frame_, base_frame_, me_pub_);
        }
        //publish footprint
        if (use_polygon_footprint_) {
            geometry_msgs::PolygonStamped msg_pub = createFootprintMsgFromVector2(minkowski_footprint_);
            polygon_pub_.publish(msg_pub);
        }
    }
}


bool MePublisher::getGlobalPose(tf::Stamped <tf::Pose> &global_pose, std::string target_frame, const ros::Time stamp) {
    //let's get the pose of the robot in the frame of the plan
    global_pose.setIdentity();
    global_pose.frame_id_ = base_frame_;
    global_pose.stamp_ = stamp;
    //global_pose.setRotation(tf::createIdentityQuaternion());
    try {
        tf_->waitForTransform(target_frame, base_frame_, global_pose.stamp_, ros::Duration(0.2));
        tf_->transformPose(target_frame, global_pose, global_pose);
    }
    catch (tf::TransformException ex) {
        ROS_WARN_THROTTLE(2,"%s", ex.what());
        ROS_WARN_THROTTLE(2, "point odom transform failed");
        return false;
    };
    return true;

}

bool MePublisher::createMeMsg(collvoid_msgs::PoseTwistWithCovariance &me_msg, std::string target_frame) {
    me_msg.header.stamp = ros::Time::now();
    me_msg.header.frame_id = target_frame;
    tf::Stamped <tf::Pose> global_pose;
    if (getGlobalPose(global_pose, target_frame, me_msg.header.stamp)) {
        geometry_msgs::PoseStamped pose_msg;
        tf::poseStampedTFToMsg(global_pose, pose_msg);
        me_msg.pose.pose = pose_msg.pose;
    }
    else {
        return false;
    }
    me_msg.twist.twist = twist_;

    me_msg.controlled = controlled_;
    me_msg.holonomic_velocity.x = holo_velocity_.x();
    me_msg.holonomic_velocity.y = holo_velocity_.y();

    me_msg.holo_robot = holo_robot_;
    me_msg.radius = (float)(uninflated_robot_radius_ + cur_loc_unc_radius_);
    me_msg.robot_id = my_id_;
    me_msg.controlled = controlled_;

    me_msg.footprint = createFootprintMsgFromVector2(minkowski_footprint_);

    return true;
}


void MePublisher::publishMePoseTwist() {
    collvoid_msgs::PoseTwistWithCovariance me_msg;
    if (createMeMsg(me_msg, global_frame_)) {
        position_share_pub_.publish(me_msg);
    }
}

void MePublisher::computeNewMinkowskiFootprint() {
    bool done = false;
    std::vector<ConvexHullPoint> convex_hull;
    std::vector<ConvexHullPoint> points;
    points.clear();


    for (int i = 0; i < (int) pose_array_weighted_.size(); i++) {
        ConvexHullPoint p;
        p.point = Vector2(pose_array_weighted_[i].second.x,
                          pose_array_weighted_[i].second.y);
        p.weight = pose_array_weighted_[i].first;
        p.index = i;
        p.orig_index = i;
        points.push_back(p);
    }
    std::sort(points.begin(), points.end(), compareVectorsLexigraphically);
    for (int i = 0; i < (int) points.size(); i++) {
        points[i].index = i;
    }

    double total_sum = 0;
    std::vector<int> skip_list;

    while (!done && points.size() >= 3) {
        double sum = 0;
        convex_hull.clear();
        convex_hull = convexHull(points, true);

        skip_list.clear();
        for (int j = 0; j < (int) convex_hull.size() - 1; j++) {
            skip_list.push_back(convex_hull[j].index);
            sum += convex_hull[j].weight;
        }
        total_sum += sum;

        //ROS_WARN("SUM = %f (%f), num Particles = %d, eps = %f", sum, total_sum, (int) points.size(), eps_);

        if (total_sum >= eps_) {
            done = true;
            break;
        }

        std::sort(skip_list.begin(), skip_list.end());
        for (int i = (int) skip_list.size() - 1; i >= 0; i--) {
            points.erase(points.begin() + skip_list[i]);
        }

        for (int i = 0; i < (int) points.size(); i++) {
            points[i].index = i;
        }
    }

    std::vector<Vector2> localization_footprint, own_footprint;
    for (int i = 0; i < (int) convex_hull.size(); i++) {
        localization_footprint.push_back(convex_hull[i].point);
    }

    for(geometry_msgs::Point32 p: footprint_msg_.polygon.points) {
        own_footprint.push_back(Vector2(p.x, p.y));
        //      ROS_WARN("footprint point p = (%f, %f) ", footprint_[i].x, footprint_[i].y);
    }
    minkowski_footprint_ = minkowskiSum(localization_footprint, own_footprint);



}

void MePublisher::computeNewLocUncertainty() {
    std::vector<ConvexHullPoint> points;

    for (int i = 0; i < (int) pose_array_weighted_.size(); i++) {
        ConvexHullPoint p;
        p.point = Vector2(pose_array_weighted_[i].second.x,
                          pose_array_weighted_[i].second.y);
        p.weight = pose_array_weighted_[i].first;
        p.index = i;
        p.orig_index = i;
        points.push_back(p);
    }

    std::sort(points.begin(), points.end(), boost::bind(&MePublisher::compareConvexHullPointsPosition, this, _1, _2));
    double sum = 0.0;
    int j = 0;
    while (sum <= 1.0 - eps_ && j < (int) points.size()) {
        sum += points[j].weight;
        j++;
    }
    cur_loc_unc_radius_ = std::min(uninflated_robot_radius_ * 2.0, collvoid::abs(points[j - 1].point));
    //ROS_ERROR("Loc Uncertainty = %f", cur_loc_unc_radius_);
    radius_ = uninflated_robot_radius_ + cur_loc_unc_radius_;
}

bool MePublisher::compareConvexHullPointsPosition(const ConvexHullPoint &p1, const ConvexHullPoint &p2) {
    return collvoid::absSqr(p1.point) <= collvoid::absSqr(p2.point);
}

void MePublisher::setMinkowskiFootprintVector2(geometry_msgs::PolygonStamped minkowski_footprint) {
    minkowski_footprint_.clear();
    for(geometry_msgs::Point32 p: minkowski_footprint.polygon.points) {
        minkowski_footprint_.push_back(Vector2(p.x, p.y));
    }
}

geometry_msgs::PolygonStamped MePublisher::createFootprintMsgFromVector2(const std::vector<Vector2> &footprint) {
    geometry_msgs::PolygonStamped result;
    result.header.frame_id = base_frame_;
    result.header.stamp = ros::Time::now();

    for(Vector2 point: footprint) {
        geometry_msgs::Point32 p;
        p.x = point.x();
        p.y = point.y();
        result.polygon.points.push_back(p);
    }

    return result;
}


void MePublisher::getFootprint(ros::NodeHandle private_nh){
    getParam(private_nh, "robot_radius", &uninflated_robot_radius_);
    radius_ = uninflated_robot_radius_;
    std::string full_param_name;
    std::vector< geometry_msgs::Point > footprint;
    if( private_nh.searchParam( "footprint", full_param_name ))
    {
        XmlRpc::XmlRpcValue footprint_xmlrpc;
        private_nh.getParam( full_param_name, footprint_xmlrpc );
        if( footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeString &&
            footprint_xmlrpc != "" && footprint_xmlrpc != "[]" )
        {
            if(!costmap_2d::makeFootprintFromString( std::string( footprint_xmlrpc ), footprint))
                ROS_ERROR("Could not read footprint");
        }
        else if( footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray )
        {
            footprint = costmap_2d::makeFootprintFromXMLRPC( footprint_xmlrpc, full_param_name );
        }
    }
    else {
        footprint = costmap_2d::makeFootprintFromRadius(radius_);
    }
    for (size_t i=0; i<footprint.size(); ++i) {
        geometry_msgs::Point32 pt;
        pt.x = (float)footprint.at(i).x;
        pt.y = (float)footprint.at(i).y;
        pt.z = (float)footprint.at(i).z;
        footprint_msg_.polygon.points.push_back(pt);
    }
    setMinkowskiFootprintVector2(footprint_msg_);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "me_publisher");
    //ros::NodeHandle nh;
    ros::NodeHandle nh;

    boost::shared_ptr<MePublisher> me(new MePublisher());
    tf::TransformListener tf;
    me->init(nh, &tf);
    ROS_INFO("ROS MePublisher initialized");
    ros::spin();

}