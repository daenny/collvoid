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

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <collvoid_msgs/PoseArrayWeighted.h>
#include <geometry_msgs/PolygonStamped.h>

#include <collvoid_msgs/PoseTwistWithCovariance.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>

#include <collvoid_local_planner/Agent.h>


namespace collvoid {
  struct Obstacle{
    Vector2 point1;
    Vector2 point2;
    ros::Time last_seen;
  };
 
  class ROSAgent : public Agent {
  public:
    typedef boost::shared_ptr<ROSAgent> ROSAgentPtr;

    ROSAgent();

    virtual ~ROSAgent() {};

    void init(ros::NodeHandle private_nh, tf::TransformListener* tf);

    void initParams(ros::NodeHandle private_nh);
    
    void initAsMe(tf::TransformListener* tf);
    void initCommon(ros::NodeHandle nh);

    void computeNewVelocity(Vector2 pref_velocity, geometry_msgs::Twist& cmd_vel);


    void computeOrcaVelocity(Vector2 pref_velocity);
    void computeClearpathVelocity(Vector2 pref_velocity);
    void computeSampledVelocity(Vector2 pref_velocity);

    
    void addNHConstraints(double min_dist, Vector2 pref_velocity);
    
    void addInflatedObstacleFromLine(Vector2 start, Vector2 end, ros::Time stamp);
    
    void computeObstacles();

    
    bool compareNeighborsPositions(const AgentPtr& agent1, const AgentPtr& agent2); 

    bool compareConvexHullPointsPosition(const ConvexHullPoint& p1, const ConvexHullPoint& p2);
    bool compareVectorPosition(const collvoid::Vector2& v1, const collvoid::Vector2& v2);

    bool isInStaticObstacle();
    void sortObstacleLines();

    collvoid::Vector2 LineSegmentToLineSegmentIntersection(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4);

    bool pointInNeighbor(collvoid::Vector2& point);

    double getDistToFootprint(collvoid::Vector2& point);
    void computeObstacleLine(Vector2& point);
    void createObstacleLine(std::vector<Vector2>& own_footprint, Vector2& obst1, Vector2& obst2);

  
    void setFootprint(geometry_msgs::PolygonStamped footprint);
    void setFootprintRadius(double footprint_radius); 
    void setMinkowskiFootprintVector2(geometry_msgs::PolygonStamped minkowski_footprint); 

    void setIsHoloRobot(bool holo_robot); 
    void setRobotBaseFrame(std::string base_link);
    void setGlobalFrame(std::string global_frame);
    void setId(std::string id);

    void setMaxVelWithObstacles(double max_vel_with_obstacles);
    void setWheelBase(double wheel_base);

    void setAccelerationConstraints(double acc_lim_x, double acc_lim_y, double acc_lim_th);

    void setMinMaxSpeeds(double min_vel_x, double max_vel_x, double min_vel_y, double max_vel_y, double min_vel_th, double max_vel_th, double min_vel_th_inplace);
    

    void setPublishPositionsPeriod(double publish_positions_period);
    void setPublishMePeriod(double publish_me_period);

    void setTimeToHolo(double time_to_holo);
    void setTimeHorizonObst(double time_horizon_obst);
    void setMinMaxErrorHolo(double min_error_holo, double max_error_holo);
    void setDeleteObservations(bool delete_observations);
    void setThresholdLastSeen(double threshold_last_seen); //implement!!
    void setLocalizationEps(double eps);

    void setTypeVO(int type_vo);

    void setOrca(bool orca);
    void setClearpath(bool clearpath);
    void setConvex(bool convex);
    void setUseTruncation(bool use_truncation);
    void setNumSamples(int num_samples);
        
    bool isHoloRobot();
    ros::Time lastSeen();


    
    void positionShareCallback(const collvoid_msgs::PoseTwistWithCovariance::ConstPtr& msg); 
    void amclPoseArrayWeightedCallback(const collvoid_msgs::PoseArrayWeighted::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void updateAllNeighbors();
    void setAgentParams(ROSAgent* agent);

    
    double vMaxAng();

    void publishMePoseTwist();

    geometry_msgs::PolygonStamped createFootprintMsgFromVector2(const std::vector<Vector2>& footprint);
    std::vector<Vector2> rotateFootprint(const std::vector<Vector2>& footprint, double angle);
    
    geometry_msgs::PoseStamped transformMapPoseToBaseLink(geometry_msgs::PoseStamped in);

    void computeNewLocUncertainty();
    
    void computeNewMinkowskiFootprint();

    void baseScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    //config
    double publish_positions_period_;
    double publish_me_period_;

    double threshold_last_seen_;
    int num_samples_;
    
    //helpers
    ros::Time last_time_positions_published_;
    ros::Time last_time_me_published_;
    
    //NH stuff
    double min_error_holo_;
    double max_error_holo_;
    bool holo_robot_;
    double time_to_holo_;
    
    //ORCA stuff
    double max_vel_with_obstacles_;
    collvoid::Vector2 holo_velocity_;


    //Obstacles
    laser_geometry::LaserProjection projector_;

    //Obstacles    
    std::vector<Obstacle> obstacles_from_laser_;
    //Obstacle Center
    std::vector<Vector2> obstacle_centers_;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub_;
    boost::shared_ptr<tf::MessageFilter<sensor_msgs::LaserScan> > laser_notifier;
    
    double min_dist_obst_;
      
    //obstacles
    bool delete_observations_;
    bool use_obstacles_;
    std::vector<collvoid::Vector2> obstacle_points_;
    double time_horizon_obst_;

    //Agent description
    std::string id_;
    std::string base_frame_, global_frame_;
    double wheel_base_;
    geometry_msgs::PolygonStamped footprint_msg_;
    double acc_lim_x_, acc_lim_y_, acc_lim_th_;
    double min_vel_x_, max_vel_x_, min_vel_y_, max_vel_y_, max_vel_th_, min_vel_th_, min_vel_th_inplace_;
    double footprint_radius_;


    bool standalone_;
    //set automatically
    bool initialized_;
    bool has_polygon_footprint_;

    std::vector< std::pair< collvoid::Vector2,collvoid::Vector2 > > footprint_lines_;

    
    ros::Time last_seen_;
    nav_msgs::Odometry base_odom_; 
    
    //LOC uncertatiny
    double eps_;
    double cur_loc_unc_radius_;

    //COLLVOID
    std::vector<Vector2> minkowski_footprint_;
    std::vector<std::pair<double, geometry_msgs::PoseStamped> > pose_array_weighted_;

    boost::mutex me_lock_, obstacle_lock_, neighbors_lock_, convex_lock_;

    //me stuff
    tf::TransformListener* tf_;
    
    //subscribers and publishers
    ros::Publisher lines_pub_, neighbors_pub_, polygon_pub_, vo_pub_, me_pub_, samples_pub_, speed_pub_, position_share_pub_, obstacles_pub_;
    ros::Subscriber amcl_posearray_sub_, position_share_sub_, odom_sub_;

    
  };

    typedef boost::shared_ptr<ROSAgent> ROSAgentPtr;


}
#endif
