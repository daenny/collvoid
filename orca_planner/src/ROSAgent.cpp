/*
 *  
 *
 *
 *  Created by Daniel Claes on 14.06.11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
//#include <std_msgs/Float32.h>

#include <visualization_msgs/Marker.h>

#include <boost/random.hpp>


#include "ROSAgent.h"
#include "orca_planner/PositionShare.h"
#include "orca_planner/OrcaDebug.h"

#include "orca_planner/StateSrv.h"


ROSAgent::ROSAgent() :
  private_nh(),
  tfListener(),
  maxSpeed_linear(),
  maxSpeed_angular(),
  minSpeed_linear(),
  minSpeed_angular(),
  maxNeighbors(),
  neighborDist(),
  timeHorizon(),
  timeHorizonObst(),
  radius(),
  radiusUncertainty(),
  obstacles_(),
  me(),
  robots(),
  goal(),
  myId(),
  tf_prefix(),
  state_(STOPPED),
  useGroundTruth(false),
  scaleRadius(),
  nrInitialGuess(0),
  lastTime(),
  debugMsgCounter(0),
  ground_truth(),odom(),
  locatedPose(),
  locError(),
  subOdom(),subOdomAll(),subCommands(),subPositionGroundTruth(), subGoal(),
  pubTwist(),pubOdom(),pubCommands(),pubInitialGuess(),pubLines(),pubDebug()
{
  this->init();
}

ROSAgent::~ROSAgent()
{
}

bool ROSAgent::atGoal(){
  if (sqrt((me.pos.x-goal.x)*(me.pos.x-goal.x) + (me.pos.y-goal.y )*(me.pos.y-goal.y) )< THRESHOLD) // && abs(me.pos.ang - goal.ang)<THRESHOLD)
    return true;
  return false;
}

double ROSAgent::vMaxAng(){
  return maxSpeed_angular - std::abs(me.twist.twist.angular.z) * WHEELBASE/2.0;
}

double ROSAgent::beta(double T, double theta){
  double vMax = vMaxAng();
  return - ((2.0 * RVO::sqr(T) * sin(theta)) / theta) * vMax;
}

double ROSAgent::gamma(double T, double theta, double error) {
  double vMax = vMaxAng();
  return (((2.0 * RVO::sqr(T) * (1.0 - cos(theta))) / RVO::sqr(theta)) * RVO::sqr(vMax) - RVO::sqr(error));
}

double ROSAgent::calcVstar(double vh, double theta){
  return vh * ((theta * sin(theta))/(2.0 * (1.0 - cos(theta))));
}

double ROSAgent::calcVstarError(double T,double theta, double error) {
  
  return calcVstar(error/T,theta) * sqrt((2.0*(1.0-cos(theta)))/(2.0*(1.0-cos(theta))-RVO::sqr(sin(theta))));
}

double ROSAgent::calculateMaxTrackSpeedAngle(double T, double theta, double error){
  if (theta/T <= maxSpeed_angular) {
    double vstarError = calcVstarError(T,theta,error);
    if (vstarError <=vMaxAng()) { //include v_max-w(t)
      return std::min(vstarError * (2.0 * (1.0 - cos(theta))) / (theta*sin(theta)),maxSpeed_linear);
    }
    else {
      double b, g;
      b = beta(T,theta);
      g = gamma(T,theta,error);
      return std::min((-b+sqrt(RVO::sqr(b)-4*RVO::sqr(T)*g))/ (2.0 * g), maxSpeed_linear);
    }
  }
  else
    return std::min(error*maxSpeed_angular/theta,maxSpeed_linear);
}


void ROSAgent::cbCommandsRobot(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard %s",msg->data.c_str());
  if (strcmp(msg->data.c_str(),"all Start") ==0 || strcmp(msg->data.c_str(), std::string(myId + " Start").c_str()) ==0 ) {
    state_ = RUNNING;
    // commandStart = true;
  }
  
  if (strcmp(msg->data.c_str(),"all Stop") ==0 || strcmp(msg->data.c_str(), std::string(myId + " Stop").c_str()) ==0 ) {
    state_ = STOPPED;
    //commandStart = false;
    geometry_msgs::Twist cmd;
    cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
    pubTwist.publish(cmd);
  }
  if (strcmp(msg->data.c_str(),"all Restart") ==0 || strcmp(msg->data.c_str(), std::string(myId + " Restart").c_str()) ==0 ) {
    // goal.x = -me.pos.x;
    // goal.y = -me.pos.y;
    robots.clear();
    // sendFinish = false;
    nrInitialGuess = 0;
    // commandStart = false;
    // sendFinish = false;
    
    state_ = STOPPED;

    // stop robot to be safe
    geometry_msgs::Twist cmd;
    cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
    pubTwist.publish(cmd);
    locError = 0;
    debugMsgCounter++;
    ground_truth = odom = nav_msgs::OdometryPtr();
    locatedPose = geometry_msgs::PoseStamped();
    me.pos.x = me.pos.y= 0;
  }
	
}

bool ROSAgent::cb_state_srv(orca_planner::StateSrv::Request &req,
			    orca_planner::StateSrv::Response &res) {
  if (req.state) {
    state_ = RUNNING;
  }
  else {
    state_ = STOPPED;
    geometry_msgs::Twist cmd;
    cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
    pubTwist.publish(cmd);
  }

  return true;

}

void ROSAgent::cbGoal(const geometry_msgs::PoseStamped::ConstPtr& msg){
  goal.x = msg->pose.position.x;
  goal.y = msg->pose.position.y;
  goal.ang = tf::getYaw(msg->pose.orientation);

  ROS_INFO("got new goal!");

  //sendFinish = false;
} 


void ROSAgent::cbPositionGroundTruth(nav_msgs::OdometryPtr msg){
  ground_truth = msg;
  double trueX,trueY,trueAng;
  trueX = -msg->pose.pose.position.y;
  trueY = msg->pose.pose.position.x;
  trueAng = tf::getYaw(msg->pose.pose.orientation)+M_PI/2.0;
  if (useGroundTruth) {
    me.pos.x = trueX;
    me.pos.y =  trueY;
    me.pos.ang = trueAng;
    me.agent->position_ = RVO::Vector2(me.pos.x,me.pos.y);
    me.agent->velocity_ = RVO::Vector2(-msg->twist.twist.linear.y,msg->twist.twist.linear.x);
    me.twist = msg->twist;
    me.twist.twist.linear.x = me.agent->velocity_.x();
    me.twist.twist.linear.y = me.agent->velocity_.y();
    me.lastSeen = msg->header.stamp;
  }
  
  if (state_ != RUNNING) {
    if (nrInitialGuess == 0)
      ROS_INFO("publishing initial guess ...");
    else if (nrInitialGuess == NR_INITIAL_GUESS)
      ROS_INFO("finished publishing initial guess!");
    else if (nrInitialGuess > NR_INITIAL_GUESS)
      return;
    
    geometry_msgs::PoseWithCovarianceStamped msgSent;
    //add initial gaussian noise
    msgSent.pose.pose.position.x = -msg->pose.pose.position.y+sampleNormal(0.0, NOISE_STD);
    msgSent.pose.pose.position.y =  msg->pose.pose.position.x+sampleNormal(0.0, NOISE_STD);
    msgSent.pose.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(msg->pose.pose.orientation)+M_PI/2.0);
    msgSent.header.frame_id = "/map";
    pubInitialGuess.publish(msgSent);
    nrInitialGuess++;
  
  } else {
    //lookup tf from map to baselink
    try{
      tf::StampedTransform transform;
      tfListener.waitForTransform("/map", tf_prefix+"/base_link",
				  msg->header.stamp, ros::Duration(0.5));
      tfListener.lookupTransform("/map", tf_prefix+"/base_link",  
				 msg->header.stamp, transform);
      geometry_msgs::Pose pose;
      tf::poseTFToMsg(transform, pose);
      locError = sqrt(RVO::sqr(trueX - pose.position.x)+RVO::sqr(trueY - pose.position.y));
      
      locatedPose.pose.position.x = trueX;
      locatedPose.pose.position.y = trueY;
      locatedPose.pose.orientation = tf::createQuaternionMsgFromYaw(trueAng);
      
      locatedPose.header = msg->header;
      // std_msgs::Float32 errMsg;
      // errMsg.data = locError;
      // pubLocError.publish(errMsg);

    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
  }
}

void ROSAgent::cbOdom(nav_msgs::OdometryPtr msg){
  odom = msg;
  //  if (state_ == STOPPED)
  // return;
  
  tf::StampedTransform transform;
	
  //lookup tf from map to baselink
  try{
    tfListener.waitForTransform("/map", tf_prefix+"/base_link",
				msg->header.stamp, ros::Duration(0.5));
    tfListener.lookupTransform("/map", tf_prefix+"/base_link",  
			       msg->header.stamp, transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return;
  };

  orca_planner::PositionShare msgShare;
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(transform, pose);
  
  msgShare.pose.pose = pose;
  msgShare.header = msg->header;
  msgShare.twist.twist = msg->twist.twist;
  
  if (useGroundTruth){
    msgShare.pose.pose.position.x = me.pos.x;
    msgShare.pose.pose.position.y = me.pos.y;
    msgShare.pose.pose.orientation = tf::createQuaternionMsgFromYaw(me.pos.ang);
    msgShare.twist.twist = me.twist.twist;    
  }
  else {
    //Set position to cur position from odom
    me.pos.ang = tf::getYaw(pose.orientation);
    me.pos.x = pose.position.x;
    me.pos.y = pose.position.y;
    me.agent->position_=RVO::Vector2(me.pos.x,me.pos.y);
    me.twist = msg->twist;
    me.lastSeen = msg->header.stamp;
    me.agent->velocity_ = RVO::Vector2(cos(me.pos.ang)*msg->twist.twist.linear.x, sin(me.pos.ang)*msg->twist.twist.linear.x);
    locatedPose.pose = pose;
    locatedPose.header = msg->header;
  }


  //msgShare.twist.twist.linear.x = me.agent->velocity_.x();
  //msgShare.twist.twist.linear.y = me.agent->velocity_.y();
    
  msgShare.holoVelocity.x = me.agent->velocity_.x();
  msgShare.holoVelocity.y = me.agent->velocity_.y();
  
 
  msgShare.radius = me.agent->radius_;
 
  msgShare.id = myId;

  pubOdom.publish(msgShare);
  //  ROS_ERROR("%s,Odom twist: x: %f, y: %f z: %f", myId.c_str(),curSpeedLin, msg->twist.twist.linear.y, curSpeedAng);
  // ROS_ERROR("I am at: [%f, %f ang= %f]", me.pos.x, me.pos.y, me.pos.ang);
  // ROS_INFO("Gaol at: [%f, %f ang= %f]", goal.x, goal.y, goal.ang);
}

void ROSAgent::cbAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  double covX, covY, covAng;
  covX = msg->pose.covariance[0];
  covY = msg->pose.covariance[7];
  covAng = msg->pose.covariance[20];
  if (!useGroundTruth && scaleRadius) {
    radiusUncertainty = std::max(sqrt(covX),sqrt(covY));
    if (radiusUncertainty == radiusUncertainty)
      me.agent->radius_ = std::min(2*radius,radius+radiusUncertainty);
    else
      radiusUncertainty = -1;
    
   
    ROS_INFO("%s, radius (uncert): %6.4f (%6.4f)", myId.c_str(),me.agent->radius_,radiusUncertainty);
  }

 }


void ROSAgent::updateAgentsPositions(const orca_planner::PositionShareConstPtr& msg) {
  int i;
  
  std::string curId = msg->id;
  if (strcmp(curId.c_str(),myId.c_str()) == 0) {
   
    return;
  }

  for(i=0; i < (int)robots.size(); i++)
    {
      if (strcmp(robots[i].id.c_str(),curId.c_str()) == 0) {
	//I found the robot
	break;
      }
		
    }
  if (i>=(int)robots.size()) { //Robot is new, so it will be added to the list
    Robot cur;
    cur.id = curId;
    RVO::Agent* neighbor = new RVO::Agent();
    neighbor->maxNeighbors_ = maxNeighbors;
    neighbor->maxSpeed_ = maxSpeed_linear;
    neighbor->neighborDist_ = neighborDist;

    neighbor->timeHorizon_ = timeHorizon;
    neighbor->timeHorizonObst_ = timeHorizonObst;
    cur.agent = neighbor;
    robots.push_back(cur);
  }
  //double difTime = (msg->header.stamp - robots[i].lastSeen).toSec();
  robots[i].lastSeen = msg->header.stamp;
  // RVO::Vector2 posChange = RVO::Vector2(msg->pose.pose.position.x-robots[i].pos.x, msg->pose.pose.position.y-robots[i].pos.y);
  robots[i].pos.x = msg->pose.pose.position.x;
  robots[i].pos.y = msg->pose.pose.position.y;
  robots[i].agent->position_= RVO::Vector2(robots[i].pos.x,robots[i].pos.y);

  robots[i].pos.ang = tf::getYaw(msg->pose.pose.orientation);
  robots[i].twist = msg->twist;
  robots[i].agent->radius_ = msg->radius;
  //robots[i].agent->velocity_ =  RVO::Vector2(msg->holoVelocity.x, msg->holoVelocity.y);
  //robots[i].agent->velocity_ = posChange / difTime;
  if(useGroundTruth)
    robots[i].agent->velocity_ = RVO::Vector2(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  else {
    robots[i].agent->velocity_ =  RVO::Vector2(cos(robots[i].pos.ang)*msg->twist.twist.linear.x, sin(robots[i].pos.ang)*msg->twist.twist.linear.x);
  }
}

void ROSAgent::addAllAgents(){
  me.agent->agentNeighbors_.clear();
  int i;
  for(i=0; i < (int)robots.size(); i++)
    {
      if (strcmp(robots[i].id.c_str(),myId.c_str()) != 0) {
	float rangeSq = RVO::sqr(me.agent->neighborDist_);
	double timeDif = (ros::Time::now()-robots[i].lastSeen).toSec();
	if (timeDif > THRESHOLD_LAST_SEEN)
	  continue;
	robots[i].agent->position_ = robots[i].agent->position_ + robots[i].agent->velocity_ * timeDif;
	//ROS_INFO("Position of robot i =%d at: [%f, %f] with vel = [%f, %f], timeDif = %f", i, robots[i].agent->position_.x(), robots[i].agent->position_.y(),robots[i].agent->velocity_.x(),robots[i].agent->velocity_.y(),timeDif);
			
	me.agent->insertAgentNeighbor(robots[i].agent,rangeSq);
      }
		
    }
	
}

void ROSAgent::update(){
  // i'm RUNNING
  geometry_msgs::Twist cmd;
  cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

  double difX, difY,ang,difAng;

  //interpolate position:
  double timeDif = (ros::Time::now()-me.lastSeen).toSec();
  me.agent->position_ = me.agent->position_ + me.agent->velocity_ * timeDif;
  me.agent->heading_ = me.pos.ang;
  //ROS_ERROR("timeDif %6.4f",timeDif);
  
  difX = goal.x - me.pos.x;
  difY = goal.y - me.pos.y;
  ang = atan2(difY,difX);
  limit_ang(&ang);
  //calculate deadreckoning to goal (could be better with global pathplanning to avoid static obstacles)
  RVO::Vector2 goalDir = RVO::Vector2(difX,difY);
  if (RVO::absSq(goalDir) > maxSpeed_linear) {
    goalDir = maxSpeed_linear*RVO::normalize(goalDir);
  }
  me.agent->prefVelocity_ = goalDir;

  addAllAgents();
  
  //ROS_INFO("I have %d neighbors",(int)me.agent->agentNeighbors_.size());

   me.agent->obstacleNeighbors_.clear();
   //   timeHorizonObst_ * maxSpeed_ + radius_
   /*   float rangeSq = RVO::sqr(me.agent->timeHorizonObst_ * me.agent->maxSpeed_ + me.agent->radius_);
   for (size_t i = 0; i < obstacles_.size(); i=i+2) {
     if(RVO::abs(obstacles_[i]->point_ - me.agent->position_) < RVO::abs(obstacles_[i+1]->point_-me.agent->position_))
       me.agent->insertObstacleNeighbor(obstacles_[i],rangeSq);
     else
        me.agent->insertObstacleNeighbor(obstacles_[i+1],rangeSq);
	}*/
  

  // compute NH-ORCA vellocity, this adds ORCA lines to set maxTrackSpeed
  double T = 0.4; //in how much time I want to be on the holonomic track

  double minError = 0.01;
  double maxError = 0.15;
  double length = RVO::abs(me.agent->velocity_);
  double error = minError + (maxError-minError) * length / vMaxAng(); // how much error do i allow?
  

  //  ROS_ERROR("error %6.4f", error);
  double minTheta = M_PI / 2.0;
  double maxTrackSpeed = calculateMaxTrackSpeedAngle(T,minTheta, error);
  me.agent->maxTrackSpeed_ = maxTrackSpeed;

  me.agent->maxSpeed_ = vMaxAng();
  me.agent->computeNewVelocity(); // compute ORCA velocity
  me.agent->velocity_ = me.agent->newVelocity_;
  ang = atan2(me.agent->newVelocity_.y(), me.agent->newVelocity_.x());
  difAng = ang - me.pos.ang;
  limit_ang(&difAng);

  //ROS_ERROR("%s x: %6.4f y: %6.4f ang: %6.4f goal.x: %6.4f goal.y: %6.4f speed.x: %6.4f speed.y: %6.4f goal_dif: %6.4f vel_ang_dif: %6.4f", myId.c_str(), me.pos.x, me.pos.y, me.pos.ang, goal.x, goal.y, me.agent->newVelocity_.x(), me.agent->newVelocity_.y(), goal_dif, difAng);

  double vel = RVO::abs(me.agent->newVelocity_);
  double vstar;
 
  if (std::abs(difAng) > 0.01)
    vstar = calcVstar(vel,difAng);
  else
    vstar = maxSpeed_linear;

  //cmd.linear.x = (vel)*cos(difAng);
  //cmd.angular.z = (vel)*sin(difAng);
    
  cmd.linear.x = std::min(vstar,vMaxAng());
  if (difAng/T > 0)
    cmd.angular.z = std::min(difAng/T,maxSpeed_angular);
  else 
    cmd.angular.z = std::max(difAng/T,-maxSpeed_angular);
  
  if (std::abs(cmd.angular.z) == maxSpeed_angular)
    cmd.linear.x = 0;
  //ROS_INFO("%s calc ang z=%6.4f vel = %6.4f",myId.c_str(),cmd.angular.z,vel);
  
  if(std::abs(cmd.angular.z)<minSpeed_angular)
    cmd.angular.z = 0.0;
  if(std::abs(cmd.linear.x)<minSpeed_linear)
    cmd.linear.x = 0.0;
  //ROS_INFO("%s calcVelo: x,y: (%6.4f, %6.4f) ",myId.c_str(),me.agent->velocity_.x(),me.agent->velocity_.y());
  //ROS_INFO("%s sendVelo: x,z: (%6.4f, %6.4f) ",myId.c_str(),cmd.linear.x,cmd.angular.z);

  // publish new velocity
  pubTwist.publish(cmd);

  me.agent->timeStep_ = std::abs((lastTime-ros::Time::now()).toSec());
  if (me.agent->timeStep_<0.1)
    me.agent->timeStep_ = 0.1;
  lastTime = ros::Time::now();

  // markers
  publishLines();

  // Publish debug Msg
  orca_planner::OrcaDebug msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = tf_prefix + "/base_link";
  msg.run = debugMsgCounter;
  msg.odom = *odom;
  msg.cmd_vel = cmd;
  msg.holo_vel.header.stamp = ros::Time::now();
  msg.holo_vel.vector.x = me.agent->velocity_.x();
  msg.holo_vel.vector.y = me.agent->velocity_.y();
  if (SIMULATION_MODE) {
    msg.ground_truth = *ground_truth;
    msg.loc_error = locError; //same timestamp as ground_truth
  }
  msg.located_pose = locatedPose;
  msg.radius_uncertainty = radiusUncertainty;
  pubDebug.publish(msg);
}

void ROSAgent::publishLines(){
  visualization_msgs::Marker line_list;
  //line_list.header.frame_id = myId + "/base_link";
  line_list.header.frame_id = "/map";
  line_list.header.stamp = ros::Time::now();
  line_list.ns = tf_prefix;
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.05;
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;
  geometry_msgs::Point p;
  p.x = me.pos.x;
  p.y = me.pos.y;
  p.z = 0.2;
  line_list.points.push_back(p);
  p.x = p.x + me.agent->velocity_.x();
  p.y = p.y + me.agent->velocity_.y();
  line_list.points.push_back(p);
  for (size_t i=0;i< me.agent->orcaLines_.size();i++) {
    geometry_msgs::Point p;
    p.x = me.pos.x + me.agent->orcaLines_[i].point.x() - me.agent->orcaLines_[i].direction.x();
    p.y = me.pos.y + me.agent->orcaLines_[i].point.y() - me.agent->orcaLines_[i].direction.y();
    
    line_list.points.push_back(p);
    p.x = p.x + 2 * me.agent->orcaLines_[i].direction.x();
    p.y = p.y + 2 * me.agent->orcaLines_[i].direction.y();
    line_list.points.push_back(p);
     
    //p.x = 2*me.agent->orcaLines_[i].point.x();
    //p.y = 2*me.agent->orcaLines_[i].point.y();
    //line_list.points.push_back(p);

    
    }
  pubLines.publish(line_list);
    
}


void ROSAgent::loop() {
  //ROS_INFO("Private nh: %s", private_nh.getNamespace().c_str());
  
  ros::Rate loop_rate(20);
  ROS_INFO("ROSAgent started looping ...");

  while (ros::ok()) {
    
      if (state_ == RUNNING) {
    
	if (atGoal()) {
	  // stop the robot
	  geometry_msgs::Twist cmd;
	  cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
	  pubTwist.publish(cmd);
      
	  // update state
	  state_ = AT_GOAL;
	} else {
	  this->update();
	}

      }

      ros::spinOnce();  
      loop_rate.sleep();
    
  } // END while
  ROS_INFO("ROSAgent stopped.");
}

std::vector<double> ROSAgent::load_param_list(std::string param_name) {
  std::vector<double> values; 
  try {
    XmlRpc::XmlRpcValue list;
    private_nh.getParam(param_name.c_str(), list);
    ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < list.size(); ++i) {
      ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      values.push_back(static_cast<double>(list[i]));
    }

  }
  catch (ros::Exception e) {
    ROS_ERROR("can't load param list (%s) : %s", param_name.c_str(), e.what());
  }
  return values;
}


void ROSAgent::init() {

  //Get Params
  private_nh.param("goal/x",goal.x,0.0);
  private_nh.param("goal/y",goal.y,0.0);
  private_nh.param("goal/ang",goal.ang,M_PI);
  private_nh.param("/useTruePos",useGroundTruth,false);
  private_nh.param("/maxNeighbors",maxNeighbors,10);
  private_nh.param("/neighborDist",neighborDist,15.0);
  private_nh.param("/timeHorizon",timeHorizon,10.0);
  private_nh.param("/timeHorizonObst",timeHorizonObst,10.0);
	
  private_nh.param("/thresholdGoal",THRESHOLD,0.1);
  private_nh.param("/thresholdLastSeen",THRESHOLD_LAST_SEEN,1.0);
  private_nh.param("/nrInitialGuess",NR_INITIAL_GUESS,20);
  private_nh.param("/simulation_mode",SIMULATION_MODE,true);
  private_nh.param("/noiseStd",NOISE_STD, 0.0);
  private_nh.param("/scaleRadius",scaleRadius,true);

  try{
    private_nh.getParam("/wheel_base",WHEELBASE);
    private_nh.getParam("/maxSpeed_linear",maxSpeed_linear);
    private_nh.getParam("/maxSpeed_angular",maxSpeed_angular);
    private_nh.getParam("/minSpeed_linear",minSpeed_linear);
    private_nh.getParam("/minSpeed_angular",minSpeed_angular);
  }
  catch (ros::InvalidNameException e){
    ROS_ERROR("%s",e.what());
    exit(-1);
  }


 //TODO set radius better via message
  private_nh.param("radius",radius,0.5);
  private_nh.setParam("ROSAgent",true);

  ROS_INFO("Params are wheel, lin(min,max), ang(min,max) %f, (%f,%f) (%f,%f)", WHEELBASE, minSpeed_linear,maxSpeed_linear,minSpeed_angular, maxSpeed_angular);
  //Init myself as RVO::Agent()

  
  me.agent = new RVO::Agent();
	
  me.agent->position_ = RVO::Vector2(0,0);
  me.agent->maxNeighbors_ = maxNeighbors;
  me.agent->maxSpeed_ = maxSpeed_linear;
  me.agent->neighborDist_ = neighborDist;
  me.agent->radius_ = radius;
  me.agent->timeHorizon_ = timeHorizon;
  me.agent->timeHorizonObst_ = timeHorizonObst;
  me.agent->velocity_ = RVO::Vector2(0,0);
  
  // get orca lines
  std::vector<double> origin_x = load_param_list("orca_lines/origin/x");
  std::vector<double> origin_y = load_param_list("orca_lines/origin/y");
  std::vector<double> dir_x = load_param_list("orca_lines/dir/x");
  std::vector<double> dir_y = load_param_list("orca_lines/dir/y");

  ROS_ASSERT(origin_x.size() == origin_y.size());
  ROS_ASSERT(dir_x.size() == dir_y.size());
  ROS_ASSERT(origin_x.size() == dir_x.size());

  for (size_t l = 0; l < origin_x.size(); l++) {
    RVO::Vector2 origin =  RVO::Vector2(origin_x[l], origin_y[l]);
    RVO::Vector2 dir =  RVO::Vector2(dir_x[l], dir_y[l]);
    RVO::Line line = RVO::Line();
    line.point = origin;
    line.direction = dir;
    me.agent->additional_orca_lines_.push_back(line);
    //ROS_ERROR("%s added new RVO line: (%f,%f), dir (%f,%f)",private_nh.getNamespace().c_str(),line.point.x(),line.point.y(),line.direction.x(),line.direction.y());
  }

  	
  //TODO uncomment for obstacle test
  /* 	
  std::vector<RVO::Vector2> wall1, wall2, wall3, wall4;
	

  wall1.push_back(RVO::Vector2(-0.1f, 5.0f));
  wall1.push_back(RVO::Vector2(-0.1f, -0.5f));
  
  wall2.push_back(RVO::Vector2(-4.1f, -0.5f));
  wall2.push_back(RVO::Vector2(-4.1f, 5.0f));

  wall3.push_back(RVO::Vector2(-0.1f, 5.0f));
  wall3.push_back(RVO::Vector2(-4.1f, 5.0f));

  wall4.push_back(RVO::Vector2(-4.1f, -0.5f));
  wall4.push_back(RVO::Vector2(-0.1f, -0.5f));



  this->addObstacle(wall1);
  this->addObstacle(wall2);
  this->addObstacle(wall3);
  this->addObstacle(wall4);
  */
	
  // %Tag(PUBLISHER)%
  pubTwist = private_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  pubOdom = private_nh.advertise<orca_planner::PositionShare>("/commonPositions", 1);
  subOdomAll = private_nh.subscribe("/commonPositions", 20, &ROSAgent::updateAgentsPositions,this);
  subOdom = private_nh.subscribe("odom", 10, &ROSAgent::cbOdom,this);
	
  pubCommands = private_nh.advertise<std_msgs::String>("/commandsRobot", 1);
  subCommands = private_nh.subscribe("/commandsRobot", 10, &ROSAgent::cbCommandsRobot,this);
  
  subGoal = private_nh.subscribe("goal", 10, &ROSAgent::cbGoal,this);
  pubInitialGuess = private_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",1);
  pubLines = private_nh.advertise<visualization_msgs::Marker>("orcaLines", 10);

  pubDebug = private_nh.advertise<orca_planner::OrcaDebug>("debug", 10);
  subAmclPose = private_nh.subscribe("amcl_pose", 10, &ROSAgent::cbAmclPose,this);
  state_srv = private_nh.advertiseService("state", &ROSAgent::cb_state_srv, this);


  myId = private_nh.getNamespace();
  if (strcmp(myId.c_str(), "/") == 0) {
    char hostname[1024];
    hostname[1023] = '\0';
    gethostname(hostname,1023) ; // TODO change to hostname
    myId = std::string(hostname);
  }
  ROS_INFO("My name is: %s",myId.c_str());


  if (SIMULATION_MODE)
    subPositionGroundTruth = private_nh.subscribe("base_pose_ground_truth", 10, &ROSAgent::cbPositionGroundTruth,this);
  

  tf_prefix = ""; // TODO change to robot if single master, e.g. simulation


  lastTime = ros::Time::now();

  /* for(int i=0;i<=180;i++){
    double maxTrackSpeed = calculateMaxTrackSpeedAngle(0.4,M_PI/180.0*i, 0.1);
    std::cout << maxTrackSpeed<< ",";
    }*/
}

size_t ROSAgent::addObstacle(const std::vector<RVO::Vector2>& vertices)
  {
    if (vertices.size() < 2) {
      return -1;
    }

    size_t obstacleNo = obstacles_.size();

    for (size_t i = 0; i < vertices.size(); ++i) {
      RVO::Obstacle* obstacle = new RVO::Obstacle();
      obstacle->point_ = vertices[i];
      if (i != 0) {
        obstacle->prevObstacle = obstacles_.back();
        obstacle->prevObstacle->nextObstacle = obstacle;
      }
      if (i == vertices.size() - 1) {
        obstacle->nextObstacle = obstacles_[obstacleNo];
        obstacle->nextObstacle->prevObstacle = obstacle;
      } 
      obstacle->unitDir_ = normalize(vertices[(i == vertices.size() - 1 ? 0 : i+1)] - vertices[i]);

      if (vertices.size() == 2) {
        obstacle->isConvex_ = true;
      } else {
        obstacle->isConvex_ = (leftOf(vertices[(i == 0 ? vertices.size() - 1 : i-1)], vertices[i], vertices[(i == vertices.size() - 1 ? 0 : i+1)]) >= 0);
      }

      obstacle->id_ = obstacles_.size();

      obstacles_.push_back(obstacle);
    }

    return obstacleNo;
  }



int main(int argc, char **argv) {
  ros::init(argc, argv, "orca_planner");
  ros::NodeHandle nh;
   
  ROS_INFO("Node initialized");
  ROSAgent thisRobot;
  thisRobot.loop();  
  
}

//Helper functions

double sampleNormal(double mean, double sigma) {
  static boost::mt19937 rng(static_cast<unsigned> (std::time(0)));
  boost::normal_distribution<double> nd(mean, sigma);

  boost::variate_generator<boost::mt19937&, 
                           boost::normal_distribution<> > var_nor(rng, nd);

  return var_nor();
}



//Put angle between -PI and PI
void limit_ang(double *angle)
{
  while (*angle > M_PI)
    *angle -= 2* M_PI;
  while (*angle <= -M_PI)
    *angle += 2* M_PI;
}

