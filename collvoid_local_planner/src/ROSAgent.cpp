/*
 *  
 *
 *
 *  Created by Daniel Claes on 14.06.11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "collvoid_local_planner/ROSAgent.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/bind.hpp>


using namespace RVO;

ROSAgent::ROSAgent() : 
  Agent(),
  timestep_(0.1),
  heading_(),
  max_track_speed_(),
  left_pref_(0.15),
  cur_allowed_error_(0.0),
  max_radius_cov_(-1),
  holo_robot_(false),
  holo_velocity_(),
  last_seen_(),
  additional_orca_lines_()
{
  ros::NodeHandle nh;
  line_pub_ = nh.advertise<visualization_msgs::Marker>("orca_lines", 10);
  neighbors_pub_ = nh.advertise<visualization_msgs::MarkerArray>("neighbors", 10);

}

ROSAgent::~ROSAgent() {
  clearNeighbors();
}

void ROSAgent::computeNewVelocity()
{
  orcaLines_.clear();

  calculateObstacleLines();
  orcaLines_.insert(orcaLines_.end(), additional_orca_lines_.begin(), additional_orca_lines_.end());

  const size_t numObstLines = orcaLines_.size();
  const float invTimeHorizon = 1.0f / timeHorizon_;

  /* Create agent ORCA lines. */
  for (size_t i = 0; i < agentNeighbors_.size(); ++i) {
    const Agent* const other = agentNeighbors_[i].second;
    if (RVO::abs(other->velocity_) < RVO_EPSILON){
      insertStationaryAgent(agentNeighbors_[i].second);
      continue;
    }
   
    
    const Vector2 relativePosition = other->position_ - position_;
    const Vector2 relativeVelocity = velocity_ - other->velocity_;
    const float distSq = absSq(relativePosition);
    const float combinedRadius = radius_ + other->radius_;
    const float combinedRadiusSq = sqr(combinedRadius);

    Line line;
    Vector2 u;

    if (distSq > combinedRadiusSq) {
      /* No collision. */
      const Vector2 w = relativeVelocity - invTimeHorizon * relativePosition;
      /* Vector from cutoff center to relative velocity. */
      const float wLengthSq = absSq(w);

      const float dotProduct1 = w * relativePosition;

      if (dotProduct1 < 0.0f && sqr(dotProduct1) > combinedRadiusSq * wLengthSq) {
	/* Project on cut-off circle. */
	const float wLength = std::sqrt(wLengthSq);
	const Vector2 unitW = w / wLength;

	line.direction = Vector2(unitW.y(), -unitW.x());
	u = (combinedRadius * invTimeHorizon - wLength) * unitW;
      } else {
	/* Project on legs. */
	const float leg = std::sqrt(distSq - combinedRadiusSq);

	if (det(relativePosition, w) > -left_pref_) {
	  /* Project on left leg. */
	  line.direction = Vector2(relativePosition.x() * leg - relativePosition.y() * combinedRadius, relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
	} else {
	  /* Project on right leg. */
	  line.direction = -Vector2(relativePosition.x() * leg + relativePosition.y() * combinedRadius, -relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
	}

	const float dotProduct2 = relativeVelocity * line.direction;

	u = dotProduct2 * line.direction - relativeVelocity;
      }
    } else {
      /* Collision. Project on cut-off circle of time timeStep. */
      //  const float invTimeStep = 1.0f / sim_->timeStep_; //TODO update with exact timestep
      const float invTimeStep = 1.0f/timestep_;
      /* Vector from cutoff center to relative velocity. */
      const Vector2 w = relativeVelocity - invTimeStep * relativePosition;

      const float wLength = abs(w);
      const Vector2 unitW = w / wLength;

      line.direction = Vector2(unitW.y(), -unitW.x());
      u = (combinedRadius * invTimeStep - wLength) * unitW;
    }

    line.point = velocity_ + 0.5f * u;
    //ros::NodeHandle nh;
    //ROS_ERROR("%s = line: (%f,%f), dir (%f,%f)",nh.getNamespace().c_str(),line.point.x(),line.point.y(),line.direction.x(),line.direction.y());
    orcaLines_.push_back(line);
  }

  size_t lineFail = linearProgram2(orcaLines_, maxSpeed_, prefVelocity_, false, newVelocity_);

  if (lineFail < orcaLines_.size()) {
    linearProgram3(orcaLines_, numObstLines, lineFail, maxSpeed_, newVelocity_);
  }

  additional_orca_lines_.clear();
}


void ROSAgent::publishOrcaLines(){
  visualization_msgs::Marker line_list;
  //line_list.header.frame_id = myId + "/base_link";
  line_list.header.frame_id = "/map";
  line_list.header.stamp = ros::Time::now();
  line_list.ns = base_odom_.header.frame_id;
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.02;
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;
  geometry_msgs::Point p;
  p.x = position_.x();
  p.y = position_.y();
  p.z = 0.2;
  line_list.points.push_back(p);
  p.x = p.x + velocity_.x();
  p.y = p.y + velocity_.y();
  line_list.points.push_back(p);
  for (size_t i=0;i< orcaLines_.size();i++) {
    geometry_msgs::Point p;
    p.x = position_.x() + orcaLines_[i].point.x() - orcaLines_[i].direction.x();
    p.y = position_.y() + orcaLines_[i].point.y() - orcaLines_[i].direction.y();
    
    line_list.points.push_back(p);
    p.x = p.x + 3 * orcaLines_[i].direction.x();
    p.y = p.y + 3 * orcaLines_[i].direction.y();
    line_list.points.push_back(p);
     
    //p.x = 2*me.agent->orcaLines_[i].point.x();
    //p.y = 2*me.agent->orcaLines_[i].point.y();
    //line_list.points.push_back(p);

    
    }
  line_pub_.publish(line_list);
    
}

void ROSAgent::publishNeighborPositions(){
  visualization_msgs::MarkerArray sphere_list;
  sphere_list.markers.resize(agentNeighbors_.size());
  for (size_t i=0;i< agentNeighbors_.size();i++) {

    //visualization_msgs::Marker neighbor;
   const Agent* const agent = agentNeighbors_[i].second;
   //sphere_list.header.frame_id = myId + "/base_link";
   sphere_list.markers[i].header.frame_id = "/map";
   sphere_list.markers[i].header.stamp = ros::Time::now();
   sphere_list.markers[i].ns = base_odom_.header.frame_id;
   sphere_list.markers[i].action = visualization_msgs::Marker::ADD;
   sphere_list.markers[i].pose.orientation.w = 1.0;
   sphere_list.markers[i].type = visualization_msgs::Marker::SPHERE;
   sphere_list.markers[i].scale.x = agentNeighbors_[i].second->radius_;
   sphere_list.markers[i].scale.y = agentNeighbors_[i].second->radius_;
   sphere_list.markers[i].scale.z = 0.1;
   sphere_list.markers[i].color.r = 1.0;
   sphere_list.markers[i].color.a = 1.0;
   sphere_list.markers[i].id = i;
  
   sphere_list.markers[i].pose.position.x = agentNeighbors_[i].second->position_.x();
   sphere_list.markers[i].pose.position.y = agentNeighbors_[i].second->position_.y();
   sphere_list.markers[i].pose.position.z = 0.2;
   geometry_msgs::Point p;
   p.x = agentNeighbors_[i].second->position_.x();
   p.y = agentNeighbors_[i].second->position_.y();
   p.z = 0.2;
   sphere_list.markers[i].points.push_back(p);
   //sphere_list.markers[i] = neighbor;
   ROS_DEBUG("%s neighbor %s at x = %f y %f wiTh radius: %f", id_.c_str(), agent->id_.c_str(), agent->position_.x(), agent->position_.y(), agent->radius_);

 }
 neighbors_pub_.publish(sphere_list);
  

}

void ROSAgent::setFootprint(std::vector<geometry_msgs::Point> footprint ){
  if (footprint.size() < 2) {
    ROS_ERROR("The footprint specified has less than two nodes");
    return;
  }
  footprint_ = footprint;
  footprint_lines_.clear();
  geometry_msgs::Point p = footprint_[0];
  RVO::Vector2 first = RVO::Vector2(p.x, p.y);
  RVO::Vector2 old = RVO::Vector2(p.x, p.y);
  //add linesegments for footprint
  for (size_t i = 0; i<footprint_.size(); i++) {
    geometry_msgs::Point p = footprint_[i];
    RVO::Vector2 point = RVO::Vector2(p.x, p.y);
    footprint_lines_.push_back(std::make_pair(old, point));
    old = point;
  }
  //add last segment
  footprint_lines_.push_back(std::make_pair(old, first));
  has_footprint_ = true;
}

void ROSAgent::setFootprintRadius(float radius){
  footprint_radius_ = radius;
  has_footprint_ = false;
}


bool ROSAgent::isHoloRobot() {
  return holo_robot_;
}

void ROSAgent::setIsHoloRobot(bool holo_robot) {
  holo_robot_ = holo_robot;
}
  
void ROSAgent::setTimeStep(float timestep) {
  this->timestep_ = timestep;
}

void ROSAgent::setHeading(float heading) {
  this->heading_ = heading;
}

float ROSAgent::getHeading() {
  return this->heading_;
}

void ROSAgent::setHoloVelocity(float x, float y){
  this->holo_velocity_ = RVO::Vector2(x,y);
}

RVO::Vector2 ROSAgent::getHoloVelocity(){
  return this->holo_velocity_;
}

void ROSAgent::setId(std::string id) {
  this->id_ = id;
}

std::string ROSAgent::getId(){
  return this->id_;
}

void ROSAgent::setLastSeen(ros::Time last_seen){
  this->last_seen_ = last_seen;
}

bool ROSAgent::compareObstacles(const RVO::Vector2& v1, const RVO::Vector2& v2){
  return RVO::absSq(position_ - v1) <= RVO::absSq(position_ - v2);
}


void ROSAgent::insertStationaryAgent(const Agent* agent) {
    double dist = RVO::abs(position_ - agent->position_);
    Line line;
    Vector2 relative_position = agent->position_ - position_;
    line.point = normalize(relative_position) * (dist - radius_ - agent->radius_ - 0.05);

    line.direction = Vector2 (-normalize(relative_position).y(),normalize(relative_position).x()) ; 
    
    orcaLines_.push_back(line);
  
}

void ROSAgent::sortObstacleLines(){
  boost::mutex::scoped_lock lock(obstacle_lock_);
  std::sort(obstacle_points_.begin(),obstacle_points_.end(), boost::bind(&ROSAgent::compareObstacles,this,_1,_2));
}


RVO::Vector2 ROSAgent::LineSegmentToLineSegmentIntersection(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4){
  float r, s, d;
  RVO::Vector2 res;
  //Make sure the lines aren't parallel
  if ((y2 - y1) / (x2 - x1) != (y4 - y3) / (x4 - x3)){
    d = (((x2 - x1) * (y4 - y3)) - (y2 - y1) * (x4 - x3));
    if (d != 0){
      r = (((y1 - y3) * (x4 - x3)) - (x1 - x3) * (y4 - y3)) / d;
      s = (((y1 - y3) * (x2 - x1)) - (x1 - x3) * (y2 - y1)) / d;
      if (r >= 0 && r <= 1){
	if (s >= 0 && s <= 1){
	  return RVO::Vector2(x1 + r * (x2 - x1), y1 + r * (y2 - y1));
	}
      }
    }
  } 
  return res;
}

float ROSAgent::getDistToFootprint(RVO::Vector2& point){
  RVO::Vector2 result, null;
  for (size_t i = 0; i < footprint_lines_.size(); i++){
    RVO::Vector2 first = footprint_lines_[i].first;
    RVO::Vector2 second = footprint_lines_[i].second;
          
    result = LineSegmentToLineSegmentIntersection(first.x(),first.y(),second.x(),second.y(), 0.0, 0.0, point.x(),point.y());
    if (result != null) {
      //ROS_DEBUG("Result = %f, %f, dist %f", result.x(), result.y(), RVO::abs(result));
      return RVO::abs(result);
    }
  }
  ROS_DEBUG("Obstacle Point within Footprint. I am close to/in collision");
  return -1;
}

void ROSAgent::calculateObstacleLines(){
  
  for(size_t i = 0; i< obstacle_points_.size(); i++){
    //ROS_DEBUG("obstacle at %f %f dist %f",obstacle_points_[i].x(),obstacle_points_[i].y(),RVO::abs(position_-obstacle_points_[i]));
    if (delete_observations_ && pointInNeighbor(obstacle_points_[i]))
      continue;
    double dist = RVO::abs(position_ - obstacle_points_[i]);
    Line line;
    Vector2 relative_position = obstacle_points_[i] - position_;
    float dist_to_footprint;
    if (!has_footprint_)
      dist_to_footprint = footprint_radius_;
    else {
      dist_to_footprint = getDistToFootprint(relative_position);
      if (dist_to_footprint == -1){
	dist_to_footprint = footprint_radius_;
      }
    }
    dist = std::min(dist - dist_to_footprint - 0.01, (double)max_vel_with_obstacles_);
    //if (dist < (double)max_vel_with_obstacles_){
    //  dist *= dist;
    //} 
    //    line.point = normalize(relative_position) * (dist - dist_to_footprint - 0.03);
    line.point = normalize(relative_position) * (dist); 
    line.direction = Vector2 (-normalize(relative_position).y(),normalize(relative_position).x()) ; 
    
    orcaLines_.push_back(line);
    
    if (dist > timeHorizonObst_ * RVO::abs(velocity_))
      return;
  }

}

void ROSAgent::setMaxVelWithObstacles(float max_vel_with_obstacles){
  max_vel_with_obstacles_ = max_vel_with_obstacles;
}

bool ROSAgent::pointInNeighbor(RVO::Vector2& point) {
  for (size_t i = 0; i<agentNeighbors_.size();i++){
    if (RVO::abs(point - agentNeighbors_[i].second->position_) <= agentNeighbors_[i].second->radius_)
      return true;
  }
  return false;
}

void ROSAgent::setMaxTrackSpeed(float max_track_speed) {
  this->max_track_speed_ = max_track_speed;
  Line maxVel1;
  Line maxVel2;

  Vector2 dir =  Vector2(cos(this->heading_), sin(this->heading_));
  maxVel1.point = this->max_track_speed_ * Vector2(-dir.y(),dir.x());
  maxVel1.direction = -dir;
  maxVel2.direction = dir;
  maxVel2.point = -this->max_track_speed_ * Vector2(-dir.y(),dir.x());
  additional_orca_lines_.push_back(maxVel1);
  additional_orca_lines_.push_back(maxVel2);
}

void ROSAgent::addMovementConstraintsDiff(double error, double T,  double max_vel_x, double max_vel_th){
  /*
  double steps = 180;
 
  for (double i = -M_PI / 2.0; i <= M_PI / 2.0; i += M_PI/steps) {
    double speed = calculateMaxTrackSpeedAngle(T,i, 0.01, max_vel_x, max_vel_th);
    double vstarErr = calcVstarError(T,i,0.01);
    
    std::cout << speed << " vstar Err " << vstarErr <<" ang " << i <<std::endl;
  }
  */

  double min_theta = M_PI / 2.0;
  double yaw = heading_;
  double max_track_speed = calculateMaxTrackSpeedAngle(T,min_theta, error, max_vel_x, max_vel_th);

  Vector2 first_point = max_track_speed * Vector2(cos(this->heading_-min_theta), sin(this->heading_-min_theta));
  double steps = 4.0;

  double step_size = - M_PI / steps;
 


  for (int i=1; i<=(int)steps; i++) {
    
    Line line;
    double cur_ang = min_theta + i* step_size;
    Vector2 second_point = Vector2(cos(yaw - cur_ang), sin(yaw - cur_ang));
    double track_speed = calculateMaxTrackSpeedAngle(T,cur_ang, error, max_vel_x, max_vel_th);
    second_point = track_speed * second_point;
    line.point = first_point;
    line.direction = normalize(second_point - first_point);
    additional_orca_lines_.push_back(line);
    //    ROS_DEBUG("line point 1 x, y, %f, %f, point 2 = %f,%f",first_point.x(),first_point.y(),second_point.x(),second_point.y());
    first_point = second_point;
  }
  //  me_->setMaxTrackSpeed(max_track_speed);
  setMaxSpeedLinear(vMaxAng(max_vel_x));
}



void ROSAgent::addAccelerationConstraintsXY(double max_vel_x, double acc_lim_x, double max_vel_y, double acc_lim_y, double sim_period){
  double max_lim_x, max_lim_y, min_lim_x, min_lim_y;
  Line line_x_back, line_x_front, line_y_left, line_y_right;
  

  Vector2 dir_front =  Vector2(cos(this->heading_), sin(this->heading_));
  Vector2 dir_right =  Vector2(dir_front.y(),-dir_front.x());
  
  double cur_x = base_odom_.twist.twist.linear.x;
  double cur_y = base_odom_.twist.twist.linear.y;
  

  max_lim_x = std::min(max_vel_x, cur_x + sim_period * acc_lim_x);
  max_lim_y = std::min(max_vel_y, cur_y + sim_period * acc_lim_y);

  min_lim_x = std::max(-max_vel_x, cur_x - sim_period * acc_lim_x);
  min_lim_y = std::max(-max_vel_y, cur_y - sim_period * acc_lim_y);

  // ROS_DEBUG("Max limints x = %f, %f, y = %f, %f", max_lim_x, min_lim_x, max_lim_y, min_lim_y);

  line_x_front.point = dir_front * max_lim_x;
  line_x_front.direction =  -dir_right;

  line_x_back.point = dir_front * min_lim_x;
  line_x_back.direction = dir_right;
  
  
  additional_orca_lines_.push_back(line_x_front);
  additional_orca_lines_.push_back(line_x_back);
  
  if(holo_robot_) {
    line_y_left.point = -dir_right * max_lim_y;
    line_y_left.direction = -dir_front;

    line_y_right.point = -dir_right * min_lim_y;
    line_y_right.direction = dir_front;
  
  
    additional_orca_lines_.push_back(line_y_left);
    additional_orca_lines_.push_back(line_y_right);

  }
}

void ROSAgent::setWheelBase(float wheel_base){
  wheel_base_ = wheel_base;
}


void ROSAgent::setLeftPref(float left_pref) {
  this->left_pref_ = left_pref;
}

void ROSAgent::setAdditionalOrcaLines(std::vector<RVO::Line> additional_orca_lines){
  this->additional_orca_lines_.clear();
  this->additional_orca_lines_ = additional_orca_lines;
}

void ROSAgent::setRadius(float radius){
  this->radius_ = radius;
}

float ROSAgent::getRadius(){
  return radius_;
}

void ROSAgent::setPosition(float x, float y){
  this->position_ = RVO::Vector2(x,y);
}

void ROSAgent::setVelocity(float x, float y){
  this->velocity_ = RVO::Vector2(x,y);
}

void ROSAgent::setMaxSpeedLinear(float max_speed_linear ){
  this->maxSpeed_ = max_speed_linear;
}
void ROSAgent::setMaxNeighbors(int max_neighbors){
  this->maxNeighbors_ = max_neighbors;
}
void ROSAgent::setNeighborDist(float neighbor_dist){
  this-> neighborDist_ = neighbor_dist;
}
void ROSAgent::setTimeHorizon(float time_horizon){
  this->timeHorizon_ = time_horizon;
}
void ROSAgent::setTimeHorizonObst(float time_horizon_obst){
  this->timeHorizonObst_ = time_horizon_obst;
}


void ROSAgent::setDeleteObservations(bool delete_observations){
  this->delete_observations_ = delete_observations;
}

void ROSAgent::clearNeighbors(){
 for (size_t i  = 0; i < agentNeighbors_.size(); i++){
        delete agentNeighbors_[i].second;
 }
 agentNeighbors_.clear();
}

double ROSAgent::vMaxAng(double max_vel_x){
  return max_vel_x - std::abs(base_odom_.twist.twist.angular.z) * wheel_base_/2.0;
}

double ROSAgent::beta(double T, double theta, double max_vel_x){
  double v_max = vMaxAng(max_vel_x);
  return - ((2.0 * RVO::sqr(T) * sin(theta)) / theta) * v_max;
}

double ROSAgent::gamma(double T, double theta, double error, double max_vel_x) {
  double v_max = vMaxAng(max_vel_x);
  return (((2.0 * RVO::sqr(T) * (1.0 - cos(theta))) / RVO::sqr(theta)) * RVO::sqr(v_max) - RVO::sqr(error));
}

double ROSAgent::calcVstar(double vh, double theta){
  return vh * ((theta * sin(theta))/(2.0 * (1.0 - cos(theta))));
}

double ROSAgent::calcVstarError(double T,double theta, double error) {
  
  return calcVstar(error/T,theta) * sqrt((2.0*(1.0-cos(theta)))/(2.0*(1.0-cos(theta))-RVO::sqr(sin(theta))));
}

double ROSAgent::calculateMaxTrackSpeedAngle(double T, double theta, double error, double max_vel_x, double max_vel_th){
  if (fabs(theta) <= RVO_EPSILON)
    return max_vel_x;
  if (fabs(theta/T) <= max_vel_th) {
    double vstar_error = calcVstarError(T,theta,error);
    double v_max_ang = vMaxAng(max_vel_x);
    if (vstar_error <= v_max_ang) { 
      //return 0;
      return std::min(vstar_error * (2.0 * (1.0 - cos(theta))) / (theta*sin(theta)),max_vel_x);
    }
    else {
      double b, g;
      b = beta(T,theta,max_vel_x);
      g = gamma(T,theta,error,max_vel_x);
      //return 1;
      return std::min((-b+sqrt(RVO::sqr(b)-4*RVO::sqr(T)*g))/ (2.0 * g), max_vel_x);
    }
  }
  else
    //return 2;
    return std::min(sign(theta)*error*max_vel_th/theta,max_vel_x);
}
