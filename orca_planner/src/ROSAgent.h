/*
 *  
 *
 *
 *  Created by Daniel Claes on 31.08.11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef ROSAGENT_H
#define ROSAGENT_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include "Agent.h"
#include "orca_planner/PositionShare.h"
#include <ros/ros.h>
#include "orca_planner/StateSrv.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


struct PositionStage {
	double x;
	double y;
	double ang;
};

struct Robot {
	RVO::Agent* agent;
	PositionStage pos;
	geometry_msgs::TwistWithCovariance twist;
	ros::Time lastSeen;
	std::string id;
};

double THRESHOLD; //goal threshold
double THRESHOLD_LAST_SEEN; //after 1 sec neglect..
int NR_INITIAL_GUESS; //send nr of initialguess
double NOISE_STD; //set the standard deviation for the noise on the initial guess
double WHEELBASE;
bool SIMULATION_MODE;

class ROSAgent{

  private:

	enum State
	{
	  RUNNING,
	  STOPPED,
	  AT_GOAL // implies stopped
	};

        ros::NodeHandle private_nh;
	tf::TransformListener tfListener;

        double maxSpeed_linear;
        double maxSpeed_angular;
	double minSpeed_linear;
	double minSpeed_angular;
	int maxNeighbors;
	double neighborDist;
	double timeHorizon;
	double timeHorizonObst;
	double radius;
	double radiusUncertainty;
	std::vector<RVO::Obstacle*> obstacles_;
        Robot me;
	
	std::vector<Robot> robots;
	PositionStage goal;
	//PositionStage cur;
	
	std::string myId;
	
	State state_;
	//bool commandStart;
	//bool sendFinish;
	
	bool useGroundTruth;
	bool scaleRadius;
	int nrInitialGuess;
	
	ros::Time lastTime;
	//double curSpeedLin;
	//double curSpeedAng;
	
	//cache messages;
	int debugMsgCounter;
	nav_msgs::OdometryPtr ground_truth, odom;
	geometry_msgs::PoseStamped locatedPose;
	//geometry_msgs::TwistStamped cmd_vel;
	//geometry_msgs::Vector3Stamped holo_vel;
	double locError;
	
	ros::Subscriber subOdom,subOdomAll,subCommands,subPositionGroundTruth, subGoal,subAmclPose;
	ros::Publisher pubTwist,pubOdom,pubCommands,pubInitialGuess,pubLines,pubDebug;

	ros::ServiceServer state_srv;
	
	void addAllAgents();
	void updateAgentsPositions(const orca_planner::PositionShareConstPtr& msg);
	void cbOdom(nav_msgs::OdometryPtr msg);
	void cbPositionGroundTruth(nav_msgs::OdometryPtr msg);
	void cbCommandsRobot(const std_msgs::String::ConstPtr& msg);
	void cbGoal(const geometry_msgs::PoseStamped::ConstPtr& msg); 
	void cbAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	
	bool cb_state_srv(orca_planner::StateSrv::Request &req,
			  orca_planner::StateSrv::Response &res);
       
	double calculateMaxTrackSpeedAngle(double T, double theta, double error);
	double calcVstarError(double T,double theta, double error);
	double calcVstar(double vh, double theta);
	double vMaxAng();
	double beta(double T, double theta);
	double gamma(double T, double theta, double error);
	void publishLines();
	size_t addObstacle(const std::vector<RVO::Vector2>& vertices);
	
	std::vector<double> load_param_list(std::string param_name);
	std::vector<RVO::Line> orca_lines_;
	
public:
	ROSAgent();
	~ROSAgent();
	void init();
	void update();
	bool atGoal();
	void loop();
};

double sampleNormal(double mean, double sigma);
void limit_ang(double *angle);

#endif
