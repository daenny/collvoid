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

#include <ros/ros.h>
#include "Agent.h"
#include "Obstacle.h"

class ROSAgent : protected RVO::Agent {
 private:
  float timeStep_;
  float heading_;
  float maxTrackSpeed_;
  float leftPref_;  
  std::vector<RVO::Line> additionalOrcaLines_;
  
  ROSAgent();
  ~ROSAgent();

  
 public:
  void setTimeStep(float timeStep);
  void setHeading(float heading);
  void setMaxTrackSpeed(float maxTrackSpeed);
  void setLeftPref(float leftPref);
  void setAdditionalOrcaLines(std::vector<RVO::Line> additionalOrcaLines);
  void computeNewVelocity();
};


#endif
