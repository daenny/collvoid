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

#ifndef COLLVOID_ORCA_H
#define COLLVOID_ORCA_H

#include "collvoid_local_planner/Agent.h"

namespace collvoid{


  
  Line createOrcaLine(Agent* me, Agent* other, double trunc_time, double timestep, double left_pref, double cur_allowed_error);

  Line createOrcaLine(double combinedRadius, const Vector2& relativePosition, const Vector2& me_vel,const Vector2& other_vel, double trunc_time, double timestep, double left_pref, double cur_allowed_error, bool controlled);

  
  Line createStationaryAgent(Agent* me, Agent* other);
  void addAccelerationConstraintsXY(double max_vel_x, double acc_lim_x, double max_vel_y, double acc_lim_y, Vector2 cur_vel, double heading, double sim_period, bool holo_robot, std::vector<Line>& additional_orca_lines);

  void addMovementConstraintsDiffSimple(double max_track_speed, double heading, std::vector<Line>& additional_orca_lines);

  void addMovementConstraintsDiff(double error, double T, double max_vel_x, double max_vel_th, double heading, double v_max_ang, std::vector<Line>& additional_orca_lines);
  double beta(double T, double theta, double v_max_ang);
  double gamma(double T, double theta, double error, double v_max_ang);
  double calcVstar(double vh, double theta);
  double calcVstarError(double T,double theta, double error);
  double calculateMaxTrackSpeedAngle(double T, double theta, double error, double max_vel_x, double max_vel_th, double v_max_ang);


  
  bool linearProgram1(const std::vector<Line>& lines, size_t lineNo, float radius, const Vector2& optVelocity, bool dirOpt, Vector2& result);
  size_t linearProgram2(const std::vector<Line>& lines, float radius, const Vector2& optVelocity, bool dirOpt, Vector2& result);
  void linearProgram3(const std::vector<Line>& lines, size_t numObstLines, size_t beginLine, float radius, Vector2& result);  

}

#endif
