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


#include "collvoid_local_planner/Agent.h"
#include "collvoid_local_planner/orca.h"
#include <angles/angles.h>

namespace collvoid {

    void Agent::computeOrcaVelocity(Vector2 pref_velocity, bool convex) {

        orca_lines_.clear();
        orca_lines_.insert(orca_lines_.end(), additional_orca_lines_.begin(), additional_orca_lines_.end());


        const size_t num_obst_lines = orca_lines_.size();

        if (controlled_) {
            /* Create agent ORCA lines. */
            for (AgentPtr agent: agent_neighbors_) {
                double timestep = agent->timestep_;
                if (timestep < EPSILON)
                    timestep = sim_period_;
                Line line;
                if (!convex) {
                    line = createOrcaLine(this, agent.get(), trunc_time_, timestep, left_pref_,
                                          cur_allowed_error_);
                }
                else {
                    VO new_agent_vo = createVO(position_, footprint_, velocity_, agent->position_,
                                               agent->footprint_, agent->velocity_, VOS);
                    line = createOrcaLine(new_agent_vo.combined_radius, new_agent_vo.relative_position,
                                          velocity_, agent->velocity_, trunc_time_, timestep, left_pref_,
                                          cur_allowed_error_, agent->controlled_);
                }
                orca_lines_.push_back(line);
            }
        }
        size_t line_fail = linearProgram2(orca_lines_, planner_util_->getCurrentLimits().max_vel_x, pref_velocity, false, new_velocity_);

        if (line_fail < orca_lines_.size()) {
            linearProgram3(orca_lines_, num_obst_lines, line_fail, planner_util_->getCurrentLimits().max_vel_x, new_velocity_);
        }

    }



    void Agent::computeHumanVOs() {
        for (AgentPtr agent: human_neighbors_) {
            VO new_agent_vo;
            bool created = false;
            //use footprint or radius to create VO
            if (use_polygon_footprint_) {
                Vector2 rel_position = agent->position_ - position_;
                Vector2 speed = Vector2(0,0);
                //if (fabs(atan(rel_position)-heading_ ) < M_PI/2.) {
                ROS_INFO("angle betwee %f", angles::shortest_angular_distance(atan(rel_position), heading_));
                if (angles::shortest_angular_distance(atan(rel_position), heading_)< M_PI/2.){
                    speed = normalize(position_ - agent->position_) * abs(agent->velocity_);

                }
                if (abs(rel_position)<2.){
                    new_agent_vo = createVO(position_, footprint_, velocity_, agent->position_,
                            //agent->footprint_, agent->velocity_, VOS);
                                            agent->footprint_, speed, VOS);
                    created = true;
                }

            }
            else {
                new_agent_vo = createVO(position_, radius_, velocity_, agent->position_, agent->radius_,
                                        agent->velocity_, VOS);
                created = true;
            }
            if (created) {
                //truncate calculate with 100 to avoid code breaking..
                if (use_truncation_){
                    new_agent_vo = createTruncVO(new_agent_vo, 20);
                }
                human_vos_.push_back(new_agent_vo);
                all_vos_.push_back(new_agent_vo);
            }

        }
    }


    void Agent::computeAgentVOs() {
        for (AgentPtr agent: agent_neighbors_) {
            VO new_agent_vo;
            //use footprint or radius to create VO
            if (use_polygon_footprint_) {
                if (agent->controlled_ && abs(agent->velocity_) > EPSILON)  {
                    new_agent_vo = createVO(position_, footprint_, velocity_, agent->position_,
                                            agent->footprint_, agent->velocity_, type_vo_);
                }
                else {
                    new_agent_vo = createVO(position_, footprint_, velocity_, agent->position_,
                                            agent->footprint_, agent->velocity_, VOS);
                }
            }
            else {
                if (agent->controlled_  && abs(agent->velocity_) > EPSILON) {
                    new_agent_vo = createVO(position_, radius_, velocity_, agent->position_, agent->radius_,
                                            agent->velocity_, type_vo_);
                }
                else {
                    new_agent_vo = createVO(position_, radius_, velocity_, agent->position_, agent->radius_,
                                            agent->velocity_, VOS);
                }

            }
            //truncate
            if (agent->controlled_ && use_truncation_) {
                if (abs(agent->velocity_) < EPSILON) {
                    new_agent_vo = createTruncVO(new_agent_vo, std::max((abs(velocity_) + abs(agent->velocity_)) * trunc_time_, 0.5));
                    static_vos_.push_back(new_agent_vo);
                }
                else {
                    new_agent_vo = createTruncVO(new_agent_vo, std::max((abs(velocity_) + abs(agent->velocity_)) * trunc_time_, 2.));
                    agent_vos_.push_back(new_agent_vo);
                }
            }
            else if(!agent->controlled_ && use_truncation_) {
                new_agent_vo = createTruncVO(new_agent_vo, 100);
                agent_vos_.push_back(new_agent_vo);
            }
            else {
                if (abs(agent->velocity_) < EPSILON && agent->controlled_) {
                    static_vos_.push_back(new_agent_vo);
                }
                else {
                    agent_vos_.push_back(new_agent_vo);
                }
            }
            all_vos_.push_back(new_agent_vo);

        }
    }

    void Agent::setLeftPref(double left_pref) {
        this->left_pref_ = left_pref;
    }

    //   void Agent::setRadius(double radius) {
    //       this->radius_ = radius;
    //   }

    void Agent::setTruncTime(double trunc_time) {
        this->trunc_time_ = trunc_time;
    }


    void Agent::setSimPeriod(double sim_period) {
        sim_period_ = sim_period;
    }


    //used in orca for orcalines creation...
    collvoid::Vector2 Agent::getPosition() {
        return this->position_;
    }

    collvoid::Vector2 Agent::getVelocity() {
        return this->velocity_;
    }

    double Agent::getRadius() {
        return radius_;
    }

}
