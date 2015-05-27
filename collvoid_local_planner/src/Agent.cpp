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
#include <boost/foreach.hpp>

namespace collvoid {

    void Agent::computeOrcaVelocity(Vector2 pref_velocity, bool convex) {

        orca_lines_.clear();
        orca_lines_.insert(orca_lines_.end(), additional_orca_lines_.begin(), additional_orca_lines_.end());


        const size_t num_obst_lines = orca_lines_.size();

        if (controlled_) {
            /* Create agent ORCA lines. */
            BOOST_FOREACH (AgentPtr agent, agent_neighbors_) {
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
        size_t line_fail = linearProgram2(orca_lines_, max_speed_x_, pref_velocity, false, new_velocity_);

        if (line_fail < orca_lines_.size()) {
            linearProgram3(orca_lines_, num_obst_lines, line_fail, max_speed_x_, new_velocity_);
        }

    }

    void Agent::computeClearpathVelocity(Vector2 pref_velocity) {
        if (controlled_) {
            computeAgentVOs();
        }
        new_velocity_ = calculateClearpathVelocity(samples_, vo_agents_, additional_orca_lines_, pref_velocity,
                                                   max_speed_x_, use_truncation_);
    }

    void Agent::computeSampledVelocity(Vector2 pref_velocity) {
        if (controlled_) {
            computeAgentVOs();
        }
        new_velocity_ = calculateNewVelocitySampled(samples_, vo_agents_, pref_velocity, max_speed_x_, use_truncation_);
    }


    void Agent::computeAgentVOs() {
        BOOST_FOREACH (AgentPtr agent, agent_neighbors_) {
                        VO new_agent_vo;
                        //use footprint or radius to create VO
                        if (convex_) {
                            if (agent->controlled_) {
                                new_agent_vo = createVO(position_, footprint_, velocity_, agent->position_,
                                                        agent->footprint_, agent->velocity_, type_vo_);
                            }
                            else {
                                new_agent_vo = createVO(position_, footprint_, velocity_, agent->position_,
                                                        agent->footprint_, agent->velocity_, VOS);
                            }
                        }
                        else {
                            if (agent->controlled_) {
                                new_agent_vo = createVO(position_, radius_, velocity_, agent->position_, agent->radius_,
                                                        agent->velocity_, type_vo_);
                            }
                            else {
                                new_agent_vo = createVO(position_, radius_, velocity_, agent->position_, agent->radius_,
                                                        agent->velocity_, VOS);
                            }

                        }
                        //truncate
                        if (use_truncation_) {
                            new_agent_vo = createTruncVO(new_agent_vo, trunc_time_);
                        }
                        vo_agents_.push_back(new_agent_vo);

                    }
    }

    void Agent::setLeftPref(double left_pref) {
        this->left_pref_ = left_pref;
    }

    void Agent::setRadius(double radius) {
        this->radius_ = radius;
    }

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
