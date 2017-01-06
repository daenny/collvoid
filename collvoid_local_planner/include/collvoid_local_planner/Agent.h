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



#ifndef AGENT_H
#define AGENT_H

#include <collvoid_local_planner/Utils.h>
#include <collvoid_local_planner/clearpath.h>
#include <boost/shared_ptr.hpp>
#include <base_local_planner/local_planner_util.h>

namespace collvoid{

    class Agent{
    public:

        //    Agent(){};
        virtual ~Agent(){};

        void computeOrcaVelocity(Vector2 pref_velocity, bool convex);


        void computeAgentVOs();
        void computeHumanVOs();

        void setLeftPref(double left_pref);
        //void setRadius(double radius);
        void setTruncTime(double trunc_time);
        void setSimPeriod(double sim_period);

        double getRadius();
        collvoid::Vector2 getPosition();
        collvoid::Vector2 getVelocity();

        base_local_planner::LocalPlannerUtil *planner_util_;


        //config
        double left_pref_;

        bool use_truncation_;
        double trunc_time_;
        double timestep_, sim_period_;

        bool controlled_;

        bool orca_; // Orca or VO?
        bool use_polygon_footprint_; // circle approx, or mink sum?

        //VO settings
        bool clearpath_; //Clearpath or sampling based
        int type_vo_; //0 = HRVO, 1 = RVO , 2 = VO

        bool new_sampling_;
        bool use_obstacles_;

        //description
        Vector2 position_;
        double heading_;
        Vector2 velocity_;

        double radius_;

        std::vector<Vector2> footprint_;

        Vector2 new_velocity_;


        //nh stuff
        double cur_allowed_error_;

        //Orca
        std::vector<Line> orca_lines_, additional_orca_lines_;

        //VO stuff
        std::vector<VO> all_vos_, agent_vos_, static_vos_, human_vos_;
        std::vector<VelocitySample> samples_, safe_samples_;



        std::vector<boost::shared_ptr<Agent> > agent_neighbors_, human_neighbors_;


    };

    typedef boost::shared_ptr<Agent> AgentPtr;


}

#endif
