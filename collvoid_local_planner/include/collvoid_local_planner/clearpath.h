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

#ifndef COLLVOID_CLEARPATH_H
#define COLLVOID_CLEARPATH_H

#include "collvoid_local_planner/Utils.h"
#include <geometry_msgs/Point.h>
#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/local_planner_util.h>


#define RAYRAY 0
#define RAYSEGMENT 1
#define SEGMENTSEGMENT 2
#define SEGMENTLINE 3
#define RAYLINE 4
#define LINELINE 5

#define HRVOS 0
#define RVOS 1
#define VOS 2

#define LEFT_PREF 0.2

namespace collvoid {

    struct Obstacle {
        std::vector<Vector2> points;
        ros::Time last_seen;
    };

    //Footprint based:
    VO createVO(Vector2 &position1, const std::vector<Vector2> &footprint1, Vector2 &position2,
                const std::vector<Vector2> &footprint2, Vector2 &vel2);

    VO createRVO(Vector2 &position1, const std::vector<Vector2> &footprint1, Vector2 &vel1, Vector2 &position2,
                 const std::vector<Vector2> &footprint2, Vector2 &vel2);

    VO createHRVO(Vector2 &position1, const std::vector<Vector2> &footprint1, Vector2 &vel1, Vector2 &position2,
                  const std::vector<Vector2> &footprint2, Vector2 &vel2);

    VO createVO(Vector2 &position1, const std::vector<Vector2> &footprint1, Vector2 &vel1, Vector2 &position2,
                const std::vector<Vector2> &footprint2, Vector2 &vel2, int TYPE);


    //Radius based VOs
    VO createVO(Vector2 &position1, double radius1, Vector2 &vel1, Vector2 &position2, double radius2, Vector2 &vel2,
                int TYPE);

    VO createVO(Vector2 &position1, double radius1, Vector2 &position2, double radius2, Vector2 &vel2);

    VO createRVO(Vector2 &position1, double radius1, Vector2 &vel1, Vector2 &position2, double radius2, Vector2 &vel2);

    VO createHRVO(Vector2 &position1, double radius1, Vector2 &vel1, Vector2 &position2, double radius2, Vector2 &vel2);


    //Truncate
    VO createTruncVO(VO &vo, double time);//, double combinedRadius, Vector2 relPosition);


    //Clearpath
    bool isInsideVO(VO vo, Vector2 point, bool use_truncation);

    bool isWithinAdditionalConstraints(const std::vector<Line> &additional_constraints, const Vector2 &point);

    void addCircleLineIntersections(std::vector<VelocitySample> &samples, const Vector2 &pref_vel, double max_speed,
                                    bool use_truncation, const Vector2 &point, const Vector2 &dir);

    void addRayVelocitySamples(std::vector<VelocitySample> &samples, const std::vector<Line> &additional_constraints,
                               const Vector2 &pref_vel, Vector2 point1, Vector2 dir1, Vector2 point2, Vector2 dir2,
                               double max_speed, int TYPE);

    void createClearpathSamples(std::vector<VelocitySample> &samples, const std::vector<VO> &all_vos,
                                const std::vector<VO> &human_vos, const std::vector<VO> &agent_vos, const std::vector<VO> &static_vos,
                                const std::vector<Line> &additional_constraints, const Vector2 &pref_vel,  const Vector2 &cur_vel,
                                double max_speed, bool use_truncation);

    //Sample based
    void createSamplesWithinMovementConstraints(std::vector<VelocitySample> &samples, double cur_vel_x,
                                                double cur_vel_y, double cur_vel_theta,
                                                base_local_planner::LocalPlannerLimits &limits, double heading,
                                                Vector2 pref_vel, double sim_period, int num_samples, bool holo_robot);

    double calculateVelCosts(const Vector2 &test_vel, const std::vector<VO> &truncated_vos, bool use_truncation);

    bool isSafeVelocity(const std::vector<VO> &truncated_vos, Vector2 vel, bool use_truncation);


    double distToVO(VO vo, Vector2 point, bool use_truncation, bool return_negative=false);


    void createSamplesAroundOptVel(std::vector<VelocitySample> &samples, double max_dist_x,
                                   double max_dist_y, double min_vel_x,
                                   double max_vel_x, double min_vel_y, double max_vel_y,
                                   Vector2 opt_vel, int num_samples);

    double minDistToVOs(const std::vector<VO> &vos, Vector2 point, bool use_truncation,  bool return_negative=false);


    std::vector<Vector2> minkowskiSum(const std::vector<Vector2> polygon1, const std::vector<Vector2> polygon2);

    //convex hull algorithm
    bool compareVectorsLexigraphically(const ConvexHullPoint &v1, const ConvexHullPoint &v2);

    double cross(const ConvexHullPoint &O, const ConvexHullPoint &A, const ConvexHullPoint &B);

    // Returns a list of points on the convex hull in counter-clockwise order.
    // Note: the last point in the returned list is the same as the first one.
    //Wikipedia Monotone chain...
    std::vector<ConvexHullPoint> convexHull(std::vector<ConvexHullPoint> P, bool sorted);

    bool compareVelocitySamples(const VelocitySample &p1, const VelocitySample &p2);

    Vector2 intersectTwoLines(Vector2 point1, Vector2 dir1, Vector2 point2, Vector2 dir2);

    Vector2 intersectTwoLines(Line line1, Line line2);

    bool LineSegmentToLineSegmentIntersection(double x1, double y1, double x2, double y2,
                                              double x3, double y3, double x4, double y4, Vector2& result);

    std::vector<Vector2> rotateFootprint(const std::vector<Vector2> &footprint, double angle);

}


#endif
