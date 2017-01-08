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



#include "collvoid_local_planner/clearpath.h"
#include <ros/ros.h>
#include <float.h>


namespace collvoid {


    VO createVO(Vector2 &position1, const std::vector<Vector2> &footprint1, Vector2 &position2,
                const std::vector<Vector2> &footprint2, Vector2 &vel2) {
        VO result;
        std::vector<Vector2> mink_sum = minkowskiSum(footprint1, footprint2);



        Vector2 min_left, min_right;
        double min_ang = 0.0;
        double max_ang = 0.0;
        Vector2 rel_position = position2 - position1;

        Vector2 rel_position_normal = normal(rel_position);
        double min_dist = abs(rel_position);
        Vector2 null;
        bool collision = true;
        for (int i = 0; i < (int) mink_sum.size(); i++) {

            double angle = angleBetween(rel_position, rel_position + mink_sum[i]);
            if (rightOf(Vector2(0.0, 0.0), rel_position, rel_position + mink_sum[i])) {
                if (-angle < min_ang) {
                    min_right = rel_position + mink_sum[i];
                    min_ang = -angle;
                }
            }
            else {
                if (angle > max_ang) {
                    min_left = rel_position + mink_sum[i];
                    max_ang = angle;
                }
            }
            Vector2 project_on_rel_position = intersectTwoLines(Vector2(0.0, 0.0), rel_position,
                                                                rel_position + mink_sum[i], rel_position_normal);

            collvoid::Vector2 first = rel_position + mink_sum[i];
            collvoid::Vector2 second = rel_position + mink_sum[(i+1) % mink_sum.size()];

            double dist = sqrt(distSqPointLineSegment(first, second, null));
            if (signedDistPointToLineSegment(first, second, null) < 0) {
                // double dist = abs(project_on_rel_position);
                //if (project_on_rel_position * rel_position < -EPSILON) {
                collision = false;

            }

            if (dist < min_dist) {
                min_dist = dist;
            }
        }

        if (collision) {
            ROS_WARN_THROTTLE(1, "Maybe Collision?");
            result.left_leg_dir = -normalize(rel_position_normal);
            result.right_leg_dir = -result.left_leg_dir;
            result.relative_position = rel_position;
            result.combined_radius = abs(rel_position) - min_dist;
            result.point = vel2;
            return result;
        }

        double ang_rel = atan2(rel_position.y(), rel_position.x());
        result.left_leg_dir = Vector2(cos(ang_rel + max_ang), sin(ang_rel + max_ang));
        result.right_leg_dir = Vector2(cos(ang_rel + min_ang), sin(ang_rel + min_ang));

        result.left_leg_dir = rotateVectorByAngle(result.left_leg_dir, 0.05); //right pref was implemented here...
        result.right_leg_dir = rotateVectorByAngle(result.right_leg_dir, -0.05);


        double ang_between = angleBetween(result.right_leg_dir, result.left_leg_dir);
        double opening_ang = ang_rel + min_ang + (ang_between) / 2.0;

        Vector2 dir_center = Vector2(cos(opening_ang), sin(opening_ang));
        min_dist = abs(rel_position);
        Vector2 min_point = rel_position;
        for (int i = 0; i < (int) mink_sum.size(); i++) {
            Vector2 proj_on_center = intersectTwoLines(Vector2(0.0, 0.0), dir_center, rel_position + mink_sum[i],
                                                       normal(dir_center));
            if (abs(proj_on_center) < min_dist) {
                min_dist = abs(proj_on_center);
                min_point = rel_position + mink_sum[i];
            }

        }

        //ROS_ERROR("min_right %f, %f, min_left %f,%f,", min_right.x(), min_right.y(), min_left.x(), min_left.y());
        double center_p, radius;
        Vector2 center_r;
        //test left/right
        if (abs(min_left) < abs(min_right)) {
            center_p = abs(min_left) / cos(ang_between / 2.0);
            radius = tan(ang_between / 2.0) * abs(min_left);
            center_r = center_p * dir_center;
        }
        else {
            center_p = abs(min_right) / cos(ang_between / 2.0);
            center_r = center_p * dir_center;
            radius = tan(ang_between / 2.0) * abs(min_right);
        }
        //check min_point, if failed stupid calc for new radius and point;
        if (abs(min_point - center_r) > radius) {
            double gamma = min_point.x() * dir_center.x() + min_point.y() * dir_center.y();
            double sqrt_exp = absSqr(min_point) / (sqr(sin(ang_between / 2.0)) - 1) +
                              sqr(gamma / (sqr(sin(ang_between / 2.0)) - 1));
            if (fabs(sqrt_exp) < EPSILON) {
                sqrt_exp = 0;
            }
            if (sqrt_exp >= 0) {
                center_p = -gamma / (sqr(sin(ang_between / 2.0)) - 1) + std::sqrt(sqrt_exp);
                center_r = center_p * dir_center;
                radius = abs(min_point - center_r);
            }
            else {
                ROS_ERROR("ang = %f, sqrt_ext = %f", ang_between, sqrt_exp);
                ROS_ERROR("rel position %f, %f, radius %f, center_p %f", rel_position.x(), rel_position.y(), radius,
                          center_p);
            }
        }
        //result.relative_position = rel_position;
        //result.combined_radius = abs(rel_position) - min_dist;

        result.relative_position = center_r;
        result.combined_radius = radius;
        result.point = vel2;
        return result;

    }


    VO createRVO(Vector2 &position1, const std::vector<Vector2> &footprint1, Vector2 &vel1, Vector2 &position2,
                 const std::vector<Vector2> &footprint2, Vector2 &vel2) {
        VO result = createVO(position1, footprint1, position2, footprint2, vel2);
        result.point = 0.5 * (vel1 + vel2);
        return result;


    }

    VO createHRVO(Vector2 &position1, const std::vector<Vector2> &footprint1, Vector2 &vel1, Vector2 &position2,
                  const std::vector<Vector2> &footprint2, Vector2 &vel2) {
        //std::vector<Vector2> mink_sum = minkowskiSum(footprint1, footprint2);
        //std::vector<Vector2> empty;
        VO result = createRVO(position1, footprint1, vel1, position2, footprint2, vel2);


        Vector2 rel_velocity = vel1 - vel2;
        // Vector2 rel_position = position2 - position1;

        // //Test if origin is inside mink_sum (Then we are in collision)!!!)
        // bool inside = true;
        // for (int i = 0; i<(int)mink_sum.size(); i++){
        //   int j = i+1;
        //   if (j == (int) mink_sum.size())
        // 	j = 0;
        //   if (leftOf(rel_position + mink_sum[i], mink_sum[j]-mink_sum[i], Vector2(0.0,0.0))) {
        // 	inside = false;

        // 	break;
        //   }
        // }
        // Vector2 new_position = position1;
        // int max_tries = 0;
        // while (result.combined_radius > abs(result.relative_position) && max_tries > 0) {
        //   //      ROS_ERROR("inside minkowskiSum!!");
        //   new_position = new_position - normalize(result.relative_position) * 0.1;
        //   //result = createRVO(new_position, footprint1,vel1, position2, footprint2, vel2);
        //   max_tries--;
        // }
        if (result.combined_radius > abs(result.relative_position)) {
            //ROS_ERROR("comb.rad. %f, relPos %f, %f (abs = %f)", result.combined_radius, result.relative_position.x(), result.relative_position.y(), abs(result.relative_position));
            //result.point = 0.5 * (vel2 + vel1) -  ((result.combined_radius - abs(result.relative_position)) / (2.0f * 0.1)) * normalize(result.relative_position);
            return result;

        }

        if (leftOf(Vector2(0.0, 0.0), result.relative_position, rel_velocity, LEFT_PREF)) { //left of centerline
            result.point = intersectTwoLines(result.point, result.left_leg_dir, vel2,
                                             result.right_leg_dir);

        }
        else { //right of centerline
            result.point = intersectTwoLines(vel2, result.left_leg_dir, result.point,
                                             result.right_leg_dir);
        }

        return result;


    }


    VO createVO(Vector2 &position1, const std::vector<Vector2> &footprint1, Vector2 &vel1, Vector2 &position2,
                const std::vector<Vector2> &footprint2, Vector2 &vel2, int TYPE) {
        if (TYPE == HRVOS) {
            return createHRVO(position1, footprint1, vel1, position2, footprint2, vel2);
        }
        else if (TYPE == RVOS) {
            return createRVO(position1, footprint1, vel1, position2, footprint2, vel2);
        }
        else {
            return createVO(position1, footprint1, position2, footprint2, vel2);
        }
    }


    VO createVO(Vector2 &position1, double radius1, Vector2 &vel1, Vector2 &position2, double radius2, Vector2 &vel2,
                int TYPE) {
        if (TYPE == HRVOS) {
            return createHRVO(position1, radius1, vel1, position2, radius2, vel2);
        }
        else if (TYPE == RVOS) {
            return createRVO(position1, radius1, vel1, position2, radius2, vel2);
        }
        else {
            return createVO(position1, radius1, position2, radius2, vel2);
        }
    }


    VO createVO(Vector2 &position1, double radius1, Vector2 &position2, double radius2, Vector2 &vel2) {
        VO result;
        Vector2 rel_position = position2 - position1;
        double ang_to_other = atan(rel_position);
        double combined_radius = radius2 + radius1;
        double angle_of_opening;
        if (abs(rel_position) < combined_radius) {
            // angle_of_opening = M_PI/2.0-EPSILON;
            result.left_leg_dir = -normalize(normal(rel_position));
            result.right_leg_dir = -result.left_leg_dir;
            // result.right_leg_dir = Vector2(std::cos(ang_to_other - angle_of_opening), std::sin(ang_to_other - angle_of_opening));
            // result.left_leg_dir = Vector2(std::cos(ang_to_other + angle_of_opening), std::sin(ang_to_other + angle_of_opening));
        }
        else {
            angle_of_opening = std::asin(combined_radius / abs(rel_position));
            result.right_leg_dir = Vector2(std::cos(ang_to_other - angle_of_opening),
                                           std::sin(ang_to_other - angle_of_opening));
            result.left_leg_dir = Vector2(std::cos(ang_to_other + angle_of_opening),
                                          std::sin(ang_to_other + angle_of_opening));
        }
        //    ROS_ERROR("angle_of_opening %f, combined_radius %f, rel_position %f", angle_of_opening, combined_radius, abs(rel_position));
        result.point = vel2;
        result.relative_position = rel_position;
        result.combined_radius = radius1 + radius2;

        return result;
    }

    VO createRVO(Vector2 &position1, double radius1, Vector2 &vel1, Vector2 &position2, double radius2, Vector2 &vel2) {
        VO result = createVO(position1, radius1, position2, radius2, vel2);
        result.point = 0.5 * (vel1 + vel2);
        return result;
    }

    VO createHRVO(Vector2 &position1, double radius1, Vector2 &vel1, Vector2 &position2, double radius2,
                  Vector2 &vel2) {

        VO result = createRVO(position1, radius1, vel1, position2, radius2, vel2);

        Vector2 rel_velocity = vel1 - vel2;
        Vector2 rel_position = position2 - position1;
        if (abs(rel_position) < radius1 + radius2) {
            result.point = 0.5 * (vel2 + vel1);
            return result;
        }

        if (leftOf(Vector2(0.0, 0.0), rel_position, rel_velocity, LEFT_PREF)) { //left of centerline
            result.point = intersectTwoLines(result.point, result.left_leg_dir, vel2,
                                             result.right_leg_dir);
        }
        else { //right of centerline
            result.point = intersectTwoLines(vel2, result.left_leg_dir, result.point,
                                             result.right_leg_dir);
        }
        return result;

    }

    VO createTruncVO(VO &vo, double time) {
        VO result;
        result.point = vo.point;
        result.left_leg_dir = vo.left_leg_dir;
        result.right_leg_dir = vo.right_leg_dir;
        result.relative_position = vo.relative_position;
        result.combined_radius = vo.combined_radius;
        double trunc_radius = vo.combined_radius / time;
        double angle_of_opening;

        if (abs(vo.relative_position) < vo.combined_radius) {
            result.trunc_left = result.point;
            result.trunc_right = result.point;
            result.trunc_line_center = result.point;
            return result;
        }
        else {
            angle_of_opening = std::asin(vo.combined_radius / abs(vo.relative_position));
            double trunc_dist = trunc_radius / std::sin(angle_of_opening) - trunc_radius;
            result.trunc_line_center = normalize(vo.relative_position) * trunc_dist;
            Vector2 intersectLeft = intersectTwoLines(result.point + result.trunc_line_center,
                                                      Vector2(result.trunc_line_center.y(),
                                                              -result.trunc_line_center.x()), result.point,
                                                      result.left_leg_dir);
            result.trunc_left = intersectLeft;
            result.trunc_right = intersectTwoLines(result.point + result.trunc_line_center,
                                                   Vector2(result.trunc_line_center.y(), -result.trunc_line_center.x()),
                                                   result.point, result.right_leg_dir);

            return result;
        }
    }

    bool isInsideVO(VO vo, Vector2 point, bool use_truncation) {
        bool trunc = leftOf(vo.trunc_left, vo.trunc_right - vo.trunc_left, point);
        if (abs(vo.trunc_left - vo.trunc_right) < EPSILON)
            trunc = true;
        return rightOf(vo.point, vo.left_leg_dir, point) && leftOf(vo.point, vo.right_leg_dir, point) &&
               (!use_truncation || trunc);
    }

    bool isWithinAdditionalConstraints(const std::vector<Line> &additional_constraints, const Vector2 &point) {
        for(Line line: additional_constraints) {
            if (rightOf(line.point, line.dir, point)) {
                return false;
            }
        }
        return true;
    }


    void addCircleLineIntersections(std::vector<VelocitySample> &samples, const Vector2 &pref_vel, double maxSpeed,
                                    bool use_truncation, const Vector2 &point, const Vector2 &dir) {

        double discriminant = sqr(maxSpeed) - sqr(det(point,
                                                      dir)); // http://stackoverflow.com/questions/1073336/circle-line-collision-detection
        if (discriminant > 0.0f) //intersection with line
        {
            double t1 = -(point * dir) + std::sqrt(discriminant); //first solution
            double t2 = -(point * dir) - std::sqrt(discriminant); //second solution
            //ROS_ERROR("Adding circle line dist %f, %f", t1, t2);
            Vector2 point1 = point + t1 * dir;
            Vector2 point2 = point + t2 * dir;

            if (t1 >= 0.0f) {
                VelocitySample intersection_point;
                intersection_point.velocity = point1;
                intersection_point.dist_to_pref_vel = absSqr(pref_vel - intersection_point.velocity);
                samples.push_back(intersection_point);
            }
            if (t2 >= 0.0f) {
                VelocitySample intersection_point;
                intersection_point.velocity = point2;
                intersection_point.dist_to_pref_vel = absSqr(pref_vel - intersection_point.velocity);
                samples.push_back(intersection_point);
            }
        }
    }


    void addRayVelocitySamples(std::vector<VelocitySample> &samples, const std::vector<Line>& additional_constraints,
                               const Vector2 &pref_vel, Vector2 point1,
                               Vector2 dir1, Vector2 point2, Vector2 dir2, double max_speed, int TYPE) {
        double r, s;

        double x1, x2, x3, x4, y1, y2, y3, y4;
        x1 = point1.x();
        y1 = point1.y();
        x2 = x1 + dir1.x();
        y2 = y1 + dir1.y();
        x3 = point2.x();
        y3 = point2.y();
        x4 = x3 + dir2.x();
        y4 = y3 + dir2.y();

        double det = (((x2 - x1) * (y4 - y3)) - (y2 - y1) * (x4 - x3));

        if (det == 0.0) {
            //ROS_WARN("No Intersection found");
            return;
        }
        if (det != 0) {
            r = (((y1 - y3) * (x4 - x3)) - (x1 - x3) * (y4 - y3)) / det;
            s = (((y1 - y3) * (x2 - x1)) - (x1 - x3) * (y2 - y1)) / det;

            if ((TYPE == LINELINE) || (TYPE == RAYLINE && r >= 0) || (TYPE == SEGMENTLINE && r >= 0 && r <= 1) ||
                (TYPE == RAYRAY && r >= 0 && s >= 0) ||
                (TYPE == RAYSEGMENT && r >= 0 && s >= 0 && s <= 1) ||
                (TYPE == SEGMENTSEGMENT && r >= 0 && s >= 0 && r <= 1 && s <= 1)) {


                VelocitySample intersection_point;
                intersection_point.velocity = Vector2(x1 + r * (x2 - x1), y1 + r * (y2 - y1));
                intersection_point.dist_to_pref_vel = absSqr(pref_vel - intersection_point.velocity);
                //if (isWithinAdditionalConstraints(additional_constraints, intersection_point.velocity)) {
                //if (absSqr(intersection_point.velocity) < sqr(1.2 * max_speed)) {
                //ROS_ERROR("adding VelocitySample");
                samples.push_back(intersection_point);
                //ROS_ERROR("size of VelocitySamples %d", (int) samples->size());

                //}
            }
        }
    }

    void createClearpathSamples(std::vector<VelocitySample> &samples, const std::vector<VO> &all_vos,
                                const std::vector<VO> &human_vos, const std::vector<VO> &agent_vos, const std::vector<VO> &static_vos,
                                const std::vector<Line> &additional_constraints, const Vector2 &pref_vel,  const Vector2 &cur_vel,
                                double max_speed, bool use_truncation
    ){
        if (!isWithinAdditionalConstraints(additional_constraints, pref_vel)) {
            for(Line line: additional_constraints) {
                VelocitySample pref_vel_sample;
                pref_vel_sample.velocity = intersectTwoLines(line.point, line.dir, pref_vel,
                                                             Vector2(line.dir.y(), -line.dir.x()));
                pref_vel_sample.dist_to_pref_vel = absSqr(pref_vel - pref_vel_sample.velocity);
                if (isWithinAdditionalConstraints(additional_constraints, pref_vel_sample.velocity)) {
                    samples.push_back(pref_vel_sample);
                }
            }
        }
        else {
            VelocitySample pref_vel_sample;
            pref_vel_sample.velocity = pref_vel;
            pref_vel_sample.dist_to_pref_vel = 0;
            samples.push_back(pref_vel_sample);
        }
        VelocitySample null_vel_sample;
        null_vel_sample.velocity = Vector2(0, 0);
        null_vel_sample.dist_to_pref_vel = absSqr(pref_vel);
        samples.push_back(null_vel_sample);

        for(Line line: additional_constraints) {
            for(Line line2: additional_constraints) {
                addRayVelocitySamples(samples, additional_constraints, pref_vel, line.point,
                                      line.dir, line2.point, line2.dir, max_speed, LINELINE);
            }
        }

        for (int i = 0; i < (int) all_vos.size(); i++) {
            if (isInsideVO(all_vos[i], pref_vel, use_truncation)) {

                VelocitySample leg_projection;
                if (leftOf(all_vos[i].point, all_vos[i].relative_position,
                           pref_vel)) { //left of centerline, project on left leg
                    leg_projection.velocity = intersectTwoLines(all_vos[i].point, all_vos[i].left_leg_dir,
                                                                pref_vel, Vector2(all_vos[i].left_leg_dir.y(),
                                                                                  -all_vos[i].left_leg_dir.x()));
                }
                else { //project on right leg
                    leg_projection.velocity = intersectTwoLines(all_vos[i].point, all_vos[i].right_leg_dir,
                                                                pref_vel, Vector2(all_vos[i].right_leg_dir.y(),
                                                                                  -all_vos[i].right_leg_dir.x()));
                }

//if(absSqr(leg_projection.velocity) < max_speed) { //only add if below max_speed
                leg_projection.dist_to_pref_vel = absSqr(pref_vel - leg_projection.velocity);
//if (isWithinAdditionalConstraints(additional_constraints, leg_projection.velocity)) {
                samples.push_back(leg_projection);
//}
//}

                if (use_truncation) {
                    addRayVelocitySamples(samples, additional_constraints, pref_vel, pref_vel,
                                          -all_vos[i].relative_position,
                                          all_vos[i].trunc_left,
                                          all_vos[i].trunc_right - all_vos[i].trunc_left, max_speed,
                                          RAYSEGMENT);
                }
            }
        }


        for (int i = 0; i < (int) all_vos.size(); i++) {
            for (int j = 0; j < (int) additional_constraints.size(); j++) {
                if (!use_truncation) {
                    addRayVelocitySamples(samples, additional_constraints, pref_vel, all_vos[i].point,
                                          all_vos[i].left_leg_dir,
                                          additional_constraints[j].point, additional_constraints[j].dir, max_speed,
                                          RAYLINE);
                    addRayVelocitySamples(samples, additional_constraints, pref_vel, all_vos[i].point,
                                          all_vos[i].right_leg_dir,
                                          additional_constraints[j].point, additional_constraints[j].dir, max_speed,
                                          RAYLINE);
                }
                else {
                    addRayVelocitySamples(samples, additional_constraints, pref_vel, all_vos[i].trunc_left,
                                          all_vos[i].left_leg_dir,
                                          additional_constraints[j].point, additional_constraints[j].dir, max_speed,
                                          RAYLINE);
                    addRayVelocitySamples(samples, additional_constraints, pref_vel, all_vos[i].trunc_right,
                                          all_vos[i].right_leg_dir, additional_constraints[j].point,
                                          additional_constraints[j].dir, max_speed, RAYLINE);
                    addRayVelocitySamples(samples, additional_constraints, pref_vel, all_vos[i].trunc_left,
                                          all_vos[i].trunc_right - all_vos[i].trunc_left,
                                          additional_constraints[j].point, additional_constraints[j].dir, max_speed,
                                          SEGMENTLINE);
                }
            }

            for (int j = i + 1; j < (int) all_vos.size(); j++) {

                if (!use_truncation) {
                    addRayVelocitySamples(samples, additional_constraints, pref_vel, all_vos[i].point,
                                          all_vos[i].left_leg_dir,
                                          all_vos[j].point, all_vos[j].left_leg_dir, max_speed, RAYRAY);
                    addRayVelocitySamples(samples, additional_constraints, pref_vel, all_vos[i].point,
                                          all_vos[i].left_leg_dir,
                                          all_vos[j].point, all_vos[j].right_leg_dir, max_speed, RAYRAY);
                    addRayVelocitySamples(samples, additional_constraints, pref_vel, all_vos[i].point,
                                          all_vos[i].right_leg_dir,
                                          all_vos[j].point, all_vos[j].left_leg_dir, max_speed, RAYRAY);
                    addRayVelocitySamples(samples, additional_constraints, pref_vel, all_vos[i].point,
                                          all_vos[i].right_leg_dir,
                                          all_vos[j].point, all_vos[j].right_leg_dir, max_speed, RAYRAY);

                }
                else {
                    addRayVelocitySamples(samples, additional_constraints, pref_vel, all_vos[i].trunc_left,
                                          all_vos[i].left_leg_dir,
                                          all_vos[j].trunc_left, all_vos[j].left_leg_dir, max_speed,
                                          RAYRAY);
                    addRayVelocitySamples(samples, additional_constraints, pref_vel, all_vos[i].trunc_left,
                                          all_vos[i].left_leg_dir,
                                          all_vos[j].trunc_right, all_vos[j].right_leg_dir, max_speed,
                                          RAYRAY);
                    addRayVelocitySamples(samples, additional_constraints, pref_vel, all_vos[i].trunc_right,
                                          all_vos[i].right_leg_dir, all_vos[j].trunc_left,
                                          all_vos[j].left_leg_dir, max_speed, RAYRAY);
                    addRayVelocitySamples(samples, additional_constraints, pref_vel, all_vos[i].trunc_right,
                                          all_vos[i].right_leg_dir, all_vos[j].trunc_left,
                                          all_vos[j].right_leg_dir, max_speed, RAYRAY);


                    addRayVelocitySamples(samples, additional_constraints, pref_vel, all_vos[i].trunc_left,
                                          all_vos[i].left_leg_dir,
                                          all_vos[j].trunc_left,
                                          all_vos[j].trunc_right - all_vos[j].trunc_left, max_speed,
                                          RAYSEGMENT); //left trunc
                    addRayVelocitySamples(samples, additional_constraints, pref_vel, all_vos[j].trunc_left,
                                          all_vos[j].left_leg_dir,
                                          all_vos[i].trunc_left,
                                          all_vos[i].trunc_right - all_vos[i].trunc_left, max_speed,
                                          RAYSEGMENT); //trunc left

                    addRayVelocitySamples(samples, additional_constraints, pref_vel, all_vos[i].trunc_right,
                                          all_vos[i].right_leg_dir, all_vos[j].trunc_left,
                                          all_vos[j].trunc_right - all_vos[j].trunc_left, max_speed,
                                          RAYSEGMENT); //right trunc
                    addRayVelocitySamples(samples, additional_constraints, pref_vel, all_vos[j].trunc_right,
                                          all_vos[j].right_leg_dir, all_vos[i].trunc_left,
                                          all_vos[i].trunc_right - all_vos[i].trunc_left, max_speed,
                                          RAYSEGMENT); //trunc right

                    addRayVelocitySamples(samples, additional_constraints, pref_vel, all_vos[i].trunc_left,
                                          all_vos[i].trunc_right - all_vos[i].trunc_left,
                                          all_vos[j].trunc_left,
                                          all_vos[j].trunc_right - all_vos[j].trunc_left, max_speed,
                                          SEGMENTSEGMENT); //trunc trunc


                }

            }
        }
    }


    void createSamplesWithinMovementConstraints(std::vector<VelocitySample> &samples, double cur_vel_x,
                                                double cur_vel_y, double cur_vel_theta, base_local_planner::LocalPlannerLimits &limits, double heading,
                                                Vector2 pref_vel, double sim_period, int num_samples, bool holo_robot) {

        if (holo_robot) {//holonomic drive
            double min_x, max_x, min_y, max_y;

            min_x = std::max(-limits.max_vel_x, cur_vel_x - limits.acc_lim_x * sim_period);
            max_x = std::min(limits.max_vel_x, cur_vel_x + limits.acc_lim_x * sim_period);

            min_y = std::max(-limits.max_vel_y, cur_vel_y - limits.acc_lim_y * sim_period);
            max_y = std::min(limits.max_vel_y, cur_vel_y + limits.acc_lim_y * sim_period);

            double step_x, step_y;

            int num_samples_per_dir = (int) std::sqrt(num_samples);

            step_x = (max_x - min_x) / num_samples_per_dir;
            step_y = (max_y - min_y) / num_samples_per_dir;

            for (int i = 0; i < num_samples_per_dir; i++) {
                for (int j = 0; j < num_samples_per_dir; j++) {
                    VelocitySample p;
                    Vector2 vel = Vector2(min_x + i * step_x, min_y + j * step_y);
                    p.dist_to_pref_vel = absSqr(vel - Vector2(cur_vel_x, cur_vel_y));
                    p.velocity = rotateVectorByAngle(Vector2(min_x + i * step_x, min_y + j * step_y), heading);
                    samples.push_back(p);
                }

            }

        }
        else {
            double min_x, max_x, min_theta, max_theta;

            min_x = -0.3;//std::max(-0.2, cur_vel_x - acc_lim_x * sim_period);
            max_x = std::min(limits.max_vel_x, cur_vel_x + limits.acc_lim_x * sim_period);

            //cur_vel_theta = 0.0; //HACK

            min_theta = -2 * limits.max_rot_vel;//std::max(-max_vel_theta, cur_vel_theta - acc_lim_theta * sim_period);
            max_theta = 2 * limits.max_rot_vel;//std::min(max_vel_theta, cur_vel_theta + acc_lim_theta * sim_period);

            double step_x, step_theta;

            int num_samples_per_dir = (int) std::sqrt(num_samples);

            step_x = (max_x - min_x) / num_samples_per_dir;
            step_theta = (max_theta - min_theta) / num_samples_per_dir;

            // ROS_ERROR("heading %f, min_theta %f, %f cur_vel_theta %f", heading, min_theta, max_theta, cur_vel_theta);

            for (int i = 0; i < num_samples_per_dir; i++) {
                for (int j = 0; j < num_samples_per_dir; j++) {
                    VelocitySample p;
                    double th_dif = min_theta + j * step_theta;
                    double x_dif = (min_x + i * step_x) * cos(heading + th_dif / 2.0);
                    double y_dif = (min_x + i * step_x) * sin(heading + th_dif / 2.0);

                    p.velocity = Vector2(x_dif, y_dif);
                    p.dist_to_pref_vel = absSqr(Vector2(cur_vel_x, cur_vel_y));

                    //p.velocity = rotateVectorByAngle(Vector2(min_x + i* step_x, 0), heading + min_theta + j * step_theta);

                    samples.push_back(p);
                }

            }
        }

    }



    double calculateVelCosts(const Vector2 &test_vel, const std::vector<VO> &truncated_vos, bool use_truncation) {
        double cost = 0.0;
        double COST_IN_VO = 2.0;
        for (int j = 0; j < (int) truncated_vos.size(); j++) {
            if (isInsideVO(truncated_vos[j], test_vel, use_truncation)) {
                cost += (truncated_vos.size() - j) * COST_IN_VO;
            }
        }
        return cost;
    }



    bool isSafeVelocity(const std::vector<VO>& truncated_vos, Vector2 vel, bool use_truncation) {
        for (int j = 0; j < (int) truncated_vos.size(); j++) {
            if (isInsideVO(truncated_vos[j],vel, use_truncation)) {
                return false;
            }
        }
        return true;
    }

    double minDistToVOs(const std::vector<VO>& vos, Vector2 point, bool use_truncation, bool return_negative) {
        double dist = DBL_MAX;
        if (vos.size() == 0) {

            return 1.;
        }
        for(VO vo: vos) {
            double d = distToVO(vo, point, use_truncation, return_negative);
            if (d < dist) {
                dist = d;
            }
        }
        return dist;
    }

    double distToVO(VO vo, Vector2 point, bool use_truncation, bool return_negative) {
        double factor = 1;
        if (isInsideVO(vo, point, use_truncation)) {
            if (!return_negative)
                return -1.;
            else
                factor = -1;
        }
        double dist;
        if (use_truncation) {
            //if (leftOf(vo.trunc_left, vo.trunc_left-vo.trunc_right, point, 2 * EPSILON)) {
            dist = sqrt(distSqPointLineSegment(vo.trunc_left, vo.trunc_right, point));
            //}
            //else if (leftOf(vo.trunc_left, vo.left_leg_dir, point, 2 * EPSILON)) {
            dist = std::min(dist,sqrt(distSqPointRay(vo.trunc_left, vo.left_leg_dir, point)));
            //}
            //else {
            dist = std::min(dist,sqrt(distSqPointRay(vo.trunc_right, vo.right_leg_dir, point)));
            //}
        }
        else {
            dist = sqrt(distSqPointRay(vo.point, vo.left_leg_dir, point));
            dist = std::min(dist, sqrt(distSqPointRay(vo.point, vo.right_leg_dir, point)));
        }

        return factor * dist;
    }


    void createSamplesAroundOptVel(std::vector<VelocitySample> &samples, double max_dist_x,
                                   double max_dist_y, double min_vel_x,
                                   double max_vel_x, double min_vel_y, double max_vel_y,
                                   Vector2 opt_vel, int num_samples) {

        double min_x, max_x, min_y, max_y;

        min_x = std::max(min_vel_x, opt_vel.x() - max_dist_x);
        max_x = std::min(max_vel_x, opt_vel.x() + max_dist_x);

        min_y = std::max(min_vel_y, opt_vel.y() - max_dist_y);
        max_y = std::min(max_vel_y, opt_vel.y() + max_dist_y);

        double step_x, step_y;

        int num_samples_per_dir = (int) std::sqrt(num_samples);

        step_x = (max_x - min_x) / num_samples_per_dir;
        step_y = (max_y - min_y) / num_samples_per_dir;
        //ROS_ERROR("step_x, step_y (%f, %f), opt_vel (%f, %f), minx, miny %f, %f", step_x, step_y, opt_vel.x(), opt_vel.y(), min_x, min_y );
        for (int i = 0; i < num_samples_per_dir; i++) {
            for (int j = 0; j < num_samples_per_dir; j++) {
                VelocitySample p;
                p.velocity = Vector2(min_x + i * step_x, min_y + j * step_y);
                p.dist_to_pref_vel = absSqr(p.velocity - opt_vel);
                samples.push_back(p);
            }

        }
    }


    std::vector<Vector2> minkowskiSum(const std::vector<Vector2> polygon1, const std::vector<Vector2> polygon2) {
        std::vector<Vector2> result;
        std::vector<ConvexHullPoint> convex_hull;


        for (int i = 0; i < (int) polygon1.size(); i++) {
            for (int j = 0; j < (int) polygon2.size(); j++) {
                ConvexHullPoint p;
                p.point = polygon1[i] + polygon2[j];
                convex_hull.push_back(p);
            }

        }
        convex_hull = convexHull(convex_hull, false);
        for (int i = 0; i < (int) convex_hull.size(); i++) {
            result.push_back(convex_hull[i].point);
        }
        return result;

    }


    bool compareVectorsLexigraphically(const ConvexHullPoint &v1, const ConvexHullPoint &v2) {
        return v1.point.x() < v2.point.x() || (v1.point.x() == v2.point.x() && v1.point.y() < v2.point.y());
    }

    double cross(const ConvexHullPoint &O, const ConvexHullPoint &A, const ConvexHullPoint &B) {
        return (A.point.x() - O.point.x()) * (B.point.y() - O.point.y()) -
               (A.point.y() - O.point.y()) * (B.point.x() - O.point.x());
    }

    // Returns a list of points on the convex hull in counter-clockwise order.
    // Note: the last point in the returned list is the same as the first one.
    //Wikipedia Monotone chain...
    std::vector<ConvexHullPoint> convexHull(std::vector<ConvexHullPoint> P, bool sorted) {
        int n = P.size(), k = 0;
        std::vector<ConvexHullPoint> result(2 * n);

        // Sort points lexicographically
        if (!sorted)
            sort(P.begin(), P.end(), compareVectorsLexigraphically);

        //    ROS_WARN("points length %d", (int)P.size());

        // Build lower hull
        for (int i = 0; i < n; i++) {
            while (k >= 2 && cross(result[k - 2], result[k - 1], P[i]) <= 0) k--;
            result[k++] = P[i];
        }

        // Build upper hull
        for (int i = n - 2, t = k + 1; i >= 0; i--) {
            while (k >= t && cross(result[k - 2], result[k - 1], P[i]) <= 0) k--;
            result[k++] = P[i];
        }
        result.resize(k);

        return result;
    }


    bool compareVelocitySamples(const VelocitySample &p1, const VelocitySample &p2) {
        return p1.dist_to_pref_vel < p2.dist_to_pref_vel;
    }

    Vector2 intersectTwoLines(Vector2 point1, Vector2 dir1, Vector2 point2, Vector2 dir2) {
        double x1, x2, x3, x4, y1, y2, y3, y4;
        x1 = point1.x();
        y1 = point1.y();
        x2 = x1 + dir1.x();
        y2 = y1 + dir1.y();
        x3 = point2.x();
        y3 = point2.y();
        x4 = x3 + dir2.x();
        y4 = y3 + dir2.y();

        double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

        if (det == 0) {
            return Vector2(0, 0); //TODO fix return NULL

        }
        double x_i = ((x3 - x4) * (x1 * y2 - y1 * x2) - (x1 - x2) * (x3 * y4 - y3 * x4)) / det;
        double y_i = ((y3 - y4) * (x1 * y2 - y1 * x2) - (y1 - y2) * (x3 * y4 - y3 * x4)) / det;

        return Vector2(x_i, y_i);
    }

    Vector2 intersectTwoLines(Line line1, Line line2) {
        return intersectTwoLines(line1.point, line1.dir, line2.point, line2.dir);
    }



    std::vector<Vector2> rotateFootprint(const std::vector<Vector2> &footprint, double angle) {
        std::vector<Vector2> result;
        for(Vector2 point: footprint) {
            Vector2 rotated = rotateVectorByAngle(point, angle);
            result.push_back(rotated);
        }
        return result;
    }



}
