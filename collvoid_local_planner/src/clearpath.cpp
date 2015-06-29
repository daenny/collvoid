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
#include <boost/foreach.hpp>



namespace collvoid {


    // VO createObstacleVO(Vector2& position1, const std::vector<Vector2>& footprint1,Vector2& vel1,  Vector2& position2, const std::vector<Vector2>& footprint2){
    //   VO result;
    //   std::vector<Vector2> mink_sum = minkowskiSum(footprint1, footprint2);

    //   Vector2 rel_position = position2 - position1;

    //   Vector2 rel_position_normal = normal(rel_position);
    //   double min_dist = abs(rel_position);

    //   for (int i = 0; i< (int) mink_sum.size(); i++){
    //     Vector2 project_on_rel_position = intersectTwoLines(Vector2(0.0, 0.0), rel_position, rel_position + mink_sum[i], rel_position_normal);
    //     double dist = abs(project_on_rel_position);
    //     if (project_on_rel_position * rel_position < EPSILON){
    // 	//	ROS_ERROR("Collision?");
    // 	dist = -dist;

    //     }

    //     if (dist < min_dist) {
    // 	min_dist = dist;
    //     }
    //   }
    //   result.left_leg_dir = - normalize(rel_position_normal);
    //   result.right_leg_dir = - result.left_leg_dir;
    //   result.relative_position = rel_position;
    //   result.combined_radius = abs(rel_position) - min_dist;

    //   result.point = normalize(rel_position) * min_dist; // * std::max(1 - abs(vel1), 0.0);
    //   result.trunc_left = result.point;
    //   result.trunc_right = result.point;
    //   result.trunc_line_center = result.point;

    //   return result;

    // }


    VO createObstacleVO(Vector2 &position1, const std::vector<Vector2> &footprint1, const std::vector<Vector2> &obst, Vector2 &obst_position) {
        VO result;
        std::vector<Vector2> mink_sum = minkowskiSum(footprint1, obst);
        Vector2 min_left, min_right;
        double min_ang = 0.0;
        double max_ang = 0.0;
        Vector2 rel_position = obst_position - position1;
        Vector2 rel_position_normal = normal(rel_position);
        double min_dist = abs(rel_position);

        for (int i = 0; i < (int) mink_sum.size(); i++) {
            double angle = angleBetween(rel_position, mink_sum[i]);
            if (rightOf(Vector2(0.0, 0.0), rel_position, mink_sum[i])) {
                if (-angle < min_ang) {
                    min_right = mink_sum[i];
                    min_ang = -angle;
                }
            }
            else {
                if (angle > max_ang) {
                    min_left = mink_sum[i];
                    max_ang = angle;
                }
            }
            Vector2 project_on_rel_position = intersectTwoLines(Vector2(0.0, 0.0), rel_position,
                                                                mink_sum[i], rel_position_normal);
            double dist = abs(project_on_rel_position);
            if (project_on_rel_position * rel_position < -EPSILON) {
                //	ROS_ERROR("Collision?");
                dist = -dist;

            }

            if (dist < min_dist) {
                min_dist = dist;
            }
        }
        if (min_dist < 0) {
            result.left_leg_dir = -normalize(rel_position_normal);
            result.right_leg_dir = -result.left_leg_dir;
            result.relative_position = rel_position;
            result.combined_radius = abs(rel_position) - min_dist;
            result.point = min_dist * normalize(rel_position);
            return result;

        }

        double ang_rel = atan2(rel_position.y(), rel_position.x());
        result.left_leg_dir = Vector2(cos(ang_rel + max_ang), sin(ang_rel + max_ang));
        result.right_leg_dir = Vector2(cos(ang_rel + min_ang), sin(ang_rel + min_ang));

        result.left_leg_dir = rotateVectorByAngle(result.left_leg_dir, 0.05);
        result.right_leg_dir = rotateVectorByAngle(result.right_leg_dir, -0.05);

        result.relative_position = rel_position;
        result.combined_radius = abs(rel_position) - min_dist;
        result.point = Vector2(0, 0);

        result.trunc_left = intersectTwoLines(result.point, result.left_leg_dir,
                                              result.combined_radius / 4.0 * normalize(rel_position), rel_position_normal);
        result.trunc_right = intersectTwoLines(result.point, result.right_leg_dir,
                                               result.combined_radius / 4.0 * normalize(rel_position), rel_position_normal);
        result.trunc_line_center = (result.trunc_left + result.trunc_right )/ 2.;
        //result = createTruncVO(result, 6.0);
        return result;
    }


    /*
    VO createObstacleVO(Vector2 &position1, double radius1, const std::vector<Vector2> &footprint1, Vector2 &obst1,
                        Vector2 &obst2) {
        VO result;

        Vector2 position_obst = 0.5 * (obst1 + obst2);

        std::vector<Vector2> obst;
        obst.push_back(obst1 - position_obst);
        obst.push_back(obst2 - position_obst);

        std::vector<Vector2> mink_sum = minkowskiSum(footprint1, obst);

        Vector2 min_left, min_right;
        double min_ang = 0.0;
        double max_ang = 0.0;
        Vector2 rel_position = position_obst - position1;

        Vector2 rel_position_normal = normal(rel_position);
        double min_dist = abs(rel_position);

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
            double dist = abs(project_on_rel_position);
            if (project_on_rel_position * rel_position < -EPSILON) {
                //	ROS_ERROR("Collision?");
                dist = -dist;

            }

            if (dist < min_dist) {
                min_dist = dist;
            }
        }
        if (min_dist < 0) {
         result.left_leg_dir = -normalize(rel_position_normal);
            result.right_leg_dir = -result.left_leg_dir;
            result.relative_position = rel_position;
            result.combined_radius = abs(rel_position) - min_dist;
            result.point = min_dist * normalize(rel_position);
            return result;

        }

        double ang_rel = atan2(rel_position.y(), rel_position.x());
        result.left_leg_dir = Vector2(cos(ang_rel + max_ang), sin(ang_rel + max_ang));
        result.right_leg_dir = Vector2(cos(ang_rel + min_ang), sin(ang_rel + min_ang));

        result.left_leg_dir = rotateVectorByAngle(result.left_leg_dir, 0.15);
        result.right_leg_dir = rotateVectorByAngle(result.right_leg_dir, -0.05);


        //double ang_between = angleBetween(result.right_leg_dir, result.left_leg_dir);
        //double opening_ang = ang_rel + min_ang + (ang_between) / 2.0;

        // Vector2 dir_center = Vector2(cos(opening_ang), sin(opening_ang));
        // min_dist = abs(rel_position);
        // Vector2 min_point = rel_position;
        // for(int i = 0; i< (int)mink_sum.size(); i++) {
        //   Vector2 proj_on_center = intersectTwoLines(Vector2(0.0,0.0), dir_center, rel_position+mink_sum[i], normal(dir_center));
        //   if (abs(proj_on_center) < min_dist) {
        // 	min_dist = abs(proj_on_center);
        // 	min_point = rel_position+mink_sum[i];
        //   }

        // }

        //result.left_leg_dir = rotateVectorByAngle(normalize(min), 0.1);
        //result.right_leg_dir = rotateVectorByAngle(normalize(max), -0.1);

        result.relative_position = rel_position;
        result.combined_radius = abs(rel_position) - min_dist;
        result.point = Vector2(0, 0);

        result.trunc_left = intersectTwoLines(result.point, result.left_leg_dir,
                                              result.combined_radius / 2.0 * normalize(rel_position), rel_position_normal);
        result.trunc_right = intersectTwoLines(result.point, result.right_leg_dir,
                                               result.combined_radius / 2.0 * normalize(rel_position), rel_position_normal);
        result.trunc_line_center = (result.trunc_left + result.trunc_right )/ 2.;
        //result = createTruncVO(result, 6.0);
        return result;

    }
 */
    // VO createObstacleVO(Vector2& position1, double radius1,Vector2& vel1,  Vector2& position2, double radius2){
    //   VO result;

    //   Vector2 rel_position = position2 - position1;

    //   Vector2 rel_position_normal = normal(rel_position);

    //   result.left_leg_dir = - normalize(rel_position_normal);
    //   result.right_leg_dir = - result.left_leg_dir;
    //   result.relative_position = rel_position;
    //   result.combined_radius = radius1 + radius2;

    //   result.point = normalize(rel_position) * (abs(rel_position) - result.combined_radius); // * std::max(1 - abs(vel1), 0.0);
    //   result.trunc_left = result.point;
    //   result.trunc_right = result.point;
    //   result.trunc_line_center = result.point;

    //   return result;

    // }

    bool LineSegmentToLineSegmentIntersection(double x1, double y1, double x2, double y2,
                                              double x3, double y3, double x4, double y4, Vector2& result) {
        double r, s, d;
        //Make sure the lines aren't parallel
        if ((y2 - y1) / (x2 - x1) != (y4 - y3) / (x4 - x3)) {
            d = (((x2 - x1) * (y4 - y3)) - (y2 - y1) * (x4 - x3));
            if (d != 0) {
                r = (((y1 - y3) * (x4 - x3)) - (x1 - x3) * (y4 - y3)) / d;
                s = (((y1 - y3) * (x2 - x1)) - (x1 - x3) * (y2 - y1)) / d;
                if (r >= 0 && r <= 1) {
                    if (s >= 0 && s <= 1) {
                        result = collvoid::Vector2(x1 + r * (x2 - x1), y1 + r * (y2 - y1));
                        return true;
                    }
                }
            }
        }
        return false;
    }


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
            ROS_ERROR("COLLISION?");
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
        BOOST_FOREACH(Line line, additional_constraints) {
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


    void createSamplesWithinMovementConstraints(std::vector<VelocitySample> &samples, double cur_vel_x,
                                                double cur_vel_y, double cur_vel_theta, double acc_lim_x,
                                                double acc_lim_y, double acc_lim_theta, double min_vel_x,
                                                double max_vel_x, double min_vel_y, double max_vel_y,
                                                double min_vel_theta, double max_vel_theta, double heading,
                                                Vector2 pref_vel, double sim_period, int num_samples, bool holo_robot) {

        if (holo_robot) {//holonomic drive
            double min_x, max_x, min_y, max_y;

            min_x = std::max(-max_vel_x, cur_vel_x - acc_lim_x * sim_period);
            max_x = std::min(max_vel_x, cur_vel_x + acc_lim_x * sim_period);

            min_y = std::max(-max_vel_y, cur_vel_y - acc_lim_y * sim_period);
            max_y = std::min(max_vel_y, cur_vel_y + acc_lim_y * sim_period);

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
            max_x = std::min(max_vel_x, cur_vel_x + acc_lim_x * sim_period);

            //cur_vel_theta = 0.0; //HACK

            min_theta = -2 * max_vel_theta;//std::max(-max_vel_theta, cur_vel_theta - acc_lim_theta * sim_period);
            max_theta = 2 * max_vel_theta;//std::min(max_vel_theta, cur_vel_theta + acc_lim_theta * sim_period);

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

    Vector2 calculateNewVelocitySampled(std::vector<VelocitySample> &samples, const std::vector<VO> &truncated_vos,
                                        const Vector2 &pref_vel, double max_speed, const Vector2 &position, double heading, const Vector2& cur_speed, bool use_truncation,
                                        std::vector<geometry_msgs::Point> footprint_spec,
                                        costmap_2d::Costmap2D* costmap,
                                        base_local_planner::WorldModel* world_model) {

        // VelocitySample pref_vel_sample;
        // pref_vel_sample.velocity = pref_vel;
        // samples.push_back(pref_vel_sample);


        // VelocitySample null_vel;
        // null_vel.velocity = Vector2(0,0);
        // null_vel.dist_to_pref_vel = absSqr(pref_vel);
        // samples.push_back(null_vel);

        double min_cost = DBL_MAX;
        Vector2 best_vel;

        double cur_x, cur_y, pos_x, pos_y;
        cur_x = position.x();
        cur_y = position.y();


        for (int i = 0; i < (int) samples.size(); i++) {

            VelocitySample cur = samples[i];
            double cost = calculateVelCosts(cur.velocity, truncated_vos, use_truncation);
            cost += 2 * absSqr(cur.velocity - pref_vel);
            //cost += std::min(-minDistToVOs(truncated_vos, cur.velocity, use_truncation),0.1);
            cost += -minDistToVOs(truncated_vos, cur.velocity, use_truncation);
            cost += 2 * absSqr(cur.velocity - cur_speed);
            pos_x = 0.1 * cur.velocity.x();
            pos_y = 0.1 * cur.velocity.y();

            double footprint_cost = footprintCost(cur_x + pos_x, cur_y + pos_y, heading, 1.0, footprint_spec, costmap, world_model);

            //ROS_ERROR("footprint_cost %f", footprint_cost);
            if (footprint_cost < 0.) {
                samples[i].cost = -1;
                continue;
            }
            samples[i].cost = cost + footprint_cost;


            if (cost < min_cost) {
                min_cost = cost;
                best_vel = cur.velocity;
            }

        }
        //ROS_ERROR("min_cost %f", min_cost);
        return best_vel;
    }


    double footprintCost (
            const double& x,
            const double& y,
            const double& th,
            double scale,
            std::vector<geometry_msgs::Point> footprint_spec,
            costmap_2d::Costmap2D* costmap,
            base_local_planner::WorldModel* world_model) {

        //check if the footprint is legal
        // TODO: Cache inscribed radius
        double footprint_cost = world_model->footprintCost(x, y, th, footprint_spec);

        if (footprint_cost < 0) {
            return -6.0;
        }
        unsigned int cell_x, cell_y;

        //we won't allow trajectories that go off the map... shouldn't happen that often anyways
        if ( ! costmap->worldToMap(x, y, cell_x, cell_y)) {
            return -7.0;
        }

        double occ_cost = std::max(std::max(0.0, footprint_cost), double(costmap->getCost(cell_x, cell_y)));

        return occ_cost;
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


    Vector2 calculateClearpathVelocity(std::vector<VelocitySample> &samples, const std::vector<VO> &all_vos,
                                       const std::vector<VO> &human_vos, const std::vector<VO> &agent_vos, const std::vector<VO> &static_vos,
                                       const std::vector<Line> &additional_constraints, const Vector2 &pref_vel,  const Vector2 &cur_vel,
                                       double max_speed, bool use_truncation, bool new_sampling, bool use_obstacles,
                                       const Vector2 &position, double heading,
                                       std::vector<geometry_msgs::Point> footprint_spec,
                                       costmap_2d::Costmap2D* costmap,
                                       base_local_planner::WorldModel* world_model) {

        if (!isWithinAdditionalConstraints(additional_constraints, pref_vel)) {
            BOOST_FOREACH (Line line, additional_constraints) {
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

        BOOST_FOREACH(Line line, additional_constraints) {
                        BOOST_FOREACH(Line line2, additional_constraints) {
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

        // for (int i= 0 ; i< (int) all_vos.size(); i++){ //intersect with max_speed circle
        //   if (!use_truncation) {
        // 	addCircleLineIntersections(samples, pref_vel, max_speed, use_truncation, all_vos[i].point, all_vos[i].left_leg_dir); //intersect with left leg_dir
        // 	addCircleLineIntersections(samples, pref_vel, max_speed, use_truncation, all_vos[i].point, all_vos[i].right_leg_dir); //intersect with right_leg_dir
        //   }
        //   else {
        // 	addCircleLineIntersections(samples, pref_vel, max_speed, use_truncation, all_vos[i].trunc_left, all_vos[i].left_leg_dir); //intersect with left leg_dir
        // 	addCircleLineIntersections(samples, pref_vel, max_speed, use_truncation, all_vos[i].trunc_right, all_vos[i].right_leg_dir); //intersect with right_leg_dir

        //  	//addCircleLineIntersections(samples, pref_vel, max_speed, use_truncation, all_vos[i].trunc_left, all_vos[i].trunc_right - all_vos[i].trunc_left, i, all_vos[i]); //intersect with truncation line
        //   }
        // }

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


        //    ROS_ERROR("projection list length  = %d", samples.size());

        std::sort(samples.begin(), samples.end(), compareVelocitySamples);

        //Vector2 new_vel = evaluateClearpathSamples(samples, all_vos, agent_vos, human_vos, additional_constraints, pref_vel, max_speed, position, heading, cur_vel, use_truncation, footprint_spec, costmap, world_model);
        Vector2 new_vel = Vector2(0, 0); // = pref_vel;

        bool valid = false;
        bool foundOutside = false;
        bool withinConstraints = true;
        int optimal = -1;
        std::vector<VelocitySample> safeSamples;
        for (int i = 0; i < (int) samples.size(); i++) {
            withinConstraints = true;
            valid = true;
            if (!isWithinAdditionalConstraints(additional_constraints, samples[i].velocity)) {
                withinConstraints = false;
            }


            for (int j = 0; j < (int) all_vos.size(); j++) {
                if (isInsideVO(all_vos[j], samples[i].velocity, use_truncation)) {
                    valid = false;
                    if (j > optimal) {
                        optimal = j;
                        new_vel = samples[i].velocity;
                    }
                    break;
                }
            }


            double pos_x = 0.1 * samples[i].velocity.x();
            double pos_y = 0.1 * samples[i].velocity.y();

            double footprint_cost = 0.;
            if (use_obstacles)
                footprint_cost = footprintCost(position.x() + pos_x, position.y() + pos_y, heading, 1.0, footprint_spec, costmap, world_model);

            //ROS_ERROR("footprint_cost %f", footprint_cost);


            if (valid && withinConstraints && footprint_cost >= 0.) {
                new_vel = samples[i].velocity;
                safeSamples.push_back(samples[i]);
            }
            if (valid && !withinConstraints && !foundOutside) {
                optimal = all_vos.size();
                new_vel = samples[i].velocity;
                foundOutside = true;
                //TODO project on movement constraints
            }

        }

        if (safeSamples.size()>0 && new_sampling) {
            new_vel = safeSamples[0].velocity;
            double d = minDistToVOs(agent_vos, new_vel, use_truncation);
            d = std::min(minDistToVOs(human_vos, new_vel, use_truncation), d);
            if (d!=DBL_MAX) {
                ros::NodeHandle nh;
                //ROS_INFO("%s opt_vel dist %f", nh.getNamespace().c_str(), d);
            }
            // sample around optimal vel to find safe vel:
            std::vector<VelocitySample> samples_around_opt;
            for (size_t i=0; i< (size_t)std::max((int)safeSamples.size(), 3); i++) {
                createSamplesAroundOptVel(samples_around_opt, 0.2, 0.2, max_speed, max_speed, max_speed, max_speed, safeSamples[i].velocity,
                                          25);
            }
            //    ROS_INFO("selected j %d, of size %d", optimal, (int) all_vos.size());
            if (d < 2 * EPSILON && (agent_vos.size() + human_vos.size()) >0 && collvoid::abs(pref_vel)>0.1) {
                double bestDist = DBL_MAX;
                BOOST_FOREACH(VelocitySample& sample, samples_around_opt) {
                                Vector2 vel = sample.velocity;
                                if (isWithinAdditionalConstraints(additional_constraints, vel) &&
                                    isSafeVelocity(all_vos, vel, use_truncation)) {
                                     VelocitySample cur = sample;
                                    double pos_x, pos_y;
                                    pos_x = 0.1 * cur.velocity.x();
                                    pos_y = 0.1 * cur.velocity.y();

                                    if (use_obstacles) {
                                        double footprint_cost = footprintCost(position.x() + pos_x,
                                                                              position.y() + pos_y, heading, 1.0,
                                                                              footprint_spec, costmap, world_model);


                                        //ROS_ERROR("footprint_cost %f", footprint_cost);
                                        if (footprint_cost < 0.) {
                                            sample.cost = -100;

                                            continue;
                                        }
                                    }
                                    double cost = 0;
                                    cost += 2 * sqrt(absSqr(cur.velocity - pref_vel));
                                    cost += 2 * (1. - minDistToVOs(agent_vos, vel, use_truncation));
                                    cost += 3. * ( 1. - minDistToVOs(human_vos, vel, use_truncation));
                                    cost += 1 * sqrt(absSqr(cur.velocity - cur_vel));

                                    sample.cost = cost;
                                    if (cost < bestDist) {
                                        bestDist = cost;
                                        //ROS_WARN("best dist = %f", bestDist);
                                        new_vel = vel;
                                    }
                                }
                                else {
                                    sample.cost = -100;
                                }
                            }
                safeSamples.insert(safeSamples.end(), samples_around_opt.begin(), samples_around_opt.end());
            }

            samples = safeSamples;
        }
        else {
            if (safeSamples.size()>0) {
                new_vel = safeSamples[0].velocity;
            }
            else {
                ROS_WARN("Did not find safe velocity, chosing outside constraints");
                new_vel = max_speed * normalize(new_vel);
            }
        }
        return new_vel;
    }


    Vector2 evaluateClearpathSamples(std::vector<VelocitySample> &sorted_samples, const std::vector<VO> &truncated_vos, const std::vector<VO> &agent_vos,const std::vector<VO> &human_vos,
                                     const std::vector<Line> &additional_constraints, const Vector2 &pref_vel, double max_speed, const Vector2 &position, double heading, const Vector2& cur_speed, bool use_truncation,
                                     std::vector<geometry_msgs::Point> footprint_spec,
                                     costmap_2d::Costmap2D* costmap,
                                     base_local_planner::WorldModel* world_model) {

        // VelocitySample pref_vel_sample;
        // pref_vel_sample.velocity = pref_vel;
        // samples.push_back(pref_vel_sample);


        // VelocitySample null_vel;
        // null_vel.velocity = Vector2(0,0);
        // null_vel.dist_to_pref_vel = absSqr(pref_vel);
        // samples.push_back(null_vel);

        double min_cost = DBL_MAX;
        Vector2 best_vel;

        double cur_x, cur_y, pos_x, pos_y;
        cur_x = position.x();
        cur_y = position.y();
        std::vector<VelocitySample> safeSamples;
        std::vector<VelocitySample> unsafeSamples;

        double found_safe = 0;
        for (int i = 0; i < (int) sorted_samples.size() || found_safe < 3; i++) {

            VelocitySample cur = sorted_samples[i];

            pos_x = 0.1 * cur.velocity.x();
            pos_y = 0.1 * cur.velocity.y();

            double footprint_cost = footprintCost(cur_x + pos_x, cur_y + pos_y, heading, 1.0, footprint_spec, costmap,
                                                  world_model);

            //ROS_ERROR("footprint_cost %f", footprint_cost);
            if (footprint_cost < 0.) {
                sorted_samples[i].cost = -1;
                continue;
            }
            double cost = calculateVelCosts(cur.velocity, truncated_vos, use_truncation);
            if (!isWithinAdditionalConstraints(additional_constraints, cur.velocity)) {
                cost += 1000;
                unsafeSamples.push_back(cur);
            }
            else {
                found_safe++;
                safeSamples.push_back(cur);
            }
        }
        if (safeSamples.size() > 0) {
            std::vector<VelocitySample> additionalSamples;

            for (int i = 0; i < (int) safeSamples.size(); i++) {
                createSamplesAroundOptVel(additionalSamples, 0.1, 0.1, max_speed, max_speed, max_speed, max_speed, safeSamples[i].velocity, 25);
            }
            for  (int i = 0; i < (int) additionalSamples.size(); i++) {
                VelocitySample cur = sorted_samples[i];

                pos_x = 0.1 * cur.velocity.x();
                pos_y = 0.1 * cur.velocity.y();

                double footprint_cost = footprintCost(cur_x + pos_x, cur_y + pos_y, heading, 1.0, footprint_spec, costmap,
                                                  world_model);

                //ROS_ERROR("footprint_cost %f", footprint_cost);
                if (footprint_cost < 0.) {
                    sorted_samples[i].cost = -1;
                    continue;
                }
                if (!isWithinAdditionalConstraints(additional_constraints, cur.velocity)) {
                    continue;
                }
                double cost = calculateVelCosts(cur.velocity, truncated_vos, use_truncation);
                cost += 4 * sqrt(absSqr(cur.velocity - pref_vel));
                //cost += std::min(-minDistToVOs(truncated_vos, cur.velocity, use_truncation),0.1);
                cost += -minDistToVOs(agent_vos, cur.velocity, use_truncation);
                cost += -2 * minDistToVOs(human_vos, cur.velocity, false);
                cost += 2 * sqrt(absSqr(cur.velocity - cur_speed));
                additionalSamples[i].cost = cost + footprint_cost;
                if (cost < min_cost) {
                    min_cost = cost;
                    best_vel = cur.velocity;
                }

            }
        }
        else {
            if (unsafeSamples.size()>0) {
                return unsafeSamples[0].velocity;
            }
            else {
                return Vector2();
            }
        }
        //ROS_ERROR("min_cost %f", min_cost);
        return best_vel;
    }



    bool isSafeVelocity(const std::vector<VO>& truncated_vos, Vector2 vel, bool use_truncation) {
        for (int j = 0; j < (int) truncated_vos.size(); j++) {
            if (isInsideVO(truncated_vos[j],vel, use_truncation)) {
                return false;
            }
        }
        return true;
    }

    double minDistToVOs(const std::vector<VO>& vos, Vector2 point, bool use_truncation) {
        double dist = DBL_MAX;
        if (vos.size() == 0) {
            return 1.;
        }
        BOOST_FOREACH(VO vo, vos) {
                        double d = distToVO(vo, point, use_truncation);
                        if (d < dist) {
                            dist = d;
                        }
                    }
        return dist;
    }

    double distToVO(VO vo, Vector2 point, bool use_truncation) {
        if (isInsideVO(vo, point, use_truncation)) {
            return -1.;
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

        return dist;
    }


    void createSamplesAroundOptVel(std::vector<VelocitySample> &samples, double max_dist_x,
                                   double max_dist_y, double min_vel_x,
                                   double max_vel_x, double min_vel_y, double max_vel_y,
                                   Vector2 opt_vel, int num_samples) {

        double min_x, max_x, min_y, max_y;

        min_x = std::max(-max_vel_x, opt_vel.x() - max_dist_x);
        max_x = std::min(max_vel_x, opt_vel.x() + max_dist_x);

        min_y = std::max(-max_vel_y, opt_vel.y() - max_dist_y);
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
        BOOST_FOREACH(Vector2 point, footprint) {
                        Vector2 rotated = rotateVectorByAngle(point, angle);
                        result.push_back(rotated);
                    }
        return result;
    }



}
