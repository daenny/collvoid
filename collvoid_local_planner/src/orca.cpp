/*
 * This file is based on the Agent.cpp form the RVO2 library. See original copyright information below.
 */


/*
 *  Agent.cpp
 *  RVO2 Library.
 *
 *
 *  Copyright (C) 2008-10 University of North Carolina at Chapel Hill.
 *  All rights reserved.
 *
 *  Permission to use, copy, modify, and distribute this software and its
 *  documentation for educational, research, and non-profit purposes, without
 *  fee, and without a written agreement is hereby granted, provided that the
 *  above copyright notice, this paragraph, and the following four paragraphs
 *  appear in all copies.
 *
 *  Permission to incorporate this software into commercial products may be
 *  obtained by contacting the University of North Carolina at Chapel Hill.
 *
 *  This software program and documentation are copyrighted by the University of
 *  North Carolina at Chapel Hill. The software program and documentation are
 *  supplied "as is", without any accompanying services from the University of
 *  North Carolina at Chapel Hill or the authors. The University of North
 *  Carolina at Chapel Hill and the authors do not warrant that the operation of
 *  the program will be uninterrupted or error-free. The end-user understands
 *  that the program was developed for research purposes and is advised not to
 *  rely exclusively on the program for any reason.
 *
 *  IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR ITS
 *  EMPLOYEES OR THE AUTHORS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT,
 *  SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS,
 *  ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE
 *  UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE AUTHORS HAVE BEEN ADVISED
 *  OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY
 *  DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY
 *  STATUTORY WARRANTY OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS
 *  ON AN "AS IS" BASIS, AND THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND
 *  THE AUTHORS HAVE NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, UPDATES,
 *  ENHANCEMENTS, OR MODIFICATIONS.
 *
 *  Please send all BUG REPORTS to:
 *
 *  geom@cs.unc.edu
 *
 *  The authors may be contacted via:
 *
 *  Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, and
 *  Dinesh Manocha
 *  Dept. of Computer Science
 *  Frederick P. Brooks Jr. Computer Science Bldg.
 *  3175 University of N.C.
 *  Chapel Hill, N.C. 27599-3175
 *  United States of America
 *
 *  http://gamma.cs.unc.edu/RVO2/
 *
 */

#include "collvoid_local_planner/orca.h"

namespace collvoid {


    Line createOrcaLine(Agent *me, Agent *other, double trunc_time, double timestep, double left_pref,
                        double cur_allowed_error) {

        Vector2 relativePosition = other->getPosition() - me->getPosition();
        double combinedRadius = me->getRadius() + other->getRadius();// - cur_allowed_error_; //why al

        return createOrcaLine(combinedRadius, relativePosition, me->getVelocity(), other->getVelocity(), trunc_time,
                              timestep, left_pref, cur_allowed_error, other->controlled_);
    }

    Line createOrcaLine(double combinedRadius, const Vector2 &relativePosition, const Vector2 &me_vel,
                        const Vector2 &other_vel, double trunc_time, double timestep, double left_pref,
                        double cur_allowed_error, bool controlled) {

        double invTimeHorizon = 1.0f / trunc_time;
        if (collvoid::abs(other_vel) < EPSILON) {
            invTimeHorizon = 1.0f / 2.0;
            //      return createStationaryAgent(me, other);
        }


        //const Vector2 relativePosition = other->getPosition() - me->getPosition();
        const Vector2 relativeVelocity = me_vel - other_vel;
        const double distSq = absSqr(relativePosition);
        //double combinedRadius = me->getRadius() + other->getRadius();// - cur_allowed_error_; //why allowed_error???
        double combinedRadiusSq = sqr(combinedRadius);

        Line line;
        Vector2 u;

        if (distSq > combinedRadiusSq) {
            combinedRadius += cur_allowed_error; //TODO Does this work? Maybe add uncertainty here
            combinedRadiusSq = sqr(combinedRadius);
            /* No collision. */
            const Vector2 w = relativeVelocity - invTimeHorizon * relativePosition;
            /* Vector from cutoff center to relative velocity. */
            const double wLengthSq = absSqr(w);

            const double dotProduct1 = w * relativePosition;

            if (dotProduct1 < 0.0f && sqr(dotProduct1) > combinedRadiusSq * wLengthSq) {
                /* Project on cut-off circle. */
                const double wLength = std::sqrt(wLengthSq);
                const Vector2 unitW = w / wLength;

                line.dir = Vector2(unitW.y(), -unitW.x());
                u = (combinedRadius * invTimeHorizon - wLength) * unitW;
            } else {
                /* Project on legs. */
                const double leg = std::sqrt(distSq - combinedRadiusSq);

                if (det(relativePosition, w) > -left_pref) { //define left_pref
                    /* Project on left leg. */
                    line.dir = Vector2(relativePosition.x() * leg - relativePosition.y() * combinedRadius,
                                       relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
                } else {
                    /* Project on right leg. */
                    line.dir = -Vector2(relativePosition.x() * leg + relativePosition.y() * combinedRadius,
                                        -relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
                }

                const double dotProduct2 = relativeVelocity * line.dir;

                u = dotProduct2 * line.dir - relativeVelocity;
            }
        } else {
            /* Collision. Project on cut-off circle of time timeStep. */
            const double invTimeStep = 1.0f / timestep;
            /* Vector from cutoff center to relative velocity. */
            const Vector2 w = relativeVelocity - invTimeStep * relativePosition;

            const double wLength = abs(w);
            const Vector2 unitW = w / wLength;
            //ROS_ERROR("collision");
            line.dir = Vector2(unitW.y(), -unitW.x());
            u = (combinedRadius * invTimeStep - wLength) * unitW;
        }

        double factor = 0.5;
        if (!controlled) {
            factor = 1.0;
        }
        line.point = me_vel + factor * u;
        //ROS_ERROR("%s = line: (%f,%f), dir (%f,%f)",nh.getNamespace().c_str(),line.point.x(),line.point.y(),line.dir.x(),line.dir.y());
        return line;
    }

    Line createStationaryAgent(Agent *me, Agent *other) {
        double dist = collvoid::abs(me->getPosition() - other->getPosition());
        Line line;
        Vector2 relative_position = other->getPosition() - me->getPosition();
        line.point = normalize(relative_position) *
                     (dist - me->getRadius() - other->getRadius() - 0.05);// TODO 5 cm security

        line.dir = Vector2(-normalize(relative_position).y(), normalize(relative_position).x());

        return line;

    }


    void addAccelerationConstraintsXY(double max_vel_x, double acc_lim_x, double max_vel_y, double acc_lim_y,
                                      Vector2 cur_vel, double heading, double sim_period, bool holo_robot,
                                      std::vector<Line> &additional_orca_lines) {
        double max_lim_x, max_lim_y, min_lim_x, min_lim_y;
        Line line_x_back, line_x_front, line_y_left, line_y_right;

        Vector2 dir_front = Vector2(cos(heading), sin(heading));
        Vector2 dir_right = Vector2(dir_front.y(), -dir_front.x());
        if (holo_robot) {

            cur_vel = rotateVectorByAngle(cur_vel.x(), cur_vel.y(), -heading);

            double cur_x = cur_vel.x();
            double cur_y = cur_vel.y();


            max_lim_x = std::min(max_vel_x, cur_x + sim_period * acc_lim_x);
            max_lim_y = std::min(max_vel_y, cur_y + sim_period * acc_lim_y);
            min_lim_x = std::max(-max_vel_x, cur_x - sim_period * acc_lim_x);
            min_lim_y = std::max(-max_vel_y, cur_y - sim_period * acc_lim_y);

            //      ROS_ERROR("Cur cvel (%f, %f) Max limints x = %f, %f, y = %f, %f", cur_x, cur_y, max_lim_x, min_lim_x, max_lim_y, min_lim_y);

            line_x_front.point = dir_front * max_lim_x;
            line_x_front.dir = -dir_right;

            line_x_back.point = dir_front * min_lim_x;
            line_x_back.dir = dir_right;


            additional_orca_lines.push_back(line_x_front);
            additional_orca_lines.push_back(line_x_back);

            line_y_left.point = -dir_right * max_lim_y;
            line_y_left.dir = -dir_front;

            line_y_right.point = -dir_right * min_lim_y;
            line_y_right.dir = dir_front;


            additional_orca_lines.push_back(line_y_left);
            additional_orca_lines.push_back(line_y_right);
        }
        else {

            double cur_x = collvoid::abs(cur_vel);
            max_lim_x = std::min(max_vel_x, cur_x + sim_period * acc_lim_x);
            min_lim_x = std::max(-max_vel_x, cur_x - sim_period * acc_lim_x);

            line_x_front.point = dir_front * max_lim_x;
            line_x_front.dir = -dir_right;

            line_x_back.point = dir_front * min_lim_x;
            line_x_back.dir = dir_right;

            //ROS_ERROR("Max limints x = %f, %f", max_lim_x, min_lim_x);
            additional_orca_lines.push_back(line_x_front);
            additional_orca_lines.push_back(line_x_back);


        }
    }

    void addMovementConstraintsDiffSimple(double max_track_speed, double heading,
                                          std::vector<Line> &additional_orca_lines) {
        Line maxVel1;
        Line maxVel2;

        Vector2 dir = Vector2(cos(heading), sin(heading));
        maxVel1.point = 0.95 * max_track_speed * Vector2(-dir.y(), dir.x());
        maxVel1.dir = -dir;
        maxVel2.dir = dir;
        maxVel2.point = -max_track_speed * Vector2(-dir.y(), dir.x());
        additional_orca_lines.push_back(maxVel1);
        additional_orca_lines.push_back(maxVel2);
        // Line line;
        // line.point = -2*max_track_speed * Vector2(cos(heading), sin(heading));
        // line.dir = normalize(maxVel2.point);
        // additional_orca_lines.push_back(line);
        // Line line2;
        // line2.point = 10*max_track_speed * Vector2(cos(heading), sin(heading));
        // line2.dir = rotateVectorByAngle(-normalize(maxVel2.point), -0.2);
        // additional_orca_lines.push_back(line2);
        // Line line3;
        // line3.point = 10*max_track_speed * Vector2(cos(heading), sin(heading));
        // line3.dir = rotateVectorByAngle(-normalize(maxVel2.point), 0.2);
        // additional_orca_lines.push_back(line3);


    }

    void addMovementConstraintsDiff(double error, double T, double max_vel_x, double max_vel_th, double heading,
                                    double v_max_ang, std::vector<Line> &additional_orca_lines) {

        // double steps2 = 180;
        // std::cout << error<< std::endl;
        // for (double i = -M_PI / 2.0; i <= M_PI / 2.0; i += M_PI/steps2) {
        //   double speed = calculateMaxTrackSpeedAngle(T,i, error, max_vel_x, max_vel_th, v_max_ang);
        //   //double vstarErr = calcVstarError(T,i,0.01);
        //   std::cout << speed*cos(i) << "," << speed * sin(i) << ";";
        //   //std::cout << speed << " vstar Err " << vstarErr <<" ang " << i <<std::endl;
        // }
        // std::cout << std::endl;

        // for (double i = -M_PI / 2.0; i <= M_PI / 2.0; i += M_PI/steps2) {
        //   double speed = calculateMaxTrackSpeedAngle(T,i, error, max_vel_x, max_vel_th, v_max_ang);
        //   std::cout << speed << "," << i << std::endl;
        // }
        // std::cout << std::endl;


        double min_theta = M_PI / 2.0;
        double max_track_speed = calculateMaxTrackSpeedAngle(T, min_theta, error, max_vel_x, max_vel_th, v_max_ang);

        Vector2 first_point = max_track_speed * Vector2(cos(heading - min_theta), sin(heading - min_theta));
        double steps = 10.0;
        double step_size = -M_PI / steps;
        // Line line;
        // line.point = -max_track_speed * Vector2(cos(heading), sin(heading));
        // line.dir = normalize(first_point);
        // additional_orca_lines.push_back(line);


        for (int i = 1; i <= (int) steps; i++) {

            Line line;
            double cur_ang = min_theta + i * step_size;
            Vector2 second_point = Vector2(cos(heading - cur_ang), sin(heading - cur_ang));
            double track_speed = calculateMaxTrackSpeedAngle(T, cur_ang, error, max_vel_x, max_vel_th, v_max_ang);
            second_point = track_speed * second_point;
            line.point = first_point;
            line.dir = normalize(second_point - first_point);
            additional_orca_lines.push_back(line);
            //    ROS_DEBUG("line point 1 x, y, %f, %f, point 2 = %f,%f",first_point.x(),first_point.y(),second_point.x(),second_point.y());
            first_point = second_point;
        }
    }

    double beta(double T, double theta, double v_max_ang) {
        return -((2.0 * T * T * sin(theta)) / theta) * v_max_ang;
    }

    double gamma(double T, double theta, double error, double v_max_ang) {
        return ((2.0 * T * T * (1.0 - cos(theta))) / (theta * theta)) * v_max_ang * v_max_ang - error * error;
    }

    double calcVstar(double vh, double theta) {
        return vh * ((theta * sin(theta)) / (2.0 * (1.0 - cos(theta))));
    }

    double calcVstarError(double T, double theta, double error) {
        return calcVstar(error / T, theta) *
               sqrt((2.0 * (1.0 - cos(theta))) / (2.0 * (1.0 - cos(theta)) - collvoid::sqr(sin(theta))));
    }

    double calculateMaxTrackSpeedAngle(double T, double theta, double error, double max_vel_x, double max_vel_th,
                                       double v_max_ang) {
        if (fabs(theta) <= EPSILON)
            return max_vel_x;
        if (fabs(theta / T) <= max_vel_th) {
            double vstar_error = calcVstarError(T, theta, error);
            if (vstar_error <= v_max_ang) {
                //return 0;
                return std::min(vstar_error * (2.0 * (1.0 - cos(theta))) / (theta * sin(theta)), max_vel_x);
            }
            else {
                double a, b, g;
                a = T * T;
                b = beta(T, theta, v_max_ang);
                g = gamma(T, theta, error, v_max_ang);
                //return 1;
                return std::min((-b + sqrt(collvoid::sqr(b) - 4 * collvoid::sqr(a) * g)) / (2.0 * g), max_vel_x);
            }
        }
        else
            //  return 2;
            return std::min(sign(theta) * error * max_vel_th / theta, max_vel_x);
    }


    bool linearProgram1(const std::vector<Line> &lines, size_t lineNo, float radius, const Vector2 &optVelocity,
                        bool dirOpt, Vector2 &result) {
        const float dotProduct = lines[lineNo].point * lines[lineNo].dir;
        const float discriminant = sqr(dotProduct) + sqr(radius) - absSqr(lines[lineNo].point);

        if (discriminant < 0.0f) {
            /* Max speed circle fully invalidates line lineNo. */
            return false;
        }

        const float sqrtDiscriminant = std::sqrt(discriminant);
        float tLeft = -dotProduct - sqrtDiscriminant;
        float tRight = -dotProduct + sqrtDiscriminant;

        for (size_t i = 0; i < lineNo; ++i) {
            const float denominator = det(lines[lineNo].dir, lines[i].dir);
            const float numerator = det(lines[i].dir, lines[lineNo].point - lines[i].point);

            if (std::fabs(denominator) <= EPSILON) {
                /* Lines lineNo and i are (almost) parallel. */
                if (numerator < 0.0f) {
                    return false;
                } else {
                    continue;
                }
            }

            const float t = numerator / denominator;

            if (denominator >= 0.0f) {
                /* Line i bounds line lineNo on the right. */
                tRight = std::min(tRight, t);
            } else {
                /* Line i bounds line lineNo on the left. */
                tLeft = std::max(tLeft, t);
            }

            if (tLeft > tRight) {
                return false;
            }
        }

        if (dirOpt) {
            /* Optimize direction. */
            if (optVelocity * lines[lineNo].dir > 0.0f) {
                /* Take right extreme. */
                result = lines[lineNo].point + tRight * lines[lineNo].dir;
            } else {
                /* Take left extreme. */
                result = lines[lineNo].point + tLeft * lines[lineNo].dir;
            }
        } else {
            /* Optimize closest point. */
            const float t = lines[lineNo].dir * (optVelocity - lines[lineNo].point);

            if (t < tLeft) {
                result = lines[lineNo].point + tLeft * lines[lineNo].dir;
            } else if (t > tRight) {
                result = lines[lineNo].point + tRight * lines[lineNo].dir;
            } else {
                result = lines[lineNo].point + t * lines[lineNo].dir;
            }
        }

        return true;
    }

    size_t linearProgram2(const std::vector<Line> &lines, float radius, const Vector2 &optVelocity, bool dirOpt,
                          Vector2 &result) {
        if (dirOpt) {
            /*
             * Optimize direction. Note that the optimization velocity is of unit
             * length in this case.
             */
            result = optVelocity * radius;
        } else if (absSqr(optVelocity) > sqr(radius)) {
            /* Optimize closest point and outside circle. */
            result = normalize(optVelocity) * radius;
        } else {
            /* Optimize closest point and inside circle. */
            result = optVelocity;
        }

        for (size_t i = 0; i < lines.size(); ++i) {
            if (det(lines[i].dir, lines[i].point - result) > 0.0f) {
                /* Result does not satisfy constraint i. Compute new optimal result. */
                const Vector2 tempResult = result;
                if (!linearProgram1(lines, i, radius, optVelocity, dirOpt, result)) {
                    result = tempResult;
                    return i;
                }
            }
        }

        return lines.size();
    }

    void linearProgram3(const std::vector<Line> &lines, size_t numObstLines, size_t beginLine, float radius,
                        Vector2 &result) {
        float distance = 0.0f;

        for (size_t i = beginLine; i < lines.size(); ++i) {
            if (det(lines[i].dir, lines[i].point - result) > distance) {
                /* Result does not satisfy constraint of line i. */
                std::vector<Line> projLines(lines.begin(), lines.begin() + numObstLines);

                for (size_t j = numObstLines; j < i; ++j) {
                    Line line;

                    float determinant = det(lines[i].dir, lines[j].dir);

                    if (std::fabs(determinant) <= EPSILON) {
                        /* Line i and line j are parallel. */
                        if (lines[i].dir * lines[j].dir > 0.0f) {
                            /* Line i and line j point in the same direction. */
                            continue;
                        } else {
                            /* Line i and line j point in opposite direction. */
                            line.point = 0.5f * (lines[i].point + lines[j].point);
                        }
                    } else {
                        line.point = lines[i].point +
                                     (det(lines[j].dir, lines[i].point - lines[j].point) / determinant) * lines[i].dir;
                    }

                    line.dir = normalize(lines[j].dir - lines[i].dir);
                    projLines.push_back(line);
                }

                const Vector2 tempResult = result;
                if (linearProgram2(projLines, radius, Vector2(-lines[i].dir.y(), lines[i].dir.x()), true, result) <
                    projLines.size()) {
                    /* This should in principle not happen.  The result is by definition
                     * already in the feasible region of this linear program. If it fails,
                     * it is due to small floating point error, and the current result is
                     * kept.
                     */
                    result = tempResult;
                }

                distance = det(lines[i].dir, lines[i].point - result);
            }
        }
    }


}

  
