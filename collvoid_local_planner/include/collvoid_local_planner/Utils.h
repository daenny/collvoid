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

#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <limits>
#include <vector>

#include "collvoid_local_planner/Vector2.h"

static const float EPSILON = 0.001f;

namespace collvoid {
    struct Line {
      Vector2 point;
      Vector2 dir;
    };

    struct VO {
      Vector2 point;

      Vector2 relative_position;
      double combined_radius;

      Vector2 left_leg_dir;
      Vector2 right_leg_dir;

      Vector2 trunc_line_center;
      Vector2 trunc_left;
      Vector2 trunc_right;

    };



    struct VelocitySample {
      Vector2 velocity;
      double dist_to_pref_vel;
        double cost;
    };

    struct ConvexHullPoint {
      Vector2 point;
      double weight;
      int index;
      int orig_index;
    };


    inline Vector2 projectPointOnLine(const Vector2 &pointLine, const Vector2 &dirLine, const Vector2 &point) {
      const double r = ((point - pointLine) * (dirLine)) / absSqr(dirLine);
      return pointLine + r * dirLine;
    }


     /*!
     *  @brief      Computes the squared distance from a ray with the
     *              specified endpoints to a specified point.
     *  @param      a               point on the ray.
     *  @param      b               direction of the ray
     *  @param      c               The point to which the squared distance is to
     *                              be calculated.
     *  @returns    The squared distance from the ray to the point.
     */
    inline double distSqPointRay(const Vector2 &ray_begin, const Vector2 &ray_dir,
                                         const Vector2 &point) {
      const double r = ((point - ray_begin) * (ray_dir)) / absSqr(ray_dir);

      if (r < 0.0f) {
        return absSqr(point - ray_begin);
      } else {
        return absSqr(point - (ray_begin + r * ray_dir));
      }
    }


    /*!
     *  @brief      Computes the squared distance from a line segment with the
     *              specified endpoints to a specified point.
     *  @param      a               The first endpoint of the line segment.
     *  @param      b               The second endpoint of the line segment.
     *  @param      c               The point to which the squared distance is to
     *                              be calculated.
     *  @returns    The squared distance from the line segment to the point.
     */
    inline double distSqPointLineSegment(const Vector2 &seg_a, const Vector2 &seg_b,
                                         const Vector2 &point) {
      const double r = ((point - seg_a) * (seg_b - seg_a)) / absSqr(seg_b - seg_a);

      if (r < 0.0f) {
        return absSqr(point - seg_a);
      } else if (r > 1.0f) {
        return absSqr(point - seg_b);
      } else {
        return absSqr(point - (seg_a + r * (seg_b - seg_a)));
      }
    }

    /*!
     *  @brief      Computes the sign from a line connecting the
     *              specified points to a specified point.
     *  @param      a               The first point on the line.
     *  @param      b               The second point on the line.
     *  @param      c               The point to which the signed distance is to
     *                              be calculated.
     *  @returns    Positive when the point c lies to the left of the line ab.
     */
    inline double signedDistPointToLineSegment(const Vector2 &a, const Vector2 &b, const Vector2 &c) {
      return det(a - c, b - a);
    }

    inline double left(const Vector2 &pointLine, const Vector2 &dirLine, const Vector2 &point) {
      return signedDistPointToLineSegment(pointLine, pointLine + dirLine, point);
    }

    inline bool leftOf(const Vector2 &pointLine, const Vector2 &dirLine, const Vector2 &point) {
      return signedDistPointToLineSegment(pointLine, pointLine + dirLine, point) > EPSILON;
    }

    inline bool leftOf(const Vector2 &pointLine, const Vector2 &dirLine, const Vector2 &point, const float left_pref) {
      return signedDistPointToLineSegment(pointLine, pointLine + dirLine, point) > EPSILON - left_pref;
    }

    inline bool rightOf(const Vector2 &pointLine, const Vector2 &dirLine, const Vector2 &point) {
      return signedDistPointToLineSegment(pointLine, pointLine + dirLine, point) < -EPSILON;
    }

    inline double sign(double x) {
      return x < 0.0 ? -1.0 : 1.0;
    }

    inline double sqr(double a) {
      return a * a;
    }




}

#endif
