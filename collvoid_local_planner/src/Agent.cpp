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

#include "collvoid_local_planner/Agent.h"


namespace RVO
{
  Agent::Agent() : agentNeighbors_(), maxNeighbors_(0), maxSpeed_(0.0f), neighborDist_(0.0f), newVelocity_(), orcaLines_(), position_(), prefVelocity_(), radius_(0.0f), timeHorizon_(0.0f), timeHorizonObst_(0.0f), velocity_(), id_()
  {
  }

  Agent::~Agent()
  {
  }

  void Agent::computeNewVelocity()
  {
    orcaLines_.clear();
    const size_t numObstLines = orcaLines_.size();

    const float invTimeHorizon = 1.0f / timeHorizon_;

    /* Create agent ORCA lines. */
    for (size_t i = 0; i < agentNeighbors_.size(); ++i) {
      const Agent* const other = agentNeighbors_[i].second;

      const Vector2 relativePosition = other->position_ - position_;
      const Vector2 relativeVelocity = velocity_ - other->velocity_;
      const float distSq = absSq(relativePosition);
      const float combinedRadius = radius_ + other->radius_;
      const float combinedRadiusSq = sqr(combinedRadius);

      Line line;
      Vector2 u;

      if (distSq > combinedRadiusSq) {
        /* No collision. */
        const Vector2 w = relativeVelocity - invTimeHorizon * relativePosition; 
        /* Vector from cutoff center to relative velocity. */
        const float wLengthSq = absSq(w);

        const float dotProduct1 = w * relativePosition;

        if (dotProduct1 < 0.0f && sqr(dotProduct1) > combinedRadiusSq * wLengthSq) {
          /* Project on cut-off circle. */
          const float wLength = std::sqrt(wLengthSq);
          const Vector2 unitW = w / wLength;

          line.direction = Vector2(unitW.y(), -unitW.x());
          u = (combinedRadius * invTimeHorizon - wLength) * unitW;
        } else {
          /* Project on legs. */
          const float leg = std::sqrt(distSq - combinedRadiusSq);

          if (det(relativePosition, w) > 0.0f) {
            /* Project on left leg. */
            line.direction = Vector2(relativePosition.x() * leg - relativePosition.y() * combinedRadius, relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
          } else {
            /* Project on right leg. */
            line.direction = -Vector2(relativePosition.x() * leg + relativePosition.y() * combinedRadius, -relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
          }

          const float dotProduct2 = relativeVelocity * line.direction;

          u = dotProduct2 * line.direction - relativeVelocity;
        }
      } else {
        /* Collision. Project on cut-off circle of time timeStep. */
        const float invTimeStep = 1.0f / 0.1;

        /* Vector from cutoff center to relative velocity. */
        const Vector2 w = relativeVelocity - invTimeStep * relativePosition; 
                
        const float wLength = abs(w);
        const Vector2 unitW = w / wLength;

        line.direction = Vector2(unitW.y(), -unitW.x());
        u = (combinedRadius * invTimeStep - wLength) * unitW;
      }

      line.point = velocity_ + 0.5f * u;
      orcaLines_.push_back(line);
    }

    size_t lineFail = linearProgram2(orcaLines_, maxSpeed_, prefVelocity_, false, newVelocity_);

    if (lineFail < orcaLines_.size()) {
      linearProgram3(orcaLines_, numObstLines, lineFail, maxSpeed_, newVelocity_);
    }
  }

  void Agent::insertAgentNeighbor(const Agent* agent, float& rangeSq)
  {
    if (this != agent) {
      const float distSq = absSq(position_ - agent->position_);

      if (distSq < rangeSq) {
        if (agentNeighbors_.size() < maxNeighbors_) {
          agentNeighbors_.push_back(std::make_pair(distSq,agent));
        }
        size_t i = agentNeighbors_.size() - 1;
        while (i != 0 && distSq < agentNeighbors_[i-1].first) {
          agentNeighbors_[i] = agentNeighbors_[i-1];
          --i;
        }
        agentNeighbors_[i] = std::make_pair(distSq, agent);

        if (agentNeighbors_.size() == maxNeighbors_) {
          rangeSq = agentNeighbors_.back().first;
        }
      }
    }

  }


  void Agent::update()
  {
    velocity_ = newVelocity_;
    position_ += velocity_ * 0.1;
  }

  bool linearProgram1(const std::vector<Line>& lines, size_t lineNo, float radius, const Vector2& optVelocity, bool directionOpt, Vector2& result)
  {
    const float dotProduct = lines[lineNo].point * lines[lineNo].direction;
    const float discriminant = sqr(dotProduct) + sqr(radius) - absSq(lines[lineNo].point);
    
    if (discriminant < 0.0f) {
      /* Max speed circle fully invalidates line lineNo. */
      return false;
    }
    
    const float sqrtDiscriminant = std::sqrt(discriminant);
    float tLeft = -dotProduct - sqrtDiscriminant;
    float tRight = -dotProduct + sqrtDiscriminant;
    
    for (size_t i = 0; i < lineNo; ++i) {
      const float denominator = det(lines[lineNo].direction, lines[i].direction);
      const float numerator = det(lines[i].direction, lines[lineNo].point - lines[i].point);

      if (std::fabs(denominator) <= RVO_EPSILON) {
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

    if (directionOpt) {
      /* Optimize direction. */
      if (optVelocity * lines[lineNo].direction > 0.0f) {
        /* Take right extreme. */
        result = lines[lineNo].point + tRight * lines[lineNo].direction;
      } else {
        /* Take left extreme. */
        result = lines[lineNo].point + tLeft * lines[lineNo].direction;
      }
    } else {
      /* Optimize closest point. */
      const float t = lines[lineNo].direction * (optVelocity - lines[lineNo].point);

      if (t < tLeft) {
        result = lines[lineNo].point + tLeft * lines[lineNo].direction;
      } else if (t > tRight) {
        result = lines[lineNo].point + tRight * lines[lineNo].direction;
      } else {
        result = lines[lineNo].point + t * lines[lineNo].direction;
      }
    }

    return true;
  }

  size_t linearProgram2(const std::vector<Line>& lines, float radius, const Vector2& optVelocity, bool directionOpt, Vector2& result)
  {
    if (directionOpt) {
      /* 
       * Optimize direction. Note that the optimization velocity is of unit
       * length in this case.
       */
      result = optVelocity * radius;
    } else if (absSq(optVelocity) > sqr(radius)) {
      /* Optimize closest point and outside circle. */
      result = normalize(optVelocity) * radius;
    } else {
      /* Optimize closest point and inside circle. */
      result = optVelocity;
    }

    for (size_t i = 0; i < lines.size(); ++i) {
      if (det(lines[i].direction, lines[i].point - result) > 0.0f) {
        /* Result does not satisfy constraint i. Compute new optimal result. */
        const Vector2 tempResult = result;
        if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, result)) {
          result = tempResult;
          return i;
        }
      }
    }

    return lines.size();
  }

  void linearProgram3(const std::vector<Line>& lines, size_t numObstLines, size_t beginLine, float radius, Vector2& result)
  {
    float distance = 0.0f;

    for (size_t i = beginLine; i < lines.size(); ++i) {
      if (det(lines[i].direction, lines[i].point - result) > distance) {
        /* Result does not satisfy constraint of line i. */
        std::vector<Line> projLines(lines.begin(), lines.begin() + numObstLines);

        for (size_t j = numObstLines; j < i; ++j) {
          Line line;

          float determinant = det(lines[i].direction, lines[j].direction);

          if (std::fabs(determinant) <= RVO_EPSILON) {
            /* Line i and line j are parallel. */
            if (lines[i].direction * lines[j].direction > 0.0f) {
              /* Line i and line j point in the same direction. */
              continue;
            } else {
              /* Line i and line j point in opposite direction. */
              line.point = 0.5f * (lines[i].point + lines[j].point);
            }
          } else {
            line.point = lines[i].point + (det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction;
          }

          line.direction = normalize(lines[j].direction - lines[i].direction);
          projLines.push_back(line);
        }

        const Vector2 tempResult = result;
        if (linearProgram2(projLines, radius, Vector2(-lines[i].direction.y(), lines[i].direction.x()), true, result) < projLines.size()) {
          /* This should in principle not happen.  The result is by definition
           * already in the feasible region of this linear program. If it fails,
           * it is due to small floating point error, and the current result is
           * kept.
           */
          result = tempResult;
        }

        distance = det(lines[i].direction, lines[i].point - result);
      }
    }
  }
}
