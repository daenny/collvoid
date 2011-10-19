/*
 *  
 *
 *
 *  Created by Daniel Claes on 14.06.11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "ROSAgent.h"

using namespace RVO;

ROSAgent::ROSAgent() : 
  Agent(),
  timeStep_(0.1),
  heading_(),
  maxTrackSpeed_(),
  additionalOrcaLines_()
{
}

void ROSAgent::computeNewVelocity()
{
  orcaLines_.clear();

  //const float invTimeHorizonObst = 1.0f / timeHorizonObst_;
  const float invTimeHorizonObst = 1.0f / timeHorizonObst_;

  /* Create obstacle ORCA lines. */
  for (size_t i = 0; i < obstacleNeighbors_.size(); ++i) {

    const Obstacle* obstacle1 = obstacleNeighbors_[i].second;
    const Obstacle* obstacle2 = obstacle1->nextObstacle;

    const Vector2 relativePosition1 = obstacle1->point_ - position_;
    const Vector2 relativePosition2 = obstacle2->point_ - position_;

    /*
     * Check if velocity obstacle of obstacle is already taken care of by
     * previously constructed obstacle ORCA lines.
     */
    bool alreadyCovered = false;

    for (size_t j = 0; j < orcaLines_.size(); ++j) {
      if (det(invTimeHorizonObst * relativePosition1 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * radius_ >= -RVO_EPSILON && det(invTimeHorizonObst * relativePosition2 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * radius_ >=  -RVO_EPSILON) {
	alreadyCovered = true;
	break;
      }
    }

    if (alreadyCovered) {
      continue;
    }

    /* Not yet covered. Check for collisions. */

    const float distSq1 = absSq(relativePosition1);
    const float distSq2 = absSq(relativePosition2);

    const float radiusSq = sqr(radius_);

    const Vector2 obstacleVector = obstacle2->point_ - obstacle1->point_;
    const float s = (-relativePosition1 * obstacleVector) / absSq(obstacleVector);
    const float distSqLine = absSq(-relativePosition1 - s * obstacleVector);

    Line line;

    if (s < 0 && distSq1 <= radiusSq) {
      /* Collision with left vertex. Ignore if non-convex. */
      if (obstacle1->isConvex_) {
	line.point = Vector2(0,0);
	line.direction = normalize(Vector2(-relativePosition1.y(), relativePosition1.x()));
	orcaLines_.push_back(line);
      }
      continue;
    } else if (s > 1 && distSq2 <= radiusSq) {
      /* Collision with right vertex. Ignore if non-convex
       * or if it will be taken care of by neighoring obstace */
      if (obstacle2->isConvex_ && det(relativePosition2, obstacle2->unitDir_) >= 0) {
	line.point = Vector2(0,0);
	line.direction = normalize(Vector2(-relativePosition2.y(), relativePosition2.x()));
	orcaLines_.push_back(line);
      }
      continue;
    } else if (s >= 0 && s < 1 && distSqLine <= radiusSq) {
      /* Collision with obstacle segment. */
      line.point = Vector2(0,0);
      line.direction = -obstacle1->unitDir_;
      orcaLines_.push_back(line);
      continue;
    }

    /*
     * No collision.
     * Compute legs. When obliquely viewed, both legs can come from a single
     * vertex. Legs extend cut-off line when nonconvex vertex.
     */

    Vector2 leftLegDirection, rightLegDirection;

    if (s < 0 && distSqLine <= radiusSq) {
      /*
       * Obstacle viewed obliquely so that left vertex
       * defines velocity obstacle.
       */
      if (!obstacle1->isConvex_) {
	/* Ignore obstacle. */
	continue;
      }

      obstacle2 = obstacle1;

      const float leg1 = std::sqrt(distSq1 - radiusSq);
      leftLegDirection = Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
      rightLegDirection = Vector2(relativePosition1.x() * leg1 + relativePosition1.y() * radius_, -relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
    } else if (s > 1 && distSqLine <= radiusSq) {
      /*
       * Obstacle viewed obliquely so that
       * right vertex defines velocity obstacle.
       */
      if (!obstacle2->isConvex_) {
	/* Ignore obstacle. */
	continue;
      }

      obstacle1 = obstacle2;

      const float leg2 = std::sqrt(distSq2 - radiusSq);
      leftLegDirection = Vector2(relativePosition2.x() * leg2 - relativePosition2.y() * radius_, relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
      rightLegDirection = Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
    } else {
      /* Usual situation. */
      if (obstacle1->isConvex_) {
	const float leg1 = std::sqrt(distSq1 - radiusSq);
	leftLegDirection = Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
      } else {
	/* Left vertex non-convex; left leg extends cut-off line. */
	leftLegDirection = -obstacle1->unitDir_;
      }

      if (obstacle2->isConvex_) {
	const float leg2 = std::sqrt(distSq2 - radiusSq);
	rightLegDirection = Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
      } else {
	/* Right vertex non-convex; right leg extends cut-off line. */
	rightLegDirection = obstacle1->unitDir_;
      }
    }

    /*
     * Legs can never point into neighboring edge when convex vertex,
     * take cutoff-line of neighboring edge instead. If velocity projected on
     * "foreign" leg, no constraint is added.
     */

    const Obstacle* const leftNeighbor = obstacle1->prevObstacle;

    bool isLeftLegForeign = false;
    bool isRightLegForeign = false;

    if (obstacle1->isConvex_ && det(leftLegDirection, -leftNeighbor->unitDir_) >= 0.0f) {
      /* Left leg points into obstacle. */
      leftLegDirection = -leftNeighbor->unitDir_;
      isLeftLegForeign = true;
    }

    if (obstacle2->isConvex_ && det(rightLegDirection, obstacle2->unitDir_) <= 0.0f) {
      /* Right leg points into obstacle. */
      rightLegDirection = obstacle2->unitDir_;
      isRightLegForeign = true;
    }

    /* Compute cut-off centers. */
    const Vector2 leftCutoff = invTimeHorizonObst * (obstacle1->point_ - position_);
    const Vector2 rightCutoff = invTimeHorizonObst * (obstacle2->point_ - position_);
    const Vector2 cutoffVec = rightCutoff - leftCutoff;

    /* Project current velocity on velocity obstacle. */

    /* Check if current velocity is projected on cutoff circles. */
    const float t = (obstacle1 == obstacle2 ? 0.5f : ((velocity_ - leftCutoff) * cutoffVec) / absSq(cutoffVec));
    const float tLeft = ((velocity_ - leftCutoff) * leftLegDirection);
    const float tRight = ((velocity_ - rightCutoff) * rightLegDirection);

    if ((t < 0.0f && tLeft < 0.0f) || (obstacle1 == obstacle2 && tLeft < 0.0f && tRight < 0.0f)) {
      /* Project on left cut-off circle. */
      const Vector2 unitW = normalize(velocity_ - leftCutoff);

      line.direction = Vector2(unitW.y(), -unitW.x());
      line.point = leftCutoff + radius_ * invTimeHorizonObst * unitW;
      orcaLines_.push_back(line);
      continue;
    } else if (t > 1.0f && tRight < 0.0f) {
      /* Project on right cut-off circle. */
      const Vector2 unitW = normalize(velocity_ - rightCutoff);

      line.direction = Vector2(unitW.y(), -unitW.x());
      line.point = rightCutoff + radius_ * invTimeHorizonObst * unitW;
      orcaLines_.push_back(line);
      continue;
    }

    /*
     * Project on left leg, right leg, or cut-off line, whichever is closest
     * to velocity.
     */
    const float distSqCutoff = ((t < 0.0f || t > 1.0f || obstacle1 == obstacle2) ? std::numeric_limits<float>::infinity() : absSq(velocity_ - (leftCutoff + t * cutoffVec)));
    const float distSqLeft = ((tLeft < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(velocity_ - (leftCutoff + tLeft * leftLegDirection)));
    const float distSqRight = ((tRight < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(velocity_ - (rightCutoff + tRight * rightLegDirection)));

    if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
      /* Project on cut-off line. */
      line.direction = -obstacle1->unitDir_;
      line.point = leftCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
      orcaLines_.push_back(line);
      continue;
    } else if (distSqLeft <= distSqRight) {
      /* Project on left leg. */
      if (isLeftLegForeign) {
	continue;
      }

      line.direction = leftLegDirection;
      line.point = leftCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
      orcaLines_.push_back(line);
      continue;
    } else {
      /* Project on right leg. */
      if (isRightLegForeign) {
	continue;
      }

      line.direction = -rightLegDirection;
      line.point = rightCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
      orcaLines_.push_back(line);
      continue;
    }
  }
  // add additional orca lines
  for (size_t l = 0; l < this->additionalOrcaLines_.size(); l++) {
    Line line = this->additionalOrcaLines_[l];
    Vector2 point = Vector2(line.point.x() - this->position_.x(), line.point.y() - this->position_.y());
    Line new_line = Line(line);
    new_line.point = point;
    orcaLines_.push_back(new_line);
  }


  //  orcaLines_.insert(orcaLines_.end(), additional_orca_lines_.begin(), additional_orca_lines_.end());

  
  Line maxVel1;
  Line maxVel2;

  Vector2 dir =  Vector2(sin(this->heading_),-cos(this->heading_));
  maxVel1.point = this->maxTrackSpeed_ * Vector2(-dir.y(),dir.x());
  maxVel1.direction = dir;
  maxVel2.direction = dir;
  maxVel2.point = -this->maxTrackSpeed_ * Vector2(-dir.y(),dir.x());
  orcaLines_.push_back(maxVel1);
  orcaLines_.push_back(maxVel2);


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

	if (det(relativePosition, w) > -leftPref_) {
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
      //  const float invTimeStep = 1.0f / sim_->timeStep_; //TODO update with exact timestep
      const float invTimeStep = 1.0f/timeStep_;
      /* Vector from cutoff center to relative velocity. */
      const Vector2 w = relativeVelocity - invTimeStep * relativePosition;

      const float wLength = abs(w);
      const Vector2 unitW = w / wLength;

      line.direction = Vector2(unitW.y(), -unitW.x());
      u = (combinedRadius * invTimeStep - wLength) * unitW;
    }

    line.point = velocity_ + 0.5f * u;
    ros::NodeHandle nh;
    //ROS_ERROR("%s = line: (%f,%f), dir (%f,%f)",nh.getNamespace().c_str(),line.point.x(),line.point.y(),line.direction.x(),line.direction.y());
    orcaLines_.push_back(line);
  }

  size_t lineFail = linearProgram2(orcaLines_, maxSpeed_, prefVelocity_, false, newVelocity_);

  if (lineFail < orcaLines_.size()) {
    linearProgram3(orcaLines_, numObstLines, lineFail, maxSpeed_, newVelocity_);
  }
}

  
void ROSAgent::setTimeStep(float timeStep) {
  this->timeStep_ = timeStep;
}

void ROSAgent::setHeading(float heading) {
  this->heading_ = heading;
}

void ROSAgent::setMaxTrackSpeed(float maxTrackSpeed) {
  this->maxTrackSpeed_ = maxTrackSpeed;
}

void ROSAgent::setAdditionalOrcaLines(std::vector<RVO::Line> additionalOrcaLines){
  this->additionalOrcaLines_.clear();
  this->additionalOrcaLines_ = additionalOrcaLines;
}

void ROSAgent::setLeftPref(float leftPref) {
  this->leftPref_ = leftPref;
}


int main(){
}

