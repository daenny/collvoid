/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2014  Samir Benmendil <samir.benmendil@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef GOALALIGNMENTCOSTFUNCTION_H
#define GOALALIGNMENTCOSTFUNCTION_H

#include <base_local_planner/trajectory_cost_function.h>
#include <geometry_msgs/PoseStamped.h>

namespace collvoid_dwa_local_planner
{

using namespace base_local_planner;

class GoalAlignmentCostFunction : public TrajectoryCostFunction
{
public:
    GoalAlignmentCostFunction() {}
    ~GoalAlignmentCostFunction() {}

    bool prepare() {
        return true;
    }
    double scoreTrajectory(Trajectory &traj);
    void setGoalPose(const geometry_msgs::PoseStamped &goalPose) { goalPose_ = goalPose; }

private:
    geometry_msgs::PoseStamped goalPose_;
};
}

#endif // GOALALIGNMENTCOSTFUNCTION_H
