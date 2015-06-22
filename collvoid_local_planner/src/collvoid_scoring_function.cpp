//
// Created by danielclaes on 22/06/15.
//

#include "collvoid_local_planner/collvoid_scoring_function.h"
#include <tf/tf.h>


namespace collvoid_dwa_local_planner
{

    bool CollvoidScoringFunction::prepare(){
        // Get neighbors

        // Calculate VOs

        // Add constraints

        //

        return true;
    }

    double CollvoidScoringFunction::scoreTrajectory(Trajectory &traj)
    {
        if (traj.getPointsSize() < 1) return 0;

        double goalHeading = tf::getYaw(goalPose_.pose.orientation);

        // TODO: check if goalHeading and endPoint are in the same reference frame
        double x, y, th;
        traj.getEndpoint(x, y, th);
        double vel_x, vel_y, vel_theta;
        vel_x = traj.xv_;
        vel_y = traj.yv_;
        vel_theta = traj.thetav_;


        //traj.x

        double delta_th = fabs(th - goalHeading);


        return delta_th * getScale();
    }
}