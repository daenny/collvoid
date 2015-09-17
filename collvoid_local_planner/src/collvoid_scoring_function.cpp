//
// Created by danielclaes on 22/06/15.
//

#include "collvoid_local_planner/collvoid_scoring_function.h"



namespace collvoid_scoring_function
{

    void CollvoidScoringFunction::init(ros::NodeHandle nh) {
        get_me_srv_ = nh.serviceClient<collvoid_srvs::GetMe>("get_me");
        get_neighbors_srv_ = nh.serviceClient<collvoid_srvs::GetNeighbors>("get_neighbors");
        neighbors_pub_ = nh.advertise<visualization_msgs::MarkerArray>("neighbors", 1);
        vo_pub_ = nh.advertise<visualization_msgs::Marker>("vo", 1);

        use_truncation_ = true;
        trunc_time_ = 10;
        convex_ = true;
        ROS_INFO("Collvoid Scoring init done!");
        //holo_robot_ = false;
    }

    bool CollvoidScoringFunction::getMe() {
        collvoid_srvs::GetMe srv;
        if (get_me_srv_.call(srv)) {
            me_ = createAgentFromMsg(srv.response.me);
            me_->use_truncation_ = use_truncation_;
            me_->trunc_time_ = trunc_time_;
            me_->convex_ = convex_;
            me_->type_vo_ = HRVOS;

            //ROS_INFO("GOT ME");
            return true;
        }
        else {
            ROS_INFO("Collvoid Scoring: Could not get me");
            return false;
        }
    }

    bool CollvoidScoringFunction::getNeighbors() {
        collvoid_srvs::GetNeighbors srv;
        if (get_neighbors_srv_.call(srv)) {
            BOOST_FOREACH(collvoid_msgs::PoseTwistWithCovariance msg, srv.response.neighbors) {
                            me_->agent_neighbors_.push_back(createAgentFromMsg(msg));
                        }

            std::sort(me_->agent_neighbors_.begin(), me_->agent_neighbors_.end(),
                      boost::bind(&CollvoidScoringFunction::compareNeighborsPositions, this, _1, _2));
            publishNeighborPositions(me_->agent_neighbors_, "/map", "/map", neighbors_pub_);
            return true;
        }
        else {
            ROS_INFO("Collvoid Scoring: Could not get nieghbors");

            return false;
        }
    }

    bool CollvoidScoringFunction::compareNeighborsPositions(const AgentPtr &agent1, const AgentPtr &agent2) {
        return compareVectorPosition(agent1->position_, agent2->position_);
    }

    bool CollvoidScoringFunction::compareVectorPosition(const collvoid::Vector2 &v1, const collvoid::Vector2 &v2) {
        return collvoid::absSqr(me_->position_ - v1) <= collvoid::absSqr(me_->position_ - v2);
    }


    AgentPtr CollvoidScoringFunction::createAgentFromMsg(collvoid_msgs::PoseTwistWithCovariance &msg) {
        AgentPtr agent = AgentPtr(new Agent());
        agent->radius_ = msg.radius;
        agent->controlled_ = msg.controlled;
        agent->position_ = Vector2(msg.pose.pose.position.x, msg.pose.pose.position.y);
        agent->heading_ = tf::getYaw(msg.pose.pose.orientation);

        std::vector<Vector2> minkowski_footprint;
        BOOST_FOREACH(geometry_msgs::Point32 p, msg.footprint.polygon.points) {
                        minkowski_footprint.push_back(Vector2(p.x, p.y));
                    }
        agent->footprint_ = rotateFootprint(minkowski_footprint, agent->heading_);

        if (msg.holo_robot) {
            agent->velocity_ = rotateVectorByAngle(msg.twist.twist.linear.x,
                                                   msg.twist.twist.linear.y, agent->heading_);
        }
        else {
            double dif_x, dif_y, dif_ang, time_dif;
            time_dif = 0.1;
            dif_ang = time_dif * msg.twist.twist.angular.z;
            dif_x = msg.twist.twist.linear.x * cos(dif_ang / 2.0);
            dif_y = msg.twist.twist.linear.x * sin(dif_ang / 2.0);
            agent->velocity_ = rotateVectorByAngle(dif_x, dif_y, agent->heading_);
        }


        return agent;
    }

    bool CollvoidScoringFunction::prepare(){
        // Get me
        if (!getMe()) {
            return false;
        }

        // Get neighbors
        if (!getNeighbors()) {
            return false;
        }
        // Calculate VOs
        me_->computeAgentVOs();

        publishVOs(me_->position_, me_->all_vos_, use_truncation_,"/map", "/map", vo_pub_);

        // Add constraints - Not necessary due to sampling?
        return true;
    }



    double CollvoidScoringFunction::scoreTrajectory(Trajectory &traj)
    {
        if (traj.getPointsSize() < 1) return 0;

        //double goal_heading = tf::getYaw(goal_pose_.pose.orientation);

        // TODO: check if goalHeading and endPoint are in the same reference frame
        double x, y, th;
        traj.getEndpoint(x, y, th);
        //ROS_INFO("END POINT %f, %f", x,y);
        double vel_x, vel_y, vel_theta;
        vel_x = traj.xv_;
        vel_y = traj.yv_;
        vel_theta = traj.thetav_;

        Vector2 test_vel = Vector2();
        if (fabs(vel_y) == 0.) {
            double dif_x, dif_y, dif_ang, time_dif;
            time_dif = 1.;//traj.time_delta_;
            dif_ang = time_dif * vel_theta;
            dif_x = vel_x * cos(dif_ang / 2.0);
            dif_y = vel_x * sin(dif_ang / 2.0);
            test_vel = rotateVectorByAngle(dif_x, dif_y, me_->heading_);
        }
        else {
            test_vel = rotateVectorByAngle(vel_x, vel_y, me_->heading_);
        }

        //test_vel = Vector2(x,y);

        double cost = calculateVelCosts(test_vel, me_->all_vos_, me_->use_truncation_);
        if (cost > 0) {
            return -1;
        }
        cost = std::max(1.-minDistToVOs(me_->all_vos_, test_vel, use_truncation_), 0.);

        //ROS_INFO("Collvoid Scoring costs: %f for vector %f, %f, speed %f, ang %f time dif %f", cost, test_vel.x(), test_vel.y(), vel_x, vel_theta, traj.time_delta_);

        //traj.x

        return cost;
    }
}