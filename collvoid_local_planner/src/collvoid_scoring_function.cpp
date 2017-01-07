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
        samples_pub_ = nh.advertise<visualization_msgs::MarkerArray>("samples", 1);
        vo_pub_ = nh.advertise<visualization_msgs::Marker>("vo", 1);

        ros::NodeHandle co_nh("~/CollvoidScoring");
        use_truncation_ = co_nh.param("use_truncation", true);
        trunc_time_ = co_nh.param("trunctime", 8.);
        use_polygon_footprint_ = co_nh.param("use_polygon_footprint", true);
        max_dist_vo_ = co_nh.param("max_dist_vo", 0.1);
        points.clear();
        ROS_INFO("Collvoid Scoring init done! Trunctime %f, max_dist_vo %f", trunc_time_, max_dist_vo_);
        //holo_robot_ = false;
    }

    bool CollvoidScoringFunction::getMe() {
        collvoid_srvs::GetMe srv;
        if (get_me_srv_.call(srv)) {
            me_ = createAgentFromMsg(srv.response.me);
            me_->use_truncation_ = use_truncation_;
            me_->trunc_time_ = trunc_time_;
            me_->use_polygon_footprint_ = use_polygon_footprint_;
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
            for(collvoid_msgs::PoseTwistWithCovariance msg: srv.response.neighbors) {
                            me_->agent_neighbors_.push_back(createAgentFromMsg(msg));
                        }

            std::sort(me_->agent_neighbors_.begin(), me_->agent_neighbors_.end(),
                      boost::bind(&CollvoidScoringFunction::compareNeighborsPositions, this, _1, _2));
            collvoid::publishNeighborPositionsBare(me_->agent_neighbors_, "/map", "/map", neighbors_pub_);
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
        for(geometry_msgs::Point32 p: msg.footprint.polygon.points) {
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

    bool CollvoidScoringFunction::prepare() {
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

        collvoid::publishVOs(me_->position_, me_->all_vos_, use_truncation_, "/map", "/map", vo_pub_);
        collvoid::publishPoints(me_->position_, points, "/map", "/map", samples_pub_);
        //for (size_t i = 0; i < me_->all_vos_.size(); ++i) {
        //    VO v = me_->all_vos_.at(i);
        //    ROS_INFO("Origin %f %f", v.point.x(), v.point.y()) ;
        // }

        points.clear();

        // Add constraints - Not necessary due to sampling?
        return true;
    }



    double CollvoidScoringFunction::scoreTrajectory(Trajectory &traj)
    {
        if (traj.getPointsSize() < 1) return 0;


        // TODO: check if goalHeading and endPoint are in the same reference frame
        double x, y, th;
        double x_s, y_s, th_s;
        traj.getEndpoint(x, y, th);
        traj.getPoint(0, x_s, y_s, th_s);


       //ROS_INFO("start orientation / end %f, %f", th_s,th);

        double time_diff = (int)traj.getPointsSize() * traj.time_delta_;
        double vel_x, vel_y, vel_theta;
        //vel_x = traj.xv_;
        //vel_y = traj.yv_;
        //vel_theta = traj.thetav_;
        vel_x = x - x_s;
        vel_y = y - y_s;
        vel_theta = th - th_s;

        Vector2 test_vel = Vector2();
        /*if (fabs(vel_y) == 0.) {
            double dif_x, dif_y, dif_ang, time_dif;
            time_dif = 5 * traj.time_delta_;
            dif_ang = 2 * time_dif * vel_theta;
            dif_x = time_dif * vel_x * cos(dif_ang / 2.0);
            dif_y = time_dif * vel_x * sin(dif_ang / 2.0);
            test_vel = rotateVectorByAngle(dif_x, dif_y, me_->heading_);
        }
        else {
            test_vel = rotateVectorByAngle(vel_x, vel_y, me_->heading_);
        }*/
        test_vel = rotateVectorByAngle(vel_x, vel_y, -th_s + me_->heading_ + vel_theta/3.)/time_diff;
        //test_vel = Vector2(vel_x,vel_y);

        double cost = calculateVelCosts(test_vel, me_->agent_vos_, me_->use_truncation_);

        if (cost > 0.) {
            int n = (int)me_->agent_vos_.size();
            if (cost >= n * 2)
                return -1;
            cost = 1 + (n * (n + 1) /2.) * 2./cost;
        }

        else {
            cost = 0.4 * std::max(
                    (max_dist_vo_ - sqrt(std::max(minDistToVOs(me_->agent_vos_, test_vel, use_truncation_, true), 0.))) /
                    max_dist_vo_, 0.);

            cost += 0.5 * std::max(( max_dist_vo_ - sqrt(std::max(minDistToVOs(me_->human_vos_, test_vel, use_truncation_, true), 0.)))/max_dist_vo_, 0.);
            cost += 0.3* std::max(( max_dist_vo_ - sqrt(std::max(minDistToVOs(me_->static_vos_, test_vel, use_truncation_, true), 0.)))/max_dist_vo_, 0.);
        }
        VelocitySample v;
        v.velocity = test_vel;
        v.cost = cost;
        points.push_back(v);

        //ROS_INFO("Collvoid Scoring costs: %f for vector %f, %f, speed %f, ang %f time dif %f", cost, test_vel.x(), test_vel.y(), vel_x, vel_theta, traj.time_delta_);

        //traj.x

        return cost;
    }
}