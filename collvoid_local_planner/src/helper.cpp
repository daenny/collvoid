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


#include <angles/angles.h>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>


#include <collvoid_local_planner/ROSAgent.h>

#include <collvoid_local_planner/orca.h>
#include <collvoid_local_planner/collvoid_publishers.h>


namespace collvoid {

    tf::TransformListener *tf_;
    std::string base_frame_, global_frame_, odom_frame_;
    std::vector <std::pair<double, geometry_msgs::PoseStamped>> pose_array_weighted_;
    double footprint_radius_, radius_, cur_loc_unc_radius_;
    std::vector <Vector2> minkowski_footprint_;
    geometry_msgs::PolygonStamped footprint_msg_;
    double eps_;

    void computeNewLocUncertainty();

    void computeNewMinkowskiFootprint();

    void amclPoseArrayWeightedCallback(const amcl::PoseArrayWeighted::ConstPtr &msg);

    double CalcArea(std::vector < Vector2 > list);


    bool compareConvexHullPointsPosition(const ConvexHullPoint &p1, const ConvexHullPoint &p2) {
        return collvoid::absSqr(p1.point) <= collvoid::absSqr(p2.point);
    }

    bool comparePoint(const Vector2 &p1, const Vector2 &p2) {
        return collvoid::absSqr(p1) <= collvoid::absSqr(p2);
    }


    void computeNewLocUncertainty() {
        std::vector <ConvexHullPoint> points;

        for (int i = 0; i < (int) pose_array_weighted_.size(); i++) {
            ConvexHullPoint p;
            p.point = Vector2(pose_array_weighted_[i].second.pose.position.x,
                              pose_array_weighted_[i].second.pose.position.y);
            p.weight = pose_array_weighted_[i].first;
            p.index = i;
            p.orig_index = i;
            points.push_back(p);
        }

        std::sort(points.begin(), points.end(), compareConvexHullPointsPosition);

        // ROS_ERROR("points in list = %d", (int)points.size());
        // double testSum = 0;
        // for (int i = 0; i<(int)points.size(); i++) {
        //   testSum += points[i].weight;
        //   //ROS_ERROR("%f, weight = %f", abs(points[i].point), points[i].weight);
        // }
        // ROS_ERROR("%f testSUm ", testSum);

        double sum = 0.0;
        int j = 0;
        while (sum <= 1.0 - eps_ && j < (int) points.size()) {
            sum += points[j].weight;
            j++;
        }
        // ROS_ERROR("CALU points %d", j);
        // for (int i = 0; i<j; i++) {
        //   ROS_ERROR("%f", abs(points[i].point));
        // }
        cur_loc_unc_radius_ = collvoid::abs(points[j - 1].point);
        //ROS_ERROR("Loc Uncertainty = %f", cur_loc_unc_radius_);
        //radius_ = footprint_radius_ + cur_loc_unc_radius_;
    }

    void computeNewMinkowskiFootprint() {
        bool done = false;
        std::vector <ConvexHullPoint> convex_hull;
        std::vector <ConvexHullPoint> points;
        points.clear();


        for (int i = 0; i < (int) pose_array_weighted_.size(); i++) {
            ConvexHullPoint p;
            p.point = Vector2(pose_array_weighted_[i].second.pose.position.x,
                              pose_array_weighted_[i].second.pose.position.y);
            p.weight = pose_array_weighted_[i].first;
            p.index = i;
            p.orig_index = i;
            points.push_back(p);
        }
        std::sort(points.begin(), points.end(), compareVectorsLexigraphically);
        for (int i = 0; i < (int) points.size(); i++) {
            points[i].index = i;
        }

        double total_sum = 0;
        std::vector <int> skip_list;

        while (!done && points.size() >= 3) {
            double sum = 0;
            convex_hull.clear();
            convex_hull = convexHull(points, true);

            skip_list.clear();
            for (int j = 0; j < (int) convex_hull.size() - 1; j++) {
                skip_list.push_back(convex_hull[j].index);
                sum += convex_hull[j].weight;
            }
            total_sum += sum;

            //ROS_WARN("SUM = %f (%f), num Particles = %d, eps = %f", sum, total_sum, (int) points.size(), eps_);

            if (total_sum >= eps_) {
                done = true;
                break;
            }

            std::sort(skip_list.begin(), skip_list.end());
            for (int i = (int) skip_list.size() - 1; i >= 0; i--) {
                points.erase(points.begin() + skip_list[i]);
            }

            for (int i = 0; i < (int) points.size(); i++) {
                points[i].index = i;
            }
        }

        std::vector <Vector2> localization_footprint, own_footprint;
        for (int i = 0; i < (int) convex_hull.size(); i++) {
            localization_footprint.push_back(convex_hull[i].point);
        }

        minkowski_footprint_ = localization_footprint;
        //    ROS_ERROR("COLLVOID points %d", (int) minkowski_footprint_.size());

        // for (int i = 0; i<(int)minkowski_footprint_.size(); i++) {
        //   ROS_ERROR("%f", abs(minkowski_footprint_[i]));
        // }

        // BOOST_FOREACH(geometry_msgs::Point32 p, footprint_msg_.polygon.points) {
        //   own_footprint.push_back(Vector2(p.x, p.y));
        //   //      ROS_WARN("footprint point p = (%f, %f) ", footprint_[i].x, footprint_[i].y);
        // }
        // minkowski_footprint_ = minkowskiSum(localization_footprint, own_footprint);

        // //publish footprint
        // geometry_msgs::PolygonStamped msg_pub = createFootprintMsgFromVector2(minkowski_footprint_);
        // polygon_pub_.publish(msg_pub);

    }


    void amclPoseArrayWeightedCallback(const amcl::PoseArrayWeighted::ConstPtr &msg) {
        pose_array_weighted_.clear();
        geometry_msgs::PoseStamped in;
        in.header = msg->header;
        for (int i = 0; i < (int) msg->poses.size(); i++) {
            in.pose = msg->poses[i];
            geometry_msgs::PoseStamped result;
            try {
                tf_->waitForTransform(global_frame_, base_frame_, msg->header.stamp, ros::Duration(0.3));
                tf_->transformPose(base_frame_, in, result);
                pose_array_weighted_.push_back(std::make_pair(msg->weights[i], result));
            }
            catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
                ROS_ERROR("point transform failed");
            };
        }
        //    if (!convex_ || orca_) {



        std::vector <double> circle, convex;

        for (eps_ = 0; eps_ <= 1; eps_ += 0.05) {
            computeNewLocUncertainty();
            computeNewMinkowskiFootprint();
            double area = CalcArea(minkowski_footprint_);
            //ROS_ERROR("eps = %f", eps_);
            //ROS_ERROR("radius = %f, area = %f", cur_loc_unc_radius_, M_PI*sqr(cur_loc_unc_radius_));
            circle.push_back(M_PI * sqr(cur_loc_unc_radius_));
            convex.push_back(area);
            //std::sort(minkowski_footprint_.begin(),minkowski_footprint_.end(), comparePoint);

            //ROS_ERROR("area conv_ = %f", area);
            //ROS_ERROR("furthest point= %f", collvoid::abs(minkowski_footprint_[int(minkowski_footprint_.size())-1]));
        }
        for (int i = 0; i < (int) circle.size(); i++) {
            std::cout << circle[i] << ", ";
        }
        for (int i = 0; i < (int) circle.size(); i++) {
            std::cout << convex[i] << ", ";
        }
        std::cout << ";" << std::endl;

    }


    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        //we assume that the odometry is published in the frame of the base
        tf::Stamped <tf::Pose> global_pose;
        //let's get the pose of the robot in the frame of the plan
        global_pose.setIdentity();
        global_pose.frame_id_ = base_frame_;
        global_pose.stamp_ = msg->header.stamp;

        try {
            tf_->waitForTransform(global_frame_, base_frame_, global_pose.stamp_, ros::Duration(0.2));
            tf_->transformPose(global_frame_, global_pose, global_pose);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            ROS_ERROR("point odom transform failed");
            return;
        };

        geometry_msgs::PoseStamped pose_msg;
        tf::poseStampedTFToMsg(global_pose, pose_msg);

        tf::Stamped <tf::Pose> odom_pose;
        //let's get the pose of the robot in the frame of the plan
        odom_pose.setIdentity();
        odom_pose.frame_id_ = base_frame_;
        odom_pose.stamp_ = msg->header.stamp;

        try {
            tf_->waitForTransform(odom_frame_, base_frame_, odom_pose.stamp_, ros::Duration(0.2));
            tf_->transformPose(odom_frame_, odom_pose, odom_pose);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            ROS_ERROR("point odom transform failed");
            return;
        };

        geometry_msgs::PoseStamped odom_pose_msg;
        tf::poseStampedTFToMsg(odom_pose, odom_pose_msg);

        //std::cout << "[" <<pose_msg.pose.position.x << ", " << pose_msg.pose.position.y <<"]" << std::endl;
        //std::cout <<"[" <<odom_pose_msg.pose.position.x << ", " << odom_pose_msg.pose.position.y <<"]" << std::endl;
    }


    double CalcArea(std::vector < Vector2 > list) {
        double area = 0; // Total area
        double diff = 0; // Difference Of Y{i + 1} - Y{i - 1}
        unsigned int last = list.size() - 1; // Size Of Vector - 1
        /* Given vertices from 1 to n, we first loop through
               the vertices 2 to n - 1. We will take into account
           vertex 1 and vertex n sepereately */
        for (unsigned int i = 1; i < last; i++) {
            diff = list[i + 1].y() - list[i - 1].y();
            area += list[i].x() * diff;
        }

        /* Now We Consider The Vertex 1 And The Vertex N */
        diff = list[1].y() - list[last].y();
        area += list[0].x() * diff; // Vertex 1
        diff = list[0].y() - list[last - 1].y();
        area += list[last].x() * diff; // Vertex N
        /* Calculate The Final Answer */
        area = 0.5 * std::abs(area);
        return area; // Return The area}
    }


}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Helper");
    ros::NodeHandle nh;
    tf::TransformListener tf;
    collvoid::pose_array_weighted_.clear();
    collvoid::odom_frame_ = "odom";
    collvoid::global_frame_ = "/map";
    collvoid::base_frame_ = "base_link";

    collvoid::tf_ = &tf;
    message_filters::Subscriber <amcl::PoseArrayWeighted> amcl_posearray_sub;
    amcl_posearray_sub.subscribe(nh, "particlecloud_weighted", 10);
    tf::MessageFilter <amcl::PoseArrayWeighted> *tf_filter;

    tf_filter = new tf::MessageFilter<amcl::PoseArrayWeighted>(amcl_posearray_sub, *collvoid::tf_,
                                                               collvoid::global_frame_, 10);
    tf_filter->registerCallback(boost::bind(&collvoid::amclPoseArrayWeightedCallback, _1));
    //ros::Subscriber odom_sub = nh.subscribe("odom", 1, collvoid::odomCallback);

    collvoid::eps_ = 0.1;
    ros::spin();

    delete tf_filter;
}

