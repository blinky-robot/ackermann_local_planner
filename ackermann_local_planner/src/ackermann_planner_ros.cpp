/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Austin Hendrix
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Austin Hendrix
*********************************************************************/

#include <ackermann_local_planner/ackermann_planner_ros.h>
#include <cmath>
#include <limits>
#include <boost/foreach.hpp>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <angles/angles.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(ackermann_local_planner::AckermannPlannerROS, nav_core::BaseLocalPlanner)

namespace ackermann_local_planner {

  bool isForwards(const geometry_msgs::PoseStamped &start,
      const geometry_msgs::PoseStamped &end) {
    double yaw1 = tf::getYaw(start.pose.orientation);
    //double yaw2 = tf::getYaw(end.pose.orientation);
    //double angle = angles::shortest_angular_distance(yaw1, yaw2);

    double dx = (end.pose.position.x - start.pose.position.x);
    double dy = (end.pose.position.y - start.pose.position.y);
    double diff_angle = atan2(dy, dx);

    if( abs(angles::shortest_angular_distance(yaw1, diff_angle)) < M_PI/2 ) {
      return true;
    } else {
      return false;
    }
  }

  inline double sq(double x) {
    return x*x;
  }

  double dist(const geometry_msgs::PoseStamped &start,
      const geometry_msgs::PoseStamped &end) {
    return hypot((end.pose.position.x - start.pose.position.x),
                 (end.pose.position.y - start.pose.position.y));
  }

  double curvature(const geometry_msgs::PoseStamped & start,
                   const geometry_msgs::PoseStamped & end ) {
    double dx = (end.pose.position.x - start.pose.position.x);
    double dy = (end.pose.position.y - start.pose.position.y);
    double yaw1 = tf::getYaw(start.pose.orientation);
    double yaw2 = tf::getYaw(end.pose.orientation);
    double dtheta = fabs(angles::shortest_angular_distance(yaw1, yaw2));
    double ds = hypot(dx, dy);
    return dtheta / ds;
  }

  void AckermannPlannerROS::reconfigureCB(AckermannPlannerConfig &config, uint32_t level) {
      max_vel_ = config.max_vel;
      min_vel_ = config.min_vel;
      min_radius_ = config.min_radius;
      acc_lim_ = config.acc_lim;

      lookahead_factor_ = config.lookahead_factor;

      // TODO(hendrix): these may be obsolete
      //vx_samples = config.vx_samples;
      radius_samples_ = config.radius_samples;

      xy_goal_tolerance_ = config.xy_goal_tolerance;
      yaw_goal_tolerance_ = config.yaw_goal_tolerance;

      move_ = config.move;
  }

  AckermannPlannerROS::AckermannPlannerROS() : initialized_(false),
    have_particlecloud_(false), have_pose_with_cow_(false),
    goal_reached_(false) {

  }

  void AckermannPlannerROS::initialize(
      std::string name,
      tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros) {
    if (! isInitialized()) {

      ros::NodeHandle private_nh("~/" + name);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      tf_ = tf;
      costmap_ros_ = costmap_ros;

      // make sure to update the costmap we'll use for this cycle
      costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

      std::string odom_topic;
      private_nh.param<std::string>("odom_topic", odom_topic, "odom");
      odom_helper_.setOdomTopic( odom_topic );

      private_nh.param<bool>("publish_goal", publish_goal_, false);
      goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("goal", 1);
      private_nh.param<bool>("publish_near_point", publish_near_point_, false);
      near_point_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>(
          "near_point", 1);
      
      initialized_ = true;

      dsrv_ = new dynamic_reconfigure::Server<AckermannPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<AckermannPlannerConfig>::CallbackType cb = boost::bind(&AckermannPlannerROS::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);
    }
    else{
      ROS_WARN_NAMED("ackermann_planner", "This planner has already been "
          "initialized, doing nothing.");
    }
  }

  int AckermannPlannerROS::nearestPoint(const int start_point,
      const tf::Stamped<tf::Pose> & pose) const {
    int plan_point = start_point;
    double best_metric = std::numeric_limits<double>::max();
    for( int i=start_point; i<plan_.size(); i++ ) {
      double dist = base_local_planner::getGoalPositionDistance(pose,
          plan_[i].pose.position.x, plan_[i].pose.position.y);
      double theta = fabs(base_local_planner::getGoalOrientationAngleDifference(
          pose, tf::getYaw(plan_[i].pose.orientation)));
      double metric = dist + theta / 10.0;
      //ROS_INFO_NAMED("ackermann_planner", "Distance to path: %f", dist);
      if( metric < best_metric ) {
        best_metric = metric;
        plan_point = i;
      }
    }
    return plan_point;
  }

  double AckermannPlannerROS::scoreTrajectory(
      const std::vector<dubins_plus::Segment> &path,
      double global_length, double global_dtheta) const {
    // score and choose a best plan
    // possible scoring parameters:
    //  - curvature. lower is better
    //  - distance/match to global plan
    //    - include x/y and angular distance
    //  - length of path compared to length of global plan
    double dtheta = 0;
    double local_length = 0;
    BOOST_FOREACH(dubins_plus::Segment s, path) {
      local_length += s.getLength();
      dtheta += std::abs(s.getCurvature() * s.getLength());
    }
    // normalized to a base of 1.0. Values > 1.0 are worse
    //  don't count paths shorter than the global path as better
    double length_cost = std::max(local_length/global_length, 1.0);

    double curve_cost;
    curve_cost = std::abs(dtheta - global_dtheta)/(M_PI/180.0);

    //ROS_INFO_NAMED("ackermann_planner", "Length cost: %f, curve cost: %f",
    //    length_cost, curve_cost);

    //return length_cost + curve_cost;
    return length_cost;
  }
  
  bool AckermannPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    // Transform global plan into local coordinate space
    /*
    tf::Stamped<tf::Pose> pose;
    costmap_ros_->getRobotPose(pose);
    ROS_INFO_NAMED("ackermann_planner", "Transforming global plan into %s frame", costmap_ros_->getGlobalFrameID().c_str());
    base_local_planner::transformGlobalPlan(*tf_, orig_global_plan, pose,
        *costmap_ros_->getCostmap(), costmap_ros_->getGlobalFrameID(), plan_);
        */
    //if( plan_ == orig_global_plan ) {
    if( false ) {
      ROS_WARN("Got the same plan again");
    } else {
      ROS_INFO("Got new plan");
      plan_ = orig_global_plan;
    }

    goal_reached_ = false;
    last_plan_point_ = 0; // we're at the beginning of the plan
    // return false here if we would like the global planner to re-plan
    return true; // TODO: figure out what the return value here means
  }

  bool AckermannPlannerROS::isGoalReached() {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    // TODO(hendrix):
    //  probably use some sort of goal tolerance parameters here
    return goal_reached_;
  }

  void AckermannPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, l_plan_pub_);
  }


  AckermannPlannerROS::~AckermannPlannerROS(){
    //make sure to clean things up
    delete dsrv_;
  }


  bool AckermannPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    // TODO(hendrix)
    //  do some real plannin' here
    // Ideas and questions:
    //  - when do we give up and ask that the global planner re-plan?
    //  - Given the global plan, pick a point(s) somewhere forward along the
    //    path, and compute the dubins/dubins++/dubins-- path to reach them
    //    - repeat across various radii
    //    - repeat for various points across the possible locations of the robot
    //      - this is where we hook into AMCL's error estimate of our position
    //    - make sure that the point we pick is strictly forward or backward
    //      from the robot's current position
    //    - publish all of the possible paths computed, for debugging
    //  - given all of the possible computed paths, extract the first step,
    //    and look for general concensus among the possibilities, ie:
    //    - mostly forward, or mostly left, or mostly reverse+left
    //    - if there is not concensus, bail and let the user figure it out.
    //      come back to this later when it comes up
    //  - publish some basic statistics over diagnostics
    //    - average path computation time
    //    - planning frequency
    //    - staticstics about how frequently we use escape behaviors
    //    - average velocity
    //    - % of time/commands spent backing up
    //  - absolutely do not publish a command that reverses direction if we
    //    not completely stopped
    //  - have a config switch that turns off the command output from the
    //    planner. default it to ON

    // if we don't have a plan, what are we doing here???
    if( plan_.size() < 2 ) {
      ROS_WARN_NAMED("ackermann_planner", "Got empty plan! Goal reached?");
      goal_reached_ = true;

      // stop moving
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
      return true;
    }

    nav_msgs::Odometry odom;
    odom_helper_.getOdom(odom);
    double linear_vel = odom.twist.twist.linear.x;
    double angular_vel = odom.twist.twist.angular.z;

    // if we have a pose cloud, get it, otherwise just use our current pose
    tf::Stamped<tf::Pose> current_pose;
    if( have_particlecloud_ ) {
      // TODO(hendrix)
      //current_poses = particlecloud_;
      ROS_INFO_NAMED("ackermann_planner", "Got position from ParticleCloud");
    } else if( have_pose_with_cow_ ) {
      // TODO(hendrix)
      ROS_INFO_NAMED("ackermann_planner", "Got position from PoseWithCov");
    } else {
      costmap_ros_->getRobotPose(current_pose);
      ROS_INFO_NAMED("ackermann_planner", "Got position from costmap");
    }
    ROS_INFO_NAMED("ackermann_planner", "Starting point (%f, %f)",
        current_pose.getOrigin().x(), current_pose.getOrigin().y());

    // get the nearest point on the global plan; both in angle space and
    // linear space
    int plan_point = nearestPoint(last_plan_point_, current_pose);

    if( abs(plan_point - last_plan_point_) > 20 ) {
      ROS_WARN_NAMED("ackermann_planner", "Whoa! We moved a lot. Not sure we're still on the right part of the plan");
    }

    last_plan_point_ = plan_point;
    geometry_msgs::PoseStamped plan_pose = plan_[plan_point];

    // publish plan_point as "here"
    if( publish_near_point_ ) {
      near_point_pub_.publish(plan_pose);
    }

    if( plan_point < plan_.size() - 1 ) {
      int i = plan_point + 1;
      geometry_msgs::PoseStamped next_pose = plan_[i];
      // get the direction (forward/backwards) on the plan
      bool forward = isForwards(plan_pose, next_pose);

      // Pure pursuit algorithm (Coulter R. Craig, 1992)
      // compute the curvature at the current point
      double local_curvature = curvature(plan_pose, next_pose);
      if( plan_point > 0 ) {
        const geometry_msgs::PoseStamped & prev_pose = plan_[plan_point-1];
        // average curvature to previous point with curvature to next point
        local_curvature = (local_curvature + curvature(prev_pose, plan_pose))/2;
      }
      // Pure pursuit lookahead
      // r = 1 / curvature
      // lookahend = factor * r
      //           = factor / curvature
      double local_radius = 1 / local_curvature;
      double forward_point_distance = lookahead_factor_ * local_radius;
      ROS_INFO_NAMED("ackermann_planner", "Local curvature %f, radius %f"
          ", lookahead distance %f", local_curvature, local_radius,
          forward_point_distance);
      // get a point forward of where we are on the plan
      double forward_dist = 0;
      double dtheta = 0;

      while( forward_dist < lookahead_factor_ / local_curvature &&
          i < plan_.size() &&
          isForwards(plan_pose, next_pose) == forward ) {
        plan_pose = plan_[i-1];
        next_pose = plan_[i];
        forward_dist += dist(plan_pose, next_pose);
        dtheta += std::abs(angles::shortest_angular_distance(
              tf::getYaw(next_pose.pose.orientation),
              tf::getYaw(plan_pose.pose.orientation)));

        double c = curvature(plan_pose, next_pose);
        if( c > local_curvature ) {
          local_curvature = c;
          local_radius = 1 / local_curvature;
          forward_point_distance = lookahead_factor_ * local_radius;
          ROS_INFO_NAMED("ackermann_planner", "Updated curvature %f, "
              "radius %f and lookahead distance: %f", local_curvature,
              local_radius, forward_point_distance);
        }

        i++;
      }

      ROS_INFO_NAMED("ackermann_planner", "Target pose #%d is %f meters away",
          i, forward_dist);

      geometry_msgs::PoseStamped goal_pose = next_pose;

      // publish goal pose
      if( publish_goal_ ) {
        goal_pub_.publish(goal_pose);
      }

      // TODO(hendrix): for each potential position
      // for each starting position
      //   for each radius/curvature
      //     generate a path from start to end
      //    choose the best path for this start point
      //    - possibly publish these
      // analyze the set of best paths for each starting point
      // - look for general patterns/concensus
      // - possibly publish all plans considered

      // Compute Dubins path to the goal
      geometry_msgs::Pose current_pose_msg;
      tf::poseTFToMsg(current_pose, current_pose_msg);

      // if the path is backwards, invert the direction of initial and final
      // poses
      if( ! forward ) {
        double start_yaw = tf::getYaw(current_pose_msg.orientation);
        start_yaw += M_PI;
        current_pose_msg.orientation = tf::createQuaternionMsgFromYaw(start_yaw);

        double end_yaw = tf::getYaw(goal_pose.pose.orientation);
        end_yaw += M_PI;
        goal_pose.pose.orientation = tf::createQuaternionMsgFromYaw(end_yaw);
      }

      // path sampling
      std::vector<dubins_plus::Segment> local_path;
      double best_score = std::numeric_limits<double>::max();
      double max_curvature = 1/min_radius_;
      ROS_INFO_NAMED("ackermann_planner", "Maximum curvature: %f", max_curvature);

      // sample across curvature
      for( int i=0; i<radius_samples_; i++ ) {
        double curvature = (max_curvature/radius_samples_) * (i+1);
        ROS_INFO_NAMED("ackermann_planner", "Considering curvature: %f", curvature);
        double radius = 1/curvature;
        std::vector<dubins_plus::Segment> path(dubins_plus::dubins_path(radius,
              current_pose_msg, goal_pose.pose));
        double score = scoreTrajectory(path, forward_dist, dtheta);
        if( score < best_score ) {
          best_score = score;
          local_path = path;
        }
      }
      

      ROS_INFO_NAMED("ackermann_planner", "Best path cost %f", best_score);

      // publish goal pose
      if( publish_goal_ ) {
        goal_pub_.publish(goal_pose);
      }


      std::vector<geometry_msgs::PoseStamped> local_plan;
      double x = current_pose_msg.position.x;
      double y = current_pose_msg.position.y;
      double theta = tf::getYaw(current_pose_msg.orientation);

      for( int i=0; i<local_path.size(); i++ ) {
        /*
        ROS_INFO_NAMED("ackermann_planner",
            "Dubins path length %f, curvature %f",
            local_path[i].getLength(), local_path[i].getCurvature());
            */
        double length = local_path[i].getLength();
        double curvature = local_path[i].getCurvature();
        double l = 0;
        static const double dl = 0.01;
        while( l < length ) {
          geometry_msgs::PoseStamped pose;
          pose.header.frame_id = costmap_ros_->getGlobalFrameID();
          pose.pose.position.x = x;
          pose.pose.position.y = y;
          pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
          local_plan.push_back(pose);

          x += dl * cos(theta);
          y += dl * sin(theta);
          theta += curvature * dl;
          l += dl;
        }
      }

      publishLocalPlan(local_plan);

      if( best_score > 10.0 ) {
        // if the best local trajectory we were able to find is 5x worse than
        // the global path, force the global planner to re-plan
        ROS_ERROR_NAMED("ackermann_planner", "Failed to find a good local plan");
        return false;
      }

      i=0;
      for( ; i<local_path.size() && local_path[i].getLength() < 0.01; i++ );
      double target_curvature = local_path[i].getCurvature();

      // target maximum velocity
      double target_speed = max_vel_;
      // limit to maximum acceleration
      target_speed = std::min(target_speed, linear_vel + acc_lim_);
      // decelerate to min_vel_ at end of path (using acc_lim_)
      //  we have forward_dist meters remaining
      // required_decel = (target_speed - min_vel_)
      // decel_time = required_decel / acc_lim_
      // x = a*t^2 + v0*t + x0
      // decel_distance = acc_lim_ * decel_time * decel_time + 
      //                  linear_vel * decel_time + 0
      double required_decel = target_speed - min_vel_;
      double decel_time = required_decel / acc_lim_;
      double decel_distance = acc_lim_ * decel_time * decel_time +
                              target_speed * decel_time +
                              0;
      // if we have less than decel_distance to the goal, we should be
      // decelerating
      // TODO: this oscillates. makes me sad.
      if( decel_distance * 1.1 >= forward_dist ) {
        target_speed = target_speed - acc_lim_;
      }
      // limit to minimum speed
      target_speed = std::max(target_speed, min_vel_);

      double target_angular = target_curvature * target_speed;

      if( ! forward ) {
        target_speed = -target_speed;
        //target_angular = -target_angular;
      }

      cmd_vel.linear.x = target_speed;
      cmd_vel.angular.z = target_angular;

    } else {
      // plan_point is the last point on the plan
      // we're here?
      //
      // ????
      ROS_INFO_NAMED("ackermann_planner", "plan_point is the last point on the"
          " plan. I guess we're here?");
      ROS_INFO_NAMED("ackermann_planner", "At point %d; %zd points in plan",
          plan_point, plan_.size());
      goal_reached_ = true;
    }


    if( !move_ ) {
      // if we're not supposed to be moving, zero out our command
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
    }

    // return true if we were able to find a path, false otherwise
    return true;
  }
};
