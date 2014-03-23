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

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(ackermann_local_planner::AckermannPlannerROS, nav_core::BaseLocalPlanner)

namespace ackermann_local_planner {

  void AckermannPlannerROS::reconfigureCB(AckermannPlannerConfig &config, uint32_t level) {
      max_vel_ = config.max_vel;
      min_vel_ = config.min_vel;
      min_radius_ = config.min_radius;
      acc_lim_ = config.acc_lim;

      forward_point_distance_ = config.forward_point_distance;

      // TODO(hendrix): these may be obsolete
      //vx_samples = config.vx_samples;
      //radius_samples = config.radius_samples;

      xy_goal_tolerance_ = config.xy_goal_tolerance;
      yaw_goal_tolerance_ = config.yaw_goal_tolerance;

      move_ = config.move;
  }

  AckermannPlannerROS::AckermannPlannerROS() : initialized_(false),
      odom_helper_("odom") {

  }

  void AckermannPlannerROS::initialize(
      std::string name,
      tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros) {
    if (! isInitialized()) {

      ros::NodeHandle private_nh("~/" + name);
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
      tf_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ros_->getRobotPose(current_pose_);

      // make sure to update the costmap we'll use for this cycle
      costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

      planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

      if( private_nh.getParam( "odom_topic", odom_topic_ ))
      {
        odom_helper_.setOdomTopic( odom_topic_ );
      }
      
      initialized_ = true;

      dsrv_ = new dynamic_reconfigure::Server<AckermannPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<AckermannPlannerConfig>::CallbackType cb = boost::bind(&AckermannPlannerROS::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);
    }
    else{
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }
  
  bool AckermannPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    ROS_INFO("Got new plan");
    return false;
  }

  bool AckermannPlannerROS::isGoalReached() {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    // TODO(hendrix):
    //  probably use some sort of goal tolerance parameters here
    return false;
  }

  void AckermannPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, l_plan_pub_);
  }


  void AckermannPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, g_plan_pub_);
  }

  AckermannPlannerROS::~AckermannPlannerROS(){
    //make sure to clean things up
    delete dsrv_;
  }


  bool AckermannPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    // TODO(hendrix)
    //  do some real plannin here
    // Ideas and questions:
    //  - when do we give up and ask that the global planner re-plan?
    //  - Given the global plan, pick a point(s) somewhere forward along the
    //    path, and compute the dubins/dubins++ path to reach them
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
    return false;
  }
};
