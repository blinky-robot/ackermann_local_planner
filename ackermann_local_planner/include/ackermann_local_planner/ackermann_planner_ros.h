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
#ifndef ACKERMANN_LOCAL_PLANNER_ACKERMANN_PLANNER_ROS_H_
#define ACKERMANN_LOCAL_PLANNER_ACKERMANN_PLANNER_ROS_H_

// SHUT UP BOOST SIGNALS
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <ackermann_local_planner/AckermannPlannerConfig.h>

#include <angles/angles.h>

#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/latched_stop_rotate_controller.h>

#include <base_local_planner/odometry_helper_ros.h>

#include <dubins_plus/dubins_plus.h>

namespace ackermann_local_planner {
  /**
   * @class AckermannPlannerROS
   * @brief ROS Wrapper for the AckermannPlanner that adheres to the
   * BaseLocalPlanner interface and can be used as a plugin for move_base.
   */
  class AckermannPlannerROS : public nav_core::BaseLocalPlanner {
    public:
      /**
       * @brief  Constructor for AckermannPlannerROS
       */
      AckermannPlannerROS();

      /**
       * @brief  Constructs the planner
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      virtual void initialize(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the planner
       */
      virtual ~AckermannPlannerROS();

      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Set the plan that the controller is following
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      virtual bool isGoalReached();

    private:
      bool isInitialized() {
        return initialized_;
      }

      /**
       * @brief Callback to update the local planner's parameters based on dynamic reconfigure
       */
      void reconfigureCB(AckermannPlannerConfig &config, uint32_t level);

      int nearestPoint(const int start_point, 
          const tf::Stamped<tf::Pose> & pose) const;
      double scoreTrajectory(const std::vector<dubins_plus::Segment> &path,
          double global_length, double global_dtheta) const;

      void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);
      void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

      tf::TransformListener* tf_; ///< @brief Used for transforming point clouds

      // for visualisation, publishers of global and local plan
      ros::Publisher g_plan_pub_, l_plan_pub_;

      costmap_2d::Costmap2DROS* costmap_ros_;

      // TODO(hendrix): shared pointer?
      dynamic_reconfigure::Server<AckermannPlannerConfig> *dsrv_;

      bool initialized_;

      base_local_planner::OdometryHelperRos odom_helper_;

      std::vector<geometry_msgs::PoseStamped> plan_;

      // Limits
      double max_vel_;
      double min_vel_;
      double min_radius_;
      double acc_lim_;

      double lookahead_factor_;

      double xy_goal_tolerance_;
      double yaw_goal_tolerance_;

      bool move_;

      int radius_samples_;

      // configuration
      bool have_particlecloud_;
      bool have_pose_with_cow_;


      bool publish_goal_;
      bool publish_near_point_;
      ros::Publisher goal_pub_;
      ros::Publisher near_point_pub_;

      // transient data
      int last_plan_point_;

      bool goal_reached_;
  };

  // Helper functions that don't need class context

  /**
   * @brief determine if one point is forward or backwards from another
   */
  bool isForwards(const geometry_msgs::PoseStamped &start,
      const geometry_msgs::PoseStamped &end);

  /**
   * @brief compute the distance between two points
   */
  double dist(const geometry_msgs::PoseStamped &start,
      const geometry_msgs::PoseStamped &end);

};
#endif
