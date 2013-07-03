/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 *********************************************************************/

/**
 * \file cart_local_planner.h
 *
 *  \date August 17, 2010
 *  \author Jonathan Scholz
 */

#ifndef CART_LOCAL_PLANNER_H_
#define CART_LOCAL_PLANNER_H_

#include <cart_local_planner/sbpl_subscriber.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <cart_pushing_msgs/RobotCartPath.h>
#include <cart_pushing_msgs/RobotCartConfiguration.h>
#include <boost/scoped_ptr.hpp>
#include <costmap_trajectory_checker/costmap_trajectory_checker.h>
#include <manipulation_transforms/manipulation_transforms.h>

namespace cart_local_planner {

typedef struct {
  double x_min;
  double x_max;
  double y_min;
  double y_max;
  double t_min;
  double t_max;
} pose_range_2D;

/** Twist operators for geometry_msgs types*/
const geometry_msgs::Twist operator+(const geometry_msgs::Twist& t1, const geometry_msgs::Twist& t2);
const geometry_msgs::Twist operator-(const geometry_msgs::Twist& t1, const geometry_msgs::Twist& t2);
double mag(geometry_msgs::Twist &t);
double mag(geometry_msgs::Pose2D &p);

class CartLocalPlanner: public nav_core::BaseLocalPlanner {
public:
  CartLocalPlanner();
  void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
  bool isGoalReached();
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

protected:
  /**
   * @brief A place to implement any additional initialization required for the planner.
   * Gets called at the end of initialize()
   */
  virtual void initialization_extras();

  /************
   * PLANNING *
   ************/
  /**
   * @brief Sets the internal control mode for the computeVelocityCommands switch based on the
   * current state of the robot and the cart.
   */
  virtual void setControlMode();

  /**
   * Implements what to do during computeVelocityCommands for each control mode.  This provides
   * a greater degree of control than the default supported by the base_local_planner API, for
   * problems such as rotating in place or evading collisions that the global planner doesn't
   * know about (e.g. cart collisions).
   */
  virtual void controlModeAction();

  /**
   * Sets goal poses for the cart and the robot given current state and way-point
   */
  virtual void setGoalPoses();

  /**
   * Applies requested operations on the robot and cart twists
   * @param filter_options An int containing OR'ed together FILTER_OPTIONS
   */
  virtual void filterTwistsCombined(int filter_options);

  /**
   * Check twists for collision in costmap
   * @return True if not in collision
   */
  virtual bool checkTwists();

  /**
   * Check twists for collision in costmap
   * @return True if not in collision
   */
  virtual bool checkTwistsMonotonic();

  /************************
   * SUPPORTING UTILITIES *
   ************************/
  /**
   * Picks a future waypoint on the global plan for the cart based on the distance from the robot
   * @param min_dist The minimum distance a future waypoint must be in order to qualify as a cart goal pose
   * @param min_x_component The amount of min_dist that must be in the X direction
   * @return The waypoint number chosen to set the target cart pose, or -1 on failure
   */
  int setCartGoalFromWaypoint(double min_dist, double min_x_component);

  // Set the current goal of the cart/robot and update the error_ variables accordingly given current actual_ pose
  void setCartPoseGoal (const tf::Stamped<tf::Pose>& goal);
  void setRobotPoseGoal (const tf::Stamped<tf::Pose>& goal);

  
  ///* Sets twist_base_ based on robot_pose_error_, returns twist magnitude
  double baseTwistFromError();

  ///* Sets twist_cart_ based on cart_pose_error_, returns twist magnitutde
  double cartTwistFromError();

  ///*  makes a new pose with yaw set from xy coordinate deltas (atan) */
  void setYawFromVec(const tf::Pose& pose1, const tf::Pose& pose2,
                     tf::Pose& res);

  ///* Transforms global_plan to plan in global frame, and does pruning */
  bool transformGlobalPlan(const tf::TransformListener& tf,
                           const std::vector<geometry_msgs::PoseStamped>& global_plan,
                           const costmap_2d::Costmap2DROS& costmap,
                           const std::string& global_frame, std::vector<
                           geometry_msgs::PoseStamped>& transformed_plan);

  ///* converts poly to footprint in the costmap global frame */
  std::vector<geometry_msgs::Point> transformFootprint(const geometry_msgs::PolygonStamped& poly) const;

  ///* set zero twist to arms and base */
  void freeze();

  ///* scale twist by constant factor */
  static void scaleTwist2D(geometry_msgs::Twist& t, double scale);

  ///* map twist_base to cart frame, thanks to KDL's frame*twist operator */
  geometry_msgs::Twist mapBaseTwistToCart(
                                          const geometry_msgs::Twist &twist_base);

  ///* callback for base controller odometry */
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

  ///* callback for invalid pose notification */
  void invalidPoseCallback (const std_msgs::Empty::ConstPtr& msg);

  // Publishers for pose and twist messages in 2D (mainly for debugging and plotting)
  void publishDebugPose(geometry_msgs::Pose &p);
  void publishDebugTwist(geometry_msgs::Twist &t);
  void publishDebugPose(tf::Pose &p);

  /****************
   * DATA MEMBERS *
   ****************/
  enum FILTER_OPTIONS {
    GLOBAL_SCALING = 0x1,
    CART_ERR_SCALING = GLOBAL_SCALING << 1,
    ALL = 0xffff
  };

  enum CONTROL_MODE {
    REGULAR
  };

  // The control mode for the computeVelocityCommands switch
  enum CONTROL_MODE control_mode_;

  tf::TransformListener* tf_;
  tf::TransformBroadcaster tb_;
  ros::NodeHandle nh_;

  costmap_2d::Costmap2DROS* costmap_ros_;

  CostmapTrajectoryChecker robot_collision_checker_;
  CostmapTrajectoryChecker cart_collision_checker_;

  ros::Publisher vel_pub_;
  ros::Subscriber odom_sub_, invalid_pose_sub_;
  nav_msgs::Odometry base_odom_;
  ros::Time goal_reached_time_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  boost::mutex odom_lock_, invalid_pose_mutex_;

  // Transforms for mapping twists around
  tf::Stamped<tf::Pose> robot_pose_actual_; 	/**< Robot pose in fixed frame */
  tf::Stamped<tf::Pose> cart_pose_actual_; 	/**< Cart pose in robot frame */

  // Transforms for holding goal poses
  tf::Stamped<tf::Pose> robot_pose_goal_; 	/**< Robot pose in fixed frame */
  tf::Stamped<tf::Pose> cart_pose_goal_; 		/**< Cart pose in robot frame */

  // Containers for robot and cart error
  tf::Pose robot_pose_error_; 	/**< Robot goal pose in current robot frame */
  geometry_msgs::Pose2D cart_pose_error_;			/**< Distance from actual cart pose to goal pose */

  // Command twists
  geometry_msgs::Twist *twist_base_; /**< Pointer to cmd_vel, the Twist for returning to move_base */
  geometry_msgs::TwistStamped twist_cart_; /**< TwistStamped for writing to cart command_vel topic */

  bool initialized_;
  double K_trans_base_, K_rot_base_; /**< P-gains for the base*/
  double K_trans_cart_, K_rot_cart_; /**< P-gains for the cart */
  int num_traj_steps_; /**< Trajectory length */
  double dt_; /**< Trajectory sample time-interval */
  unsigned int current_waypoint_; /**< Waypoint counter */
  double tolerance_trans_, tolerance_rot_, tolerance_timeout_; /**< Goal threshold tolerances */
  double trans_stopped_velocity_, rot_stopped_velocity_; /**< Used to check if base is stopped at goal */

  // Whether we directly subscribe to sbpl plan rather than getting it from move base
  bool subscribe_sbpl_plan_;

  // For controlling cart position w/ arms
  ros::Publisher cart_pose_pub_; /**< Publisher for controlling cart position */
  ros::Publisher cart_twist_pub_; /**< Publisher for controlling cart velocity */

  // For checking the relative pose between cart and robot
  pose_range_2D cart_range;

  ros::Publisher pose2D_pub_; // A publisher for 2d poses (just for debugging)
  geometry_msgs::Twist twist_cart_max_, twist_base_max_;

  // A flag that indicates whether we are printing debug messages on this iteration of computeVelocityCommand
  bool debug_print_;

  // A timestamp indicating the last time that we received an invalid cart pose message from the
  // articulate cart server
  ros::Time last_invalid_pose_time_;

  boost::shared_ptr<cart_local_planner::SBPLSubscriber<cart_pushing_msgs::RobotCartPath> > sbpl_subscriber_;
  std::vector<geometry_msgs::PoseStamped> original_global_plan_;

  bool getNextFewWaypointsIndices(const std::vector<geometry_msgs::PoseStamped> &current_plan, 
				  const int &current_waypoint_index, 
				  const int &max_num_waypoints, 
				  const double &max_translation,
				  const double &max_rotation,
				  std::vector<unsigned int> &waypoint_indices);

  bool checkTrajectoryMonotonic(const std::vector<unsigned int> &indices);



};

} // namespace
#endif // guard
