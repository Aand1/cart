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
 * \file cart_local_planner.cpp
 *
 *  \date August 17, 2010
 *  \author Jonathan Scholz
 */

#include "cart_local_planner/cart_local_planner.h"
#include <base_local_planner/goal_functions.h>
#include <tf_conversions/tf_kdl.h>
#include <pluginlib/class_list_macros.h>
#include <vector>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>

PLUGINLIB_DECLARE_CLASS(cart_local_planner, CartLocalPlanner, cart_local_planner::CartLocalPlanner, nav_core::BaseLocalPlanner)

namespace gm = geometry_msgs;


namespace cart_pushing_msgs {


bool operator==(const cart_pushing_msgs::RobotCartPath& sbpl_plan,
		const std::vector<gm::PoseStamped>& mb_plan)

{
	const unsigned n = sbpl_plan.path.size();
	if (n != mb_plan.size())
  {
    ROS_ERROR("Plans don't have the same size: %d, SBPL: %d",(int)mb_plan.size(),(int)sbpl_plan.path.size());
		return false;
  }
	for (unsigned i = 0; i < n; i++) {
		// Check if poses are the same
		const gm::Pose& p = mb_plan[i].pose;
		const gm::Pose& p2 = sbpl_plan.path[i].robot_pose;
		if ((p.position.x != p2.position.x) || (p.position.y != p2.position.y)
				|| (p.position.z != p2.position.z) || (p.orientation.x
				!= p2.orientation.x) || (p.orientation.y != p2.orientation.y)
				|| (p.orientation.z != p2.orientation.z) || (p.orientation.w
				!= p2.orientation.w))
    {
      ROS_ERROR("Plans don't have the same waypoint: %d",i);
			return false;
    }
	}
	return true;
}
} // namespace cart_pushing_msgs

namespace cart_local_planner {
using std::vector;
using std::max;
using std::string;
using boost::optional;
const double SBPL_DTHRESH=-0.013; // SBPL mPrims translate about 0.0186 on a straight shot
typedef tf::Stamped<tf::Pose> StampedPose;

template<class T>
T clamp(T val, T abs_max) {
	return val > 0 ? (val > abs_max ? abs_max : val)
			: (val < -abs_max ? -abs_max : val);
}

const gm::Twist operator+(const gm::Twist& t1, const gm::Twist& t2)
{
	gm::Twist res;
	res.linear.x = t1.linear.x + t2.linear.x;
	res.linear.y = t1.linear.y + t2.linear.y;
	res.linear.z = t1.linear.z + t2.linear.z;
	res.angular.x = t1.angular.x + t2.angular.x;
	res.angular.y = t1.angular.y + t2.angular.y;
	res.angular.z = t1.angular.z + t2.angular.z;

  return res;
}

const gm::Twist operator-(const gm::Twist& t1, const gm::Twist& t2)
{
	gm::Twist res;
	res.linear.x = t1.linear.x - t2.linear.x;
	res.linear.y = t1.linear.y - t2.linear.y;
	res.linear.z = t1.linear.z - t2.linear.z;
	res.angular.x = t1.angular.x - t2.angular.x;
	res.angular.y = t1.angular.y - t2.angular.y;
	res.angular.z = t1.angular.z - t2.angular.z;

  return res;
}

double mag(gm::Twist &t) {
	return sqrt(t.linear.x*t.linear.x + t.linear.y*t.linear.y + t.linear.z*t.linear.z +
			t.angular.x*t.angular.x + t.angular.y*t.angular.y + t.angular.z*t.angular.z);
}

double mag(geometry_msgs::Pose2D &p) {
	return sqrt(p.x*p.x+p.y*p.y+p.theta*p.theta);
}

CartLocalPlanner::CartLocalPlanner() :
  tf_(NULL), nh_("cart_planner_costmap"), costmap_ros_(NULL), initialized_(false)
{
}

template <typename T>
void getParam (const ros::NodeHandle nh, const string& name, T* place)
{
  bool found = nh.getParam(name, *place);
  ROS_ASSERT_MSG (found, "Did not find parameter %s", nh.resolveName(name).c_str());
}

void CartLocalPlanner::initialize(std::string name, tf::TransformListener* tf,
		costmap_2d::Costmap2DROS* costmap_ros) {
	tf_ = tf;
	costmap_ros_ = costmap_ros;
	current_waypoint_ = 0;
	goal_reached_time_ = ros::Time::now();
	ros::NodeHandle cart_nh("cart_pushing");
        ros::NodeHandle nh("~/"+name);

	// Get a nodehandle for reading cart params
	std::string cart_param_ns;
        getParam(nh, "cart_param_namespace", &cart_param_ns);
	ros::NodeHandle cart_param_nh(cart_param_ns);

	// initialize robot collision checker
	robot_collision_checker_.initialize(costmap_ros_, "robot_trajectory");

	// Get twist saturation limits
	getParam(cart_nh, "max_vel_base_x", &twist_base_max_.linear.x);
	getParam(cart_nh, "max_vel_base_y", &twist_base_max_.linear.y);
	getParam(cart_nh, "max_vel_base_theta", &twist_base_max_.angular.z);
	getParam(cart_nh, "max_vel_cart_x", &twist_cart_max_.linear.x);
	getParam(cart_nh, "max_vel_cart_y", &twist_cart_max_.linear.y);
	getParam(cart_nh, "max_vel_cart_theta", &twist_cart_max_.angular.z);

	// Get trajectory rollout params
	getParam(nh, "dt", &dt_);
	getParam(nh, "num_traj_steps", &num_traj_steps_);

	// Get cart footprint dimensions
	double cart_length, cart_width, cart_x_offset, cart_y_offset;
	getParam(cart_param_nh, "length", &cart_length);
	getParam(cart_param_nh, "width", &cart_width);
	getParam(cart_param_nh, "footprint_x_offset", &cart_x_offset);
	getParam(cart_param_nh, "footprint_y_offset", &cart_y_offset);
	cart_collision_checker_.initialize(costmap_ros_, "cart_trajectory");
	cart_collision_checker_.setFootprint(cart_length, cart_width, cart_x_offset, cart_y_offset);
	cart_collision_checker_.setRobotFrameID("cart");

	getParam(nh, "k_trans_base", &K_trans_base_);
	getParam(nh, "k_rot_base", &K_rot_base_);
	getParam(nh, "k_trans_cart", &K_trans_cart_);
	getParam(nh, "k_rot_cart", &K_rot_cart_);

	getParam(nh, "subscribe_sbpl_plan", &subscribe_sbpl_plan_);

	ros::NodeHandle node;
	odom_sub_ = node.subscribe<nav_msgs::Odometry> ("odom", 1, &CartLocalPlanner::odomCallback, this);
	invalid_pose_sub_ = node.subscribe<std_msgs::Empty> ("cart_pushing/articulate_cart_server/invalid_pose",
                                                       1, &CartLocalPlanner::invalidPoseCallback, this);
	vel_pub_ = node.advertise<gm::Twist> ("base_controller/command", 10);

        if(subscribe_sbpl_plan_)
        {
          ROS_INFO("Setting up SBPL subscriber");
          sbpl_subscriber_.reset(new cart_local_planner::SBPLSubscriber<cart_pushing_msgs::RobotCartPath>(node,"/move_base_node/SBPLCartPlanner/sbpl_robot_cart_plan"));
        }

	// Attach to cart control topic
	std::string cart_articulation_ns; // Get a nodehandle for reading cart params
        getParam(nh, "cart_articulation_namespace", &cart_articulation_ns);
	cart_pose_pub_ = node.advertise<gm::PoseStamped>(cart_articulation_ns + "/command_pose", 10);
	cart_twist_pub_ = node.advertise<gm::TwistStamped>(cart_articulation_ns + "/command_twist", 10);

	// Get workspace range params (for filtering cart commands as well as base commands)
	getParam(cart_param_nh, "x_min", &cart_range.x_min);
	getParam(cart_param_nh, "x_max", &cart_range.x_max);
	getParam(cart_param_nh, "y_min", &cart_range.y_min);
	getParam(cart_param_nh, "y_max", &cart_range.y_max);
	getParam(cart_param_nh, "t_min", &cart_range.t_min);
	getParam(cart_param_nh, "t_max", &cart_range.t_max);
	ROS_DEBUG("Loaded workspace range from %s: %.2lf %.2lf, %.2lf %.2lf, %.2lf %.2lf", cart_param_nh.getNamespace().c_str(), cart_range.x_min, cart_range.x_max,
				cart_range.y_min, cart_range.y_max, cart_range.t_min, cart_range.t_max);

	// Get Termination and stop criteria params
	getParam(nh, "tolerance_trans", &tolerance_trans_);
	getParam(nh, "tolerance_rot", &tolerance_rot_);
	getParam(nh, "tolerance_timeout", &tolerance_timeout_);
	getParam(nh, "trans_stopped_velocity", &trans_stopped_velocity_);
	getParam(nh, "rot_stopped_velocity", &rot_stopped_velocity_);

	// Apply any extra initialization
	initialization_extras();

	// Publishers for plotting poses and twists (mainly for debugging)
	pose2D_pub_ = nh.advertise<gm::Pose2D> ("debug_pose", 10);

	initialized_ = true;
	ROS_DEBUG("Initialized");
}

void CartLocalPlanner::initialization_extras()
{
}

bool CartLocalPlanner::computeVelocityCommands(gm::Twist& cmd_vel)
{
  { // Check if we're printing debug messages, which happens every N iterations
    static unsigned debug_counter=0;
    debug_counter++;
    debug_print_ = !(debug_counter%20);
  }
  
  /// Get the current pose of the robot in the fixed frame
  tf::Stamped<tf::Pose> robot_pose;
  static ros::Time last_good_command;

  ROS_DEBUG_COND_NAMED (debug_print_, "loop", "Computing velocity commands");

  // So if we get a pose that's too old, we'll freeze, but not actually signal failure unless it's
  // been a while since we succeeded
  if (!costmap_ros_->getRobotPose(robot_pose)) {
    freeze();
    ros::Duration time_since_last_good_command = ros::Time::now()-last_good_command;
    ROS_WARN_STREAM("Can't get robot pose; time since last good command is " << time_since_last_good_command);
    return time_since_last_good_command < ros::Duration(2.0);
  }
  /// Get the current pose of the cart in the base frame
  tf::StampedTransform cart_pose;
  tf_->lookupTransform("base_footprint", "cart", ros::Time(), cart_pose);

  /// Save member copies of cart and robot states (the slow way, because TF is annoying)
  robot_pose_actual_.setBasis(robot_pose.getBasis());
  robot_pose_actual_.setOrigin(robot_pose.getOrigin());
  cart_pose_actual_.setBasis(cart_pose.getBasis());
  cart_pose_actual_.setOrigin(cart_pose.getOrigin());

  /// Alias cmd_vel with our local reference
  twist_base_ = &cmd_vel;

  /// Set goals for the robot and cart given the current state
  setGoalPoses();

  ROS_DEBUG_COND_NAMED (debug_print_, "loop", " Set goal poses");

  /// Set a control mode based on current state
  setControlMode();

  /// Generate controls for current control mode
  controlModeAction();

  ROS_DEBUG_COND_NAMED (debug_print_, "loop", " Generated controls");


  bool found_good_vels = false;

  /*  if(!subscribe_sbpl_plan_)
      {*/
      /// Check final twists before returning
      found_good_vels = checkTwists();
      if (found_good_vels) 
	{
	  last_good_command = ros::Time::now();
	  twist_cart_.twist.linear.z = 0.0;
	  twist_cart_.twist.angular.x = 0.0;
	  twist_cart_.twist.angular.y = 0.0;
	  cart_twist_pub_.publish(twist_cart_);
	}
      else 
	{
	  found_good_vels = checkTwistsMonotonic();
	  if (found_good_vels) 
	    {
	      ROS_INFO("Twists are taking us away from collision");
	      last_good_command = ros::Time::now();
	      twist_cart_.twist.linear.z = 0.0;
	      twist_cart_.twist.angular.x = 0.0;
	      twist_cart_.twist.angular.y = 0.0;
	      cart_twist_pub_.publish(twist_cart_);
	    }
	  else
	    {
	      ROS_WARN_THROTTLE(1, "Freezing because the twists are leading to collision");
	      freeze();      
	    }
	}
      /*    }
  else
    {
      std::vector<unsigned int> waypoint_indices;
      getNextFewWaypointsIndices(global_plan_,current_waypoint_,20,0.25,0.25,waypoint_indices);
      found_good_vels = checkTrajectoryMonotonic(waypoint_indices);
      if (found_good_vels) 
	{
	  ROS_DEBUG("Twists are taking us away from collision");
	  last_good_command = ros::Time::now();
	  twist_cart_.twist.linear.z = 0.0;
	  twist_cart_.twist.angular.x = 0.0;
	  twist_cart_.twist.angular.y = 0.0;
	  cart_twist_pub_.publish(twist_cart_);
	}
      else
	{
	  ROS_WARN_THROTTLE(1, "Freezing because the twists are leading to collision");
	  freeze();      
	}
	}*/
  publishDebugPose(robot_pose_actual_);
  ROS_DEBUG_COND_NAMED(debug_print_, "loop", "final base twist: [%.3f,%.3f][%.3f]",
                       twist_base_->linear.x, twist_base_->linear.y, twist_base_->angular.z);
  ROS_DEBUG_COND_NAMED(debug_print_, "loop", "final cart twist: [%.3f,%.3f][%.3f]",
                       twist_cart_.twist.linear.x, twist_cart_.twist.linear.y, twist_cart_.twist.angular.z);

  return found_good_vels;
}

bool CartLocalPlanner::checkTrajectoryMonotonic(const std::vector<unsigned int> &indices)
{
  boost::optional<cart_pushing_msgs::RobotCartPath> sbpl_plan = sbpl_subscriber_->lookupPlan(original_global_plan_);
  if(!sbpl_plan)
  {
    ROS_ERROR("Could not find SBPL plan");
    return false;
  }
  std::vector<geometry_msgs::Pose2D> robot_path, cart_path;

  for(unsigned int i=0; i <indices.size(); i++)
  {
    unsigned int index = indices[i];

    geometry_msgs::Pose2D robot_pose_2d;
    robot_pose_2d.x = global_plan_[index].pose.position.x;
    robot_pose_2d.y = global_plan_[index].pose.position.y;
    robot_pose_2d.theta = tf::getYaw(global_plan_[index].pose.orientation);
    robot_path.push_back(robot_pose_2d);

    tf::Pose robot_pose, cart_pose_local, cart_pose_global;
    tf::poseMsgToTF(global_plan_[index].pose,robot_pose);
    tf::poseMsgToTF(sbpl_plan->path[index].cart_pose,cart_pose_local);
    cart_pose_global = robot_pose*cart_pose_local;

    geometry_msgs::Pose2D cart_pose;
    cart_pose.x = cart_pose_global.getOrigin().x();
    cart_pose.y = cart_pose_global.getOrigin().y();
    cart_pose.theta = tf::getYaw(cart_pose_global.getRotation());
    cart_path.push_back(cart_pose);

  }

  // Check base twist for collisions on the base footprint
  double base_footprint_cost=robot_collision_checker_.checkTrajectoryMonotonic(robot_path,true,true,5);
  if(base_footprint_cost < 0.0)
  {
    ROS_INFO("Base footprint in collision");
    return false;
  }
  // Check the base+cart twists for collisions on the cart footprint
  double cart_footprint_cost=cart_collision_checker_.checkTrajectoryMonotonic(cart_path,true,true,5);
  if(cart_footprint_cost < 0.0)
  {
    ROS_INFO("Cart footprint in collision");
    return false;
  }
  return true;
}
  
bool CartLocalPlanner::getNextFewWaypointsIndices(const std::vector<geometry_msgs::PoseStamped> &current_plan, 
						const int &current_waypoint_index, 
						const int &max_num_waypoints, 
						const double &max_translation,
						const double &max_rotation,
						std::vector<unsigned int> &waypoint_indices)
{
  if(current_waypoint_index > (int)current_plan.size()-1)
  {
    ROS_ERROR("Current waypoint exceeds number of points in plan");
    return false;
  }
  unsigned int num_waypoints = 0;
  waypoint_indices.clear();

  tf::Stamped<tf::Pose> current_waypoint;
  tf::poseStampedMsgToTF(current_plan[current_waypoint_index],current_waypoint);
  for(unsigned int i = current_waypoint_index; i < current_plan.size(); i++)
  {
    tf::Stamped<tf::Pose> next_waypoint;
    tf::poseStampedMsgToTF(current_plan[i],next_waypoint);
    tf::Pose error = current_waypoint.inverseTimes(next_waypoint);
    if(error.getOrigin().length() > max_translation)
      break;
    if(fabs(tf::getYaw(error.getRotation())) > max_rotation)
      break;
    waypoint_indices.push_back(i);
    num_waypoints++;
    if((int)num_waypoints > max_num_waypoints)
      break;
  }
  return true;
}

void CartLocalPlanner::setRobotPoseGoal (const StampedPose& goal)
{
  robot_pose_goal_ = goal;
  robot_pose_error_ = robot_pose_actual_.inverseTimes(robot_pose_goal_);
}

void CartLocalPlanner::setCartPoseGoal (const StampedPose& goal)
{
  cart_pose_goal_ = goal;
  cart_pose_error_.x = cart_pose_goal_.getOrigin().x() - cart_pose_actual_.getOrigin().x();
  cart_pose_error_.y = cart_pose_goal_.getOrigin().y() - cart_pose_actual_.getOrigin().y();
  const double dtheta = tf::getYaw(cart_pose_goal_.getRotation()) - tf::getYaw(cart_pose_actual_.getRotation());
  cart_pose_error_.theta = angles::normalize_angle(dtheta);
  ROS_DEBUG("Cart pose error: %f %f %f",cart_pose_error_.x,cart_pose_error_.y,cart_pose_error_.theta);
}

void CartLocalPlanner::setControlMode()
{
	control_mode_ = REGULAR;
}

void CartLocalPlanner::controlModeAction()
{
  switch (control_mode_) {
  case REGULAR:
    {
      // Compute base twist
      baseTwistFromError();

      // Compute cart twist
      cartTwistFromError();

      // Coordinate base and cart velocities for smooth and safe action
      filterTwistsCombined(GLOBAL_SCALING);

      if (robot_pose_error_.getOrigin().length() < tolerance_trans_ && current_waypoint_ < global_plan_.size()-1 && mag(cart_pose_error_) < 0.1)
        current_waypoint_++;
    }
    break;
  default:
    ROS_WARN("Unrecognized control mode requested");
    break;
  }
}

void CartLocalPlanner::setGoalPoses()
{
  ros::Time now(ros::Time::now());
  StampedPose robot_pose_goal;
  tf::poseStampedMsgToTF(global_plan_[current_waypoint_], robot_pose_goal);
  setRobotPoseGoal(robot_pose_goal);
  setCartGoalFromWaypoint((cart_range.x_max + cart_range.x_min)/2, cart_range.x_min);

  tb_.sendTransform(tf::StampedTransform(robot_pose_goal_, now, costmap_ros_->getGlobalFrameID(),
                                         "base_target_pose"));
  tb_.sendTransform(tf::StampedTransform(cart_pose_goal_, now, "base_footprint", "cart_target_unfiltered"));  

  StampedPose cart_pose_goal = cart_pose_goal_;
    
  // Filter cart pose
  if (cart_pose_goal.getOrigin().x() < cart_range.x_min)
    cart_pose_goal.getOrigin().setX(cart_range.x_min);
  if (cart_pose_goal.getOrigin().x() > cart_range.x_max)
    cart_pose_goal.getOrigin().setX(cart_range.x_max);

  if (cart_pose_goal.getOrigin().y() < cart_range.y_min)
    cart_pose_goal.getOrigin().setY(cart_range.y_min);
  if (cart_pose_goal.getOrigin().y() > cart_range.y_max)
    cart_pose_goal.getOrigin().setY(cart_range.y_max);

  double yaw, pitch, roll;
  cart_pose_goal.getBasis().getEulerYPR(yaw, pitch, roll);
  if (yaw < cart_range.t_min)
    cart_pose_goal.getBasis().setEulerYPR(cart_range.t_min, pitch, roll);
  if (yaw > cart_range.t_max)
    cart_pose_goal.getBasis().setEulerYPR(cart_range.t_max, pitch, roll);

  // Update the cart pose and the error
  setCartPoseGoal(cart_pose_goal);

  tb_.sendTransform(tf::StampedTransform(cart_pose_goal_, now, "base_footprint", "cart_target_pose"));

}

void CartLocalPlanner::filterTwistsCombined(int filter_options)
{
  /// 1: Scale everything by its max;
  if (filter_options & GLOBAL_SCALING) {
    double xv_scale = fabs(twist_base_->linear.x) / twist_base_max_.linear.x;
    double yv_scale = fabs(twist_base_->linear.y) / twist_base_max_.linear.y;
    double tv_scale = fabs(twist_base_->angular.z) / twist_base_max_.angular.z;

    double base_scaling_factor = std::max(xv_scale, std::max(yv_scale, tv_scale));

    double xv_cart_scale = fabs(twist_cart_.twist.linear.x) / twist_cart_max_.linear.x;
    double yv_cart_scale = fabs(twist_cart_.twist.linear.y) / twist_cart_max_.linear.y;
    double tv_cart_scale = fabs(twist_cart_.twist.angular.z)/ twist_cart_max_.angular.z;

    double cart_scaling_factor = std::max(xv_cart_scale, std::max(yv_cart_scale, tv_cart_scale));
    double scaling_factor = std::max(base_scaling_factor, cart_scaling_factor);

    // Scales both twists together such that none are above their limits
    if (scaling_factor > 1.0) {
      double scale_mult = 1.0 / scaling_factor;
      scaleTwist2D(*twist_base_, scale_mult);
      scaleTwist2D(twist_cart_.twist, scale_mult);
      ROS_DEBUG_STREAM_COND_NAMED (debug_print_, "twist_filter",
                                   "Scaling, to keep things in range, cart and base twists by " << scale_mult);
    }
  }

  /// 2: Scale base vel based on cart error (turns out to be a very handy trick for keeping the cart on track - this is basically a gaussian)
  if (filter_options & CART_ERR_SCALING) {
    const double scaling_factor = pow(M_E, -50.0 * pow(mag(twist_cart_.twist), 2)); // falls off around .1
    scaleTwist2D(*twist_base_, scaling_factor);
    ROS_DEBUG_STREAM_COND_NAMED (debug_print_, "twist_filter",
                                 "Scaling, based on cart error, base velocity by a factor of " << scaling_factor);
  }
}


/// Note: discards non-2d components
gm::Twist scaleTwist (const gm::Twist& twist, const double scaling_factor)
{
  gm::Twist result;
  result.linear.x = twist.linear.x*scaling_factor;
  result.linear.y = twist.linear.y*scaling_factor;
  result.angular.z = twist.angular.z*scaling_factor;
  return result;
}

bool CartLocalPlanner::checkTwists()
{
  double base_footprint_cost, cart_footprint_cost;
  optional<double> cost_of_valid_twist; // has value iff last twist found is collision-free

  for (double scaling_factor=1.0; scaling_factor > 0.7 && !cost_of_valid_twist; scaling_factor *= 0.9) {
    const gm::Twist scaled_base_twist = scaleTwist(*twist_base_, scaling_factor);
    const gm::Twist scaled_cart_twist = scaleTwist(twist_cart_.twist, scaling_factor);
    const gm::Twist base_twist_at_cart = mapBaseTwistToCart(scaled_base_twist);
    const gm::Twist net_twist = base_twist_at_cart + scaled_cart_twist;
    ROS_DEBUG_STREAM_COND_NAMED (debug_print_, "check_twists", "Checking twists scaled by " << scaling_factor <<
                                 ": " << base_twist_at_cart << ", " << net_twist);

    // Check base twist for collisions on the base footprint
    base_footprint_cost=robot_collision_checker_.checkTwist(scaled_base_twist, num_traj_steps_, dt_, true, true);
    // Check the base+cart twists for collisions on the cart footprint
    cart_footprint_cost=cart_collision_checker_.checkTwist(net_twist, num_traj_steps_, dt_, true, false);

    if ((base_footprint_cost >= 0) && (cart_footprint_cost >= 0))
      cost_of_valid_twist = max(base_footprint_cost, cart_footprint_cost);      
  }

  // We found a collision-free twist
  if (cost_of_valid_twist) {
    ROS_DEBUG_COND_NAMED (debug_print_, "check_twists", "Found collision-free twist with cost %.2f",
                          *cost_of_valid_twist);
    return true;
  }
  else {
    ROS_DEBUG_COND_NAMED (debug_print_, "check_twists", "No valid twist found");
    return false;
  }
}

bool CartLocalPlanner::checkTwistsMonotonic()
{
  double base_footprint_cost, cart_footprint_cost;
  optional<double> cost_of_valid_twist; // has value iff last twist found is collision-free
  robot_collision_checker_.clearFootprint(true);
  cart_collision_checker_.clearFootprint(false);
  gm::Twist scaled_base_twist = scaleTwist(*twist_base_, 1.0);
  gm::Twist scaled_cart_twist = scaleTwist(twist_cart_.twist, 1.0);
  gm::Twist base_twist_at_cart = mapBaseTwistToCart(scaled_base_twist);
  gm::Twist net_twist = base_twist_at_cart + scaled_cart_twist;
  for (double scaling_factor=1.0; scaling_factor > 0.7 && !cost_of_valid_twist; scaling_factor *= 0.9) {
    scaled_base_twist = scaleTwist(*twist_base_, scaling_factor);
    scaled_cart_twist = scaleTwist(twist_cart_.twist, scaling_factor);
    base_twist_at_cart = mapBaseTwistToCart(scaled_base_twist);
    net_twist = base_twist_at_cart + scaled_cart_twist;
    ROS_DEBUG_STREAM_COND_NAMED (debug_print_, "check_twists", "Checking twists scaled by " << scaling_factor <<
                                 ": " << base_twist_at_cart << ", " << net_twist);

    // Check base twist for collisions on the base footprint
    base_footprint_cost=robot_collision_checker_.checkTwistMonotonic(scaled_base_twist, num_traj_steps_, dt_, false, false);
    // Check the base+cart twists for collisions on the cart footprint
    cart_footprint_cost=cart_collision_checker_.checkTwistMonotonic(net_twist, num_traj_steps_, dt_, false, false);

    if ((base_footprint_cost >= 0) && (cart_footprint_cost >= 0))
      cost_of_valid_twist = max(base_footprint_cost, cart_footprint_cost);      
  }

  // We found a collision-free twist
  if (cost_of_valid_twist) {
    ROS_DEBUG_COND_NAMED (debug_print_, "check_twists", "Found collision-free twist with cost %.2f",
                          *cost_of_valid_twist);
    twist_cart_.twist = scaled_cart_twist;
    *twist_base_ = scaled_base_twist;
    return true;
  }
  else {
    ROS_DEBUG_COND_NAMED (debug_print_, "check_twists", "No valid twist found");
    return false;
  }
}


int CartLocalPlanner::setCartGoalFromWaypoint(double min_dist, double min_x_component) 
{
  StampedPose goal;
  if(subscribe_sbpl_plan_)
  {
    boost::optional<cart_pushing_msgs::RobotCartPath> sbpl_plan = sbpl_subscriber_->lookupPlan(original_global_plan_);
    if(sbpl_plan)
    {
      tf::Transform goal_transform;
      tf::poseMsgToTF(sbpl_plan->path[current_waypoint_].cart_pose,goal_transform);//assume this is in robot frame of reference
      ROS_DEBUG("Cart pose: %f %f %f, %f %f %f %f",
               sbpl_plan->path[current_waypoint_].cart_pose.position.x,
               sbpl_plan->path[current_waypoint_].cart_pose.position.y,
               sbpl_plan->path[current_waypoint_].cart_pose.position.z,
               sbpl_plan->path[current_waypoint_].cart_pose.orientation.x,
               sbpl_plan->path[current_waypoint_].cart_pose.orientation.y,
               sbpl_plan->path[current_waypoint_].cart_pose.orientation.z,
               sbpl_plan->path[current_waypoint_].cart_pose.orientation.w);               
      goal.setRotation(goal_transform.getRotation());
      goal.setOrigin(goal_transform.getOrigin());
      setCartPoseGoal(goal);
      return current_waypoint_;
    }
    else
    {
      // DO NOTHING
      ROS_ERROR("Could not find sbpl plan");
      return 0;
    }
  }
  // find the future waypoint that provides the closest relpose to ideal cart distance
  tf::Transform tr, rel_tr;
  for (uint i = current_waypoint_; i < global_plan_.size(); ++i) {
    tf::poseMsgToTF(global_plan_[i].pose, tr);
    rel_tr = robot_pose_actual_.inverseTimes(tr);
    if (rel_tr.getOrigin().length() > min_dist && rel_tr.getOrigin().x()
        > min_x_component) {
      goal.setRotation(rel_tr.getRotation());
      goal.setOrigin(rel_tr.getOrigin());
      setCartPoseGoal(goal);
      return i;
    }
  }
  // If we reached the end without satisfying the goal, set to relative pose of robot to last waypoint w/ standard offset
  goal.setRotation(rel_tr.getRotation());
  goal.setOrigin(tf::Vector3(min_dist, 0, 0));
  setCartPoseGoal(goal);
  return 0;
}


double CartLocalPlanner::baseTwistFromError()
{
	twist_base_->linear.x = robot_pose_error_.getOrigin().x() * K_trans_base_;
	twist_base_->linear.y = robot_pose_error_.getOrigin().y() * K_trans_base_;
	twist_base_->angular.z = tf::getYaw(robot_pose_error_.getRotation()) * K_rot_base_;

	return mag(*twist_base_);
}

double CartLocalPlanner::cartTwistFromError()
{
	// Use relative pose as twist orientation errors
	twist_cart_.twist.linear.x  = cart_pose_error_.x * K_trans_cart_;
	twist_cart_.twist.linear.y  = cart_pose_error_.y * K_trans_cart_;
	twist_cart_.twist.angular.z = cart_pose_error_.theta * K_rot_cart_;
	twist_cart_.header.frame_id = "base_footprint";
	twist_cart_.header.stamp = ros::Time::now();

	return mag(twist_cart_.twist);
}

void CartLocalPlanner::setYawFromVec(const tf::Pose& pose1, const tf::Pose& pose2, tf::Pose& res)
{
  res = pose1;
  // Extracts yaw using relative
  const tf::Vector3 v = pose1.getOrigin() - pose2.getOrigin();
  double yaw = 0;

  const double waypoint_dist = v.length();
  if (waypoint_dist < 0.01) { // 0.01
    // if points are super close by, just leave at pose1 yaw
    ROS_WARN("WAYPOINTS TOO CLOSE - HOLDING YAW FIXED");
    yaw = tf::getYaw(pose1.getRotation());
  }
  else {
    // otherwise set using v
    yaw = atan2(v.getY(), v.getX());
  }

  tf::Quaternion q;
  q.setRPY(0, 0, yaw);
  res.setRotation(q);
}

bool CartLocalPlanner::setPlan(
		const std::vector<gm::PoseStamped>& global_plan) {
	current_waypoint_ = 0;
	goal_reached_time_ = ros::Time::now();
  original_global_plan_ = global_plan;
	if (!transformGlobalPlan(*tf_, global_plan, *costmap_ros_,
			costmap_ros_->getGlobalFrameID(), global_plan_)) {
		ROS_ERROR("Could not transform the global plan to the frame of the controller");
		return false;
	}
	return true;
}

bool CartLocalPlanner::transformGlobalPlan(const tf::TransformListener& tf,
                                           const std::vector<gm::PoseStamped>& global_plan,
                                           const costmap_2d::Costmap2DROS& costmap,
                                           const std::string& global_frame,
                                           std::vector<gm::PoseStamped>& transformed_plan) {
  if (!global_plan.size() > 0) {
    ROS_ERROR("Received plan with zero length");
    return false;
  }
  const gm::PoseStamped& pose = global_plan[0];
  transformed_plan.clear();

  tf::StampedTransform transform;
                
  try {
    tf.lookupTransform(global_frame, ros::Time(),
                       pose.header.frame_id, pose.header.stamp,
                       pose.header.frame_id, transform);
  }
  catch (const tf::TransformException& ex) {
    ROS_ERROR ("TF Exception while transforming global plan: %s", ex.what());
    return false;
  }

  tf::Stamped<tf::Pose> tf_pose;
  gm::PoseStamped newer_pose;
  //now we'll transform until points are outside of our distance threshold
  // (jon): what threshold?  It looks like we're doing all of them
  // (jon): anyway, i'm gonna use this as an opportunity to do some path smoothing
  /* [logic being, since we're not taking SBPL's word for it on orientation, having
     waypoints that differ only in yaw are kinda pointless]*/

  tf::Stamped<tf::Pose> last_pose;
  // Do the first one alone
  poseStampedMsgToTF(pose, last_pose);
  last_pose.setData(transform * last_pose);
  last_pose.stamp_ = transform.stamp_;
  last_pose.frame_id_ = global_frame;
  poseStampedTFToMsg(last_pose, newer_pose);
  transformed_plan.push_back(newer_pose);

  // now loop over the rest, and add them if they pass a translation threshold
  int pts_skipped = 0;
  for (unsigned int i = 1; i < global_plan.size(); ++i) {
    poseStampedMsgToTF(global_plan[i], tf_pose);
    const tf::Pose tmp_pose = tf_pose.inverseTimes(last_pose);
    const double dist = tmp_pose.getOrigin().length();
    if (dist > SBPL_DTHRESH) {
      // Get transformed plan pose directly from global plan
      tf_pose.setData(transform * tf_pose);
      if(!subscribe_sbpl_plan_)
        // If requested, don't trust SBPL yaw param - compute yaw from vector difference from last pose
        setYawFromVec(tf_pose, last_pose, tf_pose);        

      //tf_pose.setData(transform * tf_pose);
      tf_pose.stamp_ = transform.stamp_;
      tf_pose.frame_id_ = global_frame;
      poseStampedTFToMsg(tf_pose, newer_pose);
      transformed_plan.push_back(newer_pose);

    }
    else {
      pts_skipped++;
    }
    last_pose = tf_pose;
  }
  ROS_DEBUG_COND_NAMED(debug_print_, "transform_global_plan", "added %zu points (%d skipped)",
                  global_plan.size()-pts_skipped, pts_skipped);
  return true;
}

vector<gm::Point> CartLocalPlanner::transformFootprint(
                                                       const gm::PolygonStamped& poly) const {
  vector<gm::Point> points;
  BOOST_FOREACH (const gm::Point32 p, poly.polygon.points)
  {
    gm::PointStamped in, out;
    in.header.frame_id = poly.header.frame_id;
    in.header.stamp = ros::Time(); // Should actually wait
    in.point.x = p.x;
    in.point.y = p.y;
    in.point.z = p.z;
    tf_->transformPoint(costmap_ros_->getGlobalFrameID(), in,
                        out);
    points.push_back(out.point);
  }
  return points;
}

void CartLocalPlanner::freeze()
{
  ROS_WARN_THROTTLE(3.0, "Robot is in frozen state in cart local planner");
  gm::Twist empty_twist;
  twist_base_->linear.x = empty_twist.linear.x;
  twist_base_->linear.y = empty_twist.linear.y;
  twist_base_->angular.z = empty_twist.angular.z;
  twist_cart_.twist = empty_twist;
  twist_cart_.header.stamp = ros::Time::now();

  cart_twist_pub_.publish(twist_cart_);
}

void CartLocalPlanner::scaleTwist2D(gm::Twist &t, double scale)
{
	t.linear.x *= scale;
	t.linear.y *= scale;
	t.angular.z *= scale;
}

gm::Twist CartLocalPlanner::mapBaseTwistToCart(const gm::Twist &twist_base)
{
	KDL::Frame cart_frame;
	tf::PoseTFToKDL(cart_pose_actual_.inverse(), cart_frame);

	KDL::Twist twist_at_base;
	tf::TwistMsgToKDL(twist_base, twist_at_base);

	KDL::Twist twist_at_cart = cart_frame * twist_at_base;

	gm::Twist t;
	tf::TwistKDLToMsg(twist_at_cart, t);
	return t;
}

void CartLocalPlanner::publishDebugPose(gm::Pose &p) {
	gm::Pose2D msg = gm::Pose2D();
	msg.x = p.position.x;
	msg.y = p.position.y;
	msg.theta = tf::getYaw(tf::Quaternion(p.orientation.x, p.orientation.y,
			p.orientation.z, p.orientation.w));
	pose2D_pub_.publish(msg);
}

void CartLocalPlanner::publishDebugTwist(gm::Twist &t) {
	gm::Pose2D msg = gm::Pose2D();
	msg.x = t.linear.x;
	msg.y = t.linear.y;
	msg.theta = t.angular.z;
	pose2D_pub_.publish(msg);
}

void CartLocalPlanner::publishDebugPose(tf::Pose &p) {
	gm::Pose2D msg = gm::Pose2D();
	msg.x = p.getOrigin().x();
	msg.y = p.getOrigin().y();
	msg.theta = tf::getYaw(p.getRotation());
	pose2D_pub_.publish(msg);
}

void CartLocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  	//we assume that the odometry is published in the frame of the base
	boost::mutex::scoped_lock lock(odom_lock_);
	base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
	base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
	base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
}

void CartLocalPlanner::invalidPoseCallback (const std_msgs::Empty::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(invalid_pose_mutex_);
  last_invalid_pose_time_ = ros::Time::now();
}

bool CartLocalPlanner::isGoalReached() {
  if (!initialized_) {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  bool quick_check = current_waypoint_ == global_plan_.size() - 1;
  if (!quick_check)
    return false;
  else {
    ROS_DEBUG_COND_NAMED(debug_print_, "is_goal_reached", "Quick check passed, checking actual goal distance");
    //copy over the odometry information
    nav_msgs::Odometry base_odom;
    {
      boost::recursive_mutex::scoped_lock(odom_lock_);
      base_odom = base_odom_;
    }

    bool val = base_local_planner::isGoalReached(*tf_, global_plan_,
                                                 *costmap_ros_, costmap_ros_->getGlobalFrameID(), base_odom,
                                                 rot_stopped_velocity_, trans_stopped_velocity_,
                                                 tolerance_trans_+0.1, tolerance_rot_+0.1);
    if (val) {
      ROS_DEBUG_COND_NAMED(debug_print_, "is_goal_reached", "Agreed with quick check... GOAL REACHED!");
      freeze();
    }
    else
      ROS_DEBUG("Tracking to last waypoint, should be at goal soon.");

    return val;
  }
}

}; // namespace cart_local_planner
