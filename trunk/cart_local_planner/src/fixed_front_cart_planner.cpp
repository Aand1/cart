/*
 * holonomic_cart_planner.cpp
 *
 *  Created on: Aug 17, 2010
 *      Author: jscholz
 */

#include "cart_local_planner/fixed_front_cart_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(cart_local_planner, FixedFrontCartPlanner, cart_local_planner::FixedFrontCartPlanner, nav_core::BaseLocalPlanner)

namespace cart_local_planner {
FixedFrontCartPlanner::FixedFrontCartPlanner()
{
}

FixedFrontCartPlanner::~FixedFrontCartPlanner()
{
}

void FixedFrontCartPlanner::initialization_extras()
{
	cart_max_y_offset_ = 0.1;
	y_compensation_gain_ = 5.0;
	extra_cart_collision_checker_ = cart_collision_checker_;
	extra_cart_collision_checker_.setPubTopic(std::string("extra_checker"));
}

void FixedFrontCartPlanner::setControlMode()
{
  if (cart_pose_goal_.getOrigin().y() - cart_pose_actual_.getOrigin().y() > cart_max_y_offset_)
    control_mode_ = RECOVERY;
  else if (fabs(tf::getYaw(robot_pose_error_.getRotation())) > M_PI_4 && control_mode_ != RECOVERY)
    control_mode_ = REGULAR;
  else
    control_mode_ = REGULAR;

        

  ROS_DEBUG("mode = %d", control_mode_);
}

void FixedFrontCartPlanner::controlModeAction()
{
  switch (control_mode_) {
  case REGULAR:
    {
      // Compute base twist
      baseTwistFromError();

      // Compute cart twist
      cartTwistFromError();

      // Coordinate base and cart velocities for smooth and safe action
      filterTwistsCombined(ALL);
      ROS_DEBUG("current_waypoint_ = %u", current_waypoint_);
      if (robot_pose_error_.getOrigin().length() < tolerance_trans_ && current_waypoint_ < global_plan_.size()-1)
        current_waypoint_++;
    }
    break;

  case ROTATING_IN_PLACE:
    {
      ROS_WARN("NOT IMPLEMENTED");
      freeze(); // until we figure out something better
    }
    break;

		case RECOVERY:
			ROS_WARN("NOT IMPLEMENTED");
			freeze(); // until we figure out something better
			break;

		default:
			ROS_WARN("Unrecognized control mode requested");
			break;
		}
}

void FixedFrontCartPlanner::filterTwistsCombined(int filter_options)
{
	/// Scale everything by its max;
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
			scaleTwist2D(*twist_base_, scale_mult); //TODO verify that this works
			scaleTwist2D(twist_cart_.twist, scale_mult);
		}
	}

	/// Map base twist to cart and subtract it from cart twist (makes cart track better as base rounds corners)
	if (filter_options & COMPENSATE_BASE_TWIST) {
		geometry_msgs::Twist base_twist_at_cart = mapBaseTwistToCart(*twist_base_);
		twist_cart_.twist.angular.z -= base_twist_at_cart.angular.z;
	}

	/// Eliminate linear.y component of cart twist, and make up for it with angular.z
	if (filter_options & HOLONOMIC_CONSTRAINT) {
		// check the pre-adjusted twist, just for visualization
		geometry_msgs::Twist base_twist_at_cart = mapBaseTwistToCart(*twist_base_);
		geometry_msgs::Twist twist_net = base_twist_at_cart + twist_cart_.twist;
		extra_cart_collision_checker_.checkTwist(twist_net, num_traj_steps_, dt_, true, false);

		// gain tuning hack:
		static int idx = 0;
		if (idx % 20 == 0) {
			ros::param::get("/ygain", y_compensation_gain_);
			ROS_WARN("fetched new ygain from param server: %lf", y_compensation_gain_);
		}
		++idx;

		ROS_INFO("cart_pose_error_.y = %.3lf", cart_pose_error_.y);
		twist_cart_.twist.linear.y = 0;
		ROS_INFO("twist.a.z old = %.3lf", twist_cart_.twist.angular.z);
		twist_cart_.twist.angular.z += cart_pose_error_.y * y_compensation_gain_;
		ROS_INFO("twist.a.z new = %.3lf", twist_cart_.twist.angular.z);
	}

	/// Scale base vel based on cart error (turns out to be a very handy trick for keeping the cart on track - this is basically a gaussian)
	if (filter_options & CART_ERR_SCALING) {
		ROS_DEBUG("cart twist mag = %.3lf, gaussian scaling factor = %.3lf", mag(twist_cart_.twist), pow(M_E, -50.0 * pow(mag(twist_cart_.twist), 2)));
		scaleTwist2D(*twist_base_, pow(M_E, -50.0 * pow(mag(twist_cart_.twist), 2))); // tuned to fall off around 0.1
		//scaleTwist(twist_base_, pow(M_E, -600.0 * pow(mag(twist_cart_.twist), 3))); // for a more aggressive version, still falling off at around 0.1
	}
}

}; // namespace cart_local_planner
