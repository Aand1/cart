/*
 * holonomic_cart_planner.cpp
 *
 *  Created on: Aug 17, 2010
 *      Author: jscholz
 */

#include <cart_local_planner/holonomic_cart_planner.h>
#include <cart_local_planner/utils.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(cart_local_planner, HolonomicCartPlanner, cart_local_planner::HolonomicCartPlanner, nav_core::BaseLocalPlanner)

namespace cart_local_planner {
HolonomicCartPlanner::HolonomicCartPlanner()
{
}

void HolonomicCartPlanner::initialization_extras()
{
}

HolonomicCartPlanner::~HolonomicCartPlanner() {
}

void HolonomicCartPlanner::setControlMode()
{
  // double yaw_err = tf::getYaw(robot_pose_error_.getRotation());
  boost::mutex::scoped_lock lock(invalid_pose_mutex_);    
  if (ros::Time::now() - last_invalid_pose_time_ < ros::Duration(3.0)) 
    control_mode_ = PULLING_ARMS_IN;
  //  else if (fabs(yaw_err) > M_PI_4 && control_mode_ != RECOVERY)
  //    control_mode_ = ROTATING_IN_PLACE;
  else
    control_mode_ = REGULAR;

  ROS_DEBUG_COND_NAMED(debug_print_, "mode", "mode = %d", control_mode_);
}

void HolonomicCartPlanner::controlModeAction()
{
  switch (control_mode_) {


  case PULLING_ARMS_IN:
    {
      /* 2010/9/3: Disabling to test the articulate cart server recovery
      ROS_DEBUG_STREAM_NAMED ("recovery", "In pulling-arms-in recovery; current cart pose goal is " <<
                              toString(cart_pose_goal_) << "; robot pose goal is " << toString(robot_pose_goal_));
      // Reset the cart goal pose to be close to the robot
      tf::Stamped<tf::Pose> goal;
      goal.setIdentity();
      goal.getOrigin().setX(0.5*(cart_range.x_min+cart_range.x_max));
      setRobotPoseGoal(robot_pose_actual_);;
      setCartPoseGoal(goal);
      ROS_DEBUG_STREAM_NAMED ("recovery", "  Modified cart pose goal to " << toString(cart_pose_goal_) <<
                              " and robot goal to " << toString(robot_pose_goal_));
      */

    }
    // We don't break, but continue with the regular behavior with the modified goals
    
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

  case ROTATING_IN_PLACE:
    {
      cartTwistFromError();
      baseTwistFromError();

      // Coordinate base and cart velocities for smooth and safe action
      filterTwistsCombined(GLOBAL_SCALING);
    }
    break;

  case RECOVERY:
    ROS_WARN("NOT IMPLEMENTED");
    break;

  default:
    ROS_WARN("Unrecognized control mode requested");
    break;
  }
}

void HolonomicCartPlanner::filterTwistsCombined(int filter_options)
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
      scaleTwist2D(*twist_base_, scale_mult); //TODO verify that this works
      scaleTwist2D(twist_cart_.twist, scale_mult);
      ROS_DEBUG_STREAM_COND_NAMED (debug_print_, "twist_filter",
                                   "Scaling, to keep things in range, cart and base twists by " << scale_mult);
    }

  }

  /// 2: Scale base vel based on cart error (turns out to be a very handy trick for keeping the cart on track - this is basically a gaussian)
  if (filter_options & CART_ERR_SCALING) {
    const double scaling_factor = pow(M_E, -50.0 * pow(mag(twist_cart_.twist), 2)); // tuned to fall off around 0.1
    scaleTwist2D(*twist_base_, scaling_factor);
    ROS_DEBUG_STREAM_COND_NAMED (debug_print_, "twist_filter",
                                 "Scaling, based on cart error, base velocity by " << scaling_factor);
  }

  /// 3: Map base twist to cart and subtract it from cart twist (makes cart track better as base rounds corners)
  if (filter_options & COMPENSATE_BASE_TWIST) {
    geometry_msgs::Twist base_twist_at_cart = mapBaseTwistToCart(*twist_base_);
    ROS_DEBUG_STREAM_COND_NAMED (debug_print_, "twist_filter", "Cart twist " << twist_cart_ <<
                                 " and base twist " << base_twist_at_cart);
    twist_cart_.twist.linear.y -= base_twist_at_cart.linear.y;
    twist_cart_.twist.angular.z -= base_twist_at_cart.angular.z;
    ROS_DEBUG_STREAM_COND_NAMED (debug_print_, "twist_filter", "After compensating for base twist, cart twist is "
                                 << twist_cart_);
  }

  /// 4: Scale everything by its max;
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
      ROS_DEBUG_STREAM_COND_NAMED (debug_print_, "twist_filter",
                                   "Scaling, to keep things in range, cart and base twists by " << scale_mult);
    }

  }

}

}; // namespace cart_local_planner
