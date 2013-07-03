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
 * \file fixed_front_cart_planner.h
 *
 *  \date August 17, 2010
 *  \author Jonathan Scholz
 */

#ifndef HOLONOMIC_CART_PLANNER_H_
#define HOLONOMIC_CART_PLANNER_H_

#include "cart_local_planner/cart_local_planner.h"

/*
 ** TODO write a CLP for steel cart?
 It'd need to have base vel be subordinate to cart vel, and have a recovery behavior of some sort:
 1: get target cart pose as usual
 2: get the required yaw to move the cart in this direction (assumes we have mobility around main axis)
 a) if the angle is +- 90, then we're in "forward" mode, and we can set this cart angle and drive the base forward
 b) if the angle is > +-90, then we're past the target waypoint, and we have to
 3) we'll often have a Y offset between the cart and the target waypoint which we need to decide how to fix.
 Ideally we'd decide to recover for the current point or for a future point based on how much space we have ahead
 For now, the simplest thing is to have a recovery mode in which the cart jockeys back and forth to pull the cart back to the target pose.
 We can accomplish this with the arms, while incrementing and decrementing the waypoint counter by some amount (*must allow negative base vels)
 ---------------
 check y error
 if greater than a threshold, we're in recovery mode
 else, we're in regular drive forward mode.
 threshold is whatever the maximum y offset the cart can have where the robot is still capable of pointing it back towards the middle


 drive forward mode: main problem is that cart has to be pointed towards the plan before we drive forward
 pick a cart angle so that the yerr is elimated in N steps, where N is a customizable param that determines how conservative the controller is.
 This param is what we'd control if we want it to be lazy in an open room, but careful in tight spaces.  Since this is getting called every loop,
 it'll actually take way longer than N, but will conveniently decay to zero yaw and tend to minimize yerr.

 recovery mode:

 ----------
 what's different from the holonomic case:

 1: base vel is strictly subordinate to cart vel
 -> solution: make virtual func for base scaling from cart err

 2: modes depend on y error and yaw error\
 ----------
 to implement:
 option 1: override the setGoals funciton to compute a target cart pose that points towards the plan, and always has a y-error of zero.  then add a filter
 operation that stops the base unless the cart error is basically zero

 option 2: try to do everything at the twist level, by filtering out the linear.y component and  biasing the angular.z component such that the y
 error will tend to reduce.

 option 2 is simpler, but dumber and tougher to tune
 */

namespace cart_local_planner {

class FixedFrontCartPlanner: public CartLocalPlanner {
public:
	FixedFrontCartPlanner();
	virtual ~FixedFrontCartPlanner();

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
	 * @brief Sets the internal control mode for the computeVelocityCommands switch
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
	 * Applies requested operations on the provided twists
	 * @param filter_options An int containing OR'ed together FILTER_OPTIONS
	 */
	virtual void filterTwistsCombined(int filter_options);

	/**
	 * Compute the pose required to point
	 * @param point_pose
	 * @return
	 */
	double pointCartAtTarget(tf::Pose &point_pose);

	/****************
	 * DATA MEMBERS *
	 ****************/
	enum CONTROL_MODE {
		REGULAR, ROTATING_IN_PLACE, RECOVERY
	};
	enum CONTROL_MODE control_mode_;

	enum FILTER_OPTIONS {
		GLOBAL_SCALING = 0x1,
		CART_ERR_SCALING = GLOBAL_SCALING << 1,
		COMPENSATE_BASE_TWIST = GLOBAL_SCALING << 2,
		HOLONOMIC_CONSTRAINT =  GLOBAL_SCALING << 3,
		ALL = 0xffff
	};

	double cart_max_y_offset_; 		///< Maximum distance in y direction allowed for cart before we're in recovery mode
	double y_compensation_gain_; 	///< scalar to multiply y-error for adujusting yaw

	CostmapTrajectoryChecker extra_cart_collision_checker_;
};

}
; // namespace cart_local_planner

#endif /* FIXED_FRONT_CART_PLANNER_H_ */
