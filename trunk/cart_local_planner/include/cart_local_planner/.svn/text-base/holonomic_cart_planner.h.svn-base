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
 * \file holonomic_cart_planner.h
 *
 *  \date August 17, 2010
 *  \author Jonathan Scholz
 */

#ifndef HOLONOMIC_CART_PLANNER_H_
#define HOLONOMIC_CART_PLANNER_H_

#include "cart_local_planner/cart_local_planner.h"

namespace cart_local_planner {

class HolonomicCartPlanner: public CartLocalPlanner {
public:
	HolonomicCartPlanner();
	virtual ~HolonomicCartPlanner();

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

	/****************
	 * DATA MEMBERS *
	 ****************/
	enum CONTROL_MODE {
          REGULAR, ROTATING_IN_PLACE, RECOVERY, PULLING_ARMS_IN
	};
	enum CONTROL_MODE control_mode_;

	enum FILTER_OPTIONS {
		GLOBAL_SCALING = 0x1,
		CART_ERR_SCALING = GLOBAL_SCALING << 1,
		COMPENSATE_BASE_TWIST = GLOBAL_SCALING << 2,
		ALL = 0xffff
	};

};

}
; // namespace cart_local_planner

#endif /* HOLONOMIC_CART_PLANNER_H_ */
