/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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
 *
 */

/**
 * \file 
 * 
 * Allows subscribing to sbpl over a backchannel and synchronizing with the 
 * global plan provided by move_base
 *
 * \author Bhaskara Marthi
 */

#ifndef CART_LOCAL_PLANNER_SBPL_SUBSCRIBER_H
#define CART_LOCAL_PLANNER_SBPL_SUBSCRIBER_H

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <boost/circular_buffer.hpp>

namespace cart_local_planner
{

template <class SBPLPlan>
class SBPLSubscriber
{
public:
  typedef std::vector<geometry_msgs::PoseStamped> MBPlan;
  
  SBPLSubscriber (const ros::NodeHandle& nh, const std::string& topic, const double timeout=0.01,
                  const unsigned buffer_size=10) :
    nh_(nh), sub_(nh_.subscribe(topic, buffer_size, &SBPLSubscriber<SBPLPlan>::planCB, this)), 
    plan_buffer_(buffer_size), timeout_(timeout)
  {
  }

  boost::optional<SBPLPlan>  lookupPlan (const MBPlan& mb_plan) const
  {
    const double num_waits = 10.0;
    ros::Rate r(num_waits/timeout_);
    boost::optional<SBPLPlan> plan;
    for (unsigned i=0; i<num_waits; i++) {
      r.sleep();
      boost::mutex::scoped_lock l(mutex_);
      typename boost::circular_buffer<SBPLPlan>::const_reverse_iterator iter =
        find(plan_buffer_.rbegin(), plan_buffer_.rend(), mb_plan);
      if (iter!=plan_buffer_.rend()) {
        plan = *iter;
        return plan;
      }
    }
    // No matching plan found, so return an uninitialized value
    return plan;
  }

private:

  mutable boost::mutex mutex_;
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  boost::circular_buffer<SBPLPlan> plan_buffer_;
  const double timeout_;

  void planCB (const SBPLPlan& sbpl_plan)
  {
    boost::mutex::scoped_lock l(mutex_);
    plan_buffer_.push_back(sbpl_plan);
  }

};

} // namespace

#endif // include guard
