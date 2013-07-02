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
 * \file ManipulationTransforms.h
 *
 *  \date Jun 28, 2010
 *  \author Jonathan Scholz
 */

#include <ros/ros.h>
#include <boost/format.hpp>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf_conversions/tf_kdl.h>

#include "manipulation_transforms/manipulation_transforms_ros.h"
#include "manipulation_transforms/util.h"

using namespace std;

ManipulationTransformsROS::ManipulationTransformsROS(const std::string &reference_frame) :
				BASE_FRAME_(reference_frame)
	{
	init_services();
}

ManipulationTransformsROS::ManipulationTransformsROS(const std::string &reference_frame, const std::string &ns) :
		BASE_FRAME_(reference_frame)
	{
	if (ros::isStarted()) {
		init_services(ns);

		// Load rigid grasp transforms
		ROS_DEBUG_STREAM("Waiting for transforms under namespace " << param_nh_.getNamespace());
		ros::Rate r = ros::Rate(15);
		while ( checkForParamServerTransforms(param_nh_, false) < 1 && !ros::isShuttingDown())
			r.sleep();

		//FIXME: there is a potential race condition here if user is attempting to load in many transforms
		//while this call is blocking, but it's almost surely never gonna matter

		if (loadParamServerTransforms(param_nh_))
			ROS_INFO("READY!");
	}
}

ManipulationTransformsROS::ManipulationTransformsROS(const std::string &reference_frame,
		geometry_msgs::PoseStamped &obj_init_pose,
		std::vector<geometry_msgs::PoseStamped> effector_init_poses) :
		BASE_FRAME_(reference_frame)
{
	ROS_WARN_STREAM_COND(obj_init_pose.header.frame_id != BASE_FRAME_, "frame_id for object_pose should be \""
			<< BASE_FRAME_ << "\", but it's \"" << obj_init_pose.header.frame_id << "\"");

	// Set number of effectors from the number of transforms provided
	n_effectors_ = effector_init_poses.size();
	ROS_INFO("Initializing with %u effector transforms", n_effectors_);

	if (n_effectors_ < 1 ) {
		ROS_ERROR("Expected transforms for at least one effector, but got %u", n_effectors_);
		ROS_WARN_COND(n_effectors_ != 1 && n_effectors_ != 2, "Querying object pose only supported for 1 or 2 effectors");
	}

	tf::Transform bt_object_pose;
	tf::poseMsgToTF(obj_init_pose.pose, bt_object_pose);

	vector<tf::Transform> bt_effector_poses;
	bt_effector_poses.resize(n_effectors_);

	for (unsigned int i = 0; i < n_effectors_; ++i) {
		ROS_WARN_STREAM_COND(effector_init_poses[i].header.frame_id != BASE_FRAME_, "frame_id for effector_pose should be \""
					<< BASE_FRAME_ << "\", but it's \"" << effector_init_poses[i].header.frame_id << "\"");
		tf::poseMsgToTF(effector_init_poses[i].pose, bt_effector_poses[i]);
	}

	// Pass service call transforms into solver
	solver_.setInitialTransforms(bt_object_pose, bt_effector_poses);
}

ManipulationTransformsROS::~ManipulationTransformsROS(){}

void ManipulationTransformsROS::init_services(const std::string &ns)
{
	// Initialize private namespace
	private_nh_ = ros::NodeHandle("~");

	// Initialize param namespace
	param_nh_ = ros::NodeHandle(ns);

	// Advertise services
	load_initial_transforms_service_ = private_nh_.advertiseService("LoadInitialTransforms", &ManipulationTransformsROS::loadInitialTransforms, this);
	set_initial_transforms_service_ = private_nh_.advertiseService("SetInitialTransforms", &ManipulationTransformsROS::setInitialTransforms, this);
	effector_poses_to_object_service_ = private_nh_.advertiseService("MapEffectorPosesToObject", &ManipulationTransformsROS::mapEffectorPosesToObject, this);
	object_pose_to_effectors_service_ = private_nh_.advertiseService("MapObjectPoseToEffectors", &ManipulationTransformsROS::mapObjectPoseToEffectors, this);
	effector_twists_to_object_service_ = private_nh_.advertiseService("MapEffectorTwistsToObject", &ManipulationTransformsROS::mapEffectorTwistsToObject, this);
	object_twist_to_effectors_service_ = private_nh_.advertiseService("MapObjectTwistToEffectors", &ManipulationTransformsROS::mapObjectTwistToEffectors, this);
	effector_trajectories_to_object_service_ = private_nh_.advertiseService("MapEffectorTrajectoriesToObject", &ManipulationTransformsROS::mapEffectorTrajectoriesToObject, this);
	object_trajectory_to_effectors_service_ = private_nh_.advertiseService("MapObjectTrajectoryToEffectors", &ManipulationTransformsROS::mapObjectTrajectoryToEffectors, this);
}

bool ManipulationTransformsROS::setInitialTransforms(manipulation_transforms::SetInitialTransforms::Request &req,
		manipulation_transforms::SetInitialTransforms::Response &resp)
{
	ROS_WARN_STREAM_COND(req.object_pose.header.frame_id != BASE_FRAME_, "frame_id for object_pose should be \""
			<< BASE_FRAME_ << "\", but it's \"" << req.object_pose.header.frame_id << "\"");

	// Set number of effectors from the number of transforms provided
	n_effectors_ = req.effector_poses.size();
	ROS_INFO("Initializing with %u effector transforms", n_effectors_);

	if (n_effectors_ < 1 ) {
		ROS_ERROR("Expected transforms for at least one effector, but got %u", n_effectors_);
		ROS_WARN_COND(n_effectors_ != 1 && n_effectors_ != 2, "Querying object pose only supported for 1 or 2 effectors");
		resp.success = false;
	}

	tf::poseMsgToTF(req.object_pose.pose, obj_initial_pose_);

	effector_init_poses.resize(n_effectors_);

	for (unsigned int i = 0; i < n_effectors_; ++i) {
		ROS_WARN_STREAM_COND(req.effector_poses[i].header.frame_id != BASE_FRAME_, "frame_id for effector_pose should be \""
					<< BASE_FRAME_ << "\", but it's \"" << req.effector_poses[i].header.frame_id << "\"");
		tf::poseMsgToTF(req.effector_poses[i].pose, effector_init_poses[i]);
	}

	// Pass service call transforms into solver
	solver_.setInitialTransforms(obj_initial_pose_, effector_init_poses);

	// Report to user
	ROS_DEBUG_STREAM("OBJECT INITIAL POSE: " << manipulation_transforms_util::btTransform_to_string(obj_initial_pose_));
	for (unsigned int i = 0; i < n_effectors_; ++i)
		ROS_DEBUG_STREAM("EFFECTOR " << i << " INITIAL POSE: " << manipulation_transforms_util::btTransform_to_string(effector_init_poses[i]));

	resp.success = true;

	return resp.success;
}

bool ManipulationTransformsROS::loadInitialTransforms(manipulation_transforms::LoadInitialTransforms::Request &req,
		manipulation_transforms::LoadInitialTransforms::Response &resp)
{
	// Allow user to load from private node handle TODO: document
	if (req.name == "useprivate") {
		resp.success = loadParamServerTransforms(param_nh_);
	}
	else {
		ros::NodeHandle nh(req.name);
		resp.success = loadParamServerTransforms(nh);
	}

	return resp.success;
}

/**
 * @brief Provides the service interface to mapEffectorPosesToObject
 */
bool ManipulationTransformsROS::mapEffectorPosesToObject(manipulation_transforms::MapEffectorPosesToObject::Request &req,
		manipulation_transforms::MapEffectorPosesToObject::Response &resp)
{
	if (n_effectors_ != req.effector_poses.size()) {
		ROS_ERROR("Expected transforms for %u effectors, but got %lu", n_effectors_, req.effector_poses.size());
	}

	// Transform request to btTransforms
	tf::Transform bt_object_pose;
	vector<tf::Transform> bt_effector_poses;
	bt_effector_poses.resize(n_effectors_);

	for (unsigned int i = 0; i < n_effectors_; ++i) {
		ROS_WARN_STREAM_COND(req.effector_poses[i].header.frame_id != BASE_FRAME_,
				"frame_id for effector pose " << i << " should be \"" << BASE_FRAME_ << "\", but it's \""
				<< req.effector_poses[i].header.frame_id << "\"");
		ROS_DEBUG_STREAM("effector " << i << " query pose: " << req.effector_poses[i].pose);
		tf::poseMsgToTF(req.effector_poses[i].pose, bt_effector_poses[i]);
	}

	// Find solution pose
	resp.error = solver_.mapEffectorPosesToObject(bt_effector_poses, bt_object_pose);

	// Transform back to message
	tf::poseTFToMsg(bt_object_pose, resp.object_pose.pose);
	resp.object_pose.header.frame_id = BASE_FRAME_;
	resp.object_pose.header.stamp = ros::Time::now();

	ROS_DEBUG_STREAM("result obj pose: " << resp.object_pose);

	return true;
}

/**
 * @brief Provides the service interface to mapObjectPoseToEffectors
 */
bool ManipulationTransformsROS::mapObjectPoseToEffectors(manipulation_transforms::MapObjectPoseToEffectors::Request &req,
		manipulation_transforms::MapObjectPoseToEffectors::Response &resp)
{
	ROS_WARN_STREAM_COND(req.object_pose.header.frame_id != BASE_FRAME_,
			"frame_id for object_pose should be \"" << BASE_FRAME_ << "\", but it's \""
			<< req.object_pose.header.frame_id << "\"");

	ROS_DEBUG_STREAM("obj query pose: " << req.object_pose);

	// Transform request to btTransforms
	tf::Transform bt_object_pose;
	tf::poseMsgToTF(req.object_pose.pose, bt_object_pose);

	// Allocate space for solution
	vector<tf::Transform> bt_effector_poses;
	bt_effector_poses.resize(n_effectors_); // will throw warnings without this

	// Find solution poses
	solver_.mapObjectPoseToEffectors(bt_object_pose, bt_effector_poses);

	// Transform back to messages
	resp.effector_poses.resize(n_effectors_);
	for (unsigned int i = 0; i < n_effectors_; ++i) {
		tf::poseTFToMsg(bt_effector_poses[i], resp.effector_poses[i].pose);
		resp.effector_poses[i].header.frame_id = BASE_FRAME_;
		resp.effector_poses[i].header.stamp = ros::Time::now();

		ROS_DEBUG_STREAM("effector " << i << " result pose: " << resp.effector_poses[i]);
	}

	return true;
}

/**
 * @brief Provides the service interface to mapEffectorTwistsToObject
 */
bool ManipulationTransformsROS::mapEffectorTwistsToObject(manipulation_transforms::MapEffectorTwistsToObject::Request &req,
		manipulation_transforms::MapEffectorTwistsToObject::Response &resp)
{
	if (n_effectors_ != req.effector_twists.size()) {
			ROS_ERROR("Expected transforms for %u effectors, but got %lu", n_effectors_, req.effector_twists.size());
		}

		// Transform request to KDL twists
		KDL::Twist object_twist;
		vector<KDL::Twist> effector_twists;
		effector_twists.resize(n_effectors_);

		for (unsigned int i = 0; i < n_effectors_; ++i) {
			ROS_WARN_STREAM_COND(req.effector_twists[i].header.frame_id != BASE_FRAME_,
					"frame_id for effector pose " << i << " should be \"" << BASE_FRAME_ << "\", but it's \""
					<< req.effector_twists[i].header.frame_id << "\"");
			ROS_DEBUG_STREAM("effector " << i << " query twist: " << req.effector_twists[i].twist);
			tf::TwistMsgToKDL(req.effector_twists[i].twist, effector_twists[i]);
		}

		// Find solution pose
		resp.error = solver_.mapEffectorTwistsToObject(effector_twists, object_twist);

		// Transform back to message
		tf::TwistKDLToMsg(object_twist, resp.object_twist.twist);
		resp.object_twist.header.frame_id = BASE_FRAME_;
		resp.object_twist.header.stamp = ros::Time::now();

		ROS_DEBUG_STREAM("result obj pose: " << resp.object_twist);

		return true;
}

/**
 * @brief Provides the service interface to mapObjectTwistToEffectors
 */
bool ManipulationTransformsROS::mapObjectTwistToEffectors(manipulation_transforms::MapObjectTwistToEffectors::Request &req,
		manipulation_transforms::MapObjectTwistToEffectors::Response &resp)
{

	ROS_WARN_STREAM_COND(req.object_twist.header.frame_id != BASE_FRAME_,
			"frame_id for object_pose should be \"" << BASE_FRAME_ << "\", but it's \""
			<< req.object_twist.header.frame_id << "\"");

	ROS_DEBUG_STREAM("obj query pose: " << req.object_twist);

	// Transform request to KDL
	KDL::Twist object_twist;
	tf::TwistMsgToKDL(req.object_twist.twist, object_twist);

	// Allocate space for solution
	vector<KDL::Twist> effector_twists;
	effector_twists.resize(n_effectors_); // will throw warnings without this

	// Find solution poses
	solver_.mapObjectTwistToEffectors(object_twist, effector_twists);

	// Transform back to messages
	resp.effector_twists.resize(n_effectors_);
	for (unsigned int i = 0; i < n_effectors_; ++i) {
		tf::TwistKDLToMsg(effector_twists[i], resp.effector_twists[i].twist);
		resp.effector_twists[i].header.frame_id = BASE_FRAME_;
		resp.effector_twists[i].header.stamp = ros::Time::now();

		ROS_DEBUG_STREAM("effector " << i << " result twist: " << resp.effector_twists[i]);
	}

	return true;
}

/**
* @brief Provides the service interface for getting effector trajectories from an object trajectory
*/
bool ManipulationTransformsROS::mapObjectTrajectoryToEffectors(manipulation_transforms::MapObjectTrajectoryToEffectors::Request &req,
		manipulation_transforms::MapObjectTrajectoryToEffectors::Response &resp)
{
	//TODO implement this
	return true;
}

/**
* @brief Provides the service interface for getting an object trajectory from effector trajectories
*/
bool ManipulationTransformsROS::mapEffectorTrajectoriesToObject(manipulation_transforms::MapEffectorTrajectoriesToObject::Request &req,
			manipulation_transforms::MapEffectorTrajectoriesToObject::Response &resp)
{
	//TODO implement this
	return true;
}

int ManipulationTransformsROS::checkForParamServerTransforms(const ros::NodeHandle &nh, bool warn)
{
	//check object transform parameters first
	bool user_obj_params = false;
	user_obj_params |= nh.hasParam("obj_init_pose/position");
	user_obj_params |= nh.hasParam("obj_init_pose/orientation");

	// Bail now if no object transforms found
	if (!user_obj_params) {
		ROS_WARN_STREAM_COND(warn,
				"Could not detect required transform parameters \"obj_init_pose\" under namespace \""
				<<  nh.getNamespace() << "\"");
		return -1;
	}

	// Otherwise, start looking for effector transforms matching "effector#_init_pose/position" and "effector#_init_pose/orientation"
	int n = 0;
	while (nh.hasParam((boost::format("effector%u_init_pose/position") % n).str()) &&
		nh.hasParam((boost::format("effector%u_init_pose/orientation") % n).str())) {
		n++;
	}

	return n;
}

bool ManipulationTransformsROS::loadParamServerTransforms(const ros::NodeHandle &nh)
{
	// Set n_effectors_ to the number of transforms found on the parameter server
	if ((n_effectors_ = checkForParamServerTransforms(nh, false)) < 1)
		return false;

	ROS_WARN_COND(n_effectors_ != 1 && n_effectors_ != 2, "Querying object pose only supported for 1 or 2 effectors");
	ROS_INFO("Initializing with %u effector transforms found on the parameter server", n_effectors_);

	obj_initial_pose_ = manipulation_transforms_util::readTransformParameter(nh, "obj_init_pose");
	effector_init_poses.resize(n_effectors_);

	for (unsigned int i = 0; i < n_effectors_; ++i)
		effector_init_poses[i] = manipulation_transforms_util::readTransformParameter(nh, (boost::format("effector%u_init_pose") % i).str());

	// Pass loaded transforms into solver
	solver_.setInitialTransforms(obj_initial_pose_, effector_init_poses);

	ROS_INFO_STREAM("Loaded new obj grasp transforms from namespace \"" << nh.getNamespace() << "\"");
	ROS_DEBUG_STREAM("OBJECT INITIAL POSE: " << manipulation_transforms_util::btTransform_to_string(obj_initial_pose_));
	for (unsigned int i = 0; i < n_effectors_; ++i)
		ROS_DEBUG_STREAM("EFFECTOR " << i << " INITIAL POSE: " << manipulation_transforms_util::btTransform_to_string(effector_init_poses[i]));

	return true;
}


