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
 * ManipulationTransforms.h
 *
 *  \date Jun 28, 2010
 *  \author Jonathan Scholz
 */
#include <iostream>
#include <tf_conversions/tf_kdl.h>

#include "manipulation_transforms/manipulation_transforms.h"


ManipulationTransforms::ManipulationTransforms()
{
	n_effectors_ = 0;
}

ManipulationTransforms::ManipulationTransforms(const tf::Transform &obj_initial_pose, const tf::Transform &effector_initial_pose)
{
	setInitialTransforms(obj_initial_pose, effector_initial_pose);
}

ManipulationTransforms::ManipulationTransforms(const tf::Transform &obj_initial_pose, const tf::Transform &effector1_initial_pose, const tf::Transform &effector2_initial_pose)
{
	setInitialTransforms(obj_initial_pose, effector1_initial_pose, effector2_initial_pose);
}

ManipulationTransforms::ManipulationTransforms(const tf::Transform & obj_initial_pose, const std::vector<tf::Transform> &effector_initial_poses)
{
	n_effectors_ = effector_initial_poses.size();
	setInitialTransforms(obj_initial_pose, effector_initial_poses);
}

ManipulationTransforms::~ManipulationTransforms(){}

/**
 * @brief Computes the relative transforms between obj and effectors given the transforms specified in the reference frame
 */
void ManipulationTransforms::setInitialTransforms(const tf::Transform & obj_initial_pose, const std::vector<tf::Transform> &effector_initial_poses)
{
	if (n_effectors_ == 0) {
		if (effector_initial_poses.size() == 0)
			std::cout << "Error: at least 1 effector pose needed" << std::endl;
		else
			n_effectors_ = effector_initial_poses.size();
	}

	// Store reference frame transforms
	obj_T_ref_ = obj_initial_pose;
	effector_T_ref_ = effector_initial_poses;

	obj_T_effectors_.resize(n_effectors_);
	effector_T_obj_.resize(n_effectors_);

	for (unsigned int i = 0; i < n_effectors_; ++i){
		obj_T_effectors_[i] = effector_initial_poses[i].inverseTimes(obj_initial_pose);
		effector_T_obj_[i] = obj_initial_pose.inverseTimes(effector_initial_poses[i]);
	}
}

void ManipulationTransforms::setInitialTransforms(const tf::Transform &obj_initial_pose, const tf::Transform &effector_initial_pose)
{
	n_effectors_ = 1;
	obj_T_ref_ = obj_initial_pose;
	std::vector<tf::Transform> effector_initial_poses;
	effector_initial_poses.push_back(effector_initial_pose);
	effector_T_ref_ = effector_initial_poses;

	obj_T_effectors_.resize(n_effectors_);
	effector_T_obj_.resize(n_effectors_);

	for (unsigned int i = 0; i < n_effectors_; ++i){
		obj_T_effectors_[i] = effector_initial_poses[i].inverseTimes(obj_initial_pose);
		effector_T_obj_[i] = obj_initial_pose.inverseTimes(effector_initial_poses[i]);
	}
}

void ManipulationTransforms::setInitialTransforms(const tf::Transform &obj_initial_pose, const tf::Transform &effector1_initial_pose, const tf::Transform &effector2_initial_pose)
{
	n_effectors_ = 2;
	obj_T_ref_ = obj_initial_pose;
	std::vector<tf::Transform> effector_initial_poses;
	effector_initial_poses.push_back(effector1_initial_pose);
	effector_initial_poses.push_back(effector2_initial_pose);
	effector_T_ref_ = effector_initial_poses;

	obj_T_effectors_.resize(n_effectors_);
	effector_T_obj_.resize(n_effectors_);

	for (unsigned int i = 0; i < n_effectors_; ++i){
		obj_T_effectors_[i] = effector_initial_poses[i].inverseTimes(obj_initial_pose);
		effector_T_obj_[i] = obj_initial_pose.inverseTimes(effector_initial_poses[i]);
	}
}

/**
 * @ brief Computes obj_pose from effector poses by applying the forward transforms
 */
double ManipulationTransforms::mapEffectorPosesToObject(const std::vector<tf::Transform> &effector_query_poses, tf::Transform &obj_pose)
{
	if (n_effectors_ != effector_query_poses.size()){
		std::cout << "Error: initialized with " << n_effectors_ << " effectors, but received query with " << effector_query_poses.size() << std::endl;
		return -1;
	}

	switch (n_effectors_) {
	case 0:
		std::cout << "Error: please initialize with effector and object transforms" << std::endl;
		break;
	case 1:
		obj_pose = effector_query_poses[0] * obj_T_effectors_[0];
		return 0;
	case 2:
	{
		/// Get obj pose according to right effector
		tf::Transform transform0 = effector_query_poses[0] * obj_T_effectors_[0];
		tf::Transform transform1 = effector_query_poses[1] * obj_T_effectors_[1];

		// Get rotation quaternion
		tf::Quaternion quat0 = transform0.getRotation();
		tf::Quaternion quat1 = transform1.getRotation();

		/// Combine two rotations equally
		tf::Quaternion mean_rot = quat0.slerp(quat1, 0.5);

		/// Combine mean rotation with mean translation into final transform
		obj_pose = tf::Transform(mean_rot, (transform0.getOrigin() + transform1.getOrigin()) / 2);

		/// Return an error metric based on agreement of rotational and translational components
		return quat0.dot(quat1) + transform0.getOrigin().dot(transform1.getOrigin());
	}
	default:
		std::cout << "Warning: currently supports only 1 or 2 arms.  Consider getting a simpler robot" << std::endl;
	}
	return -1;
}

double ManipulationTransforms::mapEffectorPosesToObject(const tf::Transform &effector_query_pose, tf::Transform &obj_pose)
{
	if (n_effectors_ != 1)
		std::cout << "Warning: initialized with " << n_effectors_ << " effectors, but received query with 1 effector!" << std::endl;

	std::vector<tf::Transform> effector_query_poses;
	effector_query_poses.push_back(effector_query_pose);
	return mapEffectorPosesToObject(effector_query_poses, obj_pose);
}

double ManipulationTransforms::mapEffectorPosesToObject(const tf::Transform &effector1_query_pose, const tf::Transform &effector2_query_pose, tf::Transform &obj_pose)
{
	if (n_effectors_ != 1)
		std::cout << "Warning: initialized with " << n_effectors_ << " effectors, but received query with 2 effector!" << std::endl;

	std::vector<tf::Transform> effector_query_poses;
	effector_query_poses.push_back(effector1_query_pose);
	effector_query_poses.push_back(effector2_query_pose);
	return mapEffectorPosesToObject(effector_query_poses, obj_pose);
}

/**
 * @brief Computes r_effector_pose and l_effector_pose from obj pose by applying the backward transforms
 */
double ManipulationTransforms::mapObjectPoseToEffectors(const tf::Transform &obj_query_pose, std::vector<tf::Transform> &effector_poses)
{
	// Check if caller has allocated transforms, and scold them if not
	if (n_effectors_ != effector_poses.size()){
		std::cout << "Warning: initialized with " << n_effectors_ << " effectors, but received query with " << effector_poses.size() << std::endl;
		effector_poses.resize(n_effectors_);
	}

	for (unsigned int i = 0; i < n_effectors_; ++i)
		effector_poses[i] = obj_query_pose * effector_T_obj_[i];

	return 0;
}

double ManipulationTransforms::mapObjectPoseToEffectors(const tf::Transform &obj_query_pose, tf::Transform &effector_pose)
{
	std::vector<tf::Transform> effector_poses;
	if (n_effectors_ != 1)
		std::cout << "Warning: initialized with " << n_effectors_ << " effectors, but received query with 1 effector!" << std::endl;

	effector_poses.resize(n_effectors_);

	mapObjectPoseToEffectors(obj_query_pose, effector_poses);
	effector_pose = effector_poses[0];
	return 0;
}

double ManipulationTransforms::mapObjectPoseToEffectors(const tf::Transform &obj_query_pose, tf::Transform &effector1_pose, tf::Transform &effector2_pose)
{
	std::vector<tf::Transform> effector_poses;
	if (n_effectors_ != 2)
		std::cout << "Warning: initialized with " << n_effectors_ << " effectors, but received query with 2 effectors!" << std::endl;

	effector_poses.resize(n_effectors_);

	mapObjectPoseToEffectors(obj_query_pose, effector_poses);
	effector1_pose = effector_poses[0];
	effector2_pose = effector_poses[1];
	return 0;
}
/**
 * @brief Sets twist of obj given effector twists
 */
double ManipulationTransforms::mapEffectorTwistsToObject(
		const std::vector<KDL::Twist> &effector_query_twists,
		KDL::Twist &obj_twist,
		bool from_ref)
{
	//TODO: check this (i think it's right)
	// Check if caller has allocated transforms, and scold them if not
	if (n_effectors_ != effector_query_twists.size())
		std::cout << "Warning: initialized with " << n_effectors_ << " effectors, but received query with " << effector_query_twists.size() << std::endl;

	std::vector<KDL::Twist> twists(n_effectors_);
	KDL::Twist twist_sum;
	twist_sum.Zero();

	// Compute the object twist according to each effector
	for (unsigned int i = 0; i < n_effectors_; ++i){
		KDL::Frame f_eff_to_obj; // object to effector
		tf::TransformTFToKDL(obj_T_effectors_[i], f_eff_to_obj);

		if (from_ref){
			KDL::Frame f_effector;   // effector (to reference frame)
			tf::TransformTFToKDL(effector_T_ref_[i], f_effector);
			//TODO: look at the logic behind using the Inverse here, since i didn't think i'd need it in the
			// forward version but i did...
			twists[i] = f_eff_to_obj * (f_effector.M.Inverse() *  effector_query_twists[i]); // rotate query twists to effector frame
		}
		else {
			twists[i] = f_eff_to_obj * effector_query_twists[i]; // the adjoint transformation, courtesy of KDL's frame*twist operator
			twist_sum += twists[i];
		}
	}

	// Take simple unweighted average
	obj_twist = twist_sum / n_effectors_;

	// Compute SSD over all twists from respective means
	double SSD = 0;
	for (unsigned int i = 0; i < n_effectors_; ++i){
		for (unsigned int j = 0; j < 6; ++j) {
			SSD += (obj_twist[j] - twists[i][j]) * (obj_twist[j] - twists[i][j]);
		}
	}

	//return flat variance
	return SSD / 6*n_effectors_;
}

double ManipulationTransforms::mapEffectorTwistsToObject(const KDL::Twist &effector_query_twist, KDL::Twist &obj_twist, bool from_ref)
{
	std::vector<KDL::Twist> effector_query_twists;
	if (n_effectors_ != 1)
		std::cout << "Warning: initialized with " << n_effectors_ << " effectors, but received query with 1 effector!" << std::endl;

	effector_query_twists.resize(n_effectors_);
	effector_query_twists[0] = effector_query_twist;
	return mapEffectorTwistsToObject(effector_query_twists, obj_twist, from_ref);
}

double ManipulationTransforms::mapEffectorTwistsToObject(const KDL::Twist &effector1_query_twist, const KDL::Twist &effector2_query_twist, KDL::Twist &obj_twist, bool from_ref)
{
	std::vector<KDL::Twist> effector_query_twists;
	if (n_effectors_ != 2)
		std::cout << "Warning: initialized with " << n_effectors_ << " effectors, but received query with 2 effectors!" << std::endl;

	effector_query_twists.resize(n_effectors_);
	effector_query_twists[0] = effector1_query_twist;
	effector_query_twists[1] = effector2_query_twist;
	return mapEffectorTwistsToObject(effector_query_twists, obj_twist, from_ref);
}

/**
 * @brief Sets effector twists given the provided object twist
 */
double ManipulationTransforms::mapObjectTwistToEffectors(
		const KDL::Twist &obj_query_twist,
		std::vector<KDL::Twist> &effector_twists,
		bool to_ref)
{
	// Check if caller has allocated transforms, and scold them if not
	if (n_effectors_ != effector_twists.size()){
		std::cout << "Warning: initialized with " << n_effectors_ << " effectors, but received query for " << effector_twists.size() << std::endl;
		effector_twists.resize(n_effectors_);
	}

	for (unsigned int i = 0; i < n_effectors_; ++i){
		KDL::Frame f_obj_to_eff; // object to effector
		tf::TransformTFToKDL(effector_T_obj_[i], f_obj_to_eff);

		if (to_ref){
			KDL::Frame f_effector;   // effector (to reference frame)
			tf::TransformTFToKDL(effector_T_ref_[i], f_effector);
			/*
			 * TODO: figure out why I need the Inverse here.  According to my notion of the frames, i *should* need
			 * just f_effector: the first transform in parens maps from obj to each gripper, which we'd then need
			 * to rotate to the reference frame.  The transform that maps a point in an effector frame to the reference
			 * frame is f_effector.  f_effector.M.Inverse should go the other way, no?
			 */
			effector_twists[i] = f_effector.M.Inverse() * (f_obj_to_eff * obj_query_twist); // rotate result to reference frame
		}
		else
			effector_twists[i] = f_obj_to_eff * obj_query_twist; // the adjoint transformation, courtesy of KDL's frame*twist operator
	}

	return 0;
}

double ManipulationTransforms::mapObjectTwistToEffectors(const KDL::Twist &obj_query_twist, KDL::Twist &effector_twist, bool to_ref)
{
	std::vector<KDL::Twist> effector_twists;
	if (n_effectors_ != 1)
		std::cout << "Warning: initialized with " << n_effectors_ << " effectors, but received query with 1 effector!" << std::endl;

	effector_twists.resize(n_effectors_);
	mapObjectTwistToEffectors(obj_query_twist, effector_twists, to_ref);
	effector_twist = effector_twists[0];
	return 0;
}

double ManipulationTransforms::mapObjectTwistToEffectors(const KDL::Twist &obj_query_twist, KDL::Twist &effector1_twist, KDL::Twist &effector2_twist, bool to_ref)
{
	std::vector<KDL::Twist> effector_twists;
	if (n_effectors_ != 2)
		std::cout << "Warning: initialized with " << n_effectors_ << " effectors, but received query with 2 effectors!" << std::endl;

	effector_twists.resize(n_effectors_);
	mapObjectTwistToEffectors(obj_query_twist, effector_twists, to_ref);
	effector1_twist = effector_twists[0];
	effector2_twist = effector_twists[1];
	return 0;
}

/**
 * @brief Sets wrench of object given effector wrenches
 */
double ManipulationTransforms::mapEffectorWrenchesToObject(const std::vector<KDL::Wrench> &effector_query_wrenches, KDL::Wrench &obj_wrench)
{
	//TODO: check this (never used)
	// Check if caller has allocated transforms, and scold them if not
	if (n_effectors_ != effector_query_wrenches.size())
		std::cout << "Warning: initialized with " << n_effectors_ << " effectors, but received query with " << effector_query_wrenches.size() << std::endl;

	std::vector<KDL::Wrench> wrenches(n_effectors_);
	obj_wrench.Zero();

	// Compute the wrench at the object according to each effector
	for (unsigned int i = 0; i < n_effectors_; ++i){
		KDL::Frame f;
		tf::TransformTFToKDL(obj_T_effectors_[i], f);
		wrenches[i] = f * effector_query_wrenches[i];
		obj_wrench += wrenches[i];
	}

	//return nothing interesting, since we can't assess how well effectors agree
	return 0;
}

/**
 * @brief Sets effector wrenches given the provided object wrench
 */
double ManipulationTransforms::mapObjectWrenchToEffectors(const KDL::Wrench &obj_query_wrench, std::vector<KDL::Wrench> &effector_wrenches)
{
	//TODO: check this (never used)
	// Check if caller has allocated transforms, and scold them if not
	if (n_effectors_ != effector_wrenches.size()){
		std::cout << "Warning: initialized with " << n_effectors_ << " effectors, but received query for " << effector_wrenches.size() << std::endl;
		effector_wrenches.resize(n_effectors_);
	}

	for (unsigned int i = 0; i < n_effectors_; ++i){
		KDL::Frame f;
		tf::TransformTFToKDL(effector_T_obj_[i], f);
		effector_wrenches[i] = (f * obj_query_wrench)/n_effectors_; // divide wrench evenly across each effector
	}

	return 0;
}
