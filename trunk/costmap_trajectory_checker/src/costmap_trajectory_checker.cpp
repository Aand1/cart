/*********************************************************************
*
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
*
* Author: Jonathan Scholz
*********************************************************************/

#include <costmap_trajectory_checker/costmap_trajectory_checker.h>
#include "nav_msgs/Path.h"
#include <ros/console.h>
#include <sys/time.h>

using namespace std;
using namespace costmap_2d;

CostmapTrajectoryChecker::CostmapTrajectoryChecker() :
		world_model_(NULL), costmap_ros_(NULL), initialized_(false) {
}

CostmapTrajectoryChecker::CostmapTrajectoryChecker(Costmap2DROS* costmap_ros, std::string topic) :
	world_model_(NULL), costmap_ros_(NULL), initialized_(false)
{
	initialize(costmap_ros, topic);
}

CostmapTrajectoryChecker::CostmapTrajectoryChecker(
		costmap_2d::Costmap2DROS* costmap_ros,
		std::string frame_id,
		std::vector<geometry_msgs::Point> footprint,
		std::string topic) :
		world_model_(NULL), costmap_ros_(NULL), initialized_(false)
{
	initialize(costmap_ros, topic);
	robot_frame_ = frame_id;
	footprint_spec_ = footprint;
}

CostmapTrajectoryChecker::CostmapTrajectoryChecker(const CostmapTrajectoryChecker &checker) : world_model_(NULL), costmap_ros_(NULL), initialized_(false)
{
	if(this == &checker)
	      return;

	initialize(checker.costmap_ros_, checker.traj_topic_name_);
	robot_frame_ = checker.robot_frame_;
	footprint_spec_ = checker.footprint_spec_;
}

CostmapTrajectoryChecker& CostmapTrajectoryChecker::operator=(const CostmapTrajectoryChecker &checker)
{
	if(this == &checker)
	      return *this;

	initialize(checker.costmap_ros_, checker.traj_topic_name_);
	robot_frame_ = checker.robot_frame_;
	footprint_spec_ = checker.footprint_spec_;
	return *this;
}

CostmapTrajectoryChecker::~CostmapTrajectoryChecker(){}

void CostmapTrajectoryChecker::initialize(Costmap2DROS* costmap_ros, std::string topic)
{
	if (!initialized_) {
		costmap_ros_ = costmap_ros;

		//initialize the copy of the costmap the controller will use
		costmap_ros_->getCostmapCopy(costmap_);

		robot_frame_ = costmap_ros_->getBaseFrameID();
		global_frame_ = costmap_ros_->getGlobalFrameID();
		footprint_spec_ = costmap_ros_->getRobotFootprint();

		//we'll get the parameters for the robot radius from the costmap we're associated with
		inscribed_radius_ = costmap_ros_->getInscribedRadius();
		circumscribed_radius_ = costmap_ros_->getCircumscribedRadius();
		inflation_radius_ = costmap_ros_->getInflationRadius();

		world_model_ = new base_local_planner::CostmapModel(costmap_);

		setPubTopic(topic);

		initialized_ = true;
	}
	else
		ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
}

void CostmapTrajectoryChecker::setFootprint(const std::vector<
		geometry_msgs::Point> &footprint) {
	footprint_spec_ = footprint;
}

void CostmapTrajectoryChecker::setFootprint(double length, double width, double x_offset, double y_offset)
{
	footprint_spec_.resize(4);
	footprint_spec_[0].x = -x_offset;
	footprint_spec_[0].y = -y_offset;
	footprint_spec_[1].x = -x_offset + length;
	footprint_spec_[1].y = -y_offset;
	footprint_spec_[2].x = -x_offset + length;
	footprint_spec_[2].y = -y_offset + width;
	footprint_spec_[3].x = -x_offset;
	footprint_spec_[3].y = -y_offset + width;
}

std::vector<geometry_msgs::Point> CostmapTrajectoryChecker::getFootprint()
{
	return footprint_spec_;
}

void CostmapTrajectoryChecker::setGlobalFrameID(std::string frame_id)
{
	global_frame_ = frame_id;
}


void CostmapTrajectoryChecker::setRobotFrameID(std::string frame_id)
{
	robot_frame_ = frame_id;
}

std::string CostmapTrajectoryChecker::getGlobalFrameID()
{
	return global_frame_;
}

std::string CostmapTrajectoryChecker::getRobotFrameID()
{
	return robot_frame_;
}

void CostmapTrajectoryChecker::setPubTopic(std::string topic)
{
	traj_topic_name_ = topic;
	if (traj_topic_name_ != "")
		traj_pub_ = nh_.advertise<nav_msgs::Path> (traj_topic_name_, 1);
}

std::string CostmapTrajectoryChecker::getPubTopic()
{
	return traj_topic_name_;
}

void CostmapTrajectoryChecker::generateTrajectory(
		const geometry_msgs::Pose2D &start_pose,
		const geometry_msgs::Twist &vel, unsigned int num_steps, double dt,
		std::vector<geometry_msgs::Pose2D>& traj) {

	traj.resize(0);
	traj.push_back(start_pose);
	geometry_msgs::Pose2D cur_pose = start_pose;

	for (unsigned int i = 1; i < num_steps; ++i) {
		integratePose(vel, dt, cur_pose);
                ROS_DEBUG("dt: %f, cur_pose: %f %f %f",dt,cur_pose.x,cur_pose.y,cur_pose.theta);
		traj.push_back(cur_pose);
	}
}

double CostmapTrajectoryChecker::checkTrajectory(const std::vector<geometry_msgs::Pose2D>& traj, bool update_map, bool clear_footprint)
{
    //make sure to update the costmap we'll use for this cycle
	if (update_map)
		costmap_ros_->getCostmapCopy(costmap_);

	if (clear_footprint) {
		vector<geometry_msgs::Point> oriented_footprint;
		getOrientedFootprint(traj[0], oriented_footprint);

		//we also want to clear the robot footprint from the costmap we're using
		if(!costmap_.setConvexPolygonCost(oriented_footprint, costmap_2d::FREE_SPACE)) {
			ROS_ERROR("Could not clear robot footprint: ");
			for (uint i = 0; i < oriented_footprint.size(); ++i)
				ROS_ERROR("oriented_footprint: %.2lf %.2lf %.2lf", oriented_footprint[i].x, oriented_footprint[i].y, oriented_footprint[i].z);
			return false;
		}
	}

	double cost = -1.0;
	for (unsigned int i = 0; i < traj.size(); ++i) {
		double newcost = footprintCost(traj[i]);
		// get footprint cost
		cost = newcost < 0 ? -1.0 : std::max(cost, newcost);
	}

	if (traj_topic_name_ != "")
		publishTrajectory(traj);

	ROS_DEBUG("trajectory cost = %.4lf", cost);
	return cost;
}

bool CostmapTrajectoryChecker::clearFootprint(const geometry_msgs::Pose2D &traj, bool update_map)
{
    //make sure to update the costmap we'll use for this cycle
  if(update_map)
    costmap_ros_->getCostmapCopy(costmap_);
  
  vector<geometry_msgs::Point> oriented_footprint;
  getOrientedFootprint(traj, oriented_footprint);
  
  //we also want to clear the robot footprint from the costmap we're using
  if(!costmap_.setConvexPolygonCost(oriented_footprint, costmap_2d::FREE_SPACE)) 
  {
      ROS_ERROR("Could not clear robot footprint: ");
      for (uint i = 0; i < oriented_footprint.size(); ++i)
	ROS_ERROR("oriented_footprint: %.2lf %.2lf %.2lf", oriented_footprint[i].x, oriented_footprint[i].y, oriented_footprint[i].z);
      return false;	
  }
  return true;
}

bool CostmapTrajectoryChecker::clearFootprint(bool update_map)
{
    //make sure to update the costmap we'll use for this cycle
  if(update_map)
    costmap_ros_->getCostmapCopy(costmap_);
  
  vector<geometry_msgs::Point> oriented_footprint;
  geometry_msgs::Pose2D pose;
  getRobotPose(pose);
  getOrientedFootprint(pose, oriented_footprint);
  
  //we also want to clear the robot footprint from the costmap we're using
  if(!costmap_.setConvexPolygonCost(oriented_footprint, costmap_2d::FREE_SPACE)) 
  {
      ROS_ERROR("Could not clear robot footprint: ");
      for (uint i = 0; i < oriented_footprint.size(); ++i)
	ROS_ERROR("oriented_footprint: %.2lf %.2lf %.2lf", oriented_footprint[i].x, oriented_footprint[i].y, oriented_footprint[i].z);
      return false;	
  }
  return true;
}


double CostmapTrajectoryChecker::checkTrajectoryMonotonic(const std::vector<geometry_msgs::Pose2D>& traj, 
							  bool update_map, 
							  bool clear_footprint, 
							  unsigned int num_consec_points_in_collision)
{
    //make sure to update the costmap we'll use for this cycle
  if (update_map)
    costmap_ros_->getCostmapCopy(costmap_);
  
  if (clear_footprint) {
    vector<geometry_msgs::Point> oriented_footprint;
    getOrientedFootprint(traj[0], oriented_footprint);
    
    //we also want to clear the robot footprint from the costmap we're using
    if(!costmap_.setConvexPolygonCost(oriented_footprint, costmap_2d::FREE_SPACE)) {
      ROS_ERROR("Could not clear robot footprint: ");
      for (uint i = 0; i < oriented_footprint.size(); ++i)
	ROS_ERROR("oriented_footprint: %.2lf %.2lf %.2lf", oriented_footprint[i].x, oriented_footprint[i].y, oriented_footprint[i].z);
      return false;
    }
  }
  
  double cost = -1.0;
  double lastcost = 0.0;
  unsigned int num_lethal_cost = 0;
  
  for (unsigned int i = 0; i < traj.size(); ++i) {
    //    ROS_INFO("Checking pose: %d :: %f %f %f",i,traj[i].x,traj[i].y,traj[i].theta);

    double newcost = footprintCost(traj[i]);
    cost = std::max(newcost,cost);
    if(newcost < 0)
      newcost = costmap_2d::LETHAL_OBSTACLE;
    
    if(i > 0 && i < (num_consec_points_in_collision+1)&& lastcost==costmap_2d::LETHAL_OBSTACLE && newcost == costmap_2d::LETHAL_OBSTACLE)
      num_lethal_cost++;
    
    if(i > 0 && lastcost < costmap_2d::LETHAL_OBSTACLE && (newcost==costmap_2d::LETHAL_OBSTACLE) )
    {
      ROS_INFO("Twist points: %d %d are leading us into collision",i-1,i);
      return -1.0;
    }

    if(num_lethal_cost > num_consec_points_in_collision || num_lethal_cost == traj.size()-1)
      return -1.0;
    lastcost = newcost;
  }
  
  if (traj_topic_name_ != "")
    publishTrajectory(traj);
  
  ROS_DEBUG("trajectory cost = %.4lf", cost);
  return cost;
}

double CostmapTrajectoryChecker::checkTwistMonotonic(const geometry_msgs::Twist &vel, unsigned int num_steps, double dt, bool update_map, bool clear_footprint)
{
	geometry_msgs::Pose2D pose;
	getRobotPose(pose);
	std::vector<geometry_msgs::Pose2D> traj;
	generateTrajectory(pose, vel, num_steps, dt, traj);

	return checkTrajectoryMonotonic(traj, update_map, clear_footprint,5);
}


double CostmapTrajectoryChecker::checkTwist(const geometry_msgs::Twist &vel, unsigned int num_steps, double dt, bool update_map, bool clear_footprint)
{
	geometry_msgs::Pose2D pose;
	getRobotPose(pose);
	std::vector<geometry_msgs::Pose2D> traj;
	generateTrajectory(pose, vel, num_steps, dt, traj);

	return checkTrajectory(traj, update_map, clear_footprint);
}

void CostmapTrajectoryChecker::getOrientedFootprint(const geometry_msgs::Pose2D &pose, std::vector<geometry_msgs::Point> &oriented_footprint)
{
	oriented_footprint.resize(0);
	double cos_th = cos(pose.theta);
	double sin_th = sin(pose.theta);
	for(unsigned int i = 0; i < footprint_spec_.size(); ++i){
		geometry_msgs::Point new_pt;
		new_pt.x = pose.x + (footprint_spec_[i].x * cos_th - footprint_spec_[i].y * sin_th);
		new_pt.y = pose.y + (footprint_spec_[i].x * sin_th + footprint_spec_[i].y * cos_th);
		oriented_footprint.push_back(new_pt);
	}
}

//we need to take the footprint of the robot into account when we calculate cost to obstacles
double CostmapTrajectoryChecker::footprintCost(const geometry_msgs::Pose2D &pose)
{
	//build the oriented footprint
	vector<geometry_msgs::Point> oriented_footprint;
	getOrientedFootprint(pose, oriented_footprint);

	geometry_msgs::Point robot_position;
	robot_position.x = pose.x;
	robot_position.y = pose.y;

	//ROS_WARN("costmap checker:");
	//ROS_WARN("robot_position: %.2lf %.2lf", robot_position.x, robot_position.y);
	//for (uint i = 0; i < oriented_footprint.size(); ++i)
	//	ROS_WARN("oriented_footprint: %.2lf %.2lf %.2lf", oriented_footprint[i].x, oriented_footprint[i].y, oriented_footprint[i].z);
	//ROS_WARN("inscribed radius = %.2lf, circumscribed_radius_ = %.2lf", inscribed_radius_, circumscribed_radius_);
	//ROS_WARN("ctc footprint cost = %.4lf", world_model_->footprintCost(robot_position, oriented_footprint, inscribed_radius_, circumscribed_radius_));
	return world_model_->footprintCost(robot_position, oriented_footprint, inscribed_radius_, circumscribed_radius_);
}

void CostmapTrajectoryChecker::poseTFToPose2D(const tf::Pose &pose_tf, geometry_msgs::Pose2D &pose_2d)
{
	btScalar pitch, roll, yaw;
	pose_tf.getBasis().getRPY(roll, pitch, yaw);

	pose_2d.x = pose_tf.getOrigin()[0];
	pose_2d.y = pose_tf.getOrigin()[1];
	pose_2d.theta = yaw;

	if (pose_tf.getOrigin()[2]>0.1)
		ROS_WARN("Large linear Z component lost in conversion to 2D pose (%.2f)", pose_tf.getOrigin()[2]);
	if (roll>0.1)
		ROS_WARN("Large roll component lost in conversion to 2D pose (%.2f)", roll);
	if (pitch>0.1)
		ROS_WARN("Large roll component lost in conversion to 2D pose (%.2f)", pitch);
}

void CostmapTrajectoryChecker::pose2DToTF(const geometry_msgs::Pose2D pose_2d, tf::Pose &pose_tf)
{
	tf::Quaternion q;
	q.setRPY(0, 0, pose_2d.theta);
	pose_tf.setRotation(q);
	pose_tf.setOrigin(btVector3(pose_2d.x, pose_2d.y, 0));
}

void CostmapTrajectoryChecker::pose2DToPose(const geometry_msgs::Pose2D pose_2d, geometry_msgs::Pose &pose)
{
	pose.position.x = pose_2d.x;
	pose.position.y = pose_2d.y;
	pose.position.z = 0;
	pose.orientation = tf::createQuaternionMsgFromYaw(pose_2d.theta);
}

void CostmapTrajectoryChecker::poseToPose2D(const geometry_msgs::Pose pose, geometry_msgs::Pose2D &pose_2d)
{
	pose_2d.x = pose.position.x;
	pose_2d.y = pose.position.y;
	pose_2d.theta = tf::getYaw(pose.orientation);
}

bool CostmapTrajectoryChecker::getRobotPose(geometry_msgs::Pose2D &pose_2d) const
{
	tf::StampedTransform pose_tf;

	//get the global pose of the robot
	try {
		tl_.lookupTransform(global_frame_, robot_frame_, ros::Time(), pose_tf);
	} catch (tf::LookupException& ex) {
		ROS_ERROR("No Transform available Error: %s\n", ex.what());
		return false;
	} catch (tf::ConnectivityException& ex) {
		ROS_ERROR("Connectivity Error: %s\n", ex.what());
		return false;
	} catch (tf::ExtrapolationException& ex) {
		ROS_ERROR("Extrapolation Error: %s\n", ex.what());
		return false;
	}

	poseTFToPose2D(pose_tf, pose_2d);

  return true;
}

void CostmapTrajectoryChecker::integratePose(const geometry_msgs::Twist &vel, double dt,
		geometry_msgs::Pose2D &pose)
{
	pose.x += (vel.linear.x * cos(pose.theta) + vel.linear.y * cos(M_PI_2 + pose.theta)) * dt;
	pose.y += (vel.linear.x * sin(pose.theta) + vel.linear.y * sin(M_PI_2 + pose.theta)) * dt;
	pose.theta += vel.angular.z * dt;
}

void CostmapTrajectoryChecker::publishTrajectory(const std::vector<geometry_msgs::Pose2D>& path)
{
  //given an empty path we won't do anything
  if(path.empty())
    return;

  //create a path message
  nav_msgs::Path gui_path;
  gui_path.poses.resize(path.size());
  gui_path.header.frame_id = global_frame_;
  gui_path.header.stamp = ros::Time::now();

  // Extract the plan in world co-ordinates, we assume the path is all in the same frame
  for(unsigned int i=0; i < path.size(); i++){
	  gui_path.poses[i].header.frame_id = global_frame_;
	  gui_path.poses[i].header.stamp = ros::Time::now();
	  pose2DToPose(path[i], gui_path.poses[i].pose);
  }

  traj_pub_.publish(gui_path);
}
