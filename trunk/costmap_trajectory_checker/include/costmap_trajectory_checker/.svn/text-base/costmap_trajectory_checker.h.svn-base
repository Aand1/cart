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
 * \file costmap_trajectory_checker.h
 *
 *  \date August 17, 2010
 *  \author Jonathan Scholz
 */

#ifndef COSTMAP_TRAJECTORY_CHECKER_H_
#define COSTMAP_TRAJECTORY_CHECKER_H_

#include <boost/thread.hpp>

#include <string>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/Pose2D.h>


/**
 * @class CostmapTrajectoryChecker
 *
 * Uses Pose2D rather than Pose messages because (a) it ensures proper input for our robots which can't fly, and (b)n
 * because it's slightly more efficient when integrating velocity to not have to convert back and forth from a
 * quaternion (though this is moot when publishing)
 */
class CostmapTrajectoryChecker {
public:
	/**
	 * @brief  Default constructor
	 */
	CostmapTrajectoryChecker();

	/**
	 *
	 * @param costmap_ros
	 * @param frame_id
	 * @param footprint
	 * @return
	 */
	CostmapTrajectoryChecker(costmap_2d::Costmap2DROS* costmap_ros,
			std::string frame_id,
			std::vector<geometry_msgs::Point> footprint,
			std::string topic = "");

	/**
	 * Barebones constructor.  Gets footprint and frame_id from the costmap
	 * @param costmap_ros
	 * @return
	 */
	CostmapTrajectoryChecker(costmap_2d::Costmap2DROS* costmap_ros, std::string topic = "");

	/// Copy constructor
	CostmapTrajectoryChecker(const CostmapTrajectoryChecker &checker);
	CostmapTrajectoryChecker& operator=(const CostmapTrajectoryChecker& checker);

	/**
	 * Initialize
	 * @param costmap_ros
	 */
	void initialize(costmap_2d::Costmap2DROS* costmap_ros, std::string topic = "");


	/**
	 * @brief  Destructor
	 */
	~CostmapTrajectoryChecker();

	/**
	 * Set footprint spec of robot
	 * @param footprint
	 */
	void setFootprint(const std::vector<geometry_msgs::Point> &footprint);

	/**
	 * Shortcut for generating rectangular footprint from box dimensions
	 * @param length
	 * @param width
	 * @param x_offset
	 * @param y_offset
	 */
	void setFootprint(double length, double width, double x_offset, double y_offset);

	/** Returns copy of footprint_spec_ */
	std::vector<geometry_msgs::Point> getFootprint();

	/// Getters and setters for frame_ids
	void setGlobalFrameID(std::string frame_id);
	void setRobotFrameID(std::string frame_id);
	std::string getGlobalFrameID();
	std::string getRobotFrameID();

	/// Getter and setter for visualization topic
	void setPubTopic(std::string topic);
	std::string getPubTopic();

	/**
	 * @brief Generates a Pose2D trajectory by integrating provided velocity
	 * @param start_pose The starting position of the robot, in world (not costmap) coordinates
	 * @param vel
	 * @param traj
	 */
	static void generateTrajectory(const geometry_msgs::Pose2D &start_pose,
			const geometry_msgs::Twist &vel, unsigned int num_steps, double dt,
			std::vector<geometry_msgs::Pose2D>& traj);

	/**
	 * @brief  Check validity of a trajectory against the provided costmap
	 * @return The maximum cost on the trajectory, or -1 if LETHAL
	 */
	double checkTrajectory(const std::vector<geometry_msgs::Pose2D>& traj, bool update_map = false, bool clear_footprint = false);

	/**
	 * @brief  Check validity of a trajectory against the provided costmap
	 * @return The maximum cost on the trajectory, or -1 if LETHAL
	 */
	double checkTrajectoryMonotonic(const std::vector<geometry_msgs::Pose2D>& traj, bool update_map = false, bool clear_footprint = false, unsigned int num_consec_points_in_collision = 5);

	/**
	 * @brief A wrapper for checkTrajectory and generateTrajectory that generates and checks
	 * a trajectory using the provided twist
	 * @param vel A Twist describing the robot's current velocity
	 * @param num_steps The length of the trajectory to generate
	 * @param dt Time between samples along the trajectory
	 * @return The maximum cost on the trajectory, or -1 if LETHAL
	 */
	double checkTwist(const geometry_msgs::Twist &vel, unsigned int num_steps = 10, double dt = 0.2,
			bool update_map = false, bool clear_footprint = false);

	/**
	 * @brief A wrapper for checkTrajectory and generateTrajectory that generates and checks
	 * a trajectory using the provided twist
	 * @param vel A Twist describing the robot's current velocity
	 * @param num_steps The length of the trajectory to generate
	 * @param dt Time between samples along the trajectory
	 * @return The maximum cost on the trajectory, or -1 if LETHAL
	 */
	double checkTwistMonotonic(const geometry_msgs::Twist &vel, unsigned int num_steps = 10, double dt = 0.2,
			bool update_map = false, bool clear_footprint = false);
	
	bool clearFootprint(const geometry_msgs::Pose2D &traj, bool update_map);

	bool clearFootprint(bool update_map);

private:
	tf::TransformListener tl_; ///< @brief Used for looking up robot pose
	base_local_planner::WorldModel* world_model_; ///< @brief The world model to use for collision detection
	costmap_2d::Costmap2DROS* costmap_ros_; ///< @brief The ROS wrapper for the costmap the controller will use
	costmap_2d::Costmap2D costmap_; ///< @brief The costmap the controller will use

	ros::NodeHandle nh_; ///< @brief A local node handle
	ros::Publisher traj_pub_; ///< @brief A publisher for visualizing trajectories
	std::string traj_topic_name_; ///< @brief The topic name for publishing trajectories

	std::string robot_frame_; ///< @brief Used as the base frame id of the robot
	std::string global_frame_; ///< @brief Used as the global frame id
	double inscribed_radius_, circumscribed_radius_, inflation_radius_;
	std::vector<geometry_msgs::Point> footprint_spec_; ///< @brief The footprint specification of the robot
	bool initialized_;

	/**
	 * @brief Returns the max cost in the costmap at the provided position
	 * @param x_i
	 * @param y_i
	 * @param theta_i
	 * @return
	 */
	double footprintCost(const geometry_msgs::Pose2D &pose);

	/**
	 * @brief Retrieves the robot pose and converts to Pose2D representation
	 * @param pose_2d Will be set to the pose of the robot in the global frame
	 * @return True if the pose was set successfully, false otherwise
	 */
	bool getRobotPose(geometry_msgs::Pose2D &pose_2d) const;

	void publishTrajectory(const std::vector<geometry_msgs::Pose2D>& path);

	void getOrientedFootprint(const geometry_msgs::Pose2D &pose, std::vector<geometry_msgs::Point> &oriented_footprint);

	/**
	 * @brief Helper function for converting from tf::pose to geometry_msgs::Pose2D
	 * Discards linear Z, Roll, and Pitch dimensions
	 * @param pose_tf TF pose to convert
	 * @param pose_2d reference to converted 2D pose
	 */
	static void poseTFToPose2D(const tf::Pose &pose_tf, geometry_msgs::Pose2D &pose_2d);

	/**
	 * @brief Helper function for converting from geometry_msgs::Pose2D to tf::pose
	 * @param pose_2d
	 * @param pose_tf
	 */
	static void pose2DToTF(const geometry_msgs::Pose2D pose_2d, tf::Pose &pose_tf);

	/**
	 * @brief Helper function for converting from geometry_msgs::Pose2D to geometry_msgs::Pose
	 * @param pose_2d
	 * @param pose
	 */
	static void pose2DToPose(const geometry_msgs::Pose2D pose_2d, geometry_msgs::Pose &pose);

	/**
	 * @brief Helper function for converting from geometry_msgs::Pose to geometry_msgs::Pose2D
	 * @param pose_2d
	 * @param pose
	 */
	static void poseToPose2D(const geometry_msgs::Pose pose, geometry_msgs::Pose2D &pose_2d);

	/**
	 * Updates pose by applying provided twist for dt seconds
	 * @param vel
	 * @param dt
	 * @param pose
	 */
	static void integratePose(const geometry_msgs::Twist &vel, double dt,
			geometry_msgs::Pose2D &pose);
};

#endif // COSTMAP_TRAJECTORY_CHECKER_H_
