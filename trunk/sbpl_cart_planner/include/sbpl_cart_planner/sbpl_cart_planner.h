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
* Author: Mike Phillips, Sachin Chitta
*********************************************************************/

#ifndef SBPL_CART_PLANNER_H
#define SBPL_CART_PLANNER_H

#include <iostream>
#include <vector>

using namespace std;

/** ROS **/
#include <ros/ros.h>

// Costmap used for the map representation
#include <costmap_2d/costmap_2d_ros.h>

// sbpl headers
#include <sbpl/headers.h>

//global representation
#include <nav_core/base_global_planner.h>

#include <sbpl_cart_planner/environment_cart_planner.h>
#include <visualization_msgs/MarkerArray.h>

class SBPLCartPlanner : public nav_core::BaseGlobalPlanner{
public:
  
  /**
   * @brief  Default constructor for the NavFnROS object
   */
  SBPLCartPlanner();

  
  /**
   * @brief  Constructor for the SBPLCartPlanner object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  SBPLCartPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);


  /**
   * @brief  Initialization function for the SBPLCartPlanner object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  virtual void initialize(std::string name, 
                          costmap_2d::Costmap2DROS* costmap_ros);
  
  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose 
   * @param goal The goal pose 
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  virtual bool makePlan(const geometry_msgs::PoseStamped& start, 
                        const geometry_msgs::PoseStamped& goal, 
                        std::vector<geometry_msgs::PoseStamped>& plan);

  virtual ~SBPLCartPlanner(){};

private:
  bool initialized_;

  SBPLPlanner* planner_;
  EnvironmentNAVXYTHETACARTLAT* env_;
  
  std::string planner_type_; /**< sbpl method to use for planning.  choices are ARAPlanner and ADPlanner */

  double allocated_time_; /**< amount of time allowed for search */
  double initial_epsilon_; /**< initial epsilon for beginning the anytime search */

  std::string environment_type_; /** what type of environment in which to plan.  choices are 2D and XYThetaLattice. */ 
  std::string cost_map_topic_; /** what topic is being used for the costmap topic */

  bool forward_search_; /** whether to use forward or backward search */
  std::string primitive_filename_; /** where to find the motion primitives for the current robot */
  int force_scratch_limit_; /** the number of cells that have to be changed in the costmap to force the planner to plan from scratch even if its an incremental planner */


  costmap_2d::Costmap2DROS* costmap_ros_; /**< manages the cost map for us */
  costmap_2d::Costmap2D cost_map_;        /**< local copy of the costmap underlying cost_map_ros_ */

  ros::Publisher plan_pub_,sbpl_plan_pub_,sbpl_plan_footprint_pub_,sbpl_robot_cart_plan_pub_, stats_publisher_;
  
  std::vector<geometry_msgs::Point> footprint_;
  std::vector<geometry_msgs::Point> loadRobotFootprint(ros::NodeHandle node);

  double sign(double x);
  sbpl_2Dpt_t cart_offset_, cart_cp_offset_;

  void convertPathToMarkerArray(const std::vector<EnvNAVXYTHETACARTLAT3Dpt_t> &sbpl_path,
                                const std::string &path_frame_id,
                                visualization_msgs::MarkerArray &ma);

  std::vector<geometry_msgs::Point> cart_footprint_,robot_footprint_;

  void transformFootprintToEdges(const geometry_msgs::Pose &robot_pose,
                                 const std::vector<geometry_msgs::Point> &footprint,
                                 std::vector<geometry_msgs::Point> &out_footprint);

  void getFootprintList(const std::vector<EnvNAVXYTHETACARTLAT3Dpt_t> &sbpl_path,
                        const std::string &path_frame_id,
                        visualization_msgs::MarkerArray &ma);
  geometry_msgs::Pose getGlobalCartPose(const EnvNAVXYTHETACARTLAT3Dpt_t& sbpl_pose);
  geometry_msgs::Pose getLocalCartPose(const EnvNAVXYTHETACARTLAT3Dpt_t& sbpl_pose);
  geometry_msgs::Pose getLocalCartControlFramePose(const EnvNAVXYTHETACARTLAT3Dpt_t& sbpl_pose);

  bool clearFootprint(const geometry_msgs::Pose &robot_pose, 
                      const std::vector<geometry_msgs::Point> &footprint);
  
  void getOrientedFootprint(const geometry_msgs::Pose &robot_pose,
                            const std::vector<geometry_msgs::Point> &footprint,
                            std::vector<geometry_msgs::Point> &oriented_footprint);

  geometry_msgs::Pose getGlobalCartPose(const geometry_msgs::Pose &robot_pose, const double &cart_angle);

  unsigned char costMapCostToSBPLCost(unsigned char newcost);

  unsigned char sbpl_cost_multiplier_;
  unsigned char lethal_obstacle_;
  unsigned char inscribed_inflated_obstacle_;
  unsigned int num_sbpl_markers_;
  int visualizer_skip_poses_;
};

#endif

