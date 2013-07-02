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
 * Filter a point cloud to remove points that lie within some polygon
 *
 * \author Bhaskara Marthi
 */

#ifndef CART_STATE_ESTIMATOR_POLYGON_POINT_CLOUD_FILTER_H
#define CART_STATE_ESTIMATOR_POLYGON_POINT_CLOUD_FILTER_H

#include <filters/filter_base.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <ros/ros.h>

namespace cart_state_estimator
{

/**
 * Filter that takes as parameters a 2d polygon and a frame
 * Given a point cloud, for each point it transforms to the frame and
 * drops the z coordinate, then checks if it lies in the polygon.
 * If it does, skip it, else keep the (untransformed) point.
 */
class PolygonPointCloudFilter : public filters::FilterBase<sensor_msgs::PointCloud>
{
public:
  
  PolygonPointCloudFilter () {}
  bool configure();
  bool update (const sensor_msgs::PointCloud& input_scan, sensor_msgs::PointCloud& filtered_scan);
  bool inPolygon (const geometry_msgs::Point32& p) const;
  
private:

  tf::TransformListener tf_;
  ros::NodeHandle nh_;
  ros::Publisher filter_polygon_pub_;
  std::vector<geometry_msgs::Point32> poly_;
  std::string frame_;

};
    

} // namespace

#endif // include guard
