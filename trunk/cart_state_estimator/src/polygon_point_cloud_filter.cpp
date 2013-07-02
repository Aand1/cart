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

#include <cart_state_estimator/polygon_point_cloud_filter.h>
#include "pluginlib/class_list_macros.h"
#include <geometry_msgs/PolygonStamped.h>

PLUGINLIB_DECLARE_CLASS(cart_state_estimator, PolygonPointCloudFilter,
                        cart_state_estimator::PolygonPointCloudFilter, filters::FilterBase<sensor_msgs::PointCloud>)

namespace cart_state_estimator
{

using sensor_msgs::PointCloud;
using geometry_msgs::Point32;
using XmlRpc::XmlRpcValue;
using geometry_msgs::PolygonStamped;

bool PolygonPointCloudFilter::configure ()
{
  filter_polygon_pub_ = nh_.advertise<PolygonStamped>("filter_polygon", 10);
  XmlRpcValue polygon;
  if (!getParam("polygon", polygon)) {
    ROS_ERROR ("PolygonPointCloudFilter needs polygon to be set");
    return false;
  }
  else if ((polygon.getType() == XmlRpcValue::TypeArray) &&
           (polygon.size() >= 3)) {
    poly_.resize(polygon.size());
    for (int i=0; i< polygon.size(); i++) {
      XmlRpcValue pt = polygon[i];
      ROS_ASSERT_MSG (pt.getType() == XmlRpcValue::TypeArray && pt.size() == 2,
                      "Invalid xml rpc value for polygon parameter");
      poly_[i].x = (double) pt[0];
      poly_[i].y = (double) pt[1];
      ROS_INFO_STREAM ("Adding point " << poly_[i]);
    }
  }
  else {
    ROS_ERROR ("Invalid xml rpc value for polygon");
    return false;
  }
  if (!getParam("frame", frame_)) {
    ROS_ERROR ("PolygonPointCloudFilter needs frame to be set");      
  }
  return true;
}

bool PolygonPointCloudFilter::update (const PointCloud& input_scan, PointCloud& filtered_scan)
{
  if (&input_scan == &filtered_scan) {
    ROS_ERROR ("This filter does not currently support in place copying");
    return false;
  }
  PointCloud laser_cloud;

  try {
    tf_.waitForTransform(input_scan.header.frame_id, frame_, input_scan.header.stamp, ros::Duration(0.2));
    tf_.transformPointCloud(frame_, input_scan, laser_cloud);
  }
  catch (tf::TransformException& ex) {
    ROS_ERROR ("Transform unavailable %s", ex.what());
    return false;
  }

  const unsigned num_channels = input_scan.channels.size();

  filtered_scan.header = input_scan.header;
  filtered_scan.points.resize (input_scan.points.size());
  filtered_scan.channels.resize(num_channels);
  for (unsigned d=0; d<num_channels; d++) {
    filtered_scan.channels[d].values.resize (input_scan.points.size());
    filtered_scan.channels[d].name = input_scan.channels[d].name;
  }

  int num_pts = 0;
  for (unsigned i=0; i<laser_cloud.points.size(); i++) {
    ROS_DEBUG_STREAM_NAMED ("polygon_filter_internal", "Looking at point " << laser_cloud.points[i]);
    if (!inPolygon(laser_cloud.points[i])) {
      ROS_DEBUG_STREAM_NAMED ("polygon_filter_internal", "Not in polygon");
      filtered_scan.points[num_pts] = input_scan.points[i];
      for (unsigned d=0; d<num_channels; d++)
        filtered_scan.channels[d].values[num_pts] = input_scan.channels[d].values[i];
      num_pts++;
    }
    else
      ROS_DEBUG_STREAM_NAMED ("polygon_filter_internal", "In polygon");
  }
  ROS_DEBUG_NAMED ("polygon_filter", "In update, received a cloud of size %zu, of which we're keeping %d",
            laser_cloud.points.size(), num_pts);

  // Resize output vectors
  filtered_scan.points.resize(num_pts);
  for (unsigned d=0; d<num_channels; d++)
    filtered_scan.channels[d].values.resize(num_pts);

  // For debugging, visualize polygon
  {
    PolygonStamped poly;
    poly.polygon.points = poly_;
    poly.header.frame_id = frame_;
    poly.header.stamp = ros::Time::now();
    filter_polygon_pub_.publish(poly);
  }

  return true;
}


bool PolygonPointCloudFilter::inPolygon (const Point32& p) const
{
  bool c=false;
  const int num_vertices = poly_.size();
  for (int i=0; i<num_vertices; i++) {
    const int j = i ? i-1 : num_vertices-1;
    if ( ((poly_[i].y>p.y) != (poly_[j].y>p.y)) &&
         (p.x < (poly_[j].x-poly_[i].x) * (p.y-poly_[i].y) / (poly_[j].y-poly_[i].y) + poly_[i].x))
      c = !c;
  }
  return c;
}

  

} // namespace cart_state_estimator

