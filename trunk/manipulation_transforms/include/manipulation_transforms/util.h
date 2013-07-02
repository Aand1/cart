/*
 * util.h
 *
 *  Created on: Aug 6, 2010
 *      Author: jscholz
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <ros/ros.h>
#include <LinearMath/btTransform.h>
#include <geometry_msgs/PolygonStamped.h>
#include <string>
#include <boost/format.hpp>

/*********
 * UTILS *
 *********/
namespace manipulation_transforms_util {
template<typename T>
T getParam(const ros::NodeHandle &nh, const std::string &param) {
	T val;
	bool found = nh.getParam(param, val);
	ROS_ASSERT_MSG(found, "Did not find param %s in %s", param.c_str(), nh.getNamespace().c_str());

	return val;
}

tf::Transform readTransformParameter(const ros::NodeHandle &nh,
		const std::string &name) {
	using XmlRpc::XmlRpcValue;
	XmlRpcValue position = getParam<XmlRpcValue> (nh, name + "/position");
	XmlRpcValue orientation = getParam<XmlRpcValue> (nh, name + "/orientation");
	ROS_ASSERT(position.getType() == XmlRpc::XmlRpcValue::TypeArray &&
			orientation.getType() == XmlRpc::XmlRpcValue::TypeArray &&
			position.size() == 3 && orientation.size() == 4);
	const double x = position[0];
	const double y = position[1];
	const double z = position[2];
	const tf::Vector3 tr(x, y, z);
	const tf::Quaternion q(orientation[0], orientation[1], orientation[2],
			orientation[3]);

	{ // debugging
		tf::Transform trans(q, tr);
		geometry_msgs::Transform transform;
		tf::transformTFToMsg(trans, transform);
		ROS_DEBUG_STREAM ("read transform param from " << nh.getNamespace() << " into transform: " << std::endl << transform);
	}
	return tf::Transform(q, tr);
}

std::string btTransform_to_string(const tf::Transform &t) {
	tf::Vector3 v = t.getOrigin();
	tf::Quaternion r = t.getRotation();
	return (boost::format("((%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f, %.3f))")
			% v.x() % v.y() % v.z() % r.x() % r.y() % r.z() % r.w()).str();
}

std::string btVector3_to_string(const tf::Vector3 &v) {
	return (boost::format("% .3f, % .3f, % .3f") % v.x() % v.y() % v.z()).str();
}

std::string btMatrix3x3_to_string(const btMatrix3x3 &m) {
	return (btVector3_to_string(m.getRow(0)) + "\n" + btVector3_to_string(
			m.getRow(1)) + "\n" + btVector3_to_string(m.getRow(2)) + "\n");
}
} // namespace manipulation_transforms_util

#endif /* UTIL_H_ */
