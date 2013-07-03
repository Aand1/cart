#!/usr/bin/env python
PKG="manipulation_transforms"
import roslib; 
roslib.load_manifest(PKG)
import rospy
from manipulation_transforms.srv import *
from geometry_msgs.msg import PoseStamped

if __name__ == "__main__":
    try:
	#ns = "/cart_pushing/"
	ns = ""
	service_name = ns + PKG + "_server/MapObjectPoseToEffectors"
	rospy.loginfo("Waiting for service \"%s\"" % service_name)
    	rospy.wait_for_service(service_name)
        getGripperPosesFromCart_srvcall = rospy.ServiceProxy(service_name, MapObjectPoseToEffectors)
        queryPose = PoseStamped()
        queryPose.header.frame_id="base_footprint"
        queryPose.pose.position.x=0.7
	queryPose.pose.orientation.w=1.0
        poses = getGripperPosesFromCart_srvcall(queryPose)
        print poses
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
