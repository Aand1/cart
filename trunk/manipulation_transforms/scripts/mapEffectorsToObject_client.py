#!/usr/bin/env python
PKG="manipulation_transforms"
import roslib; 
roslib.load_manifest(PKG)
import rospy
import tf
from manipulation_transforms.srv import *
from geometry_msgs.msg import *

tl=None

def get_effector_poses():
    """
    """
    r_pose_tup = get_tf_transform('base_footprint', 'r_gripper_tool_frame')
    l_pose_tup = get_tf_transform('base_footprint', 'l_gripper_tool_frame')

    l_queryPose = PoseStamped()
    l_queryPose.header.frame_id="base_footprint"
    l_queryPose.pose = pose_tup_to_msg(l_pose_tup)
    
    r_queryPose = PoseStamped()
    r_queryPose.header.frame_id="base_footprint"
    r_queryPose.pose = pose_tup_to_msg(r_pose_tup)

    query_poses=[]
    query_poses.append(r_queryPose)
    query_poses.append(l_queryPose)
    return query_poses

def get_tf_transform(ref_frame, target_frame):
    """An exception-handling method for querying a transform from tf"""
    try:
        tl.waitForTransform(ref_frame, target_frame, rospy.Time(0), rospy.Duration(4.0))
        return tl.lookupTransform(ref_frame, target_frame, rospy.Time(0)) 
    except (tf.Exception, tf.LookupException, tf.ConnectivityException) as ex:
        print ex
        return None

def pose_tup_to_msg(tup):
    """Returns Pose message assembled from tf query tuple """
    msg = Pose()
    msg.position.x = tup[0][0]
    msg.position.y = tup[0][1]
    msg.position.z = tup[0][2]
    msg.orientation.x = tup[1][0]
    msg.orientation.y = tup[1][1]
    msg.orientation.z = tup[1][2]
    msg.orientation.w = tup[1][3]
    return msg

if __name__ == "__main__":
    rospy.init_node("MapEffectorPosesToObject")
    tl=tf.TransformListener()
    try:
	ns = "/cart_pushing/"
	#ns = ""
	service_name = ns + PKG + "_server/MapEffectorPosesToObject"
	rospy.loginfo("Waiting for service \"%s\"" % service_name)
    	rospy.wait_for_service(service_name)
        getCartPoseFromGrippers_srvcall = rospy.ServiceProxy(service_name, MapEffectorPosesToObject)

        # l_queryPose = PoseStamped()
	# l_queryPose.pose.orientation.w=1.0
        # l_queryPose.header.frame_id="base_footprint"

        # r_queryPose = PoseStamped()
	# r_queryPose.pose.orientation.w=1.0
        # r_queryPose.header.frame_id="base_footprint"

	# query_poses=[]
	# query_poses.append(r_queryPose)
	# query_poses.append(l_queryPose)

        query_poses = get_effector_poses()
	print "query poses:", query_poses

        cart_pose = getCartPoseFromGrippers_srvcall(query_poses)
        print cart_pose
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
