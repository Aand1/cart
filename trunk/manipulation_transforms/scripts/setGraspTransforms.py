#!/usr/bin/env python

PKG="manipulation_transforms"
import roslib; 
roslib.load_manifest(PKG)
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from manipulation_transforms.msg import *
from manipulation_transforms.srv import *
from roslib.msg import Header

if __name__ == "__main__":

    rospy.init_node(PKG)
    ns = "/cart_pushing/"
    #ns = ""
    service_name = ns + PKG + "_server/SetInitialTransforms"
    rospy.loginfo("Waiting for service \"%s\"" % service_name)
    rospy.wait_for_service(service_name)

    try:
        SetInitialTransforms_srv = rospy.ServiceProxy(service_name, SetInitialTransforms)
        obj_init_pose = PoseStamped(
            header = Header(frame_id="base_footprint",
                            stamp=rospy.Time.now()),
            pose = Pose(
                position = Point(0.5, 0.0, 0.344),
                orientation = Quaternion(-0.5, -0.44, -0.45, 0.59)))

        gripper_init_poses = []
        gripper_init_poses.append(PoseStamped(
                header=Header(frame_id="base_footprint",
                              stamp=rospy.Time.now()),
                pose=Pose(position=Point(0.5, -0.12, 0.344),
                          orientation=Quaternion(-0.5, -0.44, -0.45, 0.59))))

        gripper_init_poses.append(PoseStamped(
                header=Header(frame_id="base_footprint",
                              stamp=rospy.Time.now()),
                pose=Pose(position=Point(0.5, 0.13, 0.344),
                          orientation=Quaternion(-0.57, -0.39, -0.52, 0.50))))
        
        SetInitialTransforms_srv(obj_init_pose, gripper_init_poses)
        print "Sent service request"
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
