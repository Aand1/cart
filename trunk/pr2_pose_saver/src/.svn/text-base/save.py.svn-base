#! /usr/bin/env python
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#         * Redistributions of source code must retain the above copyright
#             notice, this list of conditions and the following disclaimer.
#         * Redistributions in binary form must reproduce the above copyright
#             notice, this list of conditions and the following disclaimer in the
#             documentation and/or other materials provided with the distribution.
#         * Neither the name of the Willow Garage, Inc. nor the names of its
#             contributors may be used to endorse or promote products derived from
#             this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Jon Scholz

PKG="pr2_pose_saver"
import roslib; roslib.load_manifest(PKG)
import rospy
import time
import sys

from geometry_msgs.msg import Twist, Pose, PoseStamped, TwistStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pr2_controllers_msgs.msg import *


def save_arm_pose_to_file(file_handle, arm):
    """ Reads pose of indicated arm and saves to textfile"""
    arm_pose = rospy.wait_for_message("%s_arm_controller/state" % arm, JointTrajectoryControllerState) # r or l
    jpos = arm_pose.actual.positions
    f.write("%s_arm:" % arm)
    f.write(jpos.__str__().strip("()"))
    f.write("\n")

def save_head_pose_to_file(file_handle):
    head_pose = rospy.wait_for_message("/head_traj_controller/state", JointTrajectoryControllerState)
    jpos = head_pose.actual.positions
    f.write("head:")
    f.write(jpos.__str__().strip("()"))
    f.write("\n")

def save_torso_position_to_file(file_handle):
    torso_pose = rospy.wait_for_message("/torso_controller/state", JointTrajectoryControllerState)
    jpos = torso_pose.actual.positions[0]
    f.write("torso:")
    f.write(jpos.__str__().strip("()"))
    f.write("\n")

def save_gripper_angle_to_file(file_handle, gripper):
    gripper_state = rospy.wait_for_message("/%s_gripper_controller/state" % gripper, JointControllerState) # r or l
    jpos = gripper_state.process_value
    f.write("%s_gripper: %f\n" % (gripper, jpos))

def save_tilting_laser_angle_to_file(file_handle):
    pass

def save_base_position_to_file(file_handle):
    pass

if __name__ == "__main__":
    from optparse import OptionParser
    usage = "usage: %prog [options] [filename]"
    parser = OptionParser(usage=usage)
    parser.add_option("-a", "--all", action="store_true", dest="save_all", 
                      help="save all", default=True)
    parser.add_option("-r", "--right_arm", action = "store_true", dest="save_right_arm",
                      help="Save pose of right arm", default=True)
    parser.add_option("-l", "--left_arm", action = "store_true", dest="save_left_arm",
                      help="Save pose of left arm", default=True)
    parser.add_option("-R", "--right-gripper", action = "store_true", dest="save_right_gripper",
                      help="Save pose of right gripper", default=True)
    parser.add_option("-L", "--left-gripper", action = "store_true", dest="save_left_gripper",
                      help="Save pose of right gripper", default=True)
    parser.add_option("-H", "--head", action = "store_true", dest="save_head",
                      help="Save pose of head", default=True)
    parser.add_option("-t", "--torso", action = "store_true", dest="save_torso",
                      help="Save pose of torso", default=True)
    parser.add_option("-T", "--tilting-laser", action = "store_true", dest="save_laser",
                      help="Save pose of tilting laser", default=True)
    (options, args) = parser.parse_args()

    rospy.init_node('bookmarker_listener', anonymous=True)

    filename = ""
    if len(args) == 1:
        if args[0].find(".pps") > 0:
            filename = args[0]
        else:
            filename = args[0] + ".pps"
    else:
        datestr="%s_%s_%s-%s:%s:%s" % (time.localtime().tm_mon,time.localtime().tm_mday,time.localtime().tm_year,
                                       time.localtime().tm_hour, time.localtime().tm_min,time.localtime().tm_sec)
        filename = "%s.pps" %  datestr

    f=open(filename, "w")
    
    # Step through requested topics and save
    if options.save_right_arm or options.save_all:
        save_arm_pose_to_file(f, 'r')
    if options.save_left_arm or options.save_all:
        save_arm_pose_to_file(f, 'l')
    if options.save_right_gripper or options.save_all:
        save_gripper_angle_to_file(f, 'r')
    if options.save_left_gripper or options.save_all:
        save_gripper_angle_to_file(f, 'l')
    if options.save_head or options.save_all:
        save_head_pose_to_file(f)
    if options.save_torso or options.save_all:
        save_torso_position_to_file(f)
    
    f.close()
