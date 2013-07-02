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
import os.path

from geometry_msgs.msg import Twist, Pose, PoseStamped, TwistStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pr2_controllers_msgs.msg import *

verbose = False
duration = 3.0

class pr2_pose_loader:
    def __init__(self, verbose = False):
        """ @package pr2_pose_saver : load
        init: defines goal state variables and opens publishers and clients
        """
        self.verbose = verbose

        ############### PRIVATE PUBLISHERS/CLIENTS ###############
        ## publisher for base commands
        self._base_pub = rospy.Publisher('base_controller/command', Twist)

        ## action client for grippers
        self._l_gripper_pub = rospy.Publisher('l_gripper_controller/command', Pr2GripperCommand)
        self._r_gripper_pub = rospy.Publisher('r_gripper_controller/command', Pr2GripperCommand)

        ## action client for torso
        self._torso_pub = rospy.Publisher('/torso_controller/command', JointTrajectory)

        ## action client for head
        self._head_pub = rospy.Publisher('/head_traj_controller/command', JointTrajectory)

        ## publisher for arms
        self._r_arm_pub = rospy.Publisher("r_arm_controller/command", JointTrajectory, latch=True)
        self._l_arm_pub = rospy.Publisher("l_arm_controller/command", JointTrajectory, latch=True)

        ## publishers dictionary
        self.publishers = {'base':self._base_pub,
                           'l_gripper':self._l_gripper_pub,
                           'r_gripper':self._r_gripper_pub,
                           'torso':self._torso_pub,
                           'head':self._head_pub, 
                           'r_arm':self._r_arm_pub,
                           'l_arm':self._l_arm_pub,
                           'r_gripper':self._r_gripper_pub,
                           'l_gripper':self._l_gripper_pub
                           }
        rospy.sleep(0.5)

    def set_arm_state(self, jvals, arm):
        """ Sets goal for indicated arm (r_arm/l_arm) using provided joint values"""
        # Build trajectory message
        command = JointTrajectory()
        command.joint_names = ['%s_shoulder_pan_joint' % arm[0], 
                               '%s_shoulder_lift_joint' % arm[0],
                               '%s_upper_arm_roll_joint' % arm[0],
                               '%s_elbow_flex_joint' % arm[0],
                               '%s_forearm_roll_joint' % arm[0],
                               '%s_wrist_flex_joint' % arm[0],
                               '%s_wrist_roll_joint' % arm[0]]
        command.points.append(JointTrajectoryPoint(
            positions=jvals,
            velocities = [0.0] * (len(command.joint_names)),
            accelerations = [],
            time_from_start =  rospy.Duration(duration)))
        command.header.stamp = rospy.get_rostime() + rospy.Duration(0.01)
        # Send
        try:
            self.publishers[arm].publish(command)
            if self.verbose:
                print "published [%s] to %s_controller/command topic" % (jvals, arm)
        except:
            print "failed to publish arm positions!"

    def set_gripper_state(self, jval, gripper):
        """ Sets goal for indicated gripper (r_gripper/l_gripper) using provided joint angle"""
        # Build trajectory message
        goal = Pr2GripperCommand()
        goal.max_effort = -1
        goal.position = jval
        try:
            self.publishers[gripper].publish(goal)
            if self.verbose:
                print "published [%s] to %s_controller/command topic" % (jval, gripper)
        except:
            print "failed to publish gripper positions!"

    def set_head_state(self, jvals):
        """ Sets goal for head using provided joint values"""
        # Build trajectory message
        head_goal = JointTrajectory()
        head_goal.joint_names.append('head_pan_joint')
        head_goal.joint_names.append('head_tilt_joint')
        head_goal.points.append(JointTrajectoryPoint())
        head_goal.points[0].time_from_start = rospy.Duration(0.25)
        #self.head_goal.header.frame_id = 'base_link'
        for i in range(len(head_goal.joint_names)):
            head_goal.points[0].positions.append(jvals[i])
            head_goal.points[0].velocities.append(1)
        head_goal.header.stamp = rospy.get_rostime() + rospy.Duration(0.01)
        try:
            #print head_goal
            self.publishers["head"].publish(head_goal)
            if self.verbose:
                print "published [%s] to head_traj_controller/command topic" % jvals
        except:
            print "failed to publish head position!"

    def set_torso_state(self, jval):
        """ Sets goal for torso using provided value"""
        # Build trajectory message
        torso_goal = JointTrajectory()
        torso_goal.joint_names.append('torso_lift_joint')
        torso_goal.points.append(JointTrajectoryPoint())
        torso_goal.points[0].time_from_start = rospy.Duration(0.25)
        torso_goal.points[0].velocities.append(0)
        torso_goal.points[0].positions.append(jval)
        torso_goal.header.stamp = rospy.get_rostime() + rospy.Duration(0.01)
        try:
            #print head_goal
            self.publishers["torso"].publish(torso_goal)
            if self.verbose:
                print "published [%s] to torso_controller/command topic" % jval
        except:
            print "failed to publish torso position!"

    def parse_bookmark_file(self, bfile):
        f=open(bfile,'r')

        for l in f.readlines():
            if l.find("arm") != -1:
                jvals_str = l.split(":")[1]
                jvals = map(lambda x: float(x),jvals_str.strip("\n").split(","))
                arm = l.split(":")[0]
                self.set_arm_state(jvals, arm)
            if l.find("gripper") != -1:
                jval_str = l.split(":")[1]
                jval = map(lambda x: float(x),jval_str.strip("\n").split(","))[0]
                gripper = l.split(":")[0]
                self.set_gripper_state(jval, gripper)
            if l.find("head") != -1:
                jvals_str = l.split(":")[1]
                jvals = map(lambda x: float(x),jvals_str.strip("\n").split(","))
                self.set_head_state(jvals)
            if l.find("torso") != -1:
                jvals_str = l.split(":")[1]
                jval = map(lambda x: float(x),jvals_str.strip("\n").split(","))[0]
                self.set_torso_state(jval)

        f.close()

if __name__ == "__main__":
    from optparse import OptionParser
    parser = OptionParser()
    parser.add_option("-v", "--verbose", action="store_true", dest="verbose", default=False,
                      help="verbose mode")
    parser.add_option("-a", "--all", action="store_true", dest="load_all", 
                      help="load all", default=True)
    parser.add_option("-r", "--right_arm", action = "store_true", dest="load_right_arm",
                      help="Load pose of right arm", default=True)
    parser.add_option("-l", "--left_arm", action = "store_true", dest="load_left_arm",
                      help="Load pose of left arm", default=True)
    parser.add_option("-R", "--right-gripper", action = "store_true", dest="load_right_gripper",
                      help="Save pose of right gripper", default=True)
    parser.add_option("-L", "--left-gripper", action = "store_true", dest="load_left_gripper",
                      help="Save pose of right gripper", default=True)
    parser.add_option("-H", "--head", action = "store_true", dest="load_head",
                      help="Load pose of head", default=True)
    parser.add_option("-t", "--torso", action = "store_true", dest="load_torso",
                      help="Load pose of torso", default=True)
    parser.add_option("-T", "--tilting-aser", action = "store_true", dest="load_laser",
                      help="Load pose of tilting laser", default=True)
    # J.Romano added time option (8.2.10)
    parser.add_option("-s", "--seconds", action="store", type="float", dest="duration",
                      help="Input number of seconds to take to achieve goal")
    parser.add_option("-b", "--completion-topic", action="store", type="string", dest="completion_topic",
                      help="Topic on which to broadcast message upon completion (not necessarily success)")
    parser.add_option("-w", "--wait", action="store", type="float", dest="wait",
                      help="Time in seconds to wait before broadcasting completion message")

    (options, args) = parser.parse_args()
    verbose = options.verbose
    if options.duration:
        duration = options.duration

    if len(args) == 0:
        print "Error: no bookmark file provided"
        print "See load.py --help for instructions"
        sys.exit()
    else:
        bookmark_file = args[0]

    if not os.path.exists(bookmark_file):
        print "Error: could not locate provided bookmark file"
        sys.exit()

    rospy.init_node('bookmarker_publisher', anonymous=True)

    loader = pr2_pose_loader(options.verbose)
    loader.parse_bookmark_file(bookmark_file)

    if options.completion_topic:
        if options.wait:
            rospy.loginfo('Messages sent.  Waiting %s seconds', options.wait)
            rospy.sleep(options.wait)
        msg = rospy.Header()
        msg.stamp = rospy.Time.now()
        pub = rospy.Publisher(options.completion_topic, rospy.Header)
        rospy.loginfo('Done; broadcasting success on %s', options.completion_topic)
        while not rospy.is_shutdown():
            pub.publish(msg)
            rospy.sleep(1)
    
    
    
