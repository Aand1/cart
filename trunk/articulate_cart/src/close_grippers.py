#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Bhaskara Marthi

import roslib; roslib.load_manifest('articulate_cart')
import rospy
import actionlib
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal
import move_base_msgs.msg
import geometry_msgs.msg

class CloseGrippers:

    def __init__(self):
        self.goal_received = False
        self.goal_sub = rospy.Subscriber('move_base/goal', move_base_msgs.msg.MoveBaseActionGoal, self.goal_cb)
        self.pose_sub = rospy.Subscriber('/initialpose', geometry_msgs.msg.PoseWithCovarianceStamped, self.pose_cb)

    def goal_cb(self, msg):
        if not self.goal_received:
            rospy.loginfo('Closing grippers now that a goal has been received')
            self.goal_received = True
        
    def pose_cb(self, msg):
        if not self.goal_received:
            rospy.loginfo('Closing grippers')
            self.goal_received = True

    def spin(self):
        l_client = actionlib.SimpleActionClient('l_gripper_controller/gripper_action', Pr2GripperCommandAction)
        r_client = actionlib.SimpleActionClient('r_gripper_controller/gripper_action', Pr2GripperCommandAction)
        l_client.wait_for_server()
        r_client.wait_for_server()

        goal = Pr2GripperCommandGoal()
        goal.command.position = 0.0
        goal.command.max_effort = 100.0


        while not rospy.is_shutdown():
            if self.goal_received:
                l_client.send_goal(goal)
                r_client.send_goal(goal)
                l_client.wait_for_result()
                r_client.wait_for_result()
            rospy.sleep(1)


def main():
    rospy.init_node('close_grippers')
    node = CloseGrippers()
    rospy.loginfo('close_grippers initialized')
    node.spin()

    


if __name__ == "__main__":
    main()
