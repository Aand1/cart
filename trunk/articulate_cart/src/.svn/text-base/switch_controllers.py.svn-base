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
from pr2_mechanism_msgs.srv import SwitchController, SwitchControllerRequest

class SwitchControllers:

    def __init__(self):
        self.goal_received = False
        self.ready_sub = rospy.Subscriber('cart_pushing/ready_pose_achieved', roslib.msg.Header, self.goal_cb)

    def goal_cb(self, msg):
        if not self.goal_received:
            rospy.loginfo('Switching controllers now that ready pose has been achieved')
            self.goal_received = True
            srv = rospy.ServiceProxy('pr2_controller_manager/switch_controller', SwitchController)
            srv(['l_cart', 'r_cart'], ['l_arm_controller', 'r_arm_controller'],
                SwitchControllerRequest.STRICT)


def switch_back_controllers():
    srv = rospy.ServiceProxy('pr2_controller_manager/switch_controller', SwitchController)
    srv(['l_arm_controller', 'r_arm_controller'],
        ['l_cart', 'r_cart'],
        SwitchControllerRequest.BEST_EFFORT)

def main():
    rospy.init_node('switch_controllers')
    node = SwitchControllers()
    rospy.on_shutdown(switch_back_controllers)
    rospy.spin()


if __name__ == "__main__":
    main()
