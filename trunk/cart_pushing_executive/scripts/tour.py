#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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

import roslib
roslib.load_manifest('cart_pushing_executive')
import rospy
import time
import actionlib
import tf

from move_base_msgs.msg import *
from geometry_msgs.msg import *
from roslib.msg import *

goals = [
    [(27.65, 17.29, 0.0), (0.0, 0.0, 0.57, 0.82)],
    [(25.1, 24.0, 0), (0, 0, 1.0, 0.0)],
    [(18.6, 24.4, 0), (0, 0, -0.83, 0.55)],
    [(18.7, 18.8, 0), (0, 0, -0.18, 0.98)],
    [(27.8, 17.9, 0), (0, 0, 0.54, 0.84)],
    [(26.1, 23.6, 0), (0, 0, 0.98, 0.20)],
    [(17.9, 18.9, 0), (0, 0, 0.71, 0.71)],
    [(18.462, 24.971, 0), (0, 0, 0.643354674336, 0.765568261496)],
    [(18.7669181824, 29.0535163879, 0), (0, 0, 0.551830767493, 0.83395611638)],
    [(20.036, 32.439, 0.0), (0, 0, 0.706, 0.708)],
    [(20.858581543, 39.1036987305, 0), (0, 0, 0.601096665154, 0.799176325438)],
    [(20.6498146057, 36.3583908081, 0), (0, 0, -0.760489363805, 0.64935038888)],
    [(19.9862194061, 33.1941947937, 0), (0, 0, -0.70200284829, 0.712174136706)],
    [(19.2100734711, 30.0183944702, 0), (0, 0, -0.836560676379, 0.547874287348)],
    [(18.6010341644, 26.2357540131, 0), (0, 0, -0.609730442973, 0.79260884862)]
    ]

def squared_dist(p1, p2):
    dx = p1.point.x - p2[0]
    dy = p1.point.y - p2[1]
    return dx*dx + dy*dy

def mb_goal(goal):
    return MoveBaseGoal(PoseStamped(Header(frame_id = 'map'),
                                      Pose(Point(goal[0][0], goal[0][1],
                                                 goal[0][2]),
                                           Quaternion(goal[1][0], goal[1][1],
                                                      goal[1][2], goal[1][3]))))
if __name__ == '__main__':
    rospy.init_node('willow_tour_node')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base")
    client.wait_for_server()
    rospy.loginfo("Sending goals")
    tf = tf.TransformListener()
    rospy.sleep(2)

    p = PointStamped()
    p.header.frame_id = 'base_footprint'
    p.header.stamp = rospy.Time()
    pos = tf.transformPoint('map', p)
    
    best_dist = 1000000
    current = 0
    for (i, g) in enumerate(goals):
        if squared_dist(pos, g[0]) < best_dist:
            best_dist = squared_dist(pos, g[0])
            current = i
    rospy.loginfo('Closest waypoint is %s: %s', current, goals[current])
    
    for c in range(5):
        while current < len(goals):
            mb = mb_goal(goals[current])
            rospy.loginfo("Sending goal %s" % goals[current])
            client.send_goal(mb)
            client.wait_for_result()
            current +=1
        current = 0

    rospy.loginfo('Done with tour')



