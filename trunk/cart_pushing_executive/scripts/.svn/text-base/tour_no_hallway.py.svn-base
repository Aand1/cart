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
    [(22.7547492981,17.2507705688,0.0),(0.0,0.0,-0.21978236622,0.975548928296)],
    [(28.1076889038,17.9328041077,0.0),(0.0,0.0,0.534407395401,0.84522703207)],
    [(26.410533905,23.2363433838,0.0),(0.0,0.0,0.975548880533,0.219782578222)],
    [(19.210647583,23.2893428802,0.0),(0.0,0.0,-0.834799107823,0.550554674467)],
    [(23.0300712585,17.0846633911,0.0),(0.0,0.0,-0.193364053165,0.981127077877)],
    [(27.9582328796,17.5433273315,0.0),(0.0,0.0,0.539956703804,0.841692793136)],
    [(34.0350074768,23.7652778625,0.0),(0.0,0.0,-0.195291409231,0.980745260239)],
    [(33.3261146545,13.9941511154,0.0),(0.0,0.0,-0.851834538099,0.523810957981)],#inside cafeteria going in
    [(30.5941886902,8.5315656662,0.0),(0.0,0.0,-0.841195162852,0.540731632138)],#inside cafeteria coming out
    [(30.3537712097,8.00482368469,0.0),(0.0,0.0,0.540731770207,0.8411950741)],#inside cafeteria coming out
    [(34.1780052185,23.7703514099,0.0),(0.0,0.0,0.984122604912,0.177489995494)],
    [(25.7120513916,23.4842967987,0.0),(0.0,0.0,0.977169336603,0.212461967425)],
    [(18.5160522461,22.5130939484,0.0),(0.0,0.0,-0.752447094618,0.658652692852)]
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



