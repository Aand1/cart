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

PKG="articulate_cart"
import roslib; roslib.load_manifest(PKG)
import rospy
import tf
import thread

from articulate_cart_server import articulateCartServer
from manipulation_transforms.srv import *

from std_msgs.msg import *
from geometry_msgs.msg import *
#from pr2_controllers_msgs.msg import *
from math import pi, pow, atan, sin, cos, sqrt
import exceptions, sys

class fakeArticulateCartServer(articulateCartServer):
  """ A fake version of the articulate_cart server which allows us to run the 
cart_pushing task in stage (real runs require cart pose estimates from 
either the checkerboard or the arms).  To do this, we present the same
interface as the regular articulate_cart server, but rather than talking
to the arms, we just use the solver to give us target poses for the grippers
and write them directly to tf.
  """

  def __init__(self, use_tf_grippers, use_gripper_midpoint, tf_object_name):
    # call parent to set up goal topics and load params and stuff
    articulateCartServer.__init__(self, use_tf_grippers, use_gripper_midpoint, tf_object_name)

    # a tf broadcaster for spoofing gripper poses
    rospy.loginfo("Creating a tf broadcaster")
    self._tb = tf.TransformBroadcaster()

    ## Initialize gripper poses from cart initial pose (for update loop) 
    rospy.loginfo("Initializing gripper poses from cart initial pose")
    try:
      self._pose_resp = self._cartPoseToGrippers_srv(self._cart_pose_msg)
    except rospy.ServiceException, e:
      print "Service did not process request: %s"%str(e)

    rospy.loginfo("Successfully initialized gripper poses")    
    # start main update loop 
    rospy.loginfo("starting thread for tf update")
    thread.start_new_thread(self._slow_tf_update_loop, ())
    rospy.loginfo("Fake Articulate Cart Server initialization complete!")

  def send_effector_poses_to_tf(self):
     # send gripper poses to tf
    if len(self._pose_resp.effector_poses) != 2:
      rospy.logerr("Expected 2 gripper poses but got %d" % len(self._pose_resp.effector_poses))
    else:
      self._tb.sendTransform((self._pose_resp.effector_poses[0].pose.position.x,
                              self._pose_resp.effector_poses[0].pose.position.y,
                              self._pose_resp.effector_poses[0].pose.position.z),
                             (self._pose_resp.effector_poses[0].pose.orientation.x,
                              self._pose_resp.effector_poses[0].pose.orientation.y,
                              self._pose_resp.effector_poses[0].pose.orientation.z,
                              self._pose_resp.effector_poses[0].pose.orientation.w),
                             rospy.get_rostime(),
                             'r_gripper_tool_frame',
                             'base_footprint')

      self._tb.sendTransform((self._pose_resp.effector_poses[1].pose.position.x,
                              self._pose_resp.effector_poses[1].pose.position.y,
                              self._pose_resp.effector_poses[1].pose.position.z),
                             (self._pose_resp.effector_poses[1].pose.orientation.x,
                              self._pose_resp.effector_poses[1].pose.orientation.y,
                              self._pose_resp.effector_poses[1].pose.orientation.z,
                              self._pose_resp.effector_poses[1].pose.orientation.w),
                             rospy.get_rostime(),
                             'l_gripper_tool_frame',
                             'base_footprint')

  def _pose_command_action(self):
    """Send the target effector poses to the arm control topics"""
    if len(self._pose_resp.effector_poses) != 2:
      rospy.logerr("Expected 2 gripper poses, but got %d" % len(self._pose_resp.effector_poses))
    else:
        self.send_effector_poses_to_tf()

  def _slow_tf_update_loop(self, r = 3):
      try:
          rate = rospy.Rate(r) # 1hz should be sufficient for this
          while not rospy.is_shutdown():
            with self._pose_mutex:
              self.send_effector_poses_to_tf()
            rate.sleep()
      except (exceptions.AttributeError) as e:
          print e
          raise e
      except:
          print "Unexpected error:", sys.exc_info()[0]
          raise

if __name__ == '__main__':
    from optparse import OptionParser
    parser = OptionParser()
    parser.add_option("-g", action="store_true", dest="use_tf_grippers", default=False,
                      help="use current gripper poses for initial grasp transforms (otherwise leaves defaults from param server)")
    parser.add_option("-m", "--midpoint", action="store_true", dest="midpoint", default=False,
                      help="set object initial pose to midpoint between grippers [default=%default]")
    parser.add_option("-o", "--object", dest="object", default="",
                      help="attempt to load an initial object transform with name OBJECT from tf [default=\"%default\"]")
    parser.add_option("-f", "--fake-twist-controller", dest="fake_twist_controller", default=True,
                      help="use the fake twist message callback, which simulates twists with pose messages [default=\"%default\"]")
    (options, args) = parser.parse_args()
    
    rospy.init_node("fake"+PKG+"_server")
    server = fakeArticulateCartServer(options.use_tf_grippers, options.midpoint, options.object)
    rospy.spin()
