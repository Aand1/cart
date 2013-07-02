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
import numpy

from manipulation_transforms.srv import *

from std_msgs.msg import *
from geometry_msgs.msg import *
#from pr2_controllers_msgs.msg import *
from math import pi, pow, atan, sin, cos, sqrt

class ArticulateCartException(Exception):
  pass

class articulateCartServer:
  """ A server for manipulating a cart being grasped by the grippers
  of the pr2.  Uses manipulation_transforms to compute gripper poses satisfying
  incoming goal requests.  Goals for the cart pose are specified in the
  base_footprint frame.  manipulation_transforms is used to map this goal to
  gripper poses in the base_footprint frame, which are then written
  to the goal topics for teleop_controllers (make sure to specify base_footprint
  as the frame_id rather than torso_lift_link).
  Accepts goals as PoseStamped messages, and issues PoseStamped messages
  to the arms.
  """

  # create messages that are used to publish feedback/result
  _r_arm_SSE = 0
  _l_arm_SSE = 0
  _r_arm_err_twist = Twist()
  _l_arm_err_twist = Twist()
  errthresh = 0.02

  ##
  # @param use_tf_grippers: lookup gripper poses from tf and re-initialize manipulation_transforms
  # @param use_gripper_midpoint: set object initial pose to midpoint between grippers, and re-initialize manipulation_transforms
  # @param tf_object_name: if non-empty, attempt to lookup object initial pose on tf with given frame name, and re-initialize manipulation_transforms 
  #
  def __init__(self, use_tf_grippers, use_gripper_midpoint, tf_object_name):
    # A tf listener for reading gripper poses and stuff
    self._tl = tf.TransformListener()

    ## Attach to all topics and services
    self._initialize_ros_coms()

    # Wait till initial pose has been set
    self.goal_received = False
    self.goal_sub = rospy.Subscriber('cart_pushing/ready_pose_achieved', rospy.Header, self.ready_pose_cb)
    rospy.loginfo('Articulate cart initialization waiting for ready pose before proceeding')
    ready_pose_counter = 0
    while not rospy.is_shutdown() and not self.goal_received:
       rospy.sleep(0.1)
       ready_pose_counter += 1
       if ready_pose_counter > 100:
         raise ArticulateCartException('Timed out when waiting for confirmation of ready pose achieved')
    rospy.loginfo('Articulate cart received ready pose')
    
    ## Initialize object and gripper poses according to constructor params
    self._initialize_poses(use_tf_grippers, use_gripper_midpoint, tf_object_name)

    ## Mutex for pose-related members {_cart_pose_tup, _cart_pose_msg, _pose_resp, _twist_req, _cart_pose_euler}
    self._pose_mutex = thread.allocate_lock()
        
    # Indicator fake twist controller loop
    self._fake_twist_loop_started = False # Toggled by first incoming twist message

    ## Spin thread for posture updater
    thread.start_new_thread(self._posture_loop, ("elbowmidr", "elbowmidl")) # elbowdownr, elbowdownl
    
    ## All done:
    print "Articulate Cart Server Ready!"

  def ready_pose_cb(self, msg):
    self.goal_received = True

  def _initialize_ros_coms(self):
    ## subscriber for goal topic
    self._goal_sub = rospy.Subscriber('~command_twist', TwistStamped, self._fake_twist_goal_cb)

    
    ## publisher for arm cartesian pose/twist controller
    self._r_arm_pose_pub = rospy.Publisher('/r_cart/command_pose', PoseStamped)
    self._l_arm_pose_pub = rospy.Publisher('/l_cart/command_pose', PoseStamped)
    self._r_arm_twist_pub = rospy.Publisher('/r_cart/command_twist', TwistStamped)
    self._l_arm_twist_pub = rospy.Publisher('/l_cart/command_twist', TwistStamped)
    
    ## Publishers for teleop_controllers' posture topics
    self._r_posture_pub = rospy.Publisher("/r_cart/command_posture", Float64MultiArray)
    self._l_posture_pub = rospy.Publisher("/l_cart/command_posture", Float64MultiArray)

    ## Publisher for when when an invalid pose is received
    self.invalid_pose_pub = rospy.Publisher("~invalid_pose", Empty)
    
    ## subscriber for arm state errors
    self._r_arm_sub = rospy.Subscriber('/r_cart/state/x_err', Twist, self._r_arm_err_cb)
    self._l_arm_sub = rospy.Subscriber('/l_cart/state/x_err', Twist, self._l_arm_err_cb)
    self._r_arm_err_lock = thread.allocate_lock()
    self._l_arm_err_lock = thread.allocate_lock()
    
    ## get handles for manipulation_transforms service calls
    rospy.loginfo("Waiting for solver services")
    rospy.wait_for_service("manipulation_transforms_server/MapObjectPoseToEffectors", 10)
    rospy.wait_for_service("manipulation_transforms_server/LoadInitialTransforms", 10)
    self._solver_reloadTransforms_srv = rospy.ServiceProxy("manipulation_transforms_server/LoadInitialTransforms",
                                                           LoadInitialTransforms)
    self._cartPoseToGrippers_srv = rospy.ServiceProxy("manipulation_transforms_server/MapObjectPoseToEffectors",
                                                        MapObjectPoseToEffectors)
    rospy.loginfo("Solver services connected")

    
      
  def _initialize_poses(self, use_tf_grippers, use_gripper_midpoint, tf_object_name):
    ## Update initial effector poses if requested
    call_reload=False
    if use_tf_grippers:
        ## Read current gripper transforms (needed if reloading solver, or computing drift)
        self._r_pose_tup = self.get_tf_transform('base_footprint', 'r_gripper_tool_frame')
        self._l_pose_tup = self.get_tf_transform('base_footprint', 'l_gripper_tool_frame')
        if self._r_pose_tup is not None and self._l_pose_tup is not None:
            use_defaults = False
        else:
            rospy.logerr("TF lookup of gripper poses failed.  Using default grasp transforms from the parameter server")
        
        call_reload=True
    else:
      # Load poses from param server
      self._r_pose_tup = self.get_transform_param("cart_pushing/r_gripper_grasp")
      self._l_pose_tup = self.get_transform_param("cart_pushing/l_gripper_grasp")
    
    ## Set object poses from grippers if requested:    
    if use_gripper_midpoint:
        # Compute object orientation as midpoint between gripper orientations
        self._cart_pose_tup = self.interpolate_pose_tup(self._r_pose_tup, self._l_pose_tup, 0.5)
        #obj_trans = ((numpy.array(list(self._r_pose_tup[0])) + numpy.array(list(self._l_pose_tup[0])))/2).tolist()            
        call_reload=True
    elif tf_object_name:
        # Load object orientation from tf
        self._cart_pose_tup = self.get_tf_transform('base_footprint', tf_object_name)
        call_reload=True
    else:
        ## read object pose from parameter server
        self._cart_pose_tup = self.get_transform_param("cart_pushing/cart_init_pose")
        
    ## Create a PoseStamped message with initial cart pose
    self._cart_pose_msg = PoseStamped()
    self._cart_pose_msg.pose.position = geometry_msgs.msg.Point(*self._cart_pose_tup[0])
    self._cart_pose_msg.pose.orientation = geometry_msgs.msg.Quaternion(*self._cart_pose_tup[1])
    self._cart_pose_msg.header.frame_id = "base_footprint"
    self._cart_pose_msg.header.stamp = rospy.Time.now()

    # Pre-compute euler pose (for fake twist controller)
    self._cart_pose_euler = self.pose_msg_to_euler_list(self._cart_pose_msg.pose)

    ## Pre-compute relative pose of left gripper (for twist cb drift error)
    self._r_pose_mtr = self.pose_tup_to_matrix(self._r_pose_tup)
    self._l_pose_mtr = self.pose_tup_to_matrix(self._l_pose_tup)
    self._l_relpose = self.get_rel_pose(self._r_pose_mtr, self._l_pose_mtr)

    ## Load cart workspace constraints from parameter server
    self._cart_x_min = rospy.get_param("cart_pushing/x_min")
    self._cart_x_max = rospy.get_param("cart_pushing/x_max")
    self._cart_y_min = rospy.get_param("cart_pushing/y_min")
    self._cart_y_max = rospy.get_param("cart_pushing/y_max")
    self._cart_t_min = rospy.get_param("cart_pushing/t_min")
    self._cart_t_max = rospy.get_param("cart_pushing/t_max")

    ## Reload solver transforms if necessary
    if call_reload:
        rospy.loginfo("Reloading solver grasp transforms with current gripper poses")
        self.set_transform_param("cart_pushing/cart_init_pose", self._cart_pose_tup)
        self.set_transform_param("cart_pushing/r_gripper_grasp", self._r_pose_tup)
        self.set_transform_param("cart_pushing/l_gripper_grasp", self._l_pose_tup)
        resp = self._solver_reloadTransforms_srv("")
        if not resp.success:
          rospy.logerror("Failed to reload solver transforms!")
        else:
          rospy.loginfo("Reloaded solver transforms using current gripper poses")

    ## Hard-code a set of arm postures
    self._postures = {
      'off': [],
      'mantis': [0, 1, 0,  -1, 3.14, -1, 3.14],
      'elbowupr': [-0.79,0,-1.6,  9999, 9999, 9999, 9999],
      'elbowupl': [0.79,0,1.6 , 9999, 9999, 9999, 9999],
      'old_elbowupr': [-0.79,0,-1.6, -0.79,3.14, -0.79,5.49],
      'old_elbowupl': [0.79,0,1.6, -0.79,3.14, -0.79,5.49],
      'elbowdownr': [-0.028262077316910873, 1.2946342642324222, -0.25785640577652386, 
                      -1.5498884526859626, -31.278913849571776, -1.0527644894829107, -1.8127318367654268],
      'elbowdownl': [-0.0088195719039858515, 1.2834828245284853, 0.20338442004843196, 
                      -1.5565279256852611, -0.096340012666916802, -1.0235018652439782, 1.7990893054129216],
      'elbowmidr': [-0.28990347455256005, 1.089312348420358, -0.94309188122730947, -1.3356881736879038, 
                     0.42316056966725524, -1.3491191190220519, -5.3263966597959937],
      'elbowmidl': [0.52521589892944154, 1.0820586985900948, 1.0342420867971822, -1.3718169587386895, 
                    -6.6154390746853364, -1.3118240320657639, -10.572623351665454]
      }
             

  def _fake_twist_goal_cb(self, msg):
    """An alternative callback for twist messages that spawns an internal controller to simulate
    the object twist action using pose commands"""
    rospy.logdebug('Read goal: [%f,%f,%f][%f,%f,%f]', 
                  msg.twist.linear.x,
                  msg.twist.linear.y,
                  msg.twist.linear.z,
                  msg.twist.angular.x,
                  msg.twist.angular.y,
                  msg.twist.angular.z)
    
    # Save twist request for fake controller
    with self._pose_mutex:
      self._twist_req = msg.twist

    # if this is the first incoming message, spin the controller thread
    if self._fake_twist_loop_started == False:
        self._fake_twist_loop_started = True;
        thread.start_new_thread(self._fake_twist_controller, ()) # at default frequency
      
  def _r_arm_err_cb(self, twist):
    self._r_arm_err_lock.acquire()
    self._r_arm_err_twist=twist;
    self._r_arm_SSE = self.get_twist_SSE(twist)
    self._r_arm_err_lock.release()

  def _l_arm_err_cb(self, twist):
    self._l_arm_err_lock.acquire()
    self._l_arm_err_twist=twist
    self._l_arm_SSE = self.get_twist_SSE(twist)
    self._l_arm_err_lock.release()

  def _posture_loop(self, r_posture, l_posture):
    """ Send updates to the posture topic every ROS second """
    while not rospy.is_shutdown():
      self._r_posture_pub.publish(Float64MultiArray(data = self._postures[r_posture]))
      self._l_posture_pub.publish(Float64MultiArray(data = self._postures[l_posture]))
      rospy.sleep(1.0)

  def _fake_twist_controller(self, r = 20, recovery_horizon = 5):
      """ Simulates the twist applied to the object, and updates the gripper poses accordingly.
      """

      counter = 0

      # Initialize object pose list as Euler representation of cart pose
      with self._pose_mutex:
          self._cart_pose_euler = self.pose_msg_to_euler_list(self._cart_pose_msg.pose)
      rate = rospy.Rate(r)
      t = 1.0/r
      
      while not rospy.is_shutdown():
          with self._pose_mutex:
            # if counter % 200 == 0:
            #  rospy.loginfo('Cart rel pose is %s', self._cart_pose_euler)
            counter += 1

            if self._check_cart_pose(self._cart_pose_euler):
              # Default case: current cart pose is valid; let's make sure it stays that way
              new_pose = forward_simulate_twist(self._cart_pose_euler, self._twist_req, t)
              if self._check_cart_pose(new_pose):
                self.set_new_pose(new_pose)
              else:
                self.invalid_pose_pub.publish()
                rospy.logwarn('Applying %s for %s secs from %s would lead to infeasible pose %s; ignoring.',
                              self._twist_req, t, self._cart_pose_euler, new_pose)

            else:
              # 'Recovery' case: current pose is not valid; we check if the twist is bringing
              # us back into validity
              succeeded = False
              for i in xrange(recovery_horizon):
                new_pose = forward_simulate_twist(self._cart_pose_euler, self._twist_req, i*t)
                if self._check_cart_pose(new_pose):
                  # Twist is bringing us back to validity, so we accept it
                  rospy.loginfo('Current pose %s invalid, but %s leads to valid pose %s after %s seconds',
                                self._cart_pose_euler, self._twist_req, i*t)
                  self.set_new_pose(new_pose)
                  succeeded = True
                  break
              if not succeeded:
                self.invalid_pose_pub.publish()
                rospy.logwarn('Currently in collision at %s, and twist %s not helping',
                              self._cart_pose_euler, self._twist_req)
          rate.sleep()

  def set_new_pose (self, p):
      self._cart_pose_euler = p
      self._cart_pose_msg.pose = self.euler_list_to_pose_msg(p)
      self._pose_resp = self._cartPoseToGrippers_srv(self._cart_pose_msg)
      self._pose_command_action() 
      
  def _pose_command_action(self):
    """Send the target effector poses to the arm control topics
    Assumes that self._pose_resp is up-to-date"""
    if len(self._pose_resp.effector_poses) != 2:
      rospy.logerr("Expected 2 gripper poses, but got %d" % len(self._pose_resp.effector_poses))
    else:
      self._r_arm_pose_pub.publish(self._pose_resp.effector_poses[0])
      self._l_arm_pose_pub.publish(self._pose_resp.effector_poses[1])
    
    #print self._pose_resp.effector_poses
    # Uncomment to wait for error to drop before returning
    # while self._l_arm_SSE + self._r_arm_SSE > self.errthresh and not rospy.is_shutdown():
    #   print self._l_arm_SSE + self._r_arm_SSE

  def get_transform_param(self, name):
      """Returns the named transform as a pose tuple
      
      Arguments:
      - `name`: string matching a pose on the param server
      """
      param = rospy.get_param(name)
      rospy.loginfo('Parameter %s is %s', name, param)
      return (tuple(param["position"]), tuple(param["orientation"]))  
  

  def set_transform_param(self, name, pose_tup):
      """Sets the provided transform param tuple on the paramater server in "cart format"
      
      Arguments:
      - `pose_tup`: a tuple containing ((trans),(rot))
      - `name`: a string to set
      """
      rospy.set_param(name, {"position":pose_tup[0], "orientation":pose_tup[1]})
  
  def pose_tup_to_matrix(self, tup):
      """Returns a numpy 4x4 transform array from a pose provided in a tuple ((vector), (quaternion))"""
      mtr = tf.transformations.quaternion_matrix(tup[1])
      mtr[0][3] = tup[0][0]
      mtr[1][3] = tup[0][1]
      mtr[2][3] = tup[0][2]
      return mtr

  def pose_tup_to_msg(self, tup):
    """Returns Pose message assembled from tf query tuple """
    msg = Pose()
    msg.position.x = tup[0][0]
    msg.position.y = tup[0][1]
    msg.position.z = tup[0][2]
    msg.orientation.x = tup[1][1]
    msg.orientation.y = tup[1][2]
    msg.orientation.z = tup[1][2]
    msg.orientation.w = tup[1][3]
    return msg
  
  def get_rel_pose(self, p1, p2):
      """Compute relative pose of p2 in frame of p1 (4x4 numpy arrays)"""      
      return numpy.dot(numpy.linalg.inv(p1), p2)

  def pose_msg_to_euler_list(self, Pose):
      """Returns a list representation of Pose [[vector],[euler angles]]"""
      return [[Pose.position.x,
               Pose.position.y,
               Pose.position.z],
               list(tf.transformations.euler_from_quaternion([Pose.orientation.x,
                                                              Pose.orientation.y,
                                                              Pose.orientation.z,
                                                              Pose.orientation.w]))]
  def euler_list_to_pose_msg(self, euler_pose):
      """ convert to pose represented as a list [[vector],[euler angles]] to a Pose msg"""
      quat = tf.transformations.quaternion_from_euler(*euler_pose[1])
      return Pose(position=Point(*euler_pose[0]),
                  orientation=Quaternion(*quat))
      
  def twist_msg_to_tup(self, msg):
      """Returns a twist tuple ((linear), (angular)) from parameters in a Twist message """
      return ((msg.linear.x, msg.linear.y, msg.linear.z),
              (msg.angular.x, msg.angular.y, msg.angular.z))

  def twist_tup_to_msg(self, tup):
      """ Returns a Twist message from parameters in a tuple ((linear), (angular))"""
      msg=Twist()
      msg.linear.x = tup[0][0]
      msg.linear.y = tup[0][1]
      msg.linear.z = tup[0][2]
      msg.angular.x = tup[1][0]
      msg.angular.y = tup[1][1]
      msg.angular.z = tup[1][2]  
      return msg
        
  def interpolate_pose_tup(self, p1, p2, blend=0.5):
      """Returns weighted interpolation of two pose tuples ((vector), (quaternion))"""
      linear = blend*numpy.array(list(p1[0])) + (1-blend)*numpy.array(list(p2[0]))
      angular = tf.transformations.quaternion_slerp(p1[1], p2[1], blend)
      return (linear.tolist(), angular.tolist())
  
  def interpolate_twist_tup(self, t1, t2, blend=0.5):
      """Returns a weighted average of two twist tuples ((linear), (angular))"""
      linear = blend*numpy.array(list(t1[0])) + (1-blend)*numpy.array(list(t2[0]))
      angular = blend*numpy.array(list(t1[1])) + (1-blend)*numpy.array(list(t2[1]))
      return (linear.tolist(), angular.tolist())

  def get_tf_transform(self, ref_frame, target_frame):
      """An exception-handling method for querying a transform from tf"""
      try:
          self._tl.waitForTransform(ref_frame, target_frame, rospy.Time(0), rospy.Duration(4.0))
          return self._tl.lookupTransform(ref_frame, target_frame, rospy.Time(0)) 
      except (tf.Exception, tf.LookupException, tf.ConnectivityException) as ex:
          print ex
          return None
  
  def get_l_drift_twist(self):
      """ Uses pose call to solver to compute error at left gripper, and returns as twist
      """
      #TODO: debug this
      
      # Compute target pose for left arm
      #r_pose_tup = self.get_tf_transform('base_footprint', 'r_gripper_tool_frame')
      #r_pose_mtr = self.pose_tup_to_matrix(r_pose_tup)
      #l_pose_goal =  
      
      # Compute relative transform to actual pose 
      ##########################################################
      # Compute actual relative pose of l_gripper in r_gripper frame
      #r_pose_tup = self.get_tf_transform('base_footprint', 'r_gripper_tool_frame')
      #l_pose_tup = self.get_tf_transform('base_footprint', 'l_gripper_tool_frame')
      #r_pose_mtr = self.pose_tup_to_matrix(r_pose_tup)
      #l_pose_mtr = self.pose_tup_to_matrix(l_pose_tup)
      #l_relpose_actual = self.get_rel_pose(r_pose_mtr, l_pose_mtr)
      
      # Compute transform from expected to actual
      #l_pose_err = self.get_rel_pose(self._l_relpose, l_relpose_actual)
      
      # Convert to euler for interpretation as a twist tuple      
      #return (tuple(tf.transformations.translation_from_matrix(l_pose_err)),
      #        tf.transformations.euler_from_matrix(l_pose_err))
      return ((0,0,0),(0,0,0))
      
  def get_twist_SSE(self, twist):
    return sum([pow(twist.linear.x,2),
               pow(twist.linear.y,2),
               pow(twist.linear.z,2),
               pow(twist.angular.x,2),
               pow(twist.angular.y,2),
               pow(twist.angular.z,2)])
  
  def _check_cart_pose(self, pose):
    """ Returns true if pose is reachable given workspace constraints of cart"""
    if pose[0][0] < self._cart_x_min or pose[0][0] > self._cart_x_max or \
       pose[0][1] < self._cart_y_min or pose[0][1] > self._cart_y_max or \
       pose[1][2] < self._cart_t_min or pose[1][2] > self._cart_t_max:
       return False
    
    return True


############################################################
# Non-member utility functions
############################################################

def forward_simulate_twist (pose, twist, t):
    """Forward simulate twist from pose for t seconds and return the resulting pose"""
    return [[pose[0][0] + twist.linear.x*t,
             pose[0][1] + twist.linear.y*t,
             pose[0][2] + twist.linear.z*t],
            [pose[1][0] + twist.angular.x*t,
             pose[1][1] + twist.angular.y*t,
             pose[1][2] + twist.angular.z*t]]


############################################################
# Node
############################################################
  
if __name__ == '__main__':
  from optparse import OptionParser
  parser = OptionParser(usage="")
  parser.add_option("-g", action="store_true", dest="use_tf_grippers", default=False,
                    help="use current gripper poses for initial grasp transforms (otherwise leaves defaults from param server)")
  parser.add_option("-m", "--midpoint", action="store_true", dest="midpoint", default=False,
                    help="set object initial pose to midpoint between grippers [default=%default]")
  parser.add_option("-o", "--object", dest="object", default="",
                    help="attempt to load an initial object transform with name OBJECT from tf [default=\"%default\"]")
  (options, args) = parser.parse_args()
  
  rospy.init_node(PKG+"_server")
  server = articulateCartServer(options.use_tf_grippers, options.midpoint, options.object)
  
  rospy.spin()


  
