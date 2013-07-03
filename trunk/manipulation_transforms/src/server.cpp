/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 * \file server.cpp
 *
 * Uses tf and ManipulationTransforms to provide a service interface
 * to ManipulationTransforms's query methods
 *
 * \date Jun 28, 2010
 * \author jscholz
 */

#include <LinearMath/btTransform.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "manipulation_transforms/manipulation_transforms_ros.h"

#include <argp.h>

 const char *argp_program_version = "manipulation_transforms_server 0.1";
 const char *argp_program_bug_address = "<jscholz@willowgarage.com>";

 /* Program documentation. */
 static char doc[] =
		"Provides a rose service node for mapping between object and gripper poses.  \n\n"
		"This node must be initialized with transforms in a common reference frame for the target "
		 "object and each gripper in contact with it.  "
		 "After initialization, manipulation_transforms_server provides service calls for querying poses, "
		 "twists, and trajectories from object to grippers, or grippers to object.  \n"
		 "If not called with the \"-l\" option, the node must be initialized by a call to the service"
		 "\'SetGraspTransforms\' before issuing queries.  ";

 /* A description of the arguments we accept. */
 static char args_doc[] = "[--load=NAMESPACE]";

 /* The options we understand. */
 static struct argp_option options[] = {
   {"load",  'l', 0, 0,  "Attempt to load the rigid transforms from parameter server (see doxy for naming convention)", 0},
   {"namespace",  'n', "NAMESPACE", 0,  "Use NAMESPACE as the top-level namespace when loading rigid transforms" , 0},
   {"frame",  'f', "FRAME", 0,  "Set the reference frame in which all initial and query transforms must be defined" , 0},
   {0,0,0,0,0,0}
 };

 /* Used by main to communicate with parse_opt. */
 struct arguments
 {
   char *args[2];                /* arg1 & arg2 */
   bool load_params;
   char *ns;
   char *reference_frame;
 };

 unsigned int nargs_required = 0;
 unsigned int nargs_max = 3;

 /* Parse a single option. */
 static error_t parse_opt (int key, char *arg, struct argp_state *state)
 {
	 /* Get the input argument from argp_parse, which we
	 know is a pointer to our arguments structure. */
	struct arguments *arguments;
	arguments = (struct arguments*) state->input;

	switch (key) {
	case 'l':
		arguments->load_params = true;
		break;

	case 'n':
		arguments->ns = arg;
		break;

	case 'f':
		arguments->reference_frame = arg;
		break;

	case ARGP_KEY_ARG:
		if (state->arg_num >= nargs_max)
			/* Too many arguments. */
			argp_usage(state);
		arguments->args[state->arg_num] = arg;
		break;

	case ARGP_KEY_END:
		if (state->arg_num < nargs_required)
			/* Not enough arguments. */
			argp_usage(state);
		break;

	default:
		//arguments->load_params = false;
		return ARGP_ERR_UNKNOWN;
	}
	return 0;
 }

 /* Our argp parser. */
 static struct argp argp = { options, parse_opt, args_doc, doc, NULL, NULL, NULL };

/* ---- */
/* MAIN */
/* ---- */
int main( int argc, char **argv ) {

  ros::init(argc, argv, "manipulation_transforms_server");
  /*
   * Initialize server solver object
   * Reads initialization transforms for solver from parameter
   * server, and sets up a pair of service calls for interacting
   * with the solver
   */

  struct arguments arguments;

  /* Default values. */
  arguments.load_params = false;
  arguments.ns = (char*)"";
  arguments.reference_frame = (char*)"base_footprint";

  argp_parse (&argp, argc, argv, 0, 0, &arguments);

  ROS_INFO("Starting manipulation_transforms_server node with FRAME = %s, NAMESPACE = %s, load params = %s",
		   arguments.reference_frame,
		   arguments.ns,
		   arguments.load_params ? "false" : "false");

  if (ros::isInitialized()) {
	  if (arguments.load_params) {
		  ManipulationTransformsROS server(arguments.reference_frame, arguments.ns);
		  ros::spin();
	  }
	  else {
		  ManipulationTransformsROS server(arguments.reference_frame);
		  ros::spin();
	  }
  }
  return 0;
}

