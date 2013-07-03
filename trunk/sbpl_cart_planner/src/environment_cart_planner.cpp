/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <iostream>
using namespace std;
#include <ros/ros.h>
#include <sbpl/headers.h>
#include <sbpl_cart_planner/environment_cart_planner.h>

#if TIME_DEBUG
static clock_t time3_addallout = 0;
static clock_t time_gethash = 0;
static clock_t time_createhash = 0;
static clock_t time_getsuccs = 0;
#endif

static long int checks_cart = 0; 

#define XYTHETACART2INDEX(X,Y,THETA,CARTANGLE) (THETA + X*NAVXYTHETACARTLAT_THETADIRS + Y*EnvNAVXYTHETACARTLATCfg.EnvWidth_c*NAVXYTHETACARTLAT_THETADIRS + CARTANGLE*EnvNAVXYTHETACARTLATCfg.EnvWidth_c*EnvNAVXYTHETACARTLATCfg.EnvHeight_c*NAVXYTHETACARTLAT_THETADIRS)

//converts discretized version of angle into continuous (radians)
//maps 0->0, 1->delta, 2->2*delta, ...
double CartDiscTheta2Cont(int nTheta, int NUMOFANGLEVALS)
{
  double thetaBinSize = (2*MAX_CART_ANGLE)/(CART_THETADIRS-1);
  return -MAX_CART_ANGLE+nTheta*thetaBinSize;
}

//converts continuous (radians) version of angle into discrete
//maps 0->0, [delta/2, 3/2*delta)->1, [3/2*delta, 5/2*delta)->2,...
int CartContTheta2Disc(double fTheta, int NUMOFANGLEVALS)
{
  if(fTheta < -MAX_CART_ANGLE)
    return 0;
  else if (fTheta > MAX_CART_ANGLE)
    return (CART_THETADIRS-1);
  else
  {
    double thetaBinSize = (2*MAX_CART_ANGLE)/(CART_THETADIRS-1);    
    return (int)((fTheta+MAX_CART_ANGLE+thetaBinSize/2.0)/thetaBinSize);
  }
}


//-----------------constructors/destructors-------------------------------
EnvironmentNAVXYTHETACARTLATTICE::EnvironmentNAVXYTHETACARTLATTICE(): initialized_(true)
{
	EnvNAVXYTHETACARTLATCfg.obsthresh = ENVNAVXYTHETACARTLAT_DEFAULTOBSTHRESH;
	EnvNAVXYTHETACARTLATCfg.cost_inscribed_thresh = EnvNAVXYTHETACARTLATCfg.obsthresh; //the value that pretty much makes it disabled
	EnvNAVXYTHETACARTLATCfg.cost_possibly_circumscribed_thresh = -1; //the value that pretty much makes it disabled

	grid2Dsearchfromstart = NULL;
	grid2Dsearchfromgoal = NULL;
	bNeedtoRecomputeStartHeuristics = true;
	bNeedtoRecomputeGoalHeuristics = true;
	iteration = 0;

	EnvNAVXYTHETACARTLAT.bInitialized = false;

	EnvNAVXYTHETACARTLATCfg.actionwidth = NAVXYTHETACARTLAT_DEFAULT_ACTIONWIDTH;

	//no memory allocated in cfg yet
	EnvNAVXYTHETACARTLATCfg.Grid2D = NULL;
	EnvNAVXYTHETACARTLATCfg.ActionsV = NULL;
	EnvNAVXYTHETACARTLATCfg.PredActionsV = NULL;
}

EnvironmentNAVXYTHETACARTLATTICE::~EnvironmentNAVXYTHETACARTLATTICE()
{
	ROS_DEBUG("Destroying XYTHETACARTLATTICE");
	if(grid2Dsearchfromstart != NULL)
		delete grid2Dsearchfromstart;
	grid2Dsearchfromstart = NULL;

	if(grid2Dsearchfromgoal != NULL)
		delete grid2Dsearchfromgoal;
	grid2Dsearchfromgoal = NULL;

	if(EnvNAVXYTHETACARTLATCfg.Grid2D != NULL)
	{	
		for (int x = 0; x < EnvNAVXYTHETACARTLATCfg.EnvWidth_c; x++) 
			delete [] EnvNAVXYTHETACARTLATCfg.Grid2D[x];
		delete [] EnvNAVXYTHETACARTLATCfg.Grid2D;
		EnvNAVXYTHETACARTLATCfg.Grid2D = NULL;
	}

	//delete actions
	if(EnvNAVXYTHETACARTLATCfg.ActionsV != NULL)
	{
		for(int tind = 0; tind < NAVXYTHETACARTLAT_THETADIRS; tind++)
			delete [] EnvNAVXYTHETACARTLATCfg.ActionsV[tind];
		delete [] EnvNAVXYTHETACARTLATCfg.ActionsV;
		EnvNAVXYTHETACARTLATCfg.ActionsV = NULL;
	}
	if(EnvNAVXYTHETACARTLATCfg.PredActionsV != NULL)
	{
		delete [] EnvNAVXYTHETACARTLATCfg.PredActionsV;
		EnvNAVXYTHETACARTLATCfg.PredActionsV = NULL;
	}
}

//---------------------------------------------------------------------


//-------------------problem specific and local functions---------------------


static unsigned int inthash(unsigned int key)
{
  key += (key << 12);
  key ^= (key >> 22);
  key += (key << 4);
  key ^= (key >> 9);
  key += (key << 10);
  key ^= (key >> 2);
  key += (key << 7);
  key ^= (key >> 12);
  return key;
}



void EnvironmentNAVXYTHETACARTLATTICE::SetConfiguration(int width, 
                                                        int height,
                                                        const unsigned char* mapdata,
                                                        int startx, 
                                                        int starty, 
                                                        int starttheta, 
                                                        int startcartangle,
                                                        int goalx, 
                                                        int goaly, 
                                                        int goaltheta, 
                                                        int goalcartangle,
                                                        double cellsize_m, 
                                                        double nominalvel_mpersecs, 
                                                        double timetoturn45degsinplace_secs, 
                                                        const vector<sbpl_2Dpt_t> & robot_perimeterV,
                                                        const vector<sbpl_2Dpt_t> & cart_perimeterV,
                                                        const sbpl_2Dpt_t &cart_offset) 
{
  EnvNAVXYTHETACARTLATCfg.EnvWidth_c = width;
  EnvNAVXYTHETACARTLATCfg.EnvHeight_c = height;
  EnvNAVXYTHETACARTLATCfg.StartX_c = startx;
  EnvNAVXYTHETACARTLATCfg.StartY_c = starty;
  EnvNAVXYTHETACARTLATCfg.StartTheta = starttheta;
  EnvNAVXYTHETACARTLATCfg.StartCartAngle = startcartangle;

 
  if(EnvNAVXYTHETACARTLATCfg.StartX_c < 0 || EnvNAVXYTHETACARTLATCfg.StartX_c >= EnvNAVXYTHETACARTLATCfg.EnvWidth_c) {
    ROS_ERROR("illegal start coordinates");
    initialized_ = false;;
    return;
  }
  if(EnvNAVXYTHETACARTLATCfg.StartY_c < 0 || EnvNAVXYTHETACARTLATCfg.StartY_c >= EnvNAVXYTHETACARTLATCfg.EnvHeight_c) {
    ROS_ERROR("illegal start coordinates");
    initialized_ = false;;
    return;
  }
  if(EnvNAVXYTHETACARTLATCfg.StartTheta < 0 || EnvNAVXYTHETACARTLATCfg.StartTheta >= NAVXYTHETACARTLAT_THETADIRS) {
    ROS_ERROR("illegal start coordinates for theta");
    initialized_ = false;;
    return;
  }
  if(EnvNAVXYTHETACARTLATCfg.StartCartAngle < 0 || EnvNAVXYTHETACARTLATCfg.StartCartAngle >= CART_THETADIRS) {
    ROS_ERROR("illegal start coordinates for theta");
    initialized_ = false;;
    return;
  }
  
  EnvNAVXYTHETACARTLATCfg.EndX_c = goalx;
  EnvNAVXYTHETACARTLATCfg.EndY_c = goaly;
  EnvNAVXYTHETACARTLATCfg.EndTheta = goaltheta;
  EnvNAVXYTHETACARTLATCfg.EndCartAngle = goalcartangle;

  if(EnvNAVXYTHETACARTLATCfg.EndX_c < 0 || EnvNAVXYTHETACARTLATCfg.EndX_c >= EnvNAVXYTHETACARTLATCfg.EnvWidth_c) {
    ROS_ERROR("illegal goal coordinates");
    initialized_ = false;;
    return;
  }
  if(EnvNAVXYTHETACARTLATCfg.EndY_c < 0 || EnvNAVXYTHETACARTLATCfg.EndY_c >= EnvNAVXYTHETACARTLATCfg.EnvHeight_c) {
    ROS_ERROR("illegal goal coordinates");
    initialized_ = false;;
    return;
  }
  if(EnvNAVXYTHETACARTLATCfg.EndTheta < 0 || EnvNAVXYTHETACARTLATCfg.EndTheta >= NAVXYTHETACARTLAT_THETADIRS) {
    ROS_ERROR("illegal goal coordinates for theta");
    initialized_ = false;;
    return;
  }
  if(EnvNAVXYTHETACARTLATCfg.EndCartAngle < 0 || EnvNAVXYTHETACARTLATCfg.EndCartAngle >= CART_THETADIRS) {
    ROS_ERROR("illegal goal coordinates for theta");
    initialized_ = false;;
    return;
  }

  EnvNAVXYTHETACARTLATCfg.FootprintPolygon = robot_perimeterV;
  EnvNAVXYTHETACARTLATCfg.CartPolygon = cart_perimeterV;
  EnvNAVXYTHETACARTLATCfg.CartOffset = cart_offset;

  EnvNAVXYTHETACARTLATCfg.CartCenterOffset.x = 0.0;
  EnvNAVXYTHETACARTLATCfg.CartCenterOffset.y = 0.0;

  for(unsigned int i= 0; i < cart_perimeterV.size(); i++)
  {
    EnvNAVXYTHETACARTLATCfg.CartCenterOffset.x += cart_perimeterV[i].x;
    EnvNAVXYTHETACARTLATCfg.CartCenterOffset.y += cart_perimeterV[i].y;
  }

  EnvNAVXYTHETACARTLATCfg.CartCenterOffset.x = EnvNAVXYTHETACARTLATCfg.CartCenterOffset.x/cart_perimeterV.size();
  EnvNAVXYTHETACARTLATCfg.CartCenterOffset.y = EnvNAVXYTHETACARTLATCfg.CartCenterOffset.y/cart_perimeterV.size();

  EnvNAVXYTHETACARTLATCfg.nominalvel_mpersecs = nominalvel_mpersecs;
  EnvNAVXYTHETACARTLATCfg.cellsize_m = cellsize_m;
  EnvNAVXYTHETACARTLATCfg.timetoturn45degsinplace_secs = timetoturn45degsinplace_secs;


  //allocate the 2D environment
  EnvNAVXYTHETACARTLATCfg.Grid2D = new unsigned char* [EnvNAVXYTHETACARTLATCfg.EnvWidth_c];
  for (int x = 0; x < EnvNAVXYTHETACARTLATCfg.EnvWidth_c; x++) {
    EnvNAVXYTHETACARTLATCfg.Grid2D[x] = new unsigned char [EnvNAVXYTHETACARTLATCfg.EnvHeight_c];
  }
  
  //environment:
  if (0 == mapdata) {
    for (int y = 0; y < EnvNAVXYTHETACARTLATCfg.EnvHeight_c; y++) {
      for (int x = 0; x < EnvNAVXYTHETACARTLATCfg.EnvWidth_c; x++) {
	EnvNAVXYTHETACARTLATCfg.Grid2D[x][y] = 0;
      }
    }
  }
  else {
    for (int y = 0; y < EnvNAVXYTHETACARTLATCfg.EnvHeight_c; y++) {
      for (int x = 0; x < EnvNAVXYTHETACARTLATCfg.EnvWidth_c; x++) {
			EnvNAVXYTHETACARTLATCfg.Grid2D[x][y] = mapdata[x+y*width];
      }
    }
  }
}

void EnvironmentNAVXYTHETACARTLATTICE::ReadConfiguration(FILE* fCfg)
{
	//read in the configuration of environment and initialize  EnvNAVXYTHETACARTLATCfg structure
	char sTemp[1024], sTemp1[1024];
	int dTemp;
	int x, y;

	//discretization(cells)
	if(fscanf(fCfg, "%s", sTemp) !=1)
  {
    ROS_ERROR("Incorrect format of config file");
		initialized_ = false;
    return;
  }

	strcpy(sTemp1, "discretization(cells):");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		ROS_ERROR("configuration file has incorrect format");
    ROS_ERROR("Expected %s got %s", sTemp1, sTemp);
		initialized_ = false;
    return;
	}

	if(fscanf(fCfg, "%s", sTemp) != 1)
  {
    ROS_ERROR("Incorrect format of config file");
		initialized_ = false;
    return;
  }
    
	EnvNAVXYTHETACARTLATCfg.EnvWidth_c = atoi(sTemp);
	if(fscanf(fCfg, "%s", sTemp) != 1)
  {
    ROS_ERROR("Incorrect format of config file");
		initialized_ = false;
    return;
  }

	EnvNAVXYTHETACARTLATCfg.EnvHeight_c = atoi(sTemp);

	//obsthresh: 
	if(fscanf(fCfg, "%s", sTemp) != 1)
  {
    ROS_ERROR("Incorrect format of config file");
		initialized_ = false;
    return;
  }

	strcpy(sTemp1, "obsthresh:");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		ROS_ERROR("Configuration file has incorrect format");
		ROS_ERROR("Expected %s got %s", sTemp1, sTemp);
		ROS_ERROR("See existing examples of env files for the right format of heading");
		initialized_ = false;;
    return;
	}
	if(fscanf(fCfg, "%s", sTemp) != 1)
  {
    ROS_ERROR("Incorrect format of config file");
		initialized_ = false;
    return;
  }

	EnvNAVXYTHETACARTLATCfg.obsthresh = atoi(sTemp);
	ROS_INFO("obsthresh = %d", EnvNAVXYTHETACARTLATCfg.obsthresh);

	//cost_inscribed_thresh: 
	if(fscanf(fCfg, "%s", sTemp) != 1)
  {
    ROS_ERROR("Incorrect format of config file");
		initialized_ = false;
    return;
  }

	strcpy(sTemp1, "cost_inscribed_thresh:");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		ROS_ERROR("Configuration file has incorrect format");
		ROS_ERROR("Expected %s got %s", sTemp1, sTemp);
		ROS_ERROR("See existing examples of env files for the right format of heading");
		initialized_ = false;
    return;   
	}
	if(fscanf(fCfg, "%s", sTemp) != 1)
  {
    ROS_ERROR("Incorrect format of config file");
		initialized_ = false;
    return;
  }

	EnvNAVXYTHETACARTLATCfg.cost_inscribed_thresh = atoi(sTemp);
	ROS_INFO("cost_inscribed_thresh = %d", EnvNAVXYTHETACARTLATCfg.cost_inscribed_thresh);


	//cost_possibly_circumscribed_thresh: 
	if(fscanf(fCfg, "%s", sTemp) != 1)
  {
    ROS_ERROR("Incorrect format of config file");
		initialized_ = false;
    return;
  }

	strcpy(sTemp1, "cost_possibly_circumscribed_thresh:");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		ROS_ERROR("Configuration file has incorrect format");
		ROS_ERROR("Expected %s got %s", sTemp1, sTemp);
		ROS_ERROR("See existing examples of env files for the right format of heading");
		initialized_ = false;;
    return;
	}
	if(fscanf(fCfg, "%s", sTemp) != 1)
  {
    ROS_ERROR("Incorrect format of config file");
		initialized_ = false;
    return;
  }

	EnvNAVXYTHETACARTLATCfg.cost_possibly_circumscribed_thresh = atoi(sTemp);
	ROS_INFO("cost_possibly_circumscribed_thresh = %d", EnvNAVXYTHETACARTLATCfg.cost_possibly_circumscribed_thresh);

	
	//cellsize
	if(fscanf(fCfg, "%s", sTemp) != 1)
  {
    ROS_ERROR("Incorrect format of config file");
		initialized_ = false;
    return;
  }

	strcpy(sTemp1, "cellsize(meters):");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		ROS_ERROR("Configuration file has incorrect format");
		ROS_ERROR("Expected %s got %s", sTemp1, sTemp);
		initialized_ = false;
    return;
	}
	if(fscanf(fCfg, "%s", sTemp) != 1)
  {
    ROS_ERROR("incorrect format of config file");
		initialized_ = false;
    return;
  }

	EnvNAVXYTHETACARTLATCfg.cellsize_m = atof(sTemp);
	
	//speeds
	if(fscanf(fCfg, "%s", sTemp) != 1)
  {
    ROS_ERROR("Incorrect format of config file");
		initialized_ = false;
    return;
  }

	strcpy(sTemp1, "nominalvel(mpersecs):");
	if(strcmp(sTemp1, sTemp) != 0)
	{
	  ROS_ERROR("Configuration file has incorrect format");
		ROS_ERROR("Expected %s got %s", sTemp1, sTemp);
		initialized_ = false;
    return;
	}
	if(fscanf(fCfg, "%s", sTemp) != 1)
  {
    ROS_ERROR("Incorrect format of config file");
		initialized_ = false;
    return;
  }

	EnvNAVXYTHETACARTLATCfg.nominalvel_mpersecs = atof(sTemp);
	if(fscanf(fCfg, "%s", sTemp) != 1)
  {
    ROS_ERROR("Incorrect format of config file");
		initialized_ = false;
    return;
  }

	strcpy(sTemp1, "timetoturn45degsinplace(secs):");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		ROS_ERROR("Configuration file has incorrect format");
		ROS_ERROR("Expected %s got %s", sTemp1, sTemp);
		initialized_ = false;
    return;
	}
	if(fscanf(fCfg, "%s", sTemp) != 1)
  {
    ROS_ERROR("Incorrect format of config file");
		initialized_ = false;
    return;
  }

	EnvNAVXYTHETACARTLATCfg.timetoturn45degsinplace_secs = atof(sTemp);


	//start(meters,rads): 
	if(fscanf(fCfg, "%s", sTemp) != 1)
  {
    ROS_ERROR("Incorrect format of config file");
		initialized_ = false;
    return;
  }

	if(fscanf(fCfg, "%s", sTemp) != 1)
  {
    ROS_ERROR("Incorrect format of config file");
		initialized_ = false;
    return;
  }

	EnvNAVXYTHETACARTLATCfg.StartX_c = CONTXY2DISC(atof(sTemp),EnvNAVXYTHETACARTLATCfg.cellsize_m);
	if(fscanf(fCfg, "%s", sTemp) != 1)
  {
    ROS_ERROR("Incorrect format of config file");
		initialized_ = false;
    return;
  }

	EnvNAVXYTHETACARTLATCfg.StartY_c = CONTXY2DISC(atof(sTemp),EnvNAVXYTHETACARTLATCfg.cellsize_m);
	if(fscanf(fCfg, "%s", sTemp) != 1)
  {
    ROS_ERROR("Incorrect format of config file");
		initialized_ = false;
    return;
  }

	EnvNAVXYTHETACARTLATCfg.StartTheta = ContTheta2Disc(atof(sTemp), NAVXYTHETACARTLAT_THETADIRS);
	if(fscanf(fCfg, "%s", sTemp) != 1)
  {
    ROS_ERROR("ERROR: incorrect format of config file");
		initialized_ = false;
    return;
  }

	EnvNAVXYTHETACARTLATCfg.StartCartAngle = CartContTheta2Disc(atof(sTemp), CART_THETADIRS);


	if(EnvNAVXYTHETACARTLATCfg.StartX_c < 0 || EnvNAVXYTHETACARTLATCfg.StartX_c >= EnvNAVXYTHETACARTLATCfg.EnvWidth_c)
	{
		ROS_ERROR("Illegal start coordinates");
		initialized_ = false;
    return;
	}
	if(EnvNAVXYTHETACARTLATCfg.StartY_c < 0 || EnvNAVXYTHETACARTLATCfg.StartY_c >= EnvNAVXYTHETACARTLATCfg.EnvHeight_c)
	{
		ROS_ERROR("Illegal start coordinates");
		initialized_ = false;
    return;
	}
	if(EnvNAVXYTHETACARTLATCfg.StartTheta < 0 || EnvNAVXYTHETACARTLATCfg.StartTheta >= NAVXYTHETACARTLAT_THETADIRS) {
		ROS_ERROR("illegal start coordinates for theta");
		initialized_ = false;
    return;
	}
	if(EnvNAVXYTHETACARTLATCfg.StartCartAngle < 0 || EnvNAVXYTHETACARTLATCfg.StartCartAngle >= CART_THETADIRS) {
		ROS_ERROR("illegal start coordinates for theta");
		initialized_ = false;
    return;
	}

	//end(meters,rads): 
	if(fscanf(fCfg, "%s", sTemp) != 1)
  {
    ROS_ERROR("incorrect format of config file");
		initialized_ = false;
    return;
  }

	if(fscanf(fCfg, "%s", sTemp) != 1)
  {
    ROS_ERROR("incorrect format of config file");
		initialized_ = false;
    return;
  }

	EnvNAVXYTHETACARTLATCfg.EndX_c = CONTXY2DISC(atof(sTemp),EnvNAVXYTHETACARTLATCfg.cellsize_m);
	if(fscanf(fCfg, "%s", sTemp) != 1)
  {
    ROS_ERROR("incorrect format of config file");
		initialized_ = false;
    return;
  }

	EnvNAVXYTHETACARTLATCfg.EndY_c = CONTXY2DISC(atof(sTemp),EnvNAVXYTHETACARTLATCfg.cellsize_m);
	if(fscanf(fCfg, "%s", sTemp) != 1)
  {
    ROS_ERROR("incorrect format of config file");
		initialized_ = false;
    return;
  }

	EnvNAVXYTHETACARTLATCfg.EndTheta = ContTheta2Disc(atof(sTemp), NAVXYTHETACARTLAT_THETADIRS);;
	if(fscanf(fCfg, "%s", sTemp) != 1)
  {
    ROS_ERROR("incorrect format of config file");
		initialized_ = false;
    return;
  }

	EnvNAVXYTHETACARTLATCfg.EndCartAngle = CartContTheta2Disc(atof(sTemp), CART_THETADIRS);;

	if(EnvNAVXYTHETACARTLATCfg.EndX_c < 0 || EnvNAVXYTHETACARTLATCfg.EndX_c >= EnvNAVXYTHETACARTLATCfg.EnvWidth_c)
	{
		ROS_ERROR("illegal end coordinates");
		initialized_ = false;
    return;
	}
	if(EnvNAVXYTHETACARTLATCfg.EndY_c < 0 || EnvNAVXYTHETACARTLATCfg.EndY_c >= EnvNAVXYTHETACARTLATCfg.EnvHeight_c)
	{
		ROS_ERROR("illegal end coordinates");
		initialized_ = false;
    return;
	}
	if(EnvNAVXYTHETACARTLATCfg.EndTheta < 0 || EnvNAVXYTHETACARTLATCfg.EndTheta >= NAVXYTHETACARTLAT_THETADIRS) {
		ROS_ERROR("illegal goal coordinates for theta");
		initialized_ = false;
    return;
	}
	if(EnvNAVXYTHETACARTLATCfg.EndCartAngle < 0 || EnvNAVXYTHETACARTLATCfg.EndCartAngle >= CART_THETADIRS) {
		ROS_ERROR("illegal goal coordinates for theta");
		initialized_ = false;
    return;
	}

	//allocate the 2D environment
	EnvNAVXYTHETACARTLATCfg.Grid2D = new unsigned char* [EnvNAVXYTHETACARTLATCfg.EnvWidth_c];
	for (x = 0; x < EnvNAVXYTHETACARTLATCfg.EnvWidth_c; x++)
	{
		EnvNAVXYTHETACARTLATCfg.Grid2D[x] = new unsigned char [EnvNAVXYTHETACARTLATCfg.EnvHeight_c];
	}

	//environment:
	if(fscanf(fCfg, "%s", sTemp) != 1)
  {
    ROS_ERROR("incorrect format of config file");
		initialized_ = false;
    return;
  }

	for (y = 0; y < EnvNAVXYTHETACARTLATCfg.EnvHeight_c; y++)
		for (x = 0; x < EnvNAVXYTHETACARTLATCfg.EnvWidth_c; x++)
		{
			if(fscanf(fCfg, "%d", &dTemp) != 1)
			{
				ROS_ERROR("incorrect format of config file");
        initialized_ = false;
        return;
			}
			EnvNAVXYTHETACARTLATCfg.Grid2D[x][y] = dTemp;
		}
}

bool EnvironmentNAVXYTHETACARTLATTICE::ReadinCell(EnvNAVXYTHETACARTLAT3Dcell_t* cell, FILE* fIn)
{
   char sTemp[60];

	if(fscanf(fIn, "%s", sTemp) == 0)
	   return false;
	cell->x = atoi(sTemp);
	if(fscanf(fIn, "%s", sTemp) == 0)
	   return false;
	cell->y = atoi(sTemp);
	if(fscanf(fIn, "%s", sTemp) == 0)
	   return false;
	cell->theta = atoi(sTemp);
    //normalize the angle
	cell->theta = NORMALIZEDISCTHETA(cell->theta, NAVXYTHETACARTLAT_THETADIRS);

	if(fscanf(fIn, "%s", sTemp) == 0)
	   return false;
	cell->cartangle = atoi(sTemp);
    //normalize the angle
  //	cell->cartangle = NORMALIZEDISCTHETA(cell->cartangle, CART_THETADIRS);

	return true;
}

bool EnvironmentNAVXYTHETACARTLATTICE::ReadinPose(EnvNAVXYTHETACARTLAT3Dpt_t* pose, FILE* fIn)
{
   char sTemp[60];

	if(fscanf(fIn, "%s", sTemp) == 0)
	   return false;
	pose->x = atof(sTemp);
	if(fscanf(fIn, "%s", sTemp) == 0)
	   return false;
	pose->y = atof(sTemp);

	if(fscanf(fIn, "%s", sTemp) == 0)
	   return false;
	pose->theta = atof(sTemp);
	pose->theta = normalizeAngle(pose->theta);

	if(fscanf(fIn, "%s", sTemp) == 0)
	   return false;
	pose->cartangle = atof(sTemp);

	return true;
}

bool EnvironmentNAVXYTHETACARTLATTICE::ReadinMotionPrimitive(SBPL_xythetacart_mprimitive* pMotPrim, FILE* fIn)
{
    char sTemp[1024];
    int dTemp;
    char sExpected[1024];
    int numofIntermPoses;

    //read in actionID
    strcpy(sExpected, "primID:");
    if(fscanf(fIn, "%s", sTemp) == 0)
        return false;
    if(strcmp(sTemp, sExpected) != 0){
        ROS_ERROR("expected %s but got %s", sExpected, sTemp);
        return false;
    }
    if(fscanf(fIn, "%d", &pMotPrim->motprimID) != 1)
        return false;

    //read in start angle
    strcpy(sExpected, "startangle_c:");
    if(fscanf(fIn, "%s", sTemp) == 0)
        return false;
    if(strcmp(sTemp, sExpected) != 0){
        ROS_ERROR("expected %s but got %s", sExpected, sTemp);
        return false;
    }
   if(fscanf(fIn, "%d", &dTemp) == 0)
   {
	   ROS_ERROR("ERROR reading startangle");
     return false;	
   }
   pMotPrim->starttheta_c = dTemp;
 
   //read in end pose
   strcpy(sExpected, "endpose_c:");
   if(fscanf(fIn, "%s", sTemp) == 0)
       return false;
   if(strcmp(sTemp, sExpected) != 0){
       ROS_ERROR("expected %s but got %s", sExpected, sTemp);
       return false;
   }

   if(ReadinCell(&pMotPrim->endcell, fIn) == false){
		ROS_ERROR("failed to read in endsearchpose");
        return false;
   }
   
    //read in action cost
    strcpy(sExpected, "additionalactioncostmult:");
    if(fscanf(fIn, "%s", sTemp) == 0)
        return false;
    if(strcmp(sTemp, sExpected) != 0){
        ROS_ERROR("expected %s but got %s", sExpected, sTemp);
        return false;
    }
    if(fscanf(fIn, "%d", &dTemp) != 1)
        return false;
	pMotPrim->additionalactioncostmult = dTemp;
    
    //read in intermediate poses
    strcpy(sExpected, "intermediateposes:");
    if(fscanf(fIn, "%s", sTemp) == 0)
        return false;
    if(strcmp(sTemp, sExpected) != 0){
        ROS_ERROR("expected %s but got %s", sExpected, sTemp);
        return false;
    }
    if(fscanf(fIn, "%d", &numofIntermPoses) != 1)
        return false;
	//all intermposes should be with respect to 0,0 as starting pose since it will be added later and should be done 
	//after the action is rotated by initial orientation
    for(int i = 0; i < numofIntermPoses; i++){
        EnvNAVXYTHETACARTLAT3Dpt_t intermpose;
        if(ReadinPose(&intermpose, fIn) == false){
            ROS_ERROR("failed to read in intermediate poses");
            return false;
        }
		pMotPrim->intermptV.push_back(intermpose);
    }

	//check that the last pose corresponds correctly to the last pose
	EnvNAVXYTHETACARTLAT3Dpt_t sourcepose;
	sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETACARTLATCfg.cellsize_m);
	sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETACARTLATCfg.cellsize_m);
	sourcepose.theta = DiscTheta2Cont(pMotPrim->starttheta_c, NAVXYTHETACARTLAT_THETADIRS);
  sourcepose.cartangle = CartDiscTheta2Cont(0, CART_THETADIRS);

	double mp_endx_m = sourcepose.x + pMotPrim->intermptV[pMotPrim->intermptV.size()-1].x;
	double mp_endy_m = sourcepose.y + pMotPrim->intermptV[pMotPrim->intermptV.size()-1].y;
	double mp_endtheta_rad = pMotPrim->intermptV[pMotPrim->intermptV.size()-1].theta;				
	double mp_endcartangle_rad = pMotPrim->intermptV[pMotPrim->intermptV.size()-1].cartangle;				
	int endx_c = CONTXY2DISC(mp_endx_m, EnvNAVXYTHETACARTLATCfg.cellsize_m);
	int endy_c = CONTXY2DISC(mp_endy_m, EnvNAVXYTHETACARTLATCfg.cellsize_m);
	int endtheta_c = ContTheta2Disc(mp_endtheta_rad, NAVXYTHETACARTLAT_THETADIRS);
	int endcartangle_c = CartContTheta2Disc(mp_endcartangle_rad, CART_THETADIRS);
	if(endx_c != pMotPrim->endcell.x || endy_c != pMotPrim->endcell.y || endtheta_c != pMotPrim->endcell.theta || endcartangle_c != pMotPrim->endcell.cartangle)
	{	
		ROS_ERROR("incorrect primitive %d with startangle=%d last interm point %f %f %f %f does not match end pose %d %d %d %d", 
			pMotPrim->motprimID, pMotPrim->starttheta_c,
			pMotPrim->intermptV[pMotPrim->intermptV.size()-1].x, pMotPrim->intermptV[pMotPrim->intermptV.size()-1].y, pMotPrim->intermptV[pMotPrim->intermptV.size()-1].theta, pMotPrim->intermptV[pMotPrim->intermptV.size()-1].cartangle,
           pMotPrim->endcell.x, pMotPrim->endcell.y,pMotPrim->endcell.theta,pMotPrim->endcell.cartangle);	
			return false;
	}

  
    return true;
}


// TODO - do we need separate angular resolution for cart angle
bool EnvironmentNAVXYTHETACARTLATTICE::ReadMotionPrimitives(FILE* fMotPrims)
{
    char sTemp[1024], sExpected[1024];
    float fTemp;
    int dTemp;
    int totalNumofActions = 0;

    ROS_DEBUG("Reading in motion primitives..., this may take some time.");
    
    //read in the resolution
    strcpy(sExpected, "resolution_m:");
    if(fscanf(fMotPrims, "%s", sTemp) == 0)
        return false;
    if(strcmp(sTemp, sExpected) != 0){
        ROS_ERROR("expected %s but got %s", sExpected, sTemp);
        return false;
    }
    if(fscanf(fMotPrims, "%f", &fTemp) == 0)
        return false;
    if(fabs(fTemp-EnvNAVXYTHETACARTLATCfg.cellsize_m) > ERR_EPS){
        ROS_ERROR("invalid resolution %f (instead of %f) in the dynamics file", 
               fTemp, EnvNAVXYTHETACARTLATCfg.cellsize_m);
        return false;
    }

    //read in the angular resolution
    strcpy(sExpected, "numberofangles:");
    if(fscanf(fMotPrims, "%s", sTemp) == 0)
        return false;
    if(strcmp(sTemp, sExpected) != 0){
        ROS_ERROR("expected %s but got %s", sExpected, sTemp);
        return false;
    }
    if(fscanf(fMotPrims, "%d", &dTemp) == 0)
        return false;
    if(dTemp != NAVXYTHETACARTLAT_THETADIRS){
        ROS_ERROR("invalid angular resolution %d angles (instead of %d angles) in the motion primitives file", 
               dTemp, NAVXYTHETACARTLAT_THETADIRS);
        return false;
    }


    //read in the total number of actions
    strcpy(sExpected, "totalnumberofprimitives:");
    if(fscanf(fMotPrims, "%s", sTemp) == 0)
        return false;
    if(strcmp(sTemp, sExpected) != 0){
        ROS_ERROR("expected %s but got %s", sExpected, sTemp);
        return false;
    }
    if(fscanf(fMotPrims, "%d", &totalNumofActions) == 0){
        return false;
    }

    for(int i = 0; i < totalNumofActions; i++){
		SBPL_xythetacart_mprimitive motprim;

		if(EnvironmentNAVXYTHETACARTLATTICE::ReadinMotionPrimitive(&motprim, fMotPrims) == false)
			return false;

		EnvNAVXYTHETACARTLATCfg.mprimV.push_back(motprim);

    }
    ROS_INFO("Done reading motion primitives");
    
    return true;
}

// TODO - does this need to be filled up
void EnvironmentNAVXYTHETACARTLATTICE::ComputeReplanningDataforAction(EnvNAVXYTHETACARTLATAction_t* action)
{
	int j;

	//iterate over all the cells involved in the action
	EnvNAVXYTHETACARTLAT3Dcell_t startcell3d, endcell3d;
	for(int i = 0; i < (int)action->intersectingcellsV.size(); i++)
	{

		//compute the translated affected search Pose - what state has an outgoing action whose intersecting cell is at 0,0
		startcell3d.theta = action->starttheta;
		startcell3d.x = - action->intersectingcellsV.at(i).x;
		startcell3d.y = - action->intersectingcellsV.at(i).y;
    startcell3d.cartangle = action->startcartangle;


		//compute the translated affected search Pose - what state has an incoming action whose intersecting cell is at 0,0
		endcell3d.theta = NORMALIZEDISCTHETA(action->endtheta, NAVXYTHETACARTLAT_THETADIRS); 
		endcell3d.x = startcell3d.x + action->dX; 
		endcell3d.y = startcell3d.y + action->dY;
    endcell3d.cartangle = action->endcartangle;

		//store the cells if not already there
		for(j = 0; j < (int)affectedsuccstatesV.size(); j++)
		{
			if(affectedsuccstatesV.at(j) == endcell3d)
				break;
		}
		if (j == (int)affectedsuccstatesV.size())
			affectedsuccstatesV.push_back(endcell3d);

		for(j = 0; j < (int)affectedpredstatesV.size(); j++)
		{
			if(affectedpredstatesV.at(j) == startcell3d)
				break;
		}
		if (j == (int)affectedpredstatesV.size())
			affectedpredstatesV.push_back(startcell3d);

    }//over intersecting cells

	

	//add the centers since with h2d we are using these in cost computations
	//---intersecting cell = origin
	//compute the translated affected search Pose - what state has an outgoing action whose intersecting cell is at 0,0
	startcell3d.theta = action->starttheta;
	startcell3d.x = - 0;
	startcell3d.y = - 0;
  startcell3d.cartangle = CartContTheta2Disc(0.0, CART_THETADIRS);

	//compute the translated affected search Pose - what state has an incoming action whose intersecting cell is at 0,0
	endcell3d.theta = NORMALIZEDISCTHETA(action->endtheta, NAVXYTHETACARTLAT_THETADIRS); 
	endcell3d.x = startcell3d.x + action->dX; 
	endcell3d.y = startcell3d.y + action->dY;
  endcell3d.cartangle = CartContTheta2Disc(0.0, CART_THETADIRS);

	//store the cells if not already there
	for(j = 0; j < (int)affectedsuccstatesV.size(); j++)
	{
		if(affectedsuccstatesV.at(j) == endcell3d)
			break;
	}
	if (j == (int)affectedsuccstatesV.size())
		affectedsuccstatesV.push_back(endcell3d);

	for(j = 0; j < (int)affectedpredstatesV.size(); j++)
	{
		if(affectedpredstatesV.at(j) == startcell3d)
			break;
	}
	if (j == (int)affectedpredstatesV.size())
		affectedpredstatesV.push_back(startcell3d);


	//---intersecting cell = outcome state
	//compute the translated affected search Pose - what state has an outgoing action whose intersecting cell is at 0,0
	startcell3d.theta = action->starttheta;
	startcell3d.x = - action->dX;
	startcell3d.y = - action->dY;

	//compute the translated affected search Pose - what state has an incoming action whose intersecting cell is at 0,0
	endcell3d.theta = NORMALIZEDISCTHETA(action->endtheta, NAVXYTHETACARTLAT_THETADIRS); 
	endcell3d.x = startcell3d.x + action->dX; 
	endcell3d.y = startcell3d.y + action->dY;

	for(j = 0; j < (int)affectedsuccstatesV.size(); j++)
	{
		if(affectedsuccstatesV.at(j) == endcell3d)
			break;
	}
	if (j == (int)affectedsuccstatesV.size())
		affectedsuccstatesV.push_back(endcell3d);

	for(j = 0; j < (int)affectedpredstatesV.size(); j++)
	{
		if(affectedpredstatesV.at(j) == startcell3d)
			break;
	}
	if (j == (int)affectedpredstatesV.size())
		affectedpredstatesV.push_back(startcell3d);


}

// TODO - does this need to be filled up?
//computes all the 3D states whose outgoing actions are potentially affected when cell (0,0) changes its status
//it also does the same for the 3D states whose incoming actions are potentially affected when cell (0,0) changes its status
void EnvironmentNAVXYTHETACARTLATTICE::ComputeReplanningData()
{

    //iterate over all actions
	//orientations
	for(int tind = 0; tind < NAVXYTHETACARTLAT_THETADIRS; tind++)
    {        
        //actions
		for(int aind = 0; aind < EnvNAVXYTHETACARTLATCfg.actionwidth; aind++)
		{
            //compute replanning data for this action 
			ComputeReplanningDataforAction(&EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind]);
		}
	}
}

// TODO - this does not seem to be used anymore
//here motionprimitivevector contains actions only for 0 angle
void EnvironmentNAVXYTHETACARTLATTICE::PrecomputeActionswithBaseMotionPrimitive(vector<SBPL_xythetacart_mprimitive>* motionprimitiveV)
{

	ROS_INFO("Pre-computing action data using base motion primitives... this may take some time.");
	EnvNAVXYTHETACARTLATCfg.ActionsV = new EnvNAVXYTHETACARTLATAction_t* [NAVXYTHETACARTLAT_THETADIRS];
	EnvNAVXYTHETACARTLATCfg.PredActionsV = new vector<EnvNAVXYTHETACARTLATAction_t*> [NAVXYTHETACARTLAT_THETADIRS];
	vector<sbpl_2Dcell_t> footprint;

	//iterate over source angles
	for(int tind = 0; tind < NAVXYTHETACARTLAT_THETADIRS; tind++)
	{
		ROS_DEBUG("pre-computing for angle %d out of %d angles", tind, NAVXYTHETACARTLAT_THETADIRS);
		EnvNAVXYTHETACARTLATCfg.ActionsV[tind] = new EnvNAVXYTHETACARTLATAction_t[motionprimitiveV->size()];

		//compute sourcepose
		EnvNAVXYTHETACARTLAT3Dpt_t sourcepose;
		sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETACARTLATCfg.cellsize_m);
		sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETACARTLATCfg.cellsize_m);
		sourcepose.theta = DiscTheta2Cont(tind, NAVXYTHETACARTLAT_THETADIRS);
    sourcepose.cartangle = 0.0;

		//iterate over motion primitives
		for(size_t aind = 0; aind < motionprimitiveV->size(); aind++)
		{
			EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].starttheta = tind;
			EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].startcartangle = CartContTheta2Disc(0, CART_THETADIRS);
			double mp_endx_m = motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size()-1].x;
			double mp_endy_m = motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size()-1].y;
			double mp_endtheta_rad = motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size()-1].theta;
      //			double mp_endcartangle_rad = motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size()-1].cartangle;
			
			double endx = sourcepose.x + (mp_endx_m*cos(sourcepose.theta) - mp_endy_m*sin(sourcepose.theta));
			double endy = sourcepose.y + (mp_endx_m*sin(sourcepose.theta) + mp_endy_m*cos(sourcepose.theta));
			
			int endx_c = CONTXY2DISC(endx, EnvNAVXYTHETACARTLATCfg.cellsize_m);
			int endy_c = CONTXY2DISC(endy, EnvNAVXYTHETACARTLATCfg.cellsize_m);

			
			EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endtheta = ContTheta2Disc(mp_endtheta_rad+sourcepose.theta, NAVXYTHETACARTLAT_THETADIRS);
      EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endcartangle = CartContTheta2Disc(0, CART_THETADIRS);
			EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dX = endx_c;
			EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dY = endy_c;
			if(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dY != 0 || EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dX != 0)
				EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].cost = (int)(ceil(NAVXYTHETACARTLAT_COSTMULT_MTOMM*EnvNAVXYTHETACARTLATCfg.cellsize_m/EnvNAVXYTHETACARTLATCfg.nominalvel_mpersecs*
								sqrt((double)(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dX*EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dX + 
								EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dY*EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dY))));
			else //cost of turn in place
				EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].cost = (int)(NAVXYTHETACARTLAT_COSTMULT_MTOMM*
						EnvNAVXYTHETACARTLATCfg.timetoturn45degsinplace_secs*fabs(computeMinUnsignedAngleDiff(mp_endtheta_rad,0))/(PI_CONST/4.0));

			//compute and store interm points as well as intersecting cells
			EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
			EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intermptV.clear();
			EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].interm3DcellsV.clear();
			EnvNAVXYTHETACARTLAT3Dcell_t previnterm3Dcell;
			previnterm3Dcell.theta = previnterm3Dcell.x = previnterm3Dcell.y = 0;
			for (int pind = 0; pind < (int)motionprimitiveV->at(aind).intermptV.size(); pind++)
			{
				EnvNAVXYTHETACARTLAT3Dpt_t intermpt = motionprimitiveV->at(aind).intermptV[pind];
		
				//rotate it appropriately
				double rotx = intermpt.x*cos(sourcepose.theta) - intermpt.y*sin(sourcepose.theta);
				double roty = intermpt.x*sin(sourcepose.theta) + intermpt.y*cos(sourcepose.theta);
				intermpt.x = rotx;
				intermpt.y = roty;
				intermpt.theta = normalizeAngle(sourcepose.theta + intermpt.theta);

				//store it (they are with reference to 0,0,stattheta (not sourcepose.x,sourcepose.y,starttheta (that is, half-bin))
				EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intermptV.push_back(intermpt);

				//now compute the intersecting cells (for this pose has to be translated by sourcepose.x,sourcepose.y
				EnvNAVXYTHETACARTLAT3Dpt_t pose;
				pose = intermpt;
				pose.x += sourcepose.x;
				pose.y += sourcepose.y;
				CalculateFootprintForPose(pose, &EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV);

				//now also store the intermediate discretized cell if not there already
				EnvNAVXYTHETACARTLAT3Dcell_t interm3Dcell;
				interm3Dcell.x = CONTXY2DISC(pose.x, EnvNAVXYTHETACARTLATCfg.cellsize_m);
				interm3Dcell.y = CONTXY2DISC(pose.y, EnvNAVXYTHETACARTLATCfg.cellsize_m);
				interm3Dcell.theta = ContTheta2Disc(pose.theta, NAVXYTHETACARTLAT_THETADIRS); 
        interm3Dcell.cartangle = CartContTheta2Disc(pose.cartangle, CART_THETADIRS);
				if(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].interm3DcellsV.size() == 0 || 
					previnterm3Dcell.theta != interm3Dcell.theta || previnterm3Dcell.x != interm3Dcell.x || previnterm3Dcell.y != interm3Dcell.y)
				{
					EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].interm3DcellsV.push_back(interm3Dcell);
				}
				previnterm3Dcell = interm3Dcell;

			}

			//now remove the source footprint
			RemoveSourceFootprint(sourcepose, &EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
			SBPL_FPRINTF(fDeb, "action tind=%d aind=%d: dX=%d dY=%d endtheta=%d (%.2f degs -> %.2f degs) cost=%d (mprim: %.2f %.2f %.2f)",
              (int) tind, (int) aind, 			
				EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dY,
				EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endtheta, sourcepose.theta*180/PI_CONST, 
				EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intermptV[EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intermptV.size()-1].theta*180/PI_CONST,	
				EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].cost,
				mp_endx_m, mp_endy_m, mp_endtheta_rad);
#endif

			//add to the list of backward actions
			int targettheta = EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endtheta;
			if (targettheta < 0)
				targettheta = targettheta + NAVXYTHETACARTLAT_THETADIRS;
			 EnvNAVXYTHETACARTLATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind]));

		}
	}

	//set number of actions
	EnvNAVXYTHETACARTLATCfg.actionwidth = motionprimitiveV->size();


	//now compute replanning data
	ComputeReplanningData();

	ROS_DEBUG("done pre-computing action data based on motion primitives");


}


//here motionprimitivevector contains actions for all angles
void EnvironmentNAVXYTHETACARTLATTICE::PrecomputeActionswithCompleteMotionPrimitive(vector<SBPL_xythetacart_mprimitive>* motionprimitiveV)
{

	ROS_DEBUG("Pre-computing action data using motion primitives for every angle...");
	EnvNAVXYTHETACARTLATCfg.ActionsV = new EnvNAVXYTHETACARTLATAction_t* [NAVXYTHETACARTLAT_THETADIRS];
	EnvNAVXYTHETACARTLATCfg.PredActionsV = new vector<EnvNAVXYTHETACARTLATAction_t*> [NAVXYTHETACARTLAT_THETADIRS];
	vector<sbpl_2Dcell_t> footprint;

	if(motionprimitiveV->size()%NAVXYTHETACARTLAT_THETADIRS != 0)
	{
		ROS_ERROR("ERROR: motionprimitives should be uniform across actions");
    initialized_ = false;
    return;
	}

	EnvNAVXYTHETACARTLATCfg.actionwidth = ((int)motionprimitiveV->size())/NAVXYTHETACARTLAT_THETADIRS;

	//iterate over source angles
	int maxnumofactions = 0;
	for(int tind = 0; tind < NAVXYTHETACARTLAT_THETADIRS; tind++)
	{
		ROS_DEBUG("Pre-computing for angle %d out of %d angles", tind, NAVXYTHETACARTLAT_THETADIRS);

		EnvNAVXYTHETACARTLATCfg.ActionsV[tind] = new EnvNAVXYTHETACARTLATAction_t[EnvNAVXYTHETACARTLATCfg.actionwidth];  

		//compute sourcepose
		EnvNAVXYTHETACARTLAT3Dpt_t sourcepose;
		sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETACARTLATCfg.cellsize_m);
		sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETACARTLATCfg.cellsize_m);
		sourcepose.theta = DiscTheta2Cont(tind, NAVXYTHETACARTLAT_THETADIRS);
    sourcepose.cartangle = CartDiscTheta2Cont(2, CART_THETADIRS);

		//iterate over motion primitives
		int numofactions = 0;
		int aind = -1;
		for(int mind = 0; mind < (int)motionprimitiveV->size(); mind++)
		{
			//find a motion primitive for this angle
			if(motionprimitiveV->at(mind).starttheta_c != tind)
				continue;
			
			aind++;
			numofactions++;

			//start angle
			EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].starttheta = tind;
			EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].startcartangle = CartContTheta2Disc(0, CART_THETADIRS);

			//compute dislocation
			EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endtheta = motionprimitiveV->at(mind).endcell.theta;
			EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endcartangle = CartContTheta2Disc(0, CART_THETADIRS);
			EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dX = motionprimitiveV->at(mind).endcell.x;
			EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dY = motionprimitiveV->at(mind).endcell.y;

			//compute cost
			if(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dY != 0 || EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dX != 0)
				EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].cost = (int)(ceil(NAVXYTHETACARTLAT_COSTMULT_MTOMM*EnvNAVXYTHETACARTLATCfg.cellsize_m/EnvNAVXYTHETACARTLATCfg.nominalvel_mpersecs*
								sqrt((double)(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dX*EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dX + 
								EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dY*EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dY))));
			else //cost of turn in place
				EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].cost = (int)(NAVXYTHETACARTLAT_COSTMULT_MTOMM*
						EnvNAVXYTHETACARTLATCfg.timetoturn45degsinplace_secs*
						fabs(computeMinUnsignedAngleDiff(DiscTheta2Cont(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endtheta, NAVXYTHETACARTLAT_THETADIRS),
														DiscTheta2Cont(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].starttheta, NAVXYTHETACARTLAT_THETADIRS)))/(PI_CONST/4.0));
			//use any additional cost multiplier
			EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].cost *= motionprimitiveV->at(mind).additionalactioncostmult;

			//compute and store interm points as well as intersecting cells
			EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
			EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intermptV.clear();
			EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].interm3DcellsV.clear();
			EnvNAVXYTHETACARTLAT3Dcell_t previnterm3Dcell;
			previnterm3Dcell.theta = 0; 
      previnterm3Dcell.x = 0; 
      previnterm3Dcell.y = 0;			
      previnterm3Dcell.cartangle = CartContTheta2Disc(0, CART_THETADIRS);
      ROS_DEBUG("Motion primitive has %d intermediate points for (%d,%d)",(int)motionprimitiveV->at(mind).intermptV.size(),tind,aind);
			for (int pind = 0; pind < (int)motionprimitiveV->at(mind).intermptV.size(); pind++)
			{
				EnvNAVXYTHETACARTLAT3Dpt_t intermpt = motionprimitiveV->at(mind).intermptV[pind];
		
				//store it (they are with reference to 0,0,stattheta (not sourcepose.x,sourcepose.y,starttheta (that is, half-bin))
				EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intermptV.push_back(intermpt);

				//now compute the intersecting cells (for this pose has to be translated by sourcepose.x,sourcepose.y
				EnvNAVXYTHETACARTLAT3Dpt_t pose;
				pose = intermpt;
				pose.x += sourcepose.x;
				pose.y += sourcepose.y;
        ROS_DEBUG("Pose for intermediate point %d is: %f %f %f %f",pind,pose.x,pose.y,pose.theta,pose.cartangle);
				CalculateFootprintForPose(pose, &EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV);

				EnvNAVXYTHETACARTLAT3Dpt_t cart_center_pose = getCartCenter(pose,  EnvNAVXYTHETACARTLATCfg.CartCenterOffset);
			
				//now also store the intermediate discretized cell if not there already
				EnvNAVXYTHETACARTLAT3Dcell_t interm3Dcell;
				interm3Dcell.x = CONTXY2DISC(pose.x, EnvNAVXYTHETACARTLATCfg.cellsize_m);
				interm3Dcell.y = CONTXY2DISC(pose.y, EnvNAVXYTHETACARTLATCfg.cellsize_m);
				interm3Dcell.theta = ContTheta2Disc(pose.theta, NAVXYTHETACARTLAT_THETADIRS); 
				interm3Dcell.cartangle = CartContTheta2Disc(pose.cartangle, CART_THETADIRS); 

        EnvNAVXYTHETACARTLAT3Dcell_t interm3Dcellcart;
				interm3Dcellcart.x = CONTXY2DISC(cart_center_pose.x, EnvNAVXYTHETACARTLATCfg.cellsize_m);
				interm3Dcellcart.y = CONTXY2DISC(cart_center_pose.y, EnvNAVXYTHETACARTLATCfg.cellsize_m);
				interm3Dcellcart.theta = ContTheta2Disc(cart_center_pose.theta, NAVXYTHETACARTLAT_THETADIRS); 
				interm3Dcellcart.cartangle = CartContTheta2Disc(cart_center_pose.cartangle, CART_THETADIRS); 
  

        if(tind == 0 && aind == 0)
          ROS_DEBUG("Intermediate point: %d %d %d %d",interm3Dcell.x, interm3Dcell.y, interm3Dcell.theta, interm3Dcell.cartangle);
        //        if(interm3Dcell.cartangle != 2)
        //          ROS_ERROR("Intermediate point: %d %d %d %d",interm3Dcell.x, interm3Dcell.y, interm3Dcell.theta, interm3Dcell.cartangle);

				if(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].interm3DcellsV.size() == 0 || 
					previnterm3Dcell.theta != interm3Dcell.theta || previnterm3Dcell.x != interm3Dcell.x || previnterm3Dcell.y != interm3Dcell.y)
				{
					EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].interm3DcellsV.push_back(interm3Dcell);
					EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].interm3DcellsV.push_back(interm3Dcellcart);
				}
				previnterm3Dcell = interm3Dcell;
        ROS_DEBUG("Intersecting cells have size: %d",(int)EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV.size());
			}
                        /*
      std::stringstream ss;
      ss << "/tmp/action_" << tind << "_" << aind;
      FILE *fid = fopen(ss.str().c_str(),"w");
      for(unsigned int i=0; i < EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV.size(); i++)
      {
        fprintf(fid, "%d %d %.3f %.3f",
                EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV.at(i).x, 
                EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV.at(i).y, 
                DISCXY2CONT(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV.at(i).x, EnvNAVXYTHETACARTLATCfg.cellsize_m), 
                DISCXY2CONT(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV.at(i).y, EnvNAVXYTHETACARTLATCfg.cellsize_m));
      }
      fclose(fid);*/
			//now remove the source footprint
			RemoveSourceFootprint(sourcepose, &EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV);
                        /*
      std::stringstream ss1;
      ss1 << "/tmp/actions/action_rf_" << tind << "_" << aind;
      FILE *fid2 = fopen(ss1.str().c_str(),"w");
      for(unsigned int i=0; i < EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV.size(); i++)
      {
        fprintf(fid2, "%d %d %.3f %.3f",
                EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV.at(i).x, 
                EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV.at(i).y, 
                DISCXY2CONT(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV.at(i).x, EnvNAVXYTHETACARTLATCfg.cellsize_m), 
                DISCXY2CONT(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV.at(i).y, EnvNAVXYTHETACARTLATCfg.cellsize_m));
      }
      fclose(fid2);*/
                        /*
      std::stringstream ss2;
      ss2 << "/tmp/actions/action_sf_" << tind << "_" << aind;
      FILE *fid3 = fopen(ss2.str().c_str(),"w");
      std::vector<sbpl_2Dcell_t> footprint_source;
      if(aind == 0 && tind == 0)
        ROS_INFO("Source pose: %f %f %f %f",sourcepose.x,sourcepose.y,sourcepose.theta,sourcepose.cartangle);
      CalculateFootprintForPose(sourcepose, &footprint_source);
      for(unsigned int i=0; i < footprint_source.size(); i++)
      {
        fprintf(fid3, "%d %d %.3f %.3f",
                footprint_source.at(i).x, 
                footprint_source.at(i).y, 
                DISCXY2CONT(footprint_source.at(i).x, EnvNAVXYTHETACARTLATCfg.cellsize_m), 
                DISCXY2CONT(footprint_source.at(i).y, EnvNAVXYTHETACARTLATCfg.cellsize_m));
      }
      fclose(fid3);*/


#if DEBUG
			SBPL_FPRINTF(fDeb, "action tind=%d aind=%d: dX=%d dY=%d endtheta=%d (%.2f degs -> %.2f degs) cost=%d (mprimID %d: %d %d %d) numofintermcells = %d numofintercells=%d",
              (int) tind, (int) aind, 			
				EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dY,
				EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endtheta, 
				EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intermptV[0].theta*180/PI_CONST, 
				EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intermptV[EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intermptV.size()-1].theta*180/PI_CONST,	
				EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].cost,
				motionprimitiveV->at(mind).motprimID, 
				motionprimitiveV->at(mind).endcell.x, motionprimitiveV->at(mind).endcell.y, motionprimitiveV->at(mind).endcell.theta,
              (int)EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].interm3DcellsV.size(),
              (int)EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV.size()); 
#endif

			//add to the list of backward actions
			int targettheta = EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endtheta;
			if (targettheta < 0)
				targettheta = targettheta + NAVXYTHETACARTLAT_THETADIRS;
			 EnvNAVXYTHETACARTLATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind]));

		}

		if(maxnumofactions < numofactions)
			maxnumofactions = numofactions;
	}



	//at this point we don't allow nonuniform number of actions
	if(motionprimitiveV->size() != (size_t)(NAVXYTHETACARTLAT_THETADIRS*maxnumofactions))
	{
		ROS_ERROR("nonuniform number of actions is not supported (maxnumofactions=%d while motprims=%d thetas=%d",
           maxnumofactions, (int)motionprimitiveV->size(), NAVXYTHETACARTLAT_THETADIRS);
		initialized_ = false;
    return;
	}

	//now compute replanning data
	ComputeReplanningData();
	ROS_INFO("done pre-computing action data based on motion primitives");
}

void EnvironmentNAVXYTHETACARTLATTICE::PrecomputeActions()
{

	//construct list of actions
	ROS_DEBUG("Pre-computing action data internally using the motion primitives for a 3D kinematic planning...");
	EnvNAVXYTHETACARTLATCfg.ActionsV = new EnvNAVXYTHETACARTLATAction_t* [NAVXYTHETACARTLAT_THETADIRS];
	EnvNAVXYTHETACARTLATCfg.PredActionsV = new vector<EnvNAVXYTHETACARTLATAction_t*> [NAVXYTHETACARTLAT_THETADIRS];
	vector<sbpl_2Dcell_t> footprint;
	//iterate over source angles
	for(int tind = 0; tind < NAVXYTHETACARTLAT_THETADIRS; tind++)
	{
		ROS_DEBUG("processing angle %d", tind);
		EnvNAVXYTHETACARTLATCfg.ActionsV[tind] = new EnvNAVXYTHETACARTLATAction_t[EnvNAVXYTHETACARTLATCfg.actionwidth];

		//compute sourcepose
		EnvNAVXYTHETACARTLAT3Dpt_t sourcepose;
		sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETACARTLATCfg.cellsize_m);
		sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETACARTLATCfg.cellsize_m);
		sourcepose.theta = DiscTheta2Cont(tind, NAVXYTHETACARTLAT_THETADIRS);

		//the construction assumes that the robot first turns and then goes along this new theta
		int aind = 0;
		for(; aind < 3; aind++)
		{
			EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].starttheta = tind;
			EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endtheta = (tind + aind - 1)%NAVXYTHETACARTLAT_THETADIRS; //-1,0,1
			double angle = DiscTheta2Cont(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endtheta, NAVXYTHETACARTLAT_THETADIRS);
			EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dX = (int)(cos(angle) + 0.5*(cos(angle)>0?1:-1));
			EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dY = (int)(sin(angle) + 0.5*(sin(angle)>0?1:-1));
			EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].cost = (int)(ceil(NAVXYTHETACARTLAT_COSTMULT_MTOMM*EnvNAVXYTHETACARTLATCfg.cellsize_m/EnvNAVXYTHETACARTLATCfg.nominalvel_mpersecs*sqrt((double)(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dX*EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dX + 
					EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dY*EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dY))));

			//compute intersecting cells
			EnvNAVXYTHETACARTLAT3Dpt_t pose;
			pose.x = DISCXY2CONT(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETACARTLATCfg.cellsize_m);
			pose.y = DISCXY2CONT(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dY, EnvNAVXYTHETACARTLATCfg.cellsize_m);
			pose.theta = angle;
			EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intermptV.clear();
			EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
			CalculateFootprintForPose(pose, &EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV);
			RemoveSourceFootprint(sourcepose, &EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
			ROS_DEBUG("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d cost=%d",
				tind, aind, EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endtheta, angle, 
				EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dY,
				EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].cost);
#endif

			//add to the list of backward actions
			int targettheta = EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endtheta;
			if (targettheta < 0)
				targettheta = targettheta + NAVXYTHETACARTLAT_THETADIRS;
			 EnvNAVXYTHETACARTLATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind]));

		}

		//decrease and increase angle without movement
		aind = 3;
		EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].starttheta = tind;
		EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endtheta = tind-1;
		if(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endtheta < 0) EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endtheta += NAVXYTHETACARTLAT_THETADIRS;
		EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dX = 0;
		EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dY = 0;
		EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].cost = (int)(NAVXYTHETACARTLAT_COSTMULT_MTOMM*EnvNAVXYTHETACARTLATCfg.timetoturn45degsinplace_secs);

		//compute intersecting cells
		EnvNAVXYTHETACARTLAT3Dpt_t pose;
		pose.x = DISCXY2CONT(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETACARTLATCfg.cellsize_m);
		pose.y = DISCXY2CONT(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dY, EnvNAVXYTHETACARTLATCfg.cellsize_m);
		pose.theta = DiscTheta2Cont(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endtheta, NAVXYTHETACARTLAT_THETADIRS);
		pose.cartangle = CartDiscTheta2Cont(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endcartangle, CART_THETADIRS);

		EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intermptV.clear();
		EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
		CalculateFootprintForPose(pose, &EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV);
		RemoveSourceFootprint(sourcepose, &EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
		ROS_DEBUG("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d cost=%d",
              tind, aind, EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endtheta, DiscTheta2Cont(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endtheta, NAVXYTHETACARTLAT_THETADIRS),
              EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dY,
              EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].cost);
#endif

		//add to the list of backward actions
		int targettheta = EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endtheta;
		if (targettheta < 0)
			targettheta = targettheta + NAVXYTHETACARTLAT_THETADIRS;
		 EnvNAVXYTHETACARTLATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind]));


		aind = 4;
		EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].starttheta = tind;
		EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endtheta = (tind + 1)%NAVXYTHETACARTLAT_THETADIRS; 
		EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dX = 0;
		EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dY = 0;
		EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].cost = (int)(NAVXYTHETACARTLAT_COSTMULT_MTOMM*EnvNAVXYTHETACARTLATCfg.timetoturn45degsinplace_secs);

		//compute intersecting cells
		pose.x = DISCXY2CONT(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETACARTLATCfg.cellsize_m);
		pose.y = DISCXY2CONT(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dY, EnvNAVXYTHETACARTLATCfg.cellsize_m);
		pose.theta = DiscTheta2Cont(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endtheta, NAVXYTHETACARTLAT_THETADIRS);
    pose.cartangle = CartDiscTheta2Cont(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endcartangle, CART_THETADIRS);
		EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intermptV.clear();
		EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
		CalculateFootprintForPose(pose, &EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV);
		RemoveSourceFootprint(sourcepose, &EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].intersectingcellsV);


#if DEBUG
		ROS_DEBUG("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d cost=%d",
              tind, aind, EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endtheta, DiscTheta2Cont(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endtheta, NAVXYTHETACARTLAT_THETADIRS),
              EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].dY,
              EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].cost);
#endif

		//add to the list of backward actions
		targettheta = EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind].endtheta;
		if (targettheta < 0)
			targettheta = targettheta + NAVXYTHETACARTLAT_THETADIRS;
		 EnvNAVXYTHETACARTLATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETACARTLATCfg.ActionsV[tind][aind]));

	}

	//now compute replanning data
	ComputeReplanningData();
	ROS_INFO("Done pre-computing action data");
}

void EnvironmentNAVXYTHETACARTLATTICE::InitializeEnvConfig(vector<SBPL_xythetacart_mprimitive>* motionprimitiveV)
{
	//aditional to configuration file initialization of EnvNAVXYTHETACARTLATCfg if necessary

	//dXY dirs
	EnvNAVXYTHETACARTLATCfg.dXY[0][0] = -1;
	EnvNAVXYTHETACARTLATCfg.dXY[0][1] = -1;
	EnvNAVXYTHETACARTLATCfg.dXY[1][0] = -1;
	EnvNAVXYTHETACARTLATCfg.dXY[1][1] = 0;
	EnvNAVXYTHETACARTLATCfg.dXY[2][0] = -1;
	EnvNAVXYTHETACARTLATCfg.dXY[2][1] = 1;
	EnvNAVXYTHETACARTLATCfg.dXY[3][0] = 0;
	EnvNAVXYTHETACARTLATCfg.dXY[3][1] = -1;
	EnvNAVXYTHETACARTLATCfg.dXY[4][0] = 0;
	EnvNAVXYTHETACARTLATCfg.dXY[4][1] = 1;
	EnvNAVXYTHETACARTLATCfg.dXY[5][0] = 1;
	EnvNAVXYTHETACARTLATCfg.dXY[5][1] = -1;
	EnvNAVXYTHETACARTLATCfg.dXY[6][0] = 1;
	EnvNAVXYTHETACARTLATCfg.dXY[6][1] = 0;
	EnvNAVXYTHETACARTLATCfg.dXY[7][0] = 1;
	EnvNAVXYTHETACARTLATCfg.dXY[7][1] = 1;


	EnvNAVXYTHETACARTLAT3Dpt_t temppose;
	temppose.x = 0.0;
	temppose.y = 0.0;
	temppose.theta = 0.0;
	vector<sbpl_2Dcell_t> footprint;
	CalculateFootprintForPose(temppose, &footprint);
	ROS_DEBUG("number of cells in footprint of the robot = %zu", footprint.size());

#if DEBUG
	SBPL_FPRINTF(fDeb, "footprint cells (size=%zu):", footprint.size());
	for(int i = 0; i < (int) footprint.size(); i++)
	{
		SBPL_FPRINTF(fDeb, "%d %d (cont: %.3f %.3f)", footprint.at(i).x, footprint.at(i).y, 
			DISCXY2CONT(footprint.at(i).x, EnvNAVXYTHETACARTLATCfg.cellsize_m), 
			DISCXY2CONT(footprint.at(i).y, EnvNAVXYTHETACARTLATCfg.cellsize_m));
	}
#endif


	if(motionprimitiveV == NULL)
		PrecomputeActions();
	else
		PrecomputeActionswithCompleteMotionPrimitive(motionprimitiveV);


}



bool EnvironmentNAVXYTHETACARTLATTICE::IsValidCell(int X, int Y)
{
	return (X >= 0 && X < EnvNAVXYTHETACARTLATCfg.EnvWidth_c && 
		Y >= 0 && Y < EnvNAVXYTHETACARTLATCfg.EnvHeight_c && 
		EnvNAVXYTHETACARTLATCfg.Grid2D[X][Y] < EnvNAVXYTHETACARTLATCfg.obsthresh);
}

bool EnvironmentNAVXYTHETACARTLATTICE::IsWithinMapCell(int X, int Y)
{
	return (X >= 0 && X < EnvNAVXYTHETACARTLATCfg.EnvWidth_c && 
		Y >= 0 && Y < EnvNAVXYTHETACARTLATCfg.EnvHeight_c);
}

bool EnvironmentNAVXYTHETACARTLATTICE::IsValidConfiguration(int X, int Y, int Theta, int CartAngle)
{
	vector<sbpl_2Dcell_t> footprint;
	EnvNAVXYTHETACARTLAT3Dpt_t pose;

	//compute continuous pose
	pose.x = DISCXY2CONT(X, EnvNAVXYTHETACARTLATCfg.cellsize_m);
	pose.y = DISCXY2CONT(Y, EnvNAVXYTHETACARTLATCfg.cellsize_m);
	pose.theta = DiscTheta2Cont(Theta, NAVXYTHETACARTLAT_THETADIRS);
  pose.cartangle = CartDiscTheta2Cont(CartAngle, CART_THETADIRS);

	//compute footprint cells
	CalculateFootprintForPose(pose, &footprint);

	//iterate over all footprint cells
	for(int find = 0; find < (int)footprint.size(); find++)
	{
		int x = footprint.at(find).x;
		int y = footprint.at(find).y;

		if (x < 0 || x >= EnvNAVXYTHETACARTLATCfg.EnvWidth_c ||
			y < 0 || y >= EnvNAVXYTHETACARTLATCfg.EnvHeight_c ||		
			EnvNAVXYTHETACARTLATCfg.Grid2D[x][y] >= EnvNAVXYTHETACARTLATCfg.obsthresh)
		{
			return false;
		}
	}

	return true;
}


int EnvironmentNAVXYTHETACARTLATTICE::GetActionCost(int SourceX, int SourceY, int SourceTheta, int SourceCartAngle, EnvNAVXYTHETACARTLATAction_t* action)
{
	sbpl_2Dcell_t cell;
	EnvNAVXYTHETACARTLAT3Dcell_t interm3Dcell;
	int i;

	//TODO - go over bounding box (minpt and maxpt) to test validity and skip testing boundaries below, also order intersect cells so that the four farthest pts go first
	if(!IsValidCell(SourceX, SourceY))
  {
		return INFINITECOST;
  }
	if(!IsValidCell(SourceX + action->dX, SourceY + action->dY))
  {
		return INFINITECOST;
  }

	if(EnvNAVXYTHETACARTLATCfg.Grid2D[SourceX + action->dX][SourceY + action->dY] >= EnvNAVXYTHETACARTLATCfg.cost_inscribed_thresh)
  {
		return INFINITECOST;
  }

	//need to iterate over discretized center cells and compute cost based on them
	unsigned char maxcellcost = 0;
	for(i = 0; i < (int)action->interm3DcellsV.size(); i++)
	{
		interm3Dcell = action->interm3DcellsV.at(i);
		interm3Dcell.x = interm3Dcell.x + SourceX;
		interm3Dcell.y = interm3Dcell.y + SourceY;
		
		if(interm3Dcell.x < 0 || interm3Dcell.x >= EnvNAVXYTHETACARTLATCfg.EnvWidth_c ||
			interm3Dcell.y < 0 || interm3Dcell.y >= EnvNAVXYTHETACARTLATCfg.EnvHeight_c)
    {
			return INFINITECOST;
    }

		maxcellcost = __max(maxcellcost, EnvNAVXYTHETACARTLATCfg.Grid2D[interm3Dcell.x][interm3Dcell.y]);

		//check that the robot is NOT in the cell at which there is no valid orientation
		if(maxcellcost >= EnvNAVXYTHETACARTLATCfg.cost_inscribed_thresh)
    {
			return INFINITECOST;
    }
	}


	//check collisions that for the particular footprint orientation along the action
	if(EnvNAVXYTHETACARTLATCfg.FootprintPolygon.size() > 1 && (int)maxcellcost >= EnvNAVXYTHETACARTLATCfg.cost_possibly_circumscribed_thresh)
	{
		checks_cart++;

		for(i = 0; i < (int)action->intersectingcellsV.size(); i++) 
		{
			//get the cell in the map
			cell = action->intersectingcellsV.at(i);
			cell.x = cell.x + SourceX;
			cell.y = cell.y + SourceY;
			
			//check validity
			if(!IsValidCell(cell.x, cell.y))
      {
        return INFINITECOST;
      }

			//if(EnvNAVXYTHETACARTLATCfg.Grid2D[cell.x][cell.y] > currentmaxcost) //cost computation changed: cost = max(cost of centers of the robot along action)
			//	currentmaxcost = EnvNAVXYTHETACARTLATCfg.Grid2D[cell.x][cell.y];	//intersecting cells are only used for collision checking
		}
	}

	//to ensure consistency of h2D:
	maxcellcost = __max(maxcellcost, EnvNAVXYTHETACARTLATCfg.Grid2D[SourceX][SourceY]);
	int currentmaxcost = (int)__max(maxcellcost, EnvNAVXYTHETACARTLATCfg.Grid2D[SourceX + action->dX][SourceY + action->dY]);


	return action->cost*(currentmaxcost+1); //use cell cost as multiplicative factor
 
}



double EnvironmentNAVXYTHETACARTLATTICE::EuclideanDistance_m(int X1, int Y1, int X2, int Y2)
{
    int sqdist = ((X1-X2)*(X1-X2)+(Y1-Y2)*(Y1-Y2));
    return EnvNAVXYTHETACARTLATCfg.cellsize_m*sqrt((double)sqdist);

}


//adds points to it (does not clear it beforehand)
void EnvironmentNAVXYTHETACARTLATTICE::CalculateFootprintForPose(EnvNAVXYTHETACARTLAT3Dpt_t pose, vector<sbpl_2Dcell_t>* footprint)
{  
	int pind;
  
#if DEBUG
//  printf("---Calculating Footprint for Pose: %f %f %f---",
//	pose.x, pose.y, pose.theta);
#endif

  //handle special case where footprint is just a point
  if(EnvNAVXYTHETACARTLATCfg.FootprintPolygon.size() <= 1)
  {
    sbpl_2Dcell_t cell;
    cell.x = CONTXY2DISC(pose.x, EnvNAVXYTHETACARTLATCfg.cellsize_m);
    cell.y = CONTXY2DISC(pose.y, EnvNAVXYTHETACARTLATCfg.cellsize_m);
    
    for(pind = 0; pind < (int)footprint->size(); pind++)
    {
      if(cell.x == footprint->at(pind).x && cell.y == footprint->at(pind).y)
        break;
    }
    if(pind == (int)footprint->size()) footprint->push_back(cell);
    return;
  }
  
  vector<sbpl_2Dpt_t> bounding_polygon;
  unsigned int find;
  double max_x = -INFINITECOST, min_x = INFINITECOST, max_y = -INFINITECOST, min_y = INFINITECOST;
  sbpl_2Dpt_t pt;// = {0,0};
  pt.x = 0;
  pt.y = 0;
  for(find = 0; find < EnvNAVXYTHETACARTLATCfg.FootprintPolygon.size(); find++){
    
    //rotate and translate the corner of the robot
    pt = EnvNAVXYTHETACARTLATCfg.FootprintPolygon[find];
    
    //rotate and translate the point
    sbpl_2Dpt_t corner;
    corner.x = cos(pose.theta)*pt.x - sin(pose.theta)*pt.y + pose.x;
    corner.y = sin(pose.theta)*pt.x + cos(pose.theta)*pt.y + pose.y;
    bounding_polygon.push_back(corner);
#if DEBUG
//    printf("Pt: %f %f, Corner: %f %f", pt.x, pt.y, corner.x, corner.y);
#endif
    if(corner.x < min_x || find==0){
      min_x = corner.x;
    }
    if(corner.x > max_x || find==0){
      max_x = corner.x;
    }
    if(corner.y < min_y || find==0){
      min_y = corner.y;
    }
    if(corner.y > max_y || find==0){
      max_y = corner.y;
    }
    
  }

#if DEBUG
//  printf("Footprint bounding box: %f %f %f %f", min_x, max_x, min_y, max_y);
#endif
  //initialize previous values to something that will fail the if condition during the first iteration in the for loop
  int prev_discrete_x = CONTXY2DISC(pt.x, EnvNAVXYTHETACARTLATCfg.cellsize_m) + 1; 
  int prev_discrete_y = CONTXY2DISC(pt.y, EnvNAVXYTHETACARTLATCfg.cellsize_m) + 1;
  int prev_inside = 0;
  int discrete_x;
  int discrete_y;

  for(double x=min_x; x<=max_x; x+=EnvNAVXYTHETACARTLATCfg.cellsize_m/3){
    for(double y=min_y; y<=max_y; y+=EnvNAVXYTHETACARTLATCfg.cellsize_m/3){
      pt.x = x;
      pt.y = y;
      discrete_x = CONTXY2DISC(pt.x, EnvNAVXYTHETACARTLATCfg.cellsize_m);
      discrete_y = CONTXY2DISC(pt.y, EnvNAVXYTHETACARTLATCfg.cellsize_m);
      
      //see if we just tested this point
      if(discrete_x != prev_discrete_x || discrete_y != prev_discrete_y || prev_inside==0){

#if DEBUG
//		printf("Testing point: %f %f Discrete: %d %d", pt.x, pt.y, discrete_x, discrete_y);
#endif
	
		if(IsInsideFootprint(pt, &bounding_polygon)){
		//convert to a grid point

#if DEBUG
//			printf("Pt Inside %f %f", pt.x, pt.y);
#endif

			sbpl_2Dcell_t cell;
			cell.x = discrete_x;
			cell.y = discrete_y;

			//insert point if not there already
			int pind = 0;
			for(pind = 0; pind < (int)footprint->size(); pind++)
			{
				if(cell.x == footprint->at(pind).x && cell.y == footprint->at(pind).y)
					break;
			}
			if(pind == (int)footprint->size()) footprint->push_back(cell);

			prev_inside = 1;

#if DEBUG
//			printf("Added pt to footprint: %f %f", pt.x, pt.y);
#endif
		}
		else{
			prev_inside = 0;
		}

      }
	  else
	  {
#if DEBUG
		//printf("Skipping pt: %f %f", pt.x, pt.y);
#endif
      }
      
      prev_discrete_x = discrete_x;
      prev_discrete_y = discrete_y;

    }//over x_min...x_max
  }


  ///////////////////////////
  // Do the same for the cart
  ///////////////////////////
  vector<sbpl_2Dpt_t> bounding_polygon_cart;
  unsigned int find_cart;
  double max_x_cart = -INFINITECOST, min_x_cart = INFINITECOST, max_y_cart = -INFINITECOST, min_y_cart = INFINITECOST;
  sbpl_2Dpt_t pt_cart;// = {0,0};
  pt_cart.x = 0;
  pt_cart.y = 0;  
sbpl_2Dpt_t cart_offset_pt;
  cart_offset_pt.x = cos(pose.theta)*EnvNAVXYTHETACARTLATCfg.CartOffset.x - sin(pose.theta)*EnvNAVXYTHETACARTLATCfg.CartOffset.y + pose.x;
  cart_offset_pt.y = sin(pose.theta)*EnvNAVXYTHETACARTLATCfg.CartOffset.x + cos(pose.theta)*EnvNAVXYTHETACARTLATCfg.CartOffset.y + pose.y;
  double cart_angle_global = pose.theta + pose.cartangle;
  for(find_cart = 0; find_cart < EnvNAVXYTHETACARTLATCfg.CartPolygon.size(); find_cart++){
    
    //rotate and translate the corner of the robot
    pt_cart = EnvNAVXYTHETACARTLATCfg.CartPolygon[find_cart];
    
    //rotate and translate the point
    sbpl_2Dpt_t corner_cart;
    corner_cart.x = cos(cart_angle_global)*pt_cart.x - sin(cart_angle_global)*pt_cart.y + cart_offset_pt.x;
    corner_cart.y = sin(cart_angle_global)*pt_cart.x + cos(cart_angle_global)*pt_cart.y + cart_offset_pt.y;
    bounding_polygon_cart.push_back(corner_cart);
#if DEBUG
//    printf("Pt: %f %f, Corner: %f %f", pt.x, pt.y, corner.x, corner.y);
#endif
    if(corner_cart.x < min_x_cart || find_cart==0){
      min_x_cart = corner_cart.x;
    }
    if(corner_cart.x > max_x_cart || find_cart==0){
      max_x_cart = corner_cart.x;
    }
    if(corner_cart.y < min_y_cart || find_cart==0){
      min_y_cart = corner_cart.y;
    }
    if(corner_cart.y > max_y_cart || find_cart==0){
      max_y_cart = corner_cart.y;
    }
    
  }

#if DEBUG
//  printf("Footprint bounding box: %f %f %f %f", min_x, max_x, min_y, max_y);
#endif
  //initialize previous values to something that will fail the if condition during the first iteration in the for loop
  int prev_discrete_x_cart = CONTXY2DISC(pt_cart.x, EnvNAVXYTHETACARTLATCfg.cellsize_m) + 1; 
  int prev_discrete_y_cart = CONTXY2DISC(pt_cart.y, EnvNAVXYTHETACARTLATCfg.cellsize_m) + 1;
  int prev_inside_cart = 0;
  int discrete_x_cart;
  int discrete_y_cart;

  for(double x_c=min_x_cart; x_c<=max_x_cart; x_c+=EnvNAVXYTHETACARTLATCfg.cellsize_m/3){
    for(double y_c=min_y_cart; y_c<=max_y_cart; y_c+=EnvNAVXYTHETACARTLATCfg.cellsize_m/3){
      pt_cart.x = x_c;
      pt_cart.y = y_c;
      discrete_x_cart = CONTXY2DISC(pt_cart.x, EnvNAVXYTHETACARTLATCfg.cellsize_m);
      discrete_y_cart = CONTXY2DISC(pt_cart.y, EnvNAVXYTHETACARTLATCfg.cellsize_m);
      
      //see if we just tested this point
      if(discrete_x_cart != prev_discrete_x_cart || discrete_y_cart != prev_discrete_y_cart || prev_inside_cart==0){

#if DEBUG
//		printf("Testing point: %f %f Discrete: %d %d", pt.x, pt.y, discrete_x, discrete_y);
#endif
	
		if(IsInsideFootprint(pt_cart, &bounding_polygon_cart)){
		//convert to a grid point

#if DEBUG
//			printf("Pt Inside %f %f", pt.x, pt.y);
#endif

			sbpl_2Dcell_t cell_cart;
			cell_cart.x = discrete_x_cart;
			cell_cart.y = discrete_y_cart;

			//insert point if not there already
			int pind_c = 0;
			for(pind_c = 0; pind_c < (int)footprint->size(); pind_c++)
			{
				if(cell_cart.x == footprint->at(pind_c).x && cell_cart.y == footprint->at(pind_c).y)
					break;
			}
			if(pind_c == (int)footprint->size()) footprint->push_back(cell_cart);

			prev_inside_cart = 1;

#if DEBUG
//			printf("Added pt to footprint: %f %f", pt.x, pt.y);
#endif
		}
		else{
			prev_inside_cart = 0;
		}

      }
	  else
	  {
#if DEBUG
		//printf("Skipping pt: %f %f", pt.x, pt.y);
#endif
      }
      
      prev_discrete_x_cart = discrete_x_cart;
      prev_discrete_y_cart = discrete_y_cart;

    }//over x_min...x_max
  }
}

EnvNAVXYTHETACARTLAT3Dpt_t EnvironmentNAVXYTHETACARTLATTICE::getCartCenter(EnvNAVXYTHETACARTLAT3Dpt_t pose, sbpl_2Dpt_t cart_center_offset)
{  
  EnvNAVXYTHETACARTLAT3Dpt_t cart_offset_pt;
  cart_offset_pt.x = cos(pose.theta)*EnvNAVXYTHETACARTLATCfg.CartOffset.x - sin(pose.theta)*EnvNAVXYTHETACARTLATCfg.CartOffset.y + pose.x;
  cart_offset_pt.y = sin(pose.theta)*EnvNAVXYTHETACARTLATCfg.CartOffset.x + cos(pose.theta)*EnvNAVXYTHETACARTLATCfg.CartOffset.y + pose.y;
  cart_offset_pt.theta = pose.theta + pose.cartangle;

  EnvNAVXYTHETACARTLAT3Dpt_t corner_cart;
  corner_cart.x = cos(cart_offset_pt.theta)*cart_center_offset.x - sin(cart_offset_pt.theta)*cart_center_offset.y + cart_offset_pt.x;
  corner_cart.y = sin(cart_offset_pt.theta)*cart_center_offset.x + cos(cart_offset_pt.theta)*cart_center_offset.y + cart_offset_pt.y;
  corner_cart.theta = cart_offset_pt.theta;
  corner_cart.cartangle = pose.cartangle;
  return corner_cart;
}

void EnvironmentNAVXYTHETACARTLATTICE::RemoveSourceFootprint(EnvNAVXYTHETACARTLAT3Dpt_t sourcepose, vector<sbpl_2Dcell_t>* footprint)
{  
	vector<sbpl_2Dcell_t> sourcefootprint;

	//compute source footprint
	CalculateFootprintForPose(sourcepose, &sourcefootprint);

	//now remove the source cells from the footprint
	for(int sind = 0; sind < (int)sourcefootprint.size(); sind++)
	{
		for(int find = 0; find < (int)footprint->size(); find++)
		{
			if(sourcefootprint.at(sind).x == footprint->at(find).x && sourcefootprint.at(sind).y == footprint->at(find).y)
			{
				footprint->erase(footprint->begin() + find);
				break;
			}
		}//over footprint
	}//over source
}


//------------------------------------------------------------------------------

//------------------------------Heuristic computation--------------------------


void EnvironmentNAVXYTHETACARTLATTICE::EnsureHeuristicsUpdated(bool bGoalHeuristics)
{

	if(bNeedtoRecomputeStartHeuristics && !bGoalHeuristics)
	{
		grid2Dsearchfromstart->search(EnvNAVXYTHETACARTLATCfg.Grid2D, EnvNAVXYTHETACARTLATCfg.cost_inscribed_thresh, 
			EnvNAVXYTHETACARTLATCfg.StartX_c, EnvNAVXYTHETACARTLATCfg.StartY_c, EnvNAVXYTHETACARTLATCfg.EndX_c, EnvNAVXYTHETACARTLATCfg.EndY_c, 
			SBPL_2DGRIDSEARCH_TERM_CONDITION_TWOTIMESOPTPATH); 
		bNeedtoRecomputeStartHeuristics = false;
		ROS_DEBUG("2dsolcost_infullunits=%d", (int)(grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(EnvNAVXYTHETACARTLATCfg.EndX_c, EnvNAVXYTHETACARTLATCfg.EndY_c)
			/EnvNAVXYTHETACARTLATCfg.nominalvel_mpersecs));

	}


	if(bNeedtoRecomputeGoalHeuristics && bGoalHeuristics)
	{
		grid2Dsearchfromgoal->search(EnvNAVXYTHETACARTLATCfg.Grid2D, EnvNAVXYTHETACARTLATCfg.cost_inscribed_thresh, 
			EnvNAVXYTHETACARTLATCfg.EndX_c, EnvNAVXYTHETACARTLATCfg.EndY_c, EnvNAVXYTHETACARTLATCfg.StartX_c, EnvNAVXYTHETACARTLATCfg.StartY_c,  
			SBPL_2DGRIDSEARCH_TERM_CONDITION_TWOTIMESOPTPATH); 
		bNeedtoRecomputeGoalHeuristics = false;
		ROS_DEBUG("2dsolcost_infullunits=%d", (int)(grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(EnvNAVXYTHETACARTLATCfg.StartX_c, EnvNAVXYTHETACARTLATCfg.StartY_c)
			/EnvNAVXYTHETACARTLATCfg.nominalvel_mpersecs));

	}


}



void EnvironmentNAVXYTHETACARTLATTICE::ComputeHeuristicValues()
{
	//whatever necessary pre-computation of heuristic values is done here 
	ROS_DEBUG("Precomputing heuristics...");
	
	//allocated 2D grid searches
	grid2Dsearchfromstart = new SBPL2DGridSearch(EnvNAVXYTHETACARTLATCfg.EnvWidth_c, EnvNAVXYTHETACARTLATCfg.EnvHeight_c, (float)EnvNAVXYTHETACARTLATCfg.cellsize_m);
	grid2Dsearchfromgoal = new SBPL2DGridSearch(EnvNAVXYTHETACARTLATCfg.EnvWidth_c, EnvNAVXYTHETACARTLATCfg.EnvHeight_c, (float)EnvNAVXYTHETACARTLATCfg.cellsize_m); 

	//set OPEN type to sliding buckets
	grid2Dsearchfromstart->setOPENdatastructure(SBPL_2DGRIDSEARCH_OPENTYPE_SLIDINGBUCKETS); 
	grid2Dsearchfromgoal->setOPENdatastructure(SBPL_2DGRIDSEARCH_OPENTYPE_SLIDINGBUCKETS);

	ROS_DEBUG("done");

}

//------------debugging functions---------------------------------------------
bool EnvironmentNAVXYTHETACARTLATTICE::CheckQuant(FILE* fOut) 
{

  for(double theta  = -10; theta < 10; theta += 2.0*PI_CONST/NAVXYTHETACARTLAT_THETADIRS*0.01)
    {
		int nTheta = ContTheta2Disc(theta, NAVXYTHETACARTLAT_THETADIRS);
		double newTheta = DiscTheta2Cont(nTheta, NAVXYTHETACARTLAT_THETADIRS);
		int nnewTheta = ContTheta2Disc(newTheta, NAVXYTHETACARTLAT_THETADIRS);

		SBPL_FPRINTF(fOut, "theta=%f(%f)->%d->%f->%d", theta, theta*180/PI_CONST, nTheta, newTheta, nnewTheta);

        if(nTheta != nnewTheta)
        {
            ROS_ERROR("invalid quantization");                     
            return false;
        }
    }

  return true;
}



//-----------------------------------------------------------------------------

//-----------interface with outside functions-----------------------------------
bool EnvironmentNAVXYTHETACARTLATTICE::InitializeEnv(const char* sEnvFile, const vector<sbpl_2Dpt_t>& perimeterptsV, const vector<sbpl_2Dpt_t>& cartperimeterptsV, sbpl_2Dpt_t& cart_offset, const char* sMotPrimFile)
{
	EnvNAVXYTHETACARTLATCfg.FootprintPolygon = perimeterptsV;
  EnvNAVXYTHETACARTLATCfg.CartPolygon = cartperimeterptsV;
  EnvNAVXYTHETACARTLATCfg.CartOffset = cart_offset;

	FILE* fCfg = fopen(sEnvFile, "r");
	if(fCfg == NULL)
	{
		ROS_ERROR("unable to open %s", sEnvFile);
		initialized_ = false;
    return false;
	}
	ReadConfiguration(fCfg);

	if(sMotPrimFile != NULL)
	{
		FILE* fMotPrim = fopen(sMotPrimFile, "r");
		if(fMotPrim == NULL)
		{
			ROS_ERROR("unable to open %s", sMotPrimFile);
			initialized_ = false;
      return false;
		}
		if(ReadMotionPrimitives(fMotPrim) == false)
		{
			ROS_ERROR("failed to read in motion primitive file");
			initialized_ = false;
      return false;
		}
		InitGeneral(&EnvNAVXYTHETACARTLATCfg.mprimV);
	}
	else
		InitGeneral(NULL);

	ROS_DEBUG("size of env: %d by %d", EnvNAVXYTHETACARTLATCfg.EnvWidth_c, EnvNAVXYTHETACARTLATCfg.EnvHeight_c);

	return true;
}


bool EnvironmentNAVXYTHETACARTLATTICE::InitializeEnv(const char* sEnvFile)
{

	FILE* fCfg = fopen(sEnvFile, "r");
	if(fCfg == NULL)
	{
		ROS_ERROR("unable to open %s", sEnvFile);
		initialized_ = false;
    return false;
	}
	ReadConfiguration(fCfg);

	InitGeneral(NULL);


	return true;
}



bool EnvironmentNAVXYTHETACARTLATTICE::InitializeEnv(int width, int height,
                                                 const unsigned char* mapdata,
                                                 double startx, double starty, double starttheta, double startcartangle,
                                                 double goalx, double goaly, double goaltheta, double goalcartangle,
                                                 double goaltol_x, double goaltol_y, double goaltol_theta, double goaltol_cartangle,
                                                 const vector<sbpl_2Dpt_t> & perimeterptsV,
                                                 const sbpl_2Dpt_t & cart_offset,
                                                 const vector<sbpl_2Dpt_t> & cart_perimeterptsV,
                                                 double cellsize_m, double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
                                                 unsigned char obsthresh,  const char* sMotPrimFile)
{

	ROS_DEBUG("env: initialize with width=%d height=%d start=%.3f %.3f %.3f goalx=%.3f %.3f %.3f cellsize=%.3f nomvel=%.3f timetoturn=%.3f, obsthresh=%d",
		width, height, startx, starty, starttheta, goalx, goaly, goaltheta, cellsize_m, nominalvel_mpersecs, timetoturn45degsinplace_secs, obsthresh);

	ROS_DEBUG("perimeter has size=%zu", perimeterptsV.size());

	for(int i = 0; i < (int)perimeterptsV.size(); i++)
	{
		ROS_DEBUG("perimeter(%d) = %.4f %.4f", i, perimeterptsV.at(i).x, perimeterptsV.at(i).y);
	}


	EnvNAVXYTHETACARTLATCfg.obsthresh = obsthresh;

	//TODO - need to set the tolerance as well

	SetConfiguration(width, 
                   height,
                   mapdata,
                   CONTXY2DISC(startx, cellsize_m), 
                   CONTXY2DISC(starty, cellsize_m), 
                   ContTheta2Disc(starttheta, NAVXYTHETACARTLAT_THETADIRS),
                   ContTheta2Disc(startcartangle, CART_THETADIRS),
                   CONTXY2DISC(goalx, cellsize_m), 
                   CONTXY2DISC(goaly, cellsize_m), 
                   ContTheta2Disc(goaltheta, NAVXYTHETACARTLAT_THETADIRS),
                   ContTheta2Disc(goalcartangle, CART_THETADIRS),
                   cellsize_m, 
                   nominalvel_mpersecs, 
                   timetoturn45degsinplace_secs, 
                   perimeterptsV,
                   cart_perimeterptsV,
                   cart_offset);

	if(sMotPrimFile != NULL)
	{
		FILE* fMotPrim = fopen(sMotPrimFile, "r");
		if(fMotPrim == NULL)
		{
			ROS_ERROR("unable to open %s", sMotPrimFile);
			initialized_ = false;
      return false;
		}

		if(ReadMotionPrimitives(fMotPrim) == false)
		{
			ROS_ERROR("failed to read in motion primitive file");
			initialized_ = false;
      return false;
		}
	}

	if(EnvNAVXYTHETACARTLATCfg.mprimV.size() != 0)
	{
		InitGeneral(&EnvNAVXYTHETACARTLATCfg.mprimV);
	}
	else
		InitGeneral(NULL);
  PrintFootprint();
	return true;
}

void EnvironmentNAVXYTHETACARTLATTICE::PrintFootprint()
{

  /*
    EnvNAVXYTHETACARTLAT3Dpt_t temppose;
    FILE *fid = fopen("/tmp/footprint_plot.m","wt");
    ROS_DEBUG ("Print footprint to file");
    for(unsigned int i=0; i < NAVXYTHETACARTLAT_THETADIRS; i++)
    {
    vector<sbpl_2Dcell_t> footprint;
    temppose.x = 0.0 + 3.0*i;
    temppose.y = 0.0;
    temppose.theta = (i*2*M_PI)/NAVXYTHETACARTLAT_THETADIRS;
    temppose.cartangle = 0.0;
    CalculateFootprintForPose(temppose, &footprint);
    for(int i = 0; i < (int) footprint.size(); i++)
    {
    fprintf(fid, "%d %d %.3f %.3f", footprint.at(i).x, footprint.at(i).y, 
    DISCXY2CONT(footprint.at(i).x, EnvNAVXYTHETACARTLATCfg.cellsize_m), 
              DISCXY2CONT(footprint.at(i).y, EnvNAVXYTHETACARTLATCfg.cellsize_m));
              }
              }
              fclose(fid);*/
  //    printf("number of cells in footprint of the robot = %d", footprint.size());
  //    printf("footprint cells (size=%d):", footprint.size());

  /*	temppose.x = DISCXY2CONT(0, EnvNAVXYTHETACARTLATCfg.cellsize_m);
	temppose.y = DISCXY2CONT(0, EnvNAVXYTHETACARTLATCfg.cellsize_m);
	temppose.theta = DiscTheta2Cont(pMotPrim->starttheta_c, NAVXYTHETACARTLAT_THETADIRS);

	temppose.x = EnvNAVXYTHETACARTLATCfg.;
	temppose.y = 0.0;
	temppose.theta = 0.0;
  temppose.cartangle = 0.0;  
  */
  double cart_angle;
  for(unsigned int i=0; i < CART_THETADIRS; i++)
  {
    cart_angle = CartDiscTheta2Cont(i, CART_THETADIRS);
    ROS_DEBUG("Cart angle: discretized: %d, actual: %f",i,cart_angle);
  }

  int cart_angle_d;
  for(int i=-10; i <= 10; i++)
  {
    cart_angle_d = CartContTheta2Disc(i*MAX_CART_ANGLE/10.0, CART_THETADIRS);
    ROS_DEBUG("Cart angle: actual: %f, discretized: %d",i*MAX_CART_ANGLE/10.0,cart_angle_d);
  }

  /*
  FILE *fid_env = fopen("/tmp/env.m","wt");
  for (int y = 0; y < EnvNAVXYTHETACARTLATCfg.EnvHeight_c; y++) {
    for (int x = 0; x < EnvNAVXYTHETACARTLATCfg.EnvWidth_c; x++) {
      fprintf(fid_env,"%d ",EnvNAVXYTHETACARTLATCfg.Grid2D[x][y]);
    }
    fprintf(fid_env,"");
  }
  fclose(fid_env); */

}


bool EnvironmentNAVXYTHETACARTLATTICE::InitGeneral(vector<SBPL_xythetacart_mprimitive>* motionprimitiveV) {


  //Initialize other parameters of the environment
  InitializeEnvConfig(motionprimitiveV);
  
  //initialize Environment
  InitializeEnvironment();
  
  //pre-compute heuristics
  ComputeHeuristicValues();

  return true;
}

bool EnvironmentNAVXYTHETACARTLATTICE::InitializeMDPCfg(MDPConfig *MDPCfg)
{
	//initialize MDPCfg with the start and goal ids	
	MDPCfg->goalstateid = EnvNAVXYTHETACARTLAT.goalstateid;
	MDPCfg->startstateid = EnvNAVXYTHETACARTLAT.startstateid;

	return true;
}


void EnvironmentNAVXYTHETACARTLATTICE::PrintHeuristicValues()
{
	FILE* fHeur = fopen("heur.txt", "w");
	SBPL2DGridSearch* grid2Dsearch = NULL;
	
	for(int i = 0; i < 2; i++)
	{
		if(i == 0 && grid2Dsearchfromstart != NULL)
		{
			grid2Dsearch = grid2Dsearchfromstart;
			SBPL_FPRINTF(fHeur, "Start heuristics:");
		}
		else if(i == 1 && grid2Dsearchfromgoal != NULL)
		{
			grid2Dsearch = grid2Dsearchfromgoal;
			SBPL_FPRINTF(fHeur, "Goal heuristics:");
		}
		else
			continue;

		for (int y = 0; y < EnvNAVXYTHETACARTLATCfg.EnvHeight_c; y++) {
			for (int x = 0; x < EnvNAVXYTHETACARTLATCfg.EnvWidth_c; x++) {
				if(grid2Dsearch->getlowerboundoncostfromstart_inmm(x, y) < INFINITECOST)
					SBPL_FPRINTF(fHeur, "%5d ", grid2Dsearch->getlowerboundoncostfromstart_inmm(x, y));
			else
				SBPL_FPRINTF(fHeur, "XXXXX ");
			}
			SBPL_FPRINTF(fHeur, " ");
		}
	}
	fclose(fHeur);
}




void EnvironmentNAVXYTHETACARTLATTICE::SetAllPreds(CMDPSTATE* state)
{
	//implement this if the planner needs access to predecessors
	
	ROS_ERROR("ERROR in EnvNAVXYTHETACARTLAT... function: SetAllPreds is undefined");
  return;
}


void EnvironmentNAVXYTHETACARTLATTICE::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
	GetSuccs(SourceStateID, SuccIDV, CostV, NULL);
}





const EnvNAVXYTHETACARTLATConfig_t* EnvironmentNAVXYTHETACARTLATTICE::GetEnvNavConfig() {
  return &EnvNAVXYTHETACARTLATCfg;
}



bool EnvironmentNAVXYTHETACARTLATTICE::UpdateCost(int x, int y, unsigned char newcost)
{

#if DEBUG
  //fprintf(fDeb, "Cost updated for cell %d %d from old cost=%d to new cost=%d", x,y,EnvNAVXYTHETACARTLATCfg.Grid2D[x][y], newcost);
#endif

    EnvNAVXYTHETACARTLATCfg.Grid2D[x][y] = newcost;

	bNeedtoRecomputeStartHeuristics = true;
	bNeedtoRecomputeGoalHeuristics = true;

    return true;
}


void EnvironmentNAVXYTHETACARTLATTICE::PrintEnv_Config(FILE* fOut)
{

	//implement this if the planner needs to print out EnvNAVXYTHETACARTLAT. configuration
	
	ROS_ERROR("ERROR in EnvNAVXYTHETACARTLAT... function: PrintEnv_Config is undefined");
  return;
}

void EnvironmentNAVXYTHETACARTLATTICE::PrintTimeStat(FILE* fOut)
{

#if TIME_DEBUG
    SBPL_FPRINTF(fOut, "time3_addallout = %f secs, time_gethash = %f secs, time_createhash = %f secs, time_getsuccs = %f",
            time3_addallout/(double)CLOCKS_PER_SEC, time_gethash/(double)CLOCKS_PER_SEC, 
            time_createhash/(double)CLOCKS_PER_SEC, time_getsuccs/(double)CLOCKS_PER_SEC);
#endif
}



bool EnvironmentNAVXYTHETACARTLATTICE::IsObstacle(int x, int y)
{

#if DEBUG
  SBPL_FPRINTF(fDeb, "Status of cell %d %d is queried. Its cost=%d", x,y,EnvNAVXYTHETACARTLATCfg.Grid2D[x][y]);
#endif


	return (EnvNAVXYTHETACARTLATCfg.Grid2D[x][y] >= EnvNAVXYTHETACARTLATCfg.obsthresh); 

}

// NOT USED
void EnvironmentNAVXYTHETACARTLATTICE::GetEnvParms(int *size_x, int *size_y, 
                                                   double* startx, double* starty, double*starttheta, double*startcartangle, 
                                                   double* goalx, double* goaly, double* goaltheta, double*goalcartangle,
                                                   double* cellsize_m, double* nominalvel_mpersecs, 
                                                   double* timetoturn45degsinplace_secs, unsigned char* obsthresh,
                                                   vector<SBPL_xythetacart_mprimitive>* mprimitiveV)
{
	*size_x = EnvNAVXYTHETACARTLATCfg.EnvWidth_c;
	*size_y = EnvNAVXYTHETACARTLATCfg.EnvHeight_c;

	*startx = DISCXY2CONT(EnvNAVXYTHETACARTLATCfg.StartX_c, EnvNAVXYTHETACARTLATCfg.cellsize_m);
	*starty = DISCXY2CONT(EnvNAVXYTHETACARTLATCfg.StartY_c, EnvNAVXYTHETACARTLATCfg.cellsize_m);
	*starttheta = DiscTheta2Cont(EnvNAVXYTHETACARTLATCfg.StartTheta, NAVXYTHETACARTLAT_THETADIRS);
	*startcartangle = CartDiscTheta2Cont(EnvNAVXYTHETACARTLATCfg.StartCartAngle, CART_THETADIRS);

	*goalx = DISCXY2CONT(EnvNAVXYTHETACARTLATCfg.EndX_c, EnvNAVXYTHETACARTLATCfg.cellsize_m);
	*goaly = DISCXY2CONT(EnvNAVXYTHETACARTLATCfg.EndY_c, EnvNAVXYTHETACARTLATCfg.cellsize_m);
	*goaltheta = DiscTheta2Cont(EnvNAVXYTHETACARTLATCfg.EndTheta, NAVXYTHETACARTLAT_THETADIRS);
	*goalcartangle = CartDiscTheta2Cont(EnvNAVXYTHETACARTLATCfg.EndCartAngle, CART_THETADIRS);

	*cellsize_m = EnvNAVXYTHETACARTLATCfg.cellsize_m;
	*nominalvel_mpersecs = EnvNAVXYTHETACARTLATCfg.nominalvel_mpersecs;
	*timetoturn45degsinplace_secs = EnvNAVXYTHETACARTLATCfg.timetoturn45degsinplace_secs;

	*obsthresh = EnvNAVXYTHETACARTLATCfg.obsthresh;

	*mprimitiveV = EnvNAVXYTHETACARTLATCfg.mprimV;
}

unsigned char EnvironmentNAVXYTHETACARTLATTICE::GetMapCost(int x, int y)
{
	return EnvNAVXYTHETACARTLATCfg.Grid2D[x][y];
}

bool EnvironmentNAVXYTHETACARTLATTICE::SetEnvParameter(const char* parameter, int value)
{

	if(EnvNAVXYTHETACARTLAT.bInitialized == true)
	{
		ROS_ERROR("all parameters must be set before initialization of the environment");
		return false;
	}

	ROS_DEBUG("setting parameter %s to %d", parameter, value);

	if(strcmp(parameter, "cost_inscribed_thresh") == 0)
	{
		if(value < 0 || value > 255)
		{
		  ROS_ERROR("invalid value %d for parameter %s", value, parameter);
			return false;
		}
		EnvNAVXYTHETACARTLATCfg.cost_inscribed_thresh = (unsigned char)value;
	}
	else if(strcmp(parameter, "cost_possibly_circumscribed_thresh") == 0)
	{
		if(value < 0 || value > 255)
		{
		  ROS_ERROR("invalid value %d for parameter %s", value, parameter);
			return false;
		}
		EnvNAVXYTHETACARTLATCfg.cost_possibly_circumscribed_thresh = value;
	}
	else if(strcmp(parameter, "cost_obsthresh") == 0)
	{
		if(value < 0 || value > 255)
		{
		  ROS_ERROR("invalid value %d for parameter %s", value, parameter);
			return false;
		}
		EnvNAVXYTHETACARTLATCfg.obsthresh = (unsigned char)value;
	}
	else
	{
		ROS_ERROR("invalid parameter %s", parameter);
		return false;
	}

	return true;
}

int EnvironmentNAVXYTHETACARTLATTICE::GetEnvParameter(const char* parameter)
{

	if(strcmp(parameter, "cost_inscribed_thresh") == 0)
	{
		return (int) EnvNAVXYTHETACARTLATCfg.cost_inscribed_thresh;
	}
	else if(strcmp(parameter, "cost_possibly_circumscribed_thresh") == 0)
	{
		return (int) EnvNAVXYTHETACARTLATCfg.cost_possibly_circumscribed_thresh;
	}
	else if(strcmp(parameter, "cost_obsthresh") == 0)
	{
		return (int) EnvNAVXYTHETACARTLATCfg.obsthresh;
	}
	else
	{
		ROS_ERROR("invalid parameter %s", parameter);
    throw new SBPL_Exception();
	}
}

//------------------------------------------------------------------------------



//-----------------XYTHETA Enivornment (child) class---------------------------

EnvironmentNAVXYTHETACARTLAT::~EnvironmentNAVXYTHETACARTLAT()
{
	ROS_DEBUG("destroying XYTHETACARTLAT");

	//delete the states themselves first
	for (int i = 0; i < (int)StateID2CoordTable.size(); i++)
	{
		delete StateID2CoordTable.at(i);
		StateID2CoordTable.at(i) = NULL;
	}
	StateID2CoordTable.clear();

	//delete hashtable
	if(Coord2StateIDHashTable != NULL)
	{
		delete [] Coord2StateIDHashTable;
		Coord2StateIDHashTable = NULL;
	}	
	if(Coord2StateIDHashTable_lookup != NULL)
	{
		delete [] Coord2StateIDHashTable_lookup;
		Coord2StateIDHashTable_lookup = NULL;
	}

}


void EnvironmentNAVXYTHETACARTLAT::GetCoordFromState(int stateID, int& x, int& y, int& theta, int &cartangle) const {
  EnvNAVXYTHETACARTLATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
  x = HashEntry->X;
  y = HashEntry->Y;
  theta = HashEntry->Theta;
  cartangle = HashEntry->CartAngle;
}

int EnvironmentNAVXYTHETACARTLAT::GetStateFromCoord(int x, int y, int theta, int cartangle) {

   EnvNAVXYTHETACARTLATHashEntry_t* OutHashEntry;
   if((OutHashEntry = (this->*GetHashEntry)(x, y, theta,cartangle)) == NULL){
        //have to create a new entry
     OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta,cartangle);
    }
    return OutHashEntry->stateID;
}

bool EnvironmentNAVXYTHETACARTLAT::ConvertStateIDPathintoXYThetaPath(vector<int>* stateIDPath, vector<EnvNAVXYTHETACARTLAT3Dpt_t>* xythetaPath)
{
	vector<EnvNAVXYTHETACARTLATAction_t*> actionV;
	vector<int> CostV;
	vector<int> SuccIDV;
	int targetx_c, targety_c, targettheta_c, targetcartangle_c;
	int sourcex_c, sourcey_c, sourcetheta_c, sourcecartangle_c;

	ROS_DEBUG("checks_cart=%ld", checks_cart);

	xythetaPath->clear();

#if DEBUG
	SBPL_FPRINTF(fDeb, "converting stateid path into coordinates:");
#endif

	for(int pind = 0; pind < (int)(stateIDPath->size())-1; pind++)
	{
		int sourceID = stateIDPath->at(pind);
		int targetID = stateIDPath->at(pind+1);

#if DEBUG
		GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c, sourcecartangle_c);
#endif


		//get successors and pick the target via the cheapest action
		SuccIDV.clear();
		CostV.clear();
		actionV.clear();
		GetSuccs(sourceID, &SuccIDV, &CostV, &actionV);
		
		int bestcost = INFINITECOST;
		int bestsind = -1;

#if DEBUG
		GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c, sourcecartangle_c);
		GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c, targetcartangle_c);
		SBPL_FPRINTF(fDeb, "looking for %d %d %d %d -> %d %d %d %d (numofsuccs=%zu)", sourcex_c, sourcey_c, sourcetheta_c, sourcecartangle_c,
            targetx_c, targety_c, targettheta_c, targetcartangle_c, SuccIDV.size()); 

#endif

		for(int sind = 0; sind < (int)SuccIDV.size(); sind++)
		{

#if DEBUG
      int x_c, y_c, theta_c, cartangle_c;
      GetCoordFromState(SuccIDV[sind], x_c, y_c, theta_c, cartangle_c);
      SBPL_FPRINTF(fDeb, "succ: %d %d %d %d", x_c, y_c, theta_c, cartangle_c); 
#endif

			if(SuccIDV[sind] == targetID && CostV[sind] <= bestcost)
			{
				bestcost = CostV[sind];
				bestsind = sind;
			}
		}
		if(bestsind == -1)
		{
			ROS_ERROR("successor not found for transition:");
			GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c, sourcecartangle_c);
			GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c, targetcartangle_c);
			ROS_ERROR("%d %d %d %d -> %d %d %d %d", sourcex_c, sourcey_c, sourcetheta_c, sourcecartangle_c,
             targetx_c, targety_c, targettheta_c, targetcartangle_c); 
      return false;
		}

		//now push in the actual path
		int sourcex_c, sourcey_c, sourcetheta_c, sourcecartangle_c;
		GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c, sourcecartangle_c);
		double sourcex, sourcey;
		sourcex = DISCXY2CONT(sourcex_c, EnvNAVXYTHETACARTLATCfg.cellsize_m);
		sourcey = DISCXY2CONT(sourcey_c, EnvNAVXYTHETACARTLATCfg.cellsize_m);
		//TODO - when there are no motion primitives we should still print source state
		for(int ipind = 0; ipind < ((int)actionV[bestsind]->intermptV.size())-1; ipind++) 
		{
			//translate appropriately
			EnvNAVXYTHETACARTLAT3Dpt_t intermpt = actionV[bestsind]->intermptV[ipind];
			intermpt.x += sourcex;
			intermpt.y += sourcey;

#if DEBUG
			int nx = CONTXY2DISC(intermpt.x, EnvNAVXYTHETACARTLATCfg.cellsize_m);
			int ny = CONTXY2DISC(intermpt.y, EnvNAVXYTHETACARTLATCfg.cellsize_m);
			SBPL_FPRINTF(fDeb, "%.3f %.3f %.3f (%d %d %d cost=%d) ", 
				intermpt.x, intermpt.y, intermpt.theta, 
				nx, ny, 
				ContTheta2Disc(intermpt.theta, NAVXYTHETACARTLAT_THETADIRS), EnvNAVXYTHETACARTLATCfg.Grid2D[nx][ny]);
			if(ipind == 0) SBPL_FPRINTF(fDeb, "first (heur=%d)", GetStartHeuristic(sourceID));
			else SBPL_FPRINTF(fDeb, "");
#endif

			//store
			xythetaPath->push_back(intermpt);
		}
	}
  return true;
}


//returns the stateid if success, and -1 otherwise
int EnvironmentNAVXYTHETACARTLAT::SetGoal(double x_m, double y_m, double theta_rad, double cartangle_rad){

	int x = CONTXY2DISC(x_m, EnvNAVXYTHETACARTLATCfg.cellsize_m);
	int y = CONTXY2DISC(y_m, EnvNAVXYTHETACARTLATCfg.cellsize_m);
	int theta = ContTheta2Disc(theta_rad, NAVXYTHETACARTLAT_THETADIRS);
	int cartangle = CartContTheta2Disc(cartangle_rad, CART_THETADIRS);

	ROS_DEBUG("Env: setting goal to %.3f %.3f %.3f %.3f (%d %d %d %d)", x_m, y_m, theta_rad, cartangle_rad, x, y, theta, cartangle);

	if(!IsWithinMapCell(x,y))
	{
		ROS_ERROR("Trying to set a goal cell %d %d that is outside of map", x,y);
		return -1;
	}

  if(!IsValidConfiguration(x,y,theta,cartangle))
	{
		ROS_WARN("Goal configuration is invalid");
	}

    EnvNAVXYTHETACARTLATHashEntry_t* OutHashEntry;
    if((OutHashEntry = (this->*GetHashEntry)(x, y, theta, cartangle)) == NULL){
        //have to create a new entry
      OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta, cartangle);
    }

	//need to recompute start heuristics?
	if(EnvNAVXYTHETACARTLAT.goalstateid != OutHashEntry->stateID)
	{
		bNeedtoRecomputeStartHeuristics = true; //because termination condition may not plan all the way to the new goal
		bNeedtoRecomputeGoalHeuristics = true; //because goal heuristics change
	}



    EnvNAVXYTHETACARTLAT.goalstateid = OutHashEntry->stateID;

	EnvNAVXYTHETACARTLATCfg.EndX_c = x;
	EnvNAVXYTHETACARTLATCfg.EndY_c = y;
	EnvNAVXYTHETACARTLATCfg.EndTheta = theta;
	EnvNAVXYTHETACARTLATCfg.EndCartAngle = cartangle;

  PrintFootprint();
  return EnvNAVXYTHETACARTLAT.goalstateid;    

}


//returns the stateid if success, and -1 otherwise
int EnvironmentNAVXYTHETACARTLAT::SetStart(double x_m, double y_m, double theta_rad, double cartangle_rad){

	int x = CONTXY2DISC(x_m, EnvNAVXYTHETACARTLATCfg.cellsize_m);
	int y = CONTXY2DISC(y_m, EnvNAVXYTHETACARTLATCfg.cellsize_m); 
	int theta = ContTheta2Disc(theta_rad, NAVXYTHETACARTLAT_THETADIRS);
	int cartangle = CartContTheta2Disc(cartangle_rad, CART_THETADIRS);

	if(!IsWithinMapCell(x,y))
	{
		ROS_ERROR("trying to set a start cell %d %d that is outside of map", x,y);
		return -1;
	}

 ROS_DEBUG("Env: setting start to %.3f %.3f %.3f %.3f (%d %d %d %d)", x_m, y_m, theta_rad, cartangle_rad, x, y, theta, cartangle);

  if(!IsValidConfiguration(x,y,theta,cartangle))
	{
		ROS_WARN("start configuration %d %d %d %d is invalid", x,y,theta,cartangle);
	}

    EnvNAVXYTHETACARTLATHashEntry_t* OutHashEntry;
    if((OutHashEntry = (this->*GetHashEntry)(x, y, theta, cartangle)) == NULL){
        //have to create a new entry
      OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta, cartangle);
    }

	//need to recompute start heuristics?
	if(EnvNAVXYTHETACARTLAT.startstateid != OutHashEntry->stateID)
	{
		bNeedtoRecomputeStartHeuristics = true;
		bNeedtoRecomputeGoalHeuristics = true; //because termination condition can be not all states TODO - make it dependent on term. condition
	}

	//set start
    EnvNAVXYTHETACARTLAT.startstateid = OutHashEntry->stateID;
	EnvNAVXYTHETACARTLATCfg.StartX_c = x;
	EnvNAVXYTHETACARTLATCfg.StartY_c = y;
	EnvNAVXYTHETACARTLATCfg.StartTheta = theta;
	EnvNAVXYTHETACARTLATCfg.StartCartAngle = cartangle;

    return EnvNAVXYTHETACARTLAT.startstateid;    

}

void EnvironmentNAVXYTHETACARTLAT::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG
	if(stateID >= (int)StateID2CoordTable.size())
	{
		ROS_ERROR("ERROR in EnvNAVXYTHETACARTLAT... function: stateID illegal (2)");
	}
#endif

	if(fOut == NULL)
		fOut = stdout;

	EnvNAVXYTHETACARTLATHashEntry_t* HashEntry = StateID2CoordTable[stateID];

	if(stateID == EnvNAVXYTHETACARTLAT.goalstateid && bVerbose)
	{
		SBPL_FPRINTF(fOut, "This state is a goal state:");
	}

    if(bVerbose)
    	SBPL_FPRINTF(fOut, "X=%d Y=%d Theta=%d CartAngle=%d", HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->CartAngle);
    else
    	SBPL_FPRINTF(fOut, "%.3f %.3f %.3f %.3f", DISCXY2CONT(HashEntry->X, EnvNAVXYTHETACARTLATCfg.cellsize_m), DISCXY2CONT(HashEntry->Y,EnvNAVXYTHETACARTLATCfg.cellsize_m), 
              DiscTheta2Cont(HashEntry->Theta, NAVXYTHETACARTLAT_THETADIRS), CartDiscTheta2Cont(HashEntry->CartAngle, CART_THETADIRS));

}


EnvNAVXYTHETACARTLATHashEntry_t* EnvironmentNAVXYTHETACARTLAT::GetHashEntry_lookup(int X, int Y, int Theta, int CartAngle)
{
	int index = XYTHETACART2INDEX(X,Y,Theta,CartAngle);	
	return Coord2StateIDHashTable_lookup[index];
}


EnvNAVXYTHETACARTLATHashEntry_t* EnvironmentNAVXYTHETACARTLAT::GetHashEntry_hash(int X, int Y, int Theta, int CartAngle)
{

#if TIME_DEBUG
	clock_t currenttime = clock();
#endif

	int binid = GETHASHBIN(X, Y, Theta, CartAngle);	

#if DEBUG
	if ((int)Coord2StateIDHashTable[binid].size() > 5)
	{
		SBPL_FPRINTF(fDeb, "WARNING: Hash table has a bin %d (X=%d Y=%d) of size %zu", 
			binid, X, Y, Coord2StateIDHashTable[binid].size());
		
		PrintHashTableHist(fDeb);		
	}
#endif

	//iterate over the states in the bin and select the perfect match
	vector<EnvNAVXYTHETACARTLATHashEntry_t*>* binV = &Coord2StateIDHashTable[binid];
	for(int ind = 0; ind < (int)binV->size(); ind++)
	{
		EnvNAVXYTHETACARTLATHashEntry_t* hashentry = binV->at(ind);
		if( hashentry->X == X  && hashentry->Y == Y && hashentry->Theta == Theta && hashentry->CartAngle == CartAngle)
		{
#if TIME_DEBUG
			time_gethash += clock()-currenttime;
#endif
			return hashentry;
		}
	}

#if TIME_DEBUG	
	time_gethash += clock()-currenttime;
#endif

	return NULL;	  
}

EnvNAVXYTHETACARTLATHashEntry_t* EnvironmentNAVXYTHETACARTLAT::CreateNewHashEntry_lookup(int X, int Y, int Theta, int CartAngle) 
{
	int i;

#if TIME_DEBUG	
	clock_t currenttime = clock();
#endif

	EnvNAVXYTHETACARTLATHashEntry_t* HashEntry = new EnvNAVXYTHETACARTLATHashEntry_t;

	HashEntry->X = X;
	HashEntry->Y = Y;
	HashEntry->Theta = Theta;
	HashEntry->CartAngle = CartAngle;
	HashEntry->iteration = 0;

	HashEntry->stateID = StateID2CoordTable.size();

	//insert into the tables
	StateID2CoordTable.push_back(HashEntry);

	int index = XYTHETACART2INDEX(X,Y,Theta,CartAngle);

#if DEBUG
	if(Coord2StateIDHashTable_lookup[index] != NULL)
	{
		ROS_ERROR("creating hash entry for non-NULL hashentry");
    throw new SBPL_Exception();
	}
#endif

	Coord2StateIDHashTable_lookup[index] = 	HashEntry;

	//insert into and initialize the mappings
	int* entry = new int [NUMOFINDICES_STATEID2IND]; 
	StateID2IndexMapping.push_back(entry);
	for(i = 0; i < NUMOFINDICES_STATEID2IND; i++)
	{
		StateID2IndexMapping[HashEntry->stateID][i] = -1;
	}

	if(HashEntry->stateID != (int)StateID2IndexMapping.size()-1)
	{
		ROS_ERROR("ERROR in Env... function: last state has incorrect stateID");
    throw new SBPL_Exception();
	}

#if TIME_DEBUG
	time_createhash += clock()-currenttime;
#endif

	return HashEntry;
}




EnvNAVXYTHETACARTLATHashEntry_t* EnvironmentNAVXYTHETACARTLAT::CreateNewHashEntry_hash(int X, int Y, int Theta, int CartAngle) 
{
	int i;

#if TIME_DEBUG	
	clock_t currenttime = clock();
#endif

	EnvNAVXYTHETACARTLATHashEntry_t* HashEntry = new EnvNAVXYTHETACARTLATHashEntry_t;

	HashEntry->X = X;
	HashEntry->Y = Y;
	HashEntry->Theta = Theta;
	HashEntry->CartAngle = CartAngle;
	HashEntry->iteration = 0;

	HashEntry->stateID = StateID2CoordTable.size();

	//insert into the tables
	StateID2CoordTable.push_back(HashEntry);


	//get the hash table bin
	i = GETHASHBIN(HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->CartAngle); 

	//insert the entry into the bin
    Coord2StateIDHashTable[i].push_back(HashEntry);

	//insert into and initialize the mappings
	int* entry = new int [NUMOFINDICES_STATEID2IND]; 
	StateID2IndexMapping.push_back(entry);
	for(i = 0; i < NUMOFINDICES_STATEID2IND; i++)
	{
		StateID2IndexMapping[HashEntry->stateID][i] = -1;
	}

	if(HashEntry->stateID != (int)StateID2IndexMapping.size()-1)
	{
		ROS_ERROR("ERROR in Env... function: last state has incorrect stateID");
    delete HashEntry;
    return NULL; // TODO
	}

#if TIME_DEBUG
	time_createhash += clock()-currenttime;
#endif

	return HashEntry;
}



void EnvironmentNAVXYTHETACARTLAT::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<EnvNAVXYTHETACARTLATAction_t*>* actionV /*=NULL*/)
{
    int aind;

#if TIME_DEBUG
		clock_t currenttime = clock();
#endif

    //clear the successor array
    SuccIDV->clear();
    CostV->clear();
    SuccIDV->reserve(EnvNAVXYTHETACARTLATCfg.actionwidth); 
    CostV->reserve(EnvNAVXYTHETACARTLATCfg.actionwidth);
	if(actionV != NULL)
	{
		actionV->clear();
		actionV->reserve(EnvNAVXYTHETACARTLATCfg.actionwidth);
	}

	//goal state should be absorbing
	if(SourceStateID == EnvNAVXYTHETACARTLAT.goalstateid)
		return;

	//get X, Y for the state
	EnvNAVXYTHETACARTLATHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];
	
	//iterate through actions
	for (aind = 0; aind < EnvNAVXYTHETACARTLATCfg.actionwidth; aind++)
	{
		EnvNAVXYTHETACARTLATAction_t* nav3daction = &EnvNAVXYTHETACARTLATCfg.ActionsV[(int)HashEntry->Theta][aind];
    int newX = HashEntry->X + nav3daction->dX;
		int newY = HashEntry->Y + nav3daction->dY;
		int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, NAVXYTHETACARTLAT_THETADIRS);	
    // TODO
		int newCartAngle = nav3daction->endcartangle;	

    //skip the invalid cells
    if(!IsValidCell(newX, newY)) 
			continue;

		//get cost
		int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->CartAngle, nav3daction);
    if(cost >= INFINITECOST)
      continue;
    
    EnvNAVXYTHETACARTLATHashEntry_t* OutHashEntry;
		if((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta, newCartAngle)) == NULL)
		{
			//have to create a new entry
			OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta, newCartAngle);
		}
    
    SuccIDV->push_back(OutHashEntry->stateID);
    CostV->push_back(cost);
		if(actionV != NULL)
			actionV->push_back(nav3daction);
	}

#if TIME_DEBUG
		time_getsuccs += clock()-currenttime;
#endif

}

void EnvironmentNAVXYTHETACARTLAT::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{

	//TODO- to support tolerance, need: a) generate preds for goal state based on all possible goal state variable settings,
	//b) change goal check condition in gethashentry c) change getpredsofchangedcells and getsuccsofchangedcells functions

    int aind;

#if TIME_DEBUG
	clock_t currenttime = clock();
#endif

	//get X, Y for the state
	EnvNAVXYTHETACARTLATHashEntry_t* HashEntry = StateID2CoordTable[TargetStateID];

    //clear the successor array
    PredIDV->clear();
    CostV->clear();
    PredIDV->reserve(EnvNAVXYTHETACARTLATCfg.PredActionsV[(int)HashEntry->Theta].size()); 
    CostV->reserve(EnvNAVXYTHETACARTLATCfg.PredActionsV[(int)HashEntry->Theta].size());
	
	//iterate through actions
    vector<EnvNAVXYTHETACARTLATAction_t*>* actionsV = &EnvNAVXYTHETACARTLATCfg.PredActionsV[(int)HashEntry->Theta];
	for (aind = 0; aind < (int)EnvNAVXYTHETACARTLATCfg.PredActionsV[(int)HashEntry->Theta].size(); aind++)
	{

		EnvNAVXYTHETACARTLATAction_t* nav3daction = actionsV->at(aind);

    int predX = HashEntry->X - nav3daction->dX;
		int predY = HashEntry->Y - nav3daction->dY;
		int predTheta = nav3daction->starttheta;	
    int predCartAngle = nav3daction->startcartangle;
	
	
		//skip the invalid cells
    if(!IsValidCell(predX, predY))
			continue;

		//get cost
		int cost = GetActionCost(predX, predY, predTheta, predCartAngle, nav3daction);
    if(cost >= INFINITECOST)
			continue;
        
    EnvNAVXYTHETACARTLATHashEntry_t* OutHashEntry;
		if((OutHashEntry = (this->*GetHashEntry)(predX, predY, predTheta, predCartAngle)) == NULL)
		{
			//have to create a new entry
			OutHashEntry = (this->*CreateNewHashEntry)(predX, predY, predTheta, predCartAngle);
		}

    PredIDV->push_back(OutHashEntry->stateID);
    CostV->push_back(cost);
	}

#if TIME_DEBUG
		time_getsuccs += clock()-currenttime;
#endif
}

void EnvironmentNAVXYTHETACARTLAT::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{

	int cost;

#if DEBUG
	if(state->StateID >= (int)StateID2CoordTable.size())
	{
	  ROS_ERROR("ERROR in Env... function: stateID illegal");
    throw new SBPL_Exception();
	}

	if((int)state->Actions.size() != 0)
	{
		ROS_ERROR("ERROR in Env_setAllActionsandAllOutcomes: actions already exist for the state");
    throw new SBPL_Exception();
	}
#endif
	

	//goal state should be absorbing
	if(state->StateID == EnvNAVXYTHETACARTLAT.goalstateid)
		return;

	//get X, Y for the state
	EnvNAVXYTHETACARTLATHashEntry_t* HashEntry = StateID2CoordTable[state->StateID];
	
	//iterate through actions
	for (int aind = 0; aind < EnvNAVXYTHETACARTLATCfg.actionwidth; aind++)
	{
		EnvNAVXYTHETACARTLATAction_t* nav3daction = &EnvNAVXYTHETACARTLATCfg.ActionsV[(int)HashEntry->Theta][aind];
    int newX = HashEntry->X + nav3daction->dX;
		int newY = HashEntry->Y + nav3daction->dY;
		int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, NAVXYTHETACARTLAT_THETADIRS);	
    int newCartAngle = NORMALIZEDISCTHETA(nav3daction->endcartangle, CART_THETADIRS);	

    //skip the invalid cells
    if(!IsValidCell(newX, newY))
			continue;
    
		//get cost
		cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->CartAngle, nav3daction);
    if(cost >= INFINITECOST)
      continue;

		//add the action
		CMDPACTION* action = state->AddAction(aind);

#if TIME_DEBUG
		clock_t currenttime = clock();
#endif

    EnvNAVXYTHETACARTLATHashEntry_t* OutHashEntry;
		if((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta, newCartAngle)) == NULL)
		{
			//have to create a new entry
			OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta, newCartAngle);
		}
		action->AddOutcome(OutHashEntry->stateID, cost, 1.0); 
    
#if TIME_DEBUG
		time3_addallout += clock()-currenttime;
#endif

	}
}


void EnvironmentNAVXYTHETACARTLAT::GetPredsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *preds_of_changededgesIDV)
{
	nav2dcell_t cell;
	EnvNAVXYTHETACARTLAT3Dcell_t affectedcell;
	EnvNAVXYTHETACARTLATHashEntry_t* affectedHashEntry;
  
	//increment iteration for processing savings
	iteration++;
  
	for(int i = 0; i < (int)changedcellsV->size(); i++) 
	{
		cell = changedcellsV->at(i);
		
		//now iterate over all states that could potentially be affected
		for(int sind = 0; sind < (int)affectedpredstatesV.size(); sind++)
		{
			affectedcell = affectedpredstatesV.at(sind);

			//translate to correct for the offset
			affectedcell.x = affectedcell.x + cell.x;
			affectedcell.y = affectedcell.y + cell.y;

			//insert only if it was actually generated
      affectedHashEntry = (this->*GetHashEntry)(affectedcell.x, affectedcell.y, affectedcell.theta, affectedcell.cartangle);
			if(affectedHashEntry != NULL && affectedHashEntry->iteration < iteration)
			{
				preds_of_changededgesIDV->push_back(affectedHashEntry->stateID);
				affectedHashEntry->iteration = iteration; //mark as already inserted
			}
		}
	}
}

void EnvironmentNAVXYTHETACARTLAT::GetSuccsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *succs_of_changededgesIDV)
{
	nav2dcell_t cell;
	EnvNAVXYTHETACARTLAT3Dcell_t affectedcell;
	EnvNAVXYTHETACARTLATHashEntry_t* affectedHashEntry;

	ROS_ERROR("getsuccs is not supported currently");
  return;

	//increment iteration for processing savings
	iteration++;

	//TODO - check
	for(int i = 0; i < (int)changedcellsV->size(); i++) 
	{
		cell = changedcellsV->at(i);
		
		//now iterate over all states that could potentially be affected
		for(int sind = 0; sind < (int)affectedsuccstatesV.size(); sind++)
		{
			affectedcell = affectedsuccstatesV.at(sind);
      
			//translate to correct for the offset
			affectedcell.x = affectedcell.x + cell.x;
			affectedcell.y = affectedcell.y + cell.y;
      
			//insert only if it was actually generated
      affectedHashEntry = (this->*GetHashEntry)(affectedcell.x, affectedcell.y, affectedcell.theta, affectedcell.cartangle);
			if(affectedHashEntry != NULL && affectedHashEntry->iteration < iteration)
			{
				succs_of_changededgesIDV->push_back(affectedHashEntry->stateID);
				affectedHashEntry->iteration = iteration; //mark as already inserted
			}
		}
	}
}

void EnvironmentNAVXYTHETACARTLAT::InitializeEnvironment()
{
	EnvNAVXYTHETACARTLATHashEntry_t* HashEntry;

	int maxsize = EnvNAVXYTHETACARTLATCfg.EnvWidth_c*EnvNAVXYTHETACARTLATCfg.EnvHeight_c*NAVXYTHETACARTLAT_THETADIRS*CART_THETADIRS;//Needs to be changed

	if(maxsize <= SBPL_XYTHETACARTLAT_MAXSTATESFORLOOKUP)
	{
		ROS_DEBUG("environment stores states in lookup table");
    
		Coord2StateIDHashTable_lookup = new EnvNAVXYTHETACARTLATHashEntry_t*[maxsize]; 
		for(int i = 0; i < maxsize; i++)
			Coord2StateIDHashTable_lookup[i] = NULL;
		GetHashEntry = &EnvironmentNAVXYTHETACARTLAT::GetHashEntry_lookup;
		CreateNewHashEntry = &EnvironmentNAVXYTHETACARTLAT::CreateNewHashEntry_lookup;
		
		//not using hash table
		HashTableSize = 0;
		Coord2StateIDHashTable = NULL;
	}
	else
	{		
		ROS_DEBUG("environment stores states in hashtable");

		//initialize the map from Coord to StateID
		HashTableSize = 4*1024*1024; //should be power of two 
		Coord2StateIDHashTable = new vector<EnvNAVXYTHETACARTLATHashEntry_t*>[HashTableSize]; 
		GetHashEntry = &EnvironmentNAVXYTHETACARTLAT::GetHashEntry_hash;
		CreateNewHashEntry = &EnvironmentNAVXYTHETACARTLAT::CreateNewHashEntry_hash;

		//not using hash
		Coord2StateIDHashTable_lookup = NULL;
	}


	//initialize the map from StateID to Coord
	StateID2CoordTable.clear();

	//create start state 
	if((HashEntry = (this->*GetHashEntry)(EnvNAVXYTHETACARTLATCfg.StartX_c, EnvNAVXYTHETACARTLATCfg.StartY_c, EnvNAVXYTHETACARTLATCfg.StartTheta, EnvNAVXYTHETACARTLATCfg.StartCartAngle)) == NULL){
    //have to create a new entry
		HashEntry = (this->*CreateNewHashEntry)(EnvNAVXYTHETACARTLATCfg.StartX_c, EnvNAVXYTHETACARTLATCfg.StartY_c, EnvNAVXYTHETACARTLATCfg.StartTheta, EnvNAVXYTHETACARTLATCfg.StartCartAngle);
	}
	EnvNAVXYTHETACARTLAT.startstateid = HashEntry->stateID;
  
	//create goal state 
	if((HashEntry = (this->*GetHashEntry)(EnvNAVXYTHETACARTLATCfg.EndX_c, EnvNAVXYTHETACARTLATCfg.EndY_c, EnvNAVXYTHETACARTLATCfg.EndTheta, EnvNAVXYTHETACARTLATCfg.EndCartAngle)) == NULL){
        //have to create a new entry
		HashEntry = (this->*CreateNewHashEntry)(EnvNAVXYTHETACARTLATCfg.EndX_c, EnvNAVXYTHETACARTLATCfg.EndY_c, EnvNAVXYTHETACARTLATCfg.EndTheta, EnvNAVXYTHETACARTLATCfg.EndCartAngle);
	}
	EnvNAVXYTHETACARTLAT.goalstateid = HashEntry->stateID;

	//initialized
	EnvNAVXYTHETACARTLAT.bInitialized = true;

}


//examples of hash functions: map state coordinates onto a hash value
//#define GETHASHBIN(X, Y) (Y*WIDTH_Y+X) 
//here we have state coord: <X1, X2, X3, X4>
unsigned int EnvironmentNAVXYTHETACARTLAT::GETHASHBIN(unsigned int X1, unsigned int X2, unsigned int Theta, unsigned int CartAngle)
{
	return inthash(inthash(X1)+(inthash(X2)<<1)+(inthash(Theta)<<2)+(inthash(CartAngle)<<3)) & (HashTableSize-1);
}

void EnvironmentNAVXYTHETACARTLAT::PrintHashTableHist(FILE* fOut)
{
	int s0=0, s1=0, s50=0, s100=0, s200=0, s300=0, slarge=0;

	for(int  j = 0; j < HashTableSize; j++)
	{
	  if((int)Coord2StateIDHashTable[j].size() == 0)
			s0++;
		else if((int)Coord2StateIDHashTable[j].size() < 5)
			s1++;
		else if((int)Coord2StateIDHashTable[j].size() < 25)
			s50++;
		else if((int)Coord2StateIDHashTable[j].size() < 50)
			s100++;
		else if((int)Coord2StateIDHashTable[j].size() < 100)
			s200++;
		else if((int)Coord2StateIDHashTable[j].size() < 400)
			s300++;
		else
			slarge++;
	}
	SBPL_FPRINTF(fOut, "hash table histogram: 0:%d, <5:%d, <25:%d, <50:%d, <100:%d, <400:%d, >400:%d",
		s0,s1, s50, s100, s200,s300,slarge);
}

int EnvironmentNAVXYTHETACARTLAT::GetFromToHeuristic(int FromStateID, int ToStateID)
{

#if USE_HEUR==0
	return 0;
#endif


#if DEBUG
	if(FromStateID >= (int)StateID2CoordTable.size() 
		|| ToStateID >= (int)StateID2CoordTable.size())
	{
		ROS_ERROR("ERROR in EnvNAVXYTHETACARTLAT... function: stateID illegal");
    throw new SBPL_Exception();
	}
#endif

	//get X, Y for the state
	EnvNAVXYTHETACARTLATHashEntry_t* FromHashEntry = StateID2CoordTable[FromStateID];
	EnvNAVXYTHETACARTLATHashEntry_t* ToHashEntry = StateID2CoordTable[ToStateID];
	
	//TODO - check if one of the gridsearches already computed and then use it.
	

	return (int)(NAVXYTHETACARTLAT_COSTMULT_MTOMM*EuclideanDistance_m(FromHashEntry->X, FromHashEntry->Y, ToHashEntry->X, ToHashEntry->Y)/EnvNAVXYTHETACARTLATCfg.nominalvel_mpersecs);	

}


int EnvironmentNAVXYTHETACARTLAT::GetGoalHeuristic(int stateID)
{
#if USE_HEUR==0
	return 0;
#endif

#if DEBUG
	if(stateID >= (int)StateID2CoordTable.size())
	{
		ROS_ERROR("ERROR in EnvNAVXYTHETACARTLAT... function: stateID illegal");
    throw new SBPL_Exception();
	}
#endif

	EnvNAVXYTHETACARTLATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
	int h2D = grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y); //computes distances from start state that is grid2D, so it is EndX_c EndY_c 
	int hEuclid = (int)(NAVXYTHETACARTLAT_COSTMULT_MTOMM*EuclideanDistance_m(HashEntry->X, HashEntry->Y, EnvNAVXYTHETACARTLATCfg.EndX_c, EnvNAVXYTHETACARTLATCfg.EndY_c));
		
	//define this function if it is used in the planner (heuristic backward search would use it)
    return (int)(((double)__max(h2D,hEuclid))/EnvNAVXYTHETACARTLATCfg.nominalvel_mpersecs); 

}


int EnvironmentNAVXYTHETACARTLAT::GetStartHeuristic(int stateID)
{


#if USE_HEUR==0
	return 0;
#endif


#if DEBUG
	if(stateID >= (int)StateID2CoordTable.size())
	{
		ROS_ERROR("ERROR in EnvNAVXYTHETACARTLAT... function: stateID illegal");
    throw new SBPL_Exception();
	}
#endif

	EnvNAVXYTHETACARTLATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
	int h2D = grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y);
	int hEuclid = (int)(NAVXYTHETACARTLAT_COSTMULT_MTOMM*EuclideanDistance_m(EnvNAVXYTHETACARTLATCfg.StartX_c, EnvNAVXYTHETACARTLATCfg.StartY_c, HashEntry->X, HashEntry->Y));
		
	//define this function if it is used in the planner (heuristic backward search would use it)
    return (int)(((double)__max(h2D,hEuclid))/EnvNAVXYTHETACARTLATCfg.nominalvel_mpersecs); 

}

int EnvironmentNAVXYTHETACARTLAT::SizeofCreatedEnv()
{
	return (int)StateID2CoordTable.size();
	
}
//------------------------------------------------------------------------------
