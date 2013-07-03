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
#ifndef __ENVIRONMENT_NAVXYTHETACARTLAT_H_
#define __ENVIRONMENT_NAVXYTHETACARTLAT_H_


//eight-connected grid
#define NAVXYTHETACARTLAT_DXYWIDTH 8

#define ENVNAVXYTHETACARTLAT_DEFAULTOBSTHRESH 254	//see explanation of the value below

#define SBPL_XYTHETACARTLAT_MAXSTATESFORLOOKUP 100000000 //maximum number of states for storing them into lookup (as opposed to hash)

//definition of theta orientations
//0 - is aligned with X-axis in the positive direction (1,0 in polar coordinates)
//theta increases as we go counterclockwise
//number of theta values - should be power of 2
#define NAVXYTHETACARTLAT_THETADIRS 16

// TODO - ContTheta2Disc
#define CART_THETADIRS 5 //Should be symmetric
#define MAX_CART_ANGLE M_PI/4.0

//number of actions per x,y,theta state
#define NAVXYTHETACARTLAT_DEFAULT_ACTIONWIDTH 5 //decrease, increase, same angle while moving plus decrease, increase angle while standing.

#define NAVXYTHETACARTLAT_COSTMULT_MTOMM 1000

typedef struct{
	double x;
	double y;
} EnvNAVXYTHETACARTLAT2Dpt_t;

typedef struct{
	double x;
	double y;
	double theta;
  double cartangle;
} EnvNAVXYTHETACARTLAT3Dpt_t;


typedef struct EnvNAVXYTHETACARTLAT3DCELL{
	int x;
	int y;
	int theta;
	int iteration;
  int cartangle;
public:
	bool operator == (EnvNAVXYTHETACARTLAT3DCELL cell) {return (x==cell.x && y==cell.y && theta==cell.theta && cartangle == cell.cartangle);}
} EnvNAVXYTHETACARTLAT3Dcell_t;


typedef struct
{
	char starttheta;
	char dX;
	char dY;
	char endtheta;
  char startcartangle;
  char endcartangle;
	unsigned int cost; 
	vector<sbpl_2Dcell_t> intersectingcellsV;
	//start at 0,0,starttheta and end at endcell in continuous domain with half-bin less to account for 0,0 start
	vector<EnvNAVXYTHETACARTLAT3Dpt_t> intermptV;
	//start at 0,0,starttheta and end at endcell in discrete domain
	vector<EnvNAVXYTHETACARTLAT3Dcell_t> interm3DcellsV;
} EnvNAVXYTHETACARTLATAction_t;


typedef struct 
{
	int stateID;
	int X;
	int Y;
	char Theta;
  char CartAngle;
	int iteration;
} EnvNAVXYTHETACARTLATHashEntry_t;


typedef struct
{
	int motprimID;
	unsigned char starttheta_c;
	int additionalactioncostmult;
	EnvNAVXYTHETACARTLAT3Dcell_t endcell;
	//intermptV start at 0,0,starttheta and end at endcell in continuous domain with half-bin less to account for 0,0 start
	vector<EnvNAVXYTHETACARTLAT3Dpt_t> intermptV; 
}SBPL_xythetacart_mprimitive;


//variables that dynamically change (e.g., array of states, ...)
typedef struct
{

	int startstateid;
	int goalstateid;

	bool bInitialized;

	//any additional variables


}EnvironmentNAVXYTHETACARTLAT_t;

//configuration parameters
typedef struct ENV_NAVXYTHETACARTLAT_CONFIG
{
	int EnvWidth_c;
	int EnvHeight_c;
	int StartX_c;
	int StartY_c;
	int StartTheta;
  int StartCartAngle;
	int EndX_c;
	int EndY_c;
	int EndTheta;
  int EndCartAngle;
	unsigned char** Grid2D;

	//the value at which and above which cells are obstacles in the maps sent from outside
	//the default is defined above
	unsigned char obsthresh; 

	//the value at which and above which until obsthresh (not including it) cells have the nearest obstacle at distance smaller than or equal to 
	//the inner circle of the robot. In other words, the robot is definitely colliding with the obstacle, independently of its orientation
	//if no such cost is known, then it should be set to obsthresh (if center of the robot collides with obstacle, then the whole robot collides with it
	//independently of its rotation)
	unsigned char cost_inscribed_thresh; 

	//the value at which and above which until cost_inscribed_thresh (not including it) cells 
	//**may** have a nearest osbtacle within the distance that is in between the robot inner circle and the robot outer circle
	//any cost below this value means that the robot will NOT collide with any obstacle, independently of its orientation
	//if no such cost is known, then it should be set to 0 or -1 (then no cell cost will lower than it, and therefore the robot's footprint will always be checked)
	int cost_possibly_circumscribed_thresh; //it has to be integer, because -1 means that it is not provided.

	double nominalvel_mpersecs;
	double timetoturn45degsinplace_secs;
	double cellsize_m;

	int dXY[NAVXYTHETACARTLAT_DXYWIDTH][2];

	EnvNAVXYTHETACARTLATAction_t** ActionsV; //array of actions, ActionsV[i][j] - jth action for sourcetheta = i
	vector<EnvNAVXYTHETACARTLATAction_t*>* PredActionsV; //PredActionsV[i] - vector of pointers to the actions that result in a state with theta = i

	int actionwidth; //number of motion primitives
	vector<SBPL_xythetacart_mprimitive> mprimV;

	vector<sbpl_2Dpt_t> FootprintPolygon;
	vector<sbpl_2Dpt_t> CartPolygon;
	sbpl_2Dpt_t CartOffset;
  sbpl_2Dpt_t CartCenterOffset;
} EnvNAVXYTHETACARTLATConfig_t;



class SBPL2DGridSearch;

/** \brief 3D (x,y,theta) planning using lattice-based graph problem. For general structure see comments on parent class DiscreteSpaceInformation
For info on lattice-based planning used here, you can check out the paper: 
Maxim Likhachev and Dave Ferguson, " Planning Long Dynamically-Feasible Maneuvers for Autonomous Vehicles", IJRR'09
*/
class EnvironmentNAVXYTHETACARTLATTICE : public DiscreteSpaceInformation
{

public:

	EnvironmentNAVXYTHETACARTLATTICE();

  /**
   * \brief initialization of environment from file. See .cfg files for examples
   *  it also takes the perimeter of the robot with respect to some reference point centered at x=0,y=0 and orientation = 0 (along x axis). 
   *  The perimeter is defined in meters as a sequence of vertices of a polygon defining the perimeter. If vector is of zero size, then
   *  robot is assumed to be point robot (you may want to inflate all obstacles by its actual radius)
   *  Motion primitives file defines the motion primitives available to the robot
  */
  // TODO - add cart perimeters and offset
	bool InitializeEnv(const char* sEnvFile, const vector<sbpl_2Dpt_t>& perimeterptsV, const vector<sbpl_2Dpt_t>& cartperimeterptsV, sbpl_2Dpt_t &cart_offset, const char* sMotPrimFile);	
  /**
	 * \brief see comments on the same function in the parent class
  */
	bool InitializeEnv(const char* sEnvFile);
  /**
	 * \brief way to set up various parameters. For a list of parameters, see the body of the function - it is pretty straightforward
  */
	virtual bool SetEnvParameter(const char* parameter, int value);
  /**
	 * \brief returns the value of specific parameter - see function body for the list of parameters
  */
	virtual int GetEnvParameter(const char* parameter);
  /**
	 * \brief see comments on the same function in the parent class
   */
	bool InitializeMDPCfg(MDPConfig *MDPCfg);
  /**
	 * \brief see comments on the same function in the parent class
   */
	virtual int  GetFromToHeuristic(int FromStateID, int ToStateID) = 0;
  /**
	 * \brief see comments on the same function in the parent class
   */
	virtual int  GetGoalHeuristic(int stateID) = 0;
  /**
	 * \brief see comments on the same function in the parent class
  */
	virtual int  GetStartHeuristic(int stateID) = 0;
  /**
	 * \brief see comments on the same function in the parent class
  */
	virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state) = 0;
  /**
	 * \brief see comments on the same function in the parent class
  */
	virtual void SetAllPreds(CMDPSTATE* state);
  /**
	 * \brief see comments on the same function in the parent class
  */
	virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV);
  /**
	 * \brief see comments on the same function in the parent class
  */
	virtual void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV) = 0;

  /**
	 * \brief see comments on the same function in the parent class
  */
	virtual void EnsureHeuristicsUpdated(bool bGoalHeuristics); 

  /**
	 * \brief see comments on the same function in the parent class
  */
	void PrintEnv_Config(FILE* fOut);

  /**
	 * \brief initialize environment. Gridworld is defined as matrix A of size width by height. 
   * So, internally, it is accessed as A[x][y] with x ranging from 0 to width-1 and and y from 0 to height-1
   * Each element in A[x][y] is unsigned char. A[x][y] = 0 corresponds to fully traversable and cost is just Euclidean distance
   * The cost of transition between two neighboring cells is EuclideanDistance*(max(A[sourcex][sourcey],A[targetx][targety])+1)
   * f A[x][y] >= obsthresh, then in the above equation it is assumed to be infinite.
   * The cost also incorporates the length of a motion primitive and its cost_multiplier (see getcost function)
   * mapdata is a pointer to the values of A. If it is null, then A is initialized to all zeros. Mapping is: A[x][y] = mapdata[x+y*width]
   * start/goal are given by startx, starty, starttheta, goalx,goaly, goaltheta in meters/radians. 
   * If they are not known yet, just set them to 0. Later setgoal/setstart can be executed
   * finally obsthresh defined obstacle threshold, as mentioned above
   * goaltolerances are currently ignored
   * for explanation of perimeter, see comments for InitializeEnv function that reads all from file
   * cellsize is discretization in meters
   * nominalvel_mpersecs is assumed velocity of vehicle while moving forward in m/sec
   * timetoturn45degsinplace_secs is rotational velocity in secs/45 degrees turn
  */
    bool InitializeEnv(int width, int height,
		       /** if mapdata is NULL the grid is initialized to all freespace */
                       const unsigned char* mapdata,
                       double startx, double starty, double starttheta, double startcartangle,
                       double goalx, double goaly, double goaltheta, double goalcartangle,
                       double goaltol_x, double goaltol_y, double goaltol_theta, double goaltol_cartangle,
                       const vector<sbpl_2Dpt_t> & perimeterptsV,
                       const sbpl_2Dpt_t & cart_offset,
                       const vector<sbpl_2Dpt_t> & cartptsV,
                       double cellsize_m, double nominalvel_mpersecs, double timetoturn45degsinplace_secs, 
                       unsigned char obsthresh, const char* sMotPrimFile);
  /**
	 * \brief update the traversability of a cell<x,y>
  */
    bool UpdateCost(int x, int y, unsigned char newcost);
  /**
   * \brief this function fill in Predecessor/Successor states of edges whose costs changed
   * It takes in an array of cells whose traversability changed, and returns (in vector preds_of_changededgesIDV) 
   * the IDs of all states that have outgoing edges that go through the changed cells
  */
	virtual void GetPredsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *preds_of_changededgesIDV) = 0;
  /**
	 * \brief same as GetPredsofChangedEdges, but returns successor states. Both functions need to be present for incremental search
  */
	virtual void GetSuccsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *succs_of_changededgesIDV) = 0;

  /**
	 * returns true if cell is untraversable
  */
	bool IsObstacle(int x, int y);
  /**
   * \brief returns false if robot intersects obstacles or lies outside of the map. Note this is pretty expensive operation since it computes the footprint
   * of the robot based on its x,y,theta
  */
	bool IsValidConfiguration(int X, int Y, int Theta, int CartAngle);

  /**
	 * \brief returns environment parameters. Useful for creating a copy environment
  */
  // TODO - add cart parameters
  void GetEnvParms(int *size_x, int *size_y, 
                   double* startx, double* starty, double*starttheta, double*startcartangle, 
                   double* goalx, double* goaly, double* goaltheta, double*goalcartangle,
                   double* cellsize_m, double* nominalvel_mpersecs, 
                   double* timetoturn45degsinplace_secs, unsigned char* obsthresh,
                   vector<SBPL_xythetacart_mprimitive>* mprimitiveV);

  /**
	 * \brief get internal configuration data structure
  */
	const EnvNAVXYTHETACARTLATConfig_t* GetEnvNavConfig();


  virtual ~EnvironmentNAVXYTHETACARTLATTICE();
  /**
   * \brief prints time statistics
  */
  void PrintTimeStat(FILE* fOut);
  /** 
   * \brief returns the cost corresponding to the cell <x,y>
  */
	unsigned char GetMapCost(int x, int y);

  /**
	 * \brief returns true if cell is within map
  */
	bool IsWithinMapCell(int X, int Y);
  

  /** \brief prints environment variables for debugging
    */
  virtual void PrintVars(){};

  bool initialized_;

 protected:

  virtual int GetActionCost(int SourceX, int SourceY, int SourceTheta, int SourceCartAngle, EnvNAVXYTHETACARTLATAction_t* action);


	//member data
	EnvNAVXYTHETACARTLATConfig_t EnvNAVXYTHETACARTLATCfg;
	EnvironmentNAVXYTHETACARTLAT_t EnvNAVXYTHETACARTLAT;
	vector<EnvNAVXYTHETACARTLAT3Dcell_t> affectedsuccstatesV; //arrays of states whose outgoing actions cross cell 0,0
	vector<EnvNAVXYTHETACARTLAT3Dcell_t> affectedpredstatesV; //arrays of states whose incoming actions cross cell 0,0
	int iteration;

	//2D search for heuristic computations
	bool bNeedtoRecomputeStartHeuristics; //set whenever grid2Dsearchfromstart needs to be re-executed
	bool bNeedtoRecomputeGoalHeuristics; //set whenever grid2Dsearchfromgoal needs to be re-executed
	SBPL2DGridSearch* grid2Dsearchfromstart; //computes h-values that estimate distances from start x,y to all cells
	SBPL2DGridSearch* grid2Dsearchfromgoal;  //computes h-values that estimate distances to goal x,y from all cells

 	virtual void ReadConfiguration(FILE* fCfg);

	void InitializeEnvConfig(vector<SBPL_xythetacart_mprimitive>* motionprimitiveV);


	bool CheckQuant(FILE* fOut);

	void SetConfiguration(int width, int height,
			      /** if mapdata is NULL the grid is initialized to all freespace */
                        const unsigned char* mapdata,
                        int startx, int starty, int starttheta, int startcartangle,
                        int goalx, int goaly, int goaltheta, int goalcartangle,
                        double cellsize_m, double nominalvel_mpersecs, 
                        double timetoturn45degsinplace_secs, 
                        const vector<sbpl_2Dpt_t> & robot_perimeterV,
                        const vector<sbpl_2Dpt_t> & cart_perimeterV, 
                        const sbpl_2Dpt_t &cart_offset);
	
	bool InitGeneral( vector<SBPL_xythetacart_mprimitive>* motionprimitiveV);
	void PrecomputeActionswithBaseMotionPrimitive(vector<SBPL_xythetacart_mprimitive>* motionprimitiveV);
	void PrecomputeActionswithCompleteMotionPrimitive(vector<SBPL_xythetacart_mprimitive>* motionprimitiveV);
	void PrecomputeActions();

	void CreateStartandGoalStates();

	virtual void InitializeEnvironment() = 0;

	void ComputeHeuristicValues();

	bool IsValidCell(int X, int Y);

	void CalculateFootprintForPose(EnvNAVXYTHETACARTLAT3Dpt_t pose, vector<sbpl_2Dcell_t>* footprint);
	void RemoveSourceFootprint(EnvNAVXYTHETACARTLAT3Dpt_t sourcepose, vector<sbpl_2Dcell_t>* footprint);

	virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<EnvNAVXYTHETACARTLATAction_t*>* actionindV=NULL) = 0;

	double EuclideanDistance_m(int X1, int Y1, int X2, int Y2);

	void ComputeReplanningData();
	void ComputeReplanningDataforAction(EnvNAVXYTHETACARTLATAction_t* action);

	bool ReadMotionPrimitives(FILE* fMotPrims);
	bool ReadinMotionPrimitive(SBPL_xythetacart_mprimitive* pMotPrim, FILE* fIn);
	bool ReadinCell(EnvNAVXYTHETACARTLAT3Dcell_t* cell, FILE* fIn);
	bool ReadinPose(EnvNAVXYTHETACARTLAT3Dpt_t* pose, FILE* fIn);

	void PrintHeuristicValues();
  void PrintFootprint();
  EnvNAVXYTHETACARTLAT3Dpt_t getCartCenter(EnvNAVXYTHETACARTLAT3Dpt_t pose, sbpl_2Dpt_t cart_center_offset);

};


class EnvironmentNAVXYTHETACARTLAT : public EnvironmentNAVXYTHETACARTLATTICE
{

 public:
  EnvironmentNAVXYTHETACARTLAT()
  {
	HashTableSize = 0;
	Coord2StateIDHashTable = NULL;
	Coord2StateIDHashTable_lookup = NULL; 
  };

  ~EnvironmentNAVXYTHETACARTLAT();

  /** \brief sets start in meters/radians
    */
  int SetStart(double x, double y, double theta, double cartangle);
  /** \brief sets goal in meters/radians
  */
  int SetGoal(double x, double y, double theta, double cartangle);
  /** \brief sets goal tolerance. (Note goal tolerance is ignored currently)
    */
  void SetGoalTolerance(double tol_x, double tol_y, double tol_theta, double tol_cartangle) { /**< not used yet */ }
  /** \brief returns state coordinates of state with ID=stateID
    */
  void GetCoordFromState(int stateID, int& x, int& y, int& theta, int& cartangle) const;
  /** \brief returns stateID for a state with coords x,y,theta
    */
  int GetStateFromCoord(int x, int y, int theta, int cartangle);

  /** \brief converts a path given by stateIDs into a sequence of coordinates. Note that since motion primitives are short actions represented as a sequence of points,
  the path returned by this function contains much more points than the number of points in the input path. The returned coordinates are in meters,meters,radians
  */
  bool ConvertStateIDPathintoXYThetaPath(vector<int>* stateIDPath, vector<EnvNAVXYTHETACARTLAT3Dpt_t>* xythetaPath); 
  /** \brief prints state info (coordinates) into file
    */
  void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL);
  /** \brief returns all predecessors states and corresponding costs of actions
    */
  virtual void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV);
  /** \brief returns all successors states, costs of corresponding actions and pointers to corresponding actions, each of which is a motion primitive
  if actionindV is NULL, then pointers to actions are not returned
  */
  virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<EnvNAVXYTHETACARTLATAction_t*>* actionindV=NULL);

  /** \brief this function fill in Predecessor/Successor states of edges whose costs changed
  It takes in an array of cells whose traversability changed, and returns (in vector preds_of_changededgesIDV) 
  the IDs of all states that have outgoing edges that go through the changed cells
  */
  void GetPredsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *preds_of_changededgesIDV);
  /** \brief same as GetPredsofChangedEdges, but returns successor states. Both functions need to be present for incremental search
    */
  void GetSuccsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *succs_of_changededgesIDV);

  /** \brief see comments on the same function in the parent class
    */
  virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state);

  /** \brief see comments on the same function in the parent class
    */
  virtual int  GetFromToHeuristic(int FromStateID, int ToStateID);
  /** \brief see comments on the same function in the parent class
    */
  virtual int  GetGoalHeuristic(int stateID);
  /** \brief see comments on the same function in the parent class
    */
  virtual int  GetStartHeuristic(int stateID);

  /** \brief see comments on the same function in the parent class
    */
  virtual int	 SizeofCreatedEnv();
  /** \brief see comments on the same function in the parent class
    */
  virtual void PrintVars(){};

 protected:

  //hash table of size x_size*y_size. Maps from coords to stateId	
  int HashTableSize;
  vector<EnvNAVXYTHETACARTLATHashEntry_t*>* Coord2StateIDHashTable;
  //vector that maps from stateID to coords	
  vector<EnvNAVXYTHETACARTLATHashEntry_t*> StateID2CoordTable;
  
  EnvNAVXYTHETACARTLATHashEntry_t** Coord2StateIDHashTable_lookup; 

  unsigned int GETHASHBIN(unsigned int X, unsigned int Y, unsigned int Theta, unsigned int CartAngle);

  EnvNAVXYTHETACARTLATHashEntry_t* GetHashEntry_hash(int X, int Y, int Theta, int CartAngle);
  EnvNAVXYTHETACARTLATHashEntry_t* CreateNewHashEntry_hash(int X, int Y, int Theta, int CartAngle);
  EnvNAVXYTHETACARTLATHashEntry_t* GetHashEntry_lookup(int X, int Y, int Theta, int CartAngle);
  EnvNAVXYTHETACARTLATHashEntry_t* CreateNewHashEntry_lookup(int X, int Y, int Theta, int CartAngle);

  //pointers to functions
  EnvNAVXYTHETACARTLATHashEntry_t* (EnvironmentNAVXYTHETACARTLAT::*GetHashEntry)(int X, int Y, int Theta, int CartAngle);
  EnvNAVXYTHETACARTLATHashEntry_t* (EnvironmentNAVXYTHETACARTLAT::*CreateNewHashEntry)(int X, int Y, int Theta, int CartAngle);


  virtual void InitializeEnvironment();

  void PrintHashTableHist(FILE* fOut);


};


#endif

