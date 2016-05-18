// ----------------------------------------------------------------------------
//
//
// OpenSteer -- Steering Behaviors for Autonomous Characters
//
// Copyright (c) 2002-2003, Sony Computer Entertainment America
// Original author: Craig Reynolds <craig_reynolds@playstation.sony.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
//
// ----------------------------------------------------------------------------
//
// 10-29-01 cwr: created
//
//
// ----------------------------------------------------------------------------
//
// An autonomous client for a navmesh (attaches to fxFlexBodies using NavPaths.)
// Follows paths, avoids collisions (with obstacles?) and other pedestrians
//
// 03-31-12  BrokeAss Games
//

#ifndef	_NAV_CLIENT_H_
#define _NAV_CLIENT_H_


#include <iomanip>
#include <sstream>
#include "../include/Pathway.h"
#include "../include/SimpleVehicle.h"
#include "../include/OpenSteerDemo.h"
#include "../include/Proximity.h"

//Ecstasy Motion
#include "console/consoleTypes.h"
#ifndef _MPOINT3_H_
#include "math/mPoint3.h"
#endif

#include "navigation/navMesh.h"
#include "navigation/navPath.h"

//#include "T3D/physicsBAG/fxFlexBody.h"
//#include "T3D/physicsBAG/nxPhysManager.h"
//#include "T3D/physicsBAG/physManager.h"
//class nxPhysManager;
//class fxFlexBody;

//#define NUM_NAV_CLIENTS  6  //FIX
//End Ecstasy Motion

using namespace OpenSteer;


// ----------------------------------------------------------------------------


typedef AbstractProximityDatabase<AbstractVehicle*> ProximityDatabase;
typedef AbstractTokenForProximityDatabase<AbstractVehicle*> ProximityToken;


// ----------------------------------------------------------------------------


// creates a path for the PlugIn
PolylinePathway* getNavPath (void);
//SphericalObstacle gObstacle1;
//SphericalObstacle gObstacle2;
//ObstacleGroup gObstacles;


// ----------------------------------------------------------------------------


class NavClient : public SimpleVehicle
{
public:

    // type for a group of Pedestrians
    typedef std::vector<NavClient*> groupType;
	//Nav::NavPath *mNavPath;
	int id;

	//F32 mMaxSpeed;//Irrelevant?
	//F32 mMaxForce;
	//F32 mRadius;
	
	//NOPE!!! Problems with trying to include physx headers, windows.h error, NOMINMAX etc.
	//fxFlexBody *mFlexBody;//give us a way to point at our owning flexbody's other children, such as recast navpath.

	Vec3 mMoveTarget;//redundant but copy this here from flexbody

	//Vehicle *mTarget;

	//float separationRadius =  5.0f;
	//float separationAngle  = -0.707f;
	//float separationWeight =  10.0f;//12.0f;

	//float alignmentRadius = 7.5f;
	//float alignmentAngle  = 0.7f;
	//float alignmentWeight = 8.0f;

	//float cohesionRadius = 19.0f;
	//float cohesionAngle  = -0.15f;
	//float cohesionWeight = 9.0f;

	bool isThinking;

	float mSeekTargetWeight;
	float mAvoidTargetWeight;
	float mSeekNeighborWeight;
	float mAvoidNeighborWeight;
	float mAvoidNavMeshEdgeWeight;
	float mWanderWeight;

	float mWanderChance;
	//float - other chances?
	float mWallRange;

    // a pointer to this boid's interface object for the proximity database
    ProximityToken* proximityToken;

    // allocate one and share amoung instances just to save memory usage
    // (change to per-instance allocation to be more MP-safe)
    static AVGroup neighbors;

    // path to be followed by this pedestrian
    // XXX Ideally this should be a generic Pathway, but we use the
    // XXX getTotalPathLength and radius methods (currently defined only
    // XXX on PolylinePathway) to set random initial positions.  Could
    // XXX there be a "random position inside path" method on Pathway?
    PolylinePathway* path;
	
	//MegaMotion
	NavPath *mDetourNavPath;//see if we can dispense with above polyline pathway in favor of all recast/detour.
	Vec3 steerForNavMesh(F32 wallRange);

    // direction for path following (upstream or downstream)
    int pathDirection;
    // constructor
    NavClient (ProximityDatabase& pd);

    // destructor
    virtual ~NavClient ();

    // reset all instance state
    void reset (void);

    // per frame simulation update
    void update (const float currentTime, const float elapsedTime);

    // compute combined steering force: move forward, avoid obstacles
    // or neighbors if needed, otherwise follow the path and wander
    Vec3 determineCombinedSteering (const float elapsedTime);


    // draw this pedestrian into scene
    void draw (void);

    // called when steerToFollowPath decides steering is required
    void annotatePathFollowing (const Vec3& future,
                                const Vec3& onPath,
                                const Vec3& target,
                                const float outside);

    // called when steerToAvoidCloseNeighbors decides steering is required
    // (parameter names commented out to prevent compiler warning from "-W")
    void annotateAvoidCloseNeighbor (const AbstractVehicle& other,
                                     const float /*additionalDistance*/);

    // (parameter names commented out to prevent compiler warning from "-W")
    void annotateAvoidNeighbor (const AbstractVehicle& threat,
                                const float /*steer*/,
                                const Vec3& ourFuture,
                                const Vec3& threatFuture);

    // xxx perhaps this should be a call to a general purpose annotation for
    // xxx "local xxx axis aligned box in XZ plane" -- same code in in
    // xxx CaptureTheFlag.cpp
    void annotateAvoidObstacle (const float minDistanceToCollision);

    // switch to new proximity database -- just for demo purposes
    void newPD (ProximityDatabase& pd);

	void setThinking(bool b) { isThinking=b; };

	void setMoveTarget(Vec3 target) { mMoveTarget=target; }

	//HERE: we need a whole set of functions and variables for defining which steering behaviors are "turned on" and 
	//setting their local arguments (radius, angle, vec target, etc.), expose all these to script.
};



// ----------------------------------------------------------------------------
// create path for PlugIn 
//
//
//        | gap |
//
//        f      b
//        |\    /\        -
//        | \  /  \       ^
//        |  \/    \      |
//        |  /\     \     |
//        | /  \     c   top
//        |/    \g  /     |
//        /        /      |
//       /|       /       V      z     y=0
//      / |______/        -      ^
//     /  e      d               |
//   a/                          |
//    |<---out-->|               o----> x
//



class NavClientPlugIn : public PlugIn
{
public:

    const AVGroup& allVehicles (void) {return (const AVGroup&) crowd;}

    // crowd: a group (STL vector) of all Pedestrians
    NavClient::groupType crowd;
    typedef NavClient::groupType::const_iterator iterator;

    Vec3 gridCenter;

    // pointer to database used to accelerate proximity queries
    ProximityDatabase* pd;

    // keep track of current flock size
    int population;

    // which of the various proximity databases is currently in use
    int cyclePD;

    const char* name (void) {return "NavClients";}
	 //nxPhysManager *mPM;


    float selectionOrderSortKey (void) {return 0.02f;}

    virtual ~NavClientPlugIn() {}// be more "nice" to avoid a compiler warning

    void open (void);

    void update (const float currentTime, const float elapsedTime);

    void redraw (const float currentTime, const float elapsedTime);

    void serialNumberAnnotationUtility (const AbstractVehicle& selected,
                                        const AbstractVehicle& nearMouse);

    void drawPathAndObstacles (void);

    void close (void);

    void reset (void);

    void handleFunctionKeys (int keyNumber);

    void printMiniHelpForFunctionKeys (void);

    void addNavClientToCrowd (void);

    void removeNavClientFromCrowd (void);

    // for purposes of demonstration, allow cycling through various
    // types of proximity databases.  this routine is called when the
    // OpenSteerDemo user pushes a function key.
    void nextPD (void);


	//MegaMotion
	void createVehicle(const Vec3 pos,float rot);//create and add to crowd.
};


//NavClientPlugIn gNavClientPlugIn;

#endif