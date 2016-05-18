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


#include "T3D/OpenSteer/Plugins/navClient.h"

//#include "T3D/physicsBAG/fxFlexBody.h"// can't do this, get windows.h NOMINMAX compile error, ???



using namespace OpenSteer;


Vec3 gEndpoint0;
Vec3 gEndpoint1;

PolylinePathway* gNavPath = NULL;

bool gUseDirectedPathFollowing = true;//false;//

// this was added for debugging tool, but I might as well leave it in
bool gWanderSwitch = false;//true;


NavClient::NavClient (ProximityDatabase& pd)
{
	// allocate a token for this boid in the proximity database
	proximityToken = NULL;
	newPD (pd);
	//mNavPath = NULL;

	//mMaxSpeed = 9.0;//irrelevant?
	//mMaxForce = 4.0;//irrelevant?
	//mRadius = 0.8;//irrelevant?

	// reset Pedestrian state
	reset ();
}

// destructor
NavClient::~NavClient ()
{
	// delete this boid's token in the proximity database
	delete proximityToken;
}

// reset all instance state
void NavClient::reset (void)
{
	// reset the vehicle 
	SimpleVehicle::reset ();

	// max speed and max steering force (maneuverability) 
	//setMaxSpeed (mMaxSpeed);//irrelevant?
	//setMaxForce (mMaxForce);//irrelevant?

	// initially stopped
	setSpeed (0);

	// size of bounding sphere, for obstacle avoidance, etc.
	//setRadius (mRadius);//irrelevant? // width = 0.7, add 0.3 margin, take half


	isThinking = false;

	mMoveTarget = Vec3::zero;

	
	mSeekTargetWeight = 1.0;
	mAvoidTargetWeight = 3.0;
	mSeekNeighborWeight = 1.0;//1.0;//TEMP, testing
	mAvoidNeighborWeight = 6.0;
	mAvoidNavMeshEdgeWeight = 1.0;
	mWanderWeight = 1.0;
	mWanderChance = 0.1;
	mWallRange = 3.0;

	//more to follow...
	mDetourNavPath = NULL;

	// set the path for this Pedestrian to follow... this is where
	// we need to get involved, this needs to be set on the fly as
	// the actor's navpath changes.

	//if (1) 
	//{
		////////////////////////////////////////////////////////
		/////// ECSTASY MOTION  -- Actually do nothing here, yet.  Set up path on demand.
		//path = getNavPath ();
		//const float d = 0;//path->getTotalPathLength() * frandom01();
		//const float r = 0;//path->radius;//(no need to randomize off my own personal path.)
		//const Vec3 randomOffset = randomVectorOnUnitRadiusXZDisk () * r;

	setPosition (Vec3(0,0,0));//(path->mapPathDistanceToPoint (d) + randomOffset);//HERE: get position from flexbody?
	path = NULL;
	pathDirection = 1;

		//...
		////////////////////////////////////////////////////////
	//} else {
		////////////////////////////////////////////////////////
		/////// ORIGINAL
		//// set initial position
		//// (random point on path + random horizontal offset)

		//path = getNavPath ();
		//const float d = path->getTotalPathLength() * frandom01();
		//const float r = path->radius;
		//const Vec3 randomOffset = randomVectorOnUnitRadiusXZDisk () * r;
		//setPosition (path->mapPathDistanceToPoint (d) + randomOffset);

		//// randomize 2D heading
		//randomizeHeadingOnXZPlane ();

		//// pick a random direction for path following (upstream or downstream)
		//pathDirection = (frandom01() > 0.5) ? -1 : +1;

		//// trail parameters: 3 seconds with 60 points along the trail
		//setTrailParameters (3, 60);
		////////////////////////////////////////////////////////
	//}

	// notify proximity database that our position has changed
	proximityToken->updateForNewPosition (position());
}

// per frame simulation update
void NavClient::update (const float currentTime, const float elapsedTime)
{
	//Con::printf("Navclient update! id %d position: %f %f %f",id,position().x,position().y,position().z);
	if (isThinking==false)
		return;

	// apply steering force to our momentum
	applySteeringForce (determineCombinedSteering (elapsedTime),
		elapsedTime);
	

	//HERE: I believe this function, or one like it, is going to be the key to steering away from the edge of the navmesh.
	//This logic should go into another steering function in determineCombinedSteering, parallel to steerForSeek etc 
	
	//mDetourNavPath->mQuery->findDistanceToWall(...);
	
	//But, WAIT, the BETTER WAY to do this would be to define a new friendly function for the NavPath interface, which 
	//calls its own mQuery but doesn't involve us in any of the details of that here. We just want to give it a vector
	//and get back a distance, and maybe a position and normal, here.



	/*
	//OBSOLETE, using nav paths from recast
	// reverse direction when we reach an endpoint
	if ((path!=NULL)&&(gUseDirectedPathFollowing))
	{
		const Vec3 darkRed (0.7f, 0, 0);

		if (Vec3::distance (position(), gEndpoint0) < path->radius)
		{
			pathDirection = +1;
			annotationXZCircle (path->radius, gEndpoint0, darkRed, 20);
		}
		if (Vec3::distance (position(), gEndpoint1) < path->radius)
		{
			pathDirection = -1;
			annotationXZCircle (path->radius, gEndpoint1, darkRed, 20);
		}
	}//OBSOLETE
	*/

	// annotation
	annotationVelocityAcceleration (5, 0);
	recordTrailVertex (currentTime, position());

	// notify proximity database that our position has changed
	proximityToken->updateForNewPosition (position());
	//Con::printf("Navclient %d position: %f %f %f",id,position().x,position().y,position().z);
}


Vec3 NavClient::steerForNavMesh(F32 wallRange)
{
	Point3F pos(position().x,position().y,position().z);//Now, do we have to translate this back to T3D coordinates?
	//NOTE: need to double check between opensteer, recast, and T3D to make sure right/left handed conversions are all solid.
	
	//F32 wallDist = mDetourNavPath->findDistanceToWall(pos,wallRange);
	//if (wallDist<0.0)//No valid navmesh returns -1.0
	//	return Vec3(0.0f,0.0f,0.0f);

	Point3F hitpos = mDetourNavPath->mHitPos;
	Point3F hitnorm = mDetourNavPath->mHitNormal;
	//if (id==11) 
	//	Con::printf("steering for navmesh, id %d distToWall %f hitpos %f %f %f",id,wallDist,hitpos.x,hitpos.y,hitpos.z);

	if (0)//(wallDist < wallRange) //if no wall found, findDistanceToWall returns the search radius you gave it.
	{
		//Point3F diff = hitpos - pos;
		Point3F diff = pos - hitpos;//This is from hitpos to my position, ie the other way from the wall
		//Here, subtract my position from hitpos, or opposite, and return it (vector away from hitpos). If necessary scale it up.
		//Con::printf("agent is steering away from the navmesh wall: %f %f %f",diff.x,diff.y,diff.z);
		diff.normalize();
		diff *= (wallRange-diff.len())/wallRange;//There, scale it so the closer we are, the bigger the repulsion force.
		diff *= 2.0;//Now that we have it going the right way, let's make it double the regular movement scale to make sure.
		return Vec3(diff.x,diff.y,diff.z);
	} else {
		return Vec3(0.0f,0.0f,0.0f);
	}
}

// compute combined steering force: move forward, avoid obstacles
// or neighbors if needed, otherwise follow the path and wander
Vec3 NavClient::determineCombinedSteering (const float elapsedTime)
{
	// move forward
	Vec3 steeringForce = forward();


	// probability that a lower priority behavior will be given a
	// chance to "drive" even if a higher priority behavior might
	// otherwise be triggered.

	// determine if obstacle avoidance is required
	//Vec3 obstacleAvoidance;
	//obstacleAvoidance = Vec3::zero;
	//if (leakThrough < frandom01())
	//{
	//    const float oTime = 6; // minTimeToCollision = 6 seconds
	//    obstacleAvoidance = steerToAvoidObstacles (oTime, gObstacles);
	//}

	// if obstacle avoidance is needed, do it
	//if (obstacleAvoidance != Vec3::zero)
	//{
	//	steeringForce += obstacleAvoidance;
	//}
	//else
	//{
	// otherwise consider avoiding collisions with others
	Vec3 neighborAvoid,navmeshEdgeAvoid,neighborSeek,targetSeek,wanderSeek;
	const float caLeadTime = 3;

	// find all neighbors within maxRadius using proximity database
	// (radius is largest distance between vehicles traveling head-on
	// where a collision is possible within caLeadTime seconds.)
	//const float maxRadius = caLeadTime * maxSpeed() * 2;
	//const float maxRadius = 30.0;//Need a much larger radius if you're looking for cohesion thatn


	float maxRadius = caLeadTime * maxSpeed() * 2;
	neighbors.clear();//if you're looking for collision avoidance.
	proximityToken->findNeighbors (position(), maxRadius, neighbors);
	if (neighbors.size()>0)
	{
		neighborAvoid = steerToAvoidNeighbors (caLeadTime, neighbors);
		if (neighborAvoid != Vec3::zero)
		{
			neighborAvoid *= mAvoidNeighborWeight;
			steeringForce += neighborAvoid;
		}
		//Con::printf("Avoiding neighbors: %d avoidForce %f %f %f",neighbors.size(),
		//			neighborAvoid.x,neighborAvoid.y,neighborAvoid.z);
	}
	
	if (frandom01() < mWanderChance)
	{
		wanderSeek = steerForWander (elapsedTime);
		if (wanderSeek != Vec3::zero)
		{
			wanderSeek *= mWanderWeight;
			steeringForce += wanderSeek;
			//Con::printf("%d adding wander %f %f %f",id,wanderSeek.x,wanderSeek.y,wanderSeek.z);
		}		
	}

	//Vec3 target(80,242,-150);//TEMP, get this from navpath

	//Now, steerForSeek is a little rude, it just returns the direct difference between our position and our 
	//target position. Let's not change it in SteerLibrary.h since that is core code likely to be updated, but
	//instead let's fix it here and scale as appropriate.
	//mMoveTarget = target;
	if (mMoveTarget != Vec3::zero)//FIX, this makes it impossible to have the origin as your actual target.
	{
		targetSeek = steerForSeek (mMoveTarget);
		if (targetSeek != Vec3::zero)
		{
			targetSeek.normalize();
			targetSeek *= mSeekTargetWeight;//starting with 1.0 to see what happens, we'll probably want higher.
			steeringForce += targetSeek;
		}
	}
	//Con::printf("seeking target: %f %f %f",mMoveTarget.x,mMoveTarget.y,mMoveTarget.z);
	//JUST TESTING... would like to experiment with other opensteer behaviors here.
	

	//neighborSeek = steerForCohesion(40.0,90.0,neighbors);
	//if (neighborSeek != Vec3::zero)
	//{
	//	steeringForce += neighborSeek;
	//	Con::printf("adding neighbor seek! %f %f %f",neighborSeek.x,neighborSeek.y,neighborSeek.z);
	//}
	
	maxRadius = 100.0;
	neighbors.clear();//if you're looking for collision avoidance.
	proximityToken->findNeighbors (position(), maxRadius, neighbors);
	if (neighbors.size()>0)
	{
		neighborSeek = steerForCohesion(maxRadius,0.0,neighbors);
		neighborSeek *= mSeekNeighborWeight;
		steeringForce += neighborSeek;
	//	Con::printf("adding neighbor seek! %f %f %f",neighborSeek.x,neighborSeek.y,neighborSeek.z);
	}


	navmeshEdgeAvoid = steerForNavMesh(5.0);//wall range, ignore past this distance.
	if (navmeshEdgeAvoid != Vec3::zero)
	{
		navmeshEdgeAvoid *= mAvoidNavMeshEdgeWeight;
		steeringForce += navmeshEdgeAvoid;
	}
	/*
	if (id==1)
	{
		Vec3 pos = position();
		Con::printf("position %f %f %f target %f %f %f seekTarget %f %f %f",
							pos.x,pos.y,pos.z,mMoveTarget.x,mMoveTarget.y,mMoveTarget.z,
							neighborSeek.x,neighborSeek.y,neighborSeek.z,
							targetSeek.x,targetSeek.y,targetSeek.z);
	}*/

	//if (id==11)
	//	Con::printf("bot 11: collisionAvoid %f %f %f seekTarget %f %f %f avoidNavmesh %f %f %f",neighborAvoid.x,
	//	neighborAvoid.y,neighborAvoid.z,targetSeek.x,targetSeek.y,targetSeek.z,navmeshEdgeAvoid.x,
	//	navmeshEdgeAvoid.y,navmeshEdgeAvoid.z);
	//Con::printf("bot %d: vehicle position: %f %f %f  steeringForce: %f %f %f",id,position().x,position().y,position().z,
	//			steeringForce.x,steeringForce.y,steeringForce.z);
	//HERE: temporarily removing the path following too, for testing.
	/// do (interactively) selected type of path following
	//const float pfLeadTime = 3;
	//const Vec3 pathFollow =
	//	(gUseDirectedPathFollowing ?
	//	steerToFollowPath (pathDirection, pfLeadTime, *path) :
	//steerToStayOnPath (pfLeadTime, *path));

	//// add in to steeringForce
	//steeringForce += pathFollow * 0.5;
	//Con::printf("pathFollow: %f %f %f, steering force: %f %f %f",
	//	pathFollow.x,pathFollow.y,pathFollow.z,
	//	steeringForce.x,steeringForce.y,steeringForce.z);

	//}

	//}


	// return steering constrained to global XZ "ground" plane
	return steeringForce.setYtoZero ();
}



// draw this pedestrian into scene
void NavClient::draw (void)
{
	//drawBasic2dCircularVehicle (*this, gGray50);
	//drawTrail ();
}


// called when steerToFollowPath decides steering is required
void NavClient::annotatePathFollowing (const Vec3& future,
									 const Vec3& onPath,
									 const Vec3& target,
									 const float outside)
{/* //MegaMotion, killing all draw functions for now, GL conflicting.
	const Vec3 yellow (1, 1, 0);
	const Vec3 lightOrange (1.0f, 0.5f, 0.0f);
	const Vec3 darkOrange  (0.6f, 0.3f, 0.0f);
	const Vec3 yellowOrange (1.0f, 0.75f, 0.0f);

	// draw line from our position to our predicted future position
	annotationLine (position(), future, yellow);

	// draw line from our position to our steering target on the path
	annotationLine (position(), target, yellowOrange);

	// draw a two-toned line between the future test point and its
	// projection onto the path, the change from dark to light color
	// indicates the boundary of the tube.
	const Vec3 boundaryOffset = (onPath - future).normalize() * outside;
	const Vec3 onPathBoundary = future + boundaryOffset;
	annotationLine (onPath, onPathBoundary, darkOrange);
	annotationLine (onPathBoundary, future, lightOrange);
	*/
}

// called when steerToAvoidCloseNeighbors decides steering is required
// (parameter names commented out to prevent compiler warning from "-W")
void NavClient::annotateAvoidCloseNeighbor (const AbstractVehicle& other,
											const float /*additionalDistance*/)
{
	// draw the word "Ouch!" above colliding vehicles
	const float headOn = forward().dot(other.forward()) < 0;
	const Vec3 green (0.4f, 0.8f, 0.1f);
	const Vec3 red (1, 0.1f, 0);
	const Vec3 color = headOn ? red : green;
	const char* string = headOn ? "OUCH!" : "pardon me";
	const Vec3 location = position() + Vec3 (0, 0.5f, 0);
	//if (OpenSteerDemo::annotationIsOn())
	//    draw2dTextAt3dLocation (*string, location, color);
}


// (parameter names commented out to prevent compiler warning from "-W")
void NavClient::annotateAvoidNeighbor (const AbstractVehicle& threat,
									 const float /*steer*/,
									 const Vec3& ourFuture,
									 const Vec3& threatFuture)
{/*
	const Vec3 green (0.15f, 0.6f, 0.0f);

	annotationLine (position(), ourFuture, green);
	annotationLine (threat.position(), threatFuture, green);
	annotationLine (ourFuture, threatFuture, gRed);
	annotationXZCircle (radius(), ourFuture,    green, 12);
	annotationXZCircle (radius(), threatFuture, green, 12);
	*/
}

// xxx perhaps this should be a call to a general purpose annotation for
// xxx "local xxx axis aligned box in XZ plane" -- same code in in
// xxx CaptureTheFlag.cpp
void NavClient::annotateAvoidObstacle (const float minDistanceToCollision)
{/*
	const Vec3 boxSide = side() * radius();
	const Vec3 boxFront = forward() * minDistanceToCollision;
	const Vec3 FR = position() + boxFront - boxSide;
	const Vec3 FL = position() + boxFront + boxSide;
	const Vec3 BR = position()            - boxSide;
	const Vec3 BL = position()            + boxSide;
	const Vec3 white (1,1,1);
	annotationLine (FR, FL, white);
	annotationLine (FL, BL, white);
	annotationLine (BL, BR, white);
	annotationLine (BR, FR, white);
	*/
}

void NavClient::newPD (ProximityDatabase& pd)
{
	// delete this boid's token in the old proximity database
	delete proximityToken;

	// allocate a token for this boid in the proximity database
	proximityToken = pd.allocateToken (this);
}

AVGroup NavClient::neighbors;





//PolylinePathway* getNavPath (void)
//{
//    if (gNavPath == NULL)
//    {
//        const float pathRadius = 2;
//
//        const int pathPointCount = 7;
//        const float size = 20;
//        const float top = 2 * size;
//        const float gap = 1.2f * size;
//        const float out = 2 * size;
//        const float h = 0.5;
//        const Vec3 pathPoints[pathPointCount] =
//            {Vec3 (h+gap-out,     0,  h+top-out),  // 0 a
//             Vec3 (h+gap,         0,  h+top),      // 1 b
//             Vec3 (h+gap+(top/2), 0,  h+top/2),    // 2 c
//             Vec3 (h+gap,         0,  h),          // 3 d
//             Vec3 (h,             0,  h),          // 4 e
//             Vec3 (h,             0,  h+top),      // 5 f
//             Vec3 (h+gap,         0,  h+top/2)};   // 6 g
//
//        //gObstacle1.center = interpolate (0.2f, pathPoints[0], pathPoints[1]);
//        //gObstacle2.center = interpolate (0.5f, pathPoints[2], pathPoints[3]);
//        //gObstacle1.radius = 3;
//        //gObstacle2.radius = 5;
//        //gObstacles.push_back (&gObstacle1);
//        //gObstacles.push_back (&gObstacle2);
//
//        gEndpoint0 = pathPoints[0];
//        gEndpoint1 = pathPoints[pathPointCount-1];
//
//        gNavPath = new PolylinePathway (pathPointCount,
//                                         pathPoints,
//                                         pathRadius,
//                                         false);
//    }
//    return gNavPath;
//}


//PolylinePathway* getNavPath (void)
//{

	//if (gNavPath==NULL)
	//{
	//	SimObject *navPathObj = NULL;
	//	navPathObj = Sim::findObject("OneWayPath");

  //  if (gTestPath == NULL)
  //  {
		////if (navPathObj)
		////if (1)
		////{
		//	//gNavPath = dynamic_cast<Nav::NavPath*>(navPathObj);
		//	//Con::printf("found our path object: count %d",gNavPath->getCount());
		//	//mNavPath->setProtectedMesh(this,"NavMeshOne",data?)
		//	const float pathRadius = 1;//2;
		//	//const int pathPointMax = 100;
		//	const int pathPointCount = 6;//gNavPath->getCount();

		//	const Vec3 pathPoints[pathPointCount] = {
		//		Vec3(0,0,0),
		//		Vec3(-12.4,0,-21.4),
		//		Vec3(-13.0,0,-22.6),
		//		Vec3(-11.5,0,-32.2),
		//		Vec3(-10.6,0,-48.7),
		//		Vec3(8,0,-69)
		//	};			
		//	
  //      gEndpoint0 = pathPoints[0];
  //      gEndpoint1 = pathPoints[pathPointCount-1];

  //      gTestPath = new PolylinePathway (pathPointCount,
  //                                       pathPoints,
  //                                       pathRadius,
  //                                       false);
	 //}
			//const Vec3 pathPoints[pathPointCount] = {
			//	Vec3(0,0,0),
			//	Vec3(-12.7,0,-21.7),
			//	Vec3(-13.3,0,-22.9),
			//	Vec3(-0.1,0,-45.7),
			//	Vec3(-0.4,0,-56.8),
			//	Vec3(-0.4,0,-57.7),
			//	Vec3(8,0,-69)
			//};
			
			//for (U32 i=0;i<pathPointCount;i++)
			//{
			//	Point3F pathNode = gNavPath->getNode(i);
			//	pathPoints[i].set(-pathNode.x,pathNode.z,pathNode.y);
			//}//No can do, "const" overrules all...


	//	}
	//} 
    //return gTestPath;
//}
// ----------------------------------------------------------------------------
// OpenSteerDemo PlugIn




NavClientPlugIn gNavClientPlugIn;



void NavClientPlugIn::open (void)
{
	// make the database used to accelerate proximity queries
	cyclePD = -1;
	nextPD ();
	//mPM = dynamic_cast<nxPhysManager*>(physManagerCommon::getPM());

	// create the specified number of Pedestrians
	population = 0;

	//HERE!! Stop doing this, and instead only add them when you add OpenSteer capability to an actor!
	//for (int i = 0; i < NUM_NAV_CLIENTS; i++) addNavClientToCrowd ();

	// initialize camera and selectedVehicle
	//Pedestrian& firstPedestrian = **crowd.begin();
	//OpenSteerDemo::init3dCamera (firstPedestrian);
	//OpenSteerDemo::camera.mode = Camera::cmFixedDistanceOffset;
	//OpenSteerDemo::camera.fixedTarget.set (15, 0, 30);
	//OpenSteerDemo::camera.fixedPosition.set (15, 70, -70);


}

void NavClientPlugIn::update (const float currentTime, const float elapsedTime)
{
	// update each Pedestrian
	for (iterator i = crowd.begin(); i != crowd.end(); i++)
	{
		(**i).update (currentTime, elapsedTime);
	}
}

void NavClientPlugIn::redraw (const float currentTime, const float elapsedTime)
{/*
 // selected Pedestrian (user can mouse click to select another)
 AbstractVehicle& selected = *OpenSteerDemo::selectedVehicle;

 // Pedestrian nearest mouse (to be highlighted)
 AbstractVehicle& nearMouse = *OpenSteerDemo::vehicleNearestToMouse ();

 // update camera
 OpenSteerDemo::updateCamera (currentTime, elapsedTime, selected);

 // draw "ground plane"
 if (OpenSteerDemo::selectedVehicle) gridCenter = selected.position();
 OpenSteerDemo::gridUtility (gridCenter);

 // draw and annotate each Pedestrian
 for (iterator i = crowd.begin(); i != crowd.end(); i++) (**i).draw (); 

 // draw the path they follow and obstacles they avoid
 drawPathAndObstacles ();

 // highlight Pedestrian nearest mouse
 OpenSteerDemo::highlightVehicleUtility (nearMouse);

 // textual annotation (at the vehicle's screen position)
 serialNumberAnnotationUtility (selected, nearMouse);

 // textual annotation for selected Pedestrian
 if (OpenSteerDemo::selectedVehicle && OpenSteerDemo::annotationIsOn())
 {
 const Vec3 color (0.8f, 0.8f, 1.0f);
 const Vec3 textOffset (0, 0.25f, 0);
 const Vec3 textPosition = selected.position() + textOffset;
 const Vec3 camPosition = OpenSteerDemo::camera.position();
 const float camDistance = Vec3::distance (selected.position(),
 camPosition);
 const char* spacer = "      ";
 std::ostringstream annote;
 annote << std::setprecision (2);
 annote << std::setiosflags (std::ios::fixed);
 annote << spacer << "1: speed: " << selected.speed() << std::endl;
 annote << std::setprecision (1);
 annote << spacer << "2: cam dist: " << camDistance << std::endl;
 annote << spacer << "3: no third thing" << std::ends;
 draw2dTextAt3dLocation (annote, textPosition, color);
 }

 // display status in the upper left corner of the window
 std::ostringstream status;
 status << "[F1/F2] Crowd size: " << population;
 status << "\n[F3] PD type: ";
 switch (cyclePD)
 {
 case 0: status << "LQ bin lattice"; break;
 case 1: status << "brute force";    break;
 }
 status << "\n[F4] ";
 if (gUseDirectedPathFollowing)
 status << "Directed path following.";
 else
 status << "Stay on the path.";
 status << "\n[F5] Wander: ";
 if (gWanderSwitch) status << "yes"; else status << "no";
 status << std::endl;
 const float h = drawGetWindowHeight ();
 const Vec3 screenLocation (10, h-50, 0);
 draw2dTextAt2dLocation (status, screenLocation, gGray80);
 */
}


void NavClientPlugIn::serialNumberAnnotationUtility (const AbstractVehicle& selected,
												const AbstractVehicle& nearMouse)
{
	// display a Pedestrian's serial number as a text label near its
	// screen position when it is near the selected vehicle or mouse.
	if (&selected && &nearMouse && OpenSteerDemo::annotationIsOn())
	{
		for (iterator i = crowd.begin(); i != crowd.end(); i++)
		{
			AbstractVehicle* vehicle = *i;
			const float nearDistance = 6;
			const Vec3& vp = vehicle->position();
			const Vec3& np = nearMouse.position();
			if ((Vec3::distance (vp, selected.position()) < nearDistance)
				||
				(&nearMouse && (Vec3::distance (vp, np) < nearDistance)))
			{
				std::ostringstream sn;
				sn << "#"
					<< ((NavClient*)vehicle)->serialNumber
					<< std::ends;
				const Vec3 textColor (0.8f, 1, 0.8f);
				const Vec3 textOffset (0, 0.25f, 0);
				const Vec3 textPos = vehicle->position() + textOffset;
				//draw2dTextAt3dLocation (sn, textPos, textColor);
			}
		}
	}
}

void NavClientPlugIn::drawPathAndObstacles (void)
{/*
 // draw a line along each segment of path
 const PolylinePathway& path = *getTestPath ();
 for (int i = 0; i < path.pointCount; i++)
 if (i > 0) drawLine (path.points[i], path.points[i-1], gRed);

 // draw obstacles
 drawXZCircle (gObstacle1.radius, gObstacle1.center, gWhite, 40);
 drawXZCircle (gObstacle2.radius, gObstacle2.center, gWhite, 40);
 */
}

void NavClientPlugIn::close (void)
{
	// delete all Pedestrians
	while (population > 0) removeNavClientFromCrowd ();
}

void NavClientPlugIn::reset (void)
{
	// reset each Pedestrian
	for (iterator i = crowd.begin(); i != crowd.end(); i++) (**i).reset ();

	// reset camera position
	//OpenSteerDemo::position2dCamera (*OpenSteerDemo::selectedVehicle);

	// make camera jump immediately to new position
	//OpenSteerDemo::camera.doNotSmoothNextMove ();
}

void NavClientPlugIn::handleFunctionKeys (int keyNumber)
{
	switch (keyNumber)
	{
	case 1:  addNavClientToCrowd ();                               break;
	case 2:  removeNavClientFromCrowd ();                          break;
	case 3:  nextPD ();                                             break;
	case 4: gUseDirectedPathFollowing = !gUseDirectedPathFollowing; break;
	case 5: gWanderSwitch = !gWanderSwitch;                         break;
	}
}

void NavClientPlugIn::printMiniHelpForFunctionKeys (void)
{
	std::ostringstream message;
	message << "Function keys handled by ";
	message << '"' << name() << '"' << ':' << std::ends;
	OpenSteerDemo::printMessage (message);
	OpenSteerDemo::printMessage (message);
	OpenSteerDemo::printMessage ("  F1     add a navClient to the crowd.");
	OpenSteerDemo::printMessage ("  F2     remove a navClient from crowd.");
	OpenSteerDemo::printMessage ("  F3     use next proximity database.");
	OpenSteerDemo::printMessage ("  F4     toggle directed path follow.");
	OpenSteerDemo::printMessage ("  F5     toggle wander component on/off.");
	OpenSteerDemo::printMessage ("");
}


//HERE: this needs to accept a vec3, vec3 + rotation, or full transform.
void NavClientPlugIn::addNavClientToCrowd (void)
{
	population++;
	NavClient* navClient = new NavClient (*pd);
	navClient->id = population-1;
	crowd.push_back (navClient);
	//Con::printf("Added navClient: %d",population);
	if (population == 1) OpenSteerDemo::selectedVehicle = navClient;
}

//HERE: this needs to have an argument specifying which one to remove.
void NavClientPlugIn::removeNavClientFromCrowd (void)
{
	if (population > 0)
	{
		// save pointer to last pedestrian, then remove it from the crowd
		const NavClient* navClient = crowd.back();
		crowd.pop_back();
		population--;

		// if it is OpenSteerDemo's selected vehicle, unselect it
		if (navClient == OpenSteerDemo::selectedVehicle)
			OpenSteerDemo::selectedVehicle = NULL;

		// delete the Pedestrian
		delete navClient;
	}
}


// for purposes of demonstration, allow cycling through various
// types of proximity databases.  this routine is called when the
// OpenSteerDemo user pushes a function key.
void NavClientPlugIn::nextPD (void)
{
	// save pointer to old PD
	ProximityDatabase* oldPD = pd;

	// allocate new PD
	const int totalPD = 2;
	switch (cyclePD = (cyclePD + 1) % totalPD)
	{
	case 0:
		{
			const Vec3 center;
			const float div = 20.0f;
			const Vec3 divisions (div, 1.0f, div);
			const float diameter = 100.0f; //??? need better way to get this
			const Vec3 dimensions (diameter, diameter, diameter);
			typedef LQProximityDatabase<AbstractVehicle*> LQPDAV;
			pd = new LQPDAV (center, dimensions, divisions);
			break;
		}
	case 1:
		{
			pd = new BruteForceProximityDatabase<AbstractVehicle*> ();
			break;
		}
	}

	// switch each boid to new PD
	for (iterator i=crowd.begin(); i!=crowd.end(); i++) (**i).newPD(*pd);

	// delete old PD (if any)
	delete oldPD;
}

// ----------------------------------------------------------------------------

//Ecstasy Motion
void NavClientPlugIn::createVehicle (const Vec3 pos,float rot)
{
	Con::printf("calling navclient plugin createVehicle!! pos %f %f %f  rot %f",pos.x,pos.y,pos.z,rot);
	population++;
	NavClient* navClient = new NavClient (*pd);
	navClient->id = population-1;
	crowd.push_back (navClient);
	navClient->setPosition(pos);
	navClient->setForward(0.0,0.0,1.0);//NEXT: rotate this by rot value, *or* exchange that for a Vec3 forward vector.
}

