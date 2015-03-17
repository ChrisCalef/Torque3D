//-----------------------------------------------------------------------------
// Authors: 
//        Chris Calef  -  indiemotionsoftware.com
//        With credit to:
//			timmy@deadlymatter.com
//			Andrew MacIntyre - Aldyre Studios - aldyre.com
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
//-----------------------------------------------------------------------------

#include "platform/platform.h"
#include "T3D/physics/physx3/px3.h"
#include "T3D/physics/physx3/px3Casts.h"
#include "T3D/physics/physx3/px3World.h"
#include "T3D/physics/physx3/px3Joint.h"
#include "T3D/physics/physicsBody.h"

/*
enum physicsJointType
{
	PHYS_JOINT_SPHERICAL = 0,
	PHYS_JOINT_REVOLUTE,
	PHYS_JOINT_PRISMATIC,
	PHYS_JOINT_FIXED,
	PHYS_JOINT_DISTANCE,
	PHYS_JOINT_D6,
	PHYS_JOINT_TYPE_COUNT
};
*/
//Snagged this from internet, sorry forgot to source, don't we have slerp for this(?)
physx::PxQuat getLookAtQuat(const Point3F& fromPos, const Point3F& toPos)
{
	// Position of toPos relative to fromPos
	Point3F relToPos = toPos - fromPos;

	/**
	* First we rotate fromPos around Y axis to look at toPos
	* This gives us Euler angle around Y axis
	*/

	// Compute the angle
	// theta = atan(z/x)
	const float yAng0 = atan(fabs(relToPos.z) / fabs(relToPos.x));

	// Fix the angle based on XZ quadrant point lies in
	float yAng;
	if (relToPos.x >= 0)
	{
		if (relToPos.z >= 0)
			yAng = 2 * M_PI - yAng0; // 360 - theta
		else
			yAng = yAng0;
	}
	else
	{
		if (relToPos.z >= 0)
			yAng = M_PI + yAng0; // 180 + theta
		else
			yAng = M_PI - yAng0; // 180 - theta
	}

	/**
	* Next fromPos will look "up" to see toPos
	* This gives us Euler angle around Z axis
	*/

	// Compute the angle
	// theta = atan( y / sqrt(x^2 + z^2))
	const float zAng0 = atan(fabs(relToPos.y) /
		sqrt(relToPos.x * relToPos.x + relToPos.z * relToPos.z));

	// Fix angle based on whether toPos is above or below XZ plane
	const float zAng = (relToPos.y >= 0) ? zAng0 : -zAng0;

	/**
	* Convert Euler angles to quaternion that rotates
	* X axis of upright orientation to point at toPos
	* Reference: PhysX Math Primer
	*/

	// Convert to quaternions
	physx::PxQuat qy(yAng, physx::PxVec3(0, 1, 0));
	physx::PxQuat qz(zAng, physx::PxVec3(0, 0, 1));

	// Rotate local axes
	physx::PxQuat q = qy * qz;

	return q;
}

//-----------------------------------------------------------------------------
// Constructor/Destructor
//-----------------------------------------------------------------------------
Px3Joint::Px3Joint(physx::PxRigidActor* A, physx::PxRigidActor* B,Px3World* world,
								physicsJointData *jD,Point3F origin,Point3F jointRots)
{
	physx::PxTransform offset0,offset1;

	Point3F posA(A->getGlobalPose().p.x,A->getGlobalPose().p.y,A->getGlobalPose().p.z);
	Point3F posB(B->getGlobalPose().p.x,B->getGlobalPose().p.y,B->getGlobalPose().p.z);

	//Now, instead of assuming the exact center between the two body positions, we are going to get it as an argument.
	//Point3F diff = posA - posB;
	//Point3F center = posB + (diff/2);

	Point3F offsetA = origin - posA;
	Point3F offsetB = origin - posB;

	physx::PxQuat lookatQuat = getLookAtQuat(posB,posA);
	//QuatF lookatQ;
	//lookatQ.shortestArc(posB,posA);//Hm, nope, totally different, ...?
	if (jointRots.len()>0)
	{
		EulerF rots(mDegToRad(jointRots.x),mDegToRad(jointRots.y),mDegToRad(jointRots.z));
		QuatF localRot = QuatF(rots);
		physx::PxQuat localQuat(localRot.x,localRot.y,localRot.z,localRot.w);
		lookatQuat = localQuat * lookatQuat;
	}
	offset0 = physx::PxTransform(physx::PxVec3(offsetA.x,offsetA.y,offsetA.z),lookatQuat);
	offset1 = physx::PxTransform(physx::PxVec3(offsetB.x,offsetB.y,offsetB.z),lookatQuat);

	world->lockScene();
	
	loadJointData(jD);//Sets up all the local variables for this joint using the values in the joint data struct.
	
	//Con::printf("Trying to make a joint. type %d jointRots %f %f %f",mJD.jointType,
	//	jointRots.x,jointRots.y,jointRots.z);

	if (mJD.jointType==PHYS_JOINT_SPHERICAL) {	

		physx::PxSphericalJoint* sphericalJoint = physx::PxSphericalJointCreate(*gPhysics3SDK,A,offset0,B,offset1);
		mJoint = dynamic_cast<physx::PxJoint*>(sphericalJoint);

		sphericalJoint->setLimitCone(physx::PxJointLimitCone(mJD.swingLimit, mJD.swingLimit2, 0.01f));
		sphericalJoint->setSphericalJointFlag(physx::PxSphericalJointFlag::eLIMIT_ENABLED, true);

	} else if  (mJD.jointType==PHYS_JOINT_REVOLUTE) {
		
		physx::PxRevoluteJoint* revoluteJoint = physx::PxRevoluteJointCreate(*gPhysics3SDK,A,offset0,B,offset1);
		mJoint = dynamic_cast<physx::PxJoint*>(revoluteJoint);

		revoluteJoint->setLimit(physx::PxJointAngularLimitPair(-mJD.swingLimit/2.0f, mJD.swingLimit/2.0f, 0.1f)); 
		revoluteJoint->setRevoluteJointFlag(physx::PxRevoluteJointFlag::eLIMIT_ENABLED, true);

	} else if  (mJD.jointType==PHYS_JOINT_PRISMATIC) {
		
		physx::PxPrismaticJoint* prismaticJoint = physx::PxPrismaticJointCreate(*gPhysics3SDK,A,offset0,B,offset1);
		mJoint = dynamic_cast<physx::PxJoint*>(prismaticJoint);

		//prismaticJoint->setLimit(physx::PxJointLimitPair(-mXLimit, mXLimit));//Hm, nvidia example is wrong...

	} else if  (mJD.jointType==PHYS_JOINT_FIXED) {

		physx::PxFixedJoint* fixedJoint = physx::PxFixedJointCreate(*gPhysics3SDK,A,offset0,B,offset1);
		mJoint = dynamic_cast<physx::PxJoint*>(fixedJoint);

		//Nothing else to do, it's fixed.

	} else if  (mJD.jointType==PHYS_JOINT_DISTANCE) {

		physx::PxDistanceJoint* distanceJoint = physx::PxDistanceJointCreate(*gPhysics3SDK,A,offset0,B,offset1);
		mJoint = dynamic_cast<physx::PxJoint*>(distanceJoint);

		distanceJoint->setMaxDistance(mJD.XLimit);
		distanceJoint->setMinDistance(mJD.XLimit/10.0f);//FIX, include min in database.

	} else if  (mJD.jointType==PHYS_JOINT_D6) {

		physx::PxD6Joint* d6Joint = physx::PxD6JointCreate(*gPhysics3SDK,B,offset1,A,offset0);//dynamic_cast<physx::PxD6Joint*>(mJoint);
		mJoint = dynamic_cast<physx::PxJoint*>(d6Joint);

		//if (jD->xLimit<=0) d6Joint->setMotion(physx::PxD6Axis::eX, physx::PxD6Motion::eLOCKED);
		//else {
			//d6Joint->setMotion(physx::PxD6Axis::eX, physx::PxD6Motion::eLIMITED);
			//d6Joint->setLinearLimit(physx::PxJointLinearLimit(jD->xLimit,0.1f));//??
		//} // HMMM, seems the help file isn't so helpful here either. Look for it later.
		d6Joint->setMotion(physx::PxD6Axis::eX, physx::PxD6Motion::eLOCKED);
		d6Joint->setMotion(physx::PxD6Axis::eY, physx::PxD6Motion::eLOCKED);
		d6Joint->setMotion(physx::PxD6Axis::eZ, physx::PxD6Motion::eLOCKED);
		d6Joint->setMotion(physx::PxD6Axis::eTWIST, physx::PxD6Motion::eLIMITED);
		d6Joint->setMotion(physx::PxD6Axis::eSWING1, physx::PxD6Motion::eLIMITED);
		d6Joint->setMotion(physx::PxD6Axis::eSWING2, physx::PxD6Motion::eLIMITED);

		d6Joint->setSwingLimit(physx::PxJointLimitCone(mJD.swingLimit, mJD.swingLimit2, 0.01f));
		d6Joint->setTwistLimit(physx::PxJointAngularLimitPair(-mJD.twistLimit,mJD.twistLimit,0.01f));

	} else {
		Con::printf("Couldn't find joint type: %d",mJD.jointType);
		world->unlockScene();
		return;
	}

	mJoint->setBreakForce(mJD.maxForce,mJD.maxTorque);

	//physx::PxTransform pxTrans0;
	//physx::PxTransform pxTrans1;
	//Con::printf("posA %f %f %f  posB %f %f %f",posA.x,posA.y,posA.z,posB.x,posB.y,posB.z);
	//physx::PxQuat pxq0,pxq1;

	//Here, I think I need slerp. I have a default orientation that puts 
	//TEMP, HACK...??? FIX
	//if (posA.z > posB.z)
	//	pxq0 = physx::PxQuat(mDegToRad(-90.0f),physx::PxVec3(0,1,0));
	//else if (posA.z < posB.z)
	//	pxq0 = physx::PxQuat(mDegToRad(90.0f),physx::PxVec3(0,1,0));

	//pxTrans0 = physx::PxTransform::createIdentity();
	//pxTrans1 = physx::PxTransform(pxq0);

	//mJoint->setLocalPose(physx::PxJointActorIndex::eACTOR0,pxTrans0);
	//mJoint->setLocalPose(physx::PxJointActorIndex::eACTOR1,pxTrans0);//pxTrans1

	mJoint->setConstraintFlag(physx::debugger::PxConstraintFlag::eVISUALIZATION,true);

	//if (mJoint) Con::printf("Created a joint: type %d offsetA %f %f %f offsetB %f %f %f",mJD.jointType,
	//	offsetA.x,offsetA.y,offsetA.z,offsetB.x,offsetB.y,offsetB.z);
	//else Con::printf("Joint creation FAILED type %d offsetA %f %f %f offsetB %f %f %f",mJD.jointType,
	//	offsetA.x,offsetA.y,offsetA.z,offsetB.x,offsetB.y,offsetB.z);

	world->unlockScene();//This may be unnecessary(?)
	//setup();
}

Px3Joint::~Px3Joint()
{
}

void Px3Joint::loadJointData(physicsJointData *jD)
{
	//Just to get this out of the way of any other function...
	mJD.jointID = jD->jointID;
	mJD.jointType = jD->jointType;
	mJD.twistLimit = jD->twistLimit;
	mJD.swingLimit = jD->swingLimit;
	mJD.swingLimit2 = jD->swingLimit2;
	mJD.XLimit = jD->XLimit;
	mJD.YLimit = jD->YLimit;
	mJD.ZLimit = jD->ZLimit;
	mJD.localAxis = jD->localAxis;
	mJD.localNormal = jD->localNormal;
	mJD.swingSpring = jD->swingSpring;
	mJD.twistSpring = jD->twistSpring;
	mJD.springDamper = jD->springDamper;
	mJD.motorSpring = jD->motorSpring;
	mJD.motorDamper = jD->motorDamper;
	mJD.maxForce = jD->maxForce;
	mJD.maxTorque = jD->maxForce;
	mJD.limitPoint = jD->limitPoint;
	mJD.limitPlaneAnchor1 = jD->limitPlaneAnchor1;
	mJD.limitPlaneNormal1 = jD->limitPlaneNormal1;
	mJD.limitPlaneAnchor2 = jD->limitPlaneAnchor2;
	mJD.limitPlaneNormal2 = jD->limitPlaneNormal2;
	mJD.limitPlaneAnchor3 = jD->limitPlaneAnchor3;
	mJD.limitPlaneNormal3 = jD->limitPlaneNormal3;
	mJD.limitPlaneAnchor4 = jD->limitPlaneAnchor4;
	mJD.limitPlaneNormal4 = jD->limitPlaneNormal4;
			   /*
		   limitPoint_x        REAL,
		   limitPoint_y        REAL,
		   limitPoint_z        REAL,
		   limitPlaneAnchor1_x REAL,
		   limitPlaneAnchor1_y REAL,
		   limitPlaneAnchor1_z REAL,
		   limitPlaneNormal1_x REAL,
		   limitPlaneNormal1_y REAL,
		   limitPlaneNormal1_z REAL,
		   limitPlaneAnchor2_x REAL,
		   limitPlaneAnchor2_y REAL,
		   limitPlaneAnchor2_z REAL,
		   limitPlaneNormal2_x REAL,
		   limitPlaneNormal2_y REAL,
		   limitPlaneNormal2_z REAL,
		   limitPlaneAnchor3_x REAL,
		   limitPlaneAnchor3_y REAL,
		   limitPlaneAnchor3_z REAL,
		   limitPlaneNormal3_x REAL,
		   limitPlaneNormal3_y REAL,
		   limitPlaneNormal3_z REAL,
		   limitPlaneAnchor4_x REAL,
		   limitPlaneAnchor4_y REAL,
		   limitPlaneAnchor4_z REAL,
		   limitPlaneNormal4_x REAL,
		   limitPlaneNormal4_y REAL,
		   limitPlaneNormal4_z REAL
		   */
	return;
}

QuatF& Px3Joint::getMotorTarget()
{
	return mMotorTarget;
}

void Px3Joint::setMotorTarget(QuatF &target)
{
	mMotorTarget = target;

	Con::printf("px3joint set motor target! %f %f %f %f",target.x,target.y,target.z,target.w);

	physx::PxD6Joint* d6joint = dynamic_cast<physx::PxD6Joint*>(mJoint);

	//I don't think this works...
	//d6joint->setDrivePosition(physx::PxTransform(physx::PxQuat(mMotorTarget.x,mMotorTarget.y,mMotorTarget.z,mMotorTarget.w)));

}
