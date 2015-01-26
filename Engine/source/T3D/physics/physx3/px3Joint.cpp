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
//-----------------------------------------------------------------------------
// Constructor/Destructor
//-----------------------------------------------------------------------------
Px3Joint::Px3Joint(physx::PxRigidActor* A, physx::PxRigidActor* B,Px3World* world,physicsJointData *jD)
{
	physx::PxTransform offset0,offset1;

	Point3F posA(A->getGlobalPose().p.x,A->getGlobalPose().p.y,A->getGlobalPose().p.z);
	Point3F posB(B->getGlobalPose().p.x,B->getGlobalPose().p.y,B->getGlobalPose().p.z);
	Point3F diff = posA - posB;
	Point3F center = posB + (diff/2);
	Point3F offsetA = center - posA;
	Point3F offsetB = center - posB;

	offset0 = physx::PxTransform(physx::PxVec3(offsetA.x,offsetA.y,offsetA.z));
	offset1 = physx::PxTransform(physx::PxVec3(offsetB.x,offsetB.y,offsetB.z));
		
	world->lockScene();
	
	loadJointData(jD);//Sets up all the local variables for this joint using the values in the joint data struct.

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

		physx::PxD6Joint* d6Joint = physx::PxD6JointCreate(*gPhysics3SDK,A,offset0,B,offset1);//dynamic_cast<physx::PxD6Joint*>(mJoint);
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
	Con::printf("Created a joint: %d",mJD.jointType);
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
