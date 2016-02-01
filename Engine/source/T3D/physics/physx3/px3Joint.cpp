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

/*
//Snagged this from internet, (http://gist.github.com/ashwin/7598571) Turned out not to be necessary for joints, but could be useful 
//anyway if we don't already have this capability in MatrixF or QuatF (?)

physx::PxQuat getLookAtPxQuat(const Point3F& fromPos, const Point3F& toPos)
{
	// Position of toPos relative to fromPos
	Point3F relToPos = toPos - fromPos;

	///
	// First we rotate fromPos around Y axis to look at toPos
	// This gives us Euler angle around Y axis
	///

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

	///
	// Next fromPos will look "up" to see toPos
	// This gives us Euler angle around Z axis
	//

	// Compute the angle
	// theta = atan( y / sqrt(x^2 + z^2))
	const float zAng0 = atan(fabs(relToPos.y) /
		sqrt(relToPos.x * relToPos.x + relToPos.z * relToPos.z));

	// Fix angle based on whether toPos is above or below XZ plane
	const float zAng = (relToPos.y >= 0) ? zAng0 : -zAng0;

	///
	// Convert Euler angles to quaternion that rotates
	// X axis of upright orientation to point at toPos
	// Reference: PhysX Math Primer
	//

	// Convert to quaternions
	physx::PxQuat qy(yAng, physx::PxVec3(0, 1, 0));
	physx::PxQuat qz(zAng, physx::PxVec3(0, 0, 1));

	// Rotate local axes
	physx::PxQuat q = qy * qz;

	return q;
}

QuatF getLookAtQuatF(const Point3F& fromPos, const Point3F& toPos)
{
	physx::PxQuat pxQ = getLookAtPxQuat(fromPos,toPos);
	QuatF out(pxQ.x,pxQ.y,pxQ.z,pxQ.w);
	return out;
}
*/
//-----------------------------------------------------------------------------
// Constructor/Destructor
//-----------------------------------------------------------------------------
Px3Joint::Px3Joint(physx::PxRigidActor* A, physx::PxRigidActor* B,Px3World* world,
								physicsJointData *jD,Point3F origin,Point3F jointRots,Point3F jointRots2,MatrixF invShapeTrans)
{
	physx::PxTransform localTrans0,localTrans1;

	Point3F posA(A->getGlobalPose().p.x,A->getGlobalPose().p.y,A->getGlobalPose().p.z);
	Point3F posB(B->getGlobalPose().p.x,B->getGlobalPose().p.y,B->getGlobalPose().p.z);

	Point3F offsetA = origin - posA;
	Point3F offsetB = origin - posB;

	physx::PxQuat localQuat(1.0,0.0,0.0,0.0);
	physx::PxQuat localQuat2(1.0,0.0,0.0,0.0);
	if (jointRots.len()>0)
	{
		EulerF rots(mDegToRad(jointRots.x),mDegToRad(jointRots.y),mDegToRad(jointRots.z));
		QuatF localRot = QuatF(rots);
		localQuat = physx::PxQuat(localRot.x,localRot.y,localRot.z,localRot.w);
	}

	
	if (jointRots2.len()>0)
	{
		EulerF rots(mDegToRad(jointRots2.x),mDegToRad(jointRots2.y),mDegToRad(jointRots2.z));
		QuatF localRot = QuatF(rots);
		localQuat2 = physx::PxQuat(localRot.x,localRot.y,localRot.z,localRot.w);
		localQuat *= localQuat2;
	}

	Point3F mulOffsetB;
	invShapeTrans.mulP(offsetB,&mulOffsetB);

	localTrans0 = physx::PxTransform(physx::PxVec3(offsetA.x,offsetA.y,offsetA.z),localQuat);
	localTrans1 = physx::PxTransform(physx::PxVec3(mulOffsetB.x,mulOffsetB.y,mulOffsetB.z),localQuat);

	loadJointData(jD);//Sets up all the local variables for this joint using the values in the joint data struct.

	if (mJD.jointType==PHYS_JOINT_SPHERICAL) {	

		physx::PxSphericalJoint* sphericalJoint = physx::PxSphericalJointCreate(*gPhysics3SDK,B,localTrans1,A,localTrans0);
		mJoint = dynamic_cast<physx::PxJoint*>(sphericalJoint);

		sphericalJoint->setLimitCone(physx::PxJointLimitCone(mJD.swingLimit, mJD.swingLimit2, 0.01f));
		sphericalJoint->setSphericalJointFlag(physx::PxSphericalJointFlag::eLIMIT_ENABLED, true);

	} else if  (mJD.jointType==PHYS_JOINT_REVOLUTE) {
		
		physx::PxRevoluteJoint* revoluteJoint = physx::PxRevoluteJointCreate(*gPhysics3SDK,B,localTrans1,A,localTrans0);
		mJoint = dynamic_cast<physx::PxJoint*>(revoluteJoint);

		revoluteJoint->setLimit(physx::PxJointAngularLimitPair(-mJD.swingLimit/2.0f, mJD.swingLimit/2.0f, 0.1f)); 
		revoluteJoint->setRevoluteJointFlag(physx::PxRevoluteJointFlag::eLIMIT_ENABLED, true);

	} else if  (mJD.jointType==PHYS_JOINT_PRISMATIC) {
		
		physx::PxPrismaticJoint* prismaticJoint = physx::PxPrismaticJointCreate(*gPhysics3SDK,B,localTrans1,A,localTrans0);
		mJoint = dynamic_cast<physx::PxJoint*>(prismaticJoint);

		//prismaticJoint->setLimit(physx::PxJointLimitPair(-mXLimit, mXLimit));//Hm, nvidia example is wrong...

	} else if  (mJD.jointType==PHYS_JOINT_FIXED) {

		physx::PxFixedJoint* fixedJoint = physx::PxFixedJointCreate(*gPhysics3SDK,B,localTrans1,A,localTrans0);
		mJoint = dynamic_cast<physx::PxJoint*>(fixedJoint);

		//Nothing else to do, it's fixed.

	} else if  (mJD.jointType==PHYS_JOINT_DISTANCE) {

		physx::PxDistanceJoint* distanceJoint = physx::PxDistanceJointCreate(*gPhysics3SDK,B,localTrans1,A,localTrans0);
		mJoint = dynamic_cast<physx::PxJoint*>(distanceJoint);

		distanceJoint->setMaxDistance(mJD.XLimit);
		distanceJoint->setMinDistance(mJD.XLimit/10.0f);//FIX, include min in database.

	} else if  (mJD.jointType==PHYS_JOINT_D6) {

		physx::PxD6Joint* d6Joint = physx::PxD6JointCreate(*gPhysics3SDK,B,localTrans1,A,localTrans0);//dynamic_cast<physx::PxD6Joint*>(mJoint);
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
		//HERE: if swingLimit, or swingLimit2, are equal to zero (or less than a tiny number) then lock them, else limit.
		d6Joint->setMotion(physx::PxD6Axis::eSWING1, physx::PxD6Motion::eLIMITED);
		
		d6Joint->setMotion(physx::PxD6Axis::eSWING1, physx::PxD6Motion::eLIMITED);
		d6Joint->setMotion(physx::PxD6Axis::eSWING2, physx::PxD6Motion::eLIMITED);

		//HERE: is this the only way to do it? How do I set an upper and lower amount per axis??
		d6Joint->setSwingLimit(physx::PxJointLimitCone(mJD.swingLimit, mJD.swingLimit2, 1.0f));
		d6Joint->setTwistLimit(physx::PxJointAngularLimitPair(-mJD.twistLimit,mJD.twistLimit,1.0f));

	} else {

		Con::printf("Couldn't find joint type: %d",mJD.jointType);
		return;
	}

	mJoint->setBreakForce(mJD.maxForce,mJD.maxTorque);

	mJoint->setConstraintFlag(physx::debugger::PxConstraintFlag::eVISUALIZATION,true);

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

	//physx::PxD6Joint* d6joint = dynamic_cast<physx::PxD6Joint*>(mJoint);

	//I don't think this works... hmm
	//d6joint->setDrivePosition(physx::PxTransform(physx::PxQuat(mMotorTarget.x,mMotorTarget.y,mMotorTarget.z,mMotorTarget.w)));

}
