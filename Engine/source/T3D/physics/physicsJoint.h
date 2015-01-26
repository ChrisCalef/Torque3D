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

#ifndef _T3D_PHYSICS_PHYSICSJOINT_H_
#define _T3D_PHYSICS_PHYSICSJOINT_H_

#ifndef _T3D_PHYSICSCOMMON_H_
#include "T3D/physics/physicsCommon.h"
#endif
#ifndef _T3D_PHYSICS_PHYSICSOBJECT_H_
#include "T3D/physics/physicsObject.h"
#endif

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


struct physicsJointData
{
	U32 jointID;

	enum physicsJointType jointType;

	F32 twistLimit;
	F32 swingLimit;
	F32 swingLimit2;

	F32 XLimit;
	F32 YLimit;
	F32 ZLimit;

	Point3F localAxis;
	Point3F localNormal;

	F32 swingSpring;
	F32 twistSpring;
	F32 springDamper;
	
	F32 motorSpring;
	F32 motorDamper;

	F32 maxForce;
	F32 maxTorque;
	
	Point3F limitPoint;//separate off to allow unlimited? Low priority, four limit planes should be enough.
	Point3F limitPlaneAnchor1;
	Point3F limitPlaneNormal1;
	Point3F limitPlaneAnchor2;
	Point3F limitPlaneNormal2;
	Point3F limitPlaneAnchor3;
	Point3F limitPlaneNormal3;
	Point3F limitPlaneAnchor4;
	Point3F limitPlaneNormal4;

};

class PhysicsJoint
{
public:
   virtual ~PhysicsJoint() {}

   PhysicsBody *mBodyA;
   PhysicsBody *mBodyB;
   
   PhysicsWorld *mWorld;

   U32 mJointID;//Database ID for this joint.

   ////// physicsJointData, from db or datablocks or xml or whatever. ////////////////
   physicsJointData mJD;

   QuatF mMotorTarget;

    ////// physicsJointData /////////////////////////////////////////////////////

   virtual QuatF &getMotorTarget()=0;
   virtual void setMotorTarget(QuatF &)=0;

   /*
   virtual void	onWorldStep()=0;
   virtual void setMotorVelocity(F32)=0;
   virtual void setMotorTargetVelocity(Point3F &)=0;
   virtual QuatF &getLastTarget()=0;
   virtual void setLastTarget(QuatF &)=0;
   virtual QuatF &getNewTarget()=0;
   virtual void setNewTarget(QuatF &)=0;
   virtual void clearMotor()=0;
   virtual void scaleMotorForce(F32)=0;
   */
};

#endif // _T3D_PHYSICS_PHYSICSJOINT_H_