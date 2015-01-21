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
	enum physicsJointType jointType;

	F32 twistLimit;
	F32 swingLimit;
	F32 swingLimit2;

	F32 xLimit;
	F32 yLimit;
	F32 zLimit;

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

   ////// physicsJointData, from db or datablocks or xml or whatever. ////////////////
   enum physicsJointType jointType;

   F32 twistLimit;
   F32 swingLimit;
   F32 swingLimit2;

   F32 xLimit;
   F32 yLimit;
   F32 zLimit;

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
   
   virtual void setTwistLimit(F32)=0;
   virtual void setSwingLimit(F32)=0;
   virtual void setSwingLimit2(F32)=0;
   
   virtual void setMaxForce(F32)=0;
   virtual void setMaxTorque(F32)=0;

   virtual void setSwingSpring(F32)=0;
   virtual void setTwistSpring(F32)=0;
   virtual void setSpringDamper(F32)=0;
   virtual void setSpringTargetAngle(F32)=0;
   virtual void setMotorSpring(F32)=0;
   virtual void setMotorDamper(F32)=0;

   virtual void setAxisA(Point3F &)=0;
   virtual void setAxisB(Point3F &)=0;
   virtual void setNormalA(Point3F &)=0;
   virtual void setNormalB(Point3F &)=0;

   virtual void setLocalAnchor0(Point3F &)=0;
   virtual void setLocalAnchor1(Point3F &)=0;
   virtual void setLocalAxis0(Point3F &)=0;
   virtual void setLocalAxis1(Point3F &)=0;
   virtual void setLocalNormal0(Point3F &)=0;
   virtual void setLocalNormal1(Point3F &)=0;

   virtual void setGlobalAnchor(Point3F &)=0;
   virtual void setGlobalAxis(Point3F &)=0;
   virtual void setGlobalNormal(Point3F &)=0;

   virtual void setLimitPoint(Point3F &)=0;
   virtual void setLimitPlaneAnchor1(Point3F &)=0;
   virtual void setLimitPlaneNormal1(Point3F &)=0;
   virtual void setLimitPlaneAnchor2(Point3F &)=0;
   virtual void setLimitPlaneNormal2(Point3F &)=0;
   virtual void setLimitPlaneAnchor3(Point3F &)=0;
   virtual void setLimitPlaneNormal3(Point3F &)=0;
   virtual void setLimitPlaneAnchor4(Point3F &)=0;
   virtual void setLimitPlaneNormal4(Point3F &)=0;

   virtual physJointType getJointType()=0;
   virtual void setJointType(physJointType)=0;

   virtual bool getHW()=0;
   virtual void setHW(bool b=true)=0;
   */

};

#endif // _T3D_PHYSICS_PHYSICSJOINT_H_