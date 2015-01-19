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

#ifndef _PX3ARTICULATED_SHAPE_H_
#define _PX3ARTICULATED_SHAPE_H_

#include "T3D/physics/physx3/px3.h"
#include "T3D/physics/physx3/px3World.h"
#include "T3D/physics/physx3/px3Plugin.h"
#include "T3D/physics/physicsArticulatedShape.h"
#include "ts/tsShapeInstance.h"

#ifndef _ITICKABLE_H_
    #include "core/iTickable.h"
#endif


class Px3ArticulatedShape : public PhysicsArticulatedShape
{
   TSShapeInstance*		mShapeInstance;

   U32 mDebugTick;

protected:
	// PhysX 3.3 Variables.
	Px3World *mWorld;
	physx::PxScene *mScene;

	// Used to track last physics reset position.
	MatrixF mTransform;

public:
   Px3ArticulatedShape();
   virtual ~Px3ArticulatedShape();
   
   bool create(TSShapeInstance* shapeInst, const MatrixF &transform);
   void release();
   void processTick();
   void setTransform( const MatrixF &mat );
};

#endif // _PX3ARTICULATED_SHAPE_H_
