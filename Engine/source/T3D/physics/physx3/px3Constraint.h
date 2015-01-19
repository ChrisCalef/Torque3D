//-----------------------------------------------------------------------------
// Authors: 
//        Chris Calef  -  indiemotionsoftware.com
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

#ifndef _PX3CONSTRAINT_H_
#define _PX3CONSTRAINT_H_

#include "T3D/physics/physx3/px3.h"
#include "T3D/physics/physx3/px3World.h"
#include "T3D/physics/physx3/px3Plugin.h"
#include "T3D/physics/physicsConstraint.h"



//-----------------------------------------------------------------------------
// Px3ClothShape provides the ability to load a shape into torque and have 
// individual mesh pieces appear as cloth in the world. The mesh must meet
// the following criteria:
//  1) Each mesh must be named Cloth-x or cloth-x where x is any positive
//     integer.
//  2) Every vertex must be weighted to either bone 0 or bone 1. Meaning two
//     bones have to be created that each cloth mesh is weighed to. The
//     vertices on bone 0 are treated as loose cloth, the ones on bone 1 are
//     treated as fixed points.
//-----------------------------------------------------------------------------

class Px3Constraint : public PhysicsConstraint
{
   Px3Constraint();
   virtual ~Px3Constraint();
   
};

#endif // _PX3CONSTRAINT_H_
