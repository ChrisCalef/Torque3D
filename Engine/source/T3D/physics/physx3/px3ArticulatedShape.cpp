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
#include "T3D/physics/physx3/Px3ArticulatedShape.h"
#include "T3D/physics/physx3/px3Casts.h"

//-----------------------------------------------------------------------------
// Constructor/Destructor
//-----------------------------------------------------------------------------
Px3ArticulatedShape::Px3ArticulatedShape()
{
    mShapeInstance = NULL;
    mWorld = NULL;
    mScene = NULL;
    mDebugTick=0;
}

Px3ArticulatedShape::~Px3ArticulatedShape()
{
}


void Px3ArticulatedShape::setTransform( const MatrixF &mat )
{
    mTransform = mat;
}


bool Px3ArticulatedShape::create(TSShapeInstance* shapeInst, const MatrixF &transform)
{
    mWorld = dynamic_cast<Px3World*>( PHYSICSMGR->getWorld( "client" ) );
      
    if ( !mWorld || !mWorld->getScene() )
    {
        Con::errorf( "Px3Cloth::onAdd() - PhysXWorld not initialized... cloth disabled!" );
        return false;
    }

    mScene = mWorld->getScene();
    //PhysicsPlugin::getPhysicsResetSignal().notify( this, &Px3ClothShape::onPhysicsReset, 1053.0f );

    // Save shape and transform data.
    mShapeInstance = shapeInst;
    mTransform = transform;

    return true;
}

void Px3ArticulatedShape::release()
{

    // Remove physics plugin callback.
    //PhysicsPlugin::getPhysicsResetSignal().remove( this, &Px3ClothShape::onPhysicsReset );
}

void Px3ArticulatedShape::processTick()
{
	if ( !mShapeInstance )
		return;

	// Update bounds.
	if ( mWorld->isEnabled() )
	{

		bool displayDebug = false;
		mDebugTick++;
		if ( mDebugTick > 32 )
		{
			displayDebug = true;
			mDebugTick = 0;
		}

	}
}
