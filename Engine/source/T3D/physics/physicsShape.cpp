//-----------------------------------------------------------------------------
// Copyright (c) 2012 GarageGames, LLC
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
#include "T3D/physics/physicsShape.h"

#include "console/consoleTypes.h"
#include "core/stream/bitStream.h"
#include "core/stream/fileStream.h"
#include "core/resourceManager.h"
#include "math/mathIO.h"
#include "T3D/physics/physicsPlugin.h"
#include "T3D/physics/physicsBody.h"
#include "T3D/physics/physicsWorld.h"
#include "T3D/physics/physicsCollision.h"
#include "collision/concretePolyList.h"
#include "ts/tsShapeInstance.h"
#include "scene/sceneRenderState.h"
#include "gfx/gfxTransformSaver.h"
#include "T3D/physics/physicsDebris.h"
#include "T3D/fx/explosion.h"
#include "T3D/containerQuery.h"
#include "lighting/lightQuery.h"
#include "console/engineAPI.h"
#include "console/SQLiteObject.h"
#include "T3D/camera.h"
#include "T3D/player.h"
#include <time.h>

#ifndef _VEHICLEDATASOURCE_H_
	#include "sim/dataSource/vehicleDataSource.h"
#endif

#ifndef OPENSTEER_OPENSTEERDEMO_H
	#include "T3D/OpenSteer/include/OpenSteerDemo.h"
#endif
#ifndef _NAV_CLIENT_H_
	#include "T3D/OpenSteer/Plugins/NavClient.h"
#endif

using namespace Torque;

Point3F gPlanePos = Point3F(0,0,0);
MatrixF gPlaneMat = MatrixF(true);

bool PhysicsShape::smNoCorrections = false;
bool PhysicsShape::smNoSmoothing = false;

ImplementEnumType( PhysicsSimType,
   "How to handle the physics simulation with the clients and server.\n"
   "@ingroup Physics\n\n")
   { PhysicsShapeData::SimType_ClientOnly,    "ClientOnly", "Only handle physics on the client.\n"  },
   { PhysicsShapeData::SimType_ServerOnly,   "ServerOnly", "Only handle physics on the server.\n" },
   { PhysicsShapeData::SimType_ClientServer,  "ClientServer", "Handle physics on both the client and server.\n"   }
EndImplementEnumType;


IMPLEMENT_CO_DATABLOCK_V1( PhysicsShapeData );

ConsoleDocClass( PhysicsShapeData,
   
   "@brief Defines the properties of a PhysicsShape.\n\n"
   "@see PhysicsShape.\n"   
   "@ingroup Physics"
);

PhysicsShapeData::PhysicsShapeData()
   :  shapeName( NULL ),
      mass( 1.0f ),
      dynamicFriction( 0.0f ),
      staticFriction( 0.0f ),
      restitution( 0.0f ),
      linearDamping( 0.0f ),
      angularDamping( 0.0f ),
      linearSleepThreshold( 0.1f ),
      angularSleepThreshold( 0.1f ),
      waterDampingScale( 1.0f ),
      buoyancyDensity( 0.0f ),
      ccdEnabled( false ),
      simType( SimType_ClientServer )      
{
}

PhysicsShapeData::~PhysicsShapeData()
{
}

void PhysicsShapeData::initPersistFields()
{
   Parent::initPersistFields();

   addGroup("Media");

      addField( "shapeName", TypeShapeFilename, Offset( shapeName, PhysicsShapeData ),
         "@brief Path to the .DAE or .DTS file to use for this shape.\n\n"
         "Compatable with Live-Asset Reloading. ");

      addField( "debris", TYPEID< SimObjectRef<PhysicsDebrisData> >(), Offset( debris, PhysicsShapeData ),
         "@brief Name of a PhysicsDebrisData to spawn when this shape is destroyed (optional)." );

      addField( "explosion", TYPEID< SimObjectRef<ExplosionData> >(), Offset( explosion, PhysicsShapeData ),
         "@brief Name of an ExplosionData to spawn when this shape is destroyed (optional)." );

      addField( "destroyedShape", TYPEID< SimObjectRef<PhysicsShapeData> >(), Offset( destroyedShape, PhysicsShapeData ),
         "@brief Name of a PhysicsShapeData to spawn when this shape is destroyed (optional)." );

   endGroup("Media");

   addGroup( "Physics" );

      addField( "ccd", TypeBool, Offset( ccdEnabled, PhysicsShapeData ),
         "@brief Enable CCD support for this body.\n\n"
         "Continuous Collision Detection support for fast moving objects.\n"
         "@note Currently only supported in the PhysX 3 physics plugin.");

      addField( "mass", TypeF32, Offset( mass, PhysicsShapeData ),
         "@brief Value representing the mass of the shape.\n\n"
         "A shape's mass influences the magnitude of any force exerted on it. "
         "For example, a PhysicsShape with a large mass requires a much larger force to move than "
         "the same shape with a smaller mass.\n"
         "@note A mass of zero will create a kinematic shape while anything greater will create a dynamic shape.");

      addField( "friction", TypeF32, Offset( dynamicFriction, PhysicsShapeData ),
         "@brief Coefficient of kinetic %friction to be applied to the shape.\n\n" 
         "Kinetic %friction reduces the velocity of a moving object while it is in contact with a surface. "
         "A higher coefficient will result in a larger velocity reduction. "
         "A shape's friction should be lower than it's staticFriction, but larger than 0.\n\n"
         "@note This value is only applied while an object is in motion. For an object starting at rest, see PhysicsShape::staticFriction");

      addField( "staticFriction", TypeF32, Offset( staticFriction, PhysicsShapeData ),
         "@brief Coefficient of static %friction to be applied to the shape.\n\n" 
         "Static %friction determines the force needed to start moving an at-rest object in contact with a surface. "
         "If the force applied onto shape cannot overcome the force of static %friction, the shape will remain at rest. "
         "A larger coefficient will require a larger force to start motion. "
         "This value should be larger than zero and the physicsShape's friction.\n\n"
         "@note This value is only applied while an object is at rest. For an object in motion, see PhysicsShape::friction");

      addField( "restitution", TypeF32, Offset( restitution, PhysicsShapeData ),
         "@brief Coeffecient of a bounce applied to the shape in response to a collision.\n\n"
         "Restitution is a ratio of a shape's velocity before and after a collision. "
         "A value of 0 will zero out a shape's post-collision velocity, making it stop on contact. "
         "Larger values will remove less velocity after a collision, making it \'bounce\' with a greater force. "
         "Normal %restitution values range between 0 and 1.0."
         "@note Values near or equaling 1.0 are likely to cause undesirable results in the physics simulation."
         " Because of this it is reccomended to avoid values close to 1.0");

      addField( "linearDamping", TypeF32, Offset( linearDamping, PhysicsShapeData ),
         "@brief Value that reduces an object's linear velocity over time.\n\n"
         "Larger values will cause velocity to decay quicker.\n\n" );

      addField( "angularDamping", TypeF32, Offset( angularDamping, PhysicsShapeData ),
         "@brief Value that reduces an object's rotational velocity over time.\n\n"
         "Larger values will cause velocity to decay quicker.\n\n" );

      addField( "linearSleepThreshold", TypeF32, Offset( linearSleepThreshold, PhysicsShapeData ),
         "@brief Minimum linear velocity before the shape can be put to sleep.\n\n"
         "This should be a positive value. Shapes put to sleep will not be simulated in order to save system resources.\n\n"
         "@note The shape must be dynamic.");

      addField( "angularSleepThreshold", TypeF32, Offset( angularSleepThreshold, PhysicsShapeData ),
         "@brief Minimum rotational velocity before the shape can be put to sleep.\n\n"
         "This should be a positive value. Shapes put to sleep will not be simulated in order to save system resources.\n\n"
         "@note The shape must be dynamic.");

      addField( "waterDampingScale", TypeF32, Offset( waterDampingScale, PhysicsShapeData ),
         "@brief Scale to apply to linear and angular dampening while underwater.\n\n "
         "Used with the waterViscosity of the  "
         "@see angularDamping linearDamping" );

      addField( "buoyancyDensity", TypeF32, Offset( buoyancyDensity, PhysicsShapeData ),
         "@brief The density of the shape for calculating buoyant forces.\n\n"
         "The result of the calculated buoyancy is relative to the density of the WaterObject the PhysicsShape is within.\n\n"
         "@see WaterObject::density");
	        
	  //addField( "isArticulated", TypeBool, Offset( isArticulated, PhysicsShapeData ),
      //   "@brief If true, body maintains arrays of PhysicsBody and PhysicsJoint objects,"
      //   "instead of just one PhysicsBody.\n\n");
	  
      addField( "shapeID", TypeS32, Offset( shapeID, PhysicsShapeData ),
         "@brief The database ID of the physicsShape, to find all the bodypart and joint data.\n\n");
	  
   endGroup( "Physics" );   

   addGroup( "Networking" );

      //addField( "simType", TYPEID< PhysicsShapeData::SimType >(), Offset( simType, PhysicsShapeData ),
      //  "@brief Controls whether this shape is simulated on the server, client, or both physics simulations.\n\n" );
	  addField( "simType", TypeS32, Offset( simType, PhysicsShapeData ),
         "@brief Controls whether this shape is simulated on the server, client, or both physics simulations.\n\n" );

   endGroup( "Networking" );   
}

void PhysicsShapeData::packData( BitStream *stream )
{ 
   Parent::packData( stream );

   stream->writeString( shapeName );

   stream->write( mass );
   stream->write( dynamicFriction );
   stream->write( staticFriction );
   stream->write( restitution );
   stream->write( linearDamping );
   stream->write( angularDamping );
   stream->write( linearSleepThreshold );
   stream->write( angularSleepThreshold );
   stream->write( waterDampingScale );
   stream->write( buoyancyDensity );
   stream->write( ccdEnabled );
   //stream->write( isArticulated );
   stream->write( shapeID );

   stream->writeInt( simType, SimType_Bits );

   stream->writeRangedU32( debris ? debris->getId() : 0, 0, DataBlockObjectIdLast );
   stream->writeRangedU32( explosion ? explosion->getId() : 0, 0, DataBlockObjectIdLast );
   stream->writeRangedU32( destroyedShape ? destroyedShape->getId() : 0, 0, DataBlockObjectIdLast );
}

void PhysicsShapeData::unpackData( BitStream *stream )
{
   Parent::unpackData(stream);

   shapeName = stream->readSTString();

   stream->read( &mass );
   stream->read( &dynamicFriction );
   stream->read( &staticFriction );
   stream->read( &restitution );
   stream->read( &linearDamping );
   stream->read( &angularDamping );
   stream->read( &linearSleepThreshold );
   stream->read( &angularSleepThreshold );
   stream->read( &waterDampingScale );
   stream->read( &buoyancyDensity );
   stream->read( &ccdEnabled );
   //stream->read( &isArticulated );
   stream->read( &shapeID );

   S32 origSimType = stream->readInt( SimType_Bits );
   simType = (SimType)origSimType;

   debris = stream->readRangedU32( 0, DataBlockObjectIdLast );
   explosion = stream->readRangedU32( 0, DataBlockObjectIdLast );   
   destroyedShape = stream->readRangedU32( 0, DataBlockObjectIdLast );
}

bool PhysicsShapeData::onAdd()
{
   if ( !Parent::onAdd() )
      return false;

   ResourceManager::get().getChangedSignal().notify( this, &PhysicsShapeData::_onResourceChanged );
   return true;
}

void PhysicsShapeData::onRemove()
{
   ResourceManager::get().getChangedSignal().remove( this, &PhysicsShapeData::_onResourceChanged );
   Parent::onRemove();
}

void PhysicsShapeData::_onResourceChanged( const Torque::Path &path )
{
	if ( path != Torque::Path( shapeName ) )
      return;

   // Reload the changed shape.
   Resource<TSShape> reloadShape;
   PhysicsCollisionRef reloadcolShape;

   reloadShape = ResourceManager::get().load( shapeName );
   if ( !bool(reloadShape) )
   {
      Con::warnf( ConsoleLogEntry::General, "PhysicsShapeData::_onResourceChanged: Could not reload %s.", path.getFileName().c_str() );
      return;
   }

   // Reload the collision shape.
   reloadcolShape = shape->buildColShape( false, Point3F::One );

   if ( bool(reloadShape) &&  bool(colShape))
   {
      shape = reloadShape;
      colShape = reloadcolShape;
   }

   mReloadSignal.trigger();
}

bool PhysicsShapeData::preload( bool server, String &errorBuffer )
{
   if ( !Parent::preload( server, errorBuffer ) )
      return false;

   // If we don't have a physics plugin active then
   // we have to fail completely.
   if ( !PHYSICSMGR )
   {
      errorBuffer = "PhysicsShapeData::preload - No physics plugin is active!";
      return false;
   }

   if ( !shapeName || !shapeName[0] ) 
   {
      errorBuffer = "PhysicsShapeData::preload - No shape name defined.";
      return false;
   }

   // Load the shape.
   shape = ResourceManager::get().load( shapeName );
   if ( bool(shape) == false )
   {
      errorBuffer = String::ToString( "PhysicsShapeData::preload - Unable to load shape '%s'.", shapeName );
      return false;
   }

   // Prepare the shared physics collision shape.
   if ( !colShape )
   {
      colShape = shape->buildColShape( false, Point3F::One );

      // If we got here and didn't get a collision shape then
      // we need to fail... can't have a shape without collision.
      if ( !colShape )
      {
         errorBuffer = String::ToString( "PhysicsShapeData::preload - No collision found for shape '%s'.", shapeName );
         return false;
      }
   }   

   // My convex decomposition test
   /*
   // Get the verts and triangles for the first visible detail.
   ConcretePolyList polyList;
   polyList.setTransform( &MatrixF::Identity, Point3F::One );
   TSShapeInstance shapeInst( shape, false );
   shapeInst.animate(0);
   if ( !shapeInst.buildPolyList( &polyList, 0 ) )
      return false;

   // Gah... Ratcliff's lib works on doubles... why, oh why?
   Vector<F64> doubleVerts;
   doubleVerts.setSize( polyList.mVertexList.size() * 3 );
   for ( U32 i=0; i < polyList.mVertexList.size(); i++ )
   {
      doubleVerts[ ( i * 3 ) + 0 ] = (F64)polyList.mVertexList[i].x;
      doubleVerts[ ( i * 3 ) + 1 ] = (F64)polyList.mVertexList[i].y;
      doubleVerts[ ( i * 3 ) + 2 ] = (F64)polyList.mVertexList[i].z;
   }

   using namespace ConvexDecomposition;

   class ConvexBuilder : public ConvexDecompInterface
   {
   public:

      ConvexBuilder() { }

      ~ConvexBuilder() 
      {
         for ( U32 i=0; i < mHulls.size(); i++ )
            delete mHulls[i];
      }

      virtual void ConvexDecompResult( ConvexResult &result )
      {
         FConvexResult *hull = new FConvexResult( result );
         mHulls.push_back( hull );
      }

      Vector<FConvexResult*> mHulls;
   };

 	DecompDesc d;
   d.mVcount       =	polyList.mVertexList.size();
   d.mVertices     = doubleVerts.address();
   d.mTcount       = polyList.mIndexList.size() / 3;
   d.mIndices      = polyList.mIndexList.address();
   d.mDepth        = 3;
   d.mCpercent     = 20.0f;
   d.mPpercent     = 30.0f;
   d.mMaxVertices  = 32;
   d.mSkinWidth    = 0.05f; // Need to expose this!

   ConvexBuilder builder;
   d.mCallback = &builder;
 
   if ( performConvexDecomposition( d ) < 1 || builder.mHulls.empty() )
      return false;

   // Add all the convex hull results into the collision shape.
   colShape = PHYSICSMGR->createCollision();
   for ( U32 i=0; i < builder.mHulls.size(); i++ )
      colShape->addConvex( (const Point3F*)builder.mHulls[i]->mHullVertices, 
                           builder.mHulls[i]->mHullVcount,
                           MatrixF::Identity );
   */

   return true;
}


IMPLEMENT_CO_NETOBJECT_V1(PhysicsShape);

ConsoleDocClass( PhysicsShape,
   
   "@brief Represents a destructible physical object simulated through the plugin system.\n\n"
   "@see PhysicsShapeData.\n"   
   "@ingroup Physics"
);

PhysicsShape::PhysicsShape()
   :  mPhysicsRep( NULL ),
      mDataBlock( NULL ),
      mWorld( NULL ),
      mJoint( NULL ),
      mShapeInstance( NULL ),
      mResetPos( MatrixF::Identity ),
      mDestroyed( false ),
      mPlayAmbient( false ),
      mAmbientThread( NULL ),
      mAmbientSeq( -1 ),
      mCurrentSeq( -1 ),
		mContactBody( -1 ),
		mHasGravity( true ),
		mIsDynamic( true ),
		//mIsArticulated( false ),
		mIsRecording( false ),
		mUseDataSource( false ),
		mIsGroundMoving( false ),
		mSaveTranslations( false ),
		mCurrentTick( 0 ),
		mLastThreadPos( 0 ),
		mStartPos( Point3F(0,0,0) ),//maybe use mResetPos for this?
		mCurrentForce( Point3F(0,0,0) ),
		mRecordSampleRate( 1 ),
		mRecordCount( 0 ),
		mShapeID( -1 ),
		mSceneID( -1 ),
		mSceneShapeID( -1 ),
		mNavMesh( NULL ),
		mNavPath( NULL ),
		mVehicle( NULL ),
		mPedId( 0 ),
		mWalkSpeed( 0.05 ),
		mJogSpeed( 1.5 ),
		mRunSpeed( 3.0 ),
		mSprintSpeed( 4.0 )
{
   mNetFlags.set( Ghostable | ScopeAlways );
   mTypeMask |= DynamicShapeObjectType;
   mLastGroundTrans = Point3F::Zero;
   mLastGroundRot = QuatF::Identity;
	mDataSource = NULL;
   mLinkTypes = LinkData(AllFlags);
}

PhysicsShape::~PhysicsShape()
{
	Con::printf("calling physicsShape destructor, sceneShapeID %d",mSceneShapeID);
}

void PhysicsShape::consoleInit()
{
   Con::addVariable( "$PhysicsShape::noCorrections", TypeBool, &PhysicsShape::smNoCorrections,
     "@brief Determines if the shape will recieve corrections from the server or "
     "will instead be allowed to diverge.\n\n"
     "In the event that the client and server object positions/orientations "
     "differ and if this variable is true, the server will attempt to \'correct\' "
     "the client object to keep it in sync. Otherwise, client and server objects may fall out of sync.\n\n");

   Con::addVariable( "$PhysicsShape::noSmoothing", TypeBool, &PhysicsShape::smNoSmoothing,
     "@brief Determines if client-side shapes will attempt to smoothly transition to "
     "their new position after reciving a correction.\n\n"
     "If true, shapes will immediately render at the position they are corrected to.\n\n");
	
   Parent::consoleInit();   
}

void PhysicsShape::initPersistFields()
{   
   addGroup( "PhysicsShape" );

      addField( "playAmbient", TypeBool, Offset( mPlayAmbient, PhysicsShape ),
            "@brief Enables or disables playing of an ambient animation upon loading the shape.\n\n"
            "@note The ambient animation must be named \"ambient\"." );
   
	  addField( "hasGravity", TypeBool, Offset( mHasGravity, PhysicsShape ),
		  "@brief Turns off gravity for this object if set to false.\n\n");

	  addField( "isDynamic", TypeBool, Offset( mIsDynamic, PhysicsShape ),
		  "@brief Turns object kinematic if set to false.\n\n");
	  
	  addField( "sceneShapeID", TypeS32, Offset( mSceneShapeID, PhysicsShape ),
		  "@brief Database ID for this sceneShape.\n\n");

	  addField( "sceneID", TypeS32, Offset( mSceneID, PhysicsShape ),
		  "@brief Database ID for this scene.\n\n");

   endGroup( "PhysicsShape" );

   Parent::initPersistFields();   

   //removeField( "scale" );//??? openSimEarth here, commenting this out... Why did they do this? 
}

void PhysicsShape::inspectPostApply()
{
   Parent::inspectPostApply();

   setMaskBits( InitialUpdateMask );
}

U32 PhysicsShape::packUpdate( NetConnection *con, U32 mask, BitStream *stream )
{
   U32 retMask = Parent::packUpdate( con, mask, stream );

   if ( stream->writeFlag( mask & InitialUpdateMask ) )
   {
      stream->writeAffineTransform( getTransform() );
      stream->writeFlag( mPlayAmbient );
      stream->writeFlag( mHasGravity );
      stream->writeFlag( mIsDynamic );

      stream->writeFlag( mDestroyed );

      return retMask;
   }

   // If we got here its not an initial update.  So only send
   // the least amount of data possible.

   if ( stream->writeFlag( mask & StateMask ) )
   {
      // This will encode the position relative to the control
      // object position.  
      //
      // This will compress the position to as little as 6.25
      // bytes if the position is within about 30 meters of the
      // control object.
      //
      // Worst case its a full 12 bytes + 2 bits if the position
      // is more than 500 meters from the control object.
      //
      stream->writeCompressedPoint( mState.position );

      // Use only 3.5 bytes to send the orientation.
      stream->writeQuat( mState.orientation, 9 );

      // If the server object has been set to sleep then
      // we don't need to send any velocity.
      if ( !stream->writeFlag( mState.sleeping ) )
      {
         // This gives me ~0.015f resolution in velocity magnitude
         // while only costing me 1 bit of the velocity is zero length,
         // <5 bytes in normal cases, and <8 bytes if the velocity is
         // greater than 1000.
         AssertWarn( mState.linVelocity.len() < 1000.0f, 
            "PhysicsShape::packUpdate - The linVelocity is out of range!" );
         stream->writeVector( mState.linVelocity, 1000.0f, 16, 9 );

         // For angular velocity we get < 0.01f resolution in magnitude
         // with the most common case being under 4 bytes.
         AssertWarn( mState.angVelocity.len() < 10.0f, 
            "PhysicsShape::packUpdate - The angVelocity is out of range!" );
         stream->writeVector( mState.angVelocity, 10.0f, 10, 9 );
      }
   }

   if ( stream->writeFlag( mask & DamageMask ) )
      stream->writeFlag( mDestroyed );

   return retMask;
}   

void PhysicsShape::unpackUpdate( NetConnection *con, BitStream *stream )
{
   Parent::unpackUpdate( con, stream );

   if ( stream->readFlag() ) // InitialUpdateMask
	{
		MatrixF mat;
		stream->readAffineTransform( &mat );
		setTransform( mat );
		mPlayAmbient = stream->readFlag();
		mHasGravity = stream->readFlag();
		mIsDynamic = stream->readFlag();

		if ( isProperlyAdded() )
			_initAmbient();

      if ( stream->readFlag() )
      {
         if ( isProperlyAdded() )
         {
            // Destroy immediately if we've already been added
            // to the scene.
            destroy();
         }
         else
         {
            // Indicate the shape should be destroyed when the
            // shape is added.
            mDestroyed = true;
         }
      }

      return;
   }

   if ( stream->readFlag() ) // StateMask
   {
      PhysicsState state;
      
      // Read the encoded and compressed position... commonly only 6.25 bytes.
      stream->readCompressedPoint( &state.position );

      // Read the compressed quaternion... 3.5 bytes.
      stream->readQuat( &state.orientation, 9 );

      state.sleeping = stream->readFlag();
      if ( !state.sleeping )
      {
         stream->readVector( &state.linVelocity, 1000.0f, 16, 9 );
         stream->readVector( &state.angVelocity, 10.0f, 10, 9 );
      }

      if ( !smNoCorrections && mPhysicsRep && mPhysicsRep->isDynamic() && !mDestroyed )
      {
         // Set the new state on the physics object immediately.

		  mPhysicsRep->applyCorrection( state.getTransform() );

		  mPhysicsRep->setSleeping( state.sleeping );
		  if ( !state.sleeping )
		  {
			  mPhysicsRep->setLinVelocity( state.linVelocity ); 
			  mPhysicsRep->setAngVelocity( state.angVelocity ); 
		  }

		  mPhysicsRep->getState( &mState );
      }

      // If there is no physics object then just set the
      // new state... the tick will take care of the 
      // interpolation and extrapolation.
      if ( !mPhysicsRep || !mPhysicsRep->isDynamic() )
         mState = state;
   }

   if ( stream->readFlag() ) // DamageMask
   {
      if ( stream->readFlag() )
         destroy();
      else
         restore();
   }
}

bool PhysicsShape::onAdd()
{
   if ( !Parent::onAdd() )
      return false;

   // If we don't have a physics plugin active then
   // we have to fail completely.
   if ( !PHYSICSMGR )
   {
      Con::errorf( "PhysicsShape::onAdd - No physics plugin is active!" );
      return false;
   }

   // 
   if ( !mPhysicsRep && !_createShape() )
   {
      Con::errorf( "PhysicsShape::onAdd() - Shape creation failed!" );
      return false;
   }

   // The reset position is the transform on the server
   // at creation time... its not used on the client.
   if ( isServerObject() )
   {
      storeRestorePos();
      PhysicsPlugin::getPhysicsResetSignal().notify( this, &PhysicsShape::_onPhysicsReset );
   }

   // Register for the resource change signal.
   //ResourceManager::get().getChangedSignal().notify( this, &PhysicsShape::_onResourceChanged );

   // Only add server objects and non-destroyed client objects to the scene.
   if ( isServerObject() || !mDestroyed)
      addToScene();
   
   if ( isClientObject() && mDestroyed )
   {
      // Disable all simulation of the body... no collision or dynamics.
      if ( mPhysicsRep )
         mPhysicsRep->setSimulationEnabled( false );

		if (mPhysicsBodies.size()>1)//formerly mIsArticulated
			for (U32 i=1;i<mPhysicsBodies.size();i++)
				mPhysicsBodies[i]->setSimulationEnabled( false );
		  
      // Stop doing tick processing for this SceneObject.
      setProcessTick( false );
   }

   return true;
}

void PhysicsShape::onRemove()
{
   removeFromScene();

   SAFE_DELETE( mPhysicsRep );
   SAFE_DELETE( mShapeInstance );
   mAmbientThread = NULL;
   mAmbientSeq = -1;
   mCurrentSeq = -1;
   mWorld = NULL;

   if ( isServerObject() )
   {
      PhysicsPlugin::getPhysicsResetSignal().remove( this, &PhysicsShape::_onPhysicsReset );

      if ( mDestroyedShape )
		  mDestroyedShape->deleteObject();
   }

   // Remove the resource change signal.
   //ResourceManager::get().getChangedSignal().remove( this, &PhysicsShape::_onResourceChanged );

   Parent::onRemove();
}

bool PhysicsShape::onNewDataBlock( GameBaseData *dptr, bool reload )
{
   mDataBlock = static_cast<PhysicsShapeData*>(dptr);
   if (!mDataBlock || !Parent::onNewDataBlock( dptr, reload ) )
      return false;

   if ( !isProperlyAdded() )
      return true;

   // If we don't have a physics plugin active then
   // we have to fail completely.
   if ( !PHYSICSMGR )
   {
      Con::errorf( "PhysicsShape::onNewDataBlock - No physics plugin is active!" );
      return false;
   }

   if ( !_createShape() )
   {
      Con::errorf( "PhysicsShape::onNewDataBlock() - Shape creation failed!" );
      return false;
   }

   return true;
}

bool PhysicsShape::_createShape()
{
   SAFE_DELETE( mPhysicsRep );
   SAFE_DELETE( mShapeInstance );
   mAmbientThread = NULL;
   mWorld = NULL;
   mAmbientSeq = -1;
   mCurrentSeq = -1;

   if ( !mDataBlock )
      return false;

   // Set the world box.
   mObjBox = mDataBlock->shape->bounds;
   resetWorldBox();

   // If this is the server and it's a client only simulation
   // object then disable our tick... the server doesn't do 
   // any work for this shape.
   if (  isServerObject() && 
         mDataBlock->simType == PhysicsShapeData::SimType_ClientOnly )
   {
      setProcessTick( false );
      return true;
   }

   // Create the shape instance.   
   mShape = mDataBlock->shape;
   mShapeInstance = new TSShapeInstance( mShape, isClientObject() );

   if ( isClientObject() )
   {
      _initAmbient();
   }
	
	SQLiteObject *kSQL = PHYSICSMGR->mSQL;//openSimEarth
	char part_query[512],datablock_query[512],joint_query[512];
	S32 result,result2,file_id;
	sqlite_resultset *resultSet,*resultSet2;
	S32 shape_id = mDataBlock->shapeID;

   // If the shape has a mass then it's dynamic... else
   // its a kinematic shape. [Edit: now allowing mIsDynamic to be set from console at 
   // object creation time, but also turn it off if mass is less than or equal to zero.
   //
   // While a kinematic is less optimal than a static body
   // it allows for us to enable/disable collision and having
   // all dynamic actors react correctly... waking up.
   // 
   //const bool isDynamic = mDataBlock->mass > 0.0f;
 
   MatrixF shapeTransform = getTransform();
   Point3F shapePos = shapeTransform.getPosition();
   shapeTransform.setPosition(Point3F(0,0,0));
   shapeTransform.inverse();

   if (mDataBlock->mass <= 0.0f) mIsDynamic = false;

   //mIsArticulated = mDataBlock->isArticulated;//Removing this, in favor of all shapes having an mPhysicsBodies[0]=mPhysicsRep.

   // If we aren't dynamic we don't need to tick. //openSimEarth - more complicated than this. Just go to processTick
   setProcessTick( 1 );//mIsDynamic || mPlayAmbient );//|| mIsArticulated );//and decide what to do there.

   // If this is the client and we're a server only object then
   // we don't need any physics representation... we're done.
   if (  isClientObject() && 
         mDataBlock->simType == PhysicsShapeData::SimType_ServerOnly )
      return true;

   mWorld = PHYSICSMGR->getWorld( isServerObject() ? "server" : "client" );

   //mWorld->lockScene();

   U32 bodyFlags = mIsDynamic ? 0 : PhysicsBody::BF_KINEMATIC; 

   if(mDataBlock->ccdEnabled)
      bodyFlags |= PhysicsBody::BF_CCD;

   if(mHasGravity)
      bodyFlags |= PhysicsBody::BF_GRAVITY;
   else
	  bodyFlags &= ~PhysicsBody::BF_GRAVITY;
   

	//NOW: instead of using mIsArticulated as our main switch, we need to move the parts query up here and let the results decide.
	S32  shapeBodyParts = 0;
	if (shape_id>0)
	{
		sprintf(part_query,"SELECT * FROM physicsShapePart WHERE physicsShape_id=%d;",shape_id);
		result = kSQL->ExecuteSQL(part_query);
		if (result>0)
		{
			resultSet = kSQL->GetResultSet(result);
			shapeBodyParts = resultSet->iNumRows;
		}
	}

   if (isServerObject())//Now, we're just gonna say for the first pass at least, that servers don't worry about multiple bodyparts.
   {//This could bite us later, but for initial purposes of ragdolls and other client side "eye candy" physics it should work fine.
		PhysicsCollision* colShape;
	   //if (shapeBodyParts==0)//And now that we mention it, I'm not going to worry about defining server side physics body shapes
			//in the database either. Server side physics can wait. Although they will be important the second we go multiplayer and
			//start throwing rocks and shooting arrows at each other.
		//{//(Simple case, no bodyparts in the database, so just assign an arbitrary little box to hold us down for now.)
		mPhysicsRep = PHYSICSMGR->createBody();//Later this could be made smarter, but not needed at the moment.	
		colShape = PHYSICSMGR->createCollision();//At least we can check for dataBlock->colShape though.
		MatrixF localTrans = MatrixF::Identity;
		colShape->addBox(Point3F(0.03,0.02,0.015),localTrans);//This box could be replaced with a capsule or a character controller.
		//}
	   //Oops, just realized this would break everything not articulated, or at least replace it with a colshape
	   if (bool(mDataBlock->colShape))
	   {
			mPhysicsRep->init(   mDataBlock->colShape, 
							mDataBlock->mass, 
							bodyFlags,  
							this, 
							mWorld );
		} else {
		   mPhysicsRep->init(   colShape, 
							mDataBlock->mass, 
							bodyFlags,  
							this, 
							mWorld );	   
	   }

	   mPhysicsRep->setMaterial( mDataBlock->restitution, mDataBlock->dynamicFriction, mDataBlock->staticFriction );

	   
	   if ( mIsDynamic )
	   {
		   mPhysicsRep->setDamping( mDataBlock->linearDamping, mDataBlock->angularDamping );
		   mPhysicsRep->setSleepThreshold( mDataBlock->linearSleepThreshold, mDataBlock->angularSleepThreshold );
	   }

	   MatrixF kTrans = getTransform();
	   EulerF kEuler = kTrans.toEuler();
	   
	   mPhysicsRep->setTransform( getTransform() );

		mPhysicsBodies.increment();
		mPhysicsBodies.last() = mPhysicsRep;

   } else { //We're on the client, so now it gets real.

	   //sprintf(part_query,"SELECT * FROM physicsShapePart WHERE physicsShape_id=%d;",shape_id);
	   //result = kSQL->ExecuteSQL(part_query);
	   //if (result==0)
		  // return NULL;

	   //resultSet = kSQL->GetResultSet(result);
	   //if (resultSet->iNumRows<=0)
		  // return NULL;
		if (shapeBodyParts==0)
		{
			//just do the same thing as above with server, make a placeholder little box colshape.
			mPhysicsRep = PHYSICSMGR->createBody();//Later this could be made smarter, but not needed at the moment.	
			PhysicsCollision* colShape = PHYSICSMGR->createCollision();//At least we can check for dataBlock->colShape though.
			MatrixF localTrans = MatrixF::Identity;
			colShape->addBox(Point3F(0.03,0.02,0.015),localTrans);//This box could be replaced with a capsule or a character controller.
			mPhysicsRep->init(   colShape, 
							mDataBlock->mass, 
							bodyFlags,  
							this, 
							mWorld );	
			mPhysicsRep->setMaterial( mDataBlock->restitution, mDataBlock->dynamicFriction, mDataBlock->staticFriction );

			if ( mIsDynamic )
			{
				mPhysicsRep->setDamping( mDataBlock->linearDamping, mDataBlock->angularDamping );
				mPhysicsRep->setSleepThreshold( mDataBlock->linearSleepThreshold, mDataBlock->angularSleepThreshold );
			}

			MatrixF kTrans = getTransform();
			EulerF kEuler = kTrans.toEuler();

			mPhysicsRep->setTransform( getTransform() );

			mPhysicsBodies.increment();
			mPhysicsBodies.last() = mPhysicsRep;
		} 
		else //else we have at least one bodypart in the database.
		{//Using resultSet from above, already checked for numRows > 0.
			for (U32 i=0;i<resultSet->iNumRows;i++)
			{
				S32 j=0;
				char baseNode[255],childNode[255];

				//////////////////////////////////////////
				// Part Data
				physicsPartData* PD = new physicsPartData;
				S32 ID = dAtoi(resultSet->vRows[i]->vColumnValues[j++]);
				j++;//We already know shapeID			
				PD->jointID = dAtoi(resultSet->vRows[i]->vColumnValues[j++]);
				j++;//We don't need to keep name.
				sprintf(baseNode,resultSet->vRows[i]->vColumnValues[j++]);
				PD->baseNode = mShape->findNode(baseNode);
				sprintf(childNode,resultSet->vRows[i]->vColumnValues[j++]);
				PD->childNode = mShape->findNode(childNode);
				PD->shapeType = dAtoi(resultSet->vRows[i]->vColumnValues[j++]);
				PD->dimensions.x = dAtof(resultSet->vRows[i]->vColumnValues[j++]) * mObjScale.x;
				PD->dimensions.y = dAtof(resultSet->vRows[i]->vColumnValues[j++]) * mObjScale.y;
				PD->dimensions.z = dAtof(resultSet->vRows[i]->vColumnValues[j++]) * mObjScale.z;
				PD->orientation.x = mDegToRad( dAtof(resultSet->vRows[i]->vColumnValues[j++]) );
				PD->orientation.y = mDegToRad( dAtof(resultSet->vRows[i]->vColumnValues[j++]) );
				PD->orientation.z = mDegToRad( dAtof(resultSet->vRows[i]->vColumnValues[j++]) );
				PD->offset.x = dAtof(resultSet->vRows[i]->vColumnValues[j++]) * mObjScale.x;
				PD->offset.y = dAtof(resultSet->vRows[i]->vColumnValues[j++]) * mObjScale.y;
				PD->offset.z = dAtof(resultSet->vRows[i]->vColumnValues[j++]) * mObjScale.z;
				PD->damageMultiplier = dAtof(resultSet->vRows[i]->vColumnValues[j++]);
				PD->isInflictor = dAtob(resultSet->vRows[i]->vColumnValues[j++]);
				PD->density = dAtof(resultSet->vRows[i]->vColumnValues[j++]);
				PD->isKinematic = dAtob(resultSet->vRows[i]->vColumnValues[j++]);
				PD->isNoGravity = dAtob(resultSet->vRows[i]->vColumnValues[j++]);
				PD->childVerts = dAtoi(resultSet->vRows[i]->vColumnValues[j++]);
				PD->parentVerts = dAtoi(resultSet->vRows[i]->vColumnValues[j++]);
				PD->farVerts = dAtoi(resultSet->vRows[i]->vColumnValues[j++]);
				PD->weightThreshold = dAtof(resultSet->vRows[i]->vColumnValues[j++]);
				PD->ragdollThreshold = dAtof(resultSet->vRows[i]->vColumnValues[j++]);
				PD->bodypartChain = dAtoi(resultSet->vRows[i]->vColumnValues[j++]);
				PD->mass = dAtof(resultSet->vRows[i]->vColumnValues[j++]);
				PD->inflictMultiplier = dAtof(resultSet->vRows[i]->vColumnValues[j++]);
				PD->jointRots.x = dAtof(resultSet->vRows[i]->vColumnValues[j++]);
				PD->jointRots.y = dAtof(resultSet->vRows[i]->vColumnValues[j++]);
				PD->jointRots.z = dAtof(resultSet->vRows[i]->vColumnValues[j++]);
				PD->jointRots2.x = dAtof(resultSet->vRows[i]->vColumnValues[j++]);
				PD->jointRots2.y = dAtof(resultSet->vRows[i]->vColumnValues[j++]);
				PD->jointRots2.z = dAtof(resultSet->vRows[i]->vColumnValues[j++]);

				//////////////////////////////////////////
				// Joint Data
				bool hasJoint = true;
				sprintf(joint_query,"SELECT * FROM px3Joint WHERE id=%d;",PD->jointID);
				physicsJointData* JD = new physicsJointData();
				result2 = kSQL->ExecuteSQL(joint_query);
				if (result2==0)
					hasJoint = false;
				if (hasJoint)
					resultSet2 = kSQL->GetResultSet(result2);
				if (resultSet2->iNumRows!=1)
					hasJoint = false;
				kSQL->ClearResultSet(result2);

				//////////////////////////////////////////
				// Create Physics Body
				PhysicsBody *partBody;
				if (mPhysicsBodies.size()==0)
				{
					mPhysicsRep = PHYSICSMGR->createBody();
					partBody = mPhysicsRep;
				} else {
					partBody = PHYSICSMGR->createBody();
				}
				PhysicsCollision* colShape;
				colShape = PHYSICSMGR->createCollision();

				MatrixF localTrans,globalTrans,shapeTrans;
				Point3F pos = PD->offset;
				QuatF orient = QuatF(EulerF(PD->orientation.x,PD->orientation.y,PD->orientation.z));
				orient.setMatrix(&localTrans);
				localTrans.setPosition(pos);

				shapeTrans = getTransform();//Main body transform.

				Point3F halfDim = PD->dimensions * 0.5;

				if (PD->shapeType==physicsShapeType::PHYS_SHAPE_BOX)
					colShape->addBox(halfDim,localTrans);
				else if (PD->shapeType==physicsShapeType::PHYS_SHAPE_CAPSULE)
					colShape->addCapsule(halfDim.x,halfDim.z,localTrans);
				else if (PD->shapeType==physicsShapeType::PHYS_SHAPE_SPHERE)
					colShape->addSphere(halfDim.x,localTrans);
				else if (PD->shapeType==physicsShapeType::PHYS_SHAPE_COLLISION)
				{
					//Do more here, steal from TSShape::_buildColShape().
					//colShape->addTriangleMesh(...)
					colShape->addBox(halfDim,localTrans);//(for now hold it down with a box)
				} 
				else if (PD->shapeType==physicsShapeType::PHYS_SHAPE_CONVEX)
				{			   
					//Do more here, find verts from the mesh using weightThreshold, childVerts, parentVerts, farVerts.
					//colShape->addConvex(...)
					colShape->addBox(halfDim,localTrans);//(for now hold it down with a box)
				}
				else if (PD->shapeType==physicsShapeType::PHYS_SHAPE_TRIMESH)
				{
					//Do more here, add the actual triangle mesh.
					//colShape->addTriangleMesh(...)
					colShape->addBox(halfDim,localTrans);//(for now hold it down with a box)
				}

				MatrixF finalTrans;
				MatrixF nodeTrans = mShapeInstance->mNodeTransforms[PD->baseNode];
				Point3F nodePos = nodeTrans.getPosition();
				nodePos *= mObjScale;
				nodeTrans.setPosition(nodePos);//Interesting, I would have thought my nodeTransforms would be scaled already, but no.
				finalTrans.mul(shapeTrans,nodeTrans);
				Point3F bodypartPos = finalTrans.getPosition();

				U32 bodyFlags;
				bodyFlags = (PD->isKinematic || !mIsDynamic) ?  PhysicsBody::BF_KINEMATIC : 0; 

				if(mDataBlock->ccdEnabled)
					bodyFlags |= PhysicsBody::BF_CCD;

				if(mHasGravity)
					bodyFlags |= PhysicsBody::BF_GRAVITY;
				else
					bodyFlags &= ~PhysicsBody::BF_GRAVITY;

				//MASS: get this from shapeParts in database, or better yet use density = 1.0 and make physx figure it out (?)
				partBody->init(   colShape, 
					PD->mass, 
					bodyFlags,  
					this, 
					mWorld );	 

				partBody->setMaterial( mDataBlock->restitution, mDataBlock->dynamicFriction, mDataBlock->staticFriction );

				if ( mIsDynamic )
				{
					partBody->setDamping( mDataBlock->linearDamping, mDataBlock->angularDamping );
					partBody->setSleepThreshold( mDataBlock->linearSleepThreshold, mDataBlock->angularSleepThreshold );
				}

				partBody->setTransform(finalTrans);

				partBody->setBodyIndex(mPhysicsBodies.size());
				partBody->setNodeIndex(PD->baseNode); 
				//Con::printf("Pushing back a physics body, index %d  node index %d finalPos %f %f %f",
				//						i,PD->baseNode,bodypartPos.x,bodypartPos.y,bodypartPos.z);
				mPhysicsBodies.push_back(partBody);

				mLastTrans.push_back(MatrixF(true));// During kinematic animation we use these

				mStates.increment();
				if (mIsDynamic) partBody->getState(&(mStates.last()));

				mBodyNodes.push_back(PD->baseNode);//Store node index for below, when we need to define parent objects.

				//////////////////////////////////////////
				// Create Joint
				if ((hasJoint)&&(PD->baseNode>0))
				{
					//Now, we need to find the parent physics body, which may involve skipping TSShape nodes in the hierarchy 
					//if they don't have physics.
					S32 parentNode,finalParentNode=-1;
					parentNode = mShape->nodes[PD->baseNode].parentIndex;
					int notForever=0;
					PhysicsBody *parentBody;
					while ((finalParentNode<0)&&(notForever<200))
					{
						for (int k=0;k<mBodyNodes.size();k++)
						{
							if (parentNode == mBodyNodes[k])
							{
								finalParentNode = parentNode;
								parentBody = mPhysicsBodies[k];
								mPhysicsBodies[mPhysicsBodies.size()-1]->setParentBodyIndex(k);
							}
						}
						parentNode = mShape->nodes[parentNode].parentIndex;
						mPhysicsBodies[mPhysicsBodies.size()-1]->setParentNodeIndex(parentNode);//Set parent node for body we just created.
						notForever++;//(Just to make sure we don't get bodypart order mixed up and wind up in forever loop.)
					}
					if (notForever==100) break;

					MatrixF partTrans,parentTrans;
					partBody->getTransform(&partTrans);
					parentBody->getTransform(&parentTrans);
					Point3F partPos = partTrans.getPosition();
					Point3F parentPos = parentTrans.getPosition();

					//Now, turns out all we need to do is make joint origin equal to the new part origin.
					PhysicsJoint *kJoint =  PHYSICSMGR->createJoint(partBody,parentBody,PD->jointID,partPos,PD->jointRots,PD->jointRots2,shapeTransform);
					if (kJoint)
					{
						mPhysicsJoints.push_back(kJoint);
					}  else Con::printf("Create joint failed, joint %d is null!",PD->jointID);
				} 
				delete PD;
			}
	   }
		   
	   kSQL->ClearResultSet(result);
	   
	   for (U32 i=0;i<mShape->nodes.size();i++)
	   {
		   mNodeBodies.increment();
		   mNodeBodies.last() = false;
	   }
	   for (U32 i=0;i<mShape->nodes.size();i++)
	   {
		   for (U32 j=0;j<mBodyNodes.size();j++)
		   {
			   if (mBodyNodes[j] == i)
				   mNodeBodies[i] = true;
		   }		   
	   }
	   setupOrderNodes();
	}

	//And, another bit of application-specific logic that should probably go somewhere else: 
	//  load range/offset of rudder, etc. Another thing that there's no point in doing on the server.
	if (isClientObject())
	{
		S32 vehicle_id = 0;
		char vehicle_query[512],offset_query[255];
		String rudderNodes,elevNodes,rightAilerNodes,leftAilerNodes,propNodes,rotorNodesA,rotorNodesB,
					tailRotorNodes;

		sprintf(vehicle_query,"SELECT vehicle_id FROM physicsShape WHERE id=%d;",shape_id);
		result = kSQL->ExecuteSQL(vehicle_query);
		resultSet = kSQL->GetResultSet(result);
		if (resultSet->iNumRows==1)
				vehicle_id = dAtoi(resultSet->vRows[0]->vColumnValues[0]);
		kSQL->ClearResultSet(result);

		if (vehicle_id>0)
		{
			sprintf(vehicle_query,"SELECT * FROM shapeVehicle WHERE id=%d;",vehicle_id);
			result = kSQL->ExecuteSQL(vehicle_query);
			resultSet = kSQL->GetResultSet(result);
			if (resultSet->iNumRows==1)
			{
				S32 rudOff,elevOff,ailOff;
				const char* p;

				int c=1;
				mRudderRange = mDegToRad(dAtof(resultSet->vRows[0]->vColumnValues[c++]));
				rudOff = dAtoi(resultSet->vRows[0]->vColumnValues[c++]);
				mElevRange = mDegToRad(dAtof(resultSet->vRows[0]->vColumnValues[c++]));
				elevOff = dAtoi(resultSet->vRows[0]->vColumnValues[c++]);
				mAilerRange = mDegToRad(dAtof(resultSet->vRows[0]->vColumnValues[c++]));
				ailOff = dAtoi(resultSet->vRows[0]->vColumnValues[c++]);

				rudderNodes = resultSet->vRows[0]->vColumnValues[c++];
				elevNodes = resultSet->vRows[0]->vColumnValues[c++];
				rightAilerNodes = resultSet->vRows[0]->vColumnValues[c++];
				leftAilerNodes = resultSet->vRows[0]->vColumnValues[c++];
				propNodes = resultSet->vRows[0]->vColumnValues[c++];
				rotorNodesA = resultSet->vRows[0]->vColumnValues[c++];
				rotorNodesB = resultSet->vRows[0]->vColumnValues[c++];
				tailRotorNodes = resultSet->vRows[0]->vColumnValues[c++];
				
				mPropBlurSpeed = dAtof(resultSet->vRows[0]->vColumnValues[c++]);
				mPropDiscSpeed = dAtof(resultSet->vRows[0]->vColumnValues[c++]);
				mPropBlurAlpha = dAtof(resultSet->vRows[0]->vColumnValues[c++]);
				mPropDiscAlpha = dAtof(resultSet->vRows[0]->vColumnValues[c++]);


				//////////////////////////////////////////////
				//REFACTOR: This is going to be an ugly mess of repetitive code for the first pass, but some kind of
				//loop would sure be nice.
				if (!rudderNodes.find(","))
				{
					mRudderNodes.increment();
					mRudderNodes.last() = rudderNodes;
				} else {
					for (p = strtok( (char *)rudderNodes.c_str(), "," );  p;  p = strtok( NULL, "," ))
					{
						mRudderNodes.increment();
						mRudderNodes.last() = p;
					}
				}//////////////////////////////////////////////
				if (!elevNodes.find(","))
				{
					mElevNodes.increment();
					mElevNodes.last() = elevNodes;
				} else {
					for (p = strtok( (char *)elevNodes.c_str(), "," );  p;  p = strtok( NULL, "," ))
					{
						mElevNodes.increment();
						mElevNodes.last() = p;
					}
				}//////////////////////////////////////////////				
				if (!rightAilerNodes.find(","))
				{
					mRightAilerNodes.increment();
					mRightAilerNodes.last() = rightAilerNodes;
				} else {
					for (p = strtok( (char *)rightAilerNodes.c_str(), "," );  p;  p = strtok( NULL, "," ))
					{
						mRightAilerNodes.increment();
						mRightAilerNodes.last() = p;
					}
				}//////////////////////////////////////////////				
				if (!leftAilerNodes.find(","))
				{
					mLeftAilerNodes.increment();
					mLeftAilerNodes.last() = leftAilerNodes;
				} else {
					for (p = strtok( (char *)leftAilerNodes.c_str(), "," );  p;  p = strtok( NULL, "," ))
					{
						mLeftAilerNodes.increment();
						mLeftAilerNodes.last() = p;
					}
				}//////////////////////////////////////////////				
				if (!propNodes.find(","))
				{
					mPropNodes.increment();
					mPropNodes.last() = propNodes;
				} else {
					for (p = strtok( (char *)propNodes.c_str(), "," );  p;  p = strtok( NULL, "," ))
					{
						mPropNodes.increment();
						mPropNodes.last() = p;
					}
				}//////////////////////////////////////////////				
				if (!rotorNodesA.find(","))
				{
					mRotorNodesA.increment();
					mRotorNodesA.last() = rotorNodesA;
				} else {
					for (p = strtok( (char *)rotorNodesA.c_str(), "," );  p;  p = strtok( NULL, "," ))
					{
						mRotorNodesA.increment();
						mRotorNodesA.last() = p;
					}
				}//////////////////////////////////////////////
				if (!rotorNodesB.find(","))
				{
					mRotorNodesB.increment();
					mRotorNodesB.last() = rotorNodesB;
				} else {
					for (p = strtok( (char *)rotorNodesB.c_str(), "," );  p;  p = strtok( NULL, "," ))
					{
						mRotorNodesB.increment();
						mRotorNodesB.last() = p;
					}
				}//////////////////////////////////////////////
				if (!tailRotorNodes.find(","))
				{
					mTailRotorNodes.increment();
					mTailRotorNodes.last() = tailRotorNodes;
				} else {
					for (p = strtok( (char *)tailRotorNodes.c_str(), "," );  p;  p = strtok( NULL, "," ))
					{
						mTailRotorNodes.increment();
						mTailRotorNodes.last() = p;
					}
				}//////////////////////////////////////////////
				
				//Next: start doing this in one big query, as in:
				/*
				%query = "SELECT ss.id as ss_id,shape_id,shapeGroup_id,behavior_tree," @ 
	         "p.x as pos_x,p.y as pos_y,p.z as pos_z," @ 
	         "r.x as rot_x,r.y as rot_y,r.z as rot_z,r.angle as rot_angle," @ 
	         "sc.x as scale_x,sc.y as scale_y,sc.z as scale_z," @ 
	         "sp.x as scene_pos_x,sp.y as scene_pos_y,sp.z as scene_pos_z," @ 
	         "sh.datablock as datablock " @ 
	         "FROM sceneShape ss " @ 
	         "JOIN scene s ON s.id=scene_id " @
	         "LEFT JOIN vector3 p ON ss.pos_id=p.id " @ 
	         "LEFT JOIN rotation r ON ss.rot_id=r.id " @ 
	         "LEFT JOIN vector3 sc ON ss.scale_id=sc.id " @ 
	         "LEFT JOIN vector3 sp ON s.pos_id=sp.id " @ 
	         "JOIN physicsShape sh ON ss.shape_id=sh.id " @ 
	         "WHERE scene_id=" @ %scene_id @ ";";  
				*/
 				sprintf(offset_query,"SELECT x,y,z FROM vector3 WHERE id=%d;",rudOff);
				result2 = kSQL->ExecuteSQL(offset_query);
				resultSet2 = kSQL->GetResultSet(result2);
				if (resultSet2->iNumRows==1)
				{
					mRudderOffset = Point3F(dAtof(resultSet2->vRows[0]->vColumnValues[0]),
													dAtof(resultSet2->vRows[0]->vColumnValues[1]),
													dAtof(resultSet2->vRows[0]->vColumnValues[2]));
					Con::printf("rudder offset: %f %f %f",mRudderOffset.x,mRudderOffset.y,mRudderOffset.z);
					kSQL->ClearResultSet(result2);
				}

 				sprintf(offset_query,"SELECT x,y,z FROM vector3 WHERE id=%d;",elevOff);
				result2 = kSQL->ExecuteSQL(offset_query);
				resultSet2 = kSQL->GetResultSet(result2);
				if (resultSet2->iNumRows==1)
				{
					mElevOffset = Point3F(dAtof(resultSet2->vRows[0]->vColumnValues[0]),
													dAtof(resultSet2->vRows[0]->vColumnValues[1]),
													dAtof(resultSet2->vRows[0]->vColumnValues[2]));
					Con::printf("elevator offset: %f %f %f",mElevOffset.x,mElevOffset.y,mElevOffset.z);
					kSQL->ClearResultSet(result2);
				}

				sprintf(offset_query,"SELECT x,y,z FROM vector3 WHERE id=%d;",ailOff);
				result2 = kSQL->ExecuteSQL(offset_query);
				resultSet2 = kSQL->GetResultSet(result2);
				if (resultSet2->iNumRows==1)
				{
					mAilerOffset = Point3F(dAtof(resultSet2->vRows[0]->vColumnValues[0]),
													dAtof(resultSet2->vRows[0]->vColumnValues[1]),
													dAtof(resultSet2->vRows[0]->vColumnValues[2]));
					kSQL->ClearResultSet(result2);
				}

				//Until we get these in the DB too.
				mRotorOffsetA.zero();
				mRotorOffsetB.zero();

			}
			kSQL->ClearResultSet(result);
		}		
	}

	//shapeMount
	if (isServerObject())//Wish I had a one line insta-scan of the database here to find out if any mounts exist for this shape...
	{
	   sprintf(part_query,"SELECT id FROM shapeFile WHERE path IN ('%s');",mDataBlock->shapeName);
	   result = kSQL->ExecuteSQL(part_query);
		if (result==0) goto EXIT; 				
		resultSet = kSQL->GetResultSet(result);
	   if (resultSet->iNumRows<=0) goto EXIT; 	
		file_id = dAtoi(resultSet->vRows[0]->vColumnValues[0]);

	   sprintf(part_query,"SELECT * FROM shapeMount WHERE parent_shape_id=%d;",file_id);
	   result = kSQL->ExecuteSQL(part_query);
	   if (result==0)
		   goto EXIT; 				

	   resultSet = kSQL->GetResultSet(result);
	   if (resultSet->iNumRows<=0)
		   goto EXIT; 	

		TSStatic *mountObj;

	   for (U32 i=0;i<resultSet->iNumRows;i++)
	   {
		   S32 j=0;//column counter
		   char baseNode[255],childNode[255];

		   //////////////////////////////////////////
		   // Mount Data
		   physicsMountData* MD = new physicsMountData;

		   S32 ID = dAtoi(resultSet->vRows[i]->vColumnValues[j++]);
		   j++;//We already know parent_shape, so skip it.		

		   MD->childShape = dAtoi(resultSet->vRows[i]->vColumnValues[j++]);
		   S32 offset_id = dAtoi(resultSet->vRows[i]->vColumnValues[j++]);
		   S32 orient_id = dAtoi(resultSet->vRows[i]->vColumnValues[j++]);
		   MD->jointID = dAtoi(resultSet->vRows[i]->vColumnValues[j++]);			
		   MD->parentNode = dAtoi(resultSet->vRows[i]->vColumnValues[j++]);	
		   MD->childNode = dAtoi(resultSet->vRows[i]->vColumnValues[j++]);
						
		   sprintf(datablock_query,"SELECT path FROM shapeFile WHERE id=%d;",MD->childShape);
			result2 = kSQL->ExecuteSQL(datablock_query);
			resultSet2 = kSQL->GetResultSet(result2);
			char shapeFile[255];
			sprintf(shapeFile,resultSet2->vRows[0]->vColumnValues[0]);
			kSQL->ClearResultSet(result2);

			sprintf(datablock_query,"SELECT x,y,z FROM vector3 WHERE id=%d;",offset_id);
			result2 = kSQL->ExecuteSQL(datablock_query);
			resultSet2 = kSQL->GetResultSet(result2);//Sure do wish I had a cleaner interface to sql results...
			MD->offset = Point3F(dAtof(resultSet2->vRows[0]->vColumnValues[0]),
										dAtof(resultSet2->vRows[0]->vColumnValues[1]),
										dAtof(resultSet2->vRows[0]->vColumnValues[2]));
			kSQL->ClearResultSet(result2);
			
			sprintf(datablock_query,"SELECT x,y,z FROM vector3 WHERE id=%d;",orient_id);
			result2 = kSQL->ExecuteSQL(datablock_query);
			resultSet2 = kSQL->GetResultSet(result2);
			MD->orient = Point3F(dAtof(resultSet2->vRows[0]->vColumnValues[0]),
										dAtof(resultSet2->vRows[0]->vColumnValues[1]),
										dAtof(resultSet2->vRows[0]->vColumnValues[2]));
			kSQL->ClearResultSet(result2);

			//Con::printf("found a shape mount! childShape %d, offset %f %f %f orient %f %f %f path %s",
			//	MD->childShape,MD->offset.x,MD->offset.y,MD->offset.z,MD->orient.x,MD->orient.y,MD->orient.z,shapeFile);
		   

			SimSet* missionGroup = NULL;
			missionGroup = dynamic_cast<SimSet*>(Sim::findObject("MissionGroup"));
			/*
			SimDataBlockGroup* pGroup = Sim::getDataBlockGroup();
			SimDataBlock* pDataBlock = NULL;
			pDataBlock = dynamic_cast<SimDataBlock*>(Sim::findObject(dbName));
			if (pDataBlock)
				Con::printf("Found the datablock with Sim::findObject! %s %d",pDataBlock->getClassName(),pDataBlock->getId());
			else 
			{
				Con::printf("FAILED to find a datablock with name %s",dbName);	
				continue;
			}*/

			EulerF kOrient = EulerF(mDegToRad(MD->orient.x),mDegToRad(MD->orient.y),mDegToRad(MD->orient.z));
			MatrixF mat(kOrient);
			mat.setPosition(MD->offset);

         //PhysicsShape *mountObj = new PhysicsShape();
			mountObj = new TSStatic();
         //mountObj->setDataBlock( dynamic_cast<GameBaseData *>(pDataBlock) );

         ////mountObj->setTransform( mat );
			char objName[255];
			sprintf(objName,"obj_%d_mount_%d",getId(),i);		
			mountObj->mShapeName = StringTable->insert(shapeFile);
         mountObj->registerObject(objName);
			missionGroup->addObject(mountObj);	
			Con::printf("object %d using shapefile: %s, parent node %d  child node %d",
				mountObj->getId(),mountObj->mShapeName,MD->parentNode,MD->childNode);
			this->mountObjectEx(dynamic_cast<SceneObject *>(mountObj),MD->parentNode,MD->childNode,mat);
         //this->mountObjectEx(dynamic_cast<SceneObject *>(mountObj),-1,-1,mat);
			
			//this->mountObjectEx(dynamic_cast<SceneObject *>(mountObj),MD->parentNode,MD->childNode,mat);
        // this->mountObjectEx(dynamic_cast<SceneObject *>(mountObj2),0,0,mat);
			//mountObj->registerObject();
		}

EXIT:
		kSQL->ClearResultSet(result);
	}


   return true;
}

void PhysicsShape::_initAmbient()
{
   if ( isServerObject() )
      return;

   bool willPlay = mPlayAmbient && mAmbientSeq != -1;

	if ( willPlay )
	{
		// Create thread if we don't already have it.
		if ( mAmbientThread == NULL )
			mAmbientThread = mShapeInstance->addThread();

		// Play the sequence.
		mShapeInstance->setSequence( mAmbientThread, mAmbientSeq, 0);
		mCurrentSeq = mAmbientSeq;
		setProcessTick(true);
	}
	else
	{
		if ( mAmbientThread != NULL )
		{
         mShapeInstance->destroyThread( mAmbientThread );
         mAmbientThread = NULL;
      }
   }
}

void PhysicsShape::_setCurrent()
{
   if ( isServerObject() )
      return;

   bool willPlay = (mCurrentSeq >= 0) && (mCurrentSeq < mShapeInstance->getShape()->sequences.size());
	
   if ( willPlay )
   {
      // Create thread if we dont already have.
      if ( mAmbientThread == NULL )
         mAmbientThread = mShapeInstance->addThread();

      // Play the sequence.
      mShapeInstance->setSequence( mAmbientThread, mCurrentSeq, 0);
      setProcessTick(true);
   }
   else
   {//Hmm, do we really want to destroy the thread every time a seq fails? I guess so, easy to recreate it above.
      if ( mAmbientThread != NULL )
      {
         mShapeInstance->destroyThread( mAmbientThread );
         mAmbientThread = NULL;
      }
   }
}

void PhysicsShape::_onPhysicsReset( PhysicsResetEvent reset )
{
   if ( reset == PhysicsResetEvent_Store )
      mResetPos = getTransform();

   else if ( reset == PhysicsResetEvent_Restore )
   {
      setTransform( mResetPos );

      // Restore to un-destroyed state.
      restore();

      // Cheat and reset the client from here.
      if ( getClientObject() )
      {
         PhysicsShape *clientObj = (PhysicsShape*)getClientObject();
         clientObj->setTransform( mResetPos );
         clientObj->restore();
      }
   }
}

void PhysicsShape::setTransform( const MatrixF &newMat )
{
   Parent::setTransform( newMat );
   

   // This is only called to set an absolute position
   // so we discard the delta state.
   if (  mPhysicsBodies.size() == 1 )//isServerObject() ||
	   mState.position = getPosition();
   else // has multiple bodies, and is on the client... try this. Might need to rotate defTrans by newMat however.
   {
	   mState.position = getPosition();// + mShapeInstance->getShape()->defaultTranslations[0];
	   //updatePhysicsBodies()??
   }
   mState.orientation.set( newMat );
   mRenderState[0] = mRenderState[1] = mState;
   setMaskBits( StateMask );

   if ( mPhysicsRep )
   {
	  MatrixF newNewMat = newMat;
	  newNewMat.setPosition(mState.position);
	  mPhysicsRep->setTransform( newNewMat );	  
   }
}

//void PhysicsShape::setScale( const VectorF &scale )
//{
   // Cannot scale PhysicsShape. //FIX
//   return;
//}

void PhysicsShape::storeRestorePos()
{
   mResetPos = getTransform();
}

F32 PhysicsShape::getMass() const 
{ 
   return mDataBlock->mass; 
}

void PhysicsShape::setIsRecording(bool record)
{
	if (isServerObject())
	{
		mIsRecording = record;
		PhysicsShape *clientShape = dynamic_cast<PhysicsShape *>(getClientObject());//SINGLE PLAYER HACK
		clientShape->setIsRecording(record);
		return;
	}

	if (record)
	{
		mIsRecording = true;
		Con::printf("client starting recording!");
		mRecordInitialPosition = getTransform().getPosition();
		MatrixF kTransform = getTransform();
		mRecordInitialOrientation.set(kTransform);

		mRecordCount = 0;
		mNodeTranslations.clear();
		mNodeRotations.clear();
	}
	else
		mIsRecording = false;
}

DefineEngineMethod( PhysicsShape, setIsRecording, void, (bool isRecording),, 
   "@brief \n\n" )
{
	object->setIsRecording(isRecording);
	return;
}

S32 PhysicsShape::getRecordingSize()
{
	S32 size;
	if (isServerObject())
	{       
		PhysicsShape *clientShape = dynamic_cast<PhysicsShape *>(getClientObject());//SINGLE PLAYER HACK
		size = clientShape->getRecordingSize();
		return size;
	}

	//This should never happen, but...
	size = (mNodeRotations.size()*sizeof(Quat16) + mNodeTranslations.size()*sizeof(Point3F));		
	return  size;
}

DefineEngineMethod( PhysicsShape, getRecordingSize, S32, (),, 
   "@brief \n\n" )
{
	return object->getRecordingSize();
}


void PhysicsShape::recordTick()
{
	if (isServerObject())
		return;//For now, all important physics is being done on the client.

	mRecordCount++;
	bool kRelativePosition = true; 
	Point3F kDiff,kPos;
	if (mPhysicsBodies.size()==0)
	{//FIX!! redundancy with following physics based logic, merge and refactor
		if (!(mRecordCount % mRecordSampleRate))
		{
			mNodeTranslations.increment();
			QuatF kRot = mRecordInitialOrientation;
			kRot.inverse();
			MatrixF bodyTransform = getTransform();
			if (kRelativePosition)
			{//TEMP - kRelativePosition: trying to figure out what will work for iClone.  Doesn't rotate  
				//bvh position when you rotate actor.			
				kDiff = bodyTransform.getPosition() - mRecordInitialPosition;
				kRot.mulP(kDiff,&kPos);
				//Con::printf("kPos recording relative: %3.2f %3.2f %3.2f, initial %3.2f %3.2f %3.2f",kPos.x,kPos.y,kPos.z,
				//	mRecordInitialPosition.x,mRecordInitialPosition.y,mRecordInitialPosition.z);
			} else {
				kPos = bodyTransform.getPosition();
				//Con::printf("kPos recording: %3.2f %3.2f %3.2f",kPos.x,kPos.y,kPos.z);
			}
			kPos /= mObjScale;
			mNodeTranslations[mNodeTranslations.size()-1] = kPos;

			QuatF q(bodyTransform);
			mNodeRotations.increment();
			mNodeRotations[mNodeRotations.size()-1].set(q);
		}
	}
	//Con::printf("recording tick! mRecordCount %d, orderNodes %d",mRecordCount,mOrderNodes.size());
	if (!(mRecordCount % mRecordSampleRate))// Should make this possible for non articulated
	{																	//shapes as well, and also for the server sim in that case.
		mNodeTranslations.increment();
		//nodeTranslations[nodeTranslations.size()-1] = mCurrPosition;
		//nodeTranslations[nodeTranslations.size()-1] = mBodyParts[0]->mRB->getLinearPosition();
		
		Point3F kDiff,kPos;
		Quat16 tempRots[200];//TEMP, make expandable, but 200 nodes should 
		Point3F tempTrans[200];// be plenty for quite a while...
		QuatF kRot = mRecordInitialOrientation;
		kRot.inverse();
		//NOW: make the conversion to global position happen 
		  //ONLY at export time.
		//bool kRelativePosition = dynamic_cast<nxPhysManager*>(mPM)->mSceneRecordLocal;
		MatrixF bodyTransform;
		mPhysicsBodies[0]->getTransform(&bodyTransform);
		if (kRelativePosition)
		{//TEMP - kRelativePosition: trying to figure out what will work for iClone.  Doesn't rotate  
			//bvh position when you rotate actor.			
			kDiff = bodyTransform.getPosition() - mRecordInitialPosition;
			kRot.mulP(kDiff,&kPos);
			//Con::printf("kPos recording relative: %3.2f %3.2f %3.2f, initial %3.2f %3.2f %3.2f",kPos.x,kPos.y,kPos.z,
			//	mRecordInitialPosition.x,mRecordInitialPosition.y,mRecordInitialPosition.z);
		} else {
			kPos = bodyTransform.getPosition();
			//Con::printf("kPos recording: %3.2f %3.2f %3.2f",kPos.x,kPos.y,kPos.z);
		}
		kPos /= mObjScale;
		mNodeTranslations[mNodeTranslations.size()-1] = kPos;

		for (U32 i=0;i<mPhysicsBodies.size();i++)
		{
			QuatF q,p;
			//q = mBodyParts[i-kNumWeapons]->mRB->getAngularPosition();
			MatrixF bodypartTransform,parentTransform;
			mPhysicsBodies[i]->getTransform(&bodypartTransform);
			q = QuatF(bodypartTransform);
			
			if (i>0) //only node 0 doesn't have a parent, unless we're a weird case.
			{
				S32 parentBody = mPhysicsBodies[i]->getParentBodyIndex();
				if (parentBody>=0) 
				{
					mPhysicsBodies[parentBody]->getTransform(&parentTransform);
					p = QuatF(parentTransform);
					p.inverse();
					q *= p;
				} 
			} else {
				if (kRelativePosition)
					q *= kRot;
			}

			tempRots[i].set(q);

			kPos = bodypartTransform.getPosition();
			kPos -= getPosition();
			kPos /= mObjScale;//HERE: also rotate! inverse of flexbody transform?
			tempTrans[i] = kPos;

		}

		for (U32 i=0;i<mPhysicsBodies.size();i++)
		{//Now, copy them all to nodeRotations in the right order.
			mNodeRotations.increment();
			mNodeRotations[mNodeRotations.size()-1] = tempRots[mOrderNodes[i]];
			if (i==0) {
				QuatF q;
				tempRots[mOrderNodes[i]].getQuatF(&q);
			}

			if ((mSaveTranslations)&&(i>0))//(We already saved i==0 above)
			{
				mNodeTranslations.increment();
				mNodeTranslations[mNodeTranslations.size()-1] = tempTrans[mOrderNodes[i]];
			}
		}
	}
}

void PhysicsShape::setupOrderNodes()
{
	U32 sortList[200];

	for (U32 i = 0; i < mPhysicsBodies.size(); i++)
	{
		sortList[i] = mPhysicsBodies[i]->getNodeIndex();
		mOrderNodes.increment();
	}

	for (U32 i = 0; i < mPhysicsBodies.size(); i++)
	{
		for (U32 j = 0; j < mPhysicsBodies.size()-1; j++)
		{
			S32 temp = -1;
			if (sortList[j] > sortList[j+1]) 
			{
				temp = sortList[j];
				sortList[j] = sortList[j+1];
				sortList[j+1] = temp;				
			}
		}
	}
	
	for (U32 i=0; i<mPhysicsBodies.size(); i++)
		for (U32 j=0; j<mPhysicsBodies.size(); j++)
			if (mPhysicsBodies[j]->getNodeIndex() == sortList[i]) 
			{
				mOrderNodes[i] = j;
			}

}

void PhysicsShape::makeSequence(const char *seqName)
{
	if (isServerObject())
	{
		PhysicsShape *clientShape = dynamic_cast<PhysicsShape *>(getClientObject());//SINGLE PLAYER HACK
		clientShape->makeSequence(seqName);
		return;
	}
	
	Con::printf("client object trying to make sequence. nodeTranslations: %d  nodeRotations %d",mNodeTranslations.size(),mNodeRotations.size());
	
	U32 importGround = 0;
	S32 numRealKeyframes;
	//Con::errorf("sequences: %d, rotations: %d, translations %d",mShape->sequences.size(),mShape->nodeRotations.size(),mShape->nodeTranslations.size());

	//NOW: make the conversion to global position happen 
	bool kRelativePosition = true;    //ONLY at export time.
	//bool kRelativePosition = dynamic_cast<nxPhysManager*>(mPM)->mSceneRecordLocal;
		
	//Thread& st = mScriptThread[0];
	//if (st.thread )
	//{
	//	st.sequence = 0;
	//	st.thread->setSequence(0,0.0);
	//	st.state = fxFlexBody::Thread::Stop;
	//	updateThread(st);
	//}
	
	const String dsqExt(".dsq");
	/*
	String seqDir(seqName);
	//remove ".dsq" from filename if it's there
	//if (dStrlen(seqDir.find(dsqExt.c_str(),0))) 
	//	seqDir.erase(seqDir.length()-dsqExt.length(),dsqExt.length());
	if (dStrstr((const char *)seqDir.c_str(),".dsq"))
		seqDir.erase(seqDir.length()-dsqExt.length(),dsqExt.length());
	//if (strpos(seqDir.c_str(),dsqExt.c_str())>-1)         // seqDir.find(dsqExt.c_str(),0,String::Case|String::Left) > -1) 
	//	seqDir.erase(seqDir.length()-dsqExt.length(),dsqExt.length());

	//then separate the sequence name, to get it by itself and the path by itself.
	U32 nameLength = dStrlen(dStrrchr(seqDir.c_str(),'/'))-1;
	String sequenceName(seqDir);
	sequenceName.erase(0,seqDir.length()-nameLength);
	seqDir.erase(seqDir.length()-nameLength,nameLength);
	*/
	String sequenceName(seqName);
	//and make sure there's no spaces
	sequenceName.replace(' ','_');

	//Now, make the new sequence.
	mShape->sequences.increment();
	TSShape::Sequence & seq = mShape->sequences.last();
	constructInPlace(&seq);
	S32 numKeys;
	if (mPhysicsBodies.size()>0) 
		numKeys = mNodeRotations.size() / mPhysicsBodies.size();//This should be safe no matter what.
	else
		numKeys = mNodeRotations.size();

	if (kRelativePosition)
		seq.numKeyframes = numKeys;//nodeTranslations.size();//safe (NO LONGER) because only base node has translations, one per keyframe.
	else//NOW - for iClone, adding ten frames at the beginning to interpolate out from (0,0,0) to starting position.
		seq.numKeyframes = numKeys + 10;//nodeTranslations.size() + 10;

	seq.duration = (F32)seq.numKeyframes * (TickSec*mRecordSampleRate);
	seq.baseRotation = mShape->nodeRotations.size();
	seq.baseTranslation = mShape->nodeTranslations.size();
	seq.baseScale = 0;
	seq.baseObjectState = 1;
	seq.baseDecalState = 0;
	seq.firstGroundFrame = mShape->groundTranslations.size();
	//if (importGround) seq.numGroundFrames = numSamples;
	//else seq.numGroundFrames = 0;//1;?
	seq.numGroundFrames = 0;//TEMP, groundRotations.size();
	seq.firstTrigger = mShape->triggers.size();
	seq.numTriggers = 0;
	seq.toolBegin = 0.0;
	seq.flags = 0;//TSShape::Cyclic;// | TSShape::Blend;// | TSShape::MakePath;
	seq.priority = 5;

	Con::errorf("New sequence!  numKeyframes %d, duration %f",seq.numKeyframes,seq.duration);
	
	seq.rotationMatters.clearAll();
	//seq.rotationMatters.set(0);
	//seq.rotationMatters.set(1);
	//seq.rotationMatters.set(2);
	//seq.rotationMatters.set(3);
	//for (U32 i=0;i<rc;i++) seq.rotationMatters.set(dtsNodes[i]);//numJoints
	for (U32 i=0;i<mPhysicsBodies.size();i++) { 
		seq.rotationMatters.set(mPhysicsBodies[i]->getNodeIndex()); 
		//Con::errorf("rotation matters: %d",mBodyParts[i]->mNodeIndex);
	}

	seq.translationMatters.clearAll();
	if (mSaveTranslations)
	{
		for (U32 i=0;i<mPhysicsBodies.size();i++) { 
			seq.translationMatters.set(mPhysicsBodies[i]->getNodeIndex()); 
		}
	} 	else {
		seq.translationMatters.set(0);//ASSUMPTION: only root node has position data
	}

	seq.scaleMatters.clearAll();
	seq.visMatters.clearAll();
	seq.frameMatters.clearAll();
	seq.matFrameMatters.clearAll();
	//seq.decalMatters.clearAll();
	//seq.iflMatters.clearAll();

	mShape->names.increment();
	mShape->names.last() = StringTable->insert(seqName);

	seq.nameIndex = mShape->findName(seqName);

	//if ((!kRelativePosition)&&(!importGround))
	//{
		//for (U32 i=0;i<10;i++)
		//{
		//	mShape->nodeTranslations.increment();
		//	//nodeTranslations[0].z = 0.0;//For iClone specifically - vertical axis has to be zero, iClone keeps 
		//	//it on ground as long as there is no Z (actually Y in bvh/iClone world) value.
		//	Point3F pos;
		//	pos.x = nodeTranslations[0].x * ((F32)i/10.0);
		//	pos.y = nodeTranslations[0].y * ((F32)i/10.0);
		//	pos.z = nodeTranslations[0].z;
		//	//mShape->nodeTranslations[mShape->nodeTranslations.size()-1] = nodeTranslations[0] * ((F32)i/10.0);
		//	mShape->nodeTranslations[mShape->nodeTranslations.size()-1] = pos;
		//}
	//	numRealKeyframes = seq.numKeyframes - 10;
	//} else numRealKeyframes = seq.numKeyframes;
	numRealKeyframes = seq.numKeyframes;

	if (mSaveTranslations==false)
	{//Normal skeleton animations, only rotation is used on bodyparts, root node has translation.
		for (U32 i=0;i<numRealKeyframes;i++)
		{
			if (importGround)
			{
				mShape->nodeTranslations.increment();
				mShape->nodeTranslations[mShape->nodeTranslations.size()-1].x = 0.0;
				mShape->nodeTranslations[mShape->nodeTranslations.size()-1].y = 0.0;
				mShape->nodeTranslations[mShape->nodeTranslations.size()-1].z = mNodeTranslations[i].z;

				mShape->groundRotations.increment();
				mShape->groundRotations[mShape->groundRotations.size()-1].identity();

				mShape->groundTranslations.increment();
				mShape->groundTranslations[mShape->groundTranslations.size()-1].x = mNodeTranslations[i].x;
				mShape->groundTranslations[mShape->groundTranslations.size()-1].y = mNodeTranslations[i].y;
				mShape->groundTranslations[mShape->groundTranslations.size()-1].z = 0.0;
			} else {
				
				
				mShape->nodeTranslations.increment();
				mShape->nodeTranslations[mShape->nodeTranslations.size()-1] = mNodeTranslations[i];
			
				//nodeTranslations[i].z = 0.0;//iClone, see above. [Nope - didn't work either.]
				//Con::errorf("nodeTranslations: %f %f %f",nodeTranslations[i].x,nodeTranslations[i].y,nodeTranslations[i].z);
			}
		}
	} else {//Non-normal situation, destructible buildings, avalanche, etc. - no joints, all node translations stored.
		for(U32 j=0;j<mPhysicsBodies.size();j++)
		{
			for (U32 i=0;i<numRealKeyframes;i++)
			{
				//TEMP
				mShape->nodeTranslations.increment();
				Point3F pos = mNodeTranslations[(i*mPhysicsBodies.size())+j];
				mShape->nodeTranslations[mShape->nodeTranslations.size()-1] = pos;
			}
		}
	}

	//U32 kTotalParts = mNumBodyParts;//for i=0 - numBodyparts, if (mWeapons[i]) kTotalParts++;
	//if (mWeapon) kTotalParts++;//((mWeapon)&&(mWeaponBodypart->mPartID>=0))
	//if (mWeapon2) kTotalParts++;//((mWeapon2)&&(mWeapon2Bodypart->mPartID>=0))
	U32 bones = 1;
	if (mPhysicsBodies.size()>0)
		bones = mPhysicsBodies.size();

	for(U32 j=0;j<bones;j++)//mPhysicsBodies.size()
	{
		if ((!kRelativePosition)&&(!importGround))
		{
			for (U32 i=0;i<10;i++)
			{
				mShape->nodeRotations.increment();
				mShape->nodeRotations[mShape->nodeRotations.size()-1] = mNodeRotations[j];//Should duplicate 
			}                                                             //first frame rotations ten times.
		}
		for (U32 i=0;i<numRealKeyframes;i++)
		{
			//q16.set(rots[(i*numJoints)+orderNodes[j]]);
			//q16.set(nodeRotations[(i*mNumBodyParts)+j]);

			//TEMP
			mShape->nodeRotations.increment();
			mShape->nodeRotations[mShape->nodeRotations.size()-1] = mNodeRotations[(i*bones)+j];
			
			QuatF qF = mNodeRotations[(i*bones)+j].getQuatF();
			//Con::printf("  final nodeRots, bodypart %d:  %f %f %f %f",j,qF.x,qF.y,qF.z,qF.w);
			//mShape->nodeRotations[mShape->nodeRotations.size()-1] = nodeRotations[(i*mNumBodyParts)+j];
		}
	}

	mNodeTranslations.clear();
	mNodeRotations.clear();
	mRecordCount = 0;

	//Con::printf("BVH -- nodes %d nodeTranslations %d nodeRotations %d sequences: %d",mShape->nodes.size(),
	//	mShape->nodeTranslations.size(),mShape->nodeRotations.size(),mShape->sequences.size());
	//for (U32 i=0;i<mShape->sequences.size();i++)
	//{
	//	TSShape::Sequence & seq = mShape->sequences[i];

	//	Con::printf("Seq[%d] %s frames: %d duration %3.2f baseObjectState %d baseScale %d baseDecalState %d toolbegin %f",
	//		i,mShape->getName(seq.nameIndex).c_str(),seq.numKeyframes,
	//		seq.duration,mShape->sequences[i].baseObjectState,mShape->sequences[i].baseScale,
	//		mShape->sequences[i].baseDecalState,seq.toolBegin);
	//	Con::printf("   groundFrames %d isBlend %d isCyclic %d flags %d",
	//		seq.numGroundFrames,seq.isBlend(),seq.isCyclic(),seq.flags);
	//}

	//HA!  Yay, finally T3D has its own exportSequence (singular) function, don't 
	//mShape->dropAllButOneSeq(mShape->sequences.size()-1); // have to do this anymore.


	//HERE: It's working, but we do need to find the directory the model lives in first.
	FileStream *outstream;
	String dsqPath(seqName);
	if (!dStrstr(dsqPath.c_str(),".dsq")) dsqPath += dsqExt;
	//if (!gResourceManager->openFileForWrite(outstream,dsqPath.c_str())) {
	if ((outstream = FileStream::createAndOpen( dsqPath.c_str(), Torque::FS::File::Write))==NULL) {
		Con::printf("whoops, name no good: %s!",dsqPath.c_str()); 
	} else {
		//mShape->exportSequences((Stream *)outstream);
		mShape->exportSequence((Stream *)outstream,seq,1);//1 = save in old format (v24) for show tool
		outstream->close();
		Con::printf("Exported sequence to file: %s",dsqPath.c_str());
		//HERE, maybe choose this moment to insert this sequence into the database.  Still need to strip
		//the non-relative part of path off of it first.
		//String relativePath(dsqPath);
		//String::SizeType relativePos = dsqPath.find("art/shapes");
		//if (relativePos>0)
		//	relativePath = dsqPath.substr(relativePos,dsqPath.length() - relativePos);
		//Con::printf("We think the relative path is:  %s",relativePath.c_str());
		//SQLiteObject *sql = new SQLiteObject();
		//if (sql)
		//{
		//	if (sql->OpenDatabase(dynamic_cast<physManagerCommon*>(mPM)->mDatabaseName.c_str()))
		//	{
		//		char insert_query[512];//WARNING id_query[512],
		//		//WARNING sqlite_resultset *resultSet;
		//		int result;

		//		sprintf(insert_query,"INSERT INTO sequence (skeleton_id,filename,name) VALUES (%d,'%s','%s');",
		//			mSkeletonID,relativePath.c_str(),seqName.c_str());
		//		result = sql->ExecuteSQL(insert_query);
		//		//sprintf(id_query,"SELECT id FROM sequence WHERE filename = '%s';",relativePath.c_str());//Could use last id generated?
		//		//resultSet = sql->GetResultSet(result);
		//		sql->CloseDatabase();
		//	}
		//	delete sql;
		//}
	}


	//Now, save any weapons:
	//for(U32 i=0;i<mNumBodyParts;i++)
	//{
	//	if (mWeapons[i])
	//	{
	//		char weapSeqName[512];
	//		sprintf(weapSeqName,"%s.%s",mWeapons[i]->mActorName,seqName.c_str());
	//		mWeapons[i]->makeSequence(weapSeqName);
	//	}
	//}


}

DefineEngineMethod( PhysicsShape, makeSequence, void, (const char *seqName),, 
   "@brief \n\n" )
{
	object->makeSequence(seqName);
	return;
}

void PhysicsShape::applyImpulse( const Point3F &pos, const VectorF &vec )
{
   if ( mPhysicsRep && mPhysicsRep->isDynamic() )
      mPhysicsRep->applyImpulse( pos, vec );
}

void PhysicsShape::applyImpulseToPart(  S32 partIndex, const Point3F &pos, const VectorF &vec)
{
	//Uh oh, need to reach across and apply this to the _client_, though.
	if (isServerObject())
	{       
		PhysicsShape *clientShape = dynamic_cast<PhysicsShape *>(getClientObject());//SINGLE PLAYER HACK
		clientShape->applyImpulseToPart(partIndex,pos,vec);
		return;
	}
	//Else we are the client.
	if ( mPhysicsBodies[partIndex] && mPhysicsBodies[partIndex]->isDynamic() )
	{
		MatrixF partTransform;
		mPhysicsBodies[partIndex]->getTransform(&partTransform);
		Point3F finalPos = partTransform.getPosition() + pos;
		//Con::printf("applying impulse to part %d: %f %f %f",partIndex,vec.x,vec.y,vec.z);
		mPhysicsBodies[partIndex]->applyImpulse( finalPos, vec );
	}
}

void PhysicsShape::applyRadialImpulse( const Point3F &origin, F32 radius, F32 magnitude )
{
   if ( !mPhysicsRep || !mPhysicsRep->isDynamic() )
      return;

   // TODO: Find a better approximation of the
   // force vector using the object box.

   VectorF force = getWorldBox().getCenter() - origin;
   F32 dist = force.magnitudeSafe();
   force.normalize();

   if ( dist == 0.0f )
      force *= magnitude;
   else
      force *= mClampF( radius / dist, 0.0f, 1.0f ) * magnitude;   

   mPhysicsRep->applyImpulse( origin, force );

   // TODO: There is no simple way to really sync this sort of an 
   // event with the client.
   //
   // The best is to send the current physics snapshot, calculate the
   // time difference from when this event occured and the time when the
   // client recieves it, and then extrapolate where it should be.
   //
   // Even then its impossible to be absolutely sure its synced.
   //
   // Bottom line... you shouldn't use physics over the network like this.
   //

   // Cheat for single player.
   //if ( getClientObject() )
      //((PhysicsShape*)getClientObject())->mPhysicsRep->applyImpulse( origin, force );
}

void PhysicsShape::applyRadialImpulseToPart(  S32 partIndex, const Point3F &origin, F32 radius, F32 magnitude )
{
	if (isServerObject())
	{
		PhysicsShape *clientShape = dynamic_cast<PhysicsShape *>(getClientObject());
		clientShape->applyRadialImpulseToPart(partIndex,origin,radius,magnitude);
		return;
	}

	if ( !mPhysicsBodies[partIndex] || !mPhysicsBodies[partIndex]->isDynamic() )
		return;

	// TODO: Find a better approximation of the
	// force vector using the object box.

	VectorF force = getWorldBox().getCenter() - origin;
	F32 dist = force.magnitudeSafe();
	force.normalize();

	if ( dist == 0.0f )
		force *= magnitude;
	else
		force *= mClampF( radius / dist, 0.0f, 1.0f ) * magnitude;   

	mPhysicsBodies[partIndex]->applyImpulse( origin, force );

}

void PhysicsShape::interpolateTick( F32 delta )
{
	
   AssertFatal( !mDestroyed, "PhysicsShape::interpolateTick - Shouldn't be processing a destroyed shape!" );
	
	/////////////////////////////////////////////////
	//openSimEarth, if mUseDataSource then we need to update vehicle position and node orientations.
	if (mUseDataSource)
	{		
		if (!mDataSource)
		{
			mDataSource = new vehicleDataSource(true);
			mPropStatus = 0;//0=blades, 1=propblur, 2=propdisc
		}

		mDataSource->tick();

		QuatF dsQuat;
		dsQuat = QuatF(mDataSource->mFGTransform);
		Point3F dsPos =  mDataSource->mFGTransform.getPosition();
		Point3F currPos = getPosition();
		MatrixF trans;
		dsQuat.setMatrix(&trans);
		trans.setPosition(dsPos);

		/////// TEMP: this is for initial FG models, off on Z by ninety degrees. FIX THE MODELS //////////////////
		MatrixF ninetyFix(EulerF(0,0,M_PI/2));
		trans *= ninetyFix;
		////////////// TEMP ////////////////////

		//mRenderState[1].position = dsPos;
		//mRenderState[1].orientation = dsQuat;//??? I have no idea how renderStates work
		setTransform(trans);
		Parent::setTransform( trans ); 
		EulerF rudderRot,elevRot,ailerRot,propRot,rotorRot;
		F32 engineRPM,rotorRPM,rotPerTick;

		PhysicsShape *servObj = (PhysicsShape*)getServerObject();
		servObj->setTransform( trans );
		servObj->setMaskBits( StateMask );

		rudderRot = EulerF(0,0,(mRudderRange * mDataSource->mFGPacket.rudder));
		for (U32 c=0;c<mRudderNodes.size();c++)
			mShapeInstance->setNodeTransform(mRudderNodes[c].c_str(),mRudderOffset,rudderRot);

		elevRot = EulerF(0,(-mElevRange * mDataSource->mFGPacket.elevator),0);
		for (U32 c=0;c<mElevNodes.size();c++)
			mShapeInstance->setNodeTransform(mElevNodes[c].c_str(),mElevOffset,elevRot);

		ailerRot = EulerF(0,(-mAilerRange * mDataSource->mFGPacket.right_aileron),0);
		for (U32 c=0;c<mRightAilerNodes.size();c++)
			mShapeInstance->setNodeTransform(mRightAilerNodes[c].c_str(),mAilerOffset,ailerRot);

		ailerRot = EulerF(0,(-mAilerRange * mDataSource->mFGPacket.left_aileron),0);
		for (U32 c=0;c<mLeftAilerNodes.size();c++)
			mShapeInstance->setNodeTransform(mLeftAilerNodes[c].c_str(),mAilerOffset,ailerRot);

		// Propeller, get rotations per tick out of the RPM value, and do blur effects.
		engineRPM = mDataSource->mFGPacket.engine_rpm;
		rotPerTick = (engineRPM / (60 * 32)) * (2 * M_PI);
		propRot = EulerF(rotPerTick,0,0);
		//For a spinning object, instead of setting the transform, add it:
		for (U32 c=0;c<mPropNodes.size();c++)
			mShapeInstance->addNodeTransform(mPropNodes[c].c_str(),mPropOffset,propRot);

		if ((engineRPM < mPropBlurSpeed) && (mPropStatus!=0))
		{
			showPropBlades();
			mPropStatus = 0;//CAUTION: assuming here that no helicopter also has a separate propeller.
		}
		else if ((rotorRPM > mPropBlurSpeed) && (rotorRPM < mPropDiscSpeed) && (mPropStatus!=1))
		{
			showPropBlur();
			mPropStatus = 1;
		}
		else if ((rotorRPM > mPropDiscSpeed) && (mPropStatus!=2))
		{
			showPropDisc();
			mPropStatus = 2;
		}

		////////////////////////////////////////////////////////////////
		rotorRPM = mDataSource->mFGPacket.rotor_rpm;
		//rotPerTick = (rotorRPM / (60 * 32)) * (2 * M_PI);//Hmm... I thought this would be right, but it's way too fast. 
		rotPerTick = (rotorRPM / (60 * 32)) * (M_PI/2);//Commence trial and error... but this appears to work.
		//And now, a remaining bit of hard coding I don't know what to do with. On the ka50, rotors and blades are 
		//a separate mounted object, on other helicopters they are part of the helicopter model. For now just checking
		//for the MainRotor.dts being mounted.
		bool isKa50 = false;//TEMP, gotta work out a generalized system for whether we have mounted rotors or not.
		for (U32 i=0;i<getMountedObjectCount();i++)
		{
			TSStatic *mountObj = dynamic_cast<TSStatic*>(getMountedObject(i));
			if (!mountObj) 
				continue;
			if (strstr(mountObj->getShapeFileName(),"MainRotor.dts")>0)
			{
				TSShapeInstance *rotorShapeInst = mountObj->getShapeInstance();

				rotorRot = EulerF(0,0,rotPerTick); 
				for (U32 c=0;c<mRotorNodesA.size();c++)
					rotorShapeInst->addNodeTransform(mRotorNodesA[c].c_str(),mRotorOffsetA,rotorRot);


				rotorRot = EulerF(0,0,-rotPerTick);
				for (U32 c=0;c<mRotorNodesB.size();c++)
					rotorShapeInst->addNodeTransform(mRotorNodesB[c].c_str(),mRotorOffsetB,rotorRot);

				isKa50 = true;
			}
		}
		if (isKa50 == false)
		{//For all (so far) other helicopters that have their rotors attached to the main model. Also mixing
			//tail rotor in here, because ka50 also doesn't have one.
			rotorRot = EulerF(0,0,rotPerTick); 
			for (U32 c=0;c<mRotorNodesA.size();c++)
				mShapeInstance->addNodeTransform(mRotorNodesA[c].c_str(),mRotorOffsetA,rotorRot);
			for (U32 c=0;c<mTailRotorNodes.size();c++)
				mShapeInstance->addNodeTransform(mTailRotorNodes[c].c_str(),mTailRotorOffset,rotorRot);
		}
		//////////////////////
		//Meanwhile anyone can use propeller/rotor blur.
		if ((rotorRPM < mPropBlurSpeed) && (mPropStatus!=0))
		{
			showRotorBlades();
			mPropStatus = 0;
		}
		else if ((rotorRPM > mPropBlurSpeed) && (rotorRPM < mPropDiscSpeed) && (mPropStatus!=1))
		{
			showRotorBlur();
			mPropStatus = 1;
		}
		else if ((rotorRPM > mPropDiscSpeed) && (mPropStatus!=2))
		{
			showRotorDisc();
			mPropStatus = 2;
		}
		/*//OOOPS. Great idea, except this would affect _every_ user of this material, like other ka50s on the runway. However, 
		if (mPropMaterials.size()>0) // it would still work for single player, one helicopter game, if a better way isn't found.
		{
		for (U32 i=0;i<mPropMaterials.size();i++)
		{
		Material *mat = mPropMaterials[i];
		if (rotorRPM>mPropBlurSpeed)
		{
		mat->mCastShadows=false;
		}
		else
		{
		mat->mCastShadows=true;					 
		}
		//Con::printf("found a prop material! isTranslucent: %d, isDoubleSided %d, dynamic shadows %d, shadows %d",
		//	 mat->isTranslucent(),mat->isDoubleSided(),mat->mCastDynamicShadows,mat->mCastShadows);
		}
		}*/
		//////////////////////
		//NOW: override the player's camera and force it to follow us around as a chase cam instead.	 
		Vector<SceneObject*> kCameras;
		Vector<SceneObject*> kPlayers;
		Box3F bounds;
		Point3F clientPos;
		bool freeCamera;
		Player *myPlayer;
		Camera *myCamera;

		bounds.set(Point3F(-FLT_MAX,-FLT_MAX,-FLT_MAX),Point3F(FLT_MAX,FLT_MAX,FLT_MAX));
		gServerContainer.findObjectList(bounds, CameraObjectType, &kCameras);
		gServerContainer.findObjectList(bounds, PlayerObjectType, &kPlayers);
		for (U32 i=0;i<kPlayers.size();i++)
		{//FIX!!! We should *not* have to find this every time!!
			myPlayer = (Player *)(kPlayers[i]);
			Point3F playerPos = myPlayer->getPosition();
			if (kCameras.size()>0)
			{
				myCamera = dynamic_cast<Camera *>(kCameras[i]);//... sort out which belongs to controlling client.
				Point3F cameraPos = myCamera->getPosition();
				GameConnection *cameraClient = myCamera->getControllingClient();
				GameConnection *playerClient = myPlayer->getControllingClient();
				if (cameraClient) 
				{
					freeCamera = true;
					clientPos = cameraPos;
				} else if (playerClient) {
					freeCamera = false;
					clientPos = playerPos;
				} 
			} else {
				clientPos = playerPos;
			}
		}
		if ((freeCamera))//&&(firstTime)
		{//Let's handle free camera first, you'll have to hit F8 before Alt F for this to work.
			//myCamera->setTrackObject(this,Point3F(0,0,2.0));
			Point3F offset(18.0,0,8.0);//3.0
			Point3F rotOffset;			 	
			MatrixF newTrans(trans);
			Point3F transPos = trans.getPosition();
			newTrans.setPosition(Point3F(0,0,0));
			newTrans.mulP(offset,&rotOffset);
			//Con::printf("rotOffset %f %f %f trans pos %f %f %f",rotOffset.x,rotOffset.y,rotOffset.z,transPos.x,transPos.y,transPos.z);
			Point3F camPos = transPos + rotOffset;

			//Point3F lastPos = clientPos;
			//Point3F camVel = camPos - lastPos;
			//camVel *= 32; //Because velocity will be in meters per second, and we are ticking 32 times per second.
			//myCamera->setVelocity(camVel);//Not entirely sure we're really accomplishing anything here though.
			//And the answer is NOPE, whatever you set this to has no impact.

			newTrans *= MatrixF(EulerF(0,0,-M_PI/2.0));//TEMP: fix the camera transform for the FG planes' 90 degree problem.

			myCamera->setTransform4x4(newTrans);//Use our new function that attempts to stuff the actual matrix	
			myCamera->setPosition(camPos);

			gPlanePos = camPos;
			gPlaneMat = newTrans;

			//Con::printf("setting camPos: %f %f %f server %d",camPos.x,camPos.y,camPos.z,myCamera->isServerObject());
			//EulerF kRot = newTrans.toEuler();
			//myCamera->setRotation(kRot);

			//gClientContainer.findObjectList(bounds, CameraObjectType, &kCameras);
			//if (kCameras.size()>0)
			//{
			//myCamera = dynamic_cast<Camera *>(kCameras[0]);
			//myCamera->setTransform4x4(newTrans);
			//myCamera->setPosition(camPos);
			//Con::printf("setting client camPos %f %f %f clock %d !!!!!!!!!!!!!!!!!!!!!!!!!!",camPos.x,camPos.y,camPos.z,clock());
			//}

			//one way, using limited rotation looking.
			//myCamera->setPosition(camPos);
			//myCamera->setVelocity(camVel);
			//myCamera->lookAt(trans.getPosition());


			//myCamera->setRenderTransform(trans);
		} else {
			//handle player camera
		}
	}
	/////////////////////////////////////////////////
	if ( !mPhysicsRep->isDynamic() )
		return;

	// Interpolate the position and rotation based on the delta.
	PhysicsState state;
	state.interpolate( mRenderState[1], mRenderState[0], delta );

	// Set the transform to the interpolated transform.
	setRenderTransform( state.getTransform() );

}

void PhysicsShape::processTick( const Move *move )
{
   AssertFatal( mPhysicsRep && !mDestroyed, "PhysicsShape::processTick - Shouldn't be processing a destroyed shape!" );

	if ((mBodyNodes.size()==0)||(mNodeBodies.size()==0))
		return;//I think we're calling this before we've created the shape...


   // Note that unlike TSStatic, the serverside PhysicsShape does not
   // need to play the ambient animation because even if the animation were
   // to move collision shapes it would not affect the physx representation.
	Point3F pos = getPosition();

	//TEMP! TESTING! Hope this works out. OpenSteer.
	if (mVehicle)
	{
		Point3F steerPos,steerVel;

		OpenSteer::Vec3 vec = mVehicle->position();
		OpenSteer::Vec3 vel = mVehicle->velocity();

		steerPos.set(Point3F(-vec.x,vec.z,vec.y));
		steerVel.set(Point3F(-vel.x,vel.z,vel.y));

		MatrixF pedMat,currMat;

		QuatF pedQuat,currQuat;
		Point3F pedVelNorm = steerVel;
		pedVelNorm.normalize();
		F32 pedVelLen = steerVel.len();
		//Con::printf("sceneShape %d  vector: %f  %f  %f   speed %f",
		//		mSceneShapeID,steerVel.x,steerVel.y,steerVel.z,pedVelLen);
		pedQuat.rotationArc(pedVelNorm,Point3F(0,1,0));
		pedQuat.setMatrix(&pedMat);
		setTransform(pedMat);
		setPosition(steerPos);

		//NOW: deal with walking.  First, turn on walk anim, make it cyclic, turn off ground animate. 
		//Second, adjust anim playback speed to the ratio of:
		//   OpenSteer velocity  ? (ground animate distance / sequence duration)
		//ground animate distance is the key point.  If we always start our ground animation 
		//locally at (0,0,0), which we should, then the length of the last ground frame 
		//translation should be all we need.

		//FIX: DB-ize gait velocities (per PhysicsShape?) 
		// Decide between "walk", "jog", and "run" based on pedVel.	
		F32 walkThreshold = 0.02;
		F32 runThreshold = 0.3;
		if ((pedVelLen <= mWalkSpeed - walkThreshold)&&(mCurrentSeq!=mActionSeqs["ambient"]))
		{
			//Con::printf("setting current sequence to idle!  pedVel %f",pedVelLen);
			setCurrentSeq(mActionSeqs["ambient"]);	//moveSeq = "walk";
		}
		else if ((pedVelLen > mWalkSpeed + walkThreshold)&&
					(pedVelLen <= mRunSpeed - runThreshold)&&
					(mCurrentSeq!=mActionSeqs["walk"]))
		{
			F32 threadPos = 0.0f;
			if (mCurrentSeq==mActionSeqs["run"])
				threadPos = mAmbientThread->getPos();

			//Con::printf("setting current sequence walk! currentSeq %d pedVel %f server %d theadPos %f",
			//	mCurrentSeq,pedVelLen,isServerObject(),threadPos);

			setCurrentSeq(mActionSeqs["walk"]);	//moveSeq = "walk";

			if (threadPos>0.0)
				mAmbientThread->setPos(threadPos);

		}
		else if ((pedVelLen > mRunSpeed + runThreshold)&&(mCurrentSeq!=mActionSeqs["run"]))
		{
			//Con::printf("setting current sequence run!  currentSeq %d pedVel %f server %d",
			//	mCurrentSeq,pedVelLen,isServerObject());
			F32 threadPos = 0.0f;
			if (mCurrentSeq==mActionSeqs["walk"])
				threadPos = mAmbientThread->getPos();

			setCurrentSeq(mActionSeqs["run"]);	//moveSeq = "run";
						
			if (threadPos>0.0)
				mAmbientThread->setPos(threadPos);

		}
		//Con::printf("PedVelLen: %f",pedVelLen);
		const TSShape::Sequence *kSeq = &(mShape->sequences[mCurrentSeq]);
		//FIX: add shape scale to groundTranslations!!
		
		Point3F gfPos = mShape->groundTranslations[kSeq->firstGroundFrame + (kSeq->numGroundFrames-1)];
		F32 groundSpeed = (gfPos.len() * mObjScale.z) / kSeq->duration ;//Scaling groundSpeed by object scale.
		F32 speedRatio = pedVelLen / groundSpeed;
		//F32 speedRatio = 1.0;
		TSThread *th = mShapeInstance->getThread(0);
		mShapeInstance->setTimeScale(th,speedRatio);

	}

	////// From EM  //////
	//String moveSeq;
		//if (mFlexBody->mMoveSequence.length()>0) 
		//{
		//	moveSeq = mFlexBody->mMoveSequence;
		//} else {
		//Con::printf("trying to pick an anim, pedVelLen %f server %d action walk %d action run %d",
		//				pedVelLen,isServerObject(),mActionSeqs["walk"],mActionSeqs["run"]);
			//}
		//mFlexBody->mMoveSequence = moveSeq;
		//Con::printf("move seq: %s, pedVel %f",moveSeq.c_str(),pedVelLen);
		//setSeqCyclic(mShape->findSequence(moveSeq.c_str()),true);
		//mIsGroundAnimating = false;
		//mFlexBody->playThread(0,moveSeq.c_str());
		//mFlexBody->setThreadPos(0,gRandom.randF(0.0,1.0));//random staggering of anims.

	/////
	//if (isServerObject())
	//	Con::printf("Server physics shape ticking! datablock %s position %f %f %f",mDataBlock->getName(),pos.x,pos.y,pos.z);
	//if (mCurrentTick==0)
	//{//TEMP: dealing with situations where we start with momentum and dynamic = true. Clear bodyparts and add forces.
		//if (!isServerObject())
		//{//Although perhaps if we go through and set mLastTrans[] to appropriate values, we can have initial velocities and not have
			//Con::printf("Client physics shape ticking! isArticulated %d pos %f %f %f",mIsArticulated,pos.x,pos.y,pos.z);
			//for (U32 i=0;i<mPhysicsBodies.size();i++)
			//{
			//	mCurrentForce = dynamic_cast<PhysicsShape*>(getServerObject())->mCurrentForce;
			//	Point3F forceModifier = mCurrentForce * -10;//Times minus 1 to flip it, and then reduce to taste. TEMP TEMP TEMP, testing.
			//	MatrixF tempTrans;
			//	mPhysicsBodies[i]->getTransform(&tempTrans);
			//	mLastTrans[i].setPosition(tempTrans.getPosition() + forceModifier);//maybe?
			//	Con::printf("last trans pos %f %f %f",mLastTrans[i].getPosition().x,mLastTrans[i].getPosition().y,mLastTrans[i].getPosition().z);
			//}
			//setDynamic(true);//to bother with adding forces to body parts explicitly.
			//Con::printf("Client physics shape ticking! datablock %s position %f %f %f bodyparts %d currentTick %d",mDataBlock->getName(),
			//				pos.x,pos.y,pos.z,mPhysicsBodies.size(),mCurrentTick);
			//for (U32 i=0;i<mPhysicsBodies.size();i++)
			//	Con::printf("physicsBody[%d] isDynamic %d",i,mPhysicsBodies[i]->getMass(),mPhysicsBodies[i]->isDynamic());
		//}
	//} else {
	//if (!isServerObject()) Con::printf("client object %d ticking %d physicsrep isDynamic %d isArticulated %d",
	//									getId(),mCurrentTick,mPhysicsRep->isDynamic(),mIsArticulated);
	//}

   mCurrentTick++;

   //Now, this also only makes sense on the client, for articulated shapes at least...
   if (mIsRecording)
	   recordTick();


	 ////////////////////////////////////////////////////////////////////////////////////////////////////
   //GroundMove: really belongs in the ts directory, or somewhere else, not related to physics, but testing it here.
   if ( (mIsGroundMoving) && (mCurrentSeq>=0) && !isServerObject() && (!mIsDynamic) && (!mUseDataSource))
   {
	   TSSequence currSeq = mShape->sequences[mCurrentSeq];
	   //Rots: maybe later, maybe not even necessary this time around.
	   //Quat16 groundRot = mShape->groundRotations[ambSeq.firstGroundFrame+(S32)(mAmbientThread->getPos()*(F32)(ambSeq.numGroundFrames))];
	   //QuatF groundQuat;
	   //groundRot.getQuatF(&groundQuat);
	   Point3F groundTrans = mShape->groundTranslations[currSeq.firstGroundFrame+(S32)(mAmbientThread->getPos()*(F32)(currSeq.numGroundFrames))];
	   Point3F mulGroundTrans;
	   mStartMat.mulP(groundTrans,&mulGroundTrans);
	   MatrixF m1 = getTransform();
	   Point3F pos = m1.getPosition();	  	   
	   if (mLastThreadPos > mAmbientThread->getPos())
	   {
		   mLastGroundTrans = Point3F::Zero;
		   //mLastGroundRot = MatrixF::Identity;
	   }
		
	   Point3F tempPos,finalPos,groundPos;
	   tempPos = pos + mulGroundTrans - mLastGroundTrans;
	   groundPos = findGroundPosition(tempPos + Point3F(0,0,1));
	   m1.setPosition(groundPos);
	   setTransform(m1);
	   mLastGroundTrans = mulGroundTrans;
		mLastThreadPos = mAmbientThread->getPos();
   } 

	// OpenSteer - don't know where to put this, just testing...  //////
	//if (mVehicle)
	//{
	//	OpenSteer::Vec3 vec = mVehicle->position();
	//	pos = Point3F(-vec.x,vec.z,vec.y);
	//	Con::printf("PhysicsShape OpenSteer Vehicle Pos: %f %f %f",pos.x,pos.y,pos.z);
	//}


	//HERE: NOW. We need to banish forever the switch between "isDynamic" and "not isDynamic", as a whole body property.
	//Instead, bodyparts are going to be dynamic or not, and in terms of the whole body, if we need to know our state, it
	//will be one of three states, not two:  1) fully kinematic   2) mixed dynamic/kinematic   3) fully dynamic
	//But this distinction may not turn out to be useful, trial and error will decide.
	for (U32 i=0;i<mPhysicsBodies.size();i++)
	{
		if (mPhysicsBodies[i]->isDynamic())
			updateNodeFromBody(i);
		else
			updateBodyFromNode(i);
	}//And, that should really be all there is to it...?

	
	//Except, we still need to deal with fingers/toes/etc, anything that isn't a physics body needs to be set according to its parent.
	Point3F defTrans,newPos;
	MatrixF m1,m2;
	m1.identity();
	for (U32 i=0;i<mShape->nodes.size();i++)
	{
		if (!mNodeBodies[i])
		{				  
			S32 parentIndex = mShape->nodes[i].parentIndex;
			if (parentIndex>=0)
			{
				defTrans = mShape->defaultTranslations[i];
				m2 = mShapeInstance->mNodeTransforms[parentIndex];
				m2.mulP(defTrans,&newPos);//There, that sets up our position, but now we need to grab our orientation
				m1 = m2;                       //directly from the parent node.
				m1.setPosition(newPos);
				mShapeInstance->mNodeTransforms[i] = m1;
			}
		}
	}
	/*
   ///////////////////////////////////////////////////////////////////
   //If kinematic, we need to drive physics bodyparts with nodeTransforms data.
   if ( !mPhysicsRep->isDynamic() )
   {												
		//Con::printf("mPhysicsRep is kinematic!");
	   if ( isClientObject() )//&& mIsArticulated ) 
	   {
			updateBodiesFromNodes();
	   }
	   return;
   }
   ///////////////////////////////////////////////////////////////////
   // Else if dynamic, we need to drive nodeTransforms with physics.
	
	if (  isClientObject() )// && getServerObject() )//PHYSICSMGR->isSinglePlayer() &&
   {  //SINGLE PLAYER HACK!!!!
     // PhysicsShape *servObj = (PhysicsShape*)getServerObject();
	  //if (!mIsDynamic)//relevant in any way?? this is our own bool, not testing physics body directly as above...
	  //{   //???
	//	  setTransform( servObj->mState.getTransform() ); 
	//	  mRenderState[0] = servObj->mRenderState[0];
	//	  mRenderState[1] = servObj->mRenderState[1];
	  //}
	  //if (mIsArticulated)
	  //{//need to do nodeTransform (ragdoll) work on client side, unless we can get them passed over from the server?
		updateNodesFromBodies();
	  //} 
      return;
   }
	*/
	//NOW, what the hell are we doing here? It would seem that whether we're dynamic or not, we will have returned before
	//now, unless we're A) dynamic, and B) on the server.

   // Store the last render state.
   mRenderState[0] = mRenderState[1];

   // If the last render state doesn't match the last simulation 
   // state then we got a correction and need to 
   Point3F errorDelta = mRenderState[1].position - mState.position;
   const bool doSmoothing = !errorDelta.isZero() && !smNoSmoothing;

   const bool wasSleeping = mState.sleeping;

   //TODO: handle all this for articulated bodies.
	//TEMP: getting assert in here about non dynamic body calling getState 
   if (mPhysicsRep->isDynamic())//(!mIsArticulated)
   {
	   // Get the new physics state.
	   if (1)//(!mIsArticulated)
	   {
		   if ( mPhysicsRep ) 
		   {
			   mPhysicsRep->getState( &mState );
			   _updateContainerForces();
		   } //else { // This is where we could extrapolate. (?)  //}
	   } else {
		   //do nothing?
	   }

	   // Smooth the correction back into the render state.
	   mRenderState[1] = mState;
	   if ( doSmoothing )
	   {
		   F32 correction = mClampF( errorDelta.len() / 20.0f, 0.1f, 0.9f );
		   mRenderState[1].position.interpolate( mState.position, mRenderState[0].position, correction );  
		   mRenderState[1].orientation.interpolate( mState.orientation, mRenderState[0].orientation, correction );
	   }

	   // If we haven't been sleeping then update our transform
	   // and set ourselves as dirty for the next client update.
	   if ( !wasSleeping || !mState.sleeping )//TEMP
	   {
		   // Set the transform on the parent so that
		   // the physics object isn't moved. //Hmm, is this distinction necessary anymore? Test later.
		   if (isServerObject())
			{
			   Parent::setTransform( mState.getTransform() );
				//Con::printf("setting parent transform!");
			}
		   else 
		   {
			   mPhysicsBodies[0]->getState( &mState );
			   Parent::setTransform( mState.getTransform() );
				//Con::printf("setting parent transform!");
		   }

		   // If we're doing server simulation then we need
		   // to send the client a state update.
		   if ( isServerObject() && mPhysicsRep && !smNoCorrections &&

			   !PHYSICSMGR->isSinglePlayer() // SINGLE PLAYER HACK!!!!

			   )
			   setMaskBits( StateMask );
	   }
   }   
}

void PhysicsShape::updateBodyFromNode(S32 i)
{
	if (mBodyNodes.size()==0)
		return;

	MatrixF m1,m2;
	if (i==0) //Here: we need to store our initial world position for purposes of recording sequences, 
	{  // and possibly other reasons.
		Point3F defTrans,mulDefTrans;
		defTrans = mShape->defaultTranslations[0];
		mStartMat = getTransform();		
		mStartMat.setPosition(Point3F(0,0,0));
		mStartMat.mulP(defTrans,&mulDefTrans);
		//Con::printf("bodypart 0 updating node   ");
		mStartPos = getPosition();// + mulDefTrans;//World ground position plus offset of hip node, multiplied by transform.
	}  // Ah, except below, we are adding this again. Just remember that it is not the hip node, later, it is the ground position.

	Point3F defTrans,rotPos,newPos;
	defTrans = mShape->defaultTranslations[mBodyNodes[i]];
	defTrans *= mObjScale;
	S32 parentInd = mShape->nodes[mBodyNodes[i]].parentIndex;
	if (parentInd>=0)
		m2 = mShapeInstance->mNodeTransforms[parentInd];
	else
	{
		m2.identity();
	}
	Point3F nodePos = m2.getPosition();
	nodePos *= mObjScale;
	m2.setPosition(nodePos);
	m2.mulP(defTrans,&rotPos);
	mStartMat.mulP(rotPos,&newPos);
	//newPos += ( mStartPos - mShape->defaultTranslations[0] );
	newPos += ( mStartPos );
	//newPos +=  mStartPos;

	m1 = mShapeInstance->mNodeTransforms[mBodyNodes[i]];
	m1 = mStartMat * m1;

	if (i==0) {
		Point3F nodePos = mShapeInstance->mNodeTransforms[mBodyNodes[i]].getPosition();
		nodePos *= mObjScale;
		Point3F mulPos;
		mStartMat.mulP(nodePos,&mulPos);
		//Con::printf("base node pos %f %f %f, mulPos %f %f %f m1pos %f %f %f m1Quat %f %f %f %f",
		//	nodePos.x,nodePos.y,nodePos.z,mulPos.x,mulPos.y,mulPos.z,m1Pos.x,m1Pos.y,m1Pos.z,m1Quat.x,m1Quat.y,m1Quat.z,m1Quat.w);
		//m1.setPosition((mStartPos-defTrans)+nodePos);//Well this is awkward, but mStartPos already has defTrans[0] in it.
		m1.setPosition(mStartPos + mulPos);

	}
	//if (i==0) // + mShapeInstance->mNodeTransforms[mBodyNodes[i]].getPosition() //NOPE!
	else m1.setPosition(newPos);

	mPhysicsBodies[i]->getTransform(&mLastTrans[i]);//store transform for when we go dynamic.
	mPhysicsBodies[i]->setTransform(m1);
	//}

}

void PhysicsShape::updateNodeFromBody(S32 i)
{	
	if (mBodyNodes.size()==0)
		return;
	
	Point3F defTrans,newPos,globalPos,mulPos;
	MatrixF m1,m2;

	MatrixF shapeTransform = getTransform();
	Point3F shapePos = shapeTransform.getPosition();
	shapeTransform.setPosition(Point3F(0,0,0));
	MatrixF invShapeTransform = shapeTransform;
	invShapeTransform.inverse();

	mPhysicsBodies[i]->getState(&mStates[i]);
	m1 = mStates[i].getTransform();
	globalPos = m1.getPosition();
	if (i==0) //hip node, or base node on non biped model, the only node with no joint.
	{
		//Point3F mulDefTrans;
		////mStartMat = m1;//Wait, why? I thought mStartMat was our stored initial orientation...?
		//defTrans = mShape->defaultTranslations[0];
		//newPos =  globalPos - mStartPos;
		//invShapeTransform.mulP(newPos,&mulPos);
		//invShapeTransform.mulP(defTrans,&mulDefTrans);
		m1 = invShapeTransform * m1;
		Point3F finalPos;// = mulPos - mulDefTrans;
		finalPos.zero();
		m1.setPosition(finalPos);
		mShapeInstance->mNodeTransforms[mBodyNodes[i]] = m1;

	} else { //all other nodes, position is relative to parent

		defTrans = mShape->defaultTranslations[mBodyNodes[i]];
		m2 = mShapeInstance->mNodeTransforms[mShape->nodes[mBodyNodes[i]].parentIndex];
		m2.mulP(defTrans,&newPos);
		m1 = invShapeTransform * m1;
		m1.setPosition(newPos);
		mShapeInstance->mNodeTransforms[mBodyNodes[i]] = m1;
	}
}


void PhysicsShape::advanceTime( F32 timeDelta )
{
	if ( isClientObject() && mPlayAmbient && mAmbientThread != NULL && !mIsDynamic )
      mShapeInstance->advanceTime( timeDelta, mAmbientThread );
}


//FIX: This needs updating for articulated bodies.
void PhysicsShape::_updateContainerForces()
{
   PROFILE_SCOPE( PhysicsShape_updateContainerForces );

   // If we're not simulating don't update forces.
   if ( !mWorld->isEnabled() )
      return;

   ContainerQueryInfo info;
   info.box = getWorldBox();
   info.mass = mDataBlock->mass;

   // Find and retrieve physics info from intersecting WaterObject(s)
   getContainer()->findObjects( getWorldBox(), WaterObjectType|PhysicalZoneObjectType, findRouter, &info );

   // Calculate buoyancy and drag
   F32 angDrag = mDataBlock->angularDamping;
   F32 linDrag = mDataBlock->linearDamping;
   F32 buoyancy = 0.0f;
   Point3F cmass = mPhysicsRep->getCMassPosition();

   F32 density = mDataBlock->buoyancyDensity;
   if ( density > 0.0f )
   {
      if ( info.waterCoverage > 0.0f )
      {
         F32 waterDragScale = info.waterViscosity * mDataBlock->waterDampingScale;
         F32 powCoverage = mPow( info.waterCoverage, 0.25f );

         angDrag = mLerp( angDrag, angDrag * waterDragScale, powCoverage );
         linDrag = mLerp( linDrag, linDrag * waterDragScale, powCoverage );
      }

      buoyancy = ( info.waterDensity / density ) * mPow( info.waterCoverage, 2.0f );
      
      // A little hackery to prevent oscillation
      // Based on this blog post:
      // (http://reinot.blogspot.com/2005/11/oh-yes-they-float-georgie-they-all.html)
      // JCF: disabled!
      Point3F buoyancyForce = buoyancy * -mWorld->getGravity() * TickSec * mDataBlock->mass;
      mPhysicsRep->applyImpulse( cmass, buoyancyForce );      
   }

   // Update the dampening as the container might have changed.
   mPhysicsRep->setDamping( linDrag, angDrag );
   
   // Apply physical zone forces.
   if ( !info.appliedForce.isZero() )
      mPhysicsRep->applyImpulse( cmass, info.appliedForce );
}

void PhysicsShape::prepRenderImage( SceneRenderState *state )
{
   AssertFatal( !mDestroyed, "PhysicsShape::prepRenderImage - Shouldn't be processing a destroyed shape!" );

   PROFILE_SCOPE( PhysicsShape_prepRenderImage );

   if( !mShapeInstance )
         return;

   Point3F cameraOffset;
   getRenderTransform().getColumn(3,&cameraOffset);
   cameraOffset -= state->getDiffuseCameraPosition();
   F32 dist = cameraOffset.len();
   if (dist < 0.01f)
      dist = 0.01f;

   F32 invScale = (1.0f/getMax(getMax(mObjScale.x,mObjScale.y),mObjScale.z));   
   if ( mShapeInstance->setDetailFromDistance( state, dist * invScale ) < 0 )
      return;

   GFXTransformSaver saver;

   // Set up our TS render state.
   TSRenderState rdata;
   rdata.setSceneState( state );
   rdata.setFadeOverride( 1.0f );

   // We might have some forward lit materials
   // so pass down a query to gather lights.
   LightQuery query;
   query.init( getWorldSphere() );
   rdata.setLightQuery( &query );

   MatrixF mat = getRenderTransform();
   mat.scale( mObjScale );
   GFX->setWorldMatrix( mat );

   mShapeInstance->animate();
   mShapeInstance->render( rdata );
}

void PhysicsShape::destroy()
{
   if ( mDestroyed )
      return;

   mDestroyed = true;
   setMaskBits( DamageMask );

   const Point3F lastLinVel = mPhysicsRep->isDynamic() ? mPhysicsRep->getLinVelocity() : Point3F::Zero;

   // Disable all simulation of the body... no collision or dynamics.
  // mPhysicsRep->setSimulationEnabled( false );//This should now be taken care of below, as mPhysicsBodies[0]

   //if ( mIsArticulated )
	for (U32 i=0;i<mPhysicsBodies.size();i++)
		mPhysicsBodies[i]->setSimulationEnabled( false );

   // On the client side we remove it from the scene graph
   // to disable rendering and volume queries.
   if ( isClientObject() )
      removeFromScene();

   // Stop doing tick processing for this SceneObject.
   setProcessTick( false );

   if ( !mDataBlock )
      return;

   const MatrixF &mat = getTransform();
   if ( isServerObject() )
   {
      // We only create the destroyed object on the server
      // and let ghosting deal with updating the client.

      if ( mDataBlock->destroyedShape )
      {
         mDestroyedShape = new PhysicsShape();
         mDestroyedShape->setDataBlock( mDataBlock->destroyedShape );
         mDestroyedShape->setTransform( mat );
         if ( !mDestroyedShape->registerObject() )
            delete mDestroyedShape.getObject();
      }

      return;
   }
   
   // Let the physics debris create itself.
   PhysicsDebris::create( mDataBlock->debris, mat, lastLinVel );

   if ( mDataBlock->explosion )
   {
      Explosion *splod = new Explosion();
      splod->setDataBlock( mDataBlock->explosion );
      splod->setTransform( mat );
      splod->setInitialState( getPosition(), mat.getUpVector(), 1.0f );
      if ( !splod->registerObject() )
         delete splod;
   }   
}

void PhysicsShape::restore()
{
   if ( !mDestroyed )
      return;

   const bool isDynamic = mDataBlock && mDataBlock->mass > 0.0f;

   if ( mDestroyedShape )   
      mDestroyedShape->deleteObject();

   // Restore tick processing, add it back to 
   // the scene, and enable collision and simulation.
   setProcessTick( isDynamic || mPlayAmbient );   
   if ( isClientObject() )
      addToScene();
   mPhysicsRep->setSimulationEnabled( true );

   mDestroyed = false;
   setMaskBits( DamageMask );
}

PhysicsBody* PhysicsShape::getPhysicsRep()
{
	return mPhysicsRep;
}

void PhysicsShape::setJointTarget(QuatF &target)
{
	if (mJoint)
		mJoint->setMotorTarget(target);

	Con::printf("physicsShape set motor target: %f %f %f %f",target.x,target.y,target.z,target.w);

}

void PhysicsShape::setHasGravity(bool hasGrav)
{	
	if (isServerObject())
	{
		PhysicsShape *clientShape = dynamic_cast<PhysicsShape *>(getClientObject());//SINGLE PLAYER HACK
		clientShape->setHasGravity(hasGrav);
		return;
	}

	for (U32 i=0;i<mPhysicsBodies.size();i++)
	{
		mPhysicsBodies[i]->setHasGravity(hasGrav);
	}
	
}

void PhysicsShape::setPartHasGravity(S32 partID,bool hasGrav)
{
	if ( (partID<0) )
		return;
	
	if (isServerObject())
	{        //SINGLE PLAYER HACK
		PhysicsShape *clientShape = dynamic_cast<PhysicsShape *>(getClientObject());
		clientShape->setPartHasGravity(partID,hasGrav);
		return;
	}
	if (partID < mPhysicsBodies.size())//Note: can't do this check above, because the server will only have one bodypart, for now.
		mPhysicsBodies[partID]->setHasGravity(hasGrav);
}

void PhysicsShape::setDynamic(bool isDynamic)
{
	if (mIsDynamic != isDynamic)
	{
		mIsDynamic = isDynamic;

		if (isServerObject())
		{   //SINGLE PLAYER HACK - really we need to do it to all ghosts of this object.
			PhysicsShape *clientShape = dynamic_cast<PhysicsShape *>(getClientObject());
			clientShape->setDynamic(isDynamic);
			return;
		}
		//Con::printf("Client object setting dynamic, articulated %d",mIsArticulated);
		//if (!mIsArticulated)
		//{
		//	mPhysicsRep->setDynamic(isDynamic);
			//HERE: add linear/angular velocity if (isDynamic==true) and we were animating/moving before.
		//} else { //Now mPhysicsBodies[0] == mPhysicsRep, so skip above.
		for (U32 i=0;i<mPhysicsBodies.size();i++)
		{
			mPhysicsBodies[i]->setDynamic(isDynamic);
			if (isDynamic)
			{//Add linear/angular velocity
				Point3F posVel;
				MatrixF curTrans,diffTrans,invLastTrans;
				mPhysicsBodies[i]->getTransform(&curTrans);
				posVel = curTrans.getPosition() - mLastTrans[i].getPosition();

				posVel *= 32;//Times tick rate per second
				mPhysicsBodies[i]->setLinVelocity(posVel);
				//Con::printf("setting body %d with linVel %f %f %f",i,posVel.x,posVel.y,posVel.z);

				//if (0)//hmm, did I even finish this? where is curTrans coming from?
				//{
				//	EulerF angVel;
				//	mLastTrans[i].invertTo(&invLastTrans);
				//	diffTrans = curTrans * invLastTrans;
				//	angVel = diffTrans.toEuler();

				//	//angVel *= 180.0/M_PI;//Maybe?
				//	angVel *= 32;//Times tick rate per second.
				//	mPhysicsBodies[i]->setAngVelocity(angVel);
				//	Con::printf("setting velocities: lin %f %f %f  ang %f %f %f",posVel.x,posVel.y,posVel.z,angVel.x,angVel.y,angVel.z);
				//	//Very difficult to see effect, but setAngVelocity does work. Not sure how to turn it up.
				//}

				mShapeInstance->mDynamicNodes.set(mPhysicsBodies[i]->getNodeIndex());
				QuatF q;  q.identity();//Just to initialize them...
				mShapeInstance->mDynamicNodeRotations[mPhysicsBodies[i]->getNodeIndex()] = q;

			} else {
				
				mShapeInstance->mDynamicNodes.clear(mPhysicsBodies[i]->getNodeIndex());

			}
		}
		//}
	}
}


bool PhysicsShape::getDynamic()
{
	return mIsDynamic;
}

void PhysicsShape::setPartDynamic(S32 partID,bool isDynamic)
{
	//set or clear dynamic for partID bodypart, NOTE this is NOT database partID, it is index into this shape's bodies.
	if ( partID<0 )
		return;

	if (isServerObject())
	{     //SINGLE PLAYER HACK
		PhysicsShape *clientShape = dynamic_cast<PhysicsShape *>(getClientObject());
		clientShape->setPartDynamic(partID,isDynamic);
		return;
	}

	if (partID >= mPhysicsBodies.size())
		return;

	if (mPhysicsBodies[partID]->isDynamic() != isDynamic)
	{
		mPhysicsBodies[partID]->setDynamic(isDynamic);

		if (isDynamic)
		{
			mShapeInstance->mDynamicNodes.set(mPhysicsBodies[partID]->getNodeIndex());
			QuatF q;  q.identity();
			mShapeInstance->mDynamicNodeRotations[mPhysicsBodies[partID]->getNodeIndex()] = q;
		}
		else
		{
			mShapeInstance->mDynamicNodes.clear(mPhysicsBodies[partID]->getNodeIndex());
		}
	}
}

bool PhysicsShape::getPartDynamic(S32 partID)
{
	if (isServerObject())
	{        //SINGLE PLAYER HACK
		PhysicsShape *clientShape = dynamic_cast<PhysicsShape *>(getClientObject());
		return clientShape->getPartDynamic(partID);
	}

	if (partID<mPhysicsBodies.size())
		return mPhysicsBodies[partID]->isDynamic();
	else
		return false;	 
}

S32 PhysicsShape::getContactBody()
{
	return mContactBody;
}

void PhysicsShape::setPosition(Point3F pos)
{
	if (isServerObject())
	{        //SINGLE PLAYER HACK
		PhysicsShape *clientShape = dynamic_cast<PhysicsShape *>(getClientObject());
		if (clientShape) 
			clientShape->setPosition(pos);
		//So weird, can't find MoveMask anymore?? How and where does Torque handle normal move ghosting again? Hmmm
	}

	MatrixF shapeTransform = getTransform();	
	shapeTransform.setPosition(pos);
	setTransform(shapeTransform);
	
	return;
}

/////////////////////////////////////////////////////////////

bool PhysicsShape::setCurrentSeq(S32 seq)
{	
	if (isServerObject())
	{
		PhysicsShape *clientShape = dynamic_cast<PhysicsShape *>(getClientObject());
		return clientShape->setCurrentSeq(seq);
	}

	if ((seq >= 0)&&(seq < mShapeInstance->getShape()->sequences.size()))
	{
		mCurrentSeq = seq;
		_setCurrent();
		return true;
	} else {
		return false;
	}
}

bool PhysicsShape::setActionSeq(const char *name,S32 seq)
{
	if ((seq >= 0)&&(seq < mShapeInstance->getShape()->sequences.size())&&(dStrlen(name)>0))
	{
		mActionSeqs[name] = seq;
		if (isServerObject())
		{
			PhysicsShape *clientShape = dynamic_cast<PhysicsShape *>(getClientObject());
			clientShape->setActionSeq(name,seq);
		}
		return true;
	} else return false;
}

bool PhysicsShape::setAmbientSeq(S32 seq)
{
	if (isServerObject())
	{
		PhysicsShape *clientShape = dynamic_cast<PhysicsShape *>(getClientObject());
		clientShape->setAmbientSeq(seq);
	}

	if ((seq >= 0)&&(seq < mShapeInstance->getShape()->sequences.size()))
	{
		mAmbientSeq = seq;
		_initAmbient();
		return true;
	} else {
		return false;
	}
}

////////////////////////////////////////////////


////////////////////////////////////////////////////


bool PhysicsShape::setAmbientSeq(const char *name)
{
	if (isServerObject())
	{
		PhysicsShape *clientShape = dynamic_cast<PhysicsShape *>(getClientObject());
		clientShape->setAmbientSeq(name);
	}

	S32 seq = mDataBlock->shape->findSequence( name );
	if ((seq >= 0)&&(seq < mShapeInstance->getShape()->sequences.size()))
	{
		mAmbientSeq = seq;
		if (!isServerObject())
			_initAmbient();
		return true;
	} else {
		return false;
	}
}

////////////////////////////////////////////////////

void PhysicsShape::orientToPosition(Point3F pos)
{
	if (isServerObject())
	{
		PhysicsShape *clientShape = dynamic_cast<PhysicsShape *>(getClientObject());
		clientShape->orientToPosition(pos);
	}
	//Do this for both client and server.
		
	Point3F bodyPos = getPosition();
	Point3F diff = pos - bodyPos;
	Point3F dir;
	QuatF arc;

	F32 moveThreshold = 0.05;//(set this in script)
	if ((diff.len()>0.05))//(0.05 = move threshold,
	{
		//Con::errorf("setting move target to target body position: %f %f %f",
		//	   mMoveTarget.x,mMoveTarget.y,mMoveTarget.z);

		Point3F diffNorm = diff;
		diffNorm.z = 0.0;
		diffNorm.normalize();
		//QuatF q(mat);
		//q.mulP(Point3F(0,1,0),&dir);//dir should now be Y vector multiplied by our world transform, i.e. the direction we are facing.

		dir.set(0,1,0);
		//Now, rotate around the Z axis so that our Y axis points at the target.
		dir.normalize(); //just to make sure, should already be.

		arc.shortestArc(diffNorm,dir);
		if (!(mIsNaN_F(arc.x)||mIsNaN_F(arc.y)||mIsNaN_F(arc.z)||mIsNaN_F(arc.w)))
		{
			MatrixF mat;
			arc.setMatrix(&mat);
			Point3F myPos = getPosition();
			mat.setPosition(myPos);
			setTransform(mat);
		}
	}
}

Point3F PhysicsShape::getClientPosition()
{
	if (isServerObject())
	{
		PhysicsShape *clientShape = dynamic_cast<PhysicsShape *>(getClientObject());
		return clientShape->getTransform().getPosition();
	} else return(Point3F(0,0,0));//should never happen.
}

Point3F PhysicsShape::findGroundPosition(Point3F pos)
{
	RayInfo ri;
	bool raySuccess = mWorld->castGroundRay(pos + Point3F(0,0,30),pos + Point3F(0,0,-1000),&ri);

	if (raySuccess)
		return ri.point;
	else 
	{
		Con::printf("ground raycast failed from %f %f %f",pos.x,pos.y,pos.z);
		return pos;
	}
}


////  UNTESTED  ///////////////////////
bool PhysicsShape::loadSequence(const char *dsqPath)
{
	//Here:  open the file, and give the shape->importSequences function a filestream, and you're done.
	FileStream  fileStream;

	//String dsqFilename = dsqPath;
	/*
	String relativePath;
	S32 gamePos = dsqFilename.find("game/art",0);//?? Arbitrary, FIX!
	if (gamePos > -1)
		relativePath = dsqFilename.substr(gamePos + 5);
	else
		relativePath = dsqFilename;
	//FIRST: find the sequence name from the database, given this filename and skeleton id, and make
	//sure we haven't already loaded it.
	char seq_name[255];
	S32 seq_id = 0;
	SQLiteObject *sql = new SQLiteObject();
	if (sql)
	{
		if (sql->OpenDatabase(dynamic_cast<physManagerCommon*>(mPM)->mDatabaseName.c_str()))
		{
			char sequence_query[512];
			int result,sequence_id=0;
			sqlite_resultset *resultSet;
			sprintf(sequence_query,"SELECT name,id FROM sequence WHERE filename LIKE '%s' AND skeleton_id=%d",
				relativePath.c_str(),mSkeletonID);
			result = sql->ExecuteSQL(sequence_query);//Should really keep sequence ids around somewhere, but just looking them up for now.
			if (result)
			{
				resultSet = sql->GetResultSet(result);
				if (resultSet->iNumRows == 1)
				{
					sprintf(seq_name,"%s",resultSet->vRows[0]->vColumnValues[0]);
					seq_id = dAtoi(resultSet->vRows[0]->vColumnValues[1]);
				}
			}
			sql->CloseDatabase();
		}
		delete sql;
	}
	*/
	const String myPath = mShapeInstance->getShapeResource()->getPath().getPath();
	const String myFileName = mShapeInstance->getShapeResource()->getPath().getFileName();
	const String myFullPath = mShapeInstance->getShapeResource()->getPath().getFullPath();
	
	Con::printf("shape adding sequence, myPath %s, myFileName %s fullPath %s",myPath.c_str(),myFileName.c_str(),myFullPath.c_str());

	//S32 seqindex = mShape->findSequence(seq_name);
	//if (seqindex>=0)
	//{
	//	Con::errorf("trying to load a sequence that is already loaded: %s",seq_name);
	//	return false;
	//}

	fileStream.open(dsqPath, Torque::FS::File::Read);

	if (fileStream.getStatus() != Stream::Ok)
	{
		Con::errorf("Missing sequence %s",dsqPath);
		return false;
	}
	if (!mShape->importSequences(&fileStream,myPath) || fileStream.getStatus()!= Stream::Ok)
	{
		fileStream.close();
		Con::errorf("Load sequence %s failed",dsqPath);
		return false;
	}
	fileStream.close();
	Con::printf("Load sequence succeeded?  %s",dsqPath);
	
}

//Hm, do we have a version of this in TSShape that permanently sets the base sequence timescale? If not, we should.
void PhysicsShape::setSequenceTimeScale(F32 timescale)
{
	if (isServerObject())
	{
		PhysicsShape *clientObj = (PhysicsShape*)getClientObject();
		clientObj->setSequenceTimeScale(timescale);
	}

	if (mAmbientThread)
	{
		mShapeInstance->setTimeScale(mAmbientThread,timescale);
	}
}

void PhysicsShape::setSequencePos(F32 pos)
{
	if (isServerObject())
	{
		PhysicsShape *clientObj = (PhysicsShape*)getClientObject();
		clientObj->setSequencePos(pos);
	}

	if (mAmbientThread)
	{
		mShapeInstance->setPos(mAmbientThread,pos);
	}
}

F32 PhysicsShape::getSequencePos()
{
	if (isServerObject())
	{
		PhysicsShape *clientObj = (PhysicsShape*)getClientObject();
		return clientObj->getSequencePos();
	}

	if (mAmbientThread)
	{
		return mShapeInstance->getPos(mAmbientThread);
	}
}


void PhysicsShape::cropSequence(U32 seq,F32 start,F32 stop,const char *name)
{
	mShape->cropSequence(seq,start,stop,name);

	FileStream *outstream;
	String dsqPath(name);
	String dsqExt(".dsq");
	if (!dStrstr(dsqPath.c_str(),dsqExt.c_str())) dsqPath += dsqExt;
	if ((outstream = FileStream::createAndOpen( dsqPath.c_str(), Torque::FS::File::Write))==NULL) {
		Con::printf("whoops, name no good: %s!",dsqPath.c_str()); 
	} else {
		TSShape::Sequence & seq = mShape->sequences.last();
		mShape->exportSequence((Stream *)outstream,seq,1);
		outstream->close();
	}
}

DefineEngineMethod( PhysicsShape, cropSequence, void, (S32 seq,F32 start,F32 stop,const char *name),,"")
{  
	object->cropSequence(seq,start,stop,name);
}


DefineEngineMethod( PhysicsShape, dropSequence, void, (S32 seq),,"")
{  
	object->mShape->dropSequence(seq);
}

DefineEngineMethod( PhysicsShape, groundCaptureSeq, void, (S32 seq),,"")
{  
	object->mShape->groundCaptureSeq(seq);
}


DefineEngineMethod( PhysicsShape, unGroundCaptureSeq, void, (S32 seq),,"")
{  
	object->mShape->unGroundCaptureSeq(seq);
}



//////////////////////////////////////////////////////////////
//ultraframe types: 0=ADJUST_NODE_POS, 1=SET_NODE_POS, 2=ADJUST_NODE_ROT, 3=SET_NODE_ROT

void PhysicsShape::addUltraframeSet(const char *name)//path)
{
	//And, whoops! We have the file path, but findSequence only works with the short name.
	//I don't think I want that. So, need to find the sequence via the path. Hmm.

	//mUFSet.sequence = mShape->findSequenceByPath(path);
	mUFSet.sequence = mShape->findSequence(name);
	mUFSet.series.clear();
}

void PhysicsShape::addUltraframeSeries(S32 type,S32 node)
{
	ultraframeSeries ufs;
	ufs.type = type;
	ufs.node = node;
	mUFSet.series.push_back(ufs);
}

void PhysicsShape::addUltraframe(S32 frame,F32 x,F32 y,F32 z)
{
	//Now, we are assuming that we are only doing this all at once, so all ultraframes are going to be
	//added in order to the last series added.
	ultraframe uf;
	uf.frame = frame;
	uf.value.x = x;
	uf.value.y = y;
	uf.value.z = z;
	mUFSet.series.last().frames.push_back(uf);
}

void PhysicsShape::applyUltraframeSet()
{
	S32 seq;
	ultraframeSet ufs;
	mShape->applyUltraframeSet(&mUFSet);
}

//////////////////////////////////////////////////////////////

DefineEngineMethod( PhysicsShape, addKeyframeSet, void, (const char *path),,"")
{  
	object->addUltraframeSet(path);
}

DefineEngineMethod( PhysicsShape, addKeyframeSeries, void, (S32 type,S32 node),,"")
{  
	object->addUltraframeSeries(type,node);
}

DefineEngineMethod( PhysicsShape, addKeyframe, void, (S32 frame,F32 x,F32 y,F32 z),,"")
{  
	object->addUltraframe(frame,x,y,z);
}

DefineEngineMethod( PhysicsShape, applyKeyframeSet, void, (),,"")
{  
	object->applyUltraframeSet();
}



////////////////////////////////////////////////////////////////////
//Temp, aircraft experimentation section.

void PhysicsShape::loadXml(const char *file)
{
	SimXMLDocument *doc = new SimXMLDocument();
	doc->registerObject();

	S32 loaded = doc->loadFile(file);
	if (loaded) 
	{
		Con::printf("we loaded an xml file!!!!!!!!!!!!!!!!!");
		
		doc->pushFirstChildElement("PropertyList");
		
		doc->pushFirstChildElement("path");

		Con::printf("path: %s",doc->getData());

		while(doc->nextSiblingElement("model"))
		{
			doc->pushFirstChildElement("offsets");
			doc->pushFirstChildElement("x-m");			
			Con::printf("x: %f",dAtof(doc->getData()));
			doc->nextSiblingElement("y-m");	
			Con::printf("y: %f",dAtof(doc->getData()));
			doc->nextSiblingElement("z-m");	
			Con::printf("z: %f",dAtof(doc->getData()));

			doc->popElement();
		}
	}
}

void PhysicsShape::setRotorTransparency(F32 rpm)
{
}

void PhysicsShape::setPropTransparency(F32 rpm)
{
}


void PhysicsShape::showPropBlades()
{

}

void PhysicsShape::showPropBlur()
{

}

void PhysicsShape::showPropDisc()
{

}
////////////////////////////////////////////////////////////////////

//TEMP: we need to start deriving vehicle etc classes from PhysicsShape, it seems. AND, need to fix this so it doesn't 
//assume mounted rotors, assume nodes on the model first, and mounted rotors as a special case.
void PhysicsShape::showRotorBlades()
{//FIX: Now I need to find my mounted rotor, and then just run through all of its mounted objects, because they're all blades.
	if (getMountedObjectCount()>0)
	{
		for (U32 i=0;i<getMountedObjectCount();i++)
		{
			TSStatic *rotor = dynamic_cast<TSStatic *>(getMountedObject(i));
			if (rotor)
			{
				if (strstr(rotor->mShapeName,"MainRotor")>0)
				{
					for (U32 j=0;j<rotor->getMountedObjectCount();j++)
					{
						TSStatic *blade = dynamic_cast<TSStatic *>(rotor->getMountedObject(j));
						if (blade)
						{
							TSShapeInstance *shpInst = blade->getShapeInstance();
							for (U32 k=0;k<8;k++)
							{
								blade->setMeshHidden(k,1);
							}
							for (U32 k=8;k<12;k++)
							{
								blade->setMeshHidden(k,0);
								shpInst->mMeshObjects[k].visible = 1.0;
							}
						}
					}
				}
			}
		}
	}
}

void PhysicsShape::showRotorBlur()
{
	if (getMountedObjectCount()>0)
	{
		for (U32 i=0;i<getMountedObjectCount();i++)
		{
			TSStatic *rotor = dynamic_cast<TSStatic *>(getMountedObject(i));
			if (rotor)
			{
				if (strstr(rotor->mShapeName,"MainRotor")>0)
				{
					for (U32 j=0;j<rotor->getMountedObjectCount();j++)
					{
						TSStatic *blade = dynamic_cast<TSStatic *>(rotor->getMountedObject(j));
						if (blade)
						{
							TSShapeInstance *shpInst = blade->getShapeInstance();
							for (U32 k=0;k<4;k++)
							{
								blade->setMeshHidden(k,1);
							}
							for (U32 k=4;k<8;k++)
							{
								blade->setMeshHidden(k,0);
								shpInst->mMeshObjects[k].visible = mPropBlurAlpha;//Hmm, this better be DB as well.
								//shpInst->mMeshObjects[k].
							}
							for (U32 k=8;k<12;k++)
							{
								blade->setMeshHidden(k,1);
							}
						}
					}
				}
			}
		}
	}
}

void PhysicsShape::showRotorDisc()
{
	if (getMountedObjectCount()>0)
	{
		for (U32 i=0;i<getMountedObjectCount();i++)
		{
			TSStatic *rotor = dynamic_cast<TSStatic *>(getMountedObject(i));
			if (rotor)
			{
				if (strstr(rotor->mShapeName,"MainRotor")>0)
				{
					for (U32 j=0;j<rotor->getMountedObjectCount();j++)
					{
						TSStatic *blade = dynamic_cast<TSStatic *>(rotor->getMountedObject(j));
						if (blade)
						{
							TSShapeInstance *shpInst = blade->getShapeInstance();
							for (U32 k=0;k<4;k++)
							{
								blade->setMeshHidden(k,0);
								shpInst->mMeshObjects[k].visible = mPropDiscAlpha;//Hmm, this better be DB as well.
							}
							for (U32 k=4;k<12;k++)
							{
								blade->setMeshHidden(k,1);
							}
						}
					}
				}
			}
		}
	}
}


/////////////////////////////////////////////////////////////////////////////////////////







//////////////////////////////////////////////////////////////////////////////////////////
DefineEngineMethod( PhysicsShape, loadSequence, void, (const char *path),,
   "@brief.\n\n")
{  
	S32 seqID = object->loadSequence(path);
	//object->setCurrentSeq(seqID);
}

DefineEngineMethod( PhysicsShape, getPath, const char *, (),,
   "@brief.\n\n")
{  
	return object->mShapeInstance->getShapeResource()->getPath().getPath();
}

DefineEngineMethod( PhysicsShape, getFile, const char *, (),,
   "@brief.\n\n")
{  
	return object->mShapeInstance->getShapeResource()->getPath().getFileName();
}

DefineEngineMethod( PhysicsShape, getFullPath, const char *, (),,
   "@brief.\n\n")
{  
	return object->mShapeInstance->getShapeResource()->getPath().getFullPath();
}

////////////////////////////////////////////////////////////////////////

DefineEngineMethod( PhysicsShape, isDestroyed, bool, (),, 
   "@brief Returns if a PhysicsShape has been destroyed or not.\n\n" )
{
   return object->isDestroyed();
}

DefineEngineMethod( PhysicsShape, destroy, void, (),,
   "@brief Disables rendering and physical simulation.\n\n"
   "Calling destroy() will also spawn any explosions, debris, and/or destroyedShape "
   "defined for it, as well as remove it from the scene graph.\n\n"
   "Destroyed objects are only created on the server. Ghosting will later update the client.\n\n"
   "@note This does not actually delete the PhysicsShape." )
{
   object->destroy();
}

DefineEngineMethod( PhysicsShape, restore, void, (),,
   "@brief Restores the shape to its state before being destroyed.\n\n"
   "Re-enables rendering and physical simulation on the object and "
   "adds it to the client's scene graph. "
   "Has no effect if the shape is not destroyed.\n\n")
{
   object->restore();
}

DefineEngineMethod( PhysicsShape, jointAttach, void, (U32 objectID,U32 jointID),,
   "@brief Attaches this object to the other object by creating a physx joint "
   "of the given type.\n\n")
{
	U32 myID = object->getId();
	
	SimObject *otherObj = Sim::findObject(objectID);
	if (strcmp(otherObj->getClassName(),"PhysicsShape"))
		Con::printf("Other object is not a PhysicsShape!");
	else
	{
		PhysicsBody *otherBody = dynamic_cast<PhysicsShape*>(otherObj)->getPhysicsRep();
		MatrixF tA,tB;
		object->getPhysicsRep()->getTransform(&tA);
		otherBody->getTransform(&tB);

		Point3F posA = tA.getPosition();
		Point3F posB = tB.getPosition();

		Point3F diff = posA - posB;
		Point3F center = posB + (diff/2);

		MatrixF baseMat;
		baseMat.identity();

		PhysicsJoint *kJoint =  PHYSICSMGR->createJoint(object->getPhysicsRep(),otherBody,jointID,center,Point3F(0,0,0),Point3F(0,0,0),baseMat);
		object->mJoint = kJoint;
	}
}

DefineEngineMethod( PhysicsShape, setJointTarget, void, (F32 x,F32 y,F32 z,F32 w),,
   "@brief Sets this object's joint motor drive to the target quat.\n\n")
{
	object->setJointTarget(QuatF(x,y,z,w));
}

DefineEngineMethod( PhysicsShape, applyImpulse, void, (Point3F pos,Point3F vec),,
   "@brief Applies vec impulse to object at pos.\n\n")
{
   object->applyImpulse(pos,vec);
}

DefineEngineMethod( PhysicsShape, applyImpulseToPart, void, (S32 partIndex,Point3F pos,Point3F vec),,
   "@brief Applies vec impulse to object at pos.\n\n")
{
	Con::printf("applying impulse to part! part %d vec %f %f %f",partIndex,vec.x,vec.y,vec.z);
   object->applyImpulseToPart(partIndex,pos,vec);
}

DefineEngineMethod( PhysicsShape, aitp, void, (S32 partIndex,Point3F vec),,
   "@brief Applies vec impulse to object at pos.\n\n")
{
   object->applyImpulseToPart(partIndex,Point3F(0,0,0),vec);
}

DefineEngineMethod( PhysicsShape, applyRadialImpulse, void, (Point3F origin,F32 radius,F32 magnitude),,
   "@brief Applies vec impulse to object at pos.\n\n")
{
   object->applyRadialImpulse(origin,radius,magnitude);
}

DefineEngineMethod( PhysicsShape, applyRadialImpulseToPart, void, (S32 partIndex,Point3F origin,F32 radius,F32 magnitude),,
   "@brief Applies vec impulse to object at pos.\n\n")
{
   object->applyRadialImpulseToPart(partIndex,origin,radius,magnitude);
}

DefineEngineMethod( PhysicsShape, setDynamic, void, (bool isDynamic),,
   "@brief Sets this object's isDynamic property.\n\n")
{ 

	object->setDynamic(isDynamic);
}

DefineEngineMethod( PhysicsShape, setPartDynamic, void, (int partID,bool isDynamic),,
   "@brief Sets this part's isDynamic property.\n\n")
{  
	object->setPartDynamic(partID,isDynamic);
}

DefineEngineMethod( PhysicsShape, setHasGravity, void, (bool hasGravity),,
   "@brief Sets this object's hasGravity property.\n\n")
{ 
	object->setHasGravity(hasGravity);
}

DefineEngineMethod( PhysicsShape, setPartHasGravity, void, (int partID,bool hasGravity),,
   "@brief Sets this part's hasGravity property.\n\n")
{  
	object->setPartHasGravity(partID,hasGravity);
}

DefineEngineMethod( PhysicsShape, getContactBody, S32, (),,
   "@brief Gets this shape's latest ContactBody.\n\n")
{  
	return object->getContactBody();
}

DefineEngineMethod( PhysicsShape, setPosition, void, (Point3F pos),,
   "@brief Sets position.\n\n")
{  
	object->setPosition(pos);
}

DefineEngineMethod( PhysicsShape, groundMove, void, (),,
   "@brief starts movement via ground transforms.\n\n")
{  
	object->mIsGroundMoving = true;
	PhysicsShape *clientShape = dynamic_cast<PhysicsShape *>(object->getClientObject());
	clientShape->mIsGroundMoving = true;
	Con::printf("ground moving is set!");
}

DefineEngineMethod( PhysicsShape, getSceneShapeID, S32, (),,
   "@brief \n\n")
{  
	return object->mSceneShapeID;	
}

///////////////////////////////////////////////////////

DefineEngineMethod( PhysicsShape, setActionSeq, bool, (const char *name,const char *seqname),,
   "@brief \n\n")
{  
	S32 seq = object->mShapeInstance->getShape()->findSequence(seqname);
	if ((seq >= 0)&&(strlen(name)>0))
	{
		object->setActionSeq(name,seq);
		return true;
	} else return false;

}

DefineEngineMethod( PhysicsShape, getActionSeq, S32, (const char *name),,
   "@brief \n\n")
{  	
	return object->mActionSeqs[name];
}

//Call this at runtime to actually use the action.
DefineEngineMethod( PhysicsShape, actionSeq, void, (const char *name),,
   "@brief.\n\n")
{  
	object->setCurrentSeq(object->mActionSeqs[name]);
}

///////////////////////////////////////////////////////

DefineEngineMethod( PhysicsShape, setAmbientSeq, bool, (S32 seq),,
   "@brief \n\n")
{  
	if ((seq >= 0)&&(seq < object->mShapeInstance->getShape()->sequences.size())) 
	{
		object->setAmbientSeq(seq);
		return true;
	} else return false;
}

DefineEngineMethod( PhysicsShape, getAmbientSeq, S32, (),,
   "@brief \n\n")
{  
	return object->mAmbientSeq;	
}

DefineEngineMethod( PhysicsShape, getAmbientSeqName, const char *, (),,
   "@brief \n\n")
{  
	return object->mShapeInstance->getShape()->getSequenceName(object->mAmbientSeq);
}

//HERE: all of these should be stored in the DB as actionSequences.
DefineEngineMethod( PhysicsShape, setAmbientSeqByName, bool, (const char* name),,
   "@brief \n\n")
{  
	return object->setAmbientSeq(name);
}

/////////////////////////////////////////////////////

DefineEngineMethod( PhysicsShape, orientToPos, void, (Point3F pos),,
   "@brief.\n\n")
{  
	object->orientToPosition(pos);
}

DefineEngineMethod( PhysicsShape, getClientPosition, Point3F, (),,
   "@brief.\n\n")
{  
	return object->getClientPosition();
}

DefineEngineMethod( PhysicsShape, findGroundPosition, Point3F, (Point3F pos),,
   "@brief.\n\n")
{  
	return object->findGroundPosition(pos);
}

DefineEngineMethod( PhysicsShape, playSeq, void, (const char *name),,
   "@brief.\n\n")
{  
	S32 seqID = object->mShapeInstance->getShape()->findSequence(name);
	object->setDynamic(0);
	object->setCurrentSeq(seqID);
}

DefineEngineMethod( PhysicsShape, playSeqByNum, void, (S32 index),,
   "@brief.\n\n")
{  
	object->setDynamic(0);
	object->setCurrentSeq(index);
}

DefineEngineMethod( PhysicsShape, showSeqs, void, (),,
   "@brief.\n\n")
{	
	TSShape *kShape = object->mShape;
	Con::errorf("ground Rotations: %d, translations %d",kShape->groundRotations.size(),
						kShape->groundTranslations.size());
	for (U32 i=0;i<kShape->sequences.size();i++)
	{
		TSShape::Sequence & seq = kShape->sequences[i];

		Con::printf("Seq[%d] %s frames: %d duration %3.2f baseObjectState %d baseScale %d baseDecalState %d toolbegin %f",
			i,kShape->getName(seq.nameIndex).c_str(),seq.numKeyframes,
			seq.duration,kShape->sequences[i].baseObjectState,kShape->sequences[i].baseScale,
			kShape->sequences[i].baseDecalState,seq.toolBegin);
		Con::printf("   groundFrames %d first %d isBlend %d isCyclic %d flags %d",
			seq.numGroundFrames,seq.firstGroundFrame,seq.isBlend(),seq.isCyclic(),seq.flags);
	}
}

DefineEngineMethod( PhysicsShape, getNumSeqs, S32, (),,
   "@brief.\n\n")
{  

	return object->mShape->sequences.size();
}


DefineEngineMethod( PhysicsShape, getSeqName, const char*, (S32 index),,
   "@brief.\n\n")
{  
	TSShape::Sequence & seq = object->mShape->sequences[index];
	return object->mShape->getName(seq.nameIndex).c_str();
}

DefineEngineMethod( PhysicsShape, getSeqNum, S32, (const char *name),,
   "@brief.\n\n")
{  
	return object->mShape->findSequence(name);
}

DefineEngineMethod( PhysicsShape, getSeqFilename, const char*, (S32 index),,
   "@brief.\n\n")
{  
	TSShape::Sequence & seq = object->mShape->sequences[index];

	char filename[512];
	String fromStr(seq.sourceData.from.c_str()); 
	Vector<String> justFilename;//(throwing away the second half of the split)
	fromStr.split("\t",justFilename);
	dSprintf(filename,256,"%s",justFilename[0].c_str());

	if (dStrlen(filename)>0)
		return filename;
}

DefineEngineMethod( PhysicsShape, getSeqPath, const char*, (S32 index),,
   "@brief.\n\n")
{  
	TSShape::Sequence & seq = object->mShape->sequences[index];

	char filename[512];
	String fromStr(seq.sourceData.from.c_str()); 
	
	dSprintf(filename,256,"%s",fromStr.c_str());

	if (dStrlen(filename)>0)
		return filename;
}

DefineEngineMethod( PhysicsShape, getSeqFrames, S32, (S32 index),,
   "@brief.\n\n")
{  
	TSShape::Sequence & seq = object->mShape->sequences[index];

	return seq.numKeyframes;
}

DefineEngineMethod( PhysicsShape, getSeqGroundFrames, S32, (S32 index),,
   "@brief.\n\n")
{  
	TSShape::Sequence & seq = object->mShape->sequences[index];

	return seq.numGroundFrames;
}

DefineEngineMethod( PhysicsShape, pauseSeq, void, (),,
   "@brief.\n\n")
{ 
	object->setSequenceTimeScale(0.0);
}

DefineEngineMethod( PhysicsShape, reverseSeq, void, (),,
   "@brief.\n\n")
{ 
	object->setSequenceTimeScale(-1.0);
	//object->setSequencePos();
}

DefineEngineMethod( PhysicsShape, forwardSeq, void, (),,
   "@brief.\n\n")
{ 
	object->setSequenceTimeScale(1.0);
}

DefineEngineMethod( PhysicsShape, getSeqPos, F32, (),,
   "@brief.\n\n")
{ 
	return object->getSequencePos();
}

DefineEngineMethod( PhysicsShape, setSeqPos, void, (F32 pos),,
   "@brief.\n\n")
{ 
	object->setSequencePos(pos);
}

DefineEngineMethod( PhysicsShape, getSeqCyclic, bool, (S32 seq),,
   "@brief.\n\n")
{ 
	return object->mShape->getSeqCyclic(seq);
}

DefineEngineMethod( PhysicsShape, setSeqCyclic, void, (S32 seq,bool val),,
   "@brief.\n\n")
{ 
	object->mShape->setSeqCyclic(seq,val);
}

DefineEngineMethod( PhysicsShape, getSeqBlend, bool, (S32 seq),,
   "@brief.\n\n")
{ 
	return object->mShape->getSeqBlend(seq);
}

DefineEngineMethod( PhysicsShape, setSeqBlend, void, (S32 seq,bool val),,
   "@brief.\n\n")
{ 
	object->mShape->setSeqBlend(seq,val);
}
/*
const char *PhysicsShape::getSeqFilename(const char *seqname)
{
	char filename[512];
	if (1) 
	{
		U32 index = mShape->findSequence(seqname);
		if ((index>=0)&&(index<mShape->sequences.size()))  
		{
			TSShape::Sequence & seq = mShape->sequences[index];
			String fromStr(seq.sourceData.from.c_str()); 
			Vector<String> justFilename;//(throwing away the second half of the split)
			fromStr.split("\t",justFilename);
			dSprintf(filename,256,"%s",justFilename[0].c_str());
		}
	}
	if (dStrlen(filename)>0)
		return filename;
*/





////////////////////////////////////////////////////////////////


DefineEngineMethod( PhysicsShape, setSceneID, void, (S32 id),,
   "@brief \n\n")
{  
	if (id>0)
		object->mSceneID = id;
}

DefineEngineMethod( PhysicsShape, getSceneID, S32, (),,
   "@brief \n\n")
{  
	return object->mSceneID;
}


DefineEngineMethod( PhysicsShape, showNodes, void, (),,
   "@brief prints all nodes\n\n")
{  
	QuatF q;
	for (U32 i=0;i<object->mShape->nodes.size();i++)
	{
		q = object->mShape->defaultRotations[i].getQuatF();
		Con::printf("nodes[%d] %s parent %d  pos %f %f %f  rot %f %f %f %f len %f",i,
			object->mShape->getName(object->mShape->nodes[i].nameIndex).c_str(),object->mShape->nodes[i].parentIndex,
			object->mShape->defaultTranslations[i].x,object->mShape->defaultTranslations[i].y,
			object->mShape->defaultTranslations[i].z,q.x,q.y,q.z,q.w,object->mShape->defaultTranslations[i].len());
	}
}

DefineEngineMethod( PhysicsShape, getNumNodes, S32, (),,
   "@brief \n\n")
{  
	return object->mShape->nodes.size();
}

DefineEngineMethod( PhysicsShape, getNodeName, const char*, (S32 index),,
   "@brief.\n\n")
{  
	return object->mShape->getName(object->mShape->nodes[index].nameIndex).c_str();
}

DefineEngineMethod( PhysicsShape, getNodeNum, S32, (const char *name),,
   "@brief \n\n")
{  
	String nameString = name;
	return object->mShape->findNode(nameString);
}


DefineEngineMethod( PhysicsShape, getNumMattersNodes, S32, (S32 index),,
   "@brief \n\n")
{  
	return object->mShape->getNumMattersNodes(index);
}

DefineEngineMethod( PhysicsShape, getMattersNodeIndex, S32, (S32 index,S32 node),,
   "@brief \n\n")
{  
	return object->mShape->getMattersNodeIndex(index,node);
}

DefineEngineMethod( PhysicsShape, getNodeMattersIndex, S32, (S32 index,S32 node),,
   "@brief \n\n")
{  
	return object->mShape->getNodeMattersIndex(index,node);
}

DefineEngineMethod( PhysicsShape, addMattersNode, void, (S32 index,S32 node),,
   "@brief \n\n")
{  
	return object->mShape->addMattersNode(index,node);
}

DefineEngineMethod( PhysicsShape, dropMattersNode, void, (S32 index,S32 node),,
   "@brief \n\n")
{  
	return object->mShape->dropMattersNode(index,node);
}

DefineEngineMethod( PhysicsShape, getNodeRot, Point3F, (S32 seq,S32 node,S32 frame),,
   "@brief \n\n")
{  
	TSShape *kShape = object->mShape;
	QuatF q;
	MatrixF mat;

	Quat16 q16 = kShape->nodeRotations[frame+kShape->sequences[seq].baseRotation+(kShape->sequences[seq].numKeyframes*node)];
	q16.getQuatF(&q);
	q.setMatrix(&mat);
	EulerF eul = mat.toEuler();

	return eul;
}

DefineEngineMethod( PhysicsShape, getNodeFrameQuat, const char *, (S32 seq,S32 mattersNode,S32 frame),,
   "@brief \n\n")
{ 	
	TSShape *kShape = object->mShape;
	QuatF q;
	char data[256];

	S32 num_frames = kShape->sequences[seq].numKeyframes;
	Quat16 q16 = kShape->nodeRotations[kShape->sequences[seq].baseRotation+(mattersNode*num_frames)+frame];
	q = q16.getQuatF();
	sprintf(data,"%f %f %f %f",q.x,q.y,q.z,q.w);

	return data;
}


DefineEngineMethod( PhysicsShape, setNodeFrameQuat, void, (S32 seq,S32 mattersNode,S32 frame,const char *quatString),,
   "@brief \n\n")
{ 	
	TSShape *kShape = object->mShape;
	S32 num_frames = kShape->sequences[seq].numKeyframes;
	QuatF q;
	
	dSscanf(quatString,"%g %g %g %g",&q.x,&q.y,&q.z,&q.w);
	
	Quat16 q16;
	q16.set(q);
	kShape->nodeRotations[kShape->sequences[seq].baseRotation+(mattersNode*num_frames)+frame] = q16;
	
	return;
}


DefineEngineMethod( PhysicsShape, getNodeFrameEuler, Point3F, (S32 seq,S32 mattersNode,S32 frame),,
   "@brief \n\n")
{ 		
	QuatF q;
	EulerF eul;
	MatrixF mat;
	TSShape *kShape = object->mShape;
	S32 num_frames = kShape->sequences[seq].numKeyframes;

	Quat16 q16 = kShape->nodeRotations[kShape->sequences[seq].baseRotation+(mattersNode*num_frames)+frame];
	q = q16.getQuatF();
	q.setMatrix(&mat);
	eul = mat.toEuler();
	eul *= 180.0/M_PI;

	return eul;
}


DefineEngineMethod( PhysicsShape, setNodeFrameEuler, void, (S32 seq,S32 mattersNode,S32 frame,Point3F eul),,
   "@brief \n\n")
{ 	
	Quat16 q16;
	TSShape *kShape = object->mShape;
	S32 num_frames = kShape->sequences[seq].numKeyframes;

	eul *= M_PI/180.0;
	QuatF q(eul);
	q16.set(q);
	kShape->nodeRotations[kShape->sequences[seq].baseRotation+(mattersNode*num_frames)+frame] = q16;

	return;
}

DefineEngineMethod( PhysicsShape, getNodeTrans, Point3F, (S32 seq,S32 frame),,
   "@brief \n\n")
{ 
	Point3F trans;
	if ((seq>=0)&&(frame>=0))
	{
		TSShape *kShape = object->mShape;
		trans = kShape->nodeTranslations[frame+kShape->sequences[seq].baseTranslation];
	}
	return trans;
}


DefineEngineMethod( PhysicsShape, zeroGroundRots, void, (S32 seq),,
   "@brief \n\n")
{ 
	TSShape *kShape = object->mShape;
	for (U32 i=kShape->sequences[seq].firstGroundFrame;i<kShape->sequences[seq].numGroundFrames+kShape->sequences[seq].firstGroundFrame;i++)
	{
		kShape->groundRotations[i].identity();
	}
	return;
}

DefineEngineMethod( PhysicsShape, addToGroundTrans, void, (S32 seq,Point3F pos),,
   "@brief \n\n")
{ 
	TSShape *kShape = object->mShape;
	Point3F gt(0,0,0);
	for (U32 i=kShape->sequences[seq].firstGroundFrame;i<kShape->sequences[seq].numGroundFrames+kShape->sequences[seq].firstGroundFrame;i++)
	{
		gt = kShape->groundTranslations[i];
		kShape->groundTranslations[i] = gt + pos;
	}
	return;
}

DefineEngineMethod( PhysicsShape, zeroGroundTransAxis, void, (S32 seq,S32 axis),,
   "zeroGroundTransAxis(S32 seq,S32 axis: 0=X,1=Y,2=Z) \n\n")
{ 
	TSShape *kShape = object->mShape;
	if ((seq<0)||(seq>=kShape->sequences.size())||(axis<0)||(axis>2))
		return;

	Point3F gt(0,0,0);
	for (U32 i=kShape->sequences[seq].firstGroundFrame;i<kShape->sequences[seq].numGroundFrames+kShape->sequences[seq].firstGroundFrame;i++)
	{
		gt = kShape->groundTranslations[i];
		if (axis==0)
			gt.x = 0.0;
		else if (axis==1)
			gt.y = 0.0;
		else 
			gt.z = 0.0;
		kShape->groundTranslations[i] = gt;
	}
	return;
}

DefineEngineMethod( PhysicsShape, scaleGroundTrans, void, (S32 seq,F32 scale),,
   "@brief \n\n")
{ 
	TSShape *kShape = object->mShape;
	Point3F gt(0,0,0);
	for (U32 i=kShape->sequences[seq].firstGroundFrame;i<kShape->sequences[seq].numGroundFrames+kShape->sequences[seq].firstGroundFrame;i++)
	{
		Point3F gt = kShape->groundTranslations[i];
		gt *= scale;
		kShape->groundTranslations[i] = gt;
	}
	return;
}


DefineEngineMethod( PhysicsShape, adjustBaseNodePosRegion, void, (S32 seq,Point3F pos,F32 start,F32 stop),,
   "@brief \n\n")
{ 
	object->mShape->adjustBaseNodePosRegion(seq,pos,start,stop);
}
/*


setNodeRotRegion(U32 seq, U32 node, EulerF &rot, F32 start, F32 stop)
addNodeSetRot(U32 node, EulerF &rot)
addNodeAdjustRot(U32 node, EulerF &rot)
doMatrixFix(U32 seq, EulerF &eul1,EulerF &eul2)
*/
DefineEngineMethod( PhysicsShape, setBaseNodePosRegion, void, (S32 seq,Point3F pos,F32 start,F32 stop),,
   "@brief \n\n")
{ 
	object->mShape->setBaseNodePosRegion(seq,pos,start,stop);
}

DefineEngineMethod( PhysicsShape, adjustNodeRotRegion, void, (S32 seq,S32 node,EulerF rot,F32 start,F32 stop),,
   "@brief \n\n")
{ 
	object->mShape->adjustNodeRotRegion(seq,node,rot,start,stop);
}

DefineEngineMethod( PhysicsShape, setNodeRotRegion, void, (S32 seq,S32 node,EulerF rot,F32 start,F32 stop),,
   "@brief \n\n")
{ 
	object->mShape->setNodeRotRegion(seq,node,rot,start,stop);
}

DefineEngineMethod( PhysicsShape, addNodeSetRot, void, (S32 node,EulerF rot),,
   "@brief \n\n")
{ 
	object->mShape->addNodeSetRot(node,rot);
}

DefineEngineMethod( PhysicsShape, addNodeAdjustRot, void, (S32 node,EulerF rot),,
   "@brief \n\n")
{ 
	object->mShape->addNodeAdjustRot(node,rot);
}

DefineEngineMethod( PhysicsShape, doMatrixFix, void, (S32 seq,EulerF eul1,EulerF eul2),,
   "@brief \n\n")
{ 
	object->mShape->doMatrixFix(seq,eul1,eul2);
}


/////////////////////////////////////////////////////////

DefineEngineMethod( PhysicsShape, loadXml, void, (const char *file),,
   "@brief \n\n")
{  	
	object->loadXml(file);
}

DefineEngineMethod( PhysicsShape, write, void,  (const char *filename),,
   "@brief \n\n")
{
	FileStream *outstream;
	if ((outstream = FileStream::createAndOpen(filename, Torque::FS::File::Write))==NULL) {
		Con::printf("whoops, name no good!"); 
	} else {
		object->mShape->write(outstream);
		outstream->close();
	}	
}

DefineEngineMethod( PhysicsShape, addNodeTransform, void, (const char *nodeName, Point3F offset, Point3F rot),,
   "@brief Sets the node rotation for (nodeName) node to (rot - degrees), around a point (offset) from the center.\n\n")
{ 
	TSStatic *clientShape = dynamic_cast<TSStatic *>(object->getClientObject());
	if (clientShape)
		clientShape->getShapeInstance()->addNodeTransform(nodeName,offset,EulerF(mDegToRad(rot.x),mDegToRad(rot.y),mDegToRad(rot.z)));
}

DefineEngineMethod( PhysicsShape, setNodeTransform, void, (const char *nodeName, Point3F offset, Point3F rot),,
   "@brief Sets the node rotation for (nodeName) node to (rot - degrees), around a point (offset) from the center.\n\n")
{ 
	TSStatic *clientShape = dynamic_cast<TSStatic *>(object->getClientObject());
	if (clientShape)
		clientShape->getShapeInstance()->setNodeTransform(nodeName,offset,EulerF(mDegToRad(rot.x),mDegToRad(rot.y),mDegToRad(rot.z)));
}

F32 PhysicsShape::getSeqDeltaSum(S32 seq,S32 currFrame,S32 baseFrame)
{
	Vector<MatrixF> baseGlobalRotations,currGlobalRotations;
	F32 deltaSum = 0.0;
	F32 fDeltaSum = 0.0;
	S32 n = 0;
	if (nodeCropStartRotations.size()==0)
		return deltaSum;

	for (U32 i=0;i<mShape->nodes.size();i++)
	{
		MatrixF bM,cM;
		QuatF bQ,cQ,bRQ,cRQ;
		Quat16 q16;
		
		baseGlobalRotations.increment();
		currGlobalRotations.increment();
		if (mShape->sequences[seq].rotationMatters.test(i))
		{
			//This uses the actual global positions, measured from the world.
			bQ = nodeCropStartRotations[i];
			cQ = QuatF(mShapeInstance->mNodeTransforms[i]);
			deltaSum += fabs(bQ.x - cQ.x) + fabs(bQ.y - cQ.y) + fabs(bQ.z - cQ.z) + fabs(bQ.w - cQ.w);
			//Con::printf("baseQ: %f %f %f %f, currQ: %f %f %f %f, deltaSum %f",
			//	bQ.x,bQ.y,bQ.z,bQ.w,cQ.x,cQ.y,cQ.z,cQ.w,deltaSum);
			n++;
		}
	}
	//fDeltaSum /= (F32)n;
	deltaSum /= (F32)n;
	//Con::printf(" deltaSum %f    fDeltaSum %f",deltaSum,fDeltaSum);
	
	return deltaSum;
}

DefineEngineMethod( PhysicsShape, getSeqDeltaSum, F32, (S32 seq,S32 currFrame,S32 baseFrame),,
   "@brief\n\n")
{ 
	return object->getSeqDeltaSum(seq,currFrame,baseFrame);
}

DefineEngineMethod( PhysicsShape, recordCropStartPositions, void, (),,
   "@brief \n\n")
{ 
	object->nodeCropStartRotations.clear();
	for (U32 i=0;i<object->mShape->nodes.size();i++)
	{
		QuatF q(object->mShapeInstance->mNodeTransforms[i]);
		object->nodeCropStartRotations.increment();
		object->nodeCropStartRotations[object->nodeCropStartRotations.size()-1] = q;
	}

	return;
}

DefineEngineMethod( PhysicsShape, smoothLoopTransition, void, (S32 seq,S32 frames),,
   "@brief \n\n")
{ 
	object->mShape->smoothLoopTransition(seq,frames);
}


///////////////////////////////////////////////////////////////////////////////

DefineEngineMethod( PhysicsShape, setUseDataSource, void, (bool useDataSource),,
   "@brief Tells physicsShape to create a vehicleDataSource and listen to it for transforms and other data.\n\n")
{ 
	Con::printf("PhysicsShape setting useDataSource: %d !!!!!!!!!!!!!!!!!!!!!!!!",useDataSource);
	object->mUseDataSource = useDataSource;
	PhysicsShape *clientShape = dynamic_cast<PhysicsShape *>(object->getClientObject());
	if (clientShape)
		clientShape->mUseDataSource = useDataSource;
}

bool PhysicsShape::setNavMesh()
{
	Sim::findObject(mNavMeshName.c_str(),mNavMesh);

	//SimObject *healthKit = Sim::findObject("myHealthKit");
	//SceneObject *myHealthKit = dynamic_cast<SceneObject*>(healthKit);

	if (mNavMesh)
	{ 
		if (mNavPath)
			delete mNavPath;//Is this at all safe?

		mNavPath = new NavPath();
		mNavPath->mMesh = mNavMesh;//This is not the normal way, supposed to use SetProtectedMesh...
		//mNavPath->setProtectedMesh(this,"NavMeshOne",data?)//Hm, why private? and what does protected mean here? Hmm


		//mNavPath->mFrom = getPosition();
		//mNavPath->mTo = myHealthKit->getPosition();
		//mNavPath->mFromSet = mNavPath->mToSet = true;
		mNavPath->mAlwaysRender = true;
		mNavPath->mLinkTypes = mLinkTypes;
		mNavPath->mXray = true;
		// Paths plan automatically upon being registered.
		if(!mNavPath->registerObject())
		{
			delete mNavPath;
			return false;
		}

		//mNavPath->plan();

		if (mVehicle)
		{
			mVehicle->mDetourNavPath = mNavPath;
		} else 
			Con::printf("setNavMesh: we don't have a vehicle yet!");
		//Con::printf("Navpath apparently successful! from %f %f %f, to %f %f %f, length %f  nodes %d",
		//	mNavPath->mFrom.x,mNavPath->mFrom.y,mNavPath->mFrom.z,mNavPath->mTo.x,mNavPath->mTo.y,mNavPath->mTo.z,
		//	mNavPath->getLength(),mNavPath->size());

		return true;
	}
	return false;
}

DefineEngineMethod(PhysicsShape,setNavMesh,void, (const char *meshName),,"")
{
	object->mNavMeshName = meshName;
	if (!object->setNavMesh())
		Con::printf("PhysicsShape failed to load navmesh: %s",meshName);
	else
		Con::printf("PhysicsShape successfully loaded navmesh %s!!!!!",meshName);
}

DefineEngineMethod(PhysicsShape,getNavPathSize,S32, (),,"")
{
	if (object->mNavPath)
		return object->mNavPath->size();
	else
		return -1;
}

DefineEngineMethod(PhysicsShape,getNavPathLength,F32, (),,"")
{
	if (object->mNavPath)
		return object->mNavPath->getLength();
	else
		return -1.0f;
}

DefineEngineMethod(PhysicsShape,getNavPathNode,Point3F, (S32 node),,"")
{
	if (object->mNavPath)
		return object->mNavPath->getNode(node);
	else
		return Point3F(0,0,0);
}

bool PhysicsShape::setNavPathTo(Point3F toPos)
{
	//Con::printf("Setting Nav Path To: %f %f %f",toPos.x,toPos.y,toPos.z);
	//HERE: we are on server, this should be client, since everything else is.
	if (mNavPath==NULL)
		return false;

	mNavPath->mFrom = getClientPosition();
	mNavPath->mTo = toPos;
	mNavPath->mFromSet = mNavPath->mToSet = true;		
	mNavPath->mAlwaysRender = true;
	mNavPath->mLinkTypes = mLinkTypes;
	mNavPath->mXray = true;

	mNavPath->plan();

	if (isServerObject())
	{
		PhysicsShape *clientShape = dynamic_cast<PhysicsShape *>(getClientObject());//SINGLE PLAYER HACK
		clientShape->mNavPath = mNavPath;
		if (clientShape->mVehicle)
			clientShape->mVehicle->mDetourNavPath = mNavPath;
	} //Con::printf("Navpath apparently successful! from %f %f %f, to %f %f %f, length %f  nodes %d",
	//		mNavPath->mFrom.x,mNavPath->mFrom.y,mNavPath->mFrom.z,mNavPath->mTo.x,mNavPath->mTo.y,mNavPath->mTo.z,
	//		mNavPath->getLength(),mNavPath->size());

	return true;
}
DefineEngineMethod(PhysicsShape,setNavPathTo,bool, (Point3F toPos),,"")
{//HERE: this should be calling the client object.
	return object->setNavPathTo(toPos);
}


/////////////////////////////////////////////////////////////////////
//OpenSteer

DefineEngineMethod(PhysicsShape,getPedID,S32, (),,"")
{
	return object->mPedId;
}

DefineEngineMethod(PhysicsShape,createVehicle,bool, (Point3F pos,float rot),,"")
{
	if (object->mVehicle)
		return false;

	OpenSteer::Vec3 vec(-pos.x,pos.z,pos.y);
	OpenSteer::OpenSteerDemo::createVehicle(vec,rot);
	object->assignLastVehicle();
	Con::printf("engine, created vehicle %f %f %f",vec.x,vec.y,vec.z);
	return true;
}

void PhysicsShape::assignLastVehicle()
{
	if (mVehicle)
		return;

	const OpenSteer::AVGroup& vehicles = OpenSteer::OpenSteerDemo::allVehiclesOfSelectedPlugIn();
	mVehicle = dynamic_cast<NavClient*>(vehicles[vehicles.size()-1]);
	mVehicle->setThinking(1);
	if ((mVehicle)&&(mNavPath))
		mVehicle->mDetourNavPath = mNavPath;
	else if (mNavPath==NULL)
		Con::printf("whoops, we don't have a navpath yet! server %d",isServerObject());
}

DefineEngineMethod(PhysicsShape,assignVehicle,bool, (),,"")
{
	if (object->mVehicle)
		return false;

	const OpenSteer::AVGroup& vehicles = OpenSteer::OpenSteerDemo::allVehiclesOfSelectedPlugIn();
	if ((vehicles.size() > 0) && (object->mPedId < vehicles.size()))
	{
		object->mVehicle = dynamic_cast<NavClient*>(vehicles[object->mPedId]);
		return true;
	}
	return false;
}


void PhysicsShape::assignVehicleNavPath()
{//OBSOLETE, use recast navpath instead.
	if ((mVehicle)&&(mNavPath))
	{
		float pathRadius = 3.0;//2.0;//FIX: get from opensteerProfile
		int pathPointCount = mNavPath->size();

		//const Vec3 pathPoints[pathPointCount] = { Vec3 (0,0,0), Vec3 (40,40,0) };
		//if (mVehicle->path)
		//	delete mVehicle->path;
		//MEGAMOTION: Fix this, declaration of NavPoints was breaking due to inability to predeclare inside the
		//OpenSteer namespace, and including openSteerDemo.h in physicsShape.h causes all kinds of winsock conflicts
		//and other weird behavior, due probably to inclusion of windows.h too early or something. Bypassing for now.
		/*
		for (U32 i=0;i<mNavPath->size();i++)
		{
			Point3F pos = mNavPath->getNode(i);
			mNavPoints[i].x = -pos.x;  
			mNavPoints[i].y =  pos.z;  
			mNavPoints[i].z =  pos.y;  
			Con::printf("nav points %d: %f %f %f",i,-pos.x,pos.z,pos.y);
		}
		mVehicle->path = new PolylinePathway (pathPointCount,
			mNavPoints,
			pathRadius,
			false);
			*/
	}
}

//WAIT... Do all of these need to be client instead of server???
DefineEngineMethod(PhysicsShape,assignVehicleNavPath,void, (),,"")
{
	if (object->mVehicle)
		return;

	object->assignVehicleNavPath();

	return;
}

DefineEngineMethod(PhysicsShape,getVehiclePos,Point3F, (),,"")
{
	Point3F pos;
	if (object->mVehicle)
	{
		OpenSteer::Vec3 vec;
		vec = object->mVehicle->position();
		pos = Point3F(-vec.x,vec.z,vec.y);
	}
	return pos;
}

DefineEngineMethod(PhysicsShape,setVehiclePos,void, (Point3F pos),,"")
{
	if (object->mVehicle)
	{
		OpenSteer::Vec3 vec(-pos.x,pos.z,pos.y);
		object->mVehicle->setPosition(vec);
	}
	return;
}

DefineEngineMethod(PhysicsShape,getVehicleVel,Point3F, (),,"")
{
	Point3F pos;
	if (object->mVehicle)
	{
		OpenSteer::Vec3 vel;
		vel = object->mVehicle->velocity();
		pos = Point3F(-vel.x,vel.z,vel.y);
	}
	return pos;
}

DefineEngineMethod(PhysicsShape,setOpenSteerMoveTarget,void, (Point3F pos),,"")
{
	if (object->mVehicle)
	{
		OpenSteer::Vec3 vec(-pos.x,pos.z,pos.y);
		object->mVehicle->setMoveTarget(vec);
	}
	return;
}

DefineEngineMethod(PhysicsShape,getOpenSteerMass,F32, (),,"")
{
	if (object->mVehicle)
		return object->mVehicle->mass();
	else 
		return 0.0;
}

DefineEngineMethod(PhysicsShape,setOpenSteerMass,void, (F32 value),,"")
{
	if (object->mVehicle)
		object->mVehicle->setMass(value);
}

DefineEngineMethod(PhysicsShape,getOpenSteerRadius,F32, (),,"")
{
	if (object->mVehicle)
		return object->mVehicle->radius();
	else 
		return 0.0;
}

DefineEngineMethod(PhysicsShape,setOpenSteerRadius,void, (F32 value),,"")
{
	if (object->mVehicle)
		object->mVehicle->setRadius(value);
}

DefineEngineMethod(PhysicsShape,getOpenSteerMaxSpeed,F32, (),,"")
{
	if (object->mVehicle)
		return object->mVehicle->maxSpeed();
	else 
		return 0.0;
}

DefineEngineMethod(PhysicsShape,setOpenSteerMaxSpeed,void, (F32 value),,"")
{
	if (object->mVehicle)
	{
		//object->mVehicle->mMaxSpeed = dAtof(argv[2]);//Any reason at all to have these variables?
		object->mVehicle->setMaxSpeed(value);//This is how you change the real value.
		//Con::printf("setting opensteer max speed: %f",object->mVehicle->mMaxSpeed);
	}
}

DefineEngineMethod(PhysicsShape,getOpenSteerMaxForce,F32, (),,"")
{
	if (object->mVehicle)
		return object->mVehicle->maxForce();
	else 
		return 0.0;
}

DefineEngineMethod(PhysicsShape,setOpenSteerMaxForce,void, (F32 value),,"")
{
	if (object->mVehicle)
		object->mVehicle->setMaxForce(value);
}


DefineEngineMethod(PhysicsShape,setOpenSteerWanderChance,void, (F32 value),,"")
{
	if (object->mVehicle)
		object->mVehicle->mWanderChance = value;//Warning, might want to sanity check.
}

DefineEngineMethod(PhysicsShape,setOpenSteerWanderWeight,void, (F32 value),,"")
{
	if (object->mVehicle)
		object->mVehicle->mWanderWeight = value;//Warning, might want to sanity check.
}

DefineEngineMethod(PhysicsShape,setOpenSteerSeekTargetWeight,void, (F32 value),,"")
{
	if (object->mVehicle)
		object->mVehicle->mSeekTargetWeight = value;//Warning, might want to sanity check.
}

DefineEngineMethod(PhysicsShape,setOpenSteerAvoidTargetWeight,void, (F32 value),,"")
{
	if (object->mVehicle)
		object->mVehicle->mAvoidTargetWeight = value;//Warning, might want to sanity check.
}

DefineEngineMethod(PhysicsShape,setOpenSteerSeekNeighborWeight,void, (F32 value),,"")
{
	if (object->mVehicle)
		object->mVehicle->mSeekNeighborWeight = value;//Warning, might want to sanity check.
}

DefineEngineMethod(PhysicsShape,setOpenSteerAvoidNeighborWeight,void, (F32 value),,"")
{
	if (object->mVehicle)
		object->mVehicle->mAvoidNeighborWeight = value;//Warning, might want to sanity check.
}

DefineEngineMethod(PhysicsShape,setOpenSteerAvoidNavMeshEdgeWeight,void, (F32 value),,"")
{
	if (object->mVehicle)
		object->mVehicle->mAvoidNavMeshEdgeWeight = value;//Warning, might want to sanity check.
}

DefineEngineMethod(PhysicsShape,setOpenSteerDetectNavMeshEdgeRange,void, (F32 value),,"")
{
	if (object->mVehicle)
		object->mVehicle->mDetectNavMeshEdgeRange = value;//Warning, might want to sanity check.
}



/*

PolylinePathway* getNavPath (void)
{
    if (gNavPath == NULL)
    {
        const float pathRadius = 2;

        const int pathPointCount = 7;
        const float size = 20;
        const float top = 2 * size;
        const float gap = 1.2f * size;
        const float out = 2 * size;
        const float h = 0.5;
        const Vec3 pathPoints[pathPointCount] =
            {Vec3 (h+gap-out,     0,  h+top-out),  // 0 a
             Vec3 (h+gap,         0,  h+top),      // 1 b
             Vec3 (h+gap+(top/2), 0,  h+top/2),    // 2 c
             Vec3 (h+gap,         0,  h),          // 3 d
             Vec3 (h,             0,  h),          // 4 e
             Vec3 (h,             0,  h+top),      // 5 f
             Vec3 (h+gap,         0,  h+top/2)};   // 6 g

        //gObstacle1.center = interpolate (0.2f, pathPoints[0], pathPoints[1]);
        //gObstacle2.center = interpolate (0.5f, pathPoints[2], pathPoints[3]);
        //gObstacle1.radius = 3;
        //gObstacle2.radius = 5;
        //gObstacles.push_back (&gObstacle1);
        //gObstacles.push_back (&gObstacle2);

        gEndpoint0 = pathPoints[0];
        gEndpoint1 = pathPoints[pathPointCount-1];

        gNavPath = new PolylinePathway (pathPointCount,
                                         pathPoints,
                                         pathRadius,
                                         false);
    }
    return gNavPath;
}

*/

/*
/////////////////////////////////////////////////////////////////////
//NavMesh

bool fxFlexBody::setNavMesh()
{
	SimObject *navMeshObj = NULL;
	navMeshObj = Sim::findObject(mNavMeshName.c_str());
	
	if (navMeshObj)
	//if (0)//TEMP: NavMesh code not ready for release yet, but leaving it in the build, shouldn't
	{ // hurt anything since everything checks if (mNavPath) first before doing anything.
		Nav::NavMesh *kNavMesh = dynamic_cast<Nav::NavMesh*>(navMeshObj);

		if (mNavPath)
			delete mNavPath;//Is this at all safe?

		mNavPath = new Nav::NavPath();
		mNavPath->mMesh = kNavMesh;//This is not the normal way, supposed to use SetProtectedMesh...
		//mNavPath->setProtectedMesh(this,"NavMeshOne",data?)
		return true;
	}
	return false;
}

ConsoleMethod(fxFlexBody,setNavMesh,void,3,3,"setNavMesh(const char *)")
{
	object->mNavMeshName = argv[2];
	if (!object->setNavMesh())
		Con::printf("Flexbody failed to load navmesh: %s",argv[2]);
}

ConsoleMethod(fxFlexBody,getNavMesh,const char*,2,2,"getNavMesh()")
{
	char* buff = Con::getReturnBuffer(100);

	if (object->mNavMeshName.length()>0)
		sprintf(buff,"%s",object->mNavMeshName.c_str());

	return buff;
}

void fxFlexBody::setNavPathTo(Point3F pos)
{
	if (mNavPath)
	{
		mNavPath->mTo = pos;
		mNavPath->mFrom = getPosition();
		mNavPath->mToSet = true;
		mNavPath->mFromSet = true;
		mNavPath->plan();
		Con::printf("path is planned!  nodes: %d, length: %f  pos %f %f %f  from %f %f %f",mNavPath->getCount(),mNavPath->getLength(),
			pos.x,pos.y,pos.z,mNavPath->mFrom.x,mNavPath->mFrom.y,mNavPath->mFrom.z);
		for (U32 i=0;i<mNavPath->getCount();i++)
		{
			Con::printf("pathnode %d: ? ? ?",i);
		}
		if (mVehicle)
			assignVehicleNavPath();
	}
}

ConsoleMethod(fxFlexBody,setNavPathTo,void,3,3,"setNavPathTo(Point3F)")
{
	Point3F pos;
	dSscanf( argv[2], "%f %f %f", &pos.x, &pos.y, &pos.z );
	Con::printf("setting nav path to: %f %f %f",pos.x,pos.y,pos.z);
	object->setNavPathTo(pos);
	return;
}

ConsoleMethod(fxFlexBody,getNavPathTo,const char*,2,2,"getNavPathTo()")
{
	char* buff = Con::getReturnBuffer(100);
	if (object->mNavPath)
	{
		Point3F pos = object->mNavPath->mTo;
		dSprintf(buff,100,"%g %g %g",pos.x,pos.y,pos.z);
	}
	return buff;
}

void fxFlexBody::setNavPathFrom(Point3F pos)
{
	if (mNavPath)
	{
		mNavPath->mFrom = pos;
	}
}

ConsoleMethod(fxFlexBody,setNavPathFrom,void,3,3,"setNavPathFrom(Point3F)")
{
	Point3F pos;
	dSscanf( argv[2], "%f %f %f", &pos.x, &pos.y, &pos.z );
	object->setNavPathFrom(pos);
	return;
}

ConsoleMethod(fxFlexBody,getNavPathFrom,const char*,2,2,"getNavPathFrom()")
{
	char* buff = Con::getReturnBuffer(100);
	if (object->mNavPath)
	{
		Point3F pos = object->mNavPath->mFrom;
		dSprintf(buff,100,"%g %g %g",pos.x,pos.y,pos.z);
	}
	return buff;
}


ConsoleMethod(fxFlexBody,getNavPathLength,F32,2,2,"getNavPathLength()")
{
	F32 len = 0.0;
	if (object->mNavPath)
	{
		len = object->mNavPath->getLength();
	}
	return len;
}

ConsoleMethod(fxFlexBody,getNavPathCount,F32,2,2,"getNavPathCount()")
{
	S32 cnt = 0;
	if (object->mNavPath)
	{
		cnt = object->mNavPath->getCount();
	}
	return cnt;
}

ConsoleMethod(fxFlexBody,getNavPathNodePos,const char*,3,3,"getNavPathNodePos(S32 index)")
{
	char* buff = Con::getReturnBuffer(100);
	if (object->mNavPath)
	{
		Point3F pos = object->mNavPath->getNode(dAtoi(argv[2]));
		dSprintf(buff,100,"%g %g %g",pos.x,pos.y,pos.z);
	}
	return buff;
}

ConsoleMethod(fxFlexBody,getNavPathNode,S32,2,2,"getNavPathNode()")
{
	return object->mNavPathNode;
}

ConsoleMethod(fxFlexBody,setNavPathNode,void,3,3,"setNavPathNode(S32)")
{
	object->mNavPathNode = dAtoi(argv[2]);
	return;
}

ConsoleMethod(fxFlexBody,hasNavPath,bool,2,2,"bool hasNavPath()")
{
	if (object->mNavPath)
		return true;
	else
		return false;
}

*/
