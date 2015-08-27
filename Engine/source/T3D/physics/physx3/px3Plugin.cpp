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
#include "console/consoleTypes.h"
#include "T3D/physics/physx3/px3World.h"
#include "T3D/physics/physx3/px3Plugin.h"
#include "T3D/physics/physx3/px3Collision.h"
#include "T3D/physics/physx3/px3Material.h"
#include "T3D/physics/physx3/px3Body.h"
#include "T3D/physics/physx3/px3Player.h"
#include "T3D/physics/physx3/Cloth/px3ClothShape.h"
#include "T3D/physics/physx3/px3Joint.h"

#include "T3D/physics/physicsShape.h"
#include "T3D/gameBase/gameProcess.h"
#include "core/util/tNamedFactory.h"
#include "console/SQLiteObject.h"

AFTER_MODULE_INIT( Sim )
{
   NamedFactory<PhysicsPlugin>::add( "PhysX3", &Px3Plugin::create );

   #if defined(TORQUE_OS_WIN) || defined(TORQUE_OS_XBOX) || defined(TORQUE_OS_XENON)   
      NamedFactory<PhysicsPlugin>::add( "default", &Px3Plugin::create );
   #endif   
}

PhysicsPlugin* Px3Plugin::create()
{
   // Only create the plugin if it hasn't been set up AND
   // the PhysX world is successfully initialized.
   bool success = Px3World::restartSDK( false );
   if ( success )
      return new Px3Plugin();
   
   return NULL;
}

Px3Plugin::Px3Plugin()
{
	mSQL = new SQLiteObject();
	if (mSQL->OpenDatabase("openSimEarth.db"))//HERE, get this from prefs???
	{
		char select_query[512],insert_query[512];
		int id,result;
		sqlite_resultset *resultSet;

		sprintf(select_query,"SELECT id,name FROM px3Joint;");
		result = mSQL->ExecuteSQL(select_query);
		if (result==0)
			return; 				
		
		resultSet = mSQL->GetResultSet(result);
		Con::printf("OPENED SQLITE DATABASE: results: %d",resultSet->iNumRows);
		for (U32 i=0;i<resultSet->iNumRows;i++)
		{
			id = dAtoi(resultSet->vRows[i]->vColumnValues[0]);
			Con::printf("Result one: %d   %s",id,resultSet->vRows[i]->vColumnValues[1]);
		}
	}
}

Px3Plugin::~Px3Plugin()
{	
	if (mSQL)
	{
		mSQL->CloseDatabase();
		delete mSQL;
	}
}

void Px3Plugin::destroyPlugin()
{
   // Cleanup any worlds that are still kicking.
   Map<StringNoCase, PhysicsWorld*>::Iterator iter = mPhysicsWorldLookup.begin();
   for ( ; iter != mPhysicsWorldLookup.end(); iter++ )
   {
      iter->value->destroyWorld();
      delete iter->value;
   }
   mPhysicsWorldLookup.clear();

   Px3World::restartSDK( true );

   delete this;
}

void Px3Plugin::reset()
{
   // First delete all the cleanup objects.
   if ( getPhysicsCleanup() )
      getPhysicsCleanup()->deleteAllObjects();

   getPhysicsResetSignal().trigger( PhysicsResetEvent_Restore );

   // Now let each world reset itself.
   Map<StringNoCase, PhysicsWorld*>::Iterator iter = mPhysicsWorldLookup.begin();
   for ( ; iter != mPhysicsWorldLookup.end(); iter++ )
      iter->value->reset();
}

PhysicsCollision* Px3Plugin::createCollision()
{
   return new Px3Collision();
}

PhysicsBody* Px3Plugin::createBody()
{
   return new Px3Body();
}

PhysicsPlayer* Px3Plugin::createPlayer()
{
   return new Px3Player();
}

bool Px3Plugin::isSimulationEnabled() const
{
   bool ret = false;
   Px3World *world = static_cast<Px3World*>( getWorld( smClientWorldName ) );
   if ( world )
   {
      ret = world->isEnabled();
      return ret;
   }

   world = static_cast<Px3World*>( getWorld( smServerWorldName ) );
   if ( world )
   {
      ret = world->isEnabled();
      return ret;
   }

   return ret;
}

void Px3Plugin::enableSimulation( const String &worldName, bool enable )
{
   Px3World *world = static_cast<Px3World*>( getWorld( worldName ) );
   if ( world )
      world->setEnabled( enable );
}

void Px3Plugin::setTimeScale( const F32 timeScale )
{
   // Grab both the client and
   // server worlds and set their time
   // scales to the passed value.
   Px3World *world = static_cast<Px3World*>( getWorld( smClientWorldName ) );
   if ( world )
      world->setEditorTimeScale( timeScale );

   world = static_cast<Px3World*>( getWorld( smServerWorldName ) );
   if ( world )
      world->setEditorTimeScale( timeScale );
}

const F32 Px3Plugin::getTimeScale() const
{
   // Grab both the client and
   // server worlds and call 
   // setEnabled( true ) on them.
   Px3World *world = static_cast<Px3World*>( getWorld( smClientWorldName ) );
   if ( !world )
   {
      world = static_cast<Px3World*>( getWorld( smServerWorldName ) );
      if ( !world )
         return 0.0f;
   }
   
   return world->getEditorTimeScale();
}

bool Px3Plugin::createWorld( const String &worldName, Point3F gravity )
{
   Map<StringNoCase, PhysicsWorld*>::Iterator iter = mPhysicsWorldLookup.find( worldName );
   PhysicsWorld *world = NULL;
   
   iter != mPhysicsWorldLookup.end() ? world = (*iter).value : world = NULL; 

   if ( world ) 
   {
      Con::errorf( "Px3Plugin::createWorld - %s world already exists!", worldName.c_str() );
      return false;
   }

   world = new Px3World();

   world->mGravity = gravity;
   
   if ( worldName.equal( smClientWorldName, String::NoCase ) )
      world->initWorld( false, ClientProcessList::get() );
   else
      world->initWorld( true, ServerProcessList::get() );

   mPhysicsWorldLookup.insert( worldName, world );

   return world != NULL;
}

void Px3Plugin::destroyWorld( const String &worldName )
{
   Map<StringNoCase, PhysicsWorld*>::Iterator iter = mPhysicsWorldLookup.find( worldName );
   if ( iter == mPhysicsWorldLookup.end() )
      return;

   PhysicsWorld *world = (*iter).value;
   world->destroyWorld();
   delete world;
   
   mPhysicsWorldLookup.erase( iter );
}

PhysicsWorld* Px3Plugin::getWorld( const String &worldName ) const
{
   if ( mPhysicsWorldLookup.isEmpty() )
      return NULL;

   Map<StringNoCase, PhysicsWorld*>::ConstIterator iter = mPhysicsWorldLookup.find( worldName );

   return iter != mPhysicsWorldLookup.end() ? (*iter).value : NULL;
}

PhysicsWorld* Px3Plugin::getWorld() const
{
   if ( mPhysicsWorldLookup.size() == 0 )
      return NULL;

   Map<StringNoCase, PhysicsWorld*>::ConstIterator iter = mPhysicsWorldLookup.begin();
   return iter->value;
}

U32 Px3Plugin::getWorldCount() const
{ 
   return mPhysicsWorldLookup.size(); 
}

// andrewmac: Cloth
PhysicsCloth* Px3Plugin::createCloth(TSShapeInstance* shapeInst, const MatrixF &transform)
{
    Px3ClothShape* cloth = new Px3ClothShape();
    cloth->create(shapeInst, transform);
    return cloth;
}

PhysicsMaterial* Px3Plugin::createMaterial(const F32 restitution,const F32 staticFriction,const F32 dynamicFritction)
{
   Px3Material* material  = new Px3Material();
   material->create(restitution,staticFriction,dynamicFritction);
   return material;
}

PhysicsJoint* Px3Plugin::createJoint(PhysicsBody* A,PhysicsBody* B,U32 jointID,Point3F origin,Point3F jointRots,Point3F jointRots2,MatrixF shapeTrans)
{
	PhysicsWorld *worldA,*worldB;
	worldA = A->getWorld();
	worldB = B->getWorld();
	if (worldA!=worldB)
	{
		Con::errorf("Physics bodies are not in the same world!");
		return NULL;
	}

	physx::PxRigidActor *actor1, *actor2;
	actor1 = dynamic_cast<Px3Body*>(A)->getActor();
	actor2 = dynamic_cast<Px3Body*>(B)->getActor();
	
	physicsJointData jD;	
	loadJointData(jointID,&jD);
	Px3Joint* joint = new Px3Joint(actor1,actor2,dynamic_cast<Px3World*>(worldA),&jD,origin,jointRots,jointRots2,shapeTrans);

	return (PhysicsJoint *)joint;
}

void Px3Plugin::loadJointData(U32 jointID, physicsJointData* jD)
{
	int i=0;
	F32 x,y,z;
	char id_query[512],insert_query[512];
	S32 result;
	sqlite_resultset *resultSet;

	sprintf(id_query,"SELECT * FROM px3Joint WHERE id=%d;",jointID);
	result = mSQL->ExecuteSQL(id_query);
	if (result==0)
		return;

	resultSet = mSQL->GetResultSet(result);
	if (resultSet->iNumRows!=1)
		return;

	jD->jointID = dAtoi(resultSet->vRows[0]->vColumnValues[i++]);
	i++;//We don't care about the joint name at this point, skipping it.
	jD->jointType = (physicsJointType)dAtoi(resultSet->vRows[0]->vColumnValues[i++]);

	jD->twistLimit = mDegToRad( dAtof(resultSet->vRows[0]->vColumnValues[i++]) );
	jD->swingLimit = mDegToRad( dAtof(resultSet->vRows[0]->vColumnValues[i++]) );
	jD->swingLimit2 = mDegToRad( dAtof(resultSet->vRows[0]->vColumnValues[i++]) );

	jD->XLimit = dAtof(resultSet->vRows[0]->vColumnValues[i++]);
	jD->YLimit = dAtof(resultSet->vRows[0]->vColumnValues[i++]);
	jD->ZLimit = dAtof(resultSet->vRows[0]->vColumnValues[i++]);

	x = dAtof(resultSet->vRows[0]->vColumnValues[i++]);
	y = dAtof(resultSet->vRows[0]->vColumnValues[i++]);
	z = dAtof(resultSet->vRows[0]->vColumnValues[i++]);
	jD->localAxis = Point3F(x,y,z);

	x = dAtof(resultSet->vRows[0]->vColumnValues[i++]);
	y = dAtof(resultSet->vRows[0]->vColumnValues[i++]);
	z = dAtof(resultSet->vRows[0]->vColumnValues[i++]);
	jD->localNormal = Point3F(x,y,z);

	jD->swingSpring = dAtof(resultSet->vRows[0]->vColumnValues[i++]);
	jD->twistSpring = dAtof(resultSet->vRows[0]->vColumnValues[i++]);
	jD->springDamper = dAtof(resultSet->vRows[0]->vColumnValues[i++]);
	
	jD->motorSpring = dAtof(resultSet->vRows[0]->vColumnValues[i++]);
	jD->motorDamper = dAtof(resultSet->vRows[0]->vColumnValues[i++]);

	jD->maxForce = dAtof(resultSet->vRows[0]->vColumnValues[i++]);
	jD->maxTorque = dAtof(resultSet->vRows[0]->vColumnValues[i++]);
	
	//And then later, the limit planes...
	//...
	
}
