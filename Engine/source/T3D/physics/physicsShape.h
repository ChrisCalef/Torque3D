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

#ifndef _PHYSICSSHAPE_H_
#define _PHYSICSSHAPE_H_

#ifndef _GAMEBASE_H_
   #include "T3D/gameBase/gameBase.h"
#endif
#ifndef __RESOURCE_H__
   #include "core/resource.h"
#endif
#ifndef _TSSHAPE_H_
   #include "ts/tsShape.h"
#endif
#ifndef _T3D_PHYSICSCOMMON_H_
   #include "T3D/physics/physicsCommon.h"
#endif
#ifndef _DYNAMIC_CONSOLETYPES_H_
   #include "console/dynamicTypes.h"
#endif
#ifndef _SIMOBJECTREF_H_
   #include "console/simObjectRef.h"
#endif
#ifndef _MATERIALDEFINITION_H_
#include "materials/materialDefinition.h"
#endif
#include <string>
#include <map>

#ifdef TORQUE_NAVIGATION_ENABLED
#ifndef _NAVPATH_H_
#include "navigation/navPath.h"
#endif
#ifndef _NAVMESH_H_
#include "navigation/navMesh.h"
#endif
#ifndef _COVERPOINT_H_
#include "navigation/coverPoint.h"
#endif
#endif // TORQUE_NAVIGATION_ENABLED


//#define MAX_NAV_NODES 80 //Max nodes in one navpath, needs a limit for opensteer 
                          //to have a Vec3 * to an array.


class TSShapeInstance;
class PhysicsBody;
class PhysicsWorld;
class PhysicsDebrisData;
class ExplosionData;
class vehicleDataSource;

class NavClient;

enum physicsShapeType
{
	PHYS_SHAPE_BOX = 0,
	PHYS_SHAPE_CAPSULE,
	PHYS_SHAPE_SPHERE,
	PHYS_SHAPE_CONVEX,
	PHYS_SHAPE_COLLISION,
	PHYS_SHAPE_TRIMESH,
	PHYS_SHAPE_TYPE_COUNT
};

struct physicsPartData
{
	S32 jointID;
	S32 baseNode;
	S32 childNode;
	S32 shapeType;
	Point3F dimensions;
	Point3F orientation;
	Point3F offset;
	F32 damageMultiplier;
	bool isInflictor;
	F32 density;
	bool isKinematic;
	bool isNoGravity;
	S32 childVerts;
	S32 parentVerts;
	S32 farVerts;
	F32 weightThreshold;
	F32 ragdollThreshold;
	S32 bodypartChain;
	F32 mass;
	F32 inflictMultiplier;
	Point3F jointRots;
	Point3F jointRots2;
};

struct physicsMountData
{
	S32 parentShape;
	S32 childShape;
	S32 parentNode;
	S32 childNode;
	Point3F offset;
	Point3F orient;
	S32 jointID;
};

class PhysicsShapeData : public GameBaseData
{
   typedef GameBaseData Parent;

   void _onResourceChanged( const Torque::Path &path );

public:

   PhysicsShapeData();
   virtual ~PhysicsShapeData();

   DECLARE_CONOBJECT(PhysicsShapeData);
   static void initPersistFields();   
   bool onAdd();
   void onRemove();
   
   // GameBaseData
   void packData(BitStream* stream);
   void unpackData(BitStream* stream);   
   bool preload(bool server, String &errorBuffer );

public:

   /// The shape to load.
   StringTableEntry shapeName;

   /// The shape resource.
   Resource<TSShape> shape;

   /// The shared unscaled collision shape.
   PhysicsCollisionRef colShape;

   //bool isArticulated; //If so, create arrays of PhysicsBody and PhysicsJoint objects, instead of just one PhysicsBody.
								//NEW WAY: get rid of this, and make mPhysicsRep = mPhysicsBodies[0].

   S32 shapeID;       //Use this to find the bodypart (physicsShapePart) body and joint data in the database.

   F32 mass;
   F32 dynamicFriction;
   F32 staticFriction;
   F32 restitution;
   F32 linearDamping;
   F32 angularDamping;
   F32 linearSleepThreshold;
   F32 angularSleepThreshold;

   // A scale applied to the normal linear and angular damping
   // when the object enters a water volume.
   F32 waterDampingScale;

   // The density of this object used for water buoyancy effects.
   F32 buoyancyDensity;

   // Continuous Collision Detection support,ignored if not supported by underlying plugin
   bool ccdEnabled;

   enum SimType
   {
      /// This physics representation only exists on the client
      /// world and the server only does ghosting.
      SimType_ClientOnly,

      /// The physics representation only exists on the server world
      /// and the client gets delta updates for rendering.
      SimType_ServerOnly,

      /// The physics representation exists on the client and the server
      /// worlds with corrections occuring when the client gets out of sync.
      SimType_ClientServer,

      /// The bits used to pack the SimType field.
      SimType_Bits = 3,

   } simType;
   
   SimObjectRef< PhysicsDebrisData > debris;   
   SimObjectRef< ExplosionData > explosion;   
   SimObjectRef< PhysicsShapeData > destroyedShape;
};

typedef PhysicsShapeData::SimType PhysicsSimType;
DefineEnumType( PhysicsSimType );

class TSThread;


/// A simple single body dynamic physics object.
class PhysicsShape : public GameBase
{
   typedef GameBase Parent;

public: //protected:
   /// Datablock
   PhysicsShapeData *mDataBlock;
   
   ///
   PhysicsWorld *mWorld;

   /// The abstracted physics actor.
   PhysicsBody *mPhysicsRep;//MegaMotion: would like to declare this redundant, and use mPhysicsBodies[0] instead.
   //However this will get confusing and touch on a lot of other files that refer to mPhysicsRep. Needs major refactor.
   
   Vector<PhysicsBody*> mPhysicsBodies; //For articulated shapes. Currently clientside only.
   PhysicsJoint *mJoint;
   Vector<PhysicsJoint*> mPhysicsJoints;
   Vector<S32> mBodyNodes;
   Vector<MatrixF>mLastTrans;//for finding velocity on set dynamic
   MatrixF mStartMat;
   MatrixF mInvStartMat;
   S32 mContactBody;//ID of mPhysicsBody having latest contact with raycast or ... ?

   Point3F mStartPos;
   Point3F mCurrentForce;

   /// The starting position to place the shape when
   /// the level begins or is reset.
   MatrixF mResetPos;

   //For recording physics results into sequences.
   Vector<Quat16>   mNodeRotations;
   Vector<Point3F>  mNodeTranslations;
   Vector<S32> mOrderNodes;

   Point3F mRecordInitialPosition;
   QuatF mRecordInitialOrientation;
   U32 mRecordSampleRate;
   U32 mRecordCount;	
   bool mIsRecording;
   bool mSaveTranslations;

   void setIsRecording(bool);
   S32 getRecordingSize();//Return sum: (nodeRotations.length()*sizeof(Quat16) + nodeTranslations.length()*sizeof(Point3F))
   void recordTick();
   void setupOrderNodes();
   void makeSequence(const char*);

   //VectorF mBuildScale;
   //F32 mBuildAngDrag;
   //F32 mBuildLinDrag;

   TSShapeInstance *mShapeInstance;
   TSShape *mShape;

   Vector <bool> mNodeBodies;//Tracks which nodes have physics.
   /// The current physics state.
   PhysicsState mState;
   Vector <PhysicsState> mStates;//For articulated shapes.

   /// The previous and current render states.
   PhysicsState mRenderState[2];
   Vector <PhysicsState> mRenderStates;//For articulated shapes. Two per part. Maybe?

   /// True if the PhysicsShape has been destroyed ( gameplay ).
   bool mDestroyed;

   /// Enables automatic playing of the animation named "ambient" (if it exists) 
   /// when the PhysicsShape is loaded.
   bool mPlayAmbient;

   S32 mCurrentSeq;//This will be set to whatever sequence we are currently playing, need one per thread when we get that far.
   
   std::map<std::string,int> mActionSeqs;//Associative arrays for C++! This should give us e.g. mActions["attack"] = 15

   S32 mAmbientSeq;//Obsolete, but need to replace all references with mActionSeqs["ambient"] now...
   TSThread* mAmbientThread;//Really this should be called mActiveThread now...

   /// If a specified to create one in the PhysicsShape data, this is the 
   /// subshape created when this PhysicsShape is destroyed.
   /// Is only assigned (non null) on the serverside PhysicsShape.
   SimObjectPtr< PhysicsShape > mDestroyedShape;

   
   bool mHasGravity;// Disables gravity on this object if not set to true.
   bool mIsDynamic;// Sets object to kinematic if true.
   //bool mIsArticulated;// If true, shape maintains arrays of PhysicsBody and PhysicsJoint objects, instead of one PhysicsBody.
   bool mIsGroundMoving;//Probably not the final answer for this, but trying it out for testing. 
   S32 mShapeID;    //Database ID of the physicsShape, to find all the physicsShapePart objects with body and joint data.
   S32 mSceneShapeID;
   S32 mSceneID;
   S32 mSkeletonID;
   S32 mCurrentTick;
   F32 mLastThreadPos;
   Point3F mLastGroundTrans;
   QuatF mLastGroundRot;

   ///
   enum MaskBits 
   {
      StateMask = Parent::NextFreeMask << 0,
      ResetPosMask = Parent::NextFreeMask << 1,
      DamageMask = Parent::NextFreeMask << 2,

      NextFreeMask = Parent::NextFreeMask << 3
   };

   bool _createShape();

   void _initAmbient();

   void _setCurrent();
  
   ///
   void _applyCorrection( const MatrixF &mat );

   void _onPhysicsReset( PhysicsResetEvent reset );

   void _updateContainerForces();

   /// If true then no corrections are sent from the server 
   /// and/or applied from the client.
   ///
   /// This is only ment for debugging.
   ///
   static bool smNoCorrections;

   /// If true then no smoothing is done on the client when
   /// applying server corrections.
   ///
   /// This is only ment for debugging.
   ///
   static bool smNoSmoothing;
	

public:
	
   PhysicsShape();
   virtual ~PhysicsShape();

   DECLARE_CONOBJECT( PhysicsShape );

   // SimObject
   static void consoleInit();
   static void initPersistFields();
   void inspectPostApply();
   bool onAdd();
   void onRemove();   
   
   // SceneObject
   void prepRenderImage( SceneRenderState *state );
   void setTransform( const MatrixF &mat );
   F32 getMass() const;
   Point3F getVelocity() const { return mState.linVelocity; }
   void applyImpulse( const Point3F &pos, const VectorF &vec );
   void applyImpulseToPart( S32 partIndex, const Point3F &pos, const VectorF &vec );
   void applyRadialImpulse( const Point3F &origin, F32 radius, F32 magnitude );
   void applyRadialImpulseToPart( S32 partIndex, const Point3F &origin, F32 radius, F32 magnitude );
   //void setScale(const VectorF & scale);//openSimEarth getting rid of this, it is apparently here just to break setScale.

   // GameBase
   bool onNewDataBlock( GameBaseData *dptr, bool reload );
   void interpolateTick( F32 delta );
   void processTick( const Move *move );
   void advanceTime( F32 timeDelta );
   U32 packUpdate( NetConnection *conn, U32 mask, BitStream *stream );
   void unpackUpdate( NetConnection *conn, BitStream *stream );

   bool isDestroyed() const { return mDestroyed; }
   void destroy();
   void restore();

   /// Save the current transform as where we return to when a physics reset
   /// event occurs. This is automatically set in onAdd but some manipulators
   /// such as Prefab need to make use of this.
   void storeRestorePos();

   void orientToPosition(Point3F pos);
	
   virtual void getNodeTransform( S32 nodeIndex, const MatrixF &xfm, MatrixF *outMat );

   PhysicsBody *getPhysicsRep();
   PhysicsJoint *getPhysicsJoint();

   void setJointTarget(QuatF &target);
   void setHasGravity(bool hasGrav);
   void setPartHasGravity(S32 partID,bool hasGrav);
   void setDynamic(bool isDynamic);
   bool getDynamic();
   void setPartDynamic(S32 partID,bool isDynamic);
   bool getPartDynamic(S32 partID);
   S32 getContactBody();
	S32 getBodyNum(String name);

   void setPosition(Point3F pos);

   Point3F getClientPosition();//probably temporary, still working out ghosting strategy
   
   Point3F findGroundPosition(Point3F pos);//use our mWorld to do a ground raycast and return the contact point.

   bool setActionSeq(const char *name, S32 seq);

   bool setAmbientSeq(const char *name);//Obsolete, but need to replace all references with mActionSeqs["ambient"] now...
   bool setAmbientSeq(S32);

   bool setCurrentSeq(S32);
   bool loadSequence(const char *path);
	
	void setSequenceTimeScale(F32);
	void setSequencePos(F32);
	F32  getSequencePos();

   bool mUseDataSource;
   vehicleDataSource *mDataSource;
	S32 mPropStatus;//Silly but need a variable to record our blade/propblur/propdisc status

	void loadXml(const char *file);

	void updateBodyFromNode(S32 body);
	void updateNodeFromBody(S32 body);

	/////openSimEarth: Aircraft-specific, should probably move these. ////////
	F32 mRudderRange;
	F32 mElevRange;
	F32 mAilerRange;

	Point3F mRudderOffset;
	Point3F mElevOffset;
	Point3F mAilerOffset;
	Point3F mPropOffset;
	Point3F mRotorOffsetA;
	Point3F mRotorOffsetB;
	Point3F mTailRotorOffset;

	F32 mPropBlurSpeed;//Speed at which we switch from blades to blur.
	F32 mPropDiscSpeed;//Speed at which we switch from blur to disc.
	F32 mPropBlurAlpha;//Transparency of blur.
	F32 mPropDiscAlpha;//Transparency of disc.

	Vector<String> mRudderNodes;
	Vector<String> mElevNodes;
	Vector<String> mRightAilerNodes;
	Vector<String> mLeftAilerNodes;
	Vector<String> mPropNodes;
	Vector<String> mRotorNodesA;
	Vector<String> mRotorNodesB;
	Vector<String> mTailRotorNodes;

	void setRotorTransparency(F32 rpm);//Hmm, this smells like something that might also get generalized and moved 
	void setPropTransparency(F32 rpm);// down to sceneObject.

	//And, now let's try moving these up to here again, or at least redefining them here, so I can access shapeInstance.
	void showPropBlades();//left specific but moved to a appropriate vehicle classes.
	void showPropBlur();//Generally better to generalize. Maybe show/hideNodeSet() and then make node sets a thing.
	void showPropDisc();
	void showRotorBlades();
	void showRotorBlur();
	void showRotorDisc();
	
   //////// NavMesh and NavPath //////////////
   //SimObjectPtr<NavMesh> mNavMesh;
	NavMesh *mNavMesh;
	NavPath *mNavPath;
   LinkData mLinkTypes;

	String mNavMeshName;
	S32 mNavPathNode;

	bool setNavMesh();
	bool setNavPathTo(Point3F);

	/////////// OpenSteer //////////////////////
	S32 mPedId;
	//OpenSteer::Vec3 mNavPoints[MAX_NAV_NODES];//wasteful?  better way?//OBSOLETE? use navPaths?
	NavClient *mVehicle;
	bool mUseSteering;

	F32 mWalkSpeed;
	F32 mJogSpeed;
	F32 mRunSpeed;
	F32 mSprintSpeed;

	//void assignVehicleNavPath();
	void assignLastVehicle();
	void updateToVehicle();
	//////OpenSteer////////////

	///// MegaMotion Keyframe Edits //////	
	ultraframeSet mUFSet;//RUH ROH! This needs to be a vector.

	void addUltraframeSet(const char *name);
	void addUltraframeSeries(S32 type,S32 node);
	void addUltraframe(S32 frame,F32 x,F32 y,F32 z);

	void applyUltraframeSet();
	void clearUltraframeSet();

	void cropSequence(U32 seq,F32 start,F32 stop,const char *name);
	
	Vector<QuatF>    nodeCropStartRotations;

	F32 getSeqDeltaSum(S32 seq,S32 currFrame,S32 baseFrame);

	void saveSequence(S32 seq,const char *filename);

	U32 loadBvhCfg(bvhCfgData *cfg,U32 profile_id);
	U32 loadBvhSkeleton(bvhCfgData *cfg,U32 profile_id);
	U32 importBvhSkeleton(const char *bvhFile,const char *profileName);
	void importBvh(bool importGround,const char *bvhFile,const char *bvhProfile,bool cache_dsqs);
	void saveBvh(U32 seqNum, const char *bvh_file, const char *bvh_format, bool isGlobal);

	void setMeshHidden(S32 index,bool hide);
	void importShapeNodes();
	void mountShapes();

	void updateFromOpenSteer();
	void updateToOpenSteer();

//#endif // TORQUE_NAVIGATION_ENABLED
};

#endif // _PHYSICSSHAPE_H_

	/*
	//NavMesh *mNavMesh;

	//grabbed from EM flexbody
	bool setNavMesh();
	void setNavPathTo(Point3F);
	void setNavPathFrom(Point3F);
	*/

	/*
//#ifdef TORQUE_NAVIGATION_ENABLED

	//MegaMotion - in the continuing (temporary?) bloat of PhysicsShape, grabbing entire navmesh 
	//section from AIPlayer. But, testing it out piece by piece. First thing we need to do is find
	//the existing navmesh from the mission.

	
   /// Should we jump?
   enum JumpStates {
      None,  ///< No, don't jump.
      Now,   ///< Jump immediately.
      Ledge, ///< Jump when we walk off a ledge.
   } mJump;

   /// Stores information about a path.
   struct PathData {
      /// Pointer to path object.
      SimObjectPtr<NavPath> path;
      /// Do we own our path? If so, we will delete it when finished.
      bool owned;
      /// Path node we're at.
      U32 index;
      /// Default constructor.
      PathData() : path(NULL)
      {
         owned = false;
         index = 0;
      }
   };

   /// Path we are currently following.
   PathData mPathData;

   /// Clear out the current path.
   void clearPath();

   /// Get the current path we're following.
   NavPath *getPath() { return mPathData.path; }

   /// Stores information about our cover.
   struct CoverData {
      /// Pointer to a cover point.
      SimObjectPtr<CoverPoint> cover;
      /// Default constructor.
      CoverData() : cover(NULL) {}
   };

   /// Current cover we're trying to get to.
   CoverData mCoverData;

   /// Stop searching for cover.
   void clearCover();

   /// Information about a target we're following.
   struct FollowData {
      /// Object to follow.
      SimObjectPtr<SceneObject> object;
      /// Distance at whcih to follow.
      F32 radius;
      Point3F lastPos;
      /// Default constructor.
      FollowData() : object(NULL)
      {
         radius = 5.0f;
         lastPos = Point3F::Zero;
      }
   };

   /// Current object we're following.
   FollowData mFollowData;

   /// Stop following me!
   void clearFollow();

   /// NavMesh we pathfind on.
   SimObjectPtr<NavMesh> mNavMesh;

   /// Move to the specified node in the current path.
   void moveToNode(S32 node);
	*/