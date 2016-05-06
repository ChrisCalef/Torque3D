//-----------------------------------------------------------------------------
// Fbx-2-DTS
// Copyright (C) 2011 BrokeAss Games, LLC
//-----------------------------------------------------------------------------

#ifndef _FBX_SHAPELOADER_H_
#define _FBX_SHAPELOADER_H_

#ifndef _TSSHAPELOADER_H_
#include "ts/loader/tsShapeLoader.h"
#endif


#include "ts/fbx/fbxUtils.h"
#include "ts/fbx/fbxAppNode.h"
#include "ts/fbx/fbxAppMesh.h"
#include "ts/fbx/fbxAppMaterial.h"
#include "ts/fbx/fbxAppSequence.h"


class FbxShapeLoader : public TSShapeLoader
{

   friend TSShape *loadFbxShape(const Torque::Path &path);
	
	KFbxSdkManager *mSdkManager;
	KFbxScene *mScene;

	KFbxAnimEvaluator* mSceneEvaluator;
	KFbxImporter *mImporter;

	KFbxNode *mFbxRootNode;
	KFbxNode *mFbxShapeRootNode;//root node for this shape, with skeletons, meshes, etc.(?)
	KFbxNode *mFbxSkeletonBaseNode;//base node for the skeleton, ie hips for humanoids

	KFbxPose *mBindPose;

	S32 mFbxLimbNodes;
	S32 mFbxMeshes;
	S32 mFbxTextures;
	//S32 mFbxSequences;
	S32 mFbxClusters;
	S32 mFbxActors;
	F32 mFbxScale;
	bool mFoundLimbNode;//Hacks to help find the correct
	bool mForceReadNode;//skeleton root node.

   //Vector<AnimChannels*>   animations;       ///< Holds all animation channels for deletion after loading
   //void processAnimation(const domAnimation* anim, F32& maxEndTime, F32& minFrameTime);
   //void cleanup();

	//KFbxImporter* mImporter;
	//KFbxExporter* mExporter;

public:
	FbxShapeLoader();
	~FbxShapeLoader();

	bool SaveScene(const char* pFilename, int pFileFormat=-1, bool pEmbedMedia=false);
	bool LoadScene(const char* pFilename);

	//void AddThumbnailToScene();
	//void AddThumbnailToTake(KString& pTakeName, int pThumbnailIndex);

   static bool canLoadCachedDTS(const Torque::Path& path);
	
   void enumerateScene();
	void postEnumerateScene();

	void examineScene(KFbxImporter *importer);
	void examineMember(KFbxObject *member);
	//void findSequences(KFbxObject *member);
	void readNode(KFbxNode *node);
   //bool ignoreNode(const String& name);
   //bool ignoreMesh(const String& name);
   //void computeBounds(Box3F& bounds);

   void fbxGenerateNodeTransform(AppNode* node, S32 frame, bool blend, F32 referenceTime,
                              QuatF& rot, Point3F& trans, QuatF& srot, Point3F& scale);
};





/*
class domFbx;
class domAnimation;
struct AnimChannels;

//-----------------------------------------------------------------------------
class FbxShapeLoader : public TSShapeLoader
{
   friend TSShape* loadFbxShape(const Torque::Path &path);

   domFbx*             root;
   Vector<AnimChannels*>   animations;       ///< Holds all animation channels for deletion after loading

   void processAnimation(const domAnimation* anim, F32& maxEndTime, F32& minFrameTime);

   void cleanup();

public:
   FbxShapeLoader(domFbx* _root);
   ~FbxShapeLoader();

   void enumerateScene();
   bool ignoreNode(const String& name);
   bool ignoreMesh(const String& name);
   void computeBounds(Box3F& bounds);

   static bool canLoadCachedDTS(const Torque::Path& path);
   static bool checkAndMountSketchup(const Torque::Path& path, String& mountPoint, Torque::Path& daePath);
   static domFbx* getDomFbx(const Torque::Path& path);
   static domFbx* readFbxFile(const String& path);
};
*/
#endif // _FBX_SHAPELOADER_H_
