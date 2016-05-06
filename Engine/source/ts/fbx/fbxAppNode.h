//-----------------------------------------------------------------------------
// Fbx-2-DTS
// Copyright (C) 2011 BrokeAss Games, LLC
//-----------------------------------------------------------------------------

#ifndef _FBX_APPNODE_H_
#define _FBX_APPNODE_H_

#ifndef _TDICTIONARY_H_
#include "core/tDictionary.h"
#endif
#ifndef _APPNODE_H_
#include "ts/loader/appNode.h"
#endif
#ifndef _FBX_EXTENSIONS_H_
#include "ts/fbx/fbxExtensions.h"
#endif

class FbxAppNode : public AppNode
{
   typedef AppNode Parent;
   friend class FbxAppMesh;
   friend class FbxShapeLoader;

protected:
	
   KFbxNode*             mNode;        ///< Pointer to the node in the Fbx
	FbxAppNode*            appParent;        ///< Parent node in Fbx-space
   //FbxExtension_node*     nodeExt;          ///< node extension
   //Vector<AnimatedFloatList>  nodeTransforms;   ///< Ordered vector of node transform elements (scale, translate etc)
  // bool                       invertMeshes;     ///< True if this node's coordinate space is inverted (left handed)

   Map<StringTableEntry, F32> mProps;           ///< Hash of float properties (converted to int or bool as needed)

   F32                        lastTransformTime;      ///< Time of the last transform lookup (getTransform)
   MatrixF                    lastTransform;          ///< Last transform lookup (getTransform)
   bool                       defaultTransformValid;  ///< Flag indicating whether the defaultNodeTransform is valid
   MatrixF                    defaultNodeTransform;   ///< Transform at DefaultTime
   MatrixF                    tempNodeTransform;      ///< Testing... momentous confusion ensues.
   MatrixF                    mBindTransform; 
   //MatrixF                    mPreRotAccum; 
	Point3F							mFinalPos;
	
	Vector<MatrixF>            nodeTransforms;//For storing anims as they come in.
	Vector<Point3F>            nodeTranslates;//For storing anims as they come in.
	Vector<QuatF>              nodeRotates;//For storing anims as they come in.

public:

	KFbxPose *mBindPose;

   FbxAppNode(KFbxNode* node, FbxAppNode* parent = 0);
   virtual ~FbxAppNode(){ return; }

	const char *getName() { return mName; }
   const char *getParentName() { return mParentName; }

   MatrixF getTransform(F32 time);
   MatrixF getNodeTransform(F32 time);
   bool animatesTransform(const AppSequence* appSeq);
   bool isParentRoot() { return (appParent == NULL); }
   void buildMeshList();
	void buildMesh(FbxAppMesh *mesh);
   void buildChildList();
	void reportModes(KFbxLayerElementNormal*,KFbxLayerElementUV*);


	// Property look-ups: only float properties are stored, the rest are
   // converted from floats as needed
   bool getFloat(const char* propName, F32& defaultVal)
   {
      Map<StringTableEntry,F32>::Iterator itr = mProps.find(propName);
      if (itr != mProps.end())
         defaultVal = itr->value;
      return false;
   }
   bool getInt(const char* propName, S32& defaultVal)
   {
      F32 value = defaultVal;
      bool ret = getFloat(propName, value);
      defaultVal = (S32)value;
      return ret;
   }
   bool getBool(const char* propName, bool& defaultVal)
   {
      F32 value = defaultVal;
      bool ret = getFloat(propName, value);
      defaultVal = (value != 0);
      return ret;
   }
   bool isEqual(AppNode* node)
   {
      const FbxAppNode* appNode = dynamic_cast<const FbxAppNode*>(node);
      return (appNode && (appNode->mNode == mNode));
   }
};


/*


class FbxAppNode : public AppNode
{
   typedef AppNode Parent;
   friend class FbxAppMesh;

   MatrixF getTransform(F32 time);
   void buildMeshList();
   void buildChildList();

protected:


public:

   FbxAppNode(const domNode* node, FbxAppNode* parent = 0);
   virtual ~FbxAppNode()
   {
      delete nodeExt;
      mProps.clear();
   }

   const domNode* getDomNode() const { return p_domNode; }

   //-----------------------------------------------------------------------
   const char *getName() { return mName; }
   const char *getParentName() { return mParentName; }




};
*/
#endif // _FBX_APPNODE_H_
