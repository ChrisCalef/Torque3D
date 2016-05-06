//-----------------------------------------------------------------------------
// Fbx-2-DTS
// Copyright (C) 2011 BrokeAss Games, LLC
//-----------------------------------------------------------------------------

#ifndef _FBX_APPMESH_H_
#define _FBX_APPMESH_H_

#ifndef _TDICTIONARY_H_
#include "core/tDictionary.h"
#endif
#ifndef _APPMESH_H_
#include "ts/loader/appMesh.h"
#endif
#ifndef _TSSHAPELOADER_H_
#include "ts/loader/tsShapeLoader.h"
#endif
#ifndef _FBX_APPNODE_H_
#include "ts/fbx/fbxAppNode.h"
#endif
#ifndef _FBX_EXTENSIONS_H_
#include "ts/fbx/fbxExtensions.h"
#endif

//-----------------------------------------------------------------------------
// Torque unifies the vert position, normal and UV values, so that a single index
// uniquely identifies all 3 elements. A triangle then contains just 3 indices,
// and from that we can get the 3 positions, 3 normals and 3 UVs.
//
// for i=1:3
//    index = indices[triangle.start + i]
//    points[index], normals[index], uvs[index]
//
// I don't know if Fbx uses unified vertex streams or not.

struct VertTuple
{
   S32 prim, vertex, normal, color, uv, uv2;

   Point3F dataVertex, dataNormal;
   ColorI  dataColor;
   Point2F dataUV, dataUV2;

   VertTuple(): prim(-1), vertex(-1), normal(-1), color(-1), uv(-1), uv2(-1) {}
   bool operator==(const VertTuple& p) const 
   {
      return   dataVertex == p.dataVertex &&
               dataColor == p.dataColor &&  
               dataNormal == p.dataNormal &&   
               dataUV == p.dataUV &&  
               dataUV2 == p.dataUV2;
   }
};


struct VertRef
{//Struct to link new vertices created to deal with UVs issue with the original vertices they are copying.
	U32 base;//index of the base vertex.
	U32 index;//index of the new vertex.
};

class FbxAppMesh : public AppMesh
{
   typedef AppMesh Parent;
   friend class FbxAppNode;
	friend class FbxShapeLoader;
	
	MatrixF mOrientation;
	Point3F mOffset;

protected:
   class FbxAppNode* appNode;    ///< Pointer to the node that owns this mesh
	KFbxMesh *mMesh;
	Vector<VertRef> vertRefs;		

   /*
	const domInstance_geometry* instanceGeom;
   const domInstance_controller* instanceCtrl;
   FbxExtension_geometry* geomExt;                ///< geometry extension

   Vector<VertTuple> vertTuples;                      ///<
   Map<StringTableEntry,U32> boundMaterials;          ///< Local map of symbols to materials

   //-----------------------------------------------------------------------

   /// Get the morph controller for this mesh (if any)
   const domMorph* getMorph()
   {
      if (instanceCtrl) {
         const domController* ctrl = daeSafeCast<domController>(instanceCtrl->getUrl().getElement());
         if (ctrl && ctrl->getSkin())
            ctrl = daeSafeCast<domController>(ctrl->getSkin()->getSource().getElement());
         return ctrl ? ctrl->getMorph() : NULL;
      }
      return NULL;
   }

   S32 addMaterial(const char* symbol);

   bool checkGeometryType(const daeElement* element);
   void getPrimitives(const domGeometry* geometry);

   void getVertexData(  const domGeometry* geometry, F32 time, const MatrixF& objectOffset,
                        Vector<Point3F>& points, Vector<Point3F>& norms, Vector<ColorI>& colors, 
                        Vector<Point2F>& uvs, Vector<Point2F>& uv2s, bool appendValues);

   void getMorphVertexData(   const domMorph* morph, F32 time, const MatrixF& objectOffset,
                              Vector<Point3F>& points, Vector<Point3F>& norms, Vector<ColorI>& colors,
                              Vector<Point2F>& uvs, Vector<Point2F>& uv2s );
*/
   static bool fixedSizeEnabled;                      ///< Set to true to fix the detail size to a particular value for all geometry
   static S32 fixedSize;                              ///< The fixed detail size value for all geometry

public:
	FbxAppMesh(KFbxMesh *mesh,FbxAppNode *node);
   //FbxAppMesh(const domInstance_geometry* instance, FbxAppNode* node);
   //FbxAppMesh(const domInstance_controller* instance, FbxAppNode* node);
   ~FbxAppMesh()
   {
      //delete geomExt;
   }

   static void fixDetailSize(bool fixed, S32 size=2)
   {
      fixedSizeEnabled = fixed;
      fixedSize = size;
   }

   /// Get the name of this mesh
   ///
   /// @return A string containing the name of this mesh
   const char *getName(bool allowFixed=true);

   //-----------------------------------------------------------------------

   /// Get a floating point property value
   ///
   /// @param propName     Name of the property to get
   /// @param defaultVal   Reference to variable to hold return value
   ///
   /// @return True if a value was set, false if not
   bool getFloat(const char *propName, F32 &defaultVal)
   {
      return appNode->getFloat(propName,defaultVal);
   }

   /// Get an integer property value
   ///
   /// @param propName     Name of the property to get
   /// @param defaultVal   Reference to variable to hold return value
   ///
   /// @return True if a value was set, false if not
   bool getInt(const char *propName, S32 &defaultVal)
   {
      return appNode->getInt(propName,defaultVal);
   }

   /// Get a boolean property value
   ///
   /// @param propName     Name of the property to get
   /// @param defaultVal   Reference to variable to hold return value
   ///
   /// @return True if a value was set, false if not
   bool getBool(const char *propName, bool &defaultVal)
   {
      return appNode->getBool(propName,defaultVal);
   }

	bool mIsSkin;
   /// Return true if this mesh is a skin
   bool isSkin()
   {//HERE: look to see if we have valid clusters, if so then make a skinmesh


      //if (instanceCtrl) {
      //   const domController* ctrl = daeSafeCast<domController>(instanceCtrl->getUrl().getElement());
      //   if (ctrl && ctrl->getSkin() &&
      //      (ctrl->getSkin()->getVertex_weights()->getV()->getValue().getCount() > 0))
      //      return true;
      //}

		//return true;
      return mIsSkin;
   }

   /// Get the skin data: bones, vertex weights etc
   void lookupSkinData();

   /// Check if the mesh visibility is animated
   ///
   /// @param appSeq   Start/end time to check
   ///
   /// @return True if the mesh visibility is animated, false if not
   bool animatesVis(const AppSequence* appSeq);

   /// Check if the material used by this mesh is animated
   ///
   /// @param appSeq   Start/end time to check
   ///
   /// @return True if the material is animated, false if not
   bool animatesMatFrame(const AppSequence* appSeq);

   /// Check if the mesh is animated
   ///
   /// @param appSeq   Start/end time to check
   ///
   /// @return True if the mesh is animated, false if not
   bool animatesFrame(const AppSequence* appSeq);

   /// Generate the vertex, normal and triangle data for the mesh.
   ///
   /// @param time           Time at which to generate the mesh data
   /// @param objectOffset   Transform to apply to the generated data (bounds transform)
   void lockMesh(F32 time, const MatrixF& objectOffset);

   /// Get the transform of this mesh at a certain time
   ///
   /// @param time   Time at which to get the transform
   ///
   /// @return The mesh transform at the specified time
   MatrixF getMeshTransform(F32 time);

   /// Get the visibility of this mesh at a certain time
   ///
   /// @param time   Time at which to get visibility info
   ///
   /// @return Visibility from 0 (invisible) to 1 (opaque)
   F32 getVisValue(F32 time);
};

#endif // _Fbx_APPMESH_H_
