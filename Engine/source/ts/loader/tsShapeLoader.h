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

#ifndef _TSSHAPE_LOADER_H_
#define _TSSHAPE_LOADER_H_

#ifndef _MMATH_H_
#include "math/mMath.h"
#endif
#ifndef _TVECTOR_H_
#include "core/util/tVector.h"
#endif
#ifndef _TSSHAPE_H_
#include "ts/tsShape.h"
#endif
#ifndef _APPNODE_H_
#include "ts/loader/appNode.h"
#endif
#ifndef _APPMESH_H_
#include "ts/loader/appMesh.h"
#endif
#ifndef _APPSEQUENCE_H_
#include "ts/loader/appSequence.h"
#endif

class TSShapeLoader
{

// Supported Format List
protected:
   struct ShapeFormat
   {
      String mName;
      String mExtension;
   };
   static Vector<ShapeFormat> smFormats;
public:
   static void addFormat(String name, String extension);
   static String getFormatExtensions();
   static String getFormatFilters();

public:
   enum eLoadPhases
   {
      Load_ReadFile = 0,
      Load_ParseFile,
      Load_ExternalRefs,
      Load_EnumerateScene,
      Load_GenerateSubshapes,
      Load_GenerateObjects,
      Load_GenerateDefaultStates,
      Load_GenerateSkins,
      Load_GenerateMaterials,
      Load_GenerateSequences,
      Load_InitShape,
      NumLoadPhases,
      Load_Complete = NumLoadPhases
   };

   static void updateProgress(S32 major, const char* msg, S32 numMinor=0, S32 minor=0);

protected:
   struct Subshape
   {
      Vector<AppNode*>           branches;         ///< Shape branches
      Vector<AppMesh*>           objMeshes;        ///< Object meshes for this subshape
      Vector<S32>                objNodes;         ///< AppNode indices with objects attached

      ~Subshape()
      {
         // Delete children
         for (S32 i = 0; i < branches.size(); i++)
            delete branches[i];
      }
   };

public:
   static const F32 DefaultTime;
   static const double MinFrameRate;
   static const double MaxFrameRate;
   static const double AppGroundFrameRate;
   static const F64 MinFrameRate;
   static const F64 MaxFrameRate;
   static const F64 AppGroundFrameRate;
   bool mHasSequences;//Ecstasy Motion, appSequences.size() crashes if none.

protected:
   // Variables used during loading that must be held until the shape is deleted
   TSShape*                      shape;
   Vector<AppMesh*>              appMeshes;

   // Variables used during loading, but that can be discarded afterwards
   static Torque::Path           shapePath;

   AppNode*                      boundsNode;
   Vector<AppNode*>              appNodes;            ///< Nodes in the loaded shape
   Vector<AppSequence*>          appSequences;

   Vector<Subshape*>             subshapes;

   Vector<QuatF*>                nodeRotCache;
   Vector<Point3F*>              nodeTransCache;
   Vector<QuatF*>                nodeScaleRotCache;
   Vector<Point3F*>              nodeScaleCache;

   Point3F                       shapeOffset;         ///< Offset used to translate the shape origin

   //--------------------------------------------------------------------------

   // Collect the nodes, objects and sequences for the scene
   virtual void enumerateScene() = 0;
   virtual void postEnumerateScene() = 0;

   bool processNode(AppNode* node);
   virtual bool ignoreNode(const String& name) { return false; }
   virtual bool ignoreMesh(const String& name) { return false; }

   void addSkin(AppMesh* mesh);
   void addDetailMesh(AppMesh* mesh);
   void addSubshape(AppNode* node);
   void addObject(AppMesh* mesh, S32 nodeIndex, S32 subShapeNum);

   // Node transform methods
   MatrixF getLocalNodeMatrix(AppNode* node, F32 t);
   void generateNodeTransform(AppNode* node, F32 t, bool blend, F32 referenceTime,
                              QuatF& rot, Point3F& trans, QuatF& srot, Point3F& scale);

   virtual void computeBounds(Box3F& bounds);

   // Create objects, materials and sequences
   void recurseSubshape(AppNode* appNode, S32 parentIndex, bool recurseChildren);

   void generateSubshapes();
   void generateObjects();
   void generateSkins();
   void generateDefaultStates();
   void generateObjectState(TSShape::Object& obj, F32 t, bool addFrame, bool addMatFrame);
   void generateFrame(TSShape::Object& obj, F32 t, bool addFrame, bool addMatFrame);

   void generateMaterialList();

   void generateSequences();

   // Determine what is actually animated in the sequence
   void setNodeMembership(TSShape::Sequence& seq, const AppSequence* appSeq);
   void setRotationMembership(TSShape::Sequence& seq);
   void setTranslationMembership(TSShape::Sequence& seq);
   void setScaleMembership(TSShape::Sequence& seq);
   void setObjectMembership(TSShape::Sequence& seq, const AppSequence* appSeq);

   // Manage a cache of all node transform elements for the sequence
   void clearNodeTransformCache();
   void fillNodeTransformCache(TSShape::Sequence& seq, const AppSequence* appSeq);

   // Add node transform elements
   void addNodeRotation(QuatF& rot, bool defaultVal);
   void addNodeTranslation(Point3F& trans, bool defaultVal);
   void addNodeUniformScale(F32 scale);
   void addNodeAlignedScale(Point3F& scale);
   void addNodeArbitraryScale(QuatF& qrot, Point3F& scale);

   // Generate animation data
   void generateNodeAnimation(TSShape::Sequence& seq);
   void generateObjectAnimation(TSShape::Sequence& seq, const AppSequence* appSeq);
   void generateGroundAnimation(TSShape::Sequence& seq, const AppSequence* appSeq);
   void generateFrameTriggers(TSShape::Sequence& seq, const AppSequence* appSeq);

   // Shape construction
   void sortDetails();
   void install();

public:
   TSShapeLoader() : boundsNode(0) { }
   virtual ~TSShapeLoader();

   static const Torque::Path& getShapePath() { return shapePath; }

   static void zapScale(MatrixF& mat);

   TSShape* generateShape(const Torque::Path& path);
};


//TEMP: This is embarrassing but I ran into trouble with undefined externals when I tried to put
//these things in the right way, for now tacking them here..

//QuatTypes.h
typedef struct {F32 x, y, z, w;} Quat; /* Quaternion */
enum QuatPart {X, Y, Z, W};
typedef F32 HMatrix[4][4]; /* Right-handed, for column vectors */
typedef Quat EulerAngles;    /* (x,y,z)=ang 1,2,3, w=order code  */

//EulerAngles.h
#define EulFrmS	     0
#define EulFrmR	     1
#define EulFrm(ord)  ((unsigned)(ord)&1)
#define EulRepNo     0
#define EulRepYes    1
#define EulRep(ord)  (((unsigned)(ord)>>1)&1)
#define EulParEven   0
#define EulParOdd    1
#define EulPar(ord)  (((unsigned)(ord)>>2)&1)
#define EulSafe	     "\000\001\002\000"
#define EulNext	     "\001\002\000\001"
#define EulAxI(ord)  ((int)(EulSafe[(((unsigned)(ord)>>3)&3)]))
#define EulAxJ(ord)  ((int)(EulNext[EulAxI(ord)+(EulPar(ord)==EulParOdd)]))
#define EulAxK(ord)  ((int)(EulNext[EulAxI(ord)+(EulPar(ord)!=EulParOdd)]))
#define EulAxH(ord)  ((EulRep(ord)==EulRepNo)?EulAxK(ord):EulAxI(ord))
    /* EulGetOrd unpacks all useful information about order simultaneously. */
#define EulGetOrd(ord,i,j,k,h,n,s,f) {unsigned o=ord;f=o&1;o>>=1;s=o&1;o>>=1;n=o&1;o>>=1;i=EulSafe[o&3];j=EulNext[i+n];k=EulNext[i+1-n];h=s?k:i;}
    /* EulOrd creates an order value between 0 and 23 from 4-tuple choices. */
#define EulOrd(i,p,r,f)	   (((((((i)<<1)+(p))<<1)+(r))<<1)+(f))
    /* Static axes */
#define EulOrdXYZs    EulOrd(X,EulParEven,EulRepNo,EulFrmS)
#define EulOrdXYXs    EulOrd(X,EulParEven,EulRepYes,EulFrmS)
#define EulOrdXZYs    EulOrd(X,EulParOdd,EulRepNo,EulFrmS)
#define EulOrdXZXs    EulOrd(X,EulParOdd,EulRepYes,EulFrmS)
#define EulOrdYZXs    EulOrd(Y,EulParEven,EulRepNo,EulFrmS)
#define EulOrdYZYs    EulOrd(Y,EulParEven,EulRepYes,EulFrmS)
#define EulOrdYXZs    EulOrd(Y,EulParOdd,EulRepNo,EulFrmS)
#define EulOrdYXYs    EulOrd(Y,EulParOdd,EulRepYes,EulFrmS)
#define EulOrdZXYs    EulOrd(Z,EulParEven,EulRepNo,EulFrmS)
#define EulOrdZXZs    EulOrd(Z,EulParEven,EulRepYes,EulFrmS)
#define EulOrdZYXs    EulOrd(Z,EulParOdd,EulRepNo,EulFrmS)
#define EulOrdZYZs    EulOrd(Z,EulParOdd,EulRepYes,EulFrmS)
    /* Rotating axes */
#define EulOrdZYXr    EulOrd(X,EulParEven,EulRepNo,EulFrmR)
#define EulOrdXYXr    EulOrd(X,EulParEven,EulRepYes,EulFrmR)
#define EulOrdYZXr    EulOrd(X,EulParOdd,EulRepNo,EulFrmR)
#define EulOrdXZXr    EulOrd(X,EulParOdd,EulRepYes,EulFrmR)
#define EulOrdXZYr    EulOrd(Y,EulParEven,EulRepNo,EulFrmR)
#define EulOrdYZYr    EulOrd(Y,EulParEven,EulRepYes,EulFrmR)
#define EulOrdZXYr    EulOrd(Y,EulParOdd,EulRepNo,EulFrmR)
#define EulOrdYXYr    EulOrd(Y,EulParOdd,EulRepYes,EulFrmR)
#define EulOrdYXZr    EulOrd(Z,EulParEven,EulRepNo,EulFrmR)
#define EulOrdZXZr    EulOrd(Z,EulParEven,EulRepYes,EulFrmR)
#define EulOrdXYZr    EulOrd(Z,EulParOdd,EulRepNo,EulFrmR)
#define EulOrdZYZr    EulOrd(Z,EulParOdd,EulRepYes,EulFrmR)

EulerAngles Eul_(F32 ai, F32 aj, F32 ah, S32 order);
Quat Eul_ToQuat(EulerAngles ea);
void Eul_ToHMatrix(EulerAngles ea, HMatrix M);
EulerAngles Eul_FromHMatrix(HMatrix M, S32 order);
EulerAngles Eul_FromQuat(Quat q, S32 order);

#endif // _TSSHAPE_LOADER_H_
