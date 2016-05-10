//-----------------------------------------------------------------------------
// Fbx-2-DTS
// Copyright (C) 2011 BrokeAss Games, LLC
//-----------------------------------------------------------------------------

#include "platform/platform.h"

#ifdef _MSC_VER
#pragma warning(disable : 4706)  // disable warning about assignment within conditional
#endif

#include "ts/loader/appSequence.h"
#include "ts/fbx/fbxExtensions.h"
#include "ts/fbx/fbxAppNode.h"
#include "ts/fbx/fbxAppMesh.h"
#include "T3D/shapeBase.h"

#include "core/stringTable.h"

#include "math/util/EulerAngles.h"

extern KFbxPose *gBindPose;//TEMP, until we figure out how to pass this from fbxShapeLoader.
/*
// Trim leading and trailing whitespace from the first word in the string
// Note that the string is modified.
static char* TrimFirstWord(char* str)
{
   char* value = str;

   // Trim leading whitespace
   while ( value && *value && dIsspace( *value ) )
      value++;

   // Trim trailing whitespace
   if ( value && *value )
   {
      char* end = value + 1;
      while ( *end && !dIsspace( *end ) )
         end++;
      *end = '\0';
   }

   return value;
}
*/

FbxAppNode::FbxAppNode(KFbxNode* node, FbxAppNode* parent)
     // : p_domNode(node), appParent(parent), nodeExt(new FbxExtension_node(node)),
     // lastTransformTime(TSShapeLoader::DefaultTime-1), defaultTransformValid(false),
    //  invertMeshes(false)
{
	mName = dStrdup(node->GetName());
	mParentName = dStrdup(parent ? parent->getName() : "ROOT");
	mNode = node;
	mScaleFactor = 1.0/100.0;//TEMP, dammit, why doesn't it work to pass it from fbxShapeLoader??
	
	Point3F pos,pos2;
	MatrixF defTransform,bindTransform,invBindTransform,origTransform,transform,parentM;
	EulerAngles ea;
	HMatrix	hMatrix;
	MatrixF m1;
	//Point4F row0,row1,row2;
	KFbxVector4 tVec;
	KFbxVector4 rVec;
	//EPivotSet pivSet;
	node->GetDefaultT(tVec);
	node->GetDefaultR(rVec);
	//pos.set(-tVec[0],tVec[2],tVec[1]);//OOPS, this would be twice, tsShapeLoader already does this.
	origTransform.set(EulerF(mDegToRad(rVec[0]),mDegToRad(rVec[1]),mDegToRad(rVec[2])));
	//origTransform.set(EulerF(mDegToRad(rVec[0]),-mDegToRad(rVec[2]),-mDegToRad(rVec[1])));
	//transform.set(EulerF(mDegToRad(rVec[0]),mDegToRad(rVec[2]),mDegToRad(rVec[1])));
	//transform.set(EulerF(mDegToRad(rVec[1]),mDegToRad(rVec[0]),mDegToRad(rVec[2])));
	//transform.set(EulerF(mDegToRad(rVec[1]),mDegToRad(rVec[2]),mDegToRad(rVec[0])));
	//transform.set(EulerF(mDegToRad(rVec[2]),mDegToRad(rVec[0]),mDegToRad(rVec[1])));
	//transform.set(EulerF(mDegToRad(rVec[2]),mDegToRad(rVec[1]),mDegToRad(rVec[0])));

	//EulerF eul(mDegToRad(rVec[0]),mDegToRad(rVec[1]),mDegToRad(rVec[2]));
	////ea = Eul_(eul.x,eul.y,eul.z,EulOrdXYZs);
	////ea = Eul_(eul.x,eul.z,eul.y,EulOrdXZYs);
	//ea = Eul_(eul.y,eul.z,eul.x,EulOrdYZXs);
	////ea = Eul_(eul.y,eul.x,eul.z,EulOrdYXZs);
	////ea = Eul_(eul.z,eul.x,eul.y,EulOrdZXYs);
	////ea = Eul_(eul.z,eul.y,eul.x,EulOrdZYXs);
	//Eul_ToHMatrix( ea,hMatrix);

	//row0.x = hMatrix[0][0]; row0.y = hMatrix[1][0]; row0.z = hMatrix[2][0];
	//row1.x = hMatrix[0][1]; row1.y = hMatrix[1][1]; row1.z = hMatrix[2][1];
	//row2.x = hMatrix[0][2]; row2.y = hMatrix[1][2]; row2.z = hMatrix[2][2];

	//transform.setRow(0,row0);
	//transform.setRow(1,row1);
	//transform.setRow(2,row2);


	///TESTING
	KFbxNode::EPivotSet pivotSource = KFbxNode::eSOURCE_SET;
	KFbxNode::EPivotSet pivotDest = KFbxNode::eDESTINATION_SET;

	ERotationOrder rotOrder;
	node->GetRotationOrder(pivotSource,rotOrder);
	//Con::printf("Node rotation order: %d",rotOrder);

	KFbxVector4 geomRotSrc = node->GetGeometricRotation(pivotSource);
	KFbxVector4 geomRotDest = node->GetGeometricRotation(pivotDest);
	KFbxVector4 postRotSrc = node->GetPostRotation(pivotSource);
	KFbxVector4 postRotDest = node->GetPostRotation(pivotDest);
	KFbxVector4 preRotSrc = node->GetPreRotation(pivotSource);
	KFbxVector4 preRotDest = node->GetPreRotation(pivotDest);
	KFbxVector4 rotOffsetSrc = node->GetRotationOffset(pivotSource);
	KFbxVector4 rotOffsetDest = node->GetRotationOffset(pivotDest);
	KFbxVector4 rotPivotSrc = node->GetRotationPivot(pivotSource);
	KFbxVector4 rotPivotDest = node->GetRotationPivot(pivotDest);

	//Con::printf("GeometricRotation, source %f %f %f %f   dest %f %f %f %f",
	//	geomRotSrc[0],geomRotSrc[1],geomRotSrc[2],geomRotSrc[3],
	//	geomRotDest[0],geomRotDest[1],geomRotDest[2],geomRotDest[3]);
	//Con::printf("PostRotation, source %f %f %f %f   dest %f %f %f %f",
	//	postRotSrc[0],postRotSrc[1],postRotSrc[2],postRotSrc[3],
	//	postRotDest[0],postRotDest[1],postRotDest[2],postRotDest[3]);
	//Con::printf("PreRotation, source %f %f %f %f   dest %f %f %f %f",
	//	preRotSrc[0],preRotSrc[1],preRotSrc[2],preRotSrc[3],
	//	preRotDest[0],preRotDest[1],preRotDest[2],preRotDest[3]);
	//Con::printf("rotOffset, source %f %f %f %f   dest %f %f %f %f",
	//	rotOffsetSrc[0],rotOffsetSrc[1],rotOffsetSrc[2],rotOffsetSrc[3],
	//	rotOffsetDest[0],rotOffsetDest[1],rotOffsetDest[2],rotOffsetDest[3]);
	//Con::printf("rotPivot, source %f %f %f %f   dest %f %f %f %f",
	//	rotPivotSrc[0],rotPivotSrc[1],rotPivotSrc[2],rotPivotSrc[3],
	//	rotPivotDest[0],rotPivotDest[1],rotPivotDest[2],rotPivotDest[3]);

	//MatrixF geomRotMat,postRotMat,preRotMat,rotOffsetMat,rotPivotMat,finalMult;
	//geomRotMat.set(EulerF(mDegToRad(geomRotSrc[0]),mDegToRad(geomRotSrc[1]),mDegToRad(geomRotSrc[2])));
	//postRotMat.set(EulerF(mDegToRad(postRotSrc[0]),mDegToRad(postRotSrc[1]),mDegToRad(postRotSrc[2])));
	//preRotMat.set(EulerF(mDegToRad(preRotSrc[0]),mDegToRad(preRotSrc[1]),mDegToRad(preRotSrc[2])));
	//rotOffsetMat.set(EulerF(mDegToRad(rotOffsetSrc[0]),mDegToRad(rotOffsetSrc[1]),mDegToRad(rotOffsetSrc[2])));
	//rotPivotMat.set(EulerF(mDegToRad(preRotSrc[0]),mDegToRad(preRotSrc[1]),mDegToRad(preRotSrc[2])));

	//HERE: haven't found a model that uses any of these except preRotMat at this time, but storing them for later.
	MatrixF geomRotMat,postRotMat,preRotMat,rotOffsetMat,rotPivotMat,finalMult;
	geomRotMat.set(EulerF(mDegToRad(geomRotSrc[0]),-mDegToRad(geomRotSrc[2]),-mDegToRad(geomRotSrc[1])));
	postRotMat.set(EulerF(mDegToRad(postRotSrc[0]),-mDegToRad(postRotSrc[2]),-mDegToRad(postRotSrc[1])));	
	preRotMat.set(EulerF(mDegToRad(preRotSrc[0]),-mDegToRad(preRotSrc[2]),-mDegToRad(preRotSrc[1])));	
	rotOffsetMat.set(EulerF(mDegToRad(rotOffsetSrc[0]),-mDegToRad(rotOffsetSrc[2]),-mDegToRad(rotOffsetSrc[1])));
	rotPivotMat.set(EulerF(mDegToRad(preRotSrc[0]),-mDegToRad(preRotSrc[2]),-mDegToRad(preRotSrc[1])));

	invBindTransform.identity();
	if (gBindPose)
	{
		for (U32 i=0;i<gBindPose->GetCount();i++)
		{
			Point4F kRow;
			if (gBindPose->GetNode(i)==node)
			{
				KFbxMatrix matrix = gBindPose->GetMatrix(i);	
				KFbxVector4 Row0 = matrix.GetRow(0);
				KFbxVector4 Row1 = matrix.GetRow(1);
				KFbxVector4 Row2 = matrix.GetRow(2);
				KFbxVector4 Row3 = matrix.GetRow(3);
				//Con::printf("Row0: %f  %f  %f  %f",Row0[0],Row0[1],Row0[2],Row0[3]);
				//Con::printf("Row1: %f  %f  %f  %f",Row1[0],Row1[1],Row1[2],Row1[3]);
				//Con::printf("Row2: %f  %f  %f  %f",Row2[0],Row2[1],Row2[2],Row2[3]);
				//Con::printf("Row3: %f  %f  %f  %f",Row3[0],Row3[1],Row3[2],Row3[3]);
				kRow.set(Row0[0],Row0[1],Row0[2],Row0[3]); bindTransform.setRow(0,kRow);
				kRow.set(Row1[0],Row1[1],Row1[2],Row1[3]); bindTransform.setRow(1,kRow);
				kRow.set(Row2[0],Row2[1],Row2[2],Row2[3]); bindTransform.setRow(2,kRow);
				kRow.set(Row3[0],Row3[1],Row3[2],Row3[3]); bindTransform.setRow(3,kRow);
				EulerF kEuler = bindTransform.toEuler();
				MatrixF newBindTrans(kEuler);
				//EulerF kEuler2 = EulerF(kEuler.x,-kEuler.z,-kEuler.y); //COORDSHIFT
				//EulerF kEuler2 = EulerF(-kEuler.x,kEuler.y,kEuler.z);
				//EulerF kEuler2 = EulerF(kEuler.x,kEuler.y,kEuler.z);
				//MatrixF newBindTrans(kEuler2);
				//newBindTrans.mul(transform);
				//MatrixF newBindTrans = bindTrans;
				//newBindTrans.mul(preRotMat);
				//Con::printf("Original bind transform for node %s!!!  Euler %f %f %f ",
				//	mName,mRadToDeg(kEuler.x),mRadToDeg(kEuler.y),mRadToDeg(kEuler.z));

				//GUESSING HERE...??
				invBindTransform.identity();
				//invBindTransform.mul(transform);
				//invBindTransform
				invBindTransform.mul(preRotMat);//should this be before or after the bindTransform?
				//MatrixF invPreRot = preRotMat;
				//invBindTransform.mul(invPreRot.inverse());
				//invBindTransform.mul(newBindTrans.inverse());
				invBindTransform.mul(newBindTrans);
				//invBindTransform.inverse();
				//invBindTransform.mul(invPreRot.inverse());
				//mBindTransform = newBindTrans; 
				mBindTransform.identity();//Hm, I don't think this part was actually doing anything anywhere...
			}
		}
	}

	//pos.set(-tVec[0],tVec[2],tVec[1]);
	pos.set(tVec[0],tVec[1],tVec[2]);
	pos *= mScaleFactor;

	transform = invBindTransform;
	//transform = origTransform;//OR THIS?  Should this be an accumulation of transforms from all parents?
	//Con::printf("Node local pos: %f %f %f, rVec %f %f %f",tVec[0],tVec[1],tVec[2],rVec[0],rVec[1],rVec[2]);
	if (parent)
	{
		//mPreRotAccum = parent->mPreRotAccum * preRotMat;
		//transform.mul(parent->mPreRotAccum);
		transform.mulP(pos,&pos2);
		parentM = parent->defaultNodeTransform;
		Point3F pPos = parentM.getPosition();
		//defaultNodeTransform =  origTransform * parentM.inverse();
		//transform.mul(parentM);
		//transform.mulP(pos,&pos2);
		//KFbxVector4 parentPos;
		//Point3F pPos,pPos2;
		//parent->mNode->GetDefaultT(parentPos);
		//pPos = Point3F(parentPos[0],parentPos[1],parentPos[2]);
		pos2 += pPos;
	} else {
		//mPreRotAccum = preRotMat;
		transform.mulP(pos,&pos2);
		//defaultNodeTransform = origTransform;
		//pos2 = pos;
	}
	defaultNodeTransform.identity();
	//defaultNodeTransform.set(EulerF(mDegToRad(rVec[0]),mDegToRad(rVec[1]),mDegToRad(rVec[2])));
	defaultNodeTransform.setPosition(pos2);
	
	//Con::printf("%s position %f %f %f",node->GetName(),pos2.x,pos2.y,pos2.z);
	
	//Con::printf("processing node: %s",mName);



	/*
	KFbxMesh* mesh = NULL;
	mesh = (KFbxMesh*) node->GetNodeAttribute();
	if (mesh)
	{
		Con::printf("We have a mesh for node %s.",node->GetName());
		int numSkins = mesh->GetDeformerCount();
		for( int s = 0; s < numSkins; ++s)
		{
			KFbxSkin* skin = (KFbxSkin*)mesh->GetDeformer(s);
			int numClusters = (int)skin->GetClusterCount();
			Con::printf("Skin %d has %d clusters.",s,numClusters);
			for(int c = 0 ; c < numClusters; ++c)
			{
				KFbxCluster* cluster = skin->GetCluster(c);
				KFbxNode* boneNode = cluster->GetLink();

				if ( boneNode == NULL )
				{
					Con::printf(" Cluster [%d] No bone attached\n",c);
					continue;
				}

				// Get Bind position ////////////////////////
				KFbxXMatrix bindMatrix;
				cluster->GetTransformLinkMatrix(bindMatrix);
				if (boneNode == node)
					Con::printf("WE FOUND OUR BONE NODE!");
			}
		}
	}
*/

	/*
   mName = dStrdup(_GetNameOrId(node));
   mParentName = dStrdup(parent ? parent->getName() : "ROOT");

   // Extract user properties from the <node> extension as whitespace separated
   // "name=value" pairs
   char* properties = dStrdup(nodeExt->user_properties);
   char* pos = properties;
   char* end = properties + dStrlen( properties );
   while ( pos < end )
   {
      // Find the '=' character to separate the name and value pair
      char* split = dStrchr( pos, '=' );
      if ( !split )
         break;

      // Get the name (whitespace trimmed string up to the '=')
      // and value (whitespace trimmed string after the '=')
      *split = '\0';
      char* name = TrimFirstWord( pos );
      char* value = TrimFirstWord( split + 1 );

      mProps.insert(StringTable->insert(name), dAtof(value));

      pos = value + dStrlen( value ) + 1;
   }

   dFree( properties );

   // Create vector of transform elements
   for (int iChild = 0; iChild < node->getContents().getCount(); iChild++) {
      switch (node->getContents()[iChild]->getElementType()) {
         case Fbx_TYPE::TRANSLATE:
         case Fbx_TYPE::ROTATE:
         case Fbx_TYPE::SCALE:
         case Fbx_TYPE::SKEW:
         case Fbx_TYPE::MATRIX:
         case Fbx_TYPE::LOOKAT:
            nodeTransforms.increment();
            nodeTransforms.last().element = node->getContents()[iChild];
            break;
      }
   }
	*/
}

// Get all child nodes
void FbxAppNode::buildChildList()
{
	for (U32 i=0;i<mNode->GetChildCount();i++)
	{
		KFbxNode *child = mNode->GetChild(i);
		if (!strcmp(child->GetTypeName().Buffer(),"LimbNode"))
		{
			mChildNodes.push_back(new FbxAppNode(child, this));
			mChildNodes.last()->mScaleFactor = mScaleFactor;
			dynamic_cast<FbxAppNode*>(mChildNodes.last())->mBindPose = mBindPose;
		}
	}

	/*
   // Process children: collect <node> and <instance_node> elements
   for (int iChild = 0; iChild < p_domNode->getContents().getCount(); iChild++) {

      daeElement* child = p_domNode->getContents()[iChild];
      switch (child->getElementType()) {

         case Fbx_TYPE::NODE:
         {
            domNode* node = daeSafeCast<domNode>(child);
            mChildNodes.push_back(new FbxAppNode(node, this));
            break;
         }

         case Fbx_TYPE::INSTANCE_NODE:
         {
            domInstance_node* instanceNode = daeSafeCast<domInstance_node>(child);
            domNode* node = daeSafeCast<domNode>(instanceNode->getUrl().getElement());
            if (node)
               mChildNodes.push_back(new FbxAppNode(node, this));
            else
               Con::warnf("Failed to resolve instance_node with url=%s", instanceNode->getUrl().originalStr().c_str());
            break;
         }
      }
   }*/
}

//New way - for fbx, build meshes one at a time here.
void FbxAppNode::buildMesh(FbxAppMesh *appMesh)
{
	KFbxMesh *kMesh = appMesh->mMesh;

	//KFbxVector4 tVec;
	//KFbxVector4 rVec;
	//kMesh->GetDefaultT(tVec);
	//kMesh->GetDefaultR(rVec);
	//Con::printf("Mesh tVec: %f %f %f",tVec[0],tVec[1],tVec[2]);

	S32 controlPnts = kMesh->GetControlPointsCount();
	S32 deformers = kMesh->GetDeformerCount();
	S32 geomMapCnt = kMesh->GetDestinationGeometryWeightedMapCount();
	S32 polyCnt = kMesh->GetPolygonCount();
	S32 uvLayerCnt = kMesh->GetUVLayerCount();
	//kMesh->GetUVLayer?
	S32 polyVertCnt = kMesh->GetPolygonVertexCount();
	int *polyverts = kMesh->GetPolygonVertices();
	S32 finalTris=0;
	
	//kMesh->GetMaterialIndices();
	//kMesh->GetAllChannelUV(0);
	S32 UVCount = kMesh->GetTextureUVCount();
	S32 layerCount = kMesh->GetLayerCount();
	//KFbxVector2 *uv;
	//kMesh->GetTextureUV(uv);//?

	KFbxVector4 *cntrlPnts = kMesh->GetControlPoints();

	//FIX:  Multiple layers please, not just 0.
	KFbxLayerElementNormal* leNormal = kMesh->GetLayer(0)->GetNormals();
	KFbxLayerElementUV* leUV = kMesh->GetLayer(0)->GetUVs();

	reportModes(leNormal,leUV);

	// Create TSMesh primitive
	appMesh->primitives.increment();
	TSDrawPrimitive& primitive = appMesh->primitives.last();
	primitive.start = appMesh->indices.size();
	//primitive.numElements = polyCnt * 3;
	//primitive.numElements = polyVertCnt;//OOOOPS.  Needs to be actual numTris * 3.
	//primitive.matIndex = TSDrawPrimitive::NoMaterial;//
	primitive.matIndex = (TSDrawPrimitive::Triangles | TSDrawPrimitive::Indexed);//TriStrip??
	//kMesh->GetNormalsIndices();
	
	KFbxNode *kNode = kMesh->GetNode();
	if (kNode)
	{
		KFbxVector4 tVec,ptVec;
		KFbxVector4 rVec,prVec;
		kNode->GetDefaultT(tVec);
		kNode->GetDefaultR(rVec);
		KFbxXMatrix pivot;
		kMesh->GetPivot(pivot);
		prVec = pivot.GetR();
		ptVec = pivot.GetT();
		Con::printf("Found my mesh node: translation %f %f %f rotation %f %f %f %f ptVec %f %f %f, prVec %f %f %f %f",
			tVec[0],tVec[1],tVec[2],rVec[0],rVec[1],rVec[2],rVec[3],
			ptVec[0],ptVec[1],ptVec[2],prVec[0],prVec[1],prVec[2],prVec[3]);

		appMesh->mOffset.set(-tVec[0]*mScaleFactor,tVec[2]*mScaleFactor,tVec[1]*mScaleFactor);


		/*  OMFG:  here are the details, regarding all the possible places we have to look in order to ensure
		that every possible FBX file will work properly:
		*     - Rotation offset (Roff)
		*     - Rotation pivot (Rp)
		*     - Pre-rotation (Rpre)
		*     - Post-rotation (Rpost)
		*     - Scaling offset (Soff)
		*     - Scaling pivot (Sp)
		*     - Geometric translation (Gt)
		*     - Geometric rotation (Gr)
		*     - Geometric scaling (Gs)
		* 
		* These values combine in the matricial form to compute the World transform of the node 
		* using the formula:
		*
		* 	World = ParentWorld * T * Roff * Rp * Rpre * R * Rpost * Rp-1 * Soff * Sp * S * Sp-1 
		*

		See "\Engine\lib\fbxsdk\include\fbxfilesdk\kfbxplugins\kfbxnode.h" for further details.

		The problem in my particular case right now turned out to be Pre Rotation Src, "-90 0 0 1",
		and even if we have only that, it is also going to have to be flipped from right to left handed.
		Would be best to only do that once, maybe I can rotate all these and then flip the result.
		*/

		
		KFbxNode::EPivotSet pivotSource = KFbxNode::eSOURCE_SET;
		KFbxNode::EPivotSet pivotDest = KFbxNode::eDESTINATION_SET;

		KFbxVector4 geomRotSrc = kNode->GetGeometricRotation(pivotSource);
		KFbxVector4 geomRotDest = kNode->GetGeometricRotation(pivotDest);
		KFbxVector4 postRotSrc = kNode->GetPostRotation(pivotSource);
		KFbxVector4 postRotDest = kNode->GetPostRotation(pivotDest);
		KFbxVector4 preRotSrc = kNode->GetPreRotation(pivotSource);
		KFbxVector4 preRotDest = kNode->GetPreRotation(pivotDest);
		KFbxVector4 rotOffsetSrc = kNode->GetRotationOffset(pivotSource);
		KFbxVector4 rotOffsetDest = kNode->GetRotationOffset(pivotDest);
		KFbxVector4 rotPivotSrc = kNode->GetRotationPivot(pivotSource);
		KFbxVector4 rotPivotDest = kNode->GetRotationPivot(pivotDest);

		Con::printf("GeometricRotation, source %f %f %f %f   dest %f %f %f %f",
			geomRotSrc[0],geomRotSrc[1],geomRotSrc[2],geomRotSrc[3],
			geomRotDest[0],geomRotDest[1],geomRotDest[2],geomRotDest[3]);
		Con::printf("PostRotation, source %f %f %f %f   dest %f %f %f %f",
			postRotSrc[0],postRotSrc[1],postRotSrc[2],postRotSrc[3],
			postRotDest[0],postRotDest[1],postRotDest[2],postRotDest[3]);
		Con::printf("PreRotation, source %f %f %f %f   dest %f %f %f %f",
			preRotSrc[0],preRotSrc[1],preRotSrc[2],preRotSrc[3],
			preRotDest[0],preRotDest[1],preRotDest[2],preRotDest[3]);
		Con::printf("rotOffset, source %f %f %f %f   dest %f %f %f %f",
			rotOffsetSrc[0],rotOffsetSrc[1],rotOffsetSrc[2],rotOffsetSrc[3],
			rotOffsetDest[0],rotOffsetDest[1],rotOffsetDest[2],rotOffsetDest[3]);
		Con::printf("rotPivot, source %f %f %f %f   dest %f %f %f %f",
			rotPivotSrc[0],rotPivotSrc[1],rotPivotSrc[2],rotPivotSrc[3],
			rotPivotDest[0],rotPivotDest[1],rotPivotDest[2],rotPivotDest[3]);

		//QuatF qPreRotSrc(mDegToRad(preRotSrc[0]),mDegToRad(-postRotSrc[2]),
		//	mDegToRad(-preRotSrc[1]),mDegToRad(preRotSrc[3]));//MAYBE... back to wishing
		//that MatrixF LeftToRightHanded function worked... need permanent solution here.

		//HERE: have to work in the entire line from above, to deal with every one of these possible
		//transforms, this is just the one that is needed for swat/soldier model.
		
		//HMMM..   Taking this out entirely is the only way to make the bird line up with the skeleton.  Maybe the whole 
		//         thing including the skeleton needs to be rotated by the Mesh preRotMat???  Hmmm...
		//appMesh->mOrientation.set(EulerF(mDegToRad(preRotSrc[0]),mDegToRad(-postRotSrc[2]),mDegToRad(-preRotSrc[1])));
		
		//TEMP: had this before, but crashing, try for identity for now 04/07/16
		//appMesh->mOrientation.set(EulerF(mDegToRad(preRotSrc[0]),mDegToRad(postRotSrc[1]),mDegToRad(preRotSrc[2])));
		appMesh->mOrientation.identity();

		//World = ParentWorld * T * Roff * Rp * Rpre * R * Rpost * Rp-1 * Soff * Sp * S * Sp-1 
	}

	for (U32 i=0;i<controlPnts;i++)
	{			
		//TESTING...
		//int id = leNormal->GetIndexArray().GetAt(i);
		//norm = leNormal->GetDirectArray().GetAt(id);

		//KFbxVector4 GetControlPointAt(int pIndex)
		KFbxVector4	thisPoint = kMesh->GetControlPointAt(i);
		//kMesh->
		//Point3F pos(-cntrlPnts[i][0]*sc,cntrlPnts[i][2]*sc,cntrlPnts[i][1]*sc);
		Point3F pos(-thisPoint[0]*mScaleFactor,thisPoint[2]*mScaleFactor,thisPoint[1]*mScaleFactor);



		//SWAT HACK:  Don't know why the guy's mesh is so out of alignment with his skeleton, but 
		// this manual hack puts it back where it should go.  (???)  If it comes down to it, expose 
		// fields for mesh offset and orientation in the loading GUI, which is the next logical step.
		//Best of course would be to find this data in the fbx file, it has to be there somewhere.
		//appMesh->mOrientation.set(EulerF(mDegToRad(-90.0),0,0));
		//appMesh->mOffset = Point3F(0,0.15,2.25);

		Point3F temp;
		appMesh->mOrientation.mulP(pos,&temp);
		temp += appMesh->mOffset;
		pos = temp;
		//END SWAT HACK
		

		if (appMesh->isSkin())
		{
			appMesh->initialVerts.push_back(pos);
		}
		appMesh->points.push_back(pos);
		//Con::printf("vert %d: %f %f %f",i,pos.x,pos.y,pos.z);

		appMesh->uvs.push_back(Point2F(0,0));//These need to match the vert list in Torque.  Create arrays
		appMesh->normals.push_back(Point3F(0,0,1));//FIX!!              here, then assign values later.

		if (appMesh->isSkin())
			appMesh->initialNorms.push_back(Point3F(0,0,1));//FIX!
		
		//Con::printf("appMesh vert %d: %f %f %f  ",
		//	appMesh->points.size(),pos.x,pos.y,pos.z);
	}

	U32 vertCnt=0;
	S32 uvRefIndirect=0;
	S32 normRefIndirect=0;
	KFbxVector2 uv;
	int id = 0;
	if (leUV)
	{
		if (leUV->GetReferenceMode()==KFbxLayerElement::eINDEX_TO_DIRECT)
			uvRefIndirect=1;//arbitrary, just shorter than above.
	}
	if (leNormal)
	{
		if (leNormal->GetReferenceMode()==KFbxLayerElement::eINDEX_TO_DIRECT)
			normRefIndirect=1;//arbitrary, just shorter than above.
	}
	U32 a,b,c;
	for (U32 i=0;i<polyCnt;i++)
	{			
		int lStartIndex = kMesh->GetPolygonVertexIndex(i);
		if (lStartIndex == -1) { 
			Con::printf("Couldn't find polygon %d",i); 
			return; 
		}
		int* lVertices = &(kMesh->GetPolygonVertices()[lStartIndex]);
		S32 polySize = kMesh->GetPolygonSize(i);
		//OMFG, this desperately needs a rewrite, got ugly as hell by the time we worked in the reality of
		//multiple UVs per vertex, which is not legal in DTS.  Have to create extra vertices to hold down 
		//the different UVs, and then have to store them with references to the original vertices, so that 
		//we know how to weight them when it comes to the clusters.
		if (polySize==3)
		{ 
			//appMesh->indices.push_back((U32)polyverts[vertCnt-0]);//flip the winding!
			//appMesh->indices.push_back((U32)polyverts[vertCnt-1]);
			//appMesh->indices.push_back((U32)polyverts[vertCnt-2]);
			//Con::printf("polySize %d, %d: %d %d %d",polySize,i,lVertices[0],lVertices[1],lVertices[2]);
			//Con::printf("tri %d: %d %d %d",finalTris,lVertices[0],lVertices[2],lVertices[1]);
			a = (U32)lVertices[0];
			b = (U32)lVertices[2];
			c = (U32)lVertices[1];
			if (uvRefIndirect)
			{
				//id = leUV->GetIndexArray().GetAt((U32)lVertices[0]);
				id = leUV->GetIndexArray().GetAt(vertCnt + 0);
				uv = leUV->GetDirectArray().GetAt(id);
				if ((appMesh->uvs[a].len()>0.0)&&(!(appMesh->uvs[a].x==(F32)uv[0] && appMesh->uvs[a].y==(F32)(1.0-uv[1]))))
				{//HERE: we already have a uv for this vert, and it isn't this value, so make a new vert and assign it to indices.
					Point3F kPos = appMesh->points[a];
					appMesh->points.push_back(kPos);
					if (appMesh->isSkin())
						appMesh->initialVerts.push_back(kPos);
					//Con::printf("a: %d  -  added a vert: %f %f %f  old uv %f %f  new uv %f %f",
					//	a,kPos.x,kPos.y,kPos.z,appMesh->uvs[a].x,appMesh->uvs[a].y,uv[0],1.0-uv[1]);
					appMesh->vertRefs.increment();
					appMesh->vertRefs.last().base = a;
					a = appMesh->points.size()-1;
					appMesh->vertRefs.last().index = a;
					appMesh->uvs.push_back(Point2F(uv[0],1.0-uv[1]));
					appMesh->normals.push_back(Point3F(0,0,1));//FIX!!!		
					if (appMesh->isSkin())
						appMesh->initialNorms.push_back(Point3F(0,0,1));//FIX!!!
				} else {
					appMesh->uvs[a].set(uv[0],1.0-uv[1]);
				}

				id = leUV->GetIndexArray().GetAt(vertCnt + 2);
				uv = leUV->GetDirectArray().GetAt(id);
				if ((appMesh->uvs[b].len()>0.0)&&(!(appMesh->uvs[b].x==(F32)uv[0] && appMesh->uvs[b].y==(F32)(1.0-uv[1]))))
				{
					Point3F kPos = appMesh->points[b];
					appMesh->points.push_back(kPos);
					if (appMesh->isSkin())
						appMesh->initialVerts.push_back(kPos);
					//Con::printf("b: %d  -  added a vert: %f %f %f  old uv %f %f  new uv %f %f",
					//	b,kPos.x,kPos.y,kPos.z,appMesh->uvs[b].x,appMesh->uvs[b].y,uv[0],1.0-uv[1]);
					appMesh->vertRefs.increment();
					appMesh->vertRefs.last().base = b;
					b = appMesh->points.size()-1;
					appMesh->vertRefs.last().index = b;
					appMesh->uvs.push_back(Point2F(uv[0],1.0-uv[1]));
					appMesh->normals.push_back(Point3F(0,0,1));
					if (appMesh->isSkin())
						appMesh->initialNorms.push_back(Point3F(0,0,1));
				} else {
					appMesh->uvs[b].set(uv[0],1.0-uv[1]);
				}

				id = leUV->GetIndexArray().GetAt(vertCnt + 1);
				uv = leUV->GetDirectArray().GetAt(id);
				if ((appMesh->uvs[c].len()>0.0)&&(!(appMesh->uvs[c].x==(F32)uv[0] && appMesh->uvs[c].y==(F32)(1.0-uv[1]))))
				{
					Point3F kPos = appMesh->points[c];
					appMesh->points.push_back(kPos);
					if (appMesh->isSkin())
						appMesh->initialVerts.push_back(kPos);
					//Con::printf("c: %d  -  added a vert: %f %f %f  old uv %f %f  new uv %f %f",
					//	c,kPos.x,kPos.y,kPos.z,appMesh->uvs[c].x,appMesh->uvs[c].y,uv[0],1.0-uv[1]);
					appMesh->vertRefs.increment();
					appMesh->vertRefs.last().base = c;
					c = appMesh->points.size()-1;
					appMesh->vertRefs.last().index = c;
					appMesh->uvs.push_back(Point2F(uv[0],1.0-uv[1]));
					appMesh->normals.push_back(Point3F(0,0,1));
					if (appMesh->isSkin())
						appMesh->initialNorms.push_back(Point3F(0,0,1));
				} else {
					appMesh->uvs[c].set(uv[0],1.0-uv[1]);
				}
			} else {
				//uv = leUV->GetDirectArray().GetAt((U32)lVertices[0]);
				uv = leUV->GetDirectArray().GetAt(a);
				appMesh->uvs[a].set(uv[0],1.0-uv[1]);
				uv = leUV->GetDirectArray().GetAt(b);
				appMesh->uvs[b].set(uv[0],1.0-uv[1]);
				uv = leUV->GetDirectArray().GetAt(c);
				appMesh->uvs[c].set(uv[0],1.0-uv[1]);
			}
			appMesh->indices.push_back(a);
			appMesh->indices.push_back(b);
			appMesh->indices.push_back(c);
			finalTris++;
			//Con::printf("tri %d: %d %d %d",finalTris,a,b,c);
			//Con::printf("Poly %d size %d startIndex %d indices %d %d %d",
			//	i,polySize,lStartIndex,lVertices[0],lVertices[1],lVertices[2]);				
				//i,polySize,lStartIndex,polyverts[vertCnt-0],polyverts[vertCnt-1],polyverts[vertCnt-2]);
		} else if (polySize==4) {
			//HERE: have to make more triangles for dts to fit into TRI-angle mesh.			
			//Con::printf("polySize %d, %d: %d %d %d %d",polySize,i,
			//	lVertices[0],lVertices[1],lVertices[2],lVertices[3]);
			a = (U32)lVertices[0];
			b = (U32)lVertices[2];
			c = (U32)lVertices[1];
			//Con::printf("tri %d: %d %d %d",finalTris,lVertices[0],lVertices[2],lVertices[1]);
			if (uvRefIndirect)
			{
				id = leUV->GetIndexArray().GetAt(vertCnt + 0);
				uv = leUV->GetDirectArray().GetAt(id);
				if ((appMesh->uvs[a].len()>0.0)&&(!(appMesh->uvs[a].x==(F32)uv[0] && appMesh->uvs[a].y==(F32)(1.0-uv[1]))))
				{//HERE: we already have a uv for this vert, and it isn't this value, so make a new vert and assign it to indices.
					Point3F kPos = appMesh->points[a];
					appMesh->points.push_back(kPos);
					if (appMesh->isSkin())
						appMesh->initialVerts.push_back(kPos);
					//Con::printf("a: %d  -  added a vert: %f %f %f  old uv %f %f  new uv %f %f",
					//	a,kPos.x,kPos.y,kPos.z,appMesh->uvs[a].x,appMesh->uvs[a].y,uv[0],1.0-uv[1]);
					appMesh->vertRefs.increment();
					appMesh->vertRefs.last().base = a;
					a = appMesh->points.size()-1;
					appMesh->vertRefs.last().index = a;
					appMesh->uvs.push_back(Point2F(uv[0],1.0-uv[1]));
					appMesh->normals.push_back(Point3F(0,0,1));
					if (appMesh->isSkin())
						appMesh->initialNorms.push_back(Point3F(0,0,1));
				} else {
					appMesh->uvs[a].set(uv[0],1.0-uv[1]);
				}

				id = leUV->GetIndexArray().GetAt(vertCnt + 2);
				uv = leUV->GetDirectArray().GetAt(id);
				if ((appMesh->uvs[b].len()>0.0)&&(!(appMesh->uvs[b].x==(F32)uv[0] && appMesh->uvs[b].y==(F32)(1.0-uv[1]))))
				{
					Point3F kPos = appMesh->points[b];
					appMesh->points.push_back(kPos);
					if (appMesh->isSkin())
						appMesh->initialVerts.push_back(kPos);
					//Con::printf("b: %d  -  added a vert: %f %f %f  old uv %f %f  new uv %f %f",
					//	b,kPos.x,kPos.y,kPos.z,appMesh->uvs[b].x,appMesh->uvs[b].y,uv[0],1.0-uv[1]);
					appMesh->vertRefs.increment();
					appMesh->vertRefs.last().base = b;
					b = appMesh->points.size()-1;
					appMesh->vertRefs.last().index = b;
					appMesh->uvs.push_back(Point2F(uv[0],1.0-uv[1]));
					appMesh->normals.push_back(Point3F(0,0,1));
					if (appMesh->isSkin())
						appMesh->initialNorms.push_back(Point3F(0,0,1));
				} else {
					appMesh->uvs[b].set(uv[0],1.0-uv[1]);
				}

				id = leUV->GetIndexArray().GetAt(vertCnt + 1);
				uv = leUV->GetDirectArray().GetAt(id);
				if ((appMesh->uvs[c].len()>0.0)&&(!(appMesh->uvs[c].x==(F32)uv[0] && appMesh->uvs[c].y==(F32)(1.0-uv[1]))))
				{
					Point3F kPos = appMesh->points[c];
					appMesh->points.push_back(kPos);
					if (appMesh->isSkin())
						appMesh->initialVerts.push_back(kPos);
					//Con::printf("c: %d  -  added a vert: %f %f %f  old uv %f %f  new uv %f %f",
					//	c,kPos.x,kPos.y,kPos.z,appMesh->uvs[c].x,appMesh->uvs[c].y,uv[0],1.0-uv[1]);
					appMesh->vertRefs.increment();
					appMesh->vertRefs.last().base = c;
					c = appMesh->points.size()-1;
					appMesh->vertRefs.last().index = c;
					appMesh->uvs.push_back(Point2F(uv[0],1.0-uv[1]));
					appMesh->normals.push_back(Point3F(0,0,1));
					if (appMesh->isSkin())
						appMesh->initialNorms.push_back(Point3F(0,0,1));
				} else {
					appMesh->uvs[c].set(uv[0],1.0-uv[1]);
				}
			} else {
				uv = leUV->GetDirectArray().GetAt((U32)lVertices[0]);
				appMesh->uvs[(U32)lVertices[0]].set(uv[0],1.0-uv[1]);
				uv = leUV->GetDirectArray().GetAt((U32)lVertices[2]);
				appMesh->uvs[(U32)lVertices[2]].set(uv[0],1.0-uv[1]);
				uv = leUV->GetDirectArray().GetAt((U32)lVertices[1]);
				appMesh->uvs[(U32)lVertices[1]].set(uv[0],1.0-uv[1]);
			}
			appMesh->indices.push_back(a);
			appMesh->indices.push_back(b);
			appMesh->indices.push_back(c);
			finalTris++;
			//Con::printf("tri %d: %d %d %d",finalTris,a,b,c);

			a = (U32)lVertices[0];
			b = (U32)lVertices[3];
			c = (U32)lVertices[2];
			//Con::printf("tri %d: %d %d %d",finalTris,lVertices[0],lVertices[3],lVertices[2]);
			if (uvRefIndirect)
			{
				id = leUV->GetIndexArray().GetAt(vertCnt + 0);
				uv = leUV->GetDirectArray().GetAt(id);
				if ((appMesh->uvs[a].len()>0.0)&&(!(appMesh->uvs[a].x==(F32)uv[0] && appMesh->uvs[a].y==(F32)(1.0-uv[1]))))
				{//HERE: we already have a uv for this vert, and it isn't this value, so make a new vert and assign it to indices.
					Point3F kPos = appMesh->points[a];
					appMesh->points.push_back(kPos);
					if (appMesh->isSkin())
						appMesh->initialVerts.push_back(kPos);
					//Con::printf("a: %d  -  added a vert: %f %f %f  old uv %f %f  new uv %f %f",
					//	a,kPos.x,kPos.y,kPos.z,appMesh->uvs[a].x,appMesh->uvs[a].y,uv[0],1.0-uv[1]);
					appMesh->vertRefs.increment();
					appMesh->vertRefs.last().base = a;
					a = appMesh->points.size()-1;
					appMesh->vertRefs.last().index = a;
					appMesh->uvs.push_back(Point2F(uv[0],1.0-uv[1]));
					appMesh->normals.push_back(Point3F(0,0,1));
					if (appMesh->isSkin())
						appMesh->initialNorms.push_back(Point3F(0,0,1));
				} else {
					appMesh->uvs[a].set(uv[0],1.0-uv[1]);
				}

				id = leUV->GetIndexArray().GetAt(vertCnt + 3);
				uv = leUV->GetDirectArray().GetAt(id);
				if ((appMesh->uvs[b].len()>0.0)&&(!(appMesh->uvs[b].x==(F32)uv[0] && appMesh->uvs[b].y==(F32)(1.0-uv[1]))))
				{
					Point3F kPos = appMesh->points[b];
					appMesh->points.push_back(kPos);
					if (appMesh->isSkin())
						appMesh->initialVerts.push_back(kPos);
					//Con::printf("b: %d  -  added a vert: %f %f %f  old uv %f %f  new uv %f %f",
					//	b,kPos.x,kPos.y,kPos.z,appMesh->uvs[b].x,appMesh->uvs[b].y,uv[0],1.0-uv[1]);
					appMesh->vertRefs.increment();
					appMesh->vertRefs.last().base = b;
					b = appMesh->points.size()-1;
					appMesh->vertRefs.last().index = b;
					appMesh->uvs.push_back(Point2F(uv[0],1.0-uv[1]));
					appMesh->normals.push_back(Point3F(0,0,1));
					if (appMesh->isSkin())
						appMesh->initialNorms.push_back(Point3F(0,0,1));
				} else {
					appMesh->uvs[b].set(uv[0],1.0-uv[1]);
				}

				id = leUV->GetIndexArray().GetAt(vertCnt + 2);
				uv = leUV->GetDirectArray().GetAt(id);
				if ((appMesh->uvs[c].len()>0.0)&&(!(appMesh->uvs[c].x==(F32)uv[0] && appMesh->uvs[c].y==(F32)(1.0-uv[1]))))
				{
					Point3F kPos = appMesh->points[c];
					appMesh->points.push_back(kPos);
					if (appMesh->isSkin())
						appMesh->initialVerts.push_back(kPos);
					//Con::printf("c: %d  -  added a vert: %f %f %f  old uv %f %f  new uv %f %f",
					//	c,kPos.x,kPos.y,kPos.z,appMesh->uvs[c].x,appMesh->uvs[c].y,uv[0],1.0-uv[1]);
					appMesh->vertRefs.increment();
					appMesh->vertRefs.last().base = c;
					c = appMesh->points.size()-1;
					appMesh->vertRefs.last().index = c;
					appMesh->uvs.push_back(Point2F(uv[0],1.0-uv[1]));
					appMesh->normals.push_back(Point3F(0,0,1));
					if (appMesh->isSkin())
						appMesh->initialNorms.push_back(Point3F(0,0,1));
				} else {
					appMesh->uvs[c].set(uv[0],1.0-uv[1]);
				}
			} else {
				uv = leUV->GetDirectArray().GetAt(a);
				appMesh->uvs[a].set(uv[0],1.0-uv[1]);
				uv = leUV->GetDirectArray().GetAt(b);
				appMesh->uvs[b].set(uv[0],1.0-uv[1]);
				uv = leUV->GetDirectArray().GetAt(c);
				appMesh->uvs[c].set(uv[0],1.0-uv[1]);
			}
			appMesh->indices.push_back(a);
			appMesh->indices.push_back(b);
			appMesh->indices.push_back(c);
			finalTris++;
			//Con::printf("tri %d: %d %d %d",finalTris,a,b,c);

			//Con::printf("Poly %d size %d startIndex %d indices %d %d %d %d",
			//	i,polySize,lStartIndex,lVertices[0],lVertices[1],lVertices[2],lVertices[3]);
				
				//polyverts[vertCnt-0],polyverts[vertCnt-1],
				//polyverts[vertCnt-2],polyverts[vertCnt-3]);
		} else if (polySize==5) {			
			//Con::printf("polySize %d, %d: %d %d %d %d %d",polySize,i,
			//	lVertices[0],lVertices[1],lVertices[2],lVertices[3],lVertices[4]);

			a = (U32)lVertices[0];
			b = (U32)lVertices[2];
			c = (U32)lVertices[1];
			//Con::printf("tri %d: %d %d %d",finalTris,lVertices[0],lVertices[2],lVertices[1]);
			if (uvRefIndirect)
			{
				id = leUV->GetIndexArray().GetAt(vertCnt + 0);
				uv = leUV->GetDirectArray().GetAt(id);
				if ((appMesh->uvs[a].len()>0.0)&&(!(appMesh->uvs[a].x==(F32)uv[0] && appMesh->uvs[a].y==(F32)(1.0-uv[1]))))
				{//HERE: we already have a uv for this vert, and it isn't this value, so make a new vert and assign it to indices.
					Point3F kPos = appMesh->points[a];
					appMesh->points.push_back(kPos);
					if (appMesh->isSkin())
						appMesh->initialVerts.push_back(kPos);
					//Con::printf("a: %d  -  added a vert: %f %f %f  old uv %f %f  new uv %f %f",
					//	a,kPos.x,kPos.y,kPos.z,appMesh->uvs[a].x,appMesh->uvs[a].y,uv[0],1.0-uv[1]);
					appMesh->vertRefs.increment();
					appMesh->vertRefs.last().base = a;
					a = appMesh->points.size()-1;
					appMesh->vertRefs.last().index = a;
					appMesh->uvs.push_back(Point2F(uv[0],1.0-uv[1]));
					appMesh->normals.push_back(Point3F(0,0,1));		
					if (appMesh->isSkin())
						appMesh->initialNorms.push_back(Point3F(0,0,1));
				} else {
					appMesh->uvs[a].set(uv[0],1.0-uv[1]);
				}

				id = leUV->GetIndexArray().GetAt(vertCnt + 2);
				uv = leUV->GetDirectArray().GetAt(id);
				if ((appMesh->uvs[b].len()>0.0)&&(!(appMesh->uvs[b].x==(F32)uv[0] && appMesh->uvs[b].y==(F32)(1.0-uv[1]))))
				{
					Point3F kPos = appMesh->points[b];
					appMesh->points.push_back(kPos);
					if (appMesh->isSkin())
						appMesh->initialVerts.push_back(kPos);
					//Con::printf("b: %d  -  added a vert: %f %f %f  old uv %f %f  new uv %f %f",
					//	b,kPos.x,kPos.y,kPos.z,appMesh->uvs[b].x,appMesh->uvs[b].y,uv[0],1.0-uv[1]);
					appMesh->vertRefs.increment();
					appMesh->vertRefs.last().base = b;
					b = appMesh->points.size()-1;
					appMesh->vertRefs.last().index = b;
					appMesh->uvs.push_back(Point2F(uv[0],1.0-uv[1]));
					appMesh->normals.push_back(Point3F(0,0,1));
					if (appMesh->isSkin())
						appMesh->initialNorms.push_back(Point3F(0,0,1));
				} else {
					appMesh->uvs[b].set(uv[0],1.0-uv[1]);
				}

				id = leUV->GetIndexArray().GetAt(vertCnt + 1);
				uv = leUV->GetDirectArray().GetAt(id);
				if ((appMesh->uvs[c].len()>0.0)&&(!(appMesh->uvs[c].x==(F32)uv[0] && appMesh->uvs[c].y==(F32)(1.0-uv[1]))))
				{
					Point3F kPos = appMesh->points[c];
					appMesh->points.push_back(kPos);
					if (appMesh->isSkin())
						appMesh->initialVerts.push_back(kPos);
					//Con::printf("c: %d  -  added a vert: %f %f %f  old uv %f %f  new uv %f %f",
					//	c,kPos.x,kPos.y,kPos.z,appMesh->uvs[c].x,appMesh->uvs[c].y,uv[0],1.0-uv[1]);
					appMesh->vertRefs.increment();
					appMesh->vertRefs.last().base = c;
					c = appMesh->points.size()-1;
					appMesh->vertRefs.last().index = c;
					appMesh->uvs.push_back(Point2F(uv[0],1.0-uv[1]));
					appMesh->normals.push_back(Point3F(0,0,1));
					if (appMesh->isSkin())
						appMesh->initialNorms.push_back(Point3F(0,0,1));
				} else {
					appMesh->uvs[c].set(uv[0],1.0-uv[1]);
				}

			} else {
				uv = leUV->GetDirectArray().GetAt((U32)lVertices[0]);
				appMesh->uvs[a].set(uv[0],1.0-uv[1]);
				uv = leUV->GetDirectArray().GetAt((U32)lVertices[2]);
				appMesh->uvs[b].set(uv[0],1.0-uv[1]);
				uv = leUV->GetDirectArray().GetAt((U32)lVertices[1]);
				appMesh->uvs[c].set(uv[0],1.0-uv[1]);
			}
			appMesh->indices.push_back(a);
			appMesh->indices.push_back(b);
			appMesh->indices.push_back(c);
			finalTris++;
			//Con::printf("tri %d: %d %d %d",finalTris,a,b,c);

			a = (U32)lVertices[0];
			b = (U32)lVertices[3];
			c = (U32)lVertices[2];
			//Con::printf("tri %d: %d %d %d",finalTris,lVertices[0],lVertices[3],lVertices[2]);
			if (uvRefIndirect)
			{
				id = leUV->GetIndexArray().GetAt(vertCnt + 0);
				uv = leUV->GetDirectArray().GetAt(id);
				if ((appMesh->uvs[a].len()>0.0)&&(!(appMesh->uvs[a].x==(F32)uv[0] && appMesh->uvs[a].y==(F32)(1.0-uv[1]))))
				{//HERE: we already have a uv for this vert, and it isn't this value, so make a new vert and assign it to indices.
					Point3F kPos = appMesh->points[a];
					appMesh->points.push_back(kPos);
					if (appMesh->isSkin())
						appMesh->initialVerts.push_back(kPos);
					//Con::printf("a: %d  -  added a vert: %f %f %f  old uv %f %f  new uv %f %f",
					//	a,kPos.x,kPos.y,kPos.z,appMesh->uvs[a].x,appMesh->uvs[a].y,uv[0],1.0-uv[1]);
					appMesh->vertRefs.increment();
					appMesh->vertRefs.last().base = a;
					a = appMesh->points.size()-1;
					appMesh->vertRefs.last().index = a;
					appMesh->uvs.push_back(Point2F(uv[0],1.0-uv[1]));
					appMesh->normals.push_back(Point3F(0,0,1));
					if (appMesh->isSkin())
						appMesh->initialNorms.push_back(Point3F(0,0,1));
				} else {
					appMesh->uvs[a].set(uv[0],1.0-uv[1]);
				}

				id = leUV->GetIndexArray().GetAt(vertCnt + 3);
				uv = leUV->GetDirectArray().GetAt(id);
				if ((appMesh->uvs[b].len()>0.0)&&(!(appMesh->uvs[b].x==(F32)uv[0] && appMesh->uvs[b].y==(F32)(1.0-uv[1]))))
				{
					Point3F kPos = appMesh->points[b];
					appMesh->points.push_back(kPos);
					if (appMesh->isSkin())
						appMesh->initialVerts.push_back(kPos);
					//Con::printf("b: %d  -  added a vert: %f %f %f  old uv %f %f  new uv %f %f",
					//	b,kPos.x,kPos.y,kPos.z,appMesh->uvs[b].x,appMesh->uvs[b].y,uv[0],1.0-uv[1]);
					appMesh->vertRefs.increment();
					appMesh->vertRefs.last().base = b;
					b = appMesh->points.size()-1;
					appMesh->vertRefs.last().index = b;
					appMesh->uvs.push_back(Point2F(uv[0],1.0-uv[1]));
					appMesh->normals.push_back(Point3F(0,0,1));
					if (appMesh->isSkin())
						appMesh->initialNorms.push_back(Point3F(0,0,1));
				} else {
					appMesh->uvs[b].set(uv[0],1.0-uv[1]);
				}

				id = leUV->GetIndexArray().GetAt(vertCnt + 2);
				uv = leUV->GetDirectArray().GetAt(id);
				if ((appMesh->uvs[c].len()>0.0)&&(!(appMesh->uvs[c].x==(F32)uv[0] && appMesh->uvs[c].y==(F32)(1.0-uv[1]))))
				{
					Point3F kPos = appMesh->points[c];
					appMesh->points.push_back(kPos);
					if (appMesh->isSkin())
						appMesh->initialVerts.push_back(kPos);
					//Con::printf("c: %d  -  added a vert: %f %f %f  old uv %f %f  new uv %f %f",
					//	c,kPos.x,kPos.y,kPos.z,appMesh->uvs[c].x,appMesh->uvs[c].y,uv[0],1.0-uv[1]);
					appMesh->vertRefs.increment();
					appMesh->vertRefs.last().base = c;
					c = appMesh->points.size()-1;
					appMesh->vertRefs.last().index = c;
					appMesh->uvs.push_back(Point2F(uv[0],1.0-uv[1]));
					appMesh->normals.push_back(Point3F(0,0,1));
					if (appMesh->isSkin())
						appMesh->initialNorms.push_back(Point3F(0,0,1));
				} else {
					appMesh->uvs[c].set(uv[0],1.0-uv[1]);
				}

			} else {
				uv = leUV->GetDirectArray().GetAt((U32)lVertices[0]);
				appMesh->uvs[a].set(uv[0],1.0-uv[1]);
				uv = leUV->GetDirectArray().GetAt((U32)lVertices[3]);
				appMesh->uvs[b].set(uv[0],1.0-uv[1]);
				uv = leUV->GetDirectArray().GetAt((U32)lVertices[2]);
				appMesh->uvs[c].set(uv[0],1.0-uv[1]);
			}
			appMesh->indices.push_back(a);
			appMesh->indices.push_back(b);
			appMesh->indices.push_back(c);
			finalTris++;
			//Con::printf("tri %d: %d %d %d",finalTris,a,b,c);

			a = (U32)lVertices[0];
			b = (U32)lVertices[4];
			c = (U32)lVertices[3];
			//Con::printf("tri %d: %d %d %d",finalTris,lVertices[0],lVertices[4],lVertices[3]);
			if (uvRefIndirect)
			{
				id = leUV->GetIndexArray().GetAt(vertCnt + 0);
				uv = leUV->GetDirectArray().GetAt(id);
				if ((appMesh->uvs[a].len()>0.0)&&(!(appMesh->uvs[a].x==(F32)uv[0] && appMesh->uvs[a].y==(F32)(1.0-uv[1]))))
				{//HERE: we already have a uv for this vert, and it isn't this value, so make a new vert and assign it to indices.
					Point3F kPos = appMesh->points[a];
					appMesh->points.push_back(kPos);
					if (appMesh->isSkin())
						appMesh->initialVerts.push_back(kPos);
					//Con::printf("a: %d  -  added a vert: %f %f %f  old uv %f %f  new uv %f %f",
					//	a,kPos.x,kPos.y,kPos.z,appMesh->uvs[a].x,appMesh->uvs[a].y,uv[0],1.0-uv[1]);
					appMesh->vertRefs.increment();
					appMesh->vertRefs.last().base = a;
					a = appMesh->points.size()-1;
					appMesh->vertRefs.last().index = a;
					appMesh->uvs.push_back(Point2F(uv[0],1.0-uv[1]));
					appMesh->normals.push_back(Point3F(0,0,1));
					if (appMesh->isSkin())
						appMesh->initialNorms.push_back(Point3F(0,0,1));
				} else {
					appMesh->uvs[a].set(uv[0],1.0-uv[1]);
				}

				id = leUV->GetIndexArray().GetAt(vertCnt + 4);
				uv = leUV->GetDirectArray().GetAt(id);
				if ((appMesh->uvs[b].len()>0.0)&&(!(appMesh->uvs[b].x==(F32)uv[0] && appMesh->uvs[b].y==(F32)(1.0-uv[1]))))
				{
					Point3F kPos = appMesh->points[b];
					appMesh->points.push_back(kPos);
					if (appMesh->isSkin())
						appMesh->initialVerts.push_back(kPos);
					//Con::printf("b: %d  -  added a vert: %f %f %f  old uv %f %f  new uv %f %f",
					//	b,kPos.x,kPos.y,kPos.z,appMesh->uvs[b].x,appMesh->uvs[b].y,uv[0],1.0-uv[1]);
					appMesh->vertRefs.increment();
					appMesh->vertRefs.last().base = b;
					b = appMesh->points.size()-1;
					appMesh->vertRefs.last().index = b;
					appMesh->uvs.push_back(Point2F(uv[0],1.0-uv[1]));
					appMesh->normals.push_back(Point3F(0,0,1));
					if (appMesh->isSkin())
						appMesh->initialNorms.push_back(Point3F(0,0,1));
				} else {
					appMesh->uvs[b].set(uv[0],1.0-uv[1]);
				}

				id = leUV->GetIndexArray().GetAt(vertCnt + 3);
				uv = leUV->GetDirectArray().GetAt(id);
				if ((appMesh->uvs[c].len()>0.0)&&(!(appMesh->uvs[c].x==(F32)uv[0] && appMesh->uvs[c].y==(F32)(1.0-uv[1]))))
				{
					Point3F kPos = appMesh->points[c];
					appMesh->points.push_back(kPos);
					if (appMesh->isSkin())
						appMesh->initialVerts.push_back(kPos);
					//Con::printf("c: %d  -  added a vert: %f %f %f  old uv %f %f  new uv %f %f",
					//	c,kPos.x,kPos.y,kPos.z,appMesh->uvs[c].x,appMesh->uvs[c].y,uv[0],1.0-uv[1]);
					appMesh->vertRefs.increment();
					appMesh->vertRefs.last().base = c;
					c = appMesh->points.size()-1;
					appMesh->vertRefs.last().index = c;
					appMesh->uvs.push_back(Point2F(uv[0],1.0-uv[1]));
					appMesh->normals.push_back(Point3F(0,0,1));
					if (appMesh->isSkin())
						appMesh->initialNorms.push_back(Point3F(0,0,1));
				} else {
					appMesh->uvs[c].set(uv[0],1.0-uv[1]);
				}

			} else {
				uv = leUV->GetDirectArray().GetAt((U32)lVertices[0]);
				appMesh->uvs[a].set(uv[0],1.0-uv[1]);
				uv = leUV->GetDirectArray().GetAt((U32)lVertices[4]);
				appMesh->uvs[b].set(uv[0],1.0-uv[1]);
				uv = leUV->GetDirectArray().GetAt((U32)lVertices[3]);
				appMesh->uvs[c].set(uv[0],1.0-uv[1]);
			}
			appMesh->indices.push_back(a);
			appMesh->indices.push_back(b);
			appMesh->indices.push_back(c);
			finalTris++;
			//Con::printf("tri %d: %d %d %d",finalTris,a,b,c);

			//Con::printf("Poly %d size %d startIndex %d indices %d %d %d %d %d",
			//	i,polySize,lStartIndex,polyverts[vertCnt-0],polyverts[vertCnt-1],
			//	polyverts[vertCnt-2],polyverts[vertCnt-3],polyverts[vertCnt-4]);
		}
		vertCnt += polySize;
	}
	primitive.numElements = finalTris * 3;
	
	//for (U32 i=0;i<primitive.numElements;i++)
	for (U32 i=0;i<polyVertCnt;i++)
	{			
		S32 vert = polyverts[i];
		//S32 vert = appMesh->indices[i];
		KFbxVector2 uv;
		
		if (leUV)
		{
			if (leUV->GetReferenceMode()==KFbxLayerElement::eINDEX_TO_DIRECT)
			{
				int id = leUV->GetIndexArray().GetAt(i);//indices[i]?
				uv = leUV->GetDirectArray().GetAt(id);
				//if ((uv[0]<0.0)||(uv[0]>1.0)||(uv[1]<0.0)||(uv[1]>1.0))
				//	Con::printf("WHOOP WHOOP!!  indirect tvert %d has values out of range:   %d  %f  %f vert %d",i,id,uv[0],uv[1],vert);
			} else {
				uv = leUV->GetDirectArray().GetAt(vert);
				//Con::printf("  direct tvert %d:       %f  %f vert %d",i,uv[0],uv[1],vert);
			}
			//Con::printf("  tvert %d:       %f  %f ",i,uv[0],1.0-uv[1]);
			//Point2F pUV(uv[0],1.0-uv[1]);//(-uv[0],uv[1])?  Handedness flipping?  probably not
			//if (appMesh->uvs[vert].len()==0.0)
			//	appMesh->uvs[vert].set(uv[0],1.0-uv[1]);//This will just take the last one used, if they're different.
			//appMesh->uvs.push_back(pUV);
		}

		KFbxVector4 norm;
		if (leNormal)
		{//HERE: figure these out above, with uvs.
			if (leNormal->GetReferenceMode()==KFbxLayerElement::eINDEX_TO_DIRECT)
			{
				int id = leNormal->GetIndexArray().GetAt(i);
				norm = leNormal->GetDirectArray().GetAt(id);
			} else {
				norm = leNormal->GetDirectArray().GetAt(vert);
			}
			Point3F normal(-norm[0],norm[2],norm[1]);
			appMesh->normals[vert] = normal;
			if (appMesh->isSkin())
				appMesh->initialNorms[vert] = normal;
		}
	}
	//for (U32 i=0;i<controlPnts;i++)
	//{
	//	Con::printf("uv[%d]:  %f  %f",i,appMesh->uvs[i].x,appMesh->uvs[i].y);
	//}
			//appMesh->indices.push_back((U32)polyverts[(i*3)+2]);//flip the winding!
			//appMesh->indices.push_back((U32)polyverts[(i*3)+1]);
			//appMesh->indices.push_back((U32)polyverts[(i*3)+0]);
	

	//for (U32 i=0;i<polyVertCnt;i++)
	//{			
	//	appMesh->indices.push_back((U32)polyverts[i]);
	//}
	//for (U32 i=0;i<appMesh->indices.size()/3;i++)
		//Con::printf("polygon %d:  %d %d %d",i,appMesh->indices[i*3],appMesh->indices[i*3+1],appMesh->indices[i*3+2]);
	Con::printf("Built Mesh! polyCnt %d  points %d  deformers %d uvLayers %d  indices %d  normals %d  uvs %d finalTris %d",
		polyCnt,appMesh->points.size(),deformers,uvLayerCnt,appMesh->indices.size(),appMesh->normals.size(),appMesh->uvs.size(),finalTris);
}
	


	//for (U32 i = 0; i < lPolygonCount; i++)
	//{
		//int lPolygonSize = kMesh->GetPolygonSize(i);
		//Con::printf( "  %d  polygon size:  %d",i,lPolygonSize);
	
		//for (U32 j = 0; j < lPolygonSize ; j++)
		//{
		//	int lControlPointIndex = kMesh->GetPolygonVertex(i, j);
		//	appMesh->indices.push_back(lControlPointIndex);
		//	//Con::printf("   control point index: %d",lControlPointIndex);
		//}
	//}
	
	//for (U32 i=0;i<polyCnt;i++)
	//{		
	//	int lStartIndex = kMesh->GetPolygonVertexIndex(i);
	//	if (lStartIndex == -1) return;
	//	int* lVertices = &(kMesh->GetPolygonVertices()[lStartIndex]);
	//	int lCount = kMesh->GetPolygonSize(i);
	//	for (int i = 0; i < lCount; i++)
	//	{
	//       int vertexID = lVertices[i];
	//		 appMesh->indices.push_back(vertexID);	       
	//    }
	//}

	//appMesh->indices.setSize(appMesh->indices.size() + primitive.numElements);
	//
	 // | addMaterial(meshPrims[iPrim]->getMaterial()


	/*
   MeshStreams streams;
   VertTupleMap tupleMap;

   // Create Torque primitives
   for (int iPrim = 0; iPrim < meshPrims.size(); iPrim++) {

      // Primitive element must have at least 1 triangle
      const domListOfUInts* pTriData = meshPrims[iPrim]->getTriangleData();
      if (!pTriData)
         continue;

      U32 numTriangles = pTriData->getCount() / meshPrims[iPrim]->getStride() / 3;
      if (!numTriangles)
         continue;

      // Create TSMesh primitive
      primitives.increment();
      TSDrawPrimitive& primitive = primitives.last();
      primitive.start = indices.size();
      primitive.matIndex = (TSDrawPrimitive::Triangles | TSDrawPrimitive::Indexed) |
                           addMaterial(meshPrims[iPrim]->getMaterial());

      // Get the AppMaterial associated with this primitive
      FbxAppMaterial* appMat = 0;
      if (!(primitive.matIndex & TSDrawPrimitive::NoMaterial))
         appMat = static_cast<FbxAppMaterial*>(appMaterials[primitive.matIndex & TSDrawPrimitive::MaterialMask]);

      // Force the material to be double-sided if this geometry is double-sided.
      if (geomExt->double_sided && appMat && appMat->effectExt)
         appMat->effectExt->double_sided = true;

      // Pre-allocate triangle indices
      primitive.numElements = numTriangles * 3;
      indices.setSize(indices.size() + primitive.numElements);
      U32* dstIndex = indices.end() - primitive.numElements;

      // Determine the offset for each element type in the stream, and also the
      // maximum input offset, which will be the number of indices per vertex we
      // need to skip.
      domInputLocalOffsetRef sortedInputs[MeshStreams::NumStreams];
      MeshStreams::classifyInputs(meshPrims[iPrim]->getInputs(), sortedInputs);

      S32 offsets[MeshStreams::NumStreams];
      for (S32 i = 0; i < MeshStreams::NumStreams; i++)
         offsets[i] = sortedInputs[i] ? sortedInputs[i]->getOffset() : -1;

      // Loop through indices
      const domUint* pSrcData = &(pTriData->get(0));

      for (U32 iTri = 0; iTri < numTriangles; iTri++) {

         // If the next triangle could cause us to index across a 16-bit
         // boundary, split this primitive and clear the tuple map to
         // ensure primitives only index verts within a 16-bit range.
         if (vertTuples.size() &&
            (((vertTuples.size()-1) ^ (vertTuples.size()+2)) & 0x10000))
         {
            // Pad vertTuples up to the next 16-bit boundary
            while (vertTuples.size() & 0xFFFF)
               vertTuples.push_back(VertTuple(vertTuples.last()));

            // Split the primitive at the current triangle
            S32 indicesRemaining = (numTriangles - iTri) * 3;
            if (iTri > 0)
            {
               daeErrorHandler::get()->handleWarning(avar("Splitting primitive "
                  "in %s: too many verts for 16-bit indices.", _GetNameOrId(geometry)));

               primitives.last().numElements -= indicesRemaining;
               primitives.push_back(TSDrawPrimitive(primitives.last()));
            }

            primitives.last().numElements = indicesRemaining;
            primitives.last().start = indices.size() - indicesRemaining;

            tupleMap.clear();
         }

         streams.reset();
         streams.readInputs(meshPrims[iPrim]->getInputs());

         for (U32 v = 0; v < 3; v++) {
            // Collect vert tuples into a single array so we can easily grab
            // vertex data later.
            VertTuple tuple;
            tuple.prim = iPrim;
            tuple.vertex = offsets[MeshStreams::Points]  >= 0 ? pSrcData[offsets[MeshStreams::Points]] : -1;
            tuple.normal = offsets[MeshStreams::Normals] >= 0 ? pSrcData[offsets[MeshStreams::Normals]] : -1;
            tuple.color  = offsets[MeshStreams::Colors]  >= 0 ? pSrcData[offsets[MeshStreams::Colors]] : -1;
            tuple.uv     = offsets[MeshStreams::UVs]     >= 0 ? pSrcData[offsets[MeshStreams::UVs]] : -1;
            tuple.uv2    = offsets[MeshStreams::UV2s]    >= 0 ? pSrcData[offsets[MeshStreams::UV2s]] : -1;

            tuple.dataVertex = tuple.vertex > -1 ? streams.points.getPoint3FValue(tuple.vertex) : Point3F::Max;
            tuple.dataNormal = tuple.normal > -1 ? streams.normals.getPoint3FValue(tuple.normal) : Point3F::Max;
            tuple.dataColor  = tuple.color > -1  ? streams.colors.getColorIValue(tuple.color) : ColorI(0,0,0);
            tuple.dataUV     = tuple.uv > -1     ? streams.uvs.getPoint2FValue(tuple.uv) : Point2F::Max;
            tuple.dataUV2    = tuple.uv2 > -1    ? streams.uv2s.getPoint2FValue(tuple.uv2) : Point2F::Max;

            VertTupleMap::Iterator itr = tupleMap.find(tuple);
            if (itr == tupleMap.end())
            {
               itr = tupleMap.insert(tuple, vertTuples.size());
               vertTuples.push_back(tuple);
            }

            // Collada uses CCW for front face and Torque uses the opposite, so
            // for normal (non-inverted) meshes, the indices are flipped.
            if (appNode->invertMeshes)
               dstIndex[v] = itr->value;
            else
               dstIndex[2 - v] = itr->value;

            pSrcData += meshPrims[iPrim]->getStride();
         }
         dstIndex += 3;
      }
   }

   for (int iPrim = 0; iPrim < meshPrims.size(); iPrim++)
      delete meshPrims[iPrim];
	*/

void FbxAppNode::reportModes(KFbxLayerElementNormal *leNormal,KFbxLayerElementUV *leUV)
{
	if (leNormal)
	{
		switch (leNormal->GetMappingMode())
		{
		case KFbxLayerElement::eBY_CONTROL_POINT:
			{
				Con::printf("Normal mapping mode eBY_CONTROL_POINT");
				switch (leNormal->GetReferenceMode())
				{
				case KFbxLayerElement::eDIRECT:
					Con::printf("Normal reference eDIRECT");
					break;
				case KFbxLayerElement::eINDEX_TO_DIRECT:
						Con::printf("Normal reference eINDEX_TO_DIRECT");
					break;
				}
			}
		case KFbxLayerElement::eBY_POLYGON_VERTEX:
			{
				Con::printf("Normal mapping mode eBY_POLYGON_VERTEX");
				//int lTextureUVIndex = pMesh->GetTextureUVIndex(i, j);
				switch (leNormal->GetReferenceMode())
				{
				case KFbxLayerElement::eDIRECT:
						Con::printf("Normal reference eDIRECT");
					break;
				case KFbxLayerElement::eINDEX_TO_DIRECT:
						Con::printf("Normal reference eINDEX_TO_DIRECT");
					break;
				}
			}
		}
	}
	if (leUV)
	{
		switch (leUV->GetMappingMode())
		{
		case KFbxLayerElement::eBY_CONTROL_POINT:
			{
				Con::printf("UV mapping mode eBY_CONTROL_POINT");
				switch (leUV->GetReferenceMode())
				{
				case KFbxLayerElement::eDIRECT:
					Con::printf("UV reference eDIRECT");
					break;
				case KFbxLayerElement::eINDEX_TO_DIRECT:
					Con::printf("UV reference eINDEX_TO_DIRECT");
					break;
				}
			}
		case KFbxLayerElement::eBY_POLYGON_VERTEX:
			{
				Con::printf("UV mapping mode eBY_POLYGON_VERTEX");
				//int lTextureUVIndex = pMesh->GetTextureUVIndex(i, j);
				switch (leUV->GetReferenceMode())
				{
				case KFbxLayerElement::eDIRECT:
					Con::printf("UV reference eDIRECT");
					break;
				case KFbxLayerElement::eINDEX_TO_DIRECT:
					Con::printf("UV reference eINDEX_TO_DIRECT");
					break;
				}
			}
		}
	}
}
// Get all geometry attached to this node
void FbxAppNode::buildMeshList()
{
	//HERE: this doesn't help us, because in at least my output fbx files, my mesh root is a child of
	//the scene root node, not any limb node that would be made into an FbxAppNode.
	//Need to load it up on scene load.

	//mNode->

	//mNode->GetChildCount();

	/*
   // Process children: collect <instance_geometry> and <instance_controller> elements
   for (int iChild = 0; iChild < p_domNode->getContents().getCount(); iChild++) {

      daeElement* child = p_domNode->getContents()[iChild];
      switch (child->getElementType()) {

         case Fbx_TYPE::INSTANCE_GEOMETRY:
            mMeshes.push_back(new FbxAppMesh(daeSafeCast<domInstance_geometry>(child), this));
            break;

         case Fbx_TYPE::INSTANCE_CONTROLLER:
            mMeshes.push_back(new FbxAppMesh(daeSafeCast<domInstance_controller>(child), this));
            break;
      }
   }*/
}

bool FbxAppNode::animatesTransform(const AppSequence* appSeq)
{/*
   // Check if any of this node's transform elements are animated during the
   // sequence interval
   for (int iTxfm = 0; iTxfm < nodeTransforms.size(); iTxfm++) {
      if (nodeTransforms[iTxfm].isAnimated(appSeq->getStart(), appSeq->getEnd()))
         return true;
   }*/
   return false;
}

/// Get the world transform of the node at the specified time
MatrixF FbxAppNode::getNodeTransform(F32 time)
{
   // Avoid re-computing the default transform if possible
   if (defaultTransformValid && time == TSShapeLoader::DefaultTime)
   {
		//Con::printf("TSShapeLoader::DefaultTime: time %f",time);
      return defaultNodeTransform;
   }
   else
   {
		//Con::printf("getNodeTransform: time %f",time);
      MatrixF nodeTransform = getTransform(time);

      // Check for inverted node coordinate spaces => can happen when modelers
      // use the 'mirror' tool in their 3d app. Shows up as negative <scale>
      // transforms in the Collada model.
      //if (m_matF_determinant(nodeTransform) < 0.0f)
      //{
      //   // Mark this node as inverted so we can mirror mesh geometry, then
      //   // de-invert the transform matrix
      //   invertMeshes = true;
      //   nodeTransform.scale(Point3F(1, 1, -1));
      //}

       //Cache the default transform
      if (time == TSShapeLoader::DefaultTime)
      {
         defaultTransformValid = true;
         defaultNodeTransform = nodeTransform;
      }

      return nodeTransform;
   }
}

MatrixF FbxAppNode::getTransform(F32 time)
{
	return defaultNodeTransform;
	/*
   // Check if we can use the last computed transform
   if (time == lastTransformTime)
      return lastTransform;

   if (appParent) {
      // Get parent node's transform
      lastTransform = appParent->getTransform(time);
   }
   else {
      // no parent (ie. root level) => scale by global shape <unit>
      lastTransform.identity();
      //lastTransform.scale(mScaleFactor);
      //if (!isBounds())
       //  FbxUtils::convertTransform(lastTransform);     // don't convert bounds node transform (or upAxis won't work!)
   }

	//S32 frame = S32(time/
   // Multiply by local node transform elements
   for (int i = 0; i < nodeTransforms.size(); i++) {

      MatrixF mat(true);
		mat = nodeTransforms[i];

      // Convert the transform element to a MatrixF
      //switch (nodeTransforms[iTxfm].element->getElementType()) {
      //   case Fbx_TYPE::TRANSLATE: mat = vecToMatrixF<domTranslate>(nodeTransforms[iTxfm].getValue(time));  break;
      //   case Fbx_TYPE::SCALE:     mat = vecToMatrixF<domScale>(nodeTransforms[iTxfm].getValue(time));      break;
      //   case Fbx_TYPE::ROTATE:    mat = vecToMatrixF<domRotate>(nodeTransforms[iTxfm].getValue(time));     break;
      //   case Fbx_TYPE::MATRIX:    mat = vecToMatrixF<domMatrix>(nodeTransforms[iTxfm].getValue(time));     break;
      //   case Fbx_TYPE::SKEW:      mat = vecToMatrixF<domSkew>(nodeTransforms[iTxfm].getValue(time));       break;
      //   case Fbx_TYPE::LOOKAT:    mat = vecToMatrixF<domLookat>(nodeTransforms[iTxfm].getValue(time));     break;
      //}

      // Remove node scaling (but keep reflections) if desired
      //if (FbxUtils::getOptions().ignoreNodeScale)
      //{
      //   Point3F invScale = mat.getScale();
      //   invScale.x = invScale.x ? (1.0f / invScale.x) : 0;
      //   invScale.y = invScale.y ? (1.0f / invScale.y) : 0;
      //   invScale.z = invScale.z ? (1.0f / invScale.z) : 0;
      //   mat.scale(invScale);
      //}

      // Post multiply the animated transform
      lastTransform.mul(mat);
   }

   lastTransformTime = time;
	
   return lastTransform;
	*/
}
