//-----------------------------------------------------------------------------
// Fbx-2-DTS
// Copyright (C) 2011 BrokeAss Games, LLC
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

/*
   Resource stream -> Buffer
   Buffer -> Fbx Scene
   Fbx Scene -> TSShapeLoader
   TSShapeLoader installed into TSShape
*/


#include "platform/platform.h"

#include "ts/fbx/fbxShapeLoader.h"
#include "ts/fbx/fbxAppNode.h"
#include "ts/fbx/fbxAppMesh.h"
#include "ts/fbx/fbxAppSequence.h"
//#include "ts/fbx/Thumbnail.h"

#include <fbxfilesdk/kfbxio/kfbxiosettings.h>

#include "core/util/tVector.h"
#include "core/strings/findMatch.h"
#include "core/stream/fileStream.h"
#include "core/fileObject.h"
#include "ts/tsShape.h"
#include "ts/tsShapeInstance.h"
#include "T3D/shapeBase.h"
#include "materials/materialManager.h"
#include "ts/tsShapeConstruct.h"
#include "core/util/zip/zipVolume.h"
#include "gfx/bitmap/gBitmap.h"


#ifdef IOS_REF
	#undef  IOS_REF
	#define IOS_REF (*(mSdkManager->GetIOSettings()))
#endif

KFbxPose *gBindPose;//Damn, passing things to the appNodes doesn't seem to work (mScaleFactor), so can't trust mBindPose either.

//-----------------------------------------------------------------------------
/// This function is invoked by the resource manager based on file extension.
TSShape* loadFbxShape(const Torque::Path &path)
{
   if (!Torque::FS::IsFile(path))
   {
      // file does not exist, bail.
		Con::errorf("Could not find the file.");
      return NULL;
   }

   TSShapeConstructor* tscon = TSShapeConstructor::findShapeConstructor(path.getFullPath());
   if (tscon)
   {
		Con::printf("Found a shape constructor.");
      //ColladaUtils::getOptions() = tscon->mOptions;
   }

   String mountPoint;
   Torque::Path fbxPath;

   // Load Fbx model and convert to 3space
   TSShape* tss = 0;

	//CHANGE: for FBX, we are going to make the loader exist on its own without
	// telling it what to load yet, so it can get its sdk manager instantiated.
	FbxShapeLoader loader;//HERE: how about deleting this when we're done??
	if (loader.LoadScene(path.getFullPath().c_str()))
	{		
		tss = loader.generateShape(path);//This is where all the work gets done - the fbxShapeLoader
		// version of enumerateScene(), which gets called from TSShapeLoader::generateShape().

		if (tss)
      {
         // Cache the FBX model to a DTS file for faster loading next time.
         FileStream dtsStream;
			Torque::Path cachedPath(path);
			cachedPath.setExtension("cached.dts");
         if (dtsStream.open(cachedPath.getFullPath(), Torque::FS::File::Write))
         {
            Con::printf("Writing cached FBX shape to %s", cachedPath.getFullPath().c_str());
            tss->write(&dtsStream);
         }
         // Add fbx materials to materials.cs
         //updateMaterialsScript(path);
		}
	}

	//(Since we created a local TSShapeLoader object, destructor should get called automatically.)
	return tss;
}


///////////////////////////////////////////////////////////////////////////////

FbxShapeLoader::FbxShapeLoader()
{
	mFbxLimbNodes = 0;
	mFbxMeshes = 0;
	mFbxTextures = 0;
	//mFbxSequences = 0;
	mFbxClusters = 0;
	mFbxActors = 0;
	mFbxScale = 1.0;//Sets up from fbx file UnitScaleFactor in enumerateScene.
	mFbxRootNode = NULL;
	mFbxShapeRootNode = NULL;
	mFbxSkeletonBaseNode = NULL;
	mFoundLimbNode = false;//FIX: this is a hack that only finds us one skeleton.
	mForceReadNode = false;
	InitializeFbxSdkObjects(mSdkManager,mScene);
}


FbxShapeLoader::~FbxShapeLoader()
{
   //for (int iAnim = 0; iAnim < animations.size(); iAnim++) {
   //   for (int iChannel = 0; iChannel < animations[iAnim]->size(); iChannel++)
   //      delete (*animations[iAnim])[iChannel];
   //   delete animations[iAnim];
   //}
   //animations.clear();

	//Hmm, wait till we find out if having the scene pointer helps the memory problem
	//before we deal with adding it here. 
	//DestroyFbxSdkObjects(mSdkManager);//,pScene);
}

bool FbxShapeLoader::LoadScene(const char* pFilename)
{
	 int lFileMajor, lFileMinor, lFileRevision;
    int lSDKMajor,  lSDKMinor,  lSDKRevision;
    //int lFileFormat = -1;
    int i, lAnimStackCount,lNodeCount,lMeshCount;
    bool lStatus;
    char lPassword[1024];

    // Get the file version number generate by the FBX SDK.
    KFbxSdkManager::GetFileFormatVersion(lSDKMajor, lSDKMinor, lSDKRevision);

    // Create an importer.
    KFbxImporter* lImporter = KFbxImporter::Create(mSdkManager,"");

    // Initialize the importer by providing a filename.
    const bool lImportStatus = lImporter->Initialize(pFilename, -1, mSdkManager->GetIOSettings());
    lImporter->GetFileVersion(lFileMajor, lFileMinor, lFileRevision);

	 if (!mScene)
	 {
		 lImporter->Destroy();
		 return false;
	 }

    if( !lImportStatus )
    {
        Con::printf("Call to KFbxImporter::Initialize() failed.\n");
        Con::printf("Error returned: %s\n\n", lImporter->GetLastErrorString());

        if (lImporter->GetLastErrorID() == KFbxIO::eFILE_VERSION_NOT_SUPPORTED_YET ||
            lImporter->GetLastErrorID() == KFbxIO::eFILE_VERSION_NOT_SUPPORTED_ANYMORE)
        {
            Con::printf("FBX version number for this FBX SDK is %d.%d.%d\n", lSDKMajor, lSDKMinor, lSDKRevision);
				Con::printf("FBX version number for file %s is %d.%d.%d\n\n", pFilename, lFileMajor, lFileMinor, lFileRevision);
		  }

		  return false;
	 }

	 Con::printf("FBX version number for this FBX SDK is %d.%d.%d\n", lSDKMajor, lSDKMinor, lSDKRevision);

	 if (lImporter->IsFBX())
	 {
		 //KFbxDocumentInfo* sceneInfo = lImporter->GetSceneInfo();
		 Con::printf("Name:  %s\n\n",lImporter->GetName());
		 //KFbxDocumentInfo* sceneInfo = lImporter->GetSceneInfo();
		 //lImporter->Import(kDocument);

		 Con::printf("FBX version number for file %s is %d.%d.%d\n\n", pFilename, lFileMajor, lFileMinor, lFileRevision);

		 // From this point, it is possible to access animation stack information without
		 // the expense of loading the entire file.


		 //kScene = lImporter->GetScene();
		 examineScene(lImporter);

		 // Import the scene.
		 lStatus = lImporter->Import(mScene);

		 //if (lStatus)
		 //{
			// Con::printf("Scene loaded!");
			// for (i = 0; i < mScene->GetMemberCount(); i++)
			// {
			//	 KFbxObject *member = mScene->GetMember(i);
			//	 examineMember(member);
			// }
		 //}
		 if(lStatus == false && lImporter->GetLastErrorID() == KFbxIO::ePASSWORD_ERROR)
		 {
			 Con::printf("Please enter password: ");

			 lPassword[0] = '\0';

			 scanf("%s", lPassword);
			 KString lString(lPassword);

			 //IOS_REF.SetStringProp(IMP_FBX_PASSWORD,      lString);
			 //IOS_REF.SetBoolProp(IMP_FBX_PASSWORD_ENABLE, true);

			 lStatus = lImporter->Import(mScene);

			 if(lStatus == false && lImporter->GetLastErrorID() == KFbxIO::ePASSWORD_ERROR)
			 {
				 Con::printf("\nPassword is wrong, import aborted.\n");
			 }
		 }

		 // Destroy the importer.
		 lImporter->Destroy();

		 return lStatus;
	 }
}

bool FbxShapeLoader::SaveScene(const char* pFilename, int pFileFormat, bool pEmbedMedia)
{
//    int lMajor, lMinor, lRevision;
//    bool lStatus = true;
//
//    // Create an exporter.
//    KFbxExporter* kExporter = KFbxExporter::Create(mSdkManager, "");
//
//    if( pFileFormat < 0 || pFileFormat >= mSdkManager->GetIOPluginRegistry()->GetWriterFormatCount() )
//    {
//        // Write in fall back format if pEmbedMedia is true
//        pFileFormat = mSdkManager->GetIOPluginRegistry()->GetNativeWriterFormat();
//
//        if (!pEmbedMedia)
//        {
//            //Try to export in ASCII if possible
//            int lFormatIndex, lFormatCount = mSdkManager->GetIOPluginRegistry()->GetWriterFormatCount();
//
//            for (lFormatIndex=0; lFormatIndex<lFormatCount; lFormatIndex++)
//            {
//                if (mSdkManager->GetIOPluginRegistry()->WriterIsFBX(lFormatIndex))
//                {
//                    KString kDesc =mSdkManager->GetIOPluginRegistry()->GetWriterFormatDescription(lFormatIndex);
//                    char *lASCII = "ascii";
//                    if (kDesc.Find(lASCII)>=0)
//                    {
//                        pFileFormat = lFormatIndex;
//                        break;
//                    }
//                }
//            }
//        }
//    }
//
//    // Set the export states. By default, the export states are always set to 
//    // true except for the option eEXPORT_TEXTURE_AS_EMBEDDED. The code below 
//    // shows how to change these states.
//
//    IOS_REF.SetBoolProp(EXP_FBX_MATERIAL,        true);
//    IOS_REF.SetBoolProp(EXP_FBX_TEXTURE,         true);
//    IOS_REF.SetBoolProp(EXP_FBX_EMBEDDED,        pEmbedMedia);
//    IOS_REF.SetBoolProp(EXP_FBX_SHAPE,           true);
//    IOS_REF.SetBoolProp(EXP_FBX_GOBO,            true);
//    IOS_REF.SetBoolProp(EXP_FBX_ANIMATION,       true);
//    IOS_REF.SetBoolProp(EXP_FBX_GLOBAL_SETTINGS, true);
//
//	 //KFbxIOSettings *IOSettings = mSdkManager->GetIOSettings();
//  //  IOSettings->SetBoolProp(EXP_FBX_MATERIAL,        true);
//  //  IOSettings->SetBoolProp(EXP_FBX_TEXTURE,         true);
//  //  IOSettings->SetBoolProp(EXP_FBX_EMBEDDED,        pEmbedMedia);
//  //  IOSettings->SetBoolProp(EXP_FBX_SHAPE,           true);
//  //  IOSettings->SetBoolProp(EXP_FBX_GOBO,            true);
//  //  IOSettings->SetBoolProp(EXP_FBX_ANIMATION,       true);
//  //  IOSettings->SetBoolProp(EXP_FBX_GLOBAL_SETTINGS, true);
//
//
//    // Initialize the exporter by providing a filename.
//    if(kExporter->Initialize(pFilename, pFileFormat, mSdkManager->GetIOSettings()) == false)//IOS_REF?
//    {
//        printf("Call to KFbxExporter::Initialize() failed.\n");
//        printf("Error returned: %s\n\n", kExporter->GetLastErrorString());
//        return false;
//    }
//
//    KFbxSdkManager::GetFileFormatVersion(lMajor, lMinor, lRevision);
//	 printf("FBX version number for this version of the FBX SDK is %d.%d.%d\n\n", lMajor, lMinor, lRevision);
//
//    // Export the scene.
//    lStatus = kExporter->Export(mScene); 
//
//    // Destroy the exporter.
//    kExporter->Destroy();
//    return lStatus;
	return false;
}

///////////////////////////////////////////////////////////////

void FbxShapeLoader::enumerateScene()
{
	Con::printf("calling FbxShapeLoader::enumerateScene!!!  shape nodes:  %d",shape->nodes.size());

	KFbxSystemUnit SceneSystemUnit = mScene->GetGlobalSettings().GetSystemUnit();
	mFbxScale = SceneSystemUnit.GetScaleFactor()/100.0;//Convert this to meters to fit into Ecstasy Motion/Torque scale.
	Con::printf("FBX unit scale:  %f m ",mFbxScale);

	
	for (U32 i=0;i<mScene->GetPoseCount();i++)
	{
		KFbxPose *kPose = mScene->GetPose(i);
		if (kPose->IsBindPose())
		{
			Con::printf("Found a Bind Pose %s",kPose->GetName());
			mBindPose = kPose;
			gBindPose = kPose;//Passing mScaleFactor to appNodes didn't work on all of them, so resorting to this. 
		}
	}

	//Got mSdkManager and mScene already fired up, and model already loaded, all we have to do 
	//now is make the nodes & mesh objects from what we have in the fbx model.
	for (U32 i = 0; i < mScene->GetMemberCount(); i++)
	{
		KFbxObject *member = mScene->GetMember(i);
		KFbxAnimStack *kAnimStack = NULL;
		kAnimStack = dynamic_cast<KFbxAnimStack *>(member);
		if (kAnimStack)
		{
			//if (kAnimStack->?) ... //Test something here to find out if we have actual sequence frames?
			appSequences.push_back(new FbxAppSequence(kAnimStack));
		}
	}

	for (U32 i = 0; i < mScene->GetMemberCount(); i++)
	{
		KFbxObject *member = mScene->GetMember(i);
		examineMember(member);//This is really just to print out information to the console, but
		//it does add up mFbxLimbNodes, Meshes, Clusters etc until we move that to where we actually 
		//make the nodes, etc.
	}

	Con::printf("\n\n LimbNodes:  %d,  Meshes:  %d,  Clusters: %d \n\n",
		mFbxLimbNodes,mFbxMeshes,mFbxClusters); 



	KFbxNode *kRootNode = NULL;
	kRootNode = mScene->GetRootNode();
	if (kRootNode)
	{
		Con::printf("Root node has %d children, total %d",kRootNode->GetChildCount(),
			kRootNode->GetChildCount(true));

		readNode(kRootNode);
	}
}


void FbxShapeLoader::examineScene(KFbxImporter* lImporter)
{
	int lFileMajor, lFileMinor, lFileRevision;
	int lSDKMajor,  lSDKMinor,  lSDKRevision;
	//int lFileFormat = -1;
	int i, lAnimStackCount,lNodeCount,lMeshCount;

	Con::printf("Animation Stack Information\n");

	lAnimStackCount = lImporter->GetAnimStackCount();

	KFbxNode *kRootNode = NULL;
	kRootNode = mScene->GetRootNode();

	if (kRootNode)
	{
		KFbxGeometry *kGeometry = mScene->GetGeometry(0);
		if (kGeometry)
			Con::printf("    FOUND GEOMETRY");

		KFbxMesh *kMesh = NULL;
		kMesh = kRootNode->GetMesh();
		if (kMesh)
		{
			Con::printf("    FOUND MESH");
			KFbxNode *kNode = kMesh->GetNode();
			//kNode->Get
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
				//appMesh->mOffset.set(-tVec[0]*sc,tVec[2]*sc,tVec[1]*sc);




				//kNode->GetPostRotation(pivotSet;
				//kNode->GetRotationOffset
			}
		}
		Con::printf("    Root node child count %d",kRootNode->GetChildCount());

		KFbxNode *kFirstChild = kRootNode->GetChild(0);
		if (kFirstChild)
			Con::printf("    First child child count %d",kFirstChild->GetChildCount());
	}

	Con::printf("\n    Number of Animation Stacks: %d\n", lAnimStackCount);
	Con::printf("    Current Animation Stack: \"%s\"\n", lImporter->GetActiveAnimStackName().Buffer());
	Con::printf("\n");

	for(i = 0; i < lAnimStackCount; i++)
	{
		KFbxTakeInfo* lTakeInfo = lImporter->GetTakeInfo(i);

		Con::printf("    Animation Stack %d\n", i);
		Con::printf("         Name: \"%s\"\n", lTakeInfo->mName.Buffer());
		Con::printf("         Description: \"%s\"\n", lTakeInfo->mDescription.Buffer());
		

		// Change the value of the import name if the animation stack should be imported 
		// under a different name.
		Con::printf("         Import Name: \"%s\"\n", lTakeInfo->mImportName.Buffer());

		// Set the value of the import state to false if the animation stack should be not
		// be imported. 
		Con::printf("         Import State: %s\n", lTakeInfo->mSelect ? "true" : "false");
	}

	Con::printf("src object count: %d",mScene->GetSrcObjectCount());

	Con::printf("name %s",mScene->GetName());
	Con::printf("nodes %d", mScene->GetNodeCount());
	Con::printf("materials %d",mScene->GetMaterialCount());
	Con::printf("textures: %d",mScene->GetTextureCount());
	Con::printf("poses: %d",mScene->GetPoseCount());
	Con::printf("characters: %d",mScene->GetCharacterCount());
	Con::printf("members: %d",mScene->GetMemberCount());

}

void FbxShapeLoader::examineMember(KFbxObject *member)
{
	S32 dstCnt = member->GetDstObjectCount();
	S32 srcCnt = member->GetSrcObjectCount();
	S32 dstPropCnt = member->GetDstPropertyCount();
	S32 implCnt = member->GetImplementationCount();
	S32 childCnt;

	//Con::printf("%s: -%s-",member->GetTypeName().Buffer(),member->GetName());

	if (!strcmp(member->GetTypeName().Buffer(),"Mesh")) 
	{
		KFbxMesh *kMesh = NULL;
		kMesh = dynamic_cast<KFbxMesh *>(member);
		if (kMesh)
		{
			mFbxMeshes++;
			//KFbxLayerElementNormal* leNormal = kMesh->GetLayer(l)->GetNormals();
			//leNormal->Get

			S32 controlPnts = kMesh->GetControlPointsCount();
			S32 deformers = kMesh->GetDeformerCount();
			S32 geomMapCnt = kMesh->GetDestinationGeometryWeightedMapCount();
			S32 polyCnt = kMesh->GetPolygonCount();
			S32 uvLayerCnt = kMesh->GetUVLayerCount();
			//kMesh->GetUVLayer?
			S32 polyVertCnt = kMesh->GetPolygonVertexCount();
			//kMesh->GetMaterialIndices();
			//kMesh->GetAllChannelUV(0);
			S32 UVCount = kMesh->GetTextureUVCount();
			S32 layerCount = kMesh->GetLayerCount();
         KFbxLayerElementUV* leUV = kMesh->GetLayer(0)->GetUVs();
			//S32 srcPropCnt = kMesh->GetSrcPropertyCount();
			//S32 dstPropCnt = kMesh->GetDstPropertyCount();
			//KFbxVector2 *uv;
			//kMesh->GetTextureUV(uv);
			Con::printf("Mesh:  %s  control points: %d, deformers %d, poly cnt %d, layer count %d, polyVertCnt  %d, UV count %d",
				member->GetName(),controlPnts,deformers,polyCnt,layerCount,polyVertCnt,UVCount);//,uv[0][0],uv[0][1]
			
			KFbxProperty prop = member->GetFirstProperty();
			while (prop.IsValid())
			{
				Con::printf("Mesh:  property %s   name %s   type %s",prop.GetLabel().Buffer(),
					prop.GetName().Buffer(),prop.GetPropertyDataType().GetName());
				prop=member->GetNextProperty(prop);
			}

			//   * The valid range for this parameter is 0 to \c KFbxMesh::GetPolygonCount().
			//   * \param pPositionInPolygon Position of polygon vertex in indexed polygon.
			//   * The valid range for this parameter is 0 to \c KFbxMesh::GetPolygonSize(pPolygonIndex).
			if ((polyCnt>0)&&(polyCnt<1000))
			{
				for (U32 k=0;k<polyCnt;k++)
				{
					for (U32 m=0;m<kMesh->GetPolygonSize(k);m++)
					{
						S32 polyIndex = kMesh->GetPolygonVertex(k,m);
						//Con::printf("   polygon %d index %d",k,polyIndex);
					}
				}
			}
		}
	} else if (!strcmp(member->GetTypeName().Buffer(),"LimbNode")) {
		KFbxNode *kNode = NULL;
		kNode = dynamic_cast<KFbxNode *>(member);
		if (kNode)
		{
			S32 childCnt = kNode->GetChildCount();
			KFbxVector4 tVec;
			KFbxVector4 rVec;
			kNode->GetDefaultT(tVec);
			kNode->GetDefaultR(rVec);
			//S32 refBy = kNode->GetReferencedByCount();
			//Con::printf("LimbNode: %s  childCnt %d, defaultT %f %f %f   ",
			//	member->GetName(),childCnt,tVec[0],tVec[1],tVec[2]);
			
			mFbxLimbNodes++;
		}
	} else if (!strcmp(member->GetTypeName().Buffer(),"Cluster")) {
		KFbxCluster *kCluster = NULL;
		kCluster = dynamic_cast<KFbxCluster *>(member);
		if (kCluster)
		{
			mFbxClusters++;
			int indicesCount = kCluster->GetControlPointIndicesCount();
			int *indices = kCluster->GetControlPointIndices();
			double *weights = kCluster->GetControlPointWeights();
			KFbxNode *link = kCluster->GetLink();
				
			KFbxXMatrix bindMatrix;
			kCluster->GetTransformLinkMatrix(bindMatrix);
			KFbxVector4 Row0 = bindMatrix.GetRow(0);
			KFbxVector4 Row1 = bindMatrix.GetRow(1);
			KFbxVector4 Row2 = bindMatrix.GetRow(2);
			KFbxVector4 Row3 = bindMatrix.GetRow(3);
			//Con::printf("Link node: %s",link->GetName());
			//Con::printf("LinkMatrix Row0: %f  %f  %f  %f",Row0[0],Row0[1],Row0[2],Row0[3]);
			//Con::printf("           Row1: %f  %f  %f  %f",Row1[0],Row1[1],Row1[2],Row1[3]);
			//Con::printf("           Row2: %f  %f  %f  %f",Row2[0],Row2[1],Row2[2],Row2[3]);
			//Con::printf("           Row3: %f  %f  %f  %f\n",Row3[0],Row3[1],Row3[2],Row3[3]);
			//Con::printf("Cluster: %s  %d  indices   %d, %d, %d  weights  %f, %f, %f, link %s",// ...     %f, %f, %f, ... 
			//	kCluster->GetName(),indicesCount,indices[0],indices[1],indices[2],weights[0],weights[1],weights[2],link->GetName());
			//if (!strncmp(kCluster->GetName(),"LeftHand",8))
			//{
			//	for (U32 i=0;i<indicesCount;i++)
			//	{
			//		Con::printf("Left hand %d indices: %d",i,indices[i]);
			//	}
			//}
		}

	} else if (!strcmp(member->GetTypeName().Buffer(),"Skin")) {
		KFbxSkin *kSkin = NULL;
		kSkin = dynamic_cast<KFbxSkin *>(member);
		if (kSkin)
		{
			S32 clusterCnt = kSkin->GetClusterCount() ;
			Con::printf("Skin:  cluster count %d",clusterCnt);
			
			//KFbxProperty prop = member->GetFirstProperty();
			//while (prop.IsValid())
			//{
			//	Con::printf("  property %s   name %s   type %s",prop.GetLabel().Buffer(),
			//		prop.GetName().Buffer(),prop.GetPropertyDataType().GetName());
			//	prop=member->GetNextProperty(prop);
			//}
		} else {
			Con::printf("%s : %s",member->GetTypeName().Buffer(),member->GetName());
		}
	}
}

void FbxShapeLoader::fbxGenerateNodeTransform(AppNode* node, S32 frame, bool blend, F32 referenceTime,
                                          QuatF& rot, Point3F& trans, QuatF& srot, Point3F& scale)
{
	//F32 t=0.0;
	//MatrixF m1 = getLocalNodeMatrix(node, t);
	//if (blend)
	//{
	//	MatrixF m0 = getLocalNodeMatrix(node, referenceTime);
	//	m1 = m0.inverse() * m1;
	//}

	//HERE:  set m1 to a matrix from an array, per node.  
	//m1 = node->Get
	FbxAppNode *appNode = dynamic_cast<FbxAppNode *>(node);
	if (!appNode)
		return;

	MatrixF m1(true),m2(true);
	Point3F pos;
	if (appNode->nodeTransforms.size()>frame)
	{
		FbxAppNode *parentNode = NULL;
		m1 = appNode->nodeTransforms[frame];

		if (appNode->mParentIndex >= 0)
		{
			parentNode = dynamic_cast<FbxAppNode *>(appNodes[appNode->mParentIndex]);
			if (parentNode)
			{
				pos = appNode->defaultNodeTransform.getPosition();//1A
				pos -= parentNode->defaultNodeTransform.getPosition();
				Point3F temp(-pos.x,pos.z,pos.y);
				pos = temp;
			}
		} else {		
				pos = m1.getPosition();//1B
		}
		
		//m1.setPosition(Point3F(-pos.x,pos.z,pos.y));//2A
		m1.setPosition(Point3F(pos.x,pos.y,pos.z));//2B

		//m1.setPosition(Point3F(-pos.x,pos.z,pos.y));
	} else {
		Point3F mPos = appNode->defaultNodeTransform.getPosition();
		m1.setPosition(Point3F(-mPos.x/mFbxScale,mPos.z/mFbxScale,mPos.y/mFbxScale));
	}
	//m1.rightToLeftHanded();//MAYBE???
   rot.set(m1);
   trans = m1.getPosition();
   srot.identity();        //@todo: srot not supported yet
   scale = m1.getScale();
}

void FbxShapeLoader::readNode(KFbxNode *node)
{//This is for the root node of the skeleton, *not* scene root node or any other node.
	//We need more analysis of the limbnodes previous to this point so we know which one this is.
	if (mFoundLimbNode)//FIX: hack that only finds one skeleton, starting with this.
		return;

	Con::printf("  skeleton root node: %s   type  %s   childcount %d",node->GetName(),
				node->GetTypeName().Buffer(),node->GetChildCount());

	//Whew, ugly if statement, what it says is: if you're a LimbNode and you have children, or if 
	//mForceReadNode is turned on, then congrats you're it, go on down to the lower block.  If none 
	//of the above, try all the children, if no children then go back to top and set mForceReadNode.
	if ( (!((!strcmp(node->GetTypeName().Buffer(),"LimbNode")) && (node->GetChildCount()>0))) &&
		     (!mForceReadNode) )
	//if (!strcmp(node->GetTypeName().Buffer(),"LimbNode"))
	  {
		for (U32 i=0;i<node->GetChildCount();i++)
		{
			readNode(node->GetChild(i));//recursive!  //OBSOLETE AGAIN?
		}		
		if (!mFoundLimbNode)
		{
			for (U32 i=0;i<node->GetChildCount();i++)
			{
				KFbxNode *child = node->GetChild(i);
				if (!strcmp(child->GetTypeName().Buffer(),"LimbNode"))
				{
					mForceReadNode = true;
					readNode(child);
				}
				if (!mFoundLimbNode)
				{//failing above, send the base node itself. (?)
					mForceReadNode = true;
					readNode(node);
				}
			}		
		}
	} 
	else	
	{//In any event, here we are, this is going to be the root node of our skeleton.
		mFoundLimbNode = true;
		FbxAppNode *appNode = new FbxAppNode(node, 0);

		appNode->mScaleFactor = mFbxScale;
		appNode->mBindPose = mBindPose;
		//in fbxAppNode::BuildChildlist?  Hmmm
		//mChildNodes.push_back(new ColladaAppNode(node, this));
      
		if (!processNode(appNode))
         delete appNode;

		//HERE: this is lame, but for first pass, going to try to attach any existing meshes
		//to the first defined LimbNode
		for (U32 i=0;i<mScene->GetMemberCount();i++)
		{
			KFbxObject *member = mScene->GetMember(i);
			if (!strcmp(member->GetTypeName().Buffer(),"Mesh"))
			{
				KFbxMesh *kMesh = NULL;
				kMesh = dynamic_cast<KFbxMesh *>(member);
				if (kMesh)
				{
					//kMesh->Get
					Con::printf("Mesh attaching to AppNode %s:  control points %d ",
						node->GetName(),kMesh->GetControlPointsCount());

					appNode->mMeshes.push_back(new FbxAppMesh(kMesh,appNode));
					appNode->buildMesh(dynamic_cast<FbxAppMesh*>(appNode->mMeshes.last()));
				}
			}
			//else if (!strcmp(member->GetTypeName().Buffer(),"Clip")) {
			//	//Not sure why it's "Clip" but this seems to be consistent, among my tiny sample set anyway. 
			//	//NEXT:  work with FBX property connections to find out which mesh this texture belongs to.
			//	//FOR NOW: working with only one mesh, one texture models, just assign to first mesh.
			//} else {
			//	Con::printf("member type:  %s,  name:  %s",member->GetTypeName().Buffer(),member->GetName());
			//}
		}
		FbxAppMesh *appMesh = NULL;
		S32 kFbxClusters = 0;
		for (U32 i=0;i<mScene->GetMemberCount();i++)
		{
			KFbxObject *member = mScene->GetMember(i);
			if (!strcmp(member->GetTypeName().Buffer(),"Clip"))
			{
				appNode->mMeshes[0]->appMaterials.push_back(new FbxAppMaterial(member->GetName()));
				Con::printf("pushing back material: %s",member->GetName());
			}
		}
		for (U32 i=0;i<mScene->GetMemberCount();i++)
		{
			KFbxObject *member = mScene->GetMember(i);
			if (!strcmp(member->GetTypeName().Buffer(),"Skin"))
			{
				appMesh = dynamic_cast<FbxAppMesh*>(appNode->mMeshes[0]);
				appMesh->mIsSkin = true;
				Con::printf("isSkin = true!");
			}
		}

		//HERE:  WAIT on clusters.  If you do all this later, you can have access to the FbxAppNode array,
		//which does not exist yet.
		//for (U32 i=0;i<mScene->GetMemberCount();i++)
		//{
		//	KFbxObject *member = mScene->GetMember(i);
		//	if (!strcmp(member->GetTypeName().Buffer(),"Cluster")) {
		//		KFbxCluster *kCluster = NULL;
		//		kCluster = dynamic_cast<KFbxCluster *>(member);
		//		if (kCluster)
		//		{
		//       ...
		//		}
		//	}
		//}

	}
	return;
}

//-----------------------------------------------------------------------------
/// Check if an up-to-date cached DTS is available for this DAE file
bool FbxShapeLoader::canLoadCachedDTS(const Torque::Path& path)
{
   // Generate the cached filename
   Torque::Path cachedPath(path);
   cachedPath.setExtension("cached.dts");

   // Check if a cached DTS newer than this file is available
   FileTime cachedModifyTime;
   if (Platform::getFileTimes(cachedPath.getFullPath(), NULL, &cachedModifyTime))
   {
      bool forceLoadFBX = Con::getBoolVariable("$Fbx::forceLoadFBX", false);

      FileTime fbxModifyTime;
      if (!Platform::getFileTimes(path.getFullPath(), NULL, &fbxModifyTime) ||
         (!forceLoadFBX && (Platform::compareFileTimes(cachedModifyTime, fbxModifyTime) >= 0) ))
      {
         // FBX not found, or cached DTS is newer
         return true;
      }
   }

   return false;
}



void FbxShapeLoader::postEnumerateScene()
{

	//Now, add clusters, AFTER we have generated our AppNodes.

	FbxAppNode *appNode = NULL;
	FbxAppMesh *appMesh = NULL;
	S32 kFbxClusters = 0;
	
	
	appNode =  dynamic_cast<FbxAppNode*>(appNodes[0]);
	appMesh = dynamic_cast<FbxAppMesh*>(appNode->mMeshes[0]);
	
	Con::printf("postEnumerate!  scene members: %d",mScene->GetMemberCount());
	for (U32 i=0;i<mScene->GetMemberCount();i++)
	{
		KFbxObject *member = mScene->GetMember(i);
		//Con::printf("member type: %s",member->GetTypeName().Buffer());
		if (!strcmp(member->GetTypeName().Buffer(),"Cluster")) 
		{
			KFbxCluster *kCluster = NULL;
			kCluster = dynamic_cast<KFbxCluster *>(member);
			if (kCluster)
			{
				int indicesCount = kCluster->GetControlPointIndicesCount();
				int *indices = kCluster->GetControlPointIndices();//HERE: 
				double *weights = kCluster->GetControlPointWeights();
				KFbxNode *link = kCluster->GetLink();
				//S32 bnIndex = link->GetBoneIndex...
				//appMesh->boneIndex.push_back(bnIndex);
				//Con::printf("cluster seeking appNode, link name %s",link->GetName());
				for (U32 j=0;j<appNodes.size();j++)
				{
					if (dynamic_cast<FbxAppNode*>(appNodes[j])->mNode==link)
					{
						appNode = dynamic_cast<FbxAppNode*>(appNodes[j]);
						//Con::printf("cluster linking to: %s",appNode->getName());
					}
				}
				for (U32 j=0;j<appMeshes.size();j++)
				{
					if (dynamic_cast<FbxAppMesh*>(appMeshes[j])->appNode==appNode)
					{
						appMesh = dynamic_cast<FbxAppMesh*>(appMeshes[j]);
						//Con::printf("cluster linking to: %s",appNode->getName());
					}
				}
				if (!appNode)
					continue;

				//FbxAppNode linkNode(link);
				//linkNode.defaultNodeTransform = appNode->defaultNodeTransform;
				appMesh->bones.push_back(new FbxAppNode(link));
				dynamic_cast<FbxAppNode*>(appMesh->bones.last())->defaultNodeTransform = appNode->defaultNodeTransform;

				appMesh->bones.last()->mScaleFactor = mFbxScale;
				dynamic_cast<FbxAppNode*>(appMesh->bones.last())->mBindPose = mBindPose;

				//appMesh->bones.push_back((AppNode*)(&linkNode));

				//appMesh->nodeIndex.push_back(kFbxClusters);//FIX!!! This MIGHT just work, but it's all wrong.
				//HERE: we need a bone index number for link, meaning a reliable index into the skeleton.  They might all
				//come in in the right order here though, they should for my models at least.
				//MatrixF initTrans;  initTrans.identity();//this probably needs a valid position...
				//KFbxVector4 tVec;
				//link->GetDefaultT(tVec);
				//Point3F pos;
				////pos.set(-tVec[0],tVec[2],tVec[1]);
				//pos.set(tVec[0],tVec[1],tVec[2]);
				//pos *= 1.0 / mFbxScale;
				//initTrans.setPosition(pos);//This probably won't work.
				//initTrans = link->defaultNodeTransform;//CRAP this doesn't exist yet!  we are reading the FIRST NODE.
				MatrixF initTrans = appNode->defaultNodeTransform;
				initTrans.inverse();
				appMesh->initialTransforms.push_back(initTrans);
				for (U32 j=0;j<indicesCount;j++)
				{
					appMesh->boneIndex.push_back(kFbxClusters);//which one?
					appMesh->vertexIndex.push_back(indices[j]);//?
					appMesh->weight.push_back(weights[j]);

					//HERE: now we need to find any new vertices that were added to deal with the problem of 
					//multiple UVs on the same vertex.  
					for (U32 k=0;k<appMesh->vertRefs.size();k++)
					{
						if (appMesh->vertRefs[k].base==indices[j])
						{
							appMesh->boneIndex.push_back(kFbxClusters);
							appMesh->vertexIndex.push_back(appMesh->vertRefs[k].index);
							appMesh->weight.push_back(weights[j]);
						}
					}
				}
				//Con::printf("vertIndex size: %d",appMesh->vertexIndex.size());
				kFbxClusters++;
				

				//Con::printf("AppMesh nodeIndex: %d   boneIndex  %d  vertexIndex  %d",
				//	appMesh->nodeIndex.size(),appMesh->boneIndex.size(),appMesh->vertexIndex.size());
				//Con::printf("Cluster: %s  %d  indices   %d, %d, %d  weights  %f, %f, %f, link %s",// ...     %f, %f, %f, ... 
				//	kCluster->GetName(),indicesCount,indices[0],indices[1],indices[2],weights[0],weights[1],weights[2],link->GetName());
			}
		} 
		else if (!strcmp(member->GetTypeName().Buffer(),"LimbNode"))
		{
			KFbxNode *kNode = NULL;
			kNode = dynamic_cast<KFbxNode *>(member);
			if (kNode)
			{
				appNode = NULL;
				for (U32 j=0;j<appNodes.size();j++)
				{
					if (dynamic_cast<FbxAppNode*>(appNodes[j])->mNode==kNode)
						appNode = dynamic_cast<FbxAppNode*>(appNodes[j]);
				}
				if (!appNode)
					continue;

				KFbxProperty prop = member->GetFirstProperty();
				S32 numSeqs = appSequences.size();
				FbxAppSequence *appSeq = NULL;
				if (numSeqs>0)
					appSeq = dynamic_cast<FbxAppSequence*>(appSequences.last());
				else
					continue;

				while (prop.IsValid())
				{
					if (!strcmp(prop.GetLabel().Buffer(),"Lcl Translation"))
					{
						KFbxAnimCurveNode *curveNode = NULL;
						KFCurve *kFCurve_X,*kFCurve_Y,*kFCurve_Z;
						KFbxAnimCurve *kCurve_X,*kCurve_Y,*kCurve_Z;
						curveNode = prop.GetCurveNode();
						if (curveNode)
						{
							S32 channelsCnt = curveNode->GetChannelsCount();
							S32 curveCnt = curveNode->GetCurveCount(0);

							Con::printf("GOT A Translation CURVE NODE:  property %s   channels %d   one %s  two %s curveCnt %d",
								prop.GetLabel().Buffer(),channelsCnt,curveNode->GetChannelName(0).Buffer(),
								curveNode->GetChannelName(1).Buffer(),curveCnt);

							if (curveCnt==0)
							{
								//Whoops, got an animstack, but no curve nodes.
								if (appSequences.size()>0)//FIX, make sure to only delete THIS sequence,
									appSequences.decrement();//if we get multiples going on.
								goto NEXT;//SORRY!  if/break is a pita...
							}

							kCurve_X = curveNode->GetCurve(0,0);
							kCurve_Y = curveNode->GetCurve(1,0);
							kCurve_Z = curveNode->GetCurve(2,0);

							if (!(kCurve_X && kCurve_Y && kCurve_Z))
								goto NEXT;//SORRY!  if/break is a pita...

							kFCurve_X = kCurve_X->GetKFCurve();
							kFCurve_Y = kCurve_Y->GetKFCurve();
							kFCurve_Z = kCurve_Z->GetKFCurve();

							//KFbxProperty T_channel = curveNode->GetChannel("T");//??? This shit is WEIRD.
							//curveNode->GetAnimationInterval(...);

							if (kFCurve_X  &&  kFCurve_Y  &&  kFCurve_Z)
							{
								float lKeyX,lKeyY,lKeyZ;
								//Con::printf("GOT A KFCURVE!  %d",kFCurve_X->KeyGetCount());
								for (U32 i=0;i<kFCurve_X->KeyGetCount();i++)
								{//(Have to assume they all have same framecount.)
									lKeyX = static_cast<float>(kFCurve_X->KeyGetValue(i));
									lKeyY = static_cast<float>(kFCurve_Y->KeyGetValue(i));
									lKeyZ = static_cast<float>(kFCurve_Z->KeyGetValue(i));
									//nodeTransCache[mFbxLimbNodes][i] = new Point3F(lKeyX,lKeyY,lKeyZ);

									//addNodeTranslation(Point3F(lKeyX,lKeyY,lKeyZ),false);
									appNode->nodeTranslates.increment();
									Point3F pos(-lKeyX,lKeyZ,lKeyY);
									pos *= mFbxScale;
									appNode->nodeTranslates.last() = pos;
									//Con::printf("    key %d:  value %f  %f  %f  pos  %f  %f  %f",i,lKeyX,lKeyY,lKeyZ,pos.x,pos.y,pos.z);
								}
							}
						}
					}
					else if (!strcmp(prop.GetLabel().Buffer(),"Lcl Rotation")) 
					{
						KFbxAnimCurveNode *curveNode = NULL;
						KFCurve *kFCurve_X,*kFCurve_Y,*kFCurve_Z;
						KFbxAnimCurve *kCurve_X,*kCurve_Y,*kCurve_Z;
						curveNode = prop.GetCurveNode();
						if (curveNode)
						{
							S32 channelsCnt = curveNode->GetChannelsCount();
							S32 curveCnt = curveNode->GetCurveCount(0);

							if (curveCnt==0)
							{
								//Whoops, got an animstack, but no curve nodes.
								if (appSequences.size()>0)//FIX, make sure to only delete THIS sequence,
									appSequences.decrement();//if we get multiples going on.
								goto NEXT;//SORRY!  if/break is a pita...
							}

							kCurve_X = curveNode->GetCurve(0,0);
							kCurve_Y = curveNode->GetCurve(1,0);
							kCurve_Z = curveNode->GetCurve(2,0);
							
							if (!(kCurve_X && kCurve_Y && kCurve_Z))
								goto NEXT;//SORRY!  if/break is a pita...

							kFCurve_X = kCurve_X->GetKFCurve();
							kFCurve_Y = kCurve_Y->GetKFCurve();
							kFCurve_Z = kCurve_Z->GetKFCurve();

							//KFbxProperty T_channel = curveNode->GetChannel("T");//??? This shit is WEIRD.
							//curveNode->GetAnimationInterval(...);
							//Con::printf("GOT A Rotation CURVE NODE:  property %s   channels %d   one %s  two %s curveCnt %d",
							//	prop.GetLabel().Buffer(),channelsCnt,curveNode->GetChannelName(0).Buffer(),
							//	curveNode->GetChannelName(1).Buffer(),curveCnt);

							if (kFCurve_X  &&  kFCurve_Y  &&  kFCurve_Z)
							{
								float lKeyX,lKeyY,lKeyZ;
								//Con::printf("GOT A KFCURVE!  %d",kFCurve_X->KeyGetCount());
								
								if (kFCurve_X->KeyGetCount() > appSeq->seqKeyframes)
								{
									appSeq->seqKeyframes = kFCurve_X->KeyGetCount();//DANGER, now are we going to be reading out of range?
									Con::printf("Node  %s  Is setting a new keyframe count:  %d",curveNode->GetName(),appSeq->seqKeyframes);
								}

								for (U32 i=0;i<kFCurve_X->KeyGetCount();i++)
								{//(Have to assume they all have same framecount.)
									lKeyX = static_cast<float>(kFCurve_X->KeyGetValue(i));
									lKeyY = static_cast<float>(kFCurve_Y->KeyGetValue(i));
									lKeyZ = static_cast<float>(kFCurve_Z->KeyGetValue(i));
									//nodeRotCache[mFbxLimbNodes][i] = new QuatF(EulerF(lKeyX,lKeyY,lKeyZ));
									//addNodeRotation(QuatF(EulerF(lKeyX,lKeyY,lKeyZ)),false);

									appNode->nodeRotates.increment();

									//EulerF eul(-lKeyX,lKeyZ,lKeyY);//_maybe_ flip it like this???
									//EulerF eul(-lKeyX * (M_PI/180.0),lKeyZ * (M_PI/180.0),lKeyY * (M_PI/180.0));
									//EulerF eul(lKeyX * (M_PI/180.0),lKeyY * (M_PI/180.0),lKeyZ * (M_PI/180.0));
									EulerF eul(lKeyX * (M_PI/180.0),-lKeyZ * (M_PI/180.0),-lKeyY * (M_PI/180.0));
									//EulerF eul(-lKeyX * (M_PI/180.0),lKeyZ * (M_PI/180.0),-lKeyY * (M_PI/180.0));
									//EulerF eul(lKeyX * (M_PI/180.0),lKeyZ * (M_PI/180.0),-lKeyY * (M_PI/180.0));
									//EulerF eul(-lKeyX * (M_PI/180.0),-lKeyZ * (M_PI/180.0),-lKeyY * (M_PI/180.0));
									//appNode->nodeRotates.last().set(eul);

									EulerAngles ea;
									HMatrix	hMatrix;
									MatrixF m1;
									Point4F row0,row1,row2;

									//ea = Eul_(eul.x,eul.y,eul.z,EulOrdXYZs);
									//ea = Eul_(eul.x,eul.z,eul.y,EulOrdXZYs);
									ea = Eul_(eul.y,eul.z,eul.x,EulOrdYZXs);
									//ea = Eul_(eul.y,eul.x,eul.z,EulOrdYXZs);
									//ea = Eul_(eul.z,eul.x,eul.y,EulOrdZXYs);
									//ea = Eul_(eul.z,eul.y,eul.x,EulOrdZYXs);
									Eul_ToHMatrix( ea,hMatrix);

									row0.x = hMatrix[0][0]; row0.y = hMatrix[1][0]; row0.z = hMatrix[2][0];
									row1.x = hMatrix[0][1]; row1.y = hMatrix[1][1]; row1.z = hMatrix[2][1];
									row2.x = hMatrix[0][2]; row2.y = hMatrix[1][2]; row2.z = hMatrix[2][2];
									
									m1.setRow(0,row0);
									m1.setRow(1,row1);
									m1.setRow(2,row2);

									//appNode->nodeRotates.last().set(m0);
									appNode->nodeRotates.last() = m1;//.set(eul);
									//Con::printf("    key %d:  value %f  %f  %f",i,lKeyX,lKeyY,lKeyZ);
								}
							}
						}
					}
NEXT:
					prop = member->GetNextProperty(prop);
				}				
				//Maybe??  This should set up our nodeTransforms for use later when making sequences.
				for (U32 i=0;i<appNode->nodeRotates.size();i++)
				{
					appNode->nodeTransforms.increment();
					MatrixF *m = &(appNode->nodeTransforms.last());
					appNode->nodeRotates[i].setMatrix(m);
				}
				for (U32 i=0;i<appNode->nodeTranslates.size();i++)
				{					
					MatrixF *m = &(appNode->nodeTransforms[i]);
					m->setPosition(appNode->nodeTranslates[i]);
				}
			}
		}
	}
}


									//MORE FAIL...
									//MatrixF m1(EulerF(eul.x,0.0,0.0));
									//MatrixF m2(EulerF(0.0,eul.y,0.0));
									//MatrixF m3(EulerF(0.0,0.0,eul.z));
									//MatrixF m0(true);
									//m0.mul(m2);
									//m0.mul(m3);
									//m0.mul(m1);


/// Add Fbx materials to materials.cs
//void FbxShapeLoader::updateMaterialsScript(const Torque::Path &path, bool copyTextures = false)
//{
//#ifdef DAE2DTS_TOOL
//   if (!FbxUtils::getOptions().forceUpdateMaterials)
//      return;
//#endif
//
//   // First see what materials we need to add... if one already
//   // exists then we can ignore it.
//   Vector<FbxAppMaterial*> materials;
//   for ( U32 iMat = 0; iMat < AppMesh::appMaterials.size(); iMat++ )
//   {
//      FbxAppMaterial *mat = dynamic_cast<FbxAppMaterial*>( AppMesh::appMaterials[iMat] );
//      if ( mat && ( FbxUtils::getOptions().forceUpdateMaterials ||
//             MATMGR->getMapEntry( mat->getName() ).isEmpty() ) )
//         materials.push_back( mat );      
//   }
//
//   if ( materials.empty() )
//      return;
//
//   Torque::Path scriptPath(path);
//   scriptPath.setFileName("materials");
//   scriptPath.setExtension("cs");
//
//   // Read the current script (if any) into memory
//   FileObject f;
//   f.readMemory(scriptPath.getFullPath());
//
//   FileStream stream;
//   if (stream.open(scriptPath, Torque::FS::File::Write)) {
//
//      String shapeName = TSShapeLoader::getShapePath().getFullFileName();
//      const char *beginMsg = avar("//--- %s MATERIALS BEGIN ---", shapeName.c_str());
//
//      // Write existing file contents up to start of auto-generated materials
//      while(!f.isEOF()) {
//         const char *buffer = (const char *)f.readLine();
//         if (dStricmp(buffer, beginMsg) == 0)
//            break;
//         stream.writeLine((const U8*)buffer);
//      }
//
//      // Write new auto-generated materials
//      stream.writeLine((const U8*)beginMsg);
//      for (int iMat = 0; iMat < materials.size(); iMat++)
//      {
//         if (copyTextures)
//         {
//            // If importing a sketchup file, the paths will point inside the KMZ so we need to cache them.
//            copySketchupTexture(path, materials[iMat]->diffuseMap);
//            copySketchupTexture(path, materials[iMat]->normalMap);
//            copySketchupTexture(path, materials[iMat]->specularMap);
//         }
//         materials[iMat]->write(stream);
//      }
//
//      const char *endMsg = avar("//--- %s MATERIALS END ---", shapeName.c_str());
//      stream.writeLine((const U8*)endMsg);
//      stream.writeLine((const U8*)"");
//
//      // Write existing file contents after end of auto-generated materials
//      while (!f.isEOF()) {
//         const char *buffer = (const char *) f.readLine();
//         if (dStricmp(buffer, endMsg) == 0)
//            break;
//      }
//
//      // Want at least one blank line after the autogen block, but need to
//      // be careful not to write it twice, or another blank will be added
//      // each time the file is written!
//      if (!f.isEOF()) {
//         const char *buffer = (const char *) f.readLine();
//         if (!dStrEqual(buffer, ""))
//            stream.writeLine((const U8*)buffer);
//      }
//      while (!f.isEOF()) {
//         const char *buffer = (const char *) f.readLine();
//         stream.writeLine((const U8*)buffer);
//      }
//      f.close();
//      stream.close();
//
//      // Execute the new script to apply the material settings
//      if (f.readMemory(scriptPath.getFullPath()))
//      {
//         String instantGroup = Con::getVariable("InstantGroup");
//         Con::setIntVariable("InstantGroup", RootGroupId);
//         Con::evaluate((const char*)f.buffer(), false, scriptPath.getFullPath());
//         Con::setVariable("InstantGroup", instantGroup.c_str());
//      }
//   }
//}





	/*

//#ifndef DAE2DTS_TOOL
//   // Generate the cached filename
//   Torque::Path cachedPath(path);
//   cachedPath.setExtension("cached.dts");
//
//   // Check if an up-to-date cached DTS version of this file exists, and
//   // if so, use that instead.
//   if (ColladaShapeLoader::canLoadCachedDTS(path))
//   {
//      FileStream cachedStream;
//      cachedStream.open(cachedPath.getFullPath(), Torque::FS::File::Read);
//      if (cachedStream.getStatus() == Stream::Ok)
//      {
//         TSShape *shape = new TSShape;
//         bool readSuccess = shape->read(&cachedStream);
//         cachedStream.close();
//
//         if (readSuccess)
//         {
//         #ifdef TORQUE_DEBUG
//            Con::printf("Loaded cached Collada shape from %s", cachedPath.getFullPath().c_str());
//         #endif
//            return shape;
//         }
//         else
//            delete shape;
//      }
//
//      Con::warnf("Failed to load cached COLLADA shape from %s", cachedPath.getFullPath().c_str());
//   }
//#endif // DAE2DTS_TOOL

   if (!Torque::FS::IsFile(path))
   {
      // file does not exist, bail.
		Con::errorf("Could not find the file.");
      return NULL;
   }

//#ifdef DAE2DTS_TOOL
   //ColladaUtils::ImportOptions cmdLineOptions = ColladaUtils::getOptions();
//#endif

   // Allow TSShapeConstructor object to override properties
   //ColladaUtils::getOptions().reset();
   TSShapeConstructor* tscon = TSShapeConstructor::findShapeConstructor(path.getFullPath());
   if (tscon)
   {
		Con::printf("Found a shape constructor.");
      //ColladaUtils::getOptions() = tscon->mOptions;

   }

   // Check if this is a Sketchup file (.kmz) and if so, mount the zip filesystem
   // and get the path to the DAE file.
   String mountPoint;
   Torque::Path fbxPath;

   // Load Collada model and convert to 3space
   TSShape* tss = 0;
	Con::printf("Created a TSShape pointer!!!!!!!!!");
	//CHANGE: for FBX, we are going to make the loader exist on its own without
	// telling it what to load yet, so it can get its sdk manager instantiated.
	FbxShapeLoader loader;

//   domCOLLADA* root = FbxShapeLoader::getDomCOLLADA(daePath);
//   if (root)
//   {
//      FbxShapeLoader loader(root);
//      tss = loader.generateShape(daePath);
//		Con::printf("shape generated! details: %d",tss->details.size());
//      if (tss)
//      {
//			//tss->fixUpAxis(1);
//			tss->examineShape();
//
//#ifndef FBX2DTS_TOOL
//         // Cache the FBX model to a DTS file for faster loading next time.
//         FileStream dtsStream;
//         if (dtsStream.open(cachedPath.getFullPath(), Torque::FS::File::Write))
//         {
//            Con::printf("Writing cached FBX shape to %s", cachedPath.getFullPath().c_str());
//            tss->write(&dtsStream);
//         }
//#endif // FBX2DTS_TOOL
//
//         // Add fbx materials to materials.cs
//         updateMaterialsScript(path, isSketchup);
//      }
//   }
//
//   // Close progress dialog
//   TSShapeLoader::updateProgress(TSShapeLoader::Load_Complete, "Import complete");

   return tss;
	*/
//}


// Add a thumbnail to the take
//void FbxShapeLoader::AddThumbnailToTake(KString& pTakeName, int pThumbnailIndex)
//{
//    KFbxTakeInfo lTakeInfo;
//    KFbxThumbnail* lThumbnail = KFbxThumbnail::Create(mSdkManager,"");
//
//    lThumbnail->SetDataFormat(KFbxThumbnail::eRGB_24);
//    lThumbnail->SetSize(KFbxThumbnail::e64x64);
//    lThumbnail->SetThumbnailImage(pThumbnailIndex == 0 ? cTakeOneThumbnail : cTakeTwoThumbnail);
//
//    lTakeInfo.mName = pTakeName;
//    lTakeInfo.mDescription = KString("Take at index ") + pThumbnailIndex;
//    lTakeInfo.SetTakeThumbnail(lThumbnail);
//
//    mScene->SetTakeInfo(lTakeInfo);
//}
//



/*
///////////////////////////////////////////

static DAE sDAE;                 // Fbx model database (holds the last loaded file)
static Torque::Path sLastPath;   // Path of the last loaded Fbx file
static FileTime sLastModTime;    // Modification time of the last loaded Fbx file

//-----------------------------------------------------------------------------
// Custom warning/error message handler
class myErrorHandler : public daeErrorHandler
{
	void handleError( daeString msg )
   {
      Con::errorf("Error: %s", msg);
   }

	void handleWarning( daeString msg )
   {
      Con::errorf("Warning: %s", msg);
   }
} sErrorHandler;

//-----------------------------------------------------------------------------

FbxShapeLoader::FbxShapeLoader(domFbx* _root)
   : root(_root)
{
   // Extract the global scale and up_axis from the top level <asset> element,
   F32 unit = 1.0f;
   domUpAxisType upAxis = UPAXISTYPE_Z_UP;
   if (root->getAsset()) {
      if (root->getAsset()->getUnit())
         unit = root->getAsset()->getUnit()->getMeter();
      if (root->getAsset()->getUp_axis())
         upAxis = root->getAsset()->getUp_axis()->getValue();
   }

   // Set import options (if they are not set to override)
   if (FbxUtils::getOptions().unit <= 0.0f)
      FbxUtils::getOptions().unit = unit;

   if (FbxUtils::getOptions().upAxis == UPAXISTYPE_COUNT)
      FbxUtils::getOptions().upAxis = upAxis;
}

FbxShapeLoader::~FbxShapeLoader()
{
   // Delete all of the animation channels
   for (int iAnim = 0; iAnim < animations.size(); iAnim++) {
      for (int iChannel = 0; iChannel < animations[iAnim]->size(); iChannel++)
         delete (*animations[iAnim])[iChannel];
      delete animations[iAnim];
   }
   animations.clear();
}

void FbxShapeLoader::processAnimation(const domAnimation* anim, F32& maxEndTime, F32& minFrameTime)
{
   const char* sRGBANames[] =   { ".R", ".G", ".B", ".A", "" };
   const char* sXYZNames[] =    { ".X", ".Y", ".Z", "" };
   const char* sXYZANames[] =   { ".X", ".Y", ".Z", ".ANGLE" };
   const char* sLOOKATNames[] = { ".POSITIONX", ".POSITIONY", ".POSITIONZ", ".TARGETX", ".TARGETY", ".TARGETZ", ".UPX", ".UPY", ".UPZ", "" };
   const char* sSKEWNames[] =   { ".ROTATEX", ".ROTATEY", ".ROTATEZ", ".AROUNDX", ".AROUNDY", ".AROUNDZ", ".ANGLE", "" };
   const char* sNullNames[] =   { "" };

   for (int iChannel = 0; iChannel < anim->getChannel_array().getCount(); iChannel++) {

      // Get the animation elements: <channel>, <sampler>
      domChannel* channel = anim->getChannel_array()[iChannel];
      domSampler* sampler = daeSafeCast<domSampler>(channel->getSource().getElement());
      if (!sampler)
         continue;

      // Find the animation channel target
      daeSIDResolver resolver(channel, channel->getTarget());
      daeElement* target = resolver.getElement();
      if (!target) {
         daeErrorHandler::get()->handleWarning(avar("Failed to resolve animation "
            "target: %s", channel->getTarget()));
         continue;
      }

      //// If the target is a <source>, point it at the array instead
      //// @todo:Only support targeting float arrays for now...
      //if (target->getElementType() == Fbx_TYPE::SOURCE)
      //{
      //   domSource* source = daeSafeCast<domSource>(target);
      //   if (source->getFloat_array())
      //      target = source->getFloat_array();
      //}

      // Get the target's animation channels (create them if not already)
      if (!AnimData::getAnimChannels(target)) {
         animations.push_back(new AnimChannels(target));
      }
      AnimChannels* targetChannels = AnimData::getAnimChannels(target);

      // Add a new animation channel to the target
      targetChannels->push_back(new AnimData());
      channel->setUserData(targetChannels->last());
      AnimData& data = *targetChannels->last();

      for (int iInput = 0; iInput < sampler->getInput_array().getCount(); iInput++) {

         const domInputLocal* input = sampler->getInput_array()[iInput];
         const domSource* source = daeSafeCast<domSource>(input->getSource().getElement());
         if (!source)
            continue;

         // @todo:don't care about the input param names for now. Could
         // validate against the target type....
         if (dStrEqual(input->getSemantic(), "INPUT")) {
            data.input.initFromSource(source);
            // Adjust the maximum sequence end time
            maxEndTime = getMax(maxEndTime, data.input.getFloatValue((S32)data.input.size()-1));

            // Detect the frame rate (minimum time between keyframes)
            for (S32 iFrame = 1; iFrame < data.input.size(); iFrame++)
            {
               F32 delta = data.input.getFloatValue( iFrame ) - data.input.getFloatValue( iFrame-1 );
               if ( delta < 0 )
               {
                  daeErrorHandler::get()->handleError(avar("<animation> INPUT '%s' "
                     "has non-monotonic keys. Animation is unlikely to be imported correctly.", source->getID()));
                  break;
               }
               minFrameTime = getMin( minFrameTime, delta );
            }
         }
         else if (dStrEqual(input->getSemantic(), "OUTPUT"))
            data.output.initFromSource(source);
         else if (dStrEqual(input->getSemantic(), "IN_TANGENT"))
            data.inTangent.initFromSource(source);
         else if (dStrEqual(input->getSemantic(), "OUT_TANGENT"))
            data.outTangent.initFromSource(source);
         else if (dStrEqual(input->getSemantic(), "INTERPOLATION"))
            data.interpolation.initFromSource(source);
      }

      // Ignore empty animations
      if (data.input.size() == 0) {
         channel->setUserData(0);
         delete targetChannels->last();
         targetChannels->pop_back();
         continue;
      }

      // Determine the number and offset the elements of the target value
      // targeted by this animation
      switch (target->getElementType()) {
         case Fbx_TYPE::COLOR:        data.parseTargetString(channel->getTarget(), 4, sRGBANames);   break;
         case Fbx_TYPE::TRANSLATE:    data.parseTargetString(channel->getTarget(), 3, sXYZNames);    break;
         case Fbx_TYPE::ROTATE:       data.parseTargetString(channel->getTarget(), 4, sXYZANames);   break;
         case Fbx_TYPE::SCALE:        data.parseTargetString(channel->getTarget(), 3, sXYZNames);    break;
         case Fbx_TYPE::LOOKAT:       data.parseTargetString(channel->getTarget(), 3, sLOOKATNames); break;
         case Fbx_TYPE::SKEW:         data.parseTargetString(channel->getTarget(), 3, sSKEWNames);   break;
         case Fbx_TYPE::MATRIX:       data.parseTargetString(channel->getTarget(), 16, sNullNames);  break;
         case Fbx_TYPE::FLOAT_ARRAY:  data.parseTargetString(channel->getTarget(), daeSafeCast<domFloat_array>(target)->getCount(), sNullNames); break;
         default:                         data.parseTargetString(channel->getTarget(), 1, sNullNames);   break;
      }
   }

   // Process child animations
   for (int iAnim = 0; iAnim < anim->getAnimation_array().getCount(); iAnim++)
      processAnimation(anim->getAnimation_array()[iAnim], maxEndTime, minFrameTime);
}

void FbxShapeLoader::enumerateScene()
{
   // Get animation clips
   Vector<const domAnimation_clip*> animationClips;
   for (int iClipLib = 0; iClipLib < root->getLibrary_animation_clips_array().getCount(); iClipLib++) {
      const domLibrary_animation_clips* libraryClips = root->getLibrary_animation_clips_array()[iClipLib];
      for (int iClip = 0; iClip < libraryClips->getAnimation_clip_array().getCount(); iClip++)
         appSequences.push_back(new FbxAppSequence(libraryClips->getAnimation_clip_array()[iClip]));
   }

   // Process all animations => this attaches animation channels to the targeted
   // Fbx elements, and determines the length of the sequence if it is not
   // already specified in the Fbx <animation_clip> element
   for (int iSeq = 0; iSeq < appSequences.size(); iSeq++) {
      FbxAppSequence* appSeq = dynamic_cast<FbxAppSequence*>(appSequences[iSeq]);
      F32 maxEndTime = 0;
      F32 minFrameTime = 1000.0f;
      for (int iAnim = 0; iAnim < appSeq->getClip()->getInstance_animation_array().getCount(); iAnim++) {
         domAnimation* anim = daeSafeCast<domAnimation>(appSeq->getClip()->getInstance_animation_array()[iAnim]->getUrl().getElement());
         if (anim)
            processAnimation(anim, maxEndTime, minFrameTime);
      }
      if (appSeq->getEnd() == 0)
         appSeq->setEnd(maxEndTime);

      // Fbx animations can be stored as sampled frames or true keyframes. For
      // sampled frames, use the same frame rate as the DAE file. For true keyframes,
      // resample at a fixed frame rate.
      appSeq->delta = mClampF(minFrameTime, 1.0f/TSShapeLoader::MaxFrameRate, 1.0f/TSShapeLoader::MinFrameRate);
   }

   // First grab all of the top-level nodes
   Vector<domNode*> sceneNodes;
   for (int iSceneLib = 0; iSceneLib < root->getLibrary_visual_scenes_array().getCount(); iSceneLib++) {
      const domLibrary_visual_scenes* libScenes = root->getLibrary_visual_scenes_array()[iSceneLib];
      for (int iScene = 0; iScene < libScenes->getVisual_scene_array().getCount(); iScene++) {
         const domVisual_scene* visualScene = libScenes->getVisual_scene_array()[iScene];
         for (int iNode = 0; iNode < visualScene->getNode_array().getCount(); iNode++)
            sceneNodes.push_back(visualScene->getNode_array()[iNode]);
      }
   }

   // Set LOD option
   bool singleDetail = true;
   switch (FbxUtils::getOptions().lodType)
   {
      case FbxUtils::ImportOptions::DetectDTS:
         // Check for a baseXX->startXX hierarchy at the top-level, if we find
         // one, use trailing numbers for LOD, otherwise use a single size
         for (int iNode = 0; singleDetail && (iNode < sceneNodes.size()); iNode++) {
            domNode* node = sceneNodes[iNode];
            if (dStrStartsWith(_GetNameOrId(node), "base")) {
               for (int iChild = 0; iChild < node->getNode_array().getCount(); iChild++) {
                  domNode* child = node->getNode_array()[iChild];
                  if (dStrStartsWith(_GetNameOrId(child), "start")) {
                     singleDetail = false;
                     break;
                  }
               }
            }
         }
         break;

      case FbxUtils::ImportOptions::SingleSize:
         singleDetail = true;
         break;

      case FbxUtils::ImportOptions::TrailingNumber:
         singleDetail = false;
         break;
         
      default:
         break;
   }

   FbxAppMesh::fixDetailSize( singleDetail, FbxUtils::getOptions().singleDetailSize );

   // Process the top level nodes
   for (S32 iNode = 0; iNode < sceneNodes.size(); iNode++) {
      FbxAppNode* node = new FbxAppNode(sceneNodes[iNode], 0);
      if (!processNode(node))
         delete node;
   }

   // Make sure that the scene has a bounds node (for getting the root scene transform)
   if (!boundsNode)
   {
      domVisual_scene* visualScene = root->getLibrary_visual_scenes_array()[0]->getVisual_scene_array()[0];
      domNode* dombounds = daeSafeCast<domNode>( visualScene->createAndPlace( "node" ) );
      dombounds->setName( "bounds" );
      FbxAppNode *appBounds = new FbxAppNode(dombounds, 0);
      if (!processNode(appBounds))
         delete appBounds;
   }
}

bool FbxShapeLoader::ignoreNode(const String& name)
{
   if (FindMatch::isMatchMultipleExprs(FbxUtils::getOptions().alwaysImport, name, false))
      return false;
   else
      return FindMatch::isMatchMultipleExprs(FbxUtils::getOptions().neverImport, name, false);
}

bool FbxShapeLoader::ignoreMesh(const String& name)
{
   if (FindMatch::isMatchMultipleExprs(FbxUtils::getOptions().alwaysImportMesh, name, false))
      return false;
   else
      return FindMatch::isMatchMultipleExprs(FbxUtils::getOptions().neverImportMesh, name, false);
}

void FbxShapeLoader::computeBounds(Box3F& bounds)
{
   TSShapeLoader::computeBounds(bounds);

   // Check if the model origin needs adjusting
   if ( bounds.isValidBox() &&
       (FbxUtils::getOptions().adjustCenter ||
        FbxUtils::getOptions().adjustFloor) )
   {
      // Compute shape offset
      Point3F shapeOffset = Point3F::Zero;
      if ( FbxUtils::getOptions().adjustCenter )
      {
         bounds.getCenter( &shapeOffset );
         shapeOffset = -shapeOffset;
      }
      if ( FbxUtils::getOptions().adjustFloor )
         shapeOffset.z = -bounds.minExtents.z;

      // Adjust bounds
      bounds.minExtents += shapeOffset;
      bounds.maxExtents += shapeOffset;

      // Now adjust all positions for root level nodes (nodes with no parent)
      for (S32 iNode = 0; iNode < shape->nodes.size(); iNode++)
      {
         if ( !appNodes[iNode]->isParentRoot() )
            continue;

         // Adjust default translation
         shape->defaultTranslations[iNode] += shapeOffset;

         // Adjust animated translations
         for (S32 iSeq = 0; iSeq < shape->sequences.size(); iSeq++)
         {
            const TSShape::Sequence& seq = shape->sequences[iSeq];
            if ( seq.translationMatters.test(iNode) )
            {
               for (S32 iFrame = 0; iFrame < seq.numKeyframes; iFrame++)
               {
                  S32 index = seq.baseTranslation + seq.translationMatters.count(iNode)*seq.numKeyframes + iFrame;
                  shape->nodeTranslations[index] += shapeOffset;
               }
            }
         }
      }
   }
}

//-----------------------------------------------------------------------------
/// Find the file extension for an extensionless texture
String findTextureExtension(const Torque::Path &texPath)
{
   Torque::Path path(texPath);
   for(S32 i = 0;i < GBitmap::sRegistrations.size();++i)
   {
      GBitmap::Registration &reg = GBitmap::sRegistrations[i];
      for(S32 j = 0;j < reg.extensions.size();++j)
      {
         path.setExtension(reg.extensions[j]);
         if (Torque::FS::IsFile(path))
            return path.getExtension();
      }
   }

   return String();
}

//-----------------------------------------------------------------------------
/// Copy a texture from a KMZ to a cache. Note that the texture filename is modified
void copySketchupTexture(const Torque::Path &path, String &textureFilename)
{
   if (textureFilename.isEmpty())
      return;

   Torque::Path texturePath(textureFilename);
   texturePath.setExtension(findTextureExtension(texturePath));

   String cachedTexFilename = String::ToString("%s_%s.cached",
      TSShapeLoader::getShapePath().getFileName().c_str(), texturePath.getFileName().c_str());

   Torque::Path cachedTexPath;
   cachedTexPath.setRoot(path.getRoot());
   cachedTexPath.setPath(path.getPath());
   cachedTexPath.setFileName(cachedTexFilename);
   cachedTexPath.setExtension(texturePath.getExtension());

   FileStream *source;
   FileStream *dest;
   if ((source = FileStream::createAndOpen(texturePath.getFullPath(), Torque::FS::File::Read)) == NULL)
      return;

   if ((dest = FileStream::createAndOpen(cachedTexPath.getFullPath(), Torque::FS::File::Write)) == NULL)
   {
      delete source;
      return;
   }

   dest->copyFrom(source);

   delete dest;
   delete source;

   // Update the filename in the material
   cachedTexPath.setExtension("");
   textureFilename = cachedTexPath.getFullPath();
}

//-----------------------------------------------------------------------------
/// Add Fbx materials to materials.cs
void updateMaterialsScript(const Torque::Path &path, bool copyTextures = false)
{
#ifdef DAE2DTS_TOOL
   if (!FbxUtils::getOptions().forceUpdateMaterials)
      return;
#endif

   // First see what materials we need to add... if one already
   // exists then we can ignore it.
   Vector<FbxAppMaterial*> materials;
   for ( U32 iMat = 0; iMat < AppMesh::appMaterials.size(); iMat++ )
   {
      FbxAppMaterial *mat = dynamic_cast<FbxAppMaterial*>( AppMesh::appMaterials[iMat] );
      if ( mat && ( FbxUtils::getOptions().forceUpdateMaterials ||
             MATMGR->getMapEntry( mat->getName() ).isEmpty() ) )
         materials.push_back( mat );      
   }

   if ( materials.empty() )
      return;

   Torque::Path scriptPath(path);
   scriptPath.setFileName("materials");
   scriptPath.setExtension("cs");

   // Read the current script (if any) into memory
   FileObject f;
   f.readMemory(scriptPath.getFullPath());

   FileStream stream;
   if (stream.open(scriptPath, Torque::FS::File::Write)) {

      String shapeName = TSShapeLoader::getShapePath().getFullFileName();
      const char *beginMsg = avar("//--- %s MATERIALS BEGIN ---", shapeName.c_str());

      // Write existing file contents up to start of auto-generated materials
      while(!f.isEOF()) {
         const char *buffer = (const char *)f.readLine();
         if (dStricmp(buffer, beginMsg) == 0)
            break;
         stream.writeLine((const U8*)buffer);
      }

      // Write new auto-generated materials
      stream.writeLine((const U8*)beginMsg);
      for (int iMat = 0; iMat < materials.size(); iMat++)
      {
         if (copyTextures)
         {
            // If importing a sketchup file, the paths will point inside the KMZ so we need to cache them.
            copySketchupTexture(path, materials[iMat]->diffuseMap);
            copySketchupTexture(path, materials[iMat]->normalMap);
            copySketchupTexture(path, materials[iMat]->specularMap);
         }
         materials[iMat]->write(stream);
      }

      const char *endMsg = avar("//--- %s MATERIALS END ---", shapeName.c_str());
      stream.writeLine((const U8*)endMsg);
      stream.writeLine((const U8*)"");

      // Write existing file contents after end of auto-generated materials
      while (!f.isEOF()) {
         const char *buffer = (const char *) f.readLine();
         if (dStricmp(buffer, endMsg) == 0)
            break;
      }

      // Want at least one blank line after the autogen block, but need to
      // be careful not to write it twice, or another blank will be added
      // each time the file is written!
      if (!f.isEOF()) {
         const char *buffer = (const char *) f.readLine();
         if (!dStrEqual(buffer, ""))
            stream.writeLine((const U8*)buffer);
      }
      while (!f.isEOF()) {
         const char *buffer = (const char *) f.readLine();
         stream.writeLine((const U8*)buffer);
      }
      f.close();
      stream.close();

      // Execute the new script to apply the material settings
      if (f.readMemory(scriptPath.getFullPath()))
      {
         String instantGroup = Con::getVariable("InstantGroup");
         Con::setIntVariable("InstantGroup", RootGroupId);
         Con::evaluate((const char*)f.buffer(), false, scriptPath.getFullPath());
         Con::setVariable("InstantGroup", instantGroup.c_str());
      }
   }
}

bool FbxShapeLoader::checkAndMountSketchup(const Torque::Path& path, String& mountPoint, Torque::Path& daePath)
{
   bool isSketchup = path.getExtension().equal("kmz", String::NoCase);
   if (isSketchup)
   {
      // Mount the zip so files can be found (it will be unmounted before we return)
      mountPoint = String("sketchup_") + path.getFileName();
      String zipPath = path.getFullPath();
      if (!Torque::FS::Mount(mountPoint, new Torque::ZipFileSystem(zipPath)))
         return false;

      Vector<String> daeFiles;
      Torque::Path findPath;
      findPath.setRoot(mountPoint);
      S32 results = Torque::FS::FindByPattern(findPath, "*.dae", true, daeFiles);
      if (results == 0 || daeFiles.size() == 0)
      {
         Torque::FS::Unmount(mountPoint);
         return false;
      }

      daePath = daeFiles[0];
   }
   else
   {
      daePath = path;
   }

   return isSketchup;
}

//-----------------------------------------------------------------------------
/// Get the root Fbx DOM element for the given DAE file
domFbx* FbxShapeLoader::getDomFbx(const Torque::Path& path)
{
   daeErrorHandler::setErrorHandler(&sErrorHandler);

   TSShapeLoader::updateProgress(TSShapeLoader::Load_ReadFile, path.getFullFileName().c_str());

   // Check if we can use the last loaded file
   FileTime daeModifyTime;
   if (Platform::getFileTimes(path.getFullPath(), NULL, &daeModifyTime))
   {
      if ((path == sLastPath) && (Platform::compareFileTimes(sLastModTime, daeModifyTime) >= 0))
         return sDAE.getRoot(path.getFullPath().c_str());
   }

   sDAE.clear();
   sDAE.setBaseURI("");

   TSShapeLoader::updateProgress(TSShapeLoader::Load_ParseFile, "Parsing XML...");
   domFbx* root = readFbxFile(path.getFullPath());
   if (!root)
   {
      TSShapeLoader::updateProgress(TSShapeLoader::Load_Complete, "Import failed");
      sDAE.clear();
      return NULL;
   }

   sLastPath = path;
   sLastModTime = daeModifyTime;

   return root;
}

domFbx* FbxShapeLoader::readFbxFile(const String& path)
{
   // Check if this file is already loaded into the database
   domFbx* root = sDAE.getRoot(path.c_str());
   if (root)
      return root;

   // Load the Fbx file into memory
   FileObject fo;
   if (!fo.readMemory(path))
   {
      daeErrorHandler::get()->handleError(avar("Could not read %s into memory", path.c_str()));
      return NULL;
   }

   root = sDAE.openFromMemory(path.c_str(), (const char*)fo.buffer());
   if (!root || !root->getLibrary_visual_scenes_array().getCount()) {
      daeErrorHandler::get()->handleError(avar("Could not parse %s", path.c_str()));
      return NULL;
   }

   // Fixup issues in the model
   FbxUtils::applyConditioners(root);

   // Recursively load external DAE references
   TSShapeLoader::updateProgress(TSShapeLoader::Load_ExternalRefs, "Loading external references...");
   for (S32 iRef = 0; iRef < root->getDocument()->getReferencedDocuments().getCount(); iRef++) {
      String refPath = (daeString)root->getDocument()->getReferencedDocuments()[iRef];
      if (refPath.endsWith(".dae") && !readFbxFile(refPath))
         daeErrorHandler::get()->handleError(avar("Failed to load external reference: %s", refPath.c_str()));
   }
   return root;
}


*///
//void FbxShapeLoader::findSequences(KFbxObject *member)
//{
//	//KFbxMesh *kMesh = NULL;
//	//KFbxNode *kNode = NULL;
//	//KFbxCluster *kCluster = NULL;
//	//KFbxSkin *kSkin = NULL;
//	KFbxAnimStack *kAnimStack = NULL;
//	KFbxAnimLayer *kAnimLayer = NULL;
//	
//	kAnimStack = dynamic_cast<KFbxAnimStack *>(member);
//	if (kAnimStack)
//	{
//		mFbxSequences++;//maybe irrelevant, given appSequences.size()?
//		appSequences.push_back(new FbxAppSequence(kAnimStack));
//
//		KTimeSpan timeSpan = kAnimStack->GetLocalTimeSpan();
//		KTime time = timeSpan.GetDuration();
//		S32 animMembers = kAnimStack->GetMemberCount();
//		S32 srcPropCnt = member->GetSrcPropertyCount();
//		S32 dstPropCnt = member->GetDstPropertyCount();
//
//		Con::printf("Found an anim stack:  initialName %s  duration (MS) %d, members %d, srcPropCnt %d, dstPropCnt %d",
//			member->GetInitialName(),time.GetMilliSeconds(),animMembers,srcPropCnt,dstPropCnt);
//			
//		//KFbxProperty prop = member->GetFirstProperty();
//		//while (prop.IsValid())
//		//{
//		//	Con::printf("  property %s   name %s   type %s",prop.GetLabel().Buffer(),
//		//		prop.GetName().Buffer(),prop.GetPropertyDataType().GetName());
//		//	prop=member->GetNextProperty(prop);
//		//}
//
//	}
//
//	kAnimLayer = dynamic_cast<KFbxAnimLayer *>(member);
//	if (kAnimLayer)
//	{
//		S32 animMembers = kAnimLayer->GetMemberCount();
//		S32 srcPropCnt = member->GetSrcPropertyCount();
//		S32 dstPropCnt = member->GetDstPropertyCount();
//
//		Con::printf("Found an anim layer: initialName %s  type %s, members %d, srcPropCnt %d dstPropCnt %d",
//			member->GetInitialName(),member->GetTypeName().Buffer(),animMembers,srcPropCnt,dstPropCnt);
//
//		//KFbxProperty prop = member->GetFirstProperty();
//		//while (prop.IsValid())
//		//{
//		//	Con::printf("  property %s   name %s   type %s",prop.GetLabel().Buffer(),
//		//		prop.GetName().Buffer(),prop.GetPropertyDataType().GetName());
//		//	prop=member->GetNextProperty(prop);
//		//}
//
//		for (U32 i=0;i<animMembers;i++)
//		{
//			KFbxObject *animCurveMember = kAnimLayer->GetMember(i);
//			Con::printf("   %s  ",animCurveMember->GetName());
//		}
//	}
//}