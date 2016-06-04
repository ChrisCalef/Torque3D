//-----------------------------------------------------------------------------
// Developed for Torque Game Engine 
// Copyright (C) Chris Calef 2005
//-----------------------------------------------------------------------------
#include "ts/tsShape.h"
#include "ts/tsShapeInstance.h"
#include "ts/tsShapeConstruct.h"
#include "ts/tsTransform.h"
#include "T3D/tsStatic.h"
#include "math/mMath.h"
#include <stdio.h>

QuatF globalQuats[200];
//Point3F nodeTrans[2500];
S32 numFrames;
F32 seqDuration;
TSShape *kork,*soldier,*jeff,*jill,*adam,*cstrike,*swordsman;//TEMP
// methods in this file are for interacting with common/editor/SequenceEditor.gui

void TSShape::moveSequence(U32 target_seq,U32 dest_seq)
{  //Arbitrary anywhere to anywhere function.  Could use this for everything -- sequenceUp and Down 
	//are merely specific applications of this function.
	//So, first insert the sequence into the place where you're going (create a new copy) then delete the old one.
/*
	//Sanity check.
	if ((target_seq==dest_seq)||(target_seq>sequences.size()-1)||(dest_seq>sequences.size()-1))
		return;

	Sequence &kSeq = sequences[target_seq];

	S32 numKeys, numGrounds, numTrigs;
	numKeys = kSeq.numKeyframes;
	numGrounds = kSeq.numGroundFrames;
	numTrigs =  kSeq.numTriggers;

	S32  baseTrans, baseRot, baseScale, baseDecal, baseObject, baseGround, baseTrig;
	baseTrans = kSeq.baseTranslation;
	baseRot = kSeq.baseRotation;
	baseScale = kSeq.baseScale;
	baseDecal = kSeq.baseDecalState;
	baseObject = kSeq.baseObjectState;
	baseGround = kSeq.firstGroundFrame;
	baseTrig = kSeq.firstTrigger;
*/
	//Now:  insert the sequence into the new position by copying it, then drop the old position.  Keep in mind that it will have
	//moved up by one if you're moving it up ahead of where it was.


}

void TSShape::cropSequence(U32 seq,F32 start,F32 stop,const char *filename)
{
	if ((seq<0)||(seq>sequences.size()))
	{
		Con::errorf("cropSequence bailing: sequence %d out of range.",seq);
		return;
	}
	
	if (start>=stop)
	{
		Con::errorf("cropSequence bailing: start value %d is not less than stop value %d.",start,stop);
		return;		
	}

	//Now, get start and end keyframe, and copy everything out.  
	//Maybe into an array, maybe directly into nodeRotations.
	S32 rot_matters_count = 0;
	S32 node_count = 0;
	Quat16 q16;
	bool importGround=false;

	Sequence &srcSeq = sequences[seq];
	S32 srcFrames = srcSeq.numKeyframes;
	S32 srcGroundFrames = srcSeq.numGroundFrames;
	if (srcGroundFrames == srcFrames) 
		importGround = true;
	S32 startFrame,stopFrame;
	startFrame = (S32)((F32)srcFrames * start);
	stopFrame = (S32)((F32)srcFrames * stop);
	Con::printf("sequence cropping: startFrame %d, stopFrame %d",startFrame,stopFrame);
	S32 destFrames = (stopFrame - startFrame);
	F32 destDuration = ((F32)destFrames / (F32)srcFrames) * srcSeq.duration;

	S32 startGroundFrame=0;
	if (srcSeq.numGroundFrames == srcSeq.numKeyframes)
	{
		startGroundFrame = srcSeq.firstGroundFrame + startFrame;//(S32)((F32)srcGroundFrames * start);
	}

	sequences.increment();
	Sequence & destSeq = sequences.last();
	constructInPlace(&destSeq);

	destSeq.numKeyframes = destFrames;
	destSeq.duration = destDuration;
	destSeq.baseRotation = nodeRotations.size();
	destSeq.baseTranslation = nodeTranslations.size();
	destSeq.baseScale = 0;
	destSeq.baseObjectState = 0;
	destSeq.baseDecalState = 0;
	destSeq.firstGroundFrame = groundTranslations.size();
	if (importGround)
		destSeq.numGroundFrames = destFrames;
	else
		destSeq.numGroundFrames = 0;
	destSeq.firstTrigger = triggers.size();
	destSeq.numTriggers = 0;
	destSeq.toolBegin = 0.0;
	destSeq.flags = 0;//Cyclic;// | TSShape::Blend;// | TSShape::MakePath;
	destSeq.priority = 5;

	destSeq.rotationMatters.clearAll();
	destSeq.translationMatters.clearAll();
	
	destSeq.scaleMatters.clearAll();
	destSeq.visMatters.clearAll();
	destSeq.frameMatters.clearAll();
	destSeq.matFrameMatters.clearAll();
	//destSeq.decalMatters.clearAll();
	//destSeq.iflMatters.clearAll();

	//Now, since we've switched over to full file path from getSaveFilename box,instead of just sequence name,
	//we will need to remove .dsq and path to get down to the sequence name.
	String name(filename);
	S32 pos = name.find(".dsq");
	if (pos > -1)
		name.erase(pos,4);

	pos = name.find("/",name.length(),String::Right);
	if (pos > -1)
		name.erase(0,pos+1);

	names.increment();
	names.last() = StringTable->insert(name);
	destSeq.nameIndex = findName(name);

	for (U32 i=0;i<srcSeq.rotationMatters.end();i++) {
		if (srcSeq.rotationMatters.test(i)) {
			destSeq.rotationMatters.set(i);
			rot_matters_count++;
		}
	}
	destSeq.translationMatters.set(0);
	Con::printf("rotation matters count: %d",rot_matters_count);
	
	for (U32 i=0;i<nodes.size();i++) 
	{
		if (destSeq.rotationMatters.test(i))
		{
			for(U32 j=startFrame;j<stopFrame;j++)
			{
				S32 srcRot = srcSeq.baseRotation + (node_count * srcFrames) + j;
				nodeRotations.increment();
				nodeRotations[nodeRotations.size()-1] = nodeRotations[srcRot];
				if (i==0) {	
					/*
					S32 srcTrans = srcSeq.baseTranslation + j;
					nodeTranslations.increment();
					nodeTranslations[nodeTranslations.size()-1] = nodeTranslations[srcTrans];
					*/
					
					Point3F Y_unit(0,1,0);
					Point3F new_Y;
					Quat16 startGroundRot = nodeRotations[srcSeq.baseRotation + (node_count * srcFrames) + startFrame];//Beginning of cropped section.
					QuatF startGrRot = startGroundRot.getQuatF();
					startGrRot.mulP(Y_unit,&new_Y);
					new_Y.z = 0.0;//Removing the vertical element, to project it onto XY plane.
					new_Y.normalize();
					F32 dotProd = mDot(Y_unit,new_Y);

					QuatF rotArc,invRotArc,nodeRot;
					Point3F negY(0,-1,0);
					if ((fabs(new_Y.x)<0.001)&&(new_Y.y==-1.0)&&(fabs(new_Y.z)<0.001)) {//(new_Y.y<-0.9999)//?
						Con::printf("found the direct 180!");
						rotArc.set(0,0,-1,0);
					} else
						rotArc.rotationArc(new_Y,Y_unit);

					MatrixF eulMat;
					rotArc.setMatrix(&eulMat);
					EulerF euler = eulMat.toEuler();

					Quat16 newGroundRot,newNodeRot;//nodeRotations[seq.baseRotation + i]
					newGroundRot.set(rotArc);

					startGrRot = rotArc;
					QuatF startGrRotInv = startGrRot.inverse();//

					//WAIT!!  Did I really just do this here, whether or not importGround is true??
					//QuatF grRot = nodeRotations[nodeRotations.size()-1].getQuatF();
					//QuatF grRotMul = grRot.mul(startGrRotInv,grRot);
					//nodeRotations[nodeRotations.size()-1].set(grRotMul);
					//Con::printf("groundRotMul: %f %f %f %f",grRotMul.x,grRotMul.y,grRotMul.z,grRotMul.w);
					
					S32 srcTrans = srcSeq.baseTranslation + j;
					//Point3F startGroundTrans = nodeTranslations[srcSeq.baseTranslation + startFrame];
					//startGroundTrans.z = 0.0;//Ignore vertical, so we don't put our hip into the ground.
					//Point3F finalGroundTrans;
					//Point3F curGroundTrans = nodeTranslations[srcTrans] - startGroundTrans;
					//startGrRotInv.mulP(curGroundTrans,&finalGroundTrans);
					nodeTranslations.increment();
					//nodeTranslations[nodeTranslations.size()-1] = finalGroundTrans;
					//if (importGround)
						//nodeTranslations[nodeTranslations.size()-1] = nodeTranslations[srcSeq.baseTranslation + startFrame];
					//else
					nodeTranslations[nodeTranslations.size()-1] = nodeTranslations[srcTrans];
				}
			}
			node_count++;
		}
	}

	if (importGround)
	{
		Quat16 startGroundRot = groundRotations[startGroundFrame];//Beginning of cropped section.
		QuatF startGrRot = startGroundRot.getQuatF();
		QuatF startGrRotInv = startGrRot.inverse();//
		for (U32 i=0;i<destFrames;i++)//destGroundFrames
		{
			groundRotations.increment();
			groundRotations[groundRotations.size()-1] = groundRotations[startGroundFrame+i];
			QuatF grRot = groundRotations[groundRotations.size()-1].getQuatF();
			QuatF grRotMul = grRot.mul(startGrRotInv,grRot);
			Con::printf("ground rot: %f %f %f %f,  grRotMul:  %f %f %f %f",grRot.x,grRot.y,grRot.z,grRot.w,grRotMul.x,grRotMul.y,grRotMul.z,grRotMul.w);
			groundRotations[groundRotations.size()-1].set(grRotMul);

			//Now, use the starting ground rot inverse to rotate the ground trans back to starting position. 
			Point3F startGroundTrans = groundTranslations[startGroundFrame];
			Point3F finalGroundTrans;
			groundTranslations.increment();
			Point3F curGroundTrans = groundTranslations[startGroundFrame+i] - startGroundTrans;
			startGrRotInv.mulP(curGroundTrans,&finalGroundTrans);
			groundTranslations[groundTranslations.size()-1] = finalGroundTrans;
			//Con::errorf("cropped groundTranslation %d: %f %f %f,  start %f %f %f",i, groundTranslations[groundTranslations.size()-1].x,
			//	groundTranslations[groundTranslations.size()-1].y,groundTranslations[groundTranslations.size()-1].z,
			//	startGroundTrans.x,startGroundTrans.y,startGroundTrans.z);
		}
	}

		
	//Here: am having occasional "playback bug" behavior here, first time when you create a sequence it
	//frequently won't play back properly until you save it and reload it.  This code is from importBvh
	//but needs to be fixed for current context.

	//Don't need to do this anymore, yay!  exportSequence instead
	//Con::errorf("dropping all but one");
	//dropAllButOneSeq(sequences.size()-1);


	//String dsqPath;
	//if (dStrlen(dsqFile)==0)
	//{
	//	//dsqPath = myPath + '/' + seqName;
	//	dsqPath = seqDir + '/' + seqName;
	//} else 
	//	dsqPath.insert(0,dsqFile);

	//FileStream *outstream;
	//String dsqExt(".dsq");
	//if (!dStrstr(dsqPath.c_str(),".dsq")) dsqPath += dsqExt;
	////if (!gResourceManager->openFileForWrite(outstream,dsqPath.c_str())) {
	//if ((outstream = FileStream::createAndOpen( dsqPath.c_str(), Torque::FS::File::Write))==NULL) {
	//	Con::printf("whoops, name no good: %s!",dsqPath.c_str()); 
	//} else {
	//	kShape->exportSequences((Stream *)outstream);
	//	outstream->close();
	//}

	////Now, load the sequence again, and drop the one we have... we hope this works.
	//
	//Con::errorf("loading sequence: %s",dsqPath.c_str());
	//loadDsq(dsqPath.c_str());

	//kShape->dropAllButOneSeq(kShape->sequences.size()-1);
}

F32 TSShape::findStop(U32 slot,F32 start)
{
	//NOPE:  This doesn't work at all, first because I tried to use angleBetween, which is fail for what we need here,
	//but more importantly because we can't use shape->nodeRotations because they are local rotations.  What works much  
	//better are global transforms, ie shapeInstance->nodeTransforms, but to get those we actually have to play through 
	//the sequence.  UNLESS we just ran through nodeRotations but we collected parent transforms and applied them here.

	//if ((slot<0)||(slot>sequences.size()))
	//{
	//	Con::errorf("findStop bailing: sequence %d out of range.",slot);
	//	return 0.0;
	//}
	//
	//if (start==1.0)
	//{
	//	Con::errorf("findStop bailing: start value is 1.0.");
	//	return 0.0;		
	//}

	//Con::errorf("starting findStop");

	////Now, get start and end keyframe, and copy everything out.  
	////Maybe into an array, maybe directly into nodeRotations.
	//S32 rot_matters_count = 0;
	//S32 node_count = 0;
	//Quat16 q16[400];
	//Quat16 tq16;
	//QuatF q,tq;
	//bool climbing;

	//F32 diffSum = 0.0;
	//F32 lastDiffSum = 0.0;
	//F32 minDiffSum = 10000000.0;
	//S32 minDiffFrame;

	//Con::errorf("declared my variables");

	//Sequence &srcSeq = sequences[slot];
	//if (srcSeq.numKeyframes) {
	//	S32 srcFrames = srcSeq.numKeyframes;
	//	S32 startFrame,stopFrame;
	//	startFrame = (S32)((F32)srcFrames * start);
	//	Con::errorf("startFrame: %d",startFrame);


	//	for (U32 i=0;i<nodes.size();i++) 
	//	{
	//		if (srcSeq.rotationMatters.test(i))
	//		{
	//			S32 srcRot = srcSeq.baseRotation + (node_count * srcFrames) + startFrame;
	//			q16[i] = nodeRotations[srcRot];
	//			node_count++;
	//		}
	//	}
	//	node_count = 0;
	//	climbing = true;
	//	for(U32 j=startFrame;j<srcSeq.numKeyframes;j++)
	//	{
	//		for (U32 i=0;i<nodes.size();i++) 
	//		{
	//			if (srcSeq.rotationMatters.test(i))
	//			{
	//				S32 srcRot = srcSeq.baseRotation + (node_count * srcFrames) + j;
	//				tq16 = nodeRotations[srcRot];
	//				q16[i].getQuatF(&q);
	//				tq16.getQuatF(&tq);
	//				diffSum += q.angleBetween(tq);
	//				//Con::printf("srcRot %d  node_count %d srcFrames %d j %d node %d diff: %f",srcRot,node_count,srcFrames,j,i,diffSum);
	//				node_count++;
	//			}
	//		}
	//		//Whoops, gonna have to finish this later, it's getting complicated.  Gotta first check when we're past the peak, 
	//		//then check when we're past the minimum.  But it's damn close.  Go.
	//		Con::printf("frame %d diff: %f",j,diffSum);
	//		if (diffSum<lastDiffSum) climbing = false;
	//		if ((diffSum>lastDiffSum)&&(climbing==false)&&(diffSum<2.0))
	//		{
	//			minDiffFrame = j-1;
	//			return ((F32)minDiffFrame/(F32)srcFrames);
	//		}
	//		//if (diffSum<minDiffSum) {
	//		//	minDiffSum = diffSum;
	//		//	minDiffFrame = j;
	//		//}
	//		lastDiffSum = diffSum;
	//		node_count = 0;
	//		diffSum = 0.0;
	//	}
	//} else return 0.0;
	return 0.0;
}

void TSShape::sequenceUp(U32 seq)
{
	if ((sequences.size() > 1)&&(seq > 0)) 
		moveSequence(seq,seq-1);
}

void TSShape::sequenceDown(U32 seq)
{
	if ((sequences.size() > 1)&&(seq < sequences.size()-1)) 
		moveSequence(seq,seq+1);
}

void TSShape::dropSequence(U32 seq)
{
	//And... at last, the official torque version of this function exists for us:
	removeSequence(names[sequences[seq].nameIndex]);

	//BUT, we do need to remove it from the shape constructor manually.
	//String myFullPath = getPath();
	//TSShapeConstructor* ctor = TSShapeConstructor::findShapeConstructor( myFullPath );
	//if (ctor)
	//{
		//Con::printf("dropping sequence %d  %s",seq,names[sequences[seq].nameIndex]);
		//TSShapeConstructor::ChangeSet::Command cmd("removeSequence");
		//No, this isn't going to work.  This would keep the addSequence command and then add
		//another removeSequence command later, which would be stupid.   What I need is to 
		//rewrite the addSequence list every time I save, from only sequences currently loaded.
		//And... damn!  There is no method for removing a command from a shape constructor changeset.  
		//Have to write that ourselves, unless it's in beta 3 - which doesn't appear likely.
		//Leaving it for now as a known bug.
	//}


	/*
	//First, don't let them delete the final sequence.  (To do: do let them do this, 
	//or warn them.  For now just exit.)
	if (sequences.size()<2) 
		return;

	//HERE:  we need to 
	//  A) drop what is necessary out of the main shape arrays: nodeRotations[] ..., 
	//  B) delete from sequences[] 
	//  C) update baseTranslation etc. in remaining sequences past the delete point

	Sequence &kSeq = sequences[seq];

	//Con::errorf("my shape has: nodeTrans %d nodeRot %d groundTranslations %d groundRotations %d triggers %d",
	//	nodeTranslations.size(),nodeRotations.size(),groundTranslations.size(),groundRotations.size(),triggers.size());

	//Con::errorf("dropping sequence %d: %s keyframes %d groundframes %d triggers %d",
	//	seq,names[kSeq.nameIndex].c_str(),kSeq.numKeyframes,kSeq.numGroundFrames,kSeq.numTriggers);
	
	S32 numKeys, numGrounds, numTrigs;
	numKeys = kSeq.numKeyframes;
	numGrounds = kSeq.numGroundFrames;
	numTrigs =  kSeq.numTriggers;

	S32  baseTrans, baseRot, baseScale, baseDecal, baseObject, baseGround, baseTrig;
	baseTrans = kSeq.baseTranslation;
	baseRot = kSeq.baseRotation;
	baseScale = kSeq.baseScale;
	baseDecal = kSeq.baseDecalState;
	baseObject = kSeq.baseObjectState;
	baseGround = kSeq.firstGroundFrame;
	baseTrig = kSeq.firstTrigger;

	//If not, we don't have to do much... we're deleting the last sequence.
	if (sequences.size()>(seq+1)) {//But now we're deleting one in the middle, so the lists need to be adjusted.
		Sequence &nSeq = sequences[seq+1];
		S32  nextTrans, nextRot, nextScale, nextDecal, nextObject, nextGround, nextTrig;
		S32  diffTrans, diffRot, diffScale, diffDecal, diffObject, diffGround, diffTrig;

		nextTrans = nSeq.baseTranslation;
		nextRot = nSeq.baseRotation;
		nextScale = nSeq.baseScale;
		nextDecal = nSeq.baseDecalState;
		nextObject = nSeq.baseObjectState;
		nextGround = nSeq.firstGroundFrame;
		nextTrig = nSeq.firstTrigger;

		diffTrans = nextTrans - baseTrans;
		diffRot = nextRot - baseRot;
		diffGround = nextGround - baseGround;
		diffTrig = nextTrig - baseTrig;

		//diffScale = next - base;
		//diffDecal = next - base;
		//diffObject = next - base;

		//NOW:  what we have to do is, first delete from the lists... hopefully erase() does exactly what we want.  test it.
		if (diffTrans) 
		{
			for (U32 i=0;i<diffTrans;i++) 
			{	
				nodeTranslations.erase(baseTrans);
			}
		}
		if (diffRot)
		{
			for (U32 i=0;i<diffRot;i++) 
			{
				nodeRotations.erase(baseRot);
			}
		}
		if (diffGround)
		{
			for (U32 i=0;i<diffGround;i++) 
			{
				groundTranslations.erase(baseGround);
				groundRotations.erase(baseGround);
			}
		}
		if (diffTrig)
		{
			for (U32 i=0;i<diffTrig;i++) 
			{
				triggers.erase(baseTrig);
			}
		}
		for (U32 s=seq;s<sequences.size();s++)
		{
			U32 newBaseTrans, newBaseRot, newBaseTrig;
			sequences[s].baseTranslation -= diffTrans;
			sequences[s].baseRotation -= diffRot;
			sequences[s].firstGroundFrame -= diffGround;
			sequences[s].firstTrigger -= diffTrig;
		}	
	}
	
	sequences.erase(seq);
	//Con::printf("successfully dropped sequence %d",seq);
	*/
}

void TSShape::dropAllSequences()
{
	while (sequences.size()>1)
		dropSequence(sequences.size()-1);
	dropSequence(0);
}

void TSShape::dropAllButOneSeq(U32 seq)
{
	if (seq == (sequences.size() - 1) )
	{//Taking on trivial case first, in which all we want to keep is the last one.  Might
		//have trouble deleting the last one, have crashed there before, but we don't _need_
		//to fix that today.
		//Con::printf("dropping the last sequence");
		if (sequences.size()>1)
		{
			while (sequences.size()>1)
				dropSequence(0);
		}
	} 
	else if (seq == 0)
	{
		while (sequences.size()>1)
			dropSequence(sequences.size()-1);
	} 
	else //we are somewhere in the middle
	{
		while (seq < (sequences.size() - 1) )
			dropSequence(sequences.size()-1);

		//Now the one we want is last, so delete all in front of it.
		while (sequences.size()>1)
			dropSequence(0);
	}
}

void TSShape::convertDefaultPose(TSShape *other, char *cfg)
{
	S32 i,j,sz,node_here,node_there,matters_count,node_count;
	F32 q1x,q1y,q1z,q2x,q2y,q2z,q3x,q3y,q3z,q4x,q4y,q4z;
	QuatF q1,q2,q3,q4,q5,q6,q7,q8,qa,qb,qc,qd;
	char buf[255];
	S32 nodes_here[100];
	S32 nodes_there[100];

	matters_count = 0; node_count = 0;

	for (i=0;i<100;i++) {
		nodes_here[i]=-1;
		nodes_there[i]=-1;
	}


	FILE *fp = fopen(cfg,"r");
	fgets(buf,255,fp);//get rid of first line(comment)
	while (fgets(buf,255,fp)) {
		sscanf(buf,"%d;%d;(%f,%f,%f);(%f,%f,%f);(%f,%f,%f);(%f,%f,%f);",&node_there,&node_here,&q1x,&q1y,&q1z,&q2x,&q2y,&q2z,&q3x,&q3y,&q3z,&q4x,&q4y,&q4z);
		nodes_there[matters_count] = node_there;
		nodes_here[matters_count] = node_here;
		matters_count++;
	}
	fclose(fp);

	fp = fopen(cfg,"r");
	fgets(buf,255,fp);//get rid of first line(comment)
	//NOW, convert the default rotations
	while (fgets(buf,255,fp)) {
		sscanf(buf,"%d;%d;(%f,%f,%f);(%f,%f,%f);(%f,%f,%f);(%f,%f,%f);",&node_there,&node_here,&q1x,&q1y,&q1z,&q2x,&q2y,&q2z,&q3x,&q3y,&q3z,&q4x,&q4y,&q4z);

		qa.set(EulerF(mDegToRad(q1x),mDegToRad(q1y),mDegToRad(q1z)));
		qb.set(EulerF(mDegToRad(q2x),mDegToRad(q2y),mDegToRad(q2z)));
		qc.set(EulerF(mDegToRad(q3x),mDegToRad(q3y),mDegToRad(q3z)));
		qd.set(EulerF(mDegToRad(q4x),mDegToRad(q4y),mDegToRad(q4z)));


		q1.mul(qb,qa);//have to reload this every time, since we invert it later.
		q2.mul(qd,qc);//this one's just along for the ride :-)

		//DANGER!  This will break if we have to drop nodes that aren't in the cfg
		other->defaultRotations[nodes_there[node_count]].getQuatF(&q3);

		q4.mul(q3,q2);
		q5.mul(q1,q4);
		q6.mul(q5,q1.inverse());

		int ni = nodes_here[node_count];
		defaultRotations[ni].set(q6);
		Con::printf("%d %d q6(%2.2f,%2.2f,%2.2f,%2.2f)",ni,node_count,q6.x,q6.y,q6.z,q6.w);
		node_count++; 
	}//wow, is that going to work??

}

void TSShape::convertSequence(TSShape *other, const char *cfg, S32 sequence,const char *shapepath)
{//FIX add Stream * functionality instead of fgets
	S32 i,j,k,sz,node_here,node_there,matters_count,matters_count_source,node_count;
	F32 q1x,q1y,q1z,q2x,q2y,q2z,q3x,q3y,q3z,q4x,q4y,q4z;
	QuatF q1,q2,q3,q4,q5,q6,q7,q8,qa,qb,qc,qd;
	char buf[255]; 
	//FIX some of these arrays are excess
	S32 nodes_here[100],nodes_here_sort[100],nodes_here_count[100];
	S32 nodes_there[100],nodes_there_count[100],nodes_there_temp[100];

	matters_count = 0; matters_count_source = 0; node_count = 0;

	for (i=0;i<100;i++) {
		nodes_here[i]=-1;
		nodes_here_sort[i]=-1;
		nodes_here_count[i]=-1;
		nodes_there[i]=-1;
		nodes_there_count[i]=-1;
		nodes_there_temp[i]=-1;
	}

	Con::printf("starting TSShape::convertSequence ===========================");

	sequences.increment();// if error, decrement back
	Sequence & seq = sequences.last();
	constructInPlace(&seq);

	seq.nameIndex = names.size();
	names.increment();
	names.last() = StringTable->insert(other->getName(other->sequences[sequence].nameIndex),false);

	//seq.baseRotation = 0; //FIX
	S32 startRot = nodeRotations.size();
	seq.baseRotation = startRot;
	seq.baseTranslation = nodeTranslations.size();

	seq.numKeyframes = other->sequences[sequence].numKeyframes;
	seq.duration = other->sequences[sequence].duration;
	seq.priority = other->sequences[sequence].priority;  
	seq.flags = other->sequences[sequence].flags;  //blend: not working

	seq.baseScale = 0; seq.baseObjectState = 0;//FIX!!
	seq.baseDecalState = 0; seq.toolBegin = 0;
	seq.numTriggers = 0;//FIX! Copy whatever triggers there are.
	//seq.numTriggers = other->sequences[sequence].numTriggers;

	
	char filename[1000];
	sprintf(filename,"%s/%s.cfg",shapepath,cfg);
	Con::printf("my cfg %s",filename);
	FILE *fp = fopen(filename,"r");
	fgets(buf,255,fp);//get rid of first line(comment)
	while (fgets(buf,255,fp)) {
		sscanf(buf,"%d;%d;(%f,%f,%f);(%f,%f,%f);(%f,%f,%f);(%f,%f,%f);",&node_there,&node_here,&q1x,&q1y,&q1z,&q2x,&q2y,&q2z,&q3x,&q3y,&q3z,&q4x,&q4y,&q4z);
		nodes_there[matters_count] = node_there;
		nodes_here[matters_count] = node_here;
		nodes_here_sort[matters_count] = node_here;
		matters_count++;
	}
	fclose(fp);
	Con::printf("loaded matters %d, closed file.",matters_count);
	Con::printf("sequence %d, rotationMatters %d.",sequence,other->sequences[sequence].rotationMatters.end());
	//There, that finds out how many we have total, and loads up the arrays.

	//gotta find out if there's any nodes in the source sequence that we can't use on the destination model
	for (i=0;i<other->sequences[sequence].rotationMatters.end();i++) {
		if (other->sequences[sequence].rotationMatters.test(i)) {
			nodes_there_temp[matters_count_source] = i;
			matters_count_source++;
		}
	}

	Con::printf("matters count source: %d",matters_count_source);

	for (i=0;i<matters_count;i++) {
		for (j=0;j<matters_count_source;j++) {
			if (nodes_there[i]==nodes_there_temp[j]) nodes_there_count[i]=j;
		}
	}
	//for (i=0;i<matters_count;i++) Con::printf("nodes there count[%d]: %d",i,nodes_there_count[i]);

	//next, need to sort the here_nodes so we know where to put them in nodeRotations.
	for (i=0;i<matters_count;i++) {
		for (j=0;j<(matters_count-1);j++) {
			S32 temp;
			if (nodes_here_sort[j]>nodes_here_sort[j+1]) {
				temp = nodes_here_sort[j];
				nodes_here_sort[j] = nodes_here_sort[j+1];
				nodes_here_sort[j+1] = temp;
			}
		}
	}

	//for (i=0;i<matters_count;i++) Con::printf("nodes_here_sort[%d]:%d",i,nodes_here_sort[i]);
	for (i=0;i<matters_count;i++) {
		for (j=0;j<matters_count;j++) {
			if (nodes_here[i]==nodes_here_sort[j]) nodes_here_count[i]=j;
		}
	}

	//for (i=0;i<matters_count;i++) Con::printf("nodes_here_count[%d]:%d",i,nodes_here_count[i]);
	//for (i=0;i<matters_count;i++) Con::printf("nodes_there_count[%d]:%d",i,nodes_there_count[i]);

	//Now, open it again and go through line by line, converting rotations.
	//fp = fopen(cfg,"r");
	fp = fopen(filename,"r");
	fgets(buf,255,fp);//get rid of first line(comment)

	nodeRotations.setSize(startRot+(seq.numKeyframes*matters_count));
	Con::printf("new NodeRot size: %d",startRot+(seq.numKeyframes*matters_count));


	//NOW, finally get to convert the actual rotations
	while (fgets(buf,255,fp)) {
		//Here: sscanf(buf,"%d;%d;(%f,%f,%f);(%f,%f,%f);(%f,%f,%f);(%f,%f,%f);",...);
		//just suck it all up in one line.
		sscanf(buf,"%d;%d;(%f,%f,%f);(%f,%f,%f);(%f,%f,%f);(%f,%f,%f);",&node_there,&node_here,&q1x,&q1y,&q1z,&q2x,&q2y,&q2z,&q3x,&q3y,&q3z,&q4x,&q4y,&q4z);

		qa.set(EulerF(mDegToRad(q1x),mDegToRad(q1y),mDegToRad(q1z)));//THIS IS ONLY TESTED WITH ONE VALUE AT A TIME
		qb.set(EulerF(mDegToRad(q2x),mDegToRad(q2y),mDegToRad(q2z)));
		qc.set(EulerF(mDegToRad(q3x),mDegToRad(q3y),mDegToRad(q3z)));
		qd.set(EulerF(mDegToRad(q4x),mDegToRad(q4y),mDegToRad(q4z)));

		//NOW: have to loop through all the nodeRotations for THIS sequence and THIS NODE only.
		seq.rotationMatters.set(node_here);
		Con::printf("rotationMatters(%d)",node_here);

		//NOW: i is only looping through the rotations for one node, not the whole sequence.
		//Con::printf("Starting: q1(%2.2f,%2.2f,%2.2f,%2.2f), q2(%2.2f,%2.2f,%2.2f,%2.2f)",q1.x,q1.y,q1.z,q1.w,q2.x,q2.y,q2.z,q2.w);
		for (i=0;i<seq.numKeyframes;i++) {

			q1.mul(qb,qa);//have to reload this every time, since we invert it later.
			q2.mul(qd,qc);//this one's just along for the ride :-)

			//DANGER!  This will break if we have to drop nodes that aren't in the cfg
			other->nodeRotations[i+(nodes_there_count[node_count]*seq.numKeyframes)+other->sequences[sequence].baseRotation].getQuatF(&q3);

			q4.mul(q3,q2);
			q5.mul(q1,q4);
			q6.mul(q5,q1.inverse());
			//q1.inverse();//OOPS!!  Turns out this isn't a read-only function!

			//OOPS - I need a sort array
			int ni = startRot+(nodes_here_count[node_count]*seq.numKeyframes)+i;
			//nodeRotations[startRot+(nodes_here_count[node_count]*seq.numKeyframes)+i].set(q6);
			nodeRotations[ni].set(q6);
			Con::printf("%d %d q6(%2.2f,%2.2f,%2.2f,%2.2f)",ni,node_count,q6.x,q6.y,q6.z,q6.w);
			//Con::printf("q1(%2.2f,%2.2f,%2.2f,%2.2f), q2(%2.2f,%2.2f,%2.2f,%2.2f)",q1.x,q1.y,q1.z,q1.w,q2.x,q2.y,q2.z,q2.w);
		}
		node_count++; // this will now track which node we're on.
	}

	seq.translationMatters.set(0);
	sz = nodeTranslations.size();
	seq.baseTranslation = sz;
	nodeTranslations.setSize(sz+seq.numKeyframes);
	for (i=0;i<seq.numKeyframes;i++) {
		//nodeTranslations[i+sz].set(0,0,0); 
		nodeTranslations[i+sz] = other->nodeTranslations[i+other->sequences[sequence].baseTranslation];
	}

	seq.firstGroundFrame = 0;
	seq.numGroundFrames = 0;

	//groundTranslations.setSize(0);
	//groundRotations.setSize(0);
	/*
	S32 startGroundFrame = groundRotations.size();
	seq.firstGroundFrame = startGroundFrame;

	if (other->sequences[sequence].firstGroundFrame) {
	sz = groundTranslations.size();
	groundTranslations.setSize(sz+seq.numKeyframes);
	for (i=0;i<seq.numKeyframes;i++) {
	groundTranslations[i+sz].set(other->groundTranslations[i+other->sequences[sequence].firstGroundFrame]);
	//Con::printf("ground trans %d: %f %f %f",sz+i,groundTranslations[sz+i].x,groundTranslations[sz+i].y,groundTranslations[sz+i].z);
	}

	sz = groundRotations.size();
	groundRotations.setSize(sz+seq.numKeyframes);
	for (i=0;i<seq.numKeyframes;i++) {
	//groundRotations[sz+i].set(other->groundRotations[i+other->sequences[sequence].firstGroundFrame]);
	groundRotations[sz+i].set(EulerF(0,0,0));
	}
	}
	*/  
	fclose(fp);


	// DONE!  Save it out elsewhere.
}

void TSShape::saveSequence(Stream *s,S32 seqNum)
{
   S32 i,sz,rot_matters_count,trans_matters_count,scale_matters_count;
	if (seqNum<0) return;
   // write version
   s->write(smVersion);

   // write node names
   // -- this is how we will map imported sequence nodes to shape nodes
   sz = nodes.size();
   s->write(sz);
   for (i=0;i<nodes.size();i++)
      writeName(s,nodes[i].nameIndex);

   // legacy write -- write zero objects, don't pretend to support object export anymore
   s->write(0);

   // on import, we will need to adjust keyframe data based on number of
   // nodes/objects in this shape...number of nodes can be inferred from
   // above, but number of objects cannot be.  Write that quantity here:
   s->write(objects.size());

   Sequence & seq = sequences[seqNum];
   //HERE: need to copy out the noderotations for this sequence, 
   //so everything from baseRotation to baseRotation + numKeyframes
   //Keep them all in a second array.  Do same with translations, groundframes, etc.

   Vector<Quat16>                   kNodeRotations;
   Vector<Point3F>                  kNodeTranslations;
   Vector<F32>                      kNodeUniformScales;
   Vector<Point3F>                  kNodeAlignedScales;
   Vector<Quat16>                   kNodeArbitraryScaleRots;
   Vector<Point3F>                  kNodeArbitraryScaleFactors;
   Vector<Quat16>                   kGroundRotations;
   Vector<Point3F>                  kGroundTranslations;
   Vector<Trigger>                  kTriggers;
   //Vector<F32>                      kIflFrameOffTimes;
   //Vector<TSLastDetail*>            kBillboardDetails;
   //Vector<ConvexHullAccelerator*>   kDetailCollisionAccelerators;
   Vector<String>                   kNames;

   rot_matters_count = 0;
   for (U32 j=0;j<nodes.size();j++) 
   	   if (seq.rotationMatters.test(j)) rot_matters_count++;
   

   trans_matters_count = 0;
   //I hope this is always one, but not making assumptions.
   for (U32 j=0;j<nodes.size();j++) 
   	   if (seq.translationMatters.test(j)) trans_matters_count++;

   scale_matters_count = 0;
   for (U32 j=0;j<nodes.size();j++) 
   	   if (seq.scaleMatters.test(j)) scale_matters_count++;

   // write node states -- skip default node states

   s->write(seq.numKeyframes * rot_matters_count);
   Con::errorf("nodeRotations: %d.  Actual nodeRotations: %d, baseRotation %d",
	   seq.numKeyframes * rot_matters_count,nodeRotations.size(),seq.baseRotation);
   for (i=seq.baseRotation;i<seq.baseRotation + (seq.numKeyframes * rot_matters_count);i++)
   {
      s->write(nodeRotations[i].x);
      s->write(nodeRotations[i].y);
      s->write(nodeRotations[i].z);
      s->write(nodeRotations[i].w);
   }

   s->write(seq.numKeyframes * trans_matters_count);
   Con::errorf("nodeTranslations: %d.  Actual nodeTranslations: %d, baseTranslation %d",
	   seq.numKeyframes * trans_matters_count,nodeTranslations.size(),seq.baseTranslation);
   for (i=seq.baseTranslation;i<seq.baseTranslation + (seq.numKeyframes * trans_matters_count); i++)
   {
      s->write(nodeTranslations[i].x);
      s->write(nodeTranslations[i].y);
	  s->write(nodeTranslations[i].z);
   }

   //I hope these really are all connected to scale_matters_count, because I don't see any other matters arrays.
   //if (seq.animatesUniformScale()) 
   //{
   s->write(seq.numKeyframes * scale_matters_count);
   Con::errorf("nodeUniformScales: %d",seq.numKeyframes * scale_matters_count);
   for (i=seq.baseScale;i<seq.baseScale + (seq.numKeyframes * scale_matters_count);i++)
	   s->write(nodeUniformScales[i]);
   //}

   //if (seq.animatesAlignedScale()) 
   //{
   s->write(seq.numKeyframes * scale_matters_count);
   Con::errorf("nodeAlignedScales: %d",seq.numKeyframes * scale_matters_count);
   for (i=seq.baseScale;i<seq.baseScale + (seq.numKeyframes * scale_matters_count);i++)
   {
	   s->write(nodeAlignedScales[i].x);
	   s->write(nodeAlignedScales[i].y);
	   s->write(nodeAlignedScales[i].z);
   }
   //}

   //if (seq.animatesArbitraryScale()) 
   //{
   s->write(seq.numKeyframes * scale_matters_count);
   Con::errorf("nodeArbitraryScaleRots: %d",seq.numKeyframes * scale_matters_count);
   for (i=seq.baseScale;i<seq.baseScale + (seq.numKeyframes * scale_matters_count);i++)
   {
	   s->write(nodeArbitraryScaleRots[i].x);
	   s->write(nodeArbitraryScaleRots[i].y);
	   s->write(nodeArbitraryScaleRots[i].z);
	   s->write(nodeArbitraryScaleRots[i].w);
   }
   for (i=seq.baseScale;i<seq.baseScale + (seq.numKeyframes * scale_matters_count);i++)
   {
	   s->write(nodeArbitraryScaleFactors[i].x);
	   s->write(nodeArbitraryScaleFactors[i].y);
	   s->write(nodeArbitraryScaleFactors[i].z);
   }
   //}

   s->write(seq.numGroundFrames);//(groundTranslations.size());
   Con::errorf("groundFrames: %d",seq.numGroundFrames);
   for (i=seq.firstGroundFrame;i<seq.firstGroundFrame + seq.numGroundFrames;i++)
   {
      s->write(groundTranslations[i].x);
      s->write(groundTranslations[i].y);
      s->write(groundTranslations[i].z);
   }
   for (i=seq.firstGroundFrame;i<seq.firstGroundFrame + seq.numGroundFrames;i++)
   {
      s->write(groundRotations[i].x);
      s->write(groundRotations[i].y);
      s->write(groundRotations[i].z);
      s->write(groundRotations[i].w);
   }

   // write object states -- legacy..no object states
   s->write((S32)0);

   // write sequences
   s->write(1);//num sequences, in this case only one.

   // first write sequence name
   writeName(s,seq.nameIndex);
   // now write the sequence itself
   seq.write(s,false); // false --> don't write name index
   

   // write out all the triggers...
   s->write(seq.numTriggers);
   for (i=seq.firstTrigger; i<seq.firstTrigger + seq.numTriggers; i++)
   {
      s->write(triggers[i].state);
      s->write(triggers[i].pos);
   }
}

S32 TSShape::getNumMattersNodes(S32 seqNum)
{
	S32 rot_matters_count = 0;
	if (seqNum>=0)
	{
		Sequence & seq = sequences[seqNum];

		for (U32 j=0;j<nodes.size();j++) 
			if (seq.rotationMatters.test(j)) rot_matters_count++;
	}
	return rot_matters_count;
}

S32 TSShape::getMattersNodeIndex(S32 seqNum,S32 nodeNum)
{//nodenum in matters index, 0 - rotation_matters_count


	S32 rot_matters_count = 0;
	S32 node_index = -1;

	if (seqNum>=0)
	{
		Sequence & seq = sequences[seqNum];
		for (U32 j=0;j<nodes.size();j++) {
			if (seq.rotationMatters.test(j)) 
			{
				if (nodeNum == rot_matters_count)
				{
					node_index = j;
					break;
				}
				rot_matters_count++;
			}
		}
	}
	return node_index;
}

S32 TSShape::getNodeMattersIndex(S32 seqNum,S32 nodeNum)
{//nodenum in shape index, 0 - nodes.size

	S32 rot_matters_count = 0;
	S32 node_index = -1;
	if (seqNum>=0)
	{
		Sequence & seq = sequences[seqNum];
		for (U32 j=0;j<nodes.size();j++) {
			if (seq.rotationMatters.test(j)) 
			{
				if (nodeNum==j)
				{
					node_index = rot_matters_count;
					break;
				}
				rot_matters_count++;
			}
		}
	}

	return node_index;
}

void TSShape::addMattersNode(S32 seqNum,S32 nodeNum)
{
	if (seqNum>=0)
	{
		Sequence & seq = sequences[seqNum];
		Con::errorf("adding matters node %d to seq %d",nodeNum,seqNum);
		if ( (nodeNum>0) && (nodeNum<nodes.size()) && !seq.rotationMatters.test(nodeNum))
		{
			seq.rotationMatters.set(nodeNum);

			//Add identity quats to nodeRotations to fill the space, move baseRotation back for every 
			//sequence past here.

			S32 rot_matters_count = getNumMattersNodes(seqNum);
			S32 mattersIndex = getNodeMattersIndex(seqNum,nodeNum);
			S32 startRot = seq.baseRotation + (mattersIndex * seq.numKeyframes);

			Con::errorf("startRot: %d, keyframes %d, mattersIndex %d",startRot,seq.numKeyframes,mattersIndex);

			//Add some blank nodeRots at the back...
			nodeRotations.increment(seq.numKeyframes);

			//then move everything from the entry point to the end, back by numKeyframes.
			for (U32 i=nodeRotations.size()-1; i>=startRot+seq.numKeyframes; i--)
				nodeRotations[i] = nodeRotations[i-seq.numKeyframes];

			//Then fill in the new space with identity quats.
			Quat16 q16; q16.identity();
			for (U32 i=startRot; i<startRot+seq.numKeyframes; i++)
				nodeRotations[i] = q16;

			//And change everybody's baseRotation to reflect the new rots.
			for (U32 s=seqNum+1;s<sequences.size();s++)
			{
				U32 newBaseTrans, newBaseRot, newBaseTrig;
				sequences[s].baseRotation += seq.numKeyframes;
			}	
		}
	}
}


void TSShape::dropMattersNode(S32 seqNum,S32 nodeNum)
{
	if (seqNum<0) return;
	Sequence & seq = sequences[seqNum];
	Con::errorf("dropping matters node %d from seq %d",nodeNum,seqNum);
	if ( (nodeNum>0) && (nodeNum<nodes.size()) && seq.rotationMatters.test(nodeNum))
	{

		//Remove this node's quats from nodeRotations, move baseRotation forward for every 
		//sequence past here.

		S32 rot_matters_count = getNumMattersNodes(seqNum);
		S32 mattersIndex = getNodeMattersIndex(seqNum,nodeNum);
		S32 startRot = seq.baseRotation + (mattersIndex * seq.numKeyframes);

		//Copy everything up to fill this node's slot.
		for (U32 i=startRot; i<nodeRotations.size()-seq.numKeyframes; i++)
			nodeRotations[i] = nodeRotations[i+seq.numKeyframes];


		Con::errorf("startRot: %d, keyframes %d, mattersIndex %d",startRot,seq.numKeyframes,mattersIndex);
		//Drop off the tail...
		nodeRotations.decrement(seq.numKeyframes);

		//And fix everybody's baseRotation.
		for (U32 s=seqNum+1;s<sequences.size();s++)
		{
			U32 newBaseTrans, newBaseRot, newBaseTrig;
			sequences[s].baseRotation -= seq.numKeyframes;
		}	
		
		seq.rotationMatters.clear(nodeNum);
	}
}

void TSShape::groundCaptureSeq(S32 seqNum)
{//HERE: grab all ground translations and rotations from this seq, convert in place.
	
	if (seqNum<0) return;
	Sequence & seq = sequences[seqNum];

	//UM, do we also need to modify firstGroundFrame for all the other sequences???

	if (seq.numGroundFrames>1) 
	{
		Con::errorf("This sequence already has ground frames!");
		return;//HERE: at a future time we could see what real world sequences actually have ground frames
		//and possibly interpolate the existing ones out to full numGroundFrames=numKeyFrames EM style.
	}
	seq.firstGroundFrame = groundTranslations.size();
	seq.numGroundFrames = seq.numKeyframes;

	for (U32 i=0;i<seq.numKeyframes;i++)
	{
		groundTranslations.increment();

		groundTranslations[groundTranslations.size()-1].x = nodeTranslations[seq.baseTranslation + i].x;
		groundTranslations[groundTranslations.size()-1].y = nodeTranslations[seq.baseTranslation + i].y;
		groundTranslations[groundTranslations.size()-1].z = 0.0;//Stays zero, height handled by nodeTranslations.
		//FIX for terrain, world elevations??

		nodeTranslations[seq.baseTranslation + i].x = 0.0;
		nodeTranslations[seq.baseTranslation + i].y = 0.0;
		//local z within this animation stays here, global z goes to groundTranslations.

		groundRotations.increment();
		//The old way - grabbing the whole transform.
		//groundRotations[groundRotations.size()-1] = nodeRotations[seq.baseRotation + i];
		//nodeRotations[seq.baseRotation + i].identity();

		//Versus the new way: attempting to isolate rotation on the XY plane (rotation around the Z
		// axis) from all other rotation.  Plan: get a +Y unit projection, run it through the groundRot
		//quaternion to get hip-node local +Y.  Then drop the Z from that to get a projection on the XY
		//plane.  Then, do a high school geometry thing to find the angle between that and a global
		//+Y unit vector, and there's your angle to rotate around Z.  Do that to an identity quat and
		//you have your groundTransform, multiply the original node transform by the INVERSE of that 
		//and you have your new node transform.  Or so the theory goes...

		Point3F Y_unit(0,1,0);
		Point3F new_Y;
		QuatF groundRot = nodeRotations[seq.baseRotation + i].getQuatF();
		groundRot.mulP(Y_unit,&new_Y);
		new_Y.z = 0.0;//Removing the vertical element, to project it onto XY plane.
		new_Y.normalize();
		F32 dotProd = mDot(Y_unit,new_Y);
		
		QuatF rotArc,invRotArc,nodeRot;
		Point3F negY(0,-1,0);
		//Point3F diff = new_Y - negY;
		//if (diff.len()==0.0){
		if ((fabs(new_Y.x)<0.001)&&(new_Y.y==-1.0)&&(fabs(new_Y.z)<0.001)) {//(new_Y.y<-0.9999)//?
			Con::printf("found the direct 180!");
			rotArc.set(0,0,-1,0);
		} else
			rotArc.rotationArc(new_Y,Y_unit);
		
		MatrixF eulMat;
		rotArc.setMatrix(&eulMat);
		EulerF euler = eulMat.toEuler();

		Quat16 newGroundRot,newNodeRot;//nodeRotations[seq.baseRotation + i]
		newGroundRot.set(rotArc);
		groundRotations[groundRotations.size()-1] = newGroundRot;

		invRotArc = rotArc;
		invRotArc.inverse();

		nodeRotations[seq.baseRotation + i].getQuatF(&nodeRot);
		QuatF tempRot = nodeRot;
		nodeRot.mul(tempRot,invRotArc);
		nodeRotations[seq.baseRotation + i].set(nodeRot);
	}
	return;
}

void TSShape::unGroundCaptureSeq(S32 seqNum)
{//HERE: put ground translations and rotations back into the regular nodeTranslations, nodeRotations.
	
	Con::printf("unGroundCapturing sequence!");
	if (seqNum<0) return;
	Sequence & seq = sequences[seqNum];
	//UM, do we also need to modify firstGroundFrame for all the other sequences???

	if (seq.numGroundFrames != seq.numKeyframes) 
	{
		Con::errorf("This sequence does not have Ecstasy Motion style ground frames!");
		return;
	}

	//HERE:  we need to convert ground trans/rots into nodeTrans/Rots.
	//And then subract the number of ground transforms we just deleted from the start

	for (U32 i=0;i<seq.numKeyframes;i++)
	{
		nodeTranslations[seq.baseTranslation + i].x = groundTranslations[seq.firstGroundFrame + i].x;
		nodeTranslations[seq.baseTranslation + i].y = groundTranslations[seq.firstGroundFrame + i].y;
		//nodeTranslations[seq.baseTranslation + i].z += groundTranslations[seq.firstGroundFrame + i].z;
		//local z within this animation stays here, global z goes to groundTranslations.

		//After this loop is done, then go through and delete all the ground trans/rots and move
		//all the other sequences forward.

		////groundRotations.increment();
		////The old way - grabbing the whole transform.
		////groundRotations[groundRotations.size()-1] = nodeRotations[seq.baseRotation + i];
		////nodeRotations[seq.baseRotation + i].identity();

		////Versus the new way: attempting to isolate rotation on the XY plane (rotation around the Z
		//// axis) from all other rotation.  Plan: get a +Y unit projection, run it through the groundRot
		////quaternion to get hip-node local +Y.  Then drop the Z from that to get a projection on the XY
		////plane.  Then, do a high school geometry thing to find the angle between that and a global
		////+Y unit vector, and there's your angle to rotate around Z.  Do that to an identity quat and
		////you have your groundTransform, multiply the original node transform by the INVERSE of that 
		////and you have your new node transform.  Or so the theory goes...

		//Point3F Y_unit(0,1,0);
		//Point3F new_Y;
		//QuatF groundRot = nodeRotations[seq.baseRotation + i].getQuatF();
		//groundRot.mulP(Y_unit,&new_Y);
		//new_Y.z = 0.0;//Removing the vertical element, to project it onto XY plane.
		//new_Y.normalize();
		//F32 dotProd = mDot(Y_unit,new_Y);
		
		//QuatF rotArc,invRotArc,nodeRot;
		//Point3F negY(0,-1,0);
		////Point3F diff = new_Y - negY;
		////if (diff.len()==0.0){
		//if ((fabs(new_Y.x)<0.001)&&(new_Y.y==-1.0)&&(fabs(new_Y.z)<0.001)) {//(new_Y.y<-0.9999)//?
		//	Con::printf("found the direct 180!");
		//	rotArc.set(0,0,-1,0);
		//} else
		//	rotArc.rotationArc(new_Y,Y_unit);
		//
		//MatrixF eulMat;
		//rotArc.setMatrix(&eulMat);
		//EulerF euler = eulMat.toEuler();

		//Quat16 newGroundRot,newNodeRot;//nodeRotations[seq.baseRotation + i]
		//newGroundRot.set(rotArc);
		//groundRotations[groundRotations.size()-1] = newGroundRot;

		//invRotArc = rotArc;
		//invRotArc.inverse();
		QuatF groundRot,nodeRot;
		nodeRotations[seq.baseRotation + i].getQuatF(&nodeRot);
		QuatF tempRot = nodeRot;

		groundRotations[seq.firstGroundFrame + i].getQuatF(&groundRot);
		nodeRot.mul(tempRot,groundRot);
		nodeRotations[seq.baseRotation + i].set(nodeRot);
	}
	//HERE: now you have to get RID of the actual ground frames and move all remaining sequences forward.
	for (U32 i=seq.firstGroundFrame;i<groundTranslations.size();i++)
	{
		groundTranslations[i] = groundTranslations[i+seq.numGroundFrames];
		groundRotations[i] = groundRotations[i+seq.numGroundFrames];
	}
	for (U32 i=0;i<seq.numGroundFrames;i++)
	{
		groundTranslations.decrement();
		groundRotations.decrement();
	}
	for (U32 i=0;i<sequences.size();i++)
	{
		if (sequences[i].firstGroundFrame > seq.firstGroundFrame)
			sequences[i].firstGroundFrame -= seq.numGroundFrames; 
	}
	seq.numGroundFrames = 0;//seq.numKeyframes;
	return;
}

//This is written for adjusting models that come in from Y up left handed DAE files,
// but it could be used with models from any source.
void TSShape::fixUpAxis(S32 axis)
{//0 = X up, 1 = Y up
	for (U32 i=0;i<defaultTranslations.size();i++)
	{
		Point3F inPos = defaultTranslations[i];
		Point3F outPos;
		if (axis == 0) {//X
			//do later...
		} else if (axis == 1) {//Y
			outPos.x = -inPos.x;
			outPos.y = inPos.z;
			outPos.z = inPos.y;
		}
		defaultTranslations[i] = outPos;
	}
	//Con::printf("shape node %d: %f %f %f changed to %f %f %f",
	//	i,inPos.x,inPos.y,inPos.z,outPos.x,outPos.y,outPos.z);
	//Next flip or rotate the verts in all the meshes.
	Con::printf("shape details: %d",details.size());
	S32 lastSubshape = -1;
	//for (U32 i=0;i<details.size(); i++)
	//{
	//	S32 ss = details[i].subShapeNum;
	//	S32 od = details[i].objectDetailNum;
	//	S32 start = subShapeFirstObject[ss];
	//	S32 end   = subShapeNumObjects[ss] + start;
	//	Con::printf("shape start object: %d, end object %d hasSkinMesh %d",start,end,mHasSkinMesh);
	//	if (ss != lastSubshape) 
	//	{
	//		for (U32 j = start; j < end; j++)
	//		{
	//			if (mHasSkinMesh) {
	//				TSSkinMesh *mesh = dynamic_cast<TSSkinMesh*>(meshes[j]);
	//				Con::printf("meshes %d verts %d ",j,mesh->batchData.initialVerts.size());
	//				for (U32 k=0;k<mesh->batchData.initialVerts.size();k++)
	//				{
	//					Point3F in; in.zero();
	//					Point3F out; out.zero();
	//					in = mesh->batchData.initialVerts[k];
	//					out.x = -in.x;
	//					out.y = in.z;
	//					out.z = in.y;
	//					mesh->batchData.initialVerts[k] = out;
	//				}
	//			} else {
	//				Con::printf("meshes %d verts %d ",j,meshes[j]->verts.size());
	//			}
	//		}
	//	}
	//	lastSubshape = ss;

	//	//TSMesh *mesh = meshes[start];
	//}
}
/*
   struct Detail
   {
      S32 nameIndex;
      S32 subShapeNum;
      S32 objectDetailNum;
      F32 size;
      F32 averageError;
      F32 maxError;
      S32 polyCount;

      /// These values are new autobillboard settings stored
      /// as part of the Detail struct in version 26 and above.
      /// @{
      S32 bbDimension;     ///< The size of the autobillboard image.
      S32 bbDetailLevel;   ///< The detail to render as the autobillboard.
      U32 bbEquatorSteps;  ///< The number of autobillboard images to capture around the equator.
      U32 bbPolarSteps;    ///< The number of autobillboard images to capture along the pole.
      F32 bbPolarAngle;    ///< The angle in radians at which the top/bottom autobillboard images should be displayed.
      U32 bbIncludePoles;  ///< If non-zero then top and bottom images are generated for the autobillboard.
      /// @}
   };
*/
void TSShape::examineShape()
{
	Con::printf("Examining a TSShape: %s",getPath());
	Con::printf("shape details: %d",details.size());
	S32 lastSubshape = -1;
	for (U32 i=0;i<details.size(); i++)
	{
		Con::printf("detail ");
		S32 ss = details[i].subShapeNum;
		S32 od = details[i].objectDetailNum;
		S32 start = subShapeFirstObject[ss];
		S32 end   = subShapeNumObjects[ss] + start;
		Con::printf("shape start object: %d, end object %d subShapeNum %d  objDetailNum %d",
			start,end,ss,od);
		if (ss != lastSubshape)
		{
			for (U32 j = start; j < end; j++)
			{
				if (mHasSkinMesh)
				{
					TSSkinMesh *mesh = dynamic_cast<TSSkinMesh*>(meshes[j]);
					Con::printf("skin mesh %d mesh verts %d initialVerts %d norms %d  initialTransforms %d nodeIndex %d ",
						j,mesh->verts.size(),mesh->batchData.initialVerts.size(),
						mesh->batchData.initialNorms.size(),mesh->batchData.initialTransforms.size(),
						mesh->batchData.nodeIndex.size());
					for (U32 k=0;k<mesh->batchData.initialTransforms.size();k++)
					{
						MatrixF transf = mesh->batchData.initialTransforms[k];
						Point3F pos = transf.getPosition();
						QuatF quat(transf);
						Con::printf("initial transform %d pos %f %f %f  quat %2.3f %2.3f %2.3f %2.3f",
							k,pos.x,pos.y,pos.z,quat.x,quat.y,quat.z,quat.w);


					}
				} else {
					TSMesh *mesh = meshes[j];
					Con::printf("TS mesh %d verts %d  norms %d  tverts %d  tverts2 %d",j,mesh->verts.size(),
						mesh->norms.size(),mesh->tverts.size(),mesh->tverts2.size());
				}
			}
		}
		lastSubshape = ss;
	}
}

//FAIL. 
S32 TSShape::findSequenceByPath(const char *path)
{
	S32 index = -1;
	Con::printf("looking for sequence path: %s",path);
	for (U32 i=0;i<sequences.size();i++)
	{
		Con::printf("comparing against sequence source: %s",sequences[i].sourceData.from);
		if (!strcmp(sequences[i].sourceData.from,path))
			index = i;
	}
	return index;
}

//ultraframe types:
#define ADJUST_NODE_POS 0
#define SET_NODE_POS 1
#define ADJUST_NODE_ROT 2
#define SET_NODE_ROT 3

void TSShape::applyUltraframeSet(ultraframeSet *ufs)
{
	//Tthe first thing we need to do is check to see whether we have a backup of this sequence. If so, that
	//means we have already altered it, and therefore we need to restore it before we alter it again.
	//If not, then we need to make one.
	sequenceBackup seqBackup;
	Sequence *kSeq = &(sequences[ufs->sequence]);
	S32 rot_matters_count;
	rot_matters_count = 0;
	for (U32 i=0;i<nodes.size();i++) 
		if (kSeq->rotationMatters.test(i)) 
			rot_matters_count++;
	
	//First, either make a backup seq, if we don't have one, or reload it if we do, to start fresh.
	S32 backupSeq = -1;
	for (U32 i=0;i<mSequenceBackups.size();i++)
	{
		if (mSequenceBackups[i].index==ufs->sequence)
			backupSeq = i;
	}
	if (backupSeq>=0)
	{
		//Make a restoreSequence() function? Or just do it here.
		for (U32 j=0;j<kSeq->numKeyframes;j++)
		{
			nodeTranslations[kSeq->baseTranslation+j] = mSequenceBackups[backupSeq].nodeTranslations[j];
		}
		for (U32 j=0;j<rot_matters_count;j++)
		{
			for (U32 k=0;k<kSeq->numKeyframes;k++)
			{
				nodeRotations[kSeq->baseRotation+(j*kSeq->numKeyframes)+k] = mSequenceBackups[backupSeq].nodeRotations[(j*kSeq->numKeyframes)+k];
			}
		}
	}
	else 
	{
		//Make a backupSequence() function? Or just do it here.
		seqBackup.index = ufs->sequence;
		for (U32 j=0;j<kSeq->numKeyframes;j++)
		{
			seqBackup.nodeTranslations.increment();
			seqBackup.nodeTranslations.last() = nodeTranslations[kSeq->baseTranslation+j];
		}
		for (U32 j=0;j<rot_matters_count;j++)
		{
			for (U32 k=0;k<kSeq->numKeyframes;k++)
			{
				seqBackup.nodeRotations.increment();
				seqBackup.nodeRotations.last() = nodeRotations[kSeq->baseRotation+(j*kSeq->numKeyframes)+k];
			}
		}
		mSequenceBackups.push_back(seqBackup);
	}

	//Now, we should be ready to apply a full set of changes to the (once again) virginal sequence data.
	
	for (U32 i=0;i<ufs->series.size();i++)
	{
		U32 type = ufs->series[i].type;
		U32 node = ufs->series[i].node;
		S32 mattersNode = getNodeMattersIndex(ufs->sequence,node);
		Vector<ultraframe> ultraframes;
		Con::printf("ultraframe series, type %d node %d",type,node);
		for (U32 j=0;j<ufs->series[i].frames.size();j++)
		{
			ultraframes.push_back(ufs->series[i].frames[j]);

			Con::printf("frame: %d  %f %f %f",ufs->series[i].frames[j].frame,
				ufs->series[i].frames[j].value.x,ufs->series[i].frames[j].value.y,ufs->series[i].frames[j].value.z);
		}
		if (ultraframes.size()==1)
		{
			//we have only a single frame to change


		}
		else 
		{
			//we have more than one, so loop from the first to the second, and then the second to the third,
			//and so on until we reach the end.
			Point3F curVal,startVal,endVal;
			curVal.zero(); startVal.zero(); endVal.zero();
			S32 startFrame,endFrame;
			Point3F newPos,basePos;
			Quat16 newQuat,baseQuat;
			startFrame = endFrame = 0;
			ultraframe *sf,*ef;
			for (U32 j=0;j<ufs->series[i].frames.size()-1;j++)
			{
				

				sf = &(ufs->series[i].frames[j]);
				ef = &(ufs->series[i].frames[j+1]);

				startFrame = sf->frame;
				startVal = sf->value;
				endFrame = ef->frame;
				endVal = ef->value;

				for (U32 k=startFrame;k<=endFrame;k++)
				{
					curVal = startVal + ((endVal - startVal)*((F32)(k-startFrame)/(F32)(endFrame-startFrame)));
					if ((type==ADJUST_NODE_POS) || (type==SET_NODE_POS))
					{
						basePos = seqBackup.nodeTranslations[k];
						if (type==ADJUST_NODE_POS)
							newPos = basePos + curVal;
						else if (type==SET_NODE_POS)
							newPos = curVal;
						nodeTranslations[k+kSeq->baseTranslation] = newPos;
					}
					else if ((type==ADJUST_NODE_ROT) || (type==SET_NODE_ROT))
					{
						//Con::printf("ultraframe mod: frame %d  node %d  value %f %f %f",k,node,curVal.x,curVal.y,curVal.z);
						curVal.x = mDegToRad(curVal.x);
						curVal.y = mDegToRad(curVal.y);
						curVal.z = mDegToRad(curVal.z);
						QuatF q((EulerF)curVal);
						if (type==SET_NODE_ROT)
						{
							newQuat.set(q);
						} else if (type==ADJUST_NODE_ROT) {
							QuatF temp;
							baseQuat = seqBackup.nodeRotations[(mattersNode*kSeq->numKeyframes)+k];
							baseQuat.getQuatF(&temp);
							temp *= q;
							newQuat.set(temp);
						}
						nodeRotations[kSeq->baseRotation+(mattersNode*kSeq->numKeyframes)+k] = newQuat;
					}
				}
			}
		}
	}
}
