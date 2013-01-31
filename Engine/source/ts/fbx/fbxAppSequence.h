//-----------------------------------------------------------------------------
// Fbx-2-DTS
// Copyright (C) 2011 BrokeAss Games, LLC
//-----------------------------------------------------------------------------

#ifndef _FBX_APPSEQUENCE_H_
#define _FBX_APPSEQUENCE_H_

#ifndef _APPSEQUENCE_H_
#include "ts/loader/appSequence.h"
#endif

//class domAnimation_clip;
//class FbxExtension_animation_clip;

class FbxAppSequence : public AppSequence
{
   //const domAnimation_clip*            pClip;
   //FbxExtension_animation_clip*    clipExt;

	KFbxAnimStack *mAnimStack;
	KFbxAnimLayer *mAnimLayer;
	KFbxAnimCurveNode *mCurveNode;

	//void setAnimationActive(const domAnimation* anim, bool active);

public:
   F32   seqStart;
   F32   seqEnd;
	F32	seqDuration;
	S32   seqKeyframes;

   //FbxAppSequence(const domAnimation_clip* clip);
	//FbxAppSequence(KFbxAnimCurveNode* curveNode);
	FbxAppSequence(KFbxAnimStack* animStack);
   ~FbxAppSequence();

   void setActive(bool active);

   //const domAnimation_clip* getClip() const { return pClip; }

   S32 getNumTriggers();
   void getTrigger(S32 index, TSShape::Trigger& trigger);

   const char* getName() const;

   F32 getStart() const { return seqStart; }
   F32 getEnd() const { return seqEnd; }
   void setEnd(F32 end) { seqEnd = end; }

   U32 getFlags() const;
   F32 getPriority();
   F32 getBlendRefTime();
};

#endif // _FBX_APPSEQUENCE_H_
