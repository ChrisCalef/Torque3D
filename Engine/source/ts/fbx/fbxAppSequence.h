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
