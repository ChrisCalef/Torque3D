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

#include "platform/platform.h"

#include "ts/fbx/fbxExtensions.h"
#include "ts/fbx/fbxAppSequence.h"

/*
FbxAppSequence::FbxAppSequence(const domAnimation_clip* clip)
   : pClip(clip), clipExt(new FbxExtension_animation_clip(clip))
{
   seqStart = pClip->getStart();
   seqEnd = pClip->getEnd();
}
*/
//FbxAppSequence::FbxAppSequence(KFbxAnimCurveNode* curveNode)
//{
//   //seqStart = pClip->getStart();
//   //seqEnd = pClip->getEnd();
//}

FbxAppSequence::FbxAppSequence(KFbxAnimStack* animStack)
{
	mAnimStack = animStack;
	KTimeSpan timeSpan = mAnimStack->GetLocalTimeSpan();
	KTime kTime = timeSpan.GetDuration();
	seqDuration = (F32)kTime.GetMilliSeconds() / 1000.0 ;//Convert to seconds.
	Con::printf("Making a new FbxAppSequence!  %f ",seqDuration);
	seqStart = 0.0;//Try this for now, fix it if multiple anims need to 
	seqEnd = seqDuration;     //stack after each other.
	seqKeyframes = 0;//Start with a safe small number.
   //seqStart = pClip->getStart();
   //seqEnd = pClip->getEnd();

}

FbxAppSequence::~FbxAppSequence()
{
   //delete clipExt;
}

const char* FbxAppSequence::getName() const
{
   return "TEMP";//_GetNameOrId(pClip);
}

S32 FbxAppSequence::getNumTriggers()
{
   return 0;//clipExt->triggers.size();
}

void FbxAppSequence::getTrigger(S32 index, TSShape::Trigger& trigger)
{
   //trigger.pos = clipExt->triggers[index].time;
   //trigger.state = clipExt->triggers[index].state;
}

U32 FbxAppSequence::getFlags() const
{
   U32 flags = 0;
   //if (clipExt->cyclic) flags |= TSShape::Cyclic;
   //if (clipExt->blend)  flags |= TSShape::Blend;
   return flags;
}

F32 FbxAppSequence::getPriority()
{
   return 0;//clipExt->priority;
}

F32 FbxAppSequence::getBlendRefTime()
{
   return 0;//clipExt->blendReferenceTime;
}

void FbxAppSequence::setActive(bool active)
{
   //for (int iAnim = 0; iAnim < getClip()->getInstance_animation_array().getCount(); iAnim++) {
   //   domAnimation* anim = daeSafeCast<domAnimation>(getClip()->getInstance_animation_array()[iAnim]->getUrl().getElement());
   //   if (anim)
   //      setAnimationActive(anim, active);
   //}
}

//void FbxAppSequence::setAnimationActive(const domAnimation* anim, bool active)
//{
//   // Enabled/disable data channels for this animation
//   for (int iChannel = 0; iChannel < anim->getChannel_array().getCount(); iChannel++) {
//      domChannel* channel = anim->getChannel_array()[iChannel];
//      AnimData* animData = reinterpret_cast<AnimData*>(channel->getUserData());
//      if (animData)
//         animData->enabled = active;
//   }
//
//   // Recurse into child animations
//   for (int iAnim = 0; iAnim < anim->getAnimation_array().getCount(); iAnim++)
//      setAnimationActive(anim->getAnimation_array()[iAnim], active);
//}
