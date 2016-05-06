//-----------------------------------------------------------------------------
// Fbx-2-DTS
// Copyright (C) 2011 BrokeAss Games, LLC
//-----------------------------------------------------------------------------

#ifndef _FBX_EXTENSIONS_H_
#define _FBX_EXTENSIONS_H_

#ifndef _TSSHAPE_LOADER_H_
#include "ts/loader/tsShapeLoader.h"
#endif
#ifndef _FBX_UTILS_H_
#include "ts/fbx/fbxUtils.h"
#endif

//-----------------------------------------------------------------------------
// Fbx allows custom data to be included with many elements using the <extra>
// tag, followed by one or more named technique profiles. eg.
// <some_element>
//   <extra>
//     <technique profile="SOME_PROFILE">
//       <custom_element0>value0</custom_element0>
//       <custom_element1>value1</custom_element1>
//       ...
//     <technique profile="ANOTHER_PROFILE">
//       <custom_element0>value0</custom_element0>
//       <custom_element1>value1</custom_element1>
//       ...
//
// This class provides an easy way to read the custom parameters into a strongly
// typed subclass.
/*
class FbxExtension
{
   // Helper macro to simplify getting named parameters
   #define GET_EXTRA_PARAM(param, defaultVal)   \
      get(#param, param, defaultVal)

protected:
   const domTechnique* pTechnique;

   /// Find the technique with the named profile
   template<class T> const domTechnique* findExtraTechnique(const T* element, const char* name) const
   {
      if (element) {
         for (int iExt = 0; iExt < element->getExtra_array().getCount(); iExt++) {
            for (int iTech = 0; iTech < element->getExtra_array()[iExt]->getTechnique_array().getCount(); iTech++) {
               if (dStrEqual(element->getExtra_array()[iExt]->getTechnique_array()[iTech]->getProfile(), name))
                  return element->getExtra_array()[iExt]->getTechnique_array()[iTech];
            }
         }         
      }
      return NULL;
   }

   /// The <texture> element does not define an extra_array, so need a specialized
   /// version of the template
   const domTechnique* findExtraTechnique(
      const domCommon_color_or_texture_type_complexType::domTexture* element, const char* name) const
   {
      if (element && element->getExtra()) {
         for (int iTech = 0; iTech < element->getExtra()->getTechnique_array().getCount(); iTech++) {
            if (dStrEqual(element->getExtra()->getTechnique_array()[iTech]->getProfile(), name))
               return element->getExtra()->getTechnique_array()[iTech];
         }
      }
      return NULL;
   }

   /// Find the parameter with the given name
   const domAny* findParam(const char* name)
   {
      if (pTechnique) {
         // search the technique contents for the desired parameter
         for (int iParam = 0; iParam < pTechnique->getContents().getCount(); iParam++) {
            const domAny* param = daeSafeCast<domAny>(pTechnique->getContents()[iParam]);
            if (param && !dStrcmp(param->getElementName(), name))
               return param;
         }
      }
      return NULL;
   }

   /// Get the value of the named parameter (use defaultVal if parameter not found)
   template<typename T> void get(const char* name, T& value, T defaultVal)
   {
      value = defaultVal;
      if (const domAny* param = findParam(name))
         value = convert<T>(param->getValue());
   }

   /// Get the value of the named animated parameter (use defaultVal if parameter not found)
   template<typename T> void get(const char* name, AnimatedElement<T>& value, T defaultVal)
   {
      value.defaultVal = defaultVal;
      if (const domAny* param = findParam(name))
         value.element = param;
   }

public:
   FbxExtension() : pTechnique(0) { }
   virtual ~FbxExtension() { }
};

/// Extensions for the <effect> element (and its children)
class FbxExtension_effect : public FbxExtension
{
   // Cached texture transform
   F32            lastAnimTime;
   MatrixF        textureTransform;

public:
   //----------------------------------
   // <effect>
   // MAX3D profile elements
   bool double_sided;

   //----------------------------------
   // <effect>.<profile_COMMON>
   // GOOGLEEARTH profile elements
   //bool double_sided;

   //----------------------------------
   // <effect>.<profile_COMMON>.<technique>.<blinn/phong/lambert>.<diffuse>.<texture>
   // MAYA profile elements
   bool           wrapU, wrapV;
   bool           mirrorU, mirrorV;
   AnimatedFloat  coverageU, coverageV;
   AnimatedFloat  translateFrameU, translateFrameV;
   AnimatedFloat  rotateFrame;
   AnimatedBool   stagger;       // @todo: not supported yet
   AnimatedFloat  repeatU, repeatV;
   AnimatedFloat  offsetU, offsetV;
   AnimatedFloat  rotateUV;
   AnimatedFloat  noiseU, noiseV;

   //----------------------------------
   // <effect>.<profile_COMMON>.<technique>
   // FFbx profile elements
   domFx_sampler2D_common_complexType*   bumpSampler;

public:
   FbxExtension_effect(const domEffect* effect)
      : lastAnimTime(TSShapeLoader::DefaultTime-1), textureTransform(true), bumpSampler(0)
   {
      //----------------------------------
      // <effect>
      // MAX3D profile
      pTechnique = findExtraTechnique(effect, "MAX3D");
      GET_EXTRA_PARAM(double_sided, false);

      //----------------------------------
      // <effect>.<profile_COMMON>
      const domProfile_COMMON* profileCommon = FbxUtils::findEffectCommonProfile(effect);

      // GOOGLEEARTH profile (same double_sided element)
      pTechnique = findExtraTechnique(profileCommon, "GOOGLEEARTH");
      GET_EXTRA_PARAM(double_sided, double_sided);

      //----------------------------------
      // <effect>.<profile_COMMON>.<technique>.<blinn/phong/lambert>.<diffuse>.<texture>
      const domCommon_color_or_texture_type_complexType* domDiffuse = FbxUtils::findEffectDiffuse(effect);
      const domFx_sampler2D_common_complexType* sampler2D = FbxUtils::getTextureSampler(effect, domDiffuse);

      // Use the sampler2D to set default values for wrap/mirror flags
      wrapU = wrapV = true;
      mirrorU = mirrorV = false;
      if (sampler2D) {
         domFx_sampler2D_common_complexType::domWrap_s* wrap_s = sampler2D->getWrap_s();
         domFx_sampler2D_common_complexType::domWrap_t* wrap_t = sampler2D->getWrap_t();

         mirrorU = (wrap_s && wrap_s->getValue() == FX_SAMPLER_WRAP_COMMON_MIRROR);
         wrapU = (mirrorU || !wrap_s || (wrap_s->getValue() == FX_SAMPLER_WRAP_COMMON_WRAP));
         mirrorV = (wrap_t && wrap_t->getValue() == FX_SAMPLER_WRAP_COMMON_MIRROR);
         wrapV = (mirrorV || !wrap_t || (wrap_t->getValue() == FX_SAMPLER_WRAP_COMMON_WRAP));
      }

      // MAYA profile
      pTechnique = findExtraTechnique(domDiffuse ? domDiffuse->getTexture() : 0, "MAYA");
      GET_EXTRA_PARAM(wrapU, wrapU);            GET_EXTRA_PARAM(wrapV, wrapV);
      GET_EXTRA_PARAM(mirrorU, mirrorU);        GET_EXTRA_PARAM(mirrorV, mirrorV);
      GET_EXTRA_PARAM(coverageU, 1.0);          GET_EXTRA_PARAM(coverageV, 1.0);
      GET_EXTRA_PARAM(translateFrameU, 0.0);    GET_EXTRA_PARAM(translateFrameV, 0.0);
      GET_EXTRA_PARAM(rotateFrame, 0.0);
      GET_EXTRA_PARAM(stagger, false);
      GET_EXTRA_PARAM(repeatU, 1.0);            GET_EXTRA_PARAM(repeatV, 1.0);
      GET_EXTRA_PARAM(offsetU, 0.0);            GET_EXTRA_PARAM(offsetV, 0.0);
      GET_EXTRA_PARAM(rotateUV, 0.0);
      GET_EXTRA_PARAM(noiseU, 0.0);             GET_EXTRA_PARAM(noiseV, 0.0);

      // FFbx profile
      if (profileCommon) {
         pTechnique = findExtraTechnique((const domProfile_COMMON::domTechnique*)profileCommon->getTechnique(), "FFbx");
         if (pTechnique) {
            domAny* bump = daeSafeCast<domAny>(const_cast<domTechnique*>(pTechnique)->getChild("bump"));
            if (bump) {
               domAny* bumpTexture = daeSafeCast<domAny>(bump->getChild("texture"));
               if (bumpTexture) {
                  daeSIDResolver resolver(const_cast<domEffect*>(effect), bumpTexture->getAttribute("texture").c_str());
                  domCommon_newparam_type* param = daeSafeCast<domCommon_newparam_type>(resolver.getElement());
                  if (param)
                     bumpSampler = param->getSampler2D();
               }
            }
         }
      }
   }

   /// Check if any of the MAYA texture transform elements are animated within
   /// the interval
   bool animatesTextureTransform(F32 start, F32 end);

   /// Apply the MAYA texture transform to the given UV coordinates
   void applyTextureTransform(Point2F& uv, F32 time);
};

/// Extensions for the <node> element
class FbxExtension_node : public FbxExtension
{
public:
   // FFbx or OpenFbx profile elements
   AnimatedFloat visibility;
   const char* user_properties;

   FbxExtension_node(const domNode* node)
   {
      // FFbx profile
      pTechnique = findExtraTechnique(node, "FFbx");
      GET_EXTRA_PARAM(visibility, 1.0);
      GET_EXTRA_PARAM(user_properties, "");

      // OpenFbx profile
      pTechnique = findExtraTechnique(node, "OpenFbx");
      GET_EXTRA_PARAM(user_properties, user_properties);
   }
};

/// Extensions for the <geometry> element
class FbxExtension_geometry : public FbxExtension
{
public:
   // MAYA profile elements
   bool double_sided;

   FbxExtension_geometry(const domGeometry* geometry)
   {
      // MAYA profile
      pTechnique = findExtraTechnique(geometry, "MAYA");
      GET_EXTRA_PARAM(double_sided, false);
   }
};

// Extensions for the <animation_clip> element
class FbxExtension_animation_clip : public FbxExtension
{
public:
   struct Trigger {
      F32 time;
      S32 state;
   };

   // Torque profile elements (none of these are animatable)
   S32 num_triggers;
   Vector<Trigger> triggers;
   bool cyclic;
   bool blend;
   F32 blendReferenceTime;
   F32 priority;

   FbxExtension_animation_clip(const domAnimation_clip* clip)
   {
      // Torque profile
      pTechnique = findExtraTechnique(clip, "Torque");
      GET_EXTRA_PARAM(num_triggers, 0);
      for (int iTrigger = 0; iTrigger < num_triggers; iTrigger++) {
         triggers.increment();
         get(avar("trigger_time%d", iTrigger), triggers.last().time, 0.0f);
         get(avar("trigger_state%d", iTrigger), triggers.last().state, 0);
      }
      GET_EXTRA_PARAM(cyclic, false);
      GET_EXTRA_PARAM(blend, false);
      GET_EXTRA_PARAM(blendReferenceTime, 0.0f);
      GET_EXTRA_PARAM(priority, 5.0f);
   }
};
*/
#endif // _FBX_EXTENSIONS_H_
