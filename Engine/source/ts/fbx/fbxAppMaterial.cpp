//-----------------------------------------------------------------------------
// Fbx-2-DTS
// Copyright (C) 2011 BrokeAss Games, LLC
//-----------------------------------------------------------------------------

#include "platform/platform.h"

#include "ts/loader/tsShapeLoader.h"
#include "ts/fbx/fbxAppMaterial.h"
#include "ts/fbx/fbxUtils.h"
#include "ts/tsMaterialList.h"

using namespace FbxUtils;
//
//String cleanString(const String& str)
//{
//   String cleanStr(str);
//
//   // Replace invalid characters with underscores
//   const String badChars(" -,.+=*/");
//   for (String::SizeType i = 0; i < badChars.length(); i++)
//      cleanStr.replace(badChars[i], '_');
//
//   // Prefix with an underscore if string starts with a number
//   if ((cleanStr[0] >= '0') && (cleanStr[0] <= '9'))
//      cleanStr.insert(0, '_');
//
//   return cleanStr;
//}


//------------------------------------------------------------------------------

FbxAppMaterial::FbxAppMaterial(const char* matName)
//:  mat(0),
   //effect(0)
   //effectExt(0)
{
   name = matName;

   // Set some defaults
   flags |= TSMaterialList::S_Wrap;
   flags |= TSMaterialList::T_Wrap;

   diffuseColor = ColorF::ONE;
   specularColor = ColorF::ONE;
   specularPower = 8.0f;
   doubleSided = false;
}
/*
FbxAppMaterial::FbxAppMaterial(const domMaterial *pMat)
:  mat(pMat),
   diffuseColor(ColorF::ONE),
   specularColor(ColorF::ONE),
   specularPower(8.0f),
   doubleSided(false)
{
   // Get the effect element for this material
   effect = daeSafeCast<domEffect>(mat->getInstance_effect()->getUrl().getElement());
   effectExt = new FbxExtension_effect(effect);

   // Get the <profile_COMMON>, <diffuse> and <specular> elements
   const domProfile_COMMON* commonProfile = FbxUtils::findEffectCommonProfile(effect);
   const domCommon_color_or_texture_type_complexType* domDiffuse = findEffectDiffuse(effect);
   const domCommon_color_or_texture_type_complexType* domSpecular = findEffectSpecular(effect);

   // Wrap flags
   if (effectExt->wrapU)
      flags |= TSMaterialList::S_Wrap;
   if (effectExt->wrapV)
      flags |= TSMaterialList::T_Wrap;

   // Set material attributes
   if (commonProfile) {

      F32 transparency = 0.0f;
      if (commonProfile->getTechnique()->getConstant()) {
         const domProfile_COMMON::domTechnique::domConstant* constant = commonProfile->getTechnique()->getConstant();
         diffuseColor.set(1.0f, 1.0f, 1.0f, 1.0f);
         resolveColor(constant->getReflective(), &specularColor);
         resolveFloat(constant->getReflectivity(), &specularPower);
         resolveTransparency(constant, &transparency);
      }
      else if (commonProfile->getTechnique()->getLambert()) {
         const domProfile_COMMON::domTechnique::domLambert* lambert = commonProfile->getTechnique()->getLambert();
         resolveColor(lambert->getDiffuse(), &diffuseColor);
         resolveColor(lambert->getReflective(), &specularColor);
         resolveFloat(lambert->getReflectivity(), &specularPower);
         resolveTransparency(lambert, &transparency);
      }
      else if (commonProfile->getTechnique()->getPhong()) {
         const domProfile_COMMON::domTechnique::domPhong* phong = commonProfile->getTechnique()->getPhong();
         resolveColor(phong->getDiffuse(), &diffuseColor);
         resolveColor(phong->getSpecular(), &specularColor);
         resolveFloat(phong->getShininess(), &specularPower);
         resolveTransparency(phong, &transparency);
      }
      else if (commonProfile->getTechnique()->getBlinn()) {
         const domProfile_COMMON::domTechnique::domBlinn* blinn = commonProfile->getTechnique()->getBlinn();
         resolveColor(blinn->getDiffuse(), &diffuseColor);
         resolveColor(blinn->getSpecular(), &specularColor);
         resolveFloat(blinn->getShininess(), &specularPower);
         resolveTransparency(blinn, &transparency);
      }

      // Normalize specularPower (1-128). Values > 1 are assumed to be
      // already normalized.
      if (specularPower <= 1.0f)
         specularPower *= 128;
      specularPower = mClampF(specularPower, 1.0f, 128.0f);

      // Set translucency
      if (transparency != 0.0f) {
         flags |= TSMaterialList::Translucent;
         if (transparency > 1.0f) {
            flags |= TSMaterialList::Additive;
            diffuseColor.alpha = transparency - 1.0f;
         }
         else if (transparency < 0.0f) {
            flags |= TSMaterialList::Subtractive;
            diffuseColor.alpha = -transparency;
         }
         else {
            diffuseColor.alpha = transparency;
         }
      }
      else
         diffuseColor.alpha = 1.0f;
   }

   // Double-sided flag
   doubleSided = effectExt->double_sided;

   // Get the paths for the various textures => Fbx indirection at its finest!
   // <texture>.<newparam>.<sampler2D>.<source>.<newparam>.<surface>.<init_from>.<image>.<init_from>
   diffuseMap = getSamplerImagePath(effect, getTextureSampler(effect, domDiffuse));
   specularMap = getSamplerImagePath(effect, getTextureSampler(effect, domSpecular));
   normalMap = getSamplerImagePath(effect, effectExt->bumpSampler);

   // Set the material name
   name = FbxUtils::getOptions().matNamePrefix;
   if ( FbxUtils::getOptions().useDiffuseNames )
   {
      Torque::Path diffusePath( diffuseMap );
      name += diffusePath.getFileName();
   }
   else
   {
      name += _GetNameOrId(mat);
   }
}

void FbxAppMaterial::resolveFloat(const domCommon_float_or_param_type* value, F32* dst)
{
   if (value && value->getFloat()) {
      *dst = value->getFloat()->getValue();
   }
}

void FbxAppMaterial::resolveColor(const domCommon_color_or_texture_type* value, ColorF* dst)
{
   if (value && value->getColor()) {
      dst->red = value->getColor()->getValue()[0];
      dst->green = value->getColor()->getValue()[1];
      dst->blue = value->getColor()->getValue()[2];
      dst->alpha = value->getColor()->getValue()[3];
   }
}

// Generate a script Material object
#define writeLine(str) { stream.write((int)dStrlen(str), str); stream.write(2, "\r\n"); }

void writeValue(Stream & stream, const char *name, const char *value)
{
   writeLine(avar("\t%s = \"%s\";", name, value));
}

void writeValue(Stream & stream, const char *name, bool value)
{
   writeLine(avar("\t%s = %s;", name, value ? "true" : "false"));
}

void writeValue(Stream & stream, const char *name, F32 value)
{
   writeLine(avar("\t%s = %g;", name, value));
}

void writeValue(Stream & stream, const char *name, const ColorF& color)
{
   writeLine(avar("\t%s = \"%g %g %g %g\";", name, color.red, color.green, color.blue, color.alpha));
}

void FbxAppMaterial::write(Stream & stream)
{
   // The filename and material name are used as TorqueScript identifiers, so
   // clean them up first
   String cleanFile = cleanString(TSShapeLoader::getShapePath().getFileName());
   String cleanName = cleanString(getName());

   // Determine the blend operation for this material
   const char* blendOp = (flags & TSMaterialList::Translucent) ? "LerpAlpha" : "None";
   if (flags & TSMaterialList::Additive)
      blendOp = "Add";
   else if (flags & TSMaterialList::Subtractive)
      blendOp = "Sub";

   writeLine(avar("singleton Material(%s_%s)", cleanFile.c_str(), cleanName.c_str()));
   writeLine("{");

   writeValue(stream, "mapTo", getName().c_str());
   writeLine("");

   writeValue(stream, "diffuseMap[0]", diffuseMap.c_str());
   writeValue(stream, "normalMap[0]", normalMap.c_str());
   writeValue(stream, "specularMap[0]", specularMap.c_str());
   writeLine("");

   writeValue(stream, "diffuseColor[0]", diffuseColor);
   writeValue(stream, "specular[0]", specularColor);
   writeValue(stream, "specularPower[0]", specularPower);
   writeLine("");

   writeValue(stream, "doubleSided", doubleSided);
   writeValue(stream, "translucent", (bool)(flags & TSMaterialList::Translucent));
   writeValue(stream, "translucentBlendOp", blendOp);

   writeLine("};");
   writeLine("");
}
*/