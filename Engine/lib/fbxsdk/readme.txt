================================================================================

                                     README

              Autodesk FBX SDK 2011.3.1 release, September 29th 2010
              ------------------------------------------------------


Welcome to the FBX SDK 2011.3.1 release! This document includes the latest
changes since the version 2011.3 of the Autodesk FBX SDK.

For more information, please visit us at http://www.autodesk.com/fbx/

Sincerely,
the Autodesk FBX team

================================================================================


TABLE OF CONTENT
----------------

    1. New And Deprecated Features
    2. Fixed And Known Issues
    3. Release Notes From Previous Releases
    4. Legal Disclaimer 


1. NEW AND DEPRECATED FEATURES
------------------------------

1.1 New Features

    No new features in this release.

1.2 Deprecated Features

    * All functions related to local and global "state" in the KFbxNode
      declaration were removed since their implementation didn't work anymore
      since the last release. They should have been set to deprecated within
      the header files, but they were forgotten.


2. FIXED AND KNOWN ISSUES
-------------------------

2.1 Fixed Issues

    * The unroll filter was inconsistent when applying its process on curves
      data, this has been corrected.

    * Fixed a crash in the function EmulateNormalsByPolygonVertex.
    
    * Visual Studio 2010 builds were reporting missing PDB files, this has been
      corrected.
      
    * Importing various .OBJ file sometime resulted in much more vertices, this
      is now working as intended.
      
    * The mesh triangulation method has been re-written for a much more robust
      algorithm. Now it can handle non-concave shapes. The method can be called
      using TriangulateMeshAdvanced.
      
    * A conversion error from Euler to Quaternion has been corrected, it is now
      much more robust for corner cases.

2.2 Known Issues

    * The static library version of the FBX SDK only ship with SECURE_SCL=0 for
      the moment. In the next major release, we intend to change this back to
      the default value, which leaves it defined to 1.


3. RELEASE NOTES FROM PREVIOUS RELEASES
---------------------------------------

2011.3

    * The FBX SDK is now also ship in a dynamic library package format. To link
      against the dynamic version, users will have to define KFBX_DLLINFO in
      their project preprocessor settings. The static version still doesn't
      require any preprocessor to be defined.

    * Augmented the KFbxCache class to correctly support multiple data channels.

    * Stereo Cameras now correctly support aim/up.
    
    * Added three new functions to the KFbxAnimEvaluator class :
    
      KFbxVector4& GetNodeLocalTranslation(...)
      KFbxVector4& GetNodeLocalRotation(...)
      KFbxVector4& GetNodeLocalScaling(...)
      
      Allow users to calculate the local translation, rotation and scaling as it
      was done in previous versions of the FBX SDK. On purpose, these new
      functions will not take pre/post rotations, offsets and pivots in
      consideration, but they still will consider translation, rotation and
      scaling limits.

    * Added the new KFbxCachedEffect node attribute. These are used to store
      other kind of vertex (or position) caching.

    * Fixed a crash at import time for various legacy FBX files when importing
      animation curves data.

    * Some UV Sets were lost in very specific cases during export, this has
      been corrected.

    * Fixed an issue with node's local transform calculation. Now it should
      correctly return the result of ParentGlobal.Inverse * Global.

    * Protein 2.0 Materials are now extract in the .fbm folder along the .fbx
      file first, rather than the user's operating system temporary folder.
      
    * The following files contain many newly deprecated calls. Please open them
      and search for the macro K_DEPRECATED to find out. Because the list is so
      big, it will not be listed here.
      
      kfbxconstraint.h, kfbxconstraintaim.h, kfbxconstraintparent.h,
      kfbxgeometry.h, kfbxnode.h, kfbxscene.h, kfbxkfcurvefilters.h,
      kfbxdocument.h, kfbxreaderfbx.h, kfbxreaderfbx6.h, kfbximporter.h,
      kfbxproperty.h

2011.2

    * Officially dropped support for PowerPC architecture. Universal binaries
      found in MacOS builds will now only contain 32/64-bit variances.

    * Fixed a crash when importing some legacy MotionBuilder FBX 5.x files.
    
    * Corrected the computation of the smoothing group for in mesh triangulation
      function.
    
    * Fixed Localization problems on the DAE, DFX and OBJ readers/writers.
    
    * Extended the file size limit for FBX files from 2GB to 4GB.
    
    * Augmented the Reference Documentation for a certain number of classes. For
      example, check out KFbxNode or KFbxObject and tell us what you think! :)

2011.1

    * Removed the KFbxTakeNodeContainer class. This was done with the redesign
      of the animation system.

    * A whole new set of classes are now available to evaluate animation with
      the FBX SDK. For more information, please look at the reference
      documentation for KFbxAnimStack, KFbxAnimLayer, KFbxAnimCurve,
      KFbxAnimCurveNode, KFbxAnimEvaluator and KFbxAnimEvalClassic classes. Also
      the evaluation result will now be stored outside the KFbxNode class and
      only created on demand, resulting in a much smaller memory footprint.

    * Removed all needed preprocessor defines to be able to correctly link with
      the static version of the FBX SDK. Namely, those defines were K_PLUGIN,
      K_FBXSDK and K_NODLL.
      
    * The FBX file format as now been upgraded to our latest new technology,
      FBX 7.1! This new FBX file format allow for much more flexibility,
      supporting any number of instances, connections by GUID, reduced file size
      with compression, embedding in ASCII files and much more!
      
    * The KFbxSystemUnit class changed so that it doesn't modify the multiplier
      parameter. Now it is simply carried along in the FBX file.
      
    * A Python Binding for FBX SDK has been released for the first time! In this
      first release, only the most basic functions to allow import/export and
      scene traversal and property query has been exposed. More will be exposed
      later on when we gather more feedback from user experience.
      
2010.2

    * Improved processing speed of the function to retrieve polygon's indexes in
      the mesh class KFbxMesh.
      
    * Removed SetFileFormat on all classes that inherit from KFbxImporter and
      KFbxExporter. Instead, the file format can be overwritten in the
      Initialize functions. By default, the file format will now be auto-
      matically be detected.
      
    * Extended the KFbxMesh class to support standard mesh smoothing. We are
      referring to edge/vertex creases, mesh smoothness, division levels,
      subdivisions, continuity and border/edge preservation.

    * Added Stereo Cameras support via the KFbxCameraStereo class.

    * Added Display Layer and Selection Sets support via the KFbxDisplayLayer
      and KFbxSelectionSet classes respectively.

    * Fixed an issue preventing the use of the slash ( / ) character in property
      names.
      
    * Fixed a stack overflow error in KFbxRenamingStrategy.
    
    * Added support for many new texture blend mode. See KFbxLayerElementTexture
      for more information.

    * Files not ending in .fbx but that still contain FBX formatting can now
      successfully be opened with the FBX SDK.
      
    * Properties can now be destroyed with KFbxProperty::Destroy().

    * Fixed FBX 5.x reader to correctly set pivot information when importing
      legacy files.
      
2010.0

    * Dropped support for Microsoft Visual Studio 2003 libraries.
    
    * Many, many issues fixed. Please refer to previous readme versions for more
      details.
      
2009.x

    * KFbxCache class supports int array.
    
    * Added the Subdivision Display Smoothness to the KFbxSubdiv class.
    
    * Added the optional argument to the IsValid() method in the KFbxTrimSurface
      class to skip the check of boundary curves CV's overlaps.
      
    * Re-factoring of the KFbxCamera class.
    
    * Updates to class documentation.

    * Added methods and properties to manipulate front/back planes & plates.
    
    * Deprecated ECameraBackgroundDrawingMode type and replaced with
      ECameraPlateDrawingMode
    
    * Deprecated ECameraBackgroundPlacementOptions. This has been replaced with
      the individual properties: FitImage, Center, KeepRatio and Crop.
    
    * Deprecated GetBackgroundPlacementOptions() since now the flags are stored
      in the above mentioned properties.
    
    * Deprecated SetViewFrustum(bool pEnable), use SetViewNearFarPlanes()
    
    * Deprecated GetViewFrustum(), use GetViewNearFarPlanes()
    
    * Support of non-convex polygons in the triangulation algorithms.
    
    * Overload of the new operator is not possible anymore (the
      FBXSDK_OVERLOAD_NEW_OPERATOR macro has been removed). The usage of the
      custom memory allocator can only be achieve by using the
      KFbxMemoryAllocator class. See the ExportScene05 for an implementation
      example.
    
    * Enhanced algorithm for smoothing groups generation.
    
    * Support of displacement map channel.
    
    * The class KFbxStreamOptions is now obsolete and is gradually being
      replaced by the class KFbxIOSettings.
    
    * Added KFbxConstraintCustom class.
    
    * Added KFbxContainerTemplate class.
    
    * Added KFbxSubdiv class.
    
    * Added KFbxEmbeddedFilesAccumulator class.
    
    * Adjusted tangents to stay closer to the real value when the weight gets
      ridiculously small.
    
    * Fixed Collada plug-in to handle operating system locale. Depending on the
      locale, the decimal point for numbers may have been represented with the
      comma instead of the point causing parsing errors.
      
    * Fixed support for the floor contact to the KFbxCharacter.
    
    * Fixed infinite loop when loading .obj files on MAC OS.
    
    * Removed some more memory leaks.
    
    * Added the HasDefaultValue(KFbxProperty&) function to check if a property
      value has changed from its default one.
    
    * Added the BumpFactor property to the SurfaceMaterial class.
    
    * Defined plug-ins of plug-ins interface in the fbxsdk manager. See the
      Autodesk FBX SDK PRW readme file for more details.
    
    * Re-factoring of the renaming strategy object.
    
    * Removed unused eCONSTRAINT from the KFbxNodeAttribute.
    
    * Deprecated FillNodeArray and FillNodeArrayRecursive
    
    * Overwrite empty relative filename in texture objects with the correct
      value.
    
    * Fix for internal TRS cache so it correctly get reset when changing takes.
    
    * Bug fixes in the Collada reader/writer.
    
    * Fixed a bug that was causing the loss of Shape animation on NURBS objects.
    
    * Fixed the fact that the layers were losing their name after a clone.
    
    * Corrections for pivot conversion functions:
    
      - Set source pivot to ACTIVE in function ResetPivotSetAndConvertAnimation.
      
      - Update default transformation values to match the results of the pivot
        conversion functions.
      
    * Fixed the endless loop in the RemoveChar() method of the KString class.
    
    * Fixed default values in the KFbxCharacter structure.


4. LEGAL DISCLAIMER
-------------------

Autodesk and FBX are registered trademarks or trademarks of Autodesk, Inc., in
the USA and/or other countries. All other brand names, product names, or trade-
marks belong to their respective holders.

                  (C) 2010 Autodesk, Inc. All Rights Reserved.

================================================================================
