<<<<<<< HEAD
Torque 3D v3.6.3 - PhysX 3.3 Advanced
=======
mountObjectEx
-------------
This branch uses the ToNode/FromNode model described [here](http://www.garagegames.com/community/blogs/view/20581) to expand mounting to any named node in a model. SceneObject has three new console methods for mounting objects.

bool mountObjectEx(SceneObject* objB, const char *toNode, const char *fromNode, TransformF txfm)

Mounts objB to this object at the desired node with fromNode on objB aligned to toNode and offset by txfm.
* objB Object to mount onto us.
* toNode   (optional) Name of the node to mount to. If ommitted, objB will mount to our origin.
* fromNode (optional) Name of the node on objB to align with toNode. If ommitted, the origin of objB will be used.
* txfm     (optional) mount offset transform.
* Returns true if successful, false if failed (objB is not valid).

S32 getNodeObjectEx(const char *nodeName)

Gets the object ID of the first object mounted to the named node.

const char *getMountedObjectNodeEx(S32 Index)

Gets the name of the node that the object at mount index is mounted to.

Scripting Example
```
    %tsExMount = new TSStatic() {
       shapeName = "art/shapes/weapons/Lurker/TP_Lurker.DAE";
       scale = "2 2 2";
    };
    %obj.mountObjectEx(%tsExMount, "Hub0", "MuzzlePoint");
```


Torque 3D v3.5.1
================
>>>>>>> OTHGMars/mount_object_ex

Sample Project Available At:

https://github.com/andr3wmac/Torque3D-PhysX-Samples

Setting up PhysX 3.3 using the Torque 3D Project Manager
------------------------------------------
 - You can find a pre compiled binary of the Torque3D Project Manager that supports PhysX 3.3 here: http://www.narivtech.com/downloads/T3DProjectManager-2-1-devel.zip and source code here: https://github.com/rextimmy/Torque3D-ProjectManager/tree/development
 - For the Project Manager to find PhysX 3.3 SDK you have two options 1)Create an environment variable called TORQUE_PHYSX3_PATH and have that pointing to the location you installed the SDK 2)Place the SDK into a folder called "Program Files"/NVIDIA Corporation/NVIDIA PhysX SDK/v3.3.0_win
 - Simply choose PhysX 3.3 physics from the modules list in the project manager and everything should be automatically taken care of.

Setting up PhysX 3.3 manually
------------------------------------------

 - You will need the latest SDK from NVIDIA. This requires signing up for their developer program. If you don't already have access to their developer site then sign up now as access is not immediate.
 - Set up a standard Torque3D project, don't include any PhysX or Bullet, just regular Torque Physics in project manager options (if you're using it)
 - Generate Projects and open the source code in Visual Studio ( or the IDE of your choice )
 - In the solution explorer in the DLL for your project you should find Source Files -> Engine -> T3D -> physics
 - Add a new filter "physx3" and then right click on it and add existing item
 - Add all the files found under Engine\Source\T3D\physics\physx3\
 - Now you need to add the PhysX SDK. 
 - Under the properties for the DLL project, under Linker -> Additional Library Directories add the lib\win32 directory for the PhysX 3.3 SDK. For example, mine is in: C:\Program Files (x86)\NVIDIA Corporation\NVIDIA PhysX SDK\v3.3.0_win\Lib\win32
 - In the same window under C/C++ you should see Additional Include Directories, you need to add the Include directory for the PhysX 3.3 SDK. For example, mine is in: C:\Program Files %28x86%29\NVIDIA Corporation\NVIDIA PhysX SDK\v3.3.0_win\Include
 - You should now be able to compile now without any issues.

The following libraries will also be needed:
 
Release , Debug

 - PhysX3_x86.lib,PhysX3CHECKED_x86.lib
 - PhysX3Common_x86.lib,PhysX3CommonCHECKED_x86.lib
 - PhysX3Extensions.lib,PhysX3ExtensionsCHECKED.lib
 - PhysX3Cooking_x86.lib,PhysX3CookingCHECKED_x86.lib
 - PxTask.lib,PxTaskCHECKED.lib
 - PhysX3CharacterKinematic_x86.lib,PhysX3CharacterKinematicCHECKED_x86.lib
 - PhysXVisualDebuggerSDK.lib, PhysXVisualDebuggerSDKCHECKED.lib
 - PhysXProfileSDK.lib, PhysXProfileSDKCHECKED.lib

With debug build feel free to change CHECKED to DEBUG if you prefer but it will still require the CHECKED dll's though.
 
Running a project
=======
* [Torque 3D 3.6 Full template](http://mit.garagegames.com/Torque3D-3-6-1-FullTemplate.zip), which contains precompiled binaries.
* [Complete Torque 3D 3.6 zip package](http://mit.garagegames.com/Torque3D-3-6-1.zip) containing the contents of this repository.
* [Windows binaries for 3.6.3](https://github.com/GarageGames/Torque3D/releases/tag/v3.6.3) which you can drop into your existing script projects.
* [Torque 3D Project Manager v2.1](http://mit.garagegames.com/T3DProjectManager-2-1.zip) on its own for use in your T3D forks.

If you're looking for an older release see the [Torque 3D Archive](https://github.com/GarageGames/Torque3D/wiki/Torque-3D-Archive)

Creating a New Project Based on a Template
------------------------------------------

 - To run a release project you will need the following from the SDK bin folder:
   1. PhysX3_x86.dll
   2. PhysX3CharacterKinematic_x86.dll
   3. PhysX3Common_x86.dll
   4. PhysX3Cooking_x86.dll
   
 - To run a debug project you will need the following from the SDK bin folder:
   1. PhysX3CHECKED_x86.dll
   2. nvToolsExt32_1.dll
   3. PhysX3CookingCHECKED_x86.dll
   4. PhysX3CommonCHECKED_x86.dll
   5. PhysX3CharacterKinematicCHECKED_x86.dll
 
Place these files along side the exe and this should get you up and running.



BadBehavior
==========
Behavior Tree System for Torque 3D
----------------------------------
### Introduction

The BadBehavior system is an experimental behavior tree implementation and editor for the [Torque3D](https://github.com/GarageGames/Torque3D) game engine. BadBehavior is very much a work in progress, but feel free to mess around with it and see what you can make it do.

### Features

* Implemented in C++ for efficiency.
* Supports the common behavior tree node types including sequences, selectors, parallels and a range of decorators.
* Shared behavior trees - a single behavior tree definition can be used by multiple objects.
* Behavior trees can be used by any type of SimObject
* Trees can be assembled from smaller sub-trees in a modular fashion
* A multi-page graphical editor with undo/redo support
* Flexible behavior leaf nodes:
  * Actions/conditions can be created entirely in the editor for rapid prototyping
  * Structured behaviors can be written in script
  * Performance critical behaviors can be written entirely in C++
* Self contained - no changes to existing engine classes

### Further information

For tutorials and guides on how to use BadBehavior in your project, refer to:
* BadBehavior [GitHub Wiki](https://github.com/BadBehavior/BadBehavior_T3D/wiki)
* BadBehavior [GitHub page](http://badbehavior.github.io/BadBehavior_T3D)

For information specific to the Torque 3D game engine:
* Torque 3D [main repository](https://github.com/GarageGames/Torque3D)
* Torque 3D [GitHub Wiki](https://github.com/GarageGames/Torque3D/wiki)

License
-------

The BadBehavior system is Copyright (c) 2014 Guy Allard

Torque 3D is Copyright (c) 2012 GarageGames, LLC

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to
deal in the Software without restriction, including without limitation the
rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
>>>>>>> badbehave/master


THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
=======
# Torque 3D

MIT Licensed Open Source version of [Torque 3D](http://torque3d.org) from [GarageGames](http://www.garagegames.com)

## More Information

* [Homepage](http://torque3d.org)
* [Torque 3D wiki](http://wiki.torque3d.org)
* [Community forum](http://forums.torque3d.org)
* [GarageGames forum](http://www.garagegames.com/community/forums)
* [GarageGames professional services](http://services.garagegames.com/)

## Pre-compiled Version

In addition to GitHub we also have a couple of pre-packaged files for you to download if you would prefer to not compile the code yourself.
They are available from the [downloads](http://wiki.torque3d.org/main:downloads) page on the wiki.

## Related repositories

* [Torque 3D main repository](https://github.com/GarageGames/Torque3D) (you are here!)
* [Project Manager repository](https://github.com/GarageGames/Torque3D-ProjectManager)
* [Offline documentation repository](https://github.com/GarageGames/Torque3D-Documentation)

# License

    Copyright (c) 2012 GarageGames, LLC

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
    deal in the Software without restriction, including without limitation the
    rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
    sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:
    
    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.
    
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
    IN THE SOFTWARE.
