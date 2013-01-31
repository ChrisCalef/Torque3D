
    
function FbxImportTreeView::onDefineIcons(%this)
{
   // Set the tree view icon indices and texture paths
   %this._imageNone = 0;
   %this._imageNode = 1;
   %this._imageMesh = 2;
   %this._imageMaterial = 3;
   %this._imageLight = 4;
   %this._imageAnimation = 5;
   %this._imageExNode = 6;
   %this._imageExMaterial = 7;

   %icons = ":" @                                                    // no icon
            "tools/gui/images/FbxImport/iconNode:" @             // normal node
            "tools/gui/images/FbxImport/iconMesh:" @             // mesh
            "tools/gui/images/FbxImport/iconMaterial:" @         // new material
            "tools/gui/images/FbxImport/iconLight:" @            // light
            "tools/gui/images/FbxImport/iconAnimation:" @        // sequence
            "tools/gui/images/FbxImport/iconIgnoreNode:" @       // ignored node
            "tools/gui/images/FbxImport/iconExistingMaterial";   // existing material

   %this.buildIconTable( %icons );
}

function FbxImportDlg::showDialog(%this, %shapePath, %cmd)
{
   %this.path = %shapePath;
   %this.cmd = %cmd;

   // Only allow loading lights if creating a new scene object
   %canLoadLights = (strstr(%this.cmd, "EWCreatorWindow.create") != -1);

   // Check for an existing TSShapeConstructor object. Need to exec the script
   // manually as the FBX resource may not have been loaded yet
   %csPath = filePath(%this.path) @ "/" @ fileBase(%this.path) @ ".cs";
   if (isFile(%csPath))
      exec(%csPath);

   %this.constructor = ShapeEditor.findConstructor(%this.path);

   // Only show the import dialog if required. Note that 'enumFbxScene' will
   // fail if the Fbx file is missing, or a cached.dts is available.
   $Fbx::forceLoadFBX = EditorSettings.value("forceLoadFBX");
   if ( (fileExt(%shapePath) $= ".dts") ||
      !enumFbxForImport(%shapePath, FbxImportTreeView) )
   {
      eval(%cmd);
      $Fbx::forceLoadFBX = false;

      // Load lights from the FBX if possible
      if (%canLoadLights && (%this.constructor > 0) && (%this.constructor.loadLights == 1))
         %this.loadLights();

      return;
   }
   $Fbx::forceLoadFBX = false;

   // Initialise GUI
   FbxImportTreeView.onDefineIcons();

   %this-->window.text = "Fbx Import:" SPC %this.path;

   %this-->upAxis.clear();
   %this-->upAxis.add("X_AXIS", 1);
   %this-->upAxis.add("Y_AXIS", 2);
   %this-->upAxis.add("Z_AXIS", 3);

   %this-->lodType.clear();
   %this-->lodType.add("DetectDTS", 1);
   %this-->lodType.add("SingleSize", 2);
   %this-->lodType.add("TrailingNumber", 3);

   %this-->loadLights.setActive(%canLoadLights);

   // Set model details
   %this-->nodes.setText(FbxImportTreeView._nodeCount);
   %this-->meshes.setText(FbxImportTreeView._meshCount);
   %this-->polygons.setText(FbxImportTreeView._polygonCount);
   %this-->materials.setText(FbxImportTreeView._materialCount);
   %this-->lights.setText(FbxImportTreeView._lightCount);
   %this-->animations.setText(FbxImportTreeView._animCount);

   %this.updateOverrideUpAxis(false);
   %this.updateOverrideScale(false);

   if (%this.constructor > 0)
   {
      if (%this.constructor.upAxis !$= "DEFAULT")
      {
         %this-->overrideUpAxis.setValue(1);
         %this-->upAxis.setText(%this.constructor.upAxis);
         %this.updateOverrideUpAxis(true);
      }
      if (%this.constructor.unit > 0)
      {
         %this-->overrideScale.setValue(1);
         %this-->scale.setText(%this.constructor.unit);
         %this.updateOverrideScale(true);
      }

      %this-->lodType.setText(%this.constructor.lodType);
      %this-->singleDetailSize.setText(%this.constructor.singleDetailSize);
      %this-->materialPrefix.setText(%this.constructor.matNamePrefix);
      %this-->alwaysImport.setText(strreplace(%this.constructor.alwaysImport, "\t", ";"));
      %this-->neverImport.setText(strreplace(%this.constructor.neverImport, "\t", ";"));
      %this-->alwaysImportMesh.setText(strreplace(%this.constructor.alwaysImportMesh, "\t", ";"));
      %this-->neverImportMesh.setText(strreplace(%this.constructor.neverImportMesh, "\t", ";"));
      %this-->ignoreNodeScale.setValue(%this.constructor.ignoreNodeScale);
      %this-->adjustCenter.setValue(%this.constructor.adjustCenter);
      %this-->adjustFloor.setValue(%this.constructor.adjustFloor);
      %this-->forceUpdateMaterials.setValue(%this.constructor.forceUpdateMaterials);
      %this-->loadLights.setValue(%this.constructor.loadLights);
   }
   else
   {
      // Default settings
      %this-->lodType.setText("DetectDTS");
      %this-->singleDetailSize.setText("2");
      %this-->materialPrefix.setText("");
      %this-->alwaysImport.setText("");
      %this-->neverImport.setText("");
      %this-->alwaysImportMesh.setText("");
      %this-->neverImportMesh.setText("");
      %this-->ignoreNodeScale.setValue(0);
      %this-->adjustCenter.setValue(0);
      %this-->adjustFloor.setValue(0);
      %this-->forceUpdateMaterials.setValue(0);
      %this-->loadLights.setValue(0);
   }

   Canvas.pushDialog(%this);

   FbxImportTreeView.refresh("all");
}

function FbxImportDlg::readDtsConfig(%this)
{
   %filename = filePath( %this.path ) @ "/" @ fileBase( %this.path ) @ ".cfg";
   %filename2 = filePath( %this.path ) @ "/" @ "dtsScene.cfg";

   %fo = new FileObject();
   if ( %fo.openForRead( %filename ) || %fo.openForRead( %filename2 ) )
   {
      %alwaysImport = "";
      %neverImport = "";

      %mode = "none";
      while ( !%fo.isEOF() )
      {
         %line = trim( %fo.readLine() );

         if ( %line $= "AlwaysExport:" )        // Start of the AlwaysExport list
            %mode = "always";
         else if ( %line $= "NeverExport:" )    // Start of the NeverExport list
            %mode = "never";
         else if ( startswith( %line, "+" ) || startswith( %line, "-" ) )   // Boolean parameters (not supported)
            %mode = "none";
         else if ( startswith( %line, "=" ) )   // Float and integer parameters (not supported)
            %mode = "none";
         else if ( !startswith( %line, "//" ) ) // Non-commented lines
         {
            switch$ (%mode)
            {
            case "always":
               %alwaysImport = %alwaysImport TAB %line;
            case "never":
               %neverImport = %neverImport TAB %line;
            }
         }
      }
      %fo.close();

      %alwaysImport = strreplace( trim( %alwaysImport ), "\t", ";" );
      %neverImport = strreplace( trim( %neverImport ), "\t", ";" );

      %this-->alwaysImport.setText( %alwaysImport );
      %this-->neverImport.setText( %neverImport );
   }
   else
   {
      error( "Failed to open " @ %filename @ " or " @ %filename2 @ " for reading" );
   }

   %fo.delete();
}

function FbxImportDlg::writeDtsConfig(%this)
{
   %filename = filePath( %this.path ) @ "/" @ fileBase( %this.path ) @ ".cfg";

   %fo = new FileObject();
   if ( %fo.openForWrite( %filename ) )
   {
      // AlwaysImport
      %fo.writeLine("AlwaysExport:");
      %alwaysImport = trim( strreplace( %this-->alwaysImport.getText(), ";", "\t" ) );
      %count = getFieldCount( %alwaysImport );
      for (%i = 0; %i < %count; %i++)
         %fo.writeLine( getField( %alwaysImport, %i ) );
      %fo.writeLine("");

      // NeverImport
      %fo.writeLine("NeverExport:");
      %neverImport = trim( strreplace( %this-->neverImport.getText(), ";", "\t" ) );
      %count = getFieldCount( %neverImport );
      for (%i = 0; %i < %count; %i++)
         %fo.writeLine( getField( %neverImport, %i ) );
      %fo.writeLine("");

      %fo.close();
   }
   else
   {
      error( "Failed to open " @ %filename @ " for writing" );
   }

   %fo.delete();
}

function FbxImportDlg::updateOverrideUpAxis(%this, %override)
{
   %this-->overrideUpAxis.setValue(%override);
   %this-->upAxis.setActive(%override);
   if (!%override)
      %this-->upAxis.setText(FbxImportTreeView._upAxis);
}

function FbxImportDlg::updateOverrideScale(%this, %override)
{
   %this-->scale.setActive(%override);
   if (!%override)
      %this-->scale.setText(FbxImportTreeView._unit);
}

function FbxImportTreeView::refresh(%this, %what)
{
   %shapeRoot = %this.getFirstRootItem();
   %materialsRoot = %this.getNextSibling(%shapeRoot);
   %animRoot = %this.getNextSibling(%materialsRoot);

   // Refresh nodes
   if ((%what $= "all") || (%what $= "nodes"))
   {
      // Indicate whether nodes will be ignored on import
      %this._alwaysImport = strreplace(FbxImportDlg-->alwaysImport.getText(), ";", "\t");
      %this._neverImport = strreplace(FbxImportDlg-->neverImport.getText(), ";", "\t");
      %this._alwaysImportMesh = strreplace(FbxImportDlg-->alwaysImportMesh.getText(), ";", "\t");
      %this._neverImportMesh = strreplace(FbxImportDlg-->neverImportMesh.getText(), ";", "\t");
      %this.refreshNode(%this.getChild(%shapeRoot));
   }

   // Refresh materials
   if ((%what $= "all") || (%what $= "materials"))
   {
      %matPrefix = FbxImportDlg-->materialPrefix.getText();
      %id = %this.getChild(%materialsRoot);
      while (%id > 0)
      {
         %baseName = %this.getItemValue(%id);
         %name = %matPrefix @ %baseName;

         // Indicate whether material name is already mapped
         %this.editItem(%id, %name, %baseName);
         %mapped = getMaterialMapping(%name);
         if (%mapped $= "")
         {
            %this.setItemTooltip(%id, "A new material will be mapped to this name");
            %this.setItemImages(%id, %this._imageMaterial, %this._imageMaterial);
         }
         else
         {
            %this.setItemTooltip(%id, %mapped SPC "is already mapped to this material name");
            %this.setItemImages(%id, %this._imageExMaterial, %this._imageExMaterial);
         }

         %id = %this.getNextSibling(%id);
      }
   }

   // Refresh animations
   if ((%what $= "all") || (%what $= "animations"))
   {
      %id = %this.getChild(%animRoot);
      while (%id > 0)
      {
         %this.setItemImages(%id, %this._imageAnim, %this._imageAnim);
         %id = %this.getNextSibling(%id);
      }
   }
}

function FbxImportTreeView::refreshNode(%this, %id)
{
   while (%id > 0)
   {
      switch$ (%this.getItemValue(%id))
      {
         case "mesh":
            // Check if this mesh will be ignored on import
            if (strIsMatchMultipleExpr(%this._alwaysImportMesh, %this.getItemText(%id)) ||
               !strIsMatchMultipleExpr(%this._neverImportMesh, %this.getItemText(%id)) )
            {
               %this.setItemTooltip(%id, "");
               %this.setItemImages(%id, %this._imageMesh, %this._imageMesh);
            }
            else
            {
               %this.setItemTooltip(%id, "This mesh will be ignored on import");
               %this.setItemImages(%id, %this._imageExNode, %this._imageExNode);
            }

         case "light":
            %this.setItemImages(%id, %this._imageLight, %this._imageLight);

         case "node":
            // Check if this node will be ignored on import
            if (strIsMatchMultipleExpr(%this._alwaysImport, %this.getItemText(%id)) ||
               !strIsMatchMultipleExpr(%this._neverImport, %this.getItemText(%id)) )
            {
               %this.setItemTooltip(%id, "");
               %this.setItemImages(%id, %this._imageNode, %this._imageNode);
            }
            else
            {
               %this.setItemTooltip(%id, "This node will be ignored on import");
               %this.setItemImages(%id, %this._imageExNode, %this._imageExNode);
            }
      }

      // recurse through children and siblings
      %this.refreshNode(%this.getChild(%id));
      %id = %this.getNextSibling(%id);
   }
}

function FbxImportDlg::onCancel(%this)
{
   Canvas.popDialog(%this);
   FbxImportTreeView.clear();
}

function FbxImportDlg::onOK(%this)
{
   Canvas.popDialog(%this);
   FbxImportTreeView.clear();

   // Need to create a TSShapeConstructor object if any settings are not
   // at the default values
   if ((%this-->overrideUpAxis.getValue() != 0)       ||
       (%this-->overrideScale.getValue() != 0)        ||
       (%this-->lodType.getText() !$= "DetectDTS")    ||
       (%this-->singleDetailSize.getText() !$= "2")   ||
       (%this-->materialPrefix.getText() !$= "")      ||
       (%this-->alwaysImport.getText() !$= "")        ||
       (%this-->neverImport.getText() !$= "")         ||
       (%this-->alwaysImportMesh.getText() !$= "")    ||
       (%this-->neverImportMesh.getText() !$= "")     ||
       (%this-->ignoreNodeScale.getValue() != 0)      ||
       (%this-->adjustCenter.getValue() != 0)         ||
       (%this-->adjustFloor.getValue() != 0)          ||
       (%this-->forceUpdateMaterials.getValue() != 0) ||
       (%this-->loadLights.getValue() != 0))
   {
      if (%this.constructor <= 0)
      {
         // Create a new TSShapeConstructor object
         %this.constructor = ShapeEditor.createConstructor(%this.path);
      }
   }

   if (%this.constructor > 0)
   {
      // Store values from GUI
      if (%this-->overrideUpAxis.getValue())
         %this.constructor.upAxis = %this-->upAxis.getText();
      else
         %this.constructor.upAxis = "DEFAULT";

      if (%this-->overrideScale.getValue())
         %this.constructor.unit = %this-->scale.getText();
      else
         %this.constructor.unit = -1;

      %this.constructor.lodType = %this-->lodType.getText();
      %this.constructor.singleDetailSize = %this-->singleDetailSize.getText();
      %this.constructor.matNamePrefix = %this-->materialPrefix.getText();
      %this.constructor.alwaysImport = strreplace(%this-->alwaysImport.getText(), ";", "\t");
      %this.constructor.neverImport = strreplace(%this-->neverImport.getText(), ";", "\t");
      %this.constructor.alwaysImportMesh = strreplace(%this-->alwaysImportMesh.getText(), ";", "\t");
      %this.constructor.neverImportMesh = strreplace(%this-->neverImportMesh.getText(), ";", "\t");
      %this.constructor.ignoreNodeScale = %this-->ignoreNodeScale.getValue();
      %this.constructor.adjustCenter = %this-->adjustCenter.getValue();
      %this.constructor.adjustFloor = %this-->adjustFloor.getValue();
      %this.constructor.forceUpdateMaterials = %this-->forceUpdateMaterials.getValue();
      %this.constructor.loadLights = %this-->loadLights.getValue();

      // Save new settings to file
      ShapeEditor.saveConstructor( %this.constructor );
   }

   // Load the shape (always from the DAE)
   $Fbx::forceLoadDAE = true;
   eval(%this.cmd);
   $Fbx::forceLoadDAE = false;

   // Optionally load the lights from the DAE as well (only if adding a new shape
   // to the scene)
   if (%this-->loadLights.getValue())
      %this.loadLights();
}

function FbxImportDlg::loadLights(%this)
{
   // Get the ID of the last object added
   %obj = MissionGroup.getObject(MissionGroup.getCount()-1);

   // Create a new SimGroup to hold the model and lights
   %group = new SimGroup();
   loadFbxLights(%this.path, %group, %obj);

   // Delete the SimGroup if no lights were found. Otherwise, add the model to
   // the group as well.
   if (%group.getCount() > 0)
   {
      %group.add(%obj);
      %group.bringToFront(%obj);
      MissionGroup.add(%group);
      if (EditorTree.isVisible())
      {
         EditorTree.removeItem(EditorTree.findItemByObjectId(%obj));
         EditorTree.buildVisibleTree(true);
      }
   }
   else
   {
      %group.delete();
   }
}


function updateTSShapeLoadProgress(%progress, %msg)
{
   // Show/Hide gui at the start/end of the loading process
   if (%progress == 0)
   {
      FbxImportProgress-->window.text = "Importing" SPC %msg;
      FbxImportProgress-->progressBar.setValue(0.001);
      FbxImportProgress-->progressText.setText("Reading file into memory...");
      Canvas.pushDialog(FbxImportProgress);
   }
   else if (%progress == 1.0)
   {
      Canvas.popDialog(FbxImportProgress);
   }
   else
   {
      FbxImportProgress-->progressBar.setValue(%progress);
      FbxImportProgress-->progressText.setText(%msg);
   }

   Canvas.repaint();
}


// Convert all Fbx models that match the given pattern (defaults to *) to DTS
function convertFbxModels(%pattern)
{
   // Force loading the Fbx file (to ensure cached DTS is updated)
   $Fbx::forceLoadDAE = true;

   %fullPath = findFirstFile("*.dae");
   while (%fullPath !$= "")
   {
      // Check if this file is inside the given path
      %fullPath = makeRelativePath(%fullPath, getMainDotCSDir());
      if ((%pattern $= "") || strIsMatchMultipleExpr(%pattern, %fullPath))
      {
         // Load the model by creating a temporary TSStatic
         echo("Converting " @ %fullPath @ " to DTS...");
         %temp = new TSStatic() {
            shapeName = %fullPath;
            collisionType = "None";
         };
         %temp.delete();
      }

      %fullPath = findNextFile("*.dae");
   }

   $Fbx::forceLoadDAE = false;
}
