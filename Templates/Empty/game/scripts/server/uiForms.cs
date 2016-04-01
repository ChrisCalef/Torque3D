

//=============================================================================
//
//                 UI FORMS
//
//=============================================================================

//(These are defined in openSimEarth.cs currently, uncomment them for a standalone uiForms project.)

function startSQL(%dbname)
{//Create the sqlite object that we will use in all the scripts.
   %sqlite = new SQLiteObject(sqlite);
   
   if (%sqlite.openDatabase(%dbname))
      echo("Successfully opened database: " @ %dbname );
   else {
      echo("We had a problem involving database: " @ %dbname );
      return;
   }
}

function stopSQL()
{
   sqlite.closeDatabase();
   sqlite.delete();      
}


$MegaMotionFormID = 1;
$uiFormID = 37;
$uiAddFormID = 103;
$uiAddElementID = 104;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//For each form that we hook up to a top menu, give it an "expose" function to load it and call setup for it.

function exposeMegaMotion()
{
   if (isDefined("MegaMotionWindow"))
      MegaMotionWindow.delete();
   
   makeSqlGuiForm($MegaMotionFormID);
   setupMegaMotionForm();   
}

function setupMegaMotionForm()
{
   if (!isDefined("MegaMotionWindow"))
      return;   
      
   %sceneSetList = MegaMotionWindow.findObjectByInternalName("sceneSetList");
   %sceneList = MegaMotionWindow.findObjectByInternalName("sceneList");
   
   %sceneSetList.add("Testing","1");
   %sceneSetList.add("Portlingrad","2");
   %sceneSetList.setSelected(1);
   
   %sceneList.add("PG_001","1");
   %sceneList.add("PG_002","2");
   %sceneList.add("PG_003","3");
   %sceneList.add("PG_004","4");
   %sceneList.setSelected(1);
   
}

//Nice try, but even with modifications to GuiWindowCtrl, there is no way to intercept a mouse
//event that lands on a control. Hence, we're screwed if we want to select controls this way.
function MegaMotionWindow::onMouseDown(%this,%pos)
{
   echo("MegaMotionWindow onMouseDown!!!!  this.pos " @ %this.getPosition() @ " mouse pos " @ %pos);  
}

function MegaMotionWindow::onMouseUp(%this,%pos)
{
   echo("MegaMotionWindow onMouseUp!!!!  this.pos " @ %this.getPosition() @ " mouse pos " @ %pos);  
}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

function addUIForm()
{
   makeSqlGuiForm($uiAddFormID);
}

function reallyAddUIForm()
{
   //NOW: check for validity of name (strlen > 0 and unique) and type (strlen>0, and hopefully isClass test)
   %msg = "";
   if ( strlen($addUIFormName)==0 && strlen($addUIFormType)==0 )
   {
      %msg = "Both Name and Type fields need valid text values.";
   } else if ( strlen($addUIFormName)==0 ) {      
      %msg = "The Name field needs a valid and unique text value.";
   } else if ( strlen($addUIFormType)==0 ) {
      %msg = "The Type field needs to be a valid Torque GUI class.";
   }
   if (!isClass($addUIFormType))
      %msg = "The Type field needs to be a valid Torque GUI class.";
      
   if (strlen(%msg)>0)
   {
      MessageBoxOK("Name or Type Invalid",%msg,"");
      return;  
   }

   %width = 60;
   if (strlen($addUIFormWidth)>0)
      %width = $addUIFormWidth;
      
   %height = 20;
   if (strlen($addUIFormHeight)>0)
      %height = $addUIFormHeight;
    
   %query = "INSERT INTO uiElement (name,type,parent_id,width,height,pos_x,pos_y) " @  
            "VALUES ('" @ $addUIFormName @ "','" @ $addUIFormType @ "',0," @ %width @ 
            "," @ %height @ ",0.5,0.5);";
   sqlite.query(%query, 0); 
   
   %query = "UPDATE uiElement SET form_id=last_insert_rowid() WHERE id=last_insert_rowid();";
   sqlite.query(%query, 0);    
   
   echo("really added UI Form name " @ $addUIFormName @ "  type " @ $addUIFormType);
   
   exposeUIFormWindow();//Re expose window, to refresh form list.
   
   
   %query = "SELECT last_insert_rowid() AS id;";//heh, does this work?
   %resultSet = sqlite.query(%query, 0); 
   if (%resultSet)
   {
      %form_id = sqlite.getColumn(%resultSet, "id");
      $formList.setSelected(%form_id);
      sqlite.clearResult(%resultSet);
   }
   
   addFormWindow.delete();
}

function deleteUIForm()
{
   MessageBoxYesNoCancel( "Delete Form", "This will permanently delete this form and all of its child elements. Are you sure you want to do this?",
 "reallyDeleteUIForm();",
 "",
 "");
}

function reallyDeleteUIForm()
{
   %query = "DELETE FROM uiElement WHERE form_id=" @ $formList.getSelected() @ ";";
   sqlite.query(%query, 0); 
   
   $formName = $formList.getText();
   $formName.delete();

   $formList.setSelected($uiFormID);
   
   exposeUIFormWindow();//Re expose window, to refresh form list.
   
}

function exposeUIFormWindow()
{
   %present = false;
   if (isDefined("uiFormWindow"))
   {
      %present = true;
      %pos = uiFormWindow.getPosition();
      %ext = uiFormWindow.getExtent();
      %form_id = $formList.getSelected();
      %form_text = $formList.getText();
      %elem_id = $elementList.getSelected();
      %elem_text = $elementList.getText();
      uiFormWindow.delete();
   }
   makeSqlGuiForm($uiFormID);
   setupUIFormWindow();  
   
   if (%present)
   {
      //schedule(1000,0,"restoreUIForm",%pos,%ext,%form_id,%elem_id);
      uiFormWindow.setPosition(getWord(%pos,0),getWord(%pos,1));
      uiFormWindow.setExtent(getWord(%ext,0),getWord(%ext,1));
      echo("present, selecting formID: " @ %form_id );
      if ($formList.findText(%form_text)>0)
         $formList.setSelected(%form_id);
      else
         $formList.setSelected($uiFormID);   
      //Watch out here, if you select something you just deleted, CRASH.
      if ($elementList.findText(%elem_text)>0)
         $elementList.setSelected(%elem_id);
      else
         $elementList.setSelected(%form_id);
   } else {
      $formList.setSelected($uiFormID);
      $elementList.setSelected($uiFormID);  
   }
}

//Hmm, something deadly here, but first let's fix the other deadly problem, maybe related.
function restoreUIForm(%pos,%ext,%form_id,%elem_id)
{
   echo("trying to restore ui form, pos " @ %pos @ " extent " @ %ext);
   if (isDefined("uiFormWindow"))
   {
      echo("setting pos!!!!!!!");
      uiFormWindow.setPosition(getWord(%pos,0),getWord(%pos,1));
      uiFormWindow.setExtent(getWord(%ext,0),getWord(%ext,1));
      if ($form_id>0)
         $formList.setSelected(%form_id);
      if (%elem_id>0)
         $elementList.setSelected(%elem_id);      
   }
}

function setupUIFormWindow()
{
   if (!isDefined("uiFormWindow"))
      return; 
   
   $formList = uiFormWindow.findObjectByInternalName("formList");
   $elementList = uiFormWindow.findObjectByInternalName("elementList");
   $parentList = uiFormWindow.findObjectByInternalName("parentList");
   $leftAnchorList = uiFormWindow.findObjectByInternalName("leftAnchorList");
   $rightAnchorList = uiFormWindow.findObjectByInternalName("rightAnchorList");
   $topAnchorList = uiFormWindow.findObjectByInternalName("topAnchorList");
   $bottomAnchorList = uiFormWindow.findObjectByInternalName("bottomAnchorList");
   $bitmapList = uiFormWindow.findObjectByInternalName("bitmapList");

   $typeEdit = uiFormWindow.findObjectByInternalName("typeEdit");
   $nameEdit = uiFormWindow.findObjectByInternalName("nameEdit");
   $contentEdit = uiFormWindow.findObjectByInternalName("contentEdit");
   $widthEdit = uiFormWindow.findObjectByInternalName("widthEdit");
   $heightEdit = uiFormWindow.findObjectByInternalName("heightEdit");
   $commandEdit = uiFormWindow.findObjectByInternalName("commandEdit");
   $tooltipEdit = uiFormWindow.findObjectByInternalName("tooltipEdit");
   $posXEdit = uiFormWindow.findObjectByInternalName("posXEdit");
   $posYEdit = uiFormWindow.findObjectByInternalName("posYEdit");
   $horizPaddingEdit = uiFormWindow.findObjectByInternalName("horizPaddingEdit");
   $vertPaddingEdit = uiFormWindow.findObjectByInternalName("vertPaddingEdit");
   $horizEdgePaddingEdit = uiFormWindow.findObjectByInternalName("horizEdgePaddingEdit");
   $vertEdgePaddingEdit = uiFormWindow.findObjectByInternalName("vertEdgePaddingEdit");
   $variableEdit = uiFormWindow.findObjectByInternalName("variableEdit");
   $buttonTypeEdit = uiFormWindow.findObjectByInternalName("buttonTypeEdit");
   $groupNumEdit = uiFormWindow.findObjectByInternalName("groupNumEdit");
   $profileEdit = uiFormWindow.findObjectByInternalName("profileEdit");
   $valueEdit = uiFormWindow.findObjectByInternalName("valueEdit");
      
      
   $formList.clear();
   $elementList.clear();
   $parentList.clear();
   $leftAnchorList.clear();
   $rightAnchorList.clear();
   $topAnchorList.clear();
   $bottomAnchorList.clear();
   $bitmapList.add("",0);
   
   %tempControlCount = 0;
   $parentList.add("",0);
   $leftAnchorList.add("",0);
   $rightAnchorList.add("",0);
   $topAnchorList.add("",0);
   $bottomAnchorList.add("",0);
   $bitmapList.add("",0);
   
   %query = "SELECT e.id,e.name FROM uiElement e " @
            "WHERE e.id = e.form_id;";   
   %resultSet = sqlite.query(%query, 0); 
   if (%resultSet)
   {
      while (!sqlite.endOfResult(%resultSet))
      {
         %id = sqlite.getColumn(%resultSet, "id");
         %name = sqlite.getColumn(%resultSet, "name");
         $formList.add(%name,%id);
         sqlite.nextRow(%resultSet);
      }
      sqlite.clearResult(%resultSet);
   }
   sqlite.clearResult(%resultSet);
   
   $horizAlignList = uiFormWindow.findObjectByInternalName("horizAlignList");
   $vertAlignList = uiFormWindow.findObjectByInternalName("vertAlignList");
   
   %query = "SELECT * FROM uiBitmap;";   
   %resultSet = sqlite.query(%query, 0); 
   if (%resultSet)
   {
      while (!sqlite.endOfResult(%resultSet))
      {
         %id = sqlite.getColumn(%resultSet, "id");
         %path = sqlite.getColumn(%resultSet, "path");
         $bitmapList.add(%path,%id);
         sqlite.nextRow(%resultSet);
      }
      sqlite.clearResult(%resultSet);
   }
   
   
   $horizAlignList.add("",0);
   $horizAlignList.add("Left",1);
   $horizAlignList.add("Center",2);
   $horizAlignList.add("Right",3);
   
   $vertAlignList.add("",0);
   $vertAlignList.add("Top",1);
   $vertAlignList.add("Center",2);
   $vertAlignList.add("Bottom",3);
   
}

function selectUIForm()
{
   echo("selecting UI form: " @ $formList.getSelected());
   if ($formList.getSelected()==0)
      return;
      
   %form_name = $formList.getText();
   %form_id = $formList.getSelected();
      
   $elementList.clear();      
   $parentList.clear(); 
   $leftAnchorList.clear(); 
   $rightAnchorList.clear(); 
   $topAnchorList.clear(); 
   $bottomAnchorList.clear(); 
      
   $parentList.add("",0);
   $leftAnchorList.add("",0); 
   $rightAnchorList.add("",0); 
   $topAnchorList.add("",0);
   $bottomAnchorList.add("",0);
   
   %query = "SELECT id,name FROM uiElement " @
            "WHERE form_id = " @ $formList.getSelected() @ ";";   
   %resultSet = sqlite.query(%query, 0); 
   if (%resultSet)
   {
      %firstID = sqlite.getColumn(%resultSet, "id");
      while (!sqlite.endOfResult(%resultSet))
      {
         %id = sqlite.getColumn(%resultSet, "id");
         %name = sqlite.getColumn(%resultSet, "name");
         
         $elementList.add(%name,%id);
         $parentList.add(%name,%id);
         $leftAnchorList.add(%name,%id);
         $rightAnchorList.add(%name,%id);
         $topAnchorList.add(%name,%id);
         $bottomAnchorList.add(%name,%id);
         
         sqlite.nextRow(%resultSet);
      }
      sqlite.clearResult(%resultSet);
   }
   
   $elementList.setSelected(%firstID);   
   echo("trying to open window " @ %form_name);
   if (%form_name !$= "uiFormWindow")
   {
      if (isDefined(%form_name))
      {
         %form_name.delete();
         echo("window " @ %form_name @ " was already open!");
      }
      makeSqlGuiForm(%form_id);
   }
}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

function addUIElement()
{
   echo("add UI Element");
   
   makeSqlGuiForm($uiAddElementID);
   
}

function reallyAddUIElement()
{
   
   %msg = "";
   if ( strlen($addUIElementName)==0 && strlen($addUIElementType)==0 )
   {
      %msg = "Both Name and Type fields need valid text values.";
   } else if ( strlen($addUIElementName)==0 ) {      
      %msg = "The Name field needs a valid and unique text value.";
   } else if ( strlen($addUIElementType)==0 ) {
      %msg = "The Type field needs to be a valid Torque GUI class.";
   }
   if (!isClass($addUIElementType))
      %msg = "The Type field needs to be a valid Torque GUI class.";
   if (!( ($addUIElementWidth>0) && ($addUIElementHeight>0))) 
   { //isNumeric($addUIElementHeight) && isNumeric($addUIElementWidth) //Thought I added this function?
      echo("found a width or height non numeric.");
      %msg = "The Width and Height fields need to be positive integer values.";
   }
   
   if (strlen(%msg)>0)
   {
      MessageBoxOK("Invalid Field",%msg,"");
      return;  
   }
   
   %width = 60;
   if (strlen($addUIElementWidth)>0)
      %width = $addUIElementWidth;
      
   %height = 20;
   if (strlen($addUIElementHeight)>0)
      %height = $addUIElementHeight;
    
   %query = "INSERT INTO uiElement (name,type,parent_id,form_id,width,height) VALUES ('" @ $addUIElementName @ 
               "','" @ $addUIElementType @ "'," @ $formList.getSelected() @ 
               "," @ $formList.getSelected() @ "," @ %width @ "," @ %height @ ");";
   sqlite.query(%query, 0); 
   
   addElementWindow.delete();
   
   $formList.setSelected($formList.getSelected());//Refresh the form, get the new element on the list.
   
   %query = "SELECT last_insert_rowid() AS id;";//heh, does this work?
   %resultSet = sqlite.query(%query, 0); 
   if (%resultSet)
   {
      %elem_id = sqlite.getColumn(%resultSet, "id");
      $elementList.setSelected(%elem_id);
      sqlite.clearResult(%resultSet);
   }   
   
   echo("really added UI Element name " @ $addUIElementName @ "  type " @ $addUIElementType);
   
}

function deleteUIElement()
{
   if ($elementList.getSelected()==$formList.getSelected())
   {
      MessageBoxOK("","Delete the form using the Form delete button, not the Element delete button.","");
      return;  
   }
   MessageBoxYesNoCancel( "Delete Element", "This will permanently delete this element and all of its child elements. Are you sure you want to do this?",
 "reallyDeleteUIElement();",
 "",
 "");
}

function reallyDeleteUIElement()
{
   //Now, this gets more complicated: we need to delete children, and children of children, recursively.
   //Next pass: hook up radio buttons to decide whether to do this, or to reassign them to parent.
   //UPDATE: On second thought, screw the recursion, just lock it down to the reassign option for now
   //and delete one element at a time. 

   %form_id = $formList.getSelected();
   %elem_id = $elementList.getSelected();
   %parent_id = $parentList.getSelected();
   
   %query = "UPDATE uiElement SET parent_id=" @ %parent_id @ " WHERE parent_id=" @ %elem_id @ " AND form_id=" @ %form_id @ ";";
   sqlite.query(%query,0);
   %query = "UPDATE uiElement SET left_anchor=" @ %parent_id @ " WHERE left_anchor=" @ %elem_id @ " AND form_id=" @ %form_id @ ";";
   sqlite.query(%query,0);
   %query = "UPDATE uiElement SET right_anchor=" @ %parent_id @ " WHERE right_anchor=" @ %elem_id @ " AND form_id=" @ %form_id @ ";";
   sqlite.query(%query,0);
   %query = "UPDATE uiElement SET top_anchor=" @ %parent_id @ " WHERE top_anchor=" @ %elem_id @ " AND form_id=" @ %form_id @ ";";
   sqlite.query(%query,0);
   %query = "UPDATE uiElement SET bottom_anchor=" @ %parent_id @ " WHERE bottom_anchor=" @ %elem_id @ " AND form_id=" @ %form_id @ ";";
   sqlite.query(%query,0);
   
   //And, finally, delete the actual element:
   %query = "DELETE FROM uiElement WHERE id=" @ %elem_id @ ";";
   sqlite.query(%query,0);
   
   //There, much easier.   
   
   exposeUIFormWindow();//Re expose window, to refresh element list.

}

/*
   %layerCount = 0;
   %layers[%layerCount] = %elem_id;   
   while (%resultSet)
   {
      %foundChildren = false;
      while (!sqlite.endOfResult(%resultSet))
      {  
         %child_id = sqlite.getColumn(%resultSet,"id");
         %query = "SELECT * FROM uiElement WHERE parent_id=" @ %child_id @ ";";
         %resultSet2 = sqlite.query(%query,0);
         if (%resultSet2)
         {              
            %layerCount++;
            %layers[%layerCount] = %child_id; 
            %foundChildren = true;
            %resultSet = %resultSet2;            
            break;//Now... this _should_ take me back to the while !endOfResult loop, only a level down.
         }
         
         sqlite.nextRow(%resultSet);
      }
      //Now, if we made it here, and foundChildren = false, then we're safe to delete.
      if (!%foundChildren)
      {
         %delete_query = "DELETE FROM uiElement WHERE parent_id=" @ %layers[%layerCount] @ ";";
         sqlite.query(%delete_query,0);
         %layerCount--;
         if (%layerCount>=0)
         {
            %query = "SELECT * FROM uiElement WHERE parent_id=" @ %layers[%layerCount] @ ";";
            %resultSet = sqlite.query(%query,0);
         } else {
            %resultSet = 0;  
         }
      }
   }
   */
   
   
function selectUIElement()
{
   %column_names[0] = "id";
   %column_names[1] = "parent_id";
   %column_names[2] = "name";
   %column_names[3] = "width";
   %column_names[4] = "height";
   %column_names[5] = "type";
   %column_names[6] = "bitmap_id";
   %column_names[7] = "left_anchor";
   %column_names[8] = "right_anchor";
   %column_names[9] = "top_anchor";
   %column_names[10] = "bottom_anchor";
   %column_names[11] = "content";
   %column_names[12] = "command";
   %column_names[13] = "tooltip";
   %column_names[14] = "horiz_align";
   %column_names[15] = "vert_align";
   %column_names[16] = "pos_x";
   %column_names[17] = "pos_y";
   %column_names[18] = "horiz_padding";
   %column_names[19] = "vert_padding";
   %column_names[20] = "horiz_edge_padding";
   %column_names[21] = "vert_edge_padding";
   %column_names[22] = "variable";
   %column_names[23] = "button_type";
   %column_names[24] = "group_num";
   %column_names[25] = "profile";
   %column_names[26] = "value";
   
   %query = "SELECT * FROM uiElement e " @ 
	         "WHERE id=" @ $elementList.getSelected() @ ";";   
   
   %resultSet = sqlite.query(%query, 0); 
   if (%resultSet)
   {
      for (%c=0;%c<27;%c++)
      {
         %resultSet[%c] = sqlite.getColumn(%resultSet, %column_names[%c]);
         if (%resultSet[%c] $= "NULL")
            %resultSet[%c] = "";
      }      
      sqlite.clearResult(%resultSet);
   }
   
   if (%resultSet[7]<0) { $anchor_left_align = true; %resultSet[7] *= -1; }
   else $anchor_left_align = false;
   if (%resultSet[8]<0) { $anchor_right_align = true; %resultSet[8] *= -1; }
   else $anchor_right_align = false;
   if (%resultSet[9]<0) { $anchor_top_align = true; %resultSet[9] *= -1; }
   else $anchor_top_align = false;
   if (%resultSet[10]<0) { $anchor_bottom_align = true; %resultSet[10] *= -1; }
   else $anchor_bottom_align = false;
   
   $parentList.setSelected(%resultSet[1]);
   $bitmapList.setSelected(%resultSet[6]); 
   $leftAnchorList.setSelected(%resultSet[7]);
   $rightAnchorList.setSelected(%resultSet[8]);
   $topAnchorList.setSelected(%resultSet[9]);
   $bottomAnchorList.setSelected(%resultSet[10]);
   $horizAlignList.setSelected(%resultSet[14]);
   $vertAlignList.setSelected(%resultSet[15]);
   
   $nameEdit.setText(%resultSet[2]);
   $widthEdit.setText(%resultSet[3]);
   $heightEdit.setText(%resultSet[4]);
   $typeEdit.setText(%resultSet[5]);
   $contentEdit.setText(%resultSet[11]);
   $commandEdit.setText(%resultSet[12]);
   $tooltipEdit.setText(%resultSet[13]);
   $posXEdit.setText(%resultSet[16]);
   $posYEdit.setText(%resultSet[17]);
   $horizPaddingEdit.setText(%resultSet[18]);
   $vertPaddingEdit.setText(%resultSet[19]);
   $horizEdgePaddingEdit.setText(%resultSet[20]);
   $vertEdgePaddingEdit.setText(%resultSet[21]);
   $variableEdit.setText(%resultSet[22]);
   $buttonTypeEdit.setText(%resultSet[23]);
   $groupNumEdit.setText(%resultSet[24]);
   $profileEdit.setText(%resultSet[25]);
   $valueEdit.setText(%resultSet[26]);
   
   //Anything else to do?   
}

function updateUIElement()
{
   %column_names[0] = "id";
   %column_names[1] = "parent_id";
   %column_names[2] = "name";
   %column_names[3] = "width";
   %column_names[4] = "height";
   %column_names[5] = "type";
   %column_names[6] = "bitmap_id";
   %column_names[7] = "left_anchor";
   %column_names[8] = "right_anchor";
   %column_names[9] = "top_anchor";
   %column_names[10] = "bottom_anchor";
   %column_names[11] = "content";
   %column_names[12] = "command";
   %column_names[13] = "tooltip";
   %column_names[14] = "horiz_align";
   %column_names[15] = "vert_align";
   %column_names[16] = "pos_x";
   %column_names[17] = "pos_y";
   %column_names[18] = "horiz_padding";
   %column_names[19] = "vert_padding";
   %column_names[20] = "horiz_edge_padding";
   %column_names[21] = "vert_edge_padding";
   %column_names[22] = "variable";
   %column_names[23] = "button_type";
   %column_names[24] = "group_num";
   %column_names[25] = "profile";
   %column_names[26] = "value";
   
   //YUP: this works perfectly. All I need now is a loop through column names, or else just do it manually.
   %query = "UPDATE uiElement SET " @   
      "parent_id=" @ $parentList.getSelected() @
      ",name=\"" @ $nameEdit.getText() @ "\"";
      
   %leftAnchor = $leftAnchorList.getSelected();
   %rightAnchor = $rightAnchorList.getSelected();
   %topAnchor = $topAnchorList.getSelected();
   %bottomAnchor = $bottomAnchorList.getSelected();
   
   if ($anchor_left_align && (strlen(%leftAnchor)>0))
      %leftAnchor *= -1;
   if ($anchor_right_align && (strlen(%rightAnchor)>0))
      %rightAnchor *= -1;
   if ($anchor_top_align && (strlen(%topAnchor)>0))
      %topAnchor *= -1;
   if ($anchor_bottom_align && (strlen(%bottomAnchor)>0))
      %bottomAnchor *= -1;
   
   if (strlen($widthEdit.getText())>0)
      %query = %query @ ",width=" @ $widthEdit.getText();
   if (strlen($heightEdit.getText())>0)
      %query = %query @ ",height=" @ $heightEdit.getText();
   if (strlen($typeEdit.getText())>0) 
      %query = %query @ ",type='" @ $typeEdit.getText() @ "'";
   if (strlen($bitmapList.getSelected())>0)
      %query = %query @ ",bitmap_id=" @ $bitmapList.getSelected();
   if (strlen(%leftAnchor)>0)
      %query = %query @ ",left_anchor=" @ %leftAnchor;
   if (strlen(%rightAnchor)>0)
      %query = %query @ ",right_anchor=" @ %rightAnchor;
   if (strlen(%topAnchor)>0)
      %query = %query @ ",top_anchor=" @ %topAnchor;
   if (strlen(%bottomAnchor)>0)
      %query = %query @ ",bottom_anchor=" @ %bottomAnchor;
   if (strlen($contentEdit.getText())>0)
      %query = %query @ ",content='" @ $contentEdit.getText() @ "'";
   if (strlen($commandEdit.getText())>0)
      %query = %query @ ",command='" @ $commandEdit.getText() @ "'";
   if (strlen($tooltipEdit.getText())>0)
      %query = %query @ ",tooltip='" @ $tooltipEdit.getText() @ "'";
   if (strlen($horizAlignList.getSelected())>0)
      %query = %query @ ",horiz_align=" @ $horizAlignList.getSelected();
   if (strlen($vertAlignList.getSelected())>0)
      %query = %query @ ",vert_align=" @ $vertAlignList.getSelected();
   if (strlen($posXEdit.getText())>0)
      %query = %query @ ",pos_x=" @ $posXEdit.getText();
   if (strlen($posYEdit.getText())>0)
      %query = %query @ ",pos_y=" @ $posYEdit.getText();
   if (strlen($horizPaddingEdit.getText())>0)
      %query = %query @ ",horiz_padding=" @ $horizPaddingEdit.getText();
   if (strlen($vertPaddingEdit.getText())>0)
      %query = %query @ ",vert_padding=" @ $vertPaddingEdit.getText();
   if (strlen($horizEdgePaddingEdit.getText())>0)
      %query = %query @ ",horiz_edge_padding=" @ $horizEdgePaddingEdit.getText();
   if (strlen($vertEdgePaddingEdit.getText())>0)
      %query = %query @ ",vert_edge_padding=" @ $vertEdgePaddingEdit.getText();
   if (strlen($variableEdit.getText())>0)
      %query = %query @ ",variable='" @ $variableEdit.getText() @ "'";
   if (strlen($buttonTypeEdit.getText())>0)
      %query = %query @ ",button_type='" @ $buttonTypeEdit.getText() @ "'";
   if (strlen($groupNumEdit.getText())>0)
      %query = %query @ ",group_num=" @ $groupNumEdit.getText();
   if (strlen($profileEdit.getText())>0)
      %query = %query @ ",profile='" @ $profileEdit.getText() @ "'";
   if (strlen($valueEdit.getText())>0)
      %query = %query @ ",value='" @ $valueEdit.getText() @ "'";
      
   %query = %query @ " WHERE id=" @ $elementList.getSelected() @ ";";
   
   echo("UPDATE QUERY: \n" @ %query );
   
   sqlite.query(%query, 0); 
   
   exposeUIFormWindow();//And, refresh the visible window.
   
}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

function saveSqlGuiXML(%form_id,%xml_file)
{   
   %count = 0;
   
   %column_names[0] = "id";
   %column_names[1] = "parent_id";
   %column_names[2] = "name";
   %column_names[3] = "width";
   %column_names[4] = "height";
   %column_names[5] = "type";
   %column_names[6] = "path";
   %column_names[7] = "left_anchor";
   %column_names[8] = "right_anchor";
   %column_names[9] = "top_anchor";
   %column_names[10] = "bottom_anchor";
   %column_names[11] = "content";
   %column_names[12] = "command";
   %column_names[13] = "tooltip";
   %column_names[14] = "horiz_align";
   %column_names[15] = "vert_align";
   %column_names[16] = "pos_x";
   %column_names[17] = "pos_y";
   %column_names[18] = "horiz_padding";
   %column_names[19] = "vert_padding";
   %column_names[20] = "horiz_edge_padding";
   %column_names[21] = "vert_edge_padding";
   %column_names[22] = "variable";
   %column_names[23] = "button_type";
   %column_names[24] = "group_num";
   %column_names[25] = "profile";
   %column_names[26] = "value";
   	         
   %query = "SELECT e.id,e.parent_id,e.bitmap_id,e.left_anchor,e.right_anchor,e.top_anchor,e.bottom_anchor,e.type," @
            "e.content,e.name,e.width,e.height,e.command,e.tooltip,e.horiz_align,e.vert_align,e.pos_x,e.pos_y,e.horiz_padding," @
            "e.vert_padding,e.horiz_edge_padding,e.vert_edge_padding,e.variable,e.button_type,e.group_num,e.profile," @
            "e.value,la.name as la_name,ra.name as ra_name,ta.name as ta_name,ba.name as ba_name,b.path " @
            "FROM uiElement e " @ 
	         "LEFT JOIN uiBitmap b ON b.id=e.bitmap_id " @ 
	         "LEFT JOIN uiElement la ON la.id=e.left_anchor " @ 
	         "LEFT JOIN uiElement ra ON ra.id=e.right_anchor " @ 
	         "LEFT JOIN uiElement ta ON ta.id=e.top_anchor " @ 
	         "LEFT JOIN uiElement ba ON ba.id=e.bottom_anchor " @ 
	         "WHERE e.form_id=" @ %form_id @ ";"; 
	%resultSet = sqlite.query(%query, 0);	
   
   if (%resultSet)
   {
      while (!sqlite.endOfResult(%resultSet))
      {
         %c = 0;         
         %results[%count,%c] = sqlite.getColumn(%resultSet, "id"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "parent_id"); %c++;
         
         %results[%count,%c] = sqlite.getColumn(%resultSet, "name"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "width"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "height"); %c++;
         
         %results[%count,%c] = sqlite.getColumn(%resultSet, "type"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "path"); %c++;

         %results[%count,%c] = sqlite.getColumn(%resultSet, "left_anchor"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "right_anchor"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "top_anchor"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "bottom_anchor"); %c++;

         %results[%count,%c] = sqlite.getColumn(%resultSet, "content"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "command"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "tooltip"); %c++;

         %results[%count,%c] = sqlite.getColumn(%resultSet, "horiz_align"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "vert_align"); %c++;
         
         %results[%count,%c] = sqlite.getColumn(%resultSet, "pos_x"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "pos_y"); %c++;

         %results[%count,%c] = sqlite.getColumn(%resultSet, "horiz_padding"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "vert_padding"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "horiz_edge_padding"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "vert_edge_padding"); %c++;

         %results[%count,%c] = sqlite.getColumn(%resultSet, "variable"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "button_type"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "group_num"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "profile"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "value"); %c++;
         
         %results[%count,%c] = false;//OR, instead of overriding type, just add one more column at the end.
         
         echo("loading result " @ %count @ " " @ %results[%count,2] @ " width " @ %results[%count,3] @ 
         " height " @ %results[%count,4] @ "  command: " @ %results[%count,12] );
         
         %count++;
         sqlite.nextRow(%resultSet);
      }
      sqlite.clearResult(%resultSet);
   }
   
   if ((strlen(%results[0,2])==0)||(strlen(%results[0,5])==0))
   {
      echo("Gui form is missing either name: " @ %results[0,2] @ ", or type: " @ %results[0,5] @ ".");   
      return;
   }

   //First, do the NULL fix on the entire array.
   for (%k=0;%k<%count;%k++)
      for (%d=0;%d<%c;%d++)
         if (%results[%k,%d] $= "NULL") %results[%k,%d] = "";

   //Next, fix all the anchors, to use names instead of IDs, and maintain the sign-flip convention.
   for (%k=0;%k<%count;%k++)
   {
      for (%d=7;%d<11;%d++)
      {
         if (strlen(%results[%k,%d])>0)
         {
            %flip = "";
            if (%results[%k,%d]<0)
            {
               %flip = "-";
               %results[%k,%d] *= -1;
            }
            for (%j=0;%j<%count;%j++)
               if (%results[%j,0]==%results[%k,%d])
                  %results[%k,%d] = %flip @ %results[%j,2];
         }
      }
   }
   
   //Next, start the form, then start looping through children.
   %xml = new SimXMLDocument() {};
   %xml.addHeader();
	
   %xml.pushNewElement("gui");
   
   //OKAY, now we're getting down to it. Loop through children, same as before, except different.
   
   %finished = false;
   %formname = "";
   %sanityCount = 0;
   %layerCount = 0;//Keeps track of how many layers deep we are in the parent hierarchy.
   %currentElement = %form_id;
   
   while ((%finished==false)&&(%sanityCount++ < 50))//For this application, there are no undefined anchors,
   {               // so we only have to worry about doing the main loop again for subcontainers.
   
      %currentCounter = 0;//First, %currentElement is a DB ID, so change it to an array counter.
      for (%k=0;%k<%count;%k++)
         if (%results[%k,0]==%currentElement)
            %currentCounter = %k;
      
      %currentChildCount = 0;//Now, make sure we have children, and count them.
      for (%k=0;%k<%count;%k++)
      {
         if (%results[%k,1]==%currentElement) // 1 = parent_id
         {
            %currentChildren[%currentChildCount] = %results[%k,0]; // 0 = id
            %currentChildCount++;//TorqueScript: don't put ++ inside brackets, or it will ++ too soon.
         }
      }
      if (%currentChildCount == 0) //Something went wrong, get us out of here.
      { 
         %finished = true;
         continue;
      }
      
      if (!%results[%currentCounter,%c]) //Don't start the form again, if we're back here to
      {  //finish the rest of the children after having grandchildren.
         if (%currentElement == %form_id)
            %xml.pushNewElement("form");
         else 
            %xml.pushNewElement("element");
            
         %xml.setAttribute(%column_names[0],%results[%currentCounter,0]);
         for (%d=2;%d<5;%d++)
            %xml.setAttribute(%column_names[%d],%results[%currentCounter,%d]);
      
         for (%d=5;%d<%c;%d++)
         {
            if (strlen(%results[%currentCounter,%d]) > 0)
            {
               if (%d==5) //This is why we put type in the array, so pushNewElement will always happen first.
                  %xml.pushNewElement(%column_names[%d]);
               else       //Afterward they are all addNewElement.
                  %xml.addNewElement(%column_names[%d]);         
               
               %xml.addData(%results[%currentCounter,%d]);
            }
         }
         %results[%currentCounter,%c] = true;
      }
      
      //Next, run through this container's children.
      for (%k=0;%k<%currentChildCount;%k++)
      {
         %childCounter = 0;//Convert this child's DB ID into an array counter.
         for (%d=0;%d<%count;%d++)
            if (%results[%d,0]==%currentChildren[%k])
               %childCounter = %d;
          
         //Then, check for finished flag. 
         if (%results[%childCounter,%c])  
            continue;
         
         //Now, check this child for its own children. If so, save current parent to %layers and increment %layerCount.
         %subfinished = true;
         %newChildCount = 0;
         for (%j=1;%j<%count;%j++)//Search through whole array of all results. (Except don't start at zero, that's always the form.)
         {
            if (%results[%j,1]==%currentChildren[%k]) // 1 = parent_id
               %newChildCount++;            
         }
         if (%newChildCount>0)
         {
            %layers[%layerCount] = %currentElement; 
            %currentElement = %currentChildren[%k];
            %k = %currentChildCount; //Go to the end, exit loop.
            %subfinished = false;
            %layerCount++;
            %xml.popElement();
            continue;
         } ////////// Full stop if children found. Exit loop and start over, one layer deeper. //////////
            
         ///////////////////////////////
         //Now, if this is a leaf node, go ahead and render it and set finished=true;
         %xml.addNewElement("element");
         
         %xml.setAttribute(%column_names[0],%results[%childCounter,0]);
         for (%d=2;%d<5;%d++)
            %xml.setAttribute(%column_names[%d],%results[%childCounter,%d]);
      
         for (%d=5;%d<%c;%d++)
         {
            if (strlen(%results[%childCounter,%d]) > 0)
            {
               if (%d==5) //This is why we put type in the array, so pushNewElement will always happen first.
                  %xml.pushNewElement(%column_names[%d]);
               else       //Afterward they are all addNewElement.
                  %xml.addNewElement(%column_names[%d]);         
               
               %xml.addData(%results[%childCounter,%d]);
            }
         }
         %results[%childCounter,%c] = true;
         %xml.popElement();
      }
      
      //Now, even though we don't have the "undefined" complication here, due to not needing to obtain
      //final positions from anchors, we still need to keep track of when we stop midway in a list of
      //children and jump down a layer, to a child's children.
      for (%k=0;%k<%currentChildCount;%k++)
      {
         %childCounter = 0;
         for (%d=0;%d<%count;%d++)
            if (%results[%d,0]==%currentChildren[%k])
               %childCounter = %d;
               
          if (!%results[%childCounter,%c])
            %subfinished = false;               
      }
      
      //But now, if subfinished is still true, that means we are done with this container, go back up.
      if (%subfinished) //Only do this if we made it to the end of the children.
      {    
         %layerCount--;
         %currentElement = %layers[%layerCount];//-1, or just %layerCount?
         %xml.popElement();
      }
      
      //Finally: run through the whole list, and if finished is true for every control, we're done.
      for (%k=0;%k<%count;%k++)
      {
         if (!%results[%k,%c])
         {//exit with finished=false the first time we find a valid type.
            %k = %count;
            continue;
         }
         if (%k==(%count-1))
            %finished=true;         
      }      
   }
   
   %xml.popElement();//form
   %xml.popElement();//gui
   
   %xml.saveFile(%xml_file);    
   echo("Saving " @ %xml_file);
}



////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
function makeSqlGuiForm(%form_id)
{
   %count = 0;
   %query = "SELECT e.id,e.parent_id,e.bitmap_id,e.left_anchor,e.right_anchor,e.top_anchor,e.bottom_anchor,e.type," @
            "e.content,e.name,e.width,e.height,e.command,e.tooltip,e.horiz_align,e.vert_align,e.pos_x,e.pos_y," @
            "e.horiz_padding,e.vert_padding,e.horiz_edge_padding,e.vert_edge_padding,e.variable,e.button_type," @
            "e.group_num,e.profile,e.value,b.path " @
            "FROM uiElement e " @ 
	         "LEFT JOIN uiBitmap b ON b.id=e.bitmap_id " @ 
	         "WHERE e.form_id=" @ %form_id @ ";"; 	         
	%resultSet = sqlite.query(%query, 0);	
   //echo( "makeSqlGuiForm2, test = " @ %test @ ", query: \n" @ %query @ "\n  NUMROWS " @ sqlite.numRows(%resultSet));
   
   if (%resultSet)
   {
      while (!sqlite.endOfResult(%resultSet))
      {
         %c = 0;         
         %results[%count,%c] = sqlite.getColumn(%resultSet, "id"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "parent_id"); %c++;
         
         %results[%count,%c] = sqlite.getColumn(%resultSet, "name"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "width"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "height"); %c++;
         
         echo("loading result " @ %count @ " " @ %results[%count,2] @ " width " @ %results[%count,3] @ 
         " height " @ %results[%count,4]  );
         
         %results[%count,%c] = sqlite.getColumn(%resultSet, "type"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "path"); %c++;

         %results[%count,%c] = sqlite.getColumn(%resultSet, "left_anchor"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "right_anchor"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "top_anchor"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "bottom_anchor"); %c++;

         %results[%count,%c] = sqlite.getColumn(%resultSet, "content"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "command"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "tooltip"); %c++;

         %results[%count,%c] = sqlite.getColumn(%resultSet, "horiz_align"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "vert_align"); %c++;
         
         %results[%count,%c] = sqlite.getColumn(%resultSet, "pos_x"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "pos_y"); %c++;

         %results[%count,%c] = sqlite.getColumn(%resultSet, "horiz_padding"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "vert_padding"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "horiz_edge_padding"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "vert_edge_padding"); %c++;

         %results[%count,%c] = sqlite.getColumn(%resultSet, "variable"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "button_type"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "group_num"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "profile"); %c++;
         %results[%count,%c] = sqlite.getColumn(%resultSet, "value"); %c++;
         
         %results[%count,%c] = false;//OR, instead of overriding type, just add one more column at the end.
         
         %count++;
         sqlite.nextRow(%resultSet);
      }
      sqlite.clearResult(%resultSet);
   }
   
   //////////////////////////////////////////////////////////////////////
   //////////////////////////////////////////////////////////////////////
      
   %script = "";
   %onclick_script = "";   
   %indent = "";
   %finished = false;
   %formname = "";
   %sanityCount = 0;
   %layers[0,0] = %form_id;//First parent is always the form itself.
   %layers[0,1] = %indent;
   %layers[0,2] = false;//Maybe? use this to see if we've added our indent yet.
   %layerCount = 1;//Keeps track of how many layers deep we are in the parent hierarchy.
   %currentElement = %form_id;
   while ((%finished == false)&&(%sanityCount++ < 50))
   {      
      //This is now a loop down through each set of children, until we hit the bottom and come back.
      %currentCounter = 0;
      for (%k=0;%k<%count;%k++)
         if (%results[%k,0]==%currentElement)
            %currentCounter = %k;
      
      %currentChildCount = 0;
      for (%k=0;%k<%count;%k++)
      {
         if (%results[%k,1]==%currentElement) // 1 = parent_id
         {
            %currentChildren[%currentChildCount] = %results[%k,0]; // 0 = id
            %currentChildCount++;//TorqueScript: don't put ++ inside brackets, or it will ++ too soon.
         }
      }
      if (%currentChildCount == 0) //Something went wrong, get us out of here.
      { 
         %finished = true;
         continue;
      }
      
      //First, clean up our data for this parent, and grab what we need, a subset of all the fields.
      for (%d=0;%d<%c;%d++)
         if (%results[%currentCounter,%d] $= "NULL") 
            %results[%currentCounter,%d] = "";

      //Then grab the subset of data we need, minus things that are only for leaf nodes.
      %id = %results[%currentCounter,0];
      %parent_id = %results[%currentCounter,1];
      %name = %results[%currentCounter,2];
      %width = %results[%currentCounter,3];
      %height = %results[%currentCounter,4];               
      %type = %results[%currentCounter,5]; 
         
      %container_type = %type;//Save this for later.
      
      %bitmap_path = %results[%currentCounter,6];       
      %left_anchor = %results[%currentCounter,7]; 
      %right_anchor = %results[%currentCounter,8];
      %top_anchor = %results[%currentCounter,9];
      %bottom_anchor = %results[%currentCounter,10];

      %content = %results[%currentCounter,11];
      
      %horiz_align = %results[%currentCounter,14];
      %vert_align = %results[%currentCounter,15];
      
      %pos_x = %results[%currentCounter,16]; 
      %pos_y = %results[%currentCounter,17]; 

      %horiz_padding = %results[%currentCounter,18];
      %vert_padding = %results[%currentCounter,19];
      %horiz_edge_padding = %results[%currentCounter,20];
      %vert_edge_padding = %results[%currentCounter,21];

      %profile = %results[%currentCounter,25];
      
      if (%currentElement==%form_id)
      {//We have to keep checking finished marker because we could be coming back up, not our first time here.
         if (!%results[%currentCounter,%c])
         {
            %formname = %name;
            //echo("starting form, id=" @ %currentElement @ " name = " @ %name @ "\n");
            %script = %script @ "%guiContent = new " @ %type @ "(" @ %name @ ") {\n";

            %editorExtents = EWorldEditor.getExtent();//FIX: I'm assuming all guis are part of the world editor,
            %editorWidth = getWord(%editorExtents,0);//       which is not a good assumption.
            %editorHeight = getWord(%editorExtents,1);
            %pos_x = %pos_x * %editorWidth;
            %pos_y = %pos_y * %editorHeight;
            %results[%currentCounter,16] = %pos_x; //Oh, this is a little awkward... in the DB, I set up pos x/y
            %results[%currentCounter,17] = %pos_y; // to be in percentages, ie float from {0.0,1.0}, but after 
                                                   // we start writing to the buffer these are in pixels.
         
            %script = %script @ "   position = \"" @ mFloor(%pos_x) @ " " @ mFloor(%pos_y) @ "\";\n";
            %script = %script @ "   extent = \"" @ %width @ " " @ %height @ "\";\n";
            %script = %script @ "   text = \"" @ %content @ "\";\n";
            %script = %script @ "   canClose = \"1\";\n";
            %script = %script @ "   useMouseEvents = \"1\";\n";
            %script = %script @ "   closeCommand = \"" @ %name @ ".setVisible(false);\";\n\n";
            
            %results[%currentCounter,%c] = true;//Set finished marker to true, we're done with this one.
         }
      } 
      else if (!%results[%currentCounter,%c])
      {  //We are not the top form, so we need to find out our anchor situation.
         //echo("adding element: " @ %currentElement @ " left " @ %left_anchor @ " right " @ %right_anchor @ 
         //   " top " @ %top_anchor @ " bottom " @ %bottom_anchor @  " parent id " @ %parent_id @ "\n");
            
         //First, find parent container padding numbers.         
         %i = 0;
         %container_width = 0;
         %container_height = 0;
         %container_pos_x = 0;
         %container_pos_y = 0;         
         %container_horiz_padding = 0;
         %container_vert_padding = 0;
         %container_horiz_edge_padding = 0;
         %container_vert_edge_padding = 0;
         while (%i < %count)
         {
            if (%results[%i,0] == %parent_id)
            {
               %container_width = %results[%i,3];
               %container_height = %results[%i,4];
               %container_pos_x = %results[%i,16];
               %container_pos_y = %results[%i,17];
               %container_horiz_padding = %results[%i,18];
               %container_vert_padding = %results[%i,19];
               %container_horiz_edge_padding = %results[%i,20];
               %container_vert_edge_padding = %results[%i,21];              
            }
            %i++;
         }
         
         %undefined = false;
         %horiz_anchor_flip = false;
         if (%left_anchor < 0)
         {
            %horiz_anchor_flip = true;
            %left_anchor *= -1;
         }
         if (%right_anchor < 0)
         {
            %horiz_anchor_flip = true;
            %right_anchor *= -1;
         }
         if (%left_anchor == %parent_id)
         {
            %i = 0;
            while (%i < %count)
            {
               if (%results[%i,0] == %parent_id)
               {
                  if (%results[%i,5] $= "Virtual")
                  {    
                     %pos_x = %container_pos_x + %container_horiz_edge_padding + %horiz_padding;      
                  } else { 
                     %pos_x = %container_horiz_edge_padding + %horiz_padding;
                  }
               }
               %i++;
            }               
         }
         else if (%left_anchor > 0 ) //%parent_id)
         {
            %i = 0;
            %found = false;
            while (%i < %count)
            {
               if ((%results[%i,0] == %left_anchor) && (%results[%i,%c]))
               {
                  %anchor_pos_x = %results[%i,16];
                  %anchor_width = %results[%i,3];
                  if (%horiz_anchor_flip == false)
                     %pos_x = %anchor_pos_x + %anchor_width + %container_horiz_padding + %horiz_padding;
                  else
                     %pos_x = %anchor_pos_x + %horiz_padding;
                  
                  %found = true;
               }
               %i++;  
            }
            if ( !%found )
               %undefined = true;
         } 
         else if (%right_anchor == %parent_id) 
         {
            %i = 0;
            while (%i < %count)
            {
               if (%results[%i,0] == %parent_id)
               {
                  if (%results[%i,5] $= "Virtual")
                  {
                     %pos_x = %container_pos_x + %container_width - %width - %container_horiz_edge_padding - %horiz_padding;                     
                  } else {
                     %pos_x = %container_width - %width - %container_horiz_edge_padding - %horiz_padding;
                  }
               }
               %i++;
            }
         } 
         else if (%right_anchor > 0 ) //%parent_id)
         {
            %i = 0;
            %found = false;
            while (%i < %count)
            {
               if ((%results[%i,0] == %right_anchor) && (%results[%i,%c]))
               {
                  %anchor_pos_x = %results[%i,16];
                  %anchor_width = %results[%i,3];
                  if (%horiz_anchor_flip == false)
                     %pos_x = %anchor_pos_x - %width - %container_horiz_padding - %horiz_padding;
                  else
                     %pos_x = %anchor_pos_x + %anchor_width - %width - %horiz_padding;
                  
                  %found = true;
               }
               %i++;  
            }
            if ( !%found )
               %undefined = true;
         }

         ////// top/bottom anchors //////////////
         %vert_anchor_flip = false;
         if (%top_anchor < 0)
         {
            %vert_anchor_flip = true;
            %top_anchor *= -1;
         }
         if (%bottom_anchor < 0)
         {
            %vert_anchor_flip = true;
            %bottom_anchor *= -1;
         }
         if (%top_anchor == %parent_id) //check for top anchor first, then bottom.
         {
            %i = 0;
            while (%i < %count)
            {
               if (%results[%i,0] == %parent_id)
               {
                  if (%results[%i,5] $= "Virtual")
                  {
                     %pos_y = %container_pos_y + %container_vert_edge_padding + %vert_padding;  
                  } else {
                     %pos_y = %container_vert_edge_padding + %vert_padding;
                  }
               }
               %i++;
            }      
         }
         else if (%top_anchor > 0 ) //%parent_id)
         {
            %i = 0;
            %found = false;
            while (%i < %count)
            {
               if ((%results[%i,0] == %top_anchor) && (%results[%i,%c]))
               {
                  %anchor_pos_y = %results[%i,17];
                  %anchor_height = %results[%i,4];
                  if (%vert_anchor_flip == false)
                     %pos_y = %anchor_pos_y + %anchor_height + %container_vert_padding + %vert_padding;
                  else
                     %pos_y = %anchor_pos_y + %vert_padding;
                  
                  %found = true;
               }
               %i++;  
            }
            if ( !%found )
               %undefined = true;            
         }   
         else if (%bottom_anchor == %parent_id) 
         {
            %i = 0;
            while (%i < %count)
            {
               if (%results[%i,0] == %parent_id)
               {
                  if (%results[%i,5] $= "Virtual")
                  {
                     %pos_y = %container_pos_y + %container_height - %height - %container_vert_edge_padding - %vert_padding; 
                  } else {
                     %pos_y = %container_height - %height - %container_vert_edge_padding - %vert_padding;
                  }
               }
               %i++;
            }             
         } 
         else if (%bottom_anchor > 0 ) //%parent_id)
         {
            %i = 0;
            %found = false;
            while (%i < %count)
            {
               if ((%results[%i,0] == %bottom_anchor) && (%results[%i,%c]))
               {
                  %anchor_pos_y = %results[%i,17];
                  %anchor_height = %results[%i,4];
                  if (%vert_anchor_flip == false)
                     %pos_y = %anchor_pos_y - %height - %container_vert_padding - %vert_padding;
                  else
                     %pos_y = %anchor_pos_y + %anchor_height - %height - %vert_padding;
                  %found = true;
               }
               %i++;  
            }
            if ( !%found )
               %undefined = true;
         }
         
         if (%undefined)
         {
            //This should never happen, we wouldn't be here if we hadn't found our anchors already below.
            echo("OOPS! Gui control has undefined anchors when it is supposed to be a parent! " @ %currentElement);
            return;
         }
         
         //Save these back to the results array so other controls can find them.
         %results[%currentCounter,16] = %pos_x; 
         %results[%currentCounter,17] = %pos_y; 
         //echo("parent control " @ %currentElement @ " found a position: " @ %pos_x @ " " @ %pos_y );
         
         if (%type !$= "Virtual")
         {  
            %script = %script @ %indent @ "new " @ %type @ "() {\n";
            %script = %script @ %indent @ "   position = \"" @ mFloor(%pos_x) @ " " @ mFloor(%pos_y) @ "\";\n";
            %script = %script @ %indent @ "   extent = \"" @ %width @ " " @ %height @ "\";\n";
            %script = %script @ %indent @ "   text = \"" @ %content @ "\";\n";
            if (strlen(%name)>0) %script = %script @ %indent @ "   internalName = \"" @ %name @ "\";\n";            
            if (strlen(%bitmap_path)>0) %script = %script @ %indent @ "   bitmap = \"" @ %bitmap_path @ "\";\n"; 
            if (strlen(%profile)>0) %script = %script @ %indent @ "   profile = \"" @ %profile @ "\";\n"; 
            //if (%test) 
            //   %onclick_script = %onclick_script @ 
            //      "function " @ %name @ "::onClick(%this)\n{\n " @ "   $elementList.setSelected(" @ 
            //      %results[%currentCounter,0] @ ");\n}\n\n";
            %script = %script @ "\n";
         }
         %results[%currentCounter,%c] = true;//Whether or not we're virtual, don't come back here.
      }
      
      //////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////
      //Next, run through the children.
      
      //Make sure we don't indent every time we loop back to the top looking for more children.
      if ((%container_type !$= "Virtual")&&(%layers[%layerCount-1,2]==false))// %layerCount-1 is the parent
      { // container, and the [n,2] column is a flag for just this: set it to false until you add the indent 
         %indent = %indent @ "   "; // for this container, and then set it to true and don't do it again.
         %layers[%layerCount-1,2] = true;
      }
      
      for (%k=0;%k<%currentChildCount;%k++)
      {
         //The %k variable is an index into the local children array, so we have to convert that  
         %childCounter = 0;// into an index into the full %results array.
         for (%d=0;%d<%count;%d++)
            if (%results[%d,0]==%currentChildren[%k])
               %childCounter = %d;

         //Change the sqlite database's "NULL" character strings into actual null strings.
         for (%d=0;%d<%c;%d++)
            if (%results[%childCounter,%d] $= "NULL") 
               %results[%childCounter,%d] = "";
               
         //Then, check for finished flag.
         if (%results[%childCounter,%c])
         {
            //echo("found a finished child " @ %currentChildren[%k] @ ", bailing.");
            continue;
         }
         /////////////////////////////////////////////////
         
         //Now, check this child for its own children. If so, save current parent to %layers and increment %layerCount.
         %subfinished = true;
         %newChildCount = 0;
         for (%j=1;%j<%count;%j++)//Search through whole array of all results. (Except don't start at zero, that's always the form.)
         {
            if (%results[%j,1]==%currentChildren[%k]) // 1 = parent_id
            {
               %newChildCount++;
            }
         }
         if (%newChildCount>0)
         {
            //echo("checking children for " @ %currentChildren[%k] @ ", found " @ %newChildCount );
            %layers[%layerCount,0] = %currentElement; 
            %layers[%layerCount,1] = %indent;       
            %layers[%layerCount,2] = false; //Indent tracker, no we have not added our indent yet.   
            %layers[%layerCount,3] = false; //Close curly braces tracker. 
            %currentElement = %currentChildren[%k];
            %k = %currentChildCount; //Go to the end, exit loop.
            %subfinished = false;
            %layerCount++;
            continue;
         }////////// Full stop if children found. Exit loop and start over, one layer deeper. //////////

         ///////////////////////////////
         //Now, if this is a leaf node, go ahead and render it and set finished=true;

         //First, give results some variable names.
         %id = %results[%childCounter,0];
         %parent_id = %results[%childCounter,1];
         %name = %results[%childCounter,2];
         %width = %results[%childCounter,3];
         %height = %results[%childCounter,4];               
         %type = %results[%childCounter,5]; 
         
         %bitmap_path = %results[%childCounter,6]; 
         %left_anchor = %results[%childCounter,7]; 
         %right_anchor = %results[%childCounter,8];
         %top_anchor = %results[%childCounter,9];
         %bottom_anchor = %results[%childCounter,10];

         %content = %results[%childCounter,11];
         %command = %results[%childCounter,12];
         %tooltip = %results[%childCounter,13];

         %horiz_align = %results[%childCounter,14];
         %vert_align = %results[%childCounter,15];
         
         %pos_x = %results[%childCounter,16]; 
         %pos_y = %results[%childCounter,17]; 

         %horiz_padding = %results[%childCounter,18];
         %vert_padding = %results[%childCounter,19];
         %horiz_edge_padding = %results[%childCounter,20];
         %vert_edge_padding = %results[%childCounter,21];

         %variable = %results[%childCounter,22];
         %button_type = %results[%childCounter,23];
         %group_num = %results[%childCounter,24];
         %profile = %results[%childCounter,25];
         %value = %results[%childCounter,26];
             
         //echo("adding element, name " @ %name @ " type " @ %type @ " id " @ %id );
         //And, UNFORTUNATELY, we (currently) need to repeat the whole block of anchor logic again here. 
         ///////////////////////////  BEGIN UGLY BLOCK OF REPEATED CODE /////////////////////////
         %i = 0;
         %container_width = 0;
         %container_height = 0;
         %container_pos_x = 0;
         %container_pos_y = 0;         
         %container_horiz_padding = 0;
         %container_vert_padding = 0;
         %container_horiz_edge_padding = 0;
         %container_vert_edge_padding = 0;
         while (%i < %count)
         {
            if (%results[%i,0] == %parent_id)
            {
               %container_width = %results[%i,3];
               %container_height = %results[%i,4];
               %container_pos_x = %results[%i,16];
               %container_pos_y = %results[%i,17];
               %container_horiz_padding = %results[%i,18];
               %container_vert_padding = %results[%i,19];
               %container_horiz_edge_padding = %results[%i,20];
               %container_vert_edge_padding = %results[%i,21];              
            }
            %i++;
         }
         
         %undefined = false;
         %horiz_anchor_flip = false;
         if (%left_anchor < 0)
         {
            %horiz_anchor_flip = true;
            %left_anchor *= -1;
         }
         if (%right_anchor < 0)
         {
            %horiz_anchor_flip = true;
            %right_anchor *= -1;
         }
         if (%left_anchor == %parent_id)
         {
            %i = 0;
            while (%i < %count)
            {
               if (%results[%i,0] == %parent_id)
               {
                  if (%results[%i,5] $= "Virtual")
                  {    
                     %pos_x = %container_pos_x + %container_horiz_edge_padding + %horiz_padding;      
                  } else { 
                     %pos_x = %container_horiz_edge_padding + %horiz_padding;
                  }
               }
               %i++;
            }               
         }
         else if (%left_anchor > 0 ) //%parent_id)
         {
            %i = 0;
            %found = false;
            while (%i < %count)
            {
               if ((%results[%i,0] == %left_anchor) && (%results[%i,%c]))
               {
                  %anchor_pos_x = %results[%i,16];
                  %anchor_width = %results[%i,3];
                  if (%horiz_anchor_flip == false)
                     %pos_x = %anchor_pos_x + %anchor_width + %container_horiz_padding + %horiz_padding;
                  else
                     %pos_x = %anchor_pos_x + %horiz_padding;
                  
                  %found = true;
               }
               %i++;  
            }
            if ( !%found )
               %undefined = true;
         } 
         else if (%right_anchor == %parent_id) 
         {
            %i = 0;
            while (%i < %count)
            {
               if (%results[%i,0] == %parent_id)
               {
                  if (%results[%i,5] $= "Virtual")
                  {
                     %pos_x = %container_pos_x + %container_width - %width - %container_horiz_edge_padding - %horiz_padding;                     
                  } else {
                     %pos_x = %container_width - %width - %container_horiz_edge_padding - %horiz_padding;
                  }
               }
               %i++;
            }
         } 
         else if (%right_anchor > 0 ) //%parent_id)
         {
            %i = 0;
            %found = false;
            while (%i < %count)
            {
               if ((%results[%i,0] == %right_anchor) && (%results[%i,%c]))
               {
                  %anchor_pos_x = %results[%i,16];
                  %anchor_width = %results[%i,3];
                  if (%horiz_anchor_flip == false)
                     %pos_x = %anchor_pos_x - %width - %container_horiz_padding - %horiz_padding;
                  else
                     %pos_x = %anchor_pos_x + %anchor_width - %width - %horiz_padding;
                  
                  %found = true;
               }
               %i++;  
            }
            if ( !%found )
               %undefined = true;
         }

         ////// top/bottom anchors //////////////
         %vert_anchor_flip = false;
         if (%top_anchor < 0)
         {
            %vert_anchor_flip = true;
            %top_anchor *= -1;
         }
         if (%bottom_anchor < 0)
         {
            %vert_anchor_flip = true;
            %bottom_anchor *= -1;
         }
         if (%top_anchor == %parent_id) //check for top anchor first, then bottom.
         {
            %i = 0;
            while (%i < %count)
            {
               if (%results[%i,0] == %parent_id)
               {
                  if (%results[%i,5] $= "Virtual")
                  {
                     %pos_y = %container_pos_y + %container_vert_edge_padding + %vert_padding;  
                  } else {
                     %pos_y = %container_vert_edge_padding + %vert_padding;
                  }
               }
               %i++;
            }      
         }
         else if (%top_anchor > 0 ) //%parent_id)
         {
            %i = 0;
            %found = false;
            while (%i < %count)
            {
               if ((%results[%i,0] == %top_anchor) && (%results[%i,%c]))
               {
                  %anchor_pos_y = %results[%i,17];
                  %anchor_height = %results[%i,4];
                  if (%vert_anchor_flip == false)
                     %pos_y = %anchor_pos_y + %anchor_height + %container_vert_padding + %vert_padding;
                  else
                     %pos_y = %anchor_pos_y + %vert_padding;
                  
                  %found = true;
               }
               %i++;  
            }
            if ( !%found )
               %undefined = true;            
         }   
         else if (%bottom_anchor == %parent_id) 
         {
            %i = 0;
            while (%i < %count)
            {
               if (%results[%i,0] == %parent_id)
               {
                  if (%results[%i,5] $= "Virtual")
                  {
                     %pos_y = %container_pos_y + %container_height - %height - %container_vert_edge_padding - %vert_padding; 
                  } else {
                     %pos_y = %container_height - %height - %container_vert_edge_padding - %vert_padding;
                  }
               }
               %i++;
            }             
         } 
         else if (%bottom_anchor > 0 ) //%parent_id)
         {
            %i = 0;
            %found = false;
            while (%i < %count)
            {
               if ((%results[%i,0] == %bottom_anchor) && (%results[%i,%c]))
               {
                  %anchor_pos_y = %results[%i,17];
                  %anchor_height = %results[%i,4];
                  if (%vert_anchor_flip == false)
                     %pos_y = %anchor_pos_y - %height - %container_vert_padding - %vert_padding;
                  else
                     %pos_y = %anchor_pos_y + %anchor_height - %height - %vert_padding;
                  %found = true;
               }
               %i++;  
            }
            if ( !%found )
               %undefined = true;
         }
         ////////////////////////////  END UGLY BLOCK OF REPEATED CODE //////////////////////////
         
         if (%undefined) //Bail if we are missing anchors, we will check again later.
         {
            //echo("control " @ %currentChildren[%k] @ " is currently undefined. Skipping for now.");
            continue;
         }
         %results[%childCounter,16] = %pos_x; 
         %results[%childCounter,17] = %pos_y; 
         //echo("Element " @ %currentChildren[%k] @ " found a position: " @ %pos_x @ " " @ %pos_y);

         if (%type !$= "Virtual") //Should never happen, but if someone accidentally adds a Virtual and doesn't
         {                       //give it any children, we still need to not render it.
            %script = %script @ %indent @ "new " @ %type @ "() {\n";
            %script = %script @ %indent @ "   position = \"" @ mFloor(%pos_x) @ " " @ mFloor(%pos_y) @ "\";\n";
            %script = %script @ %indent @ "   extent = \"" @ %width @ " " @ %height @ "\";\n";
            if (strlen(%content)>0) %script = %script @ %indent @ "   text = \"" @ %content @ "\";\n";
            if (strlen(%name)>0) %script = %script @ %indent @ "   internalName = \"" @ %name @ "\";\n";            
            //if ((!%test)&&(strlen(%command)>0)) %script = %script @ %indent @ "   command = \"" @ %command @ "\";\n";
            if (strlen(%command)>0) %script = %script @ %indent @ "   command = \"" @ %command @ "\";\n";
            if (strlen(%tooltip)>0) %script = %script @ %indent @ "   tooltip = \"" @ %tooltip @ "\";\n"; 
            if (strlen(%tooltip)>0) %script = %script @ %indent @ "   tooltipprofile = \"GuiToolTipProfile\";\n";
            if (strlen(%bitmap_path)>0) %script = %script @ %indent @ "   bitmap = \"" @ %bitmap_path @ "\";\n"; 
            if (strlen(%variable)>0) %script = %script @ %indent @ "   variable = \"" @ %variable @ "\";\n"; 
            if (strlen(%button_type)>0) %script = %script @ %indent @ "   buttonType = \"" @ %button_type @ "\";\n"; 
            if (strlen(%group_num)>0) %script = %script @ %indent @ "   groupNum = \"" @ %group_num @ "\";\n"; 
            if (strlen(%profile)>0) %script = %script @ %indent @ "   profile = \"" @ %profile @ "\";\n"; 
            //if (%test) 
            //   %onclick_script = %onclick_script @ 
            //      "function " @ %name @ "::onClick(%this)\n{\n " @ "   $elementList.setSelected(" @ 
            //      %results[%childCounter,0] @ ");\n}\n\n";
            %script = %script @ %indent @ "};\n";
            %results[%childCounter,%c] = true; 
         }
         
         //Clear all, so you don't end up with the last control's tooltip because this one doesn't have one...
         %width = "";
         %height = "";
         %left_anchor = "";
         %right_anchor = "";
         %top_anchor = "";
         %bottom_anchor = "";
         %name = "";
         %type = "";
         %content = "";
         %command = "";
         %tooltip = "";
         %bitmap_path = "";
         %variable = "";
         %button_type = "";
         %group_num = "";
         %value = "";
         %profile = "";
         
      }//end of for (0..%currentChildCount) loop.
      
      //Now, double check, did we finish everybody?
      //Two ways we can not be done: either we were sent out of the loop because we need to go into a deeper
      //child layer, or we have children we passed by earlier because of undefined anchors.
      //This is for checking the second case. 
      for (%k=0;%k<%currentChildCount;%k++)
      {
         %childCounter = 0;
         for (%d=0;%d<%count;%d++)
            if (%results[%d,0]==%currentChildren[%k])
               %childCounter = %d;
               
          if (!%results[%childCounter,%c])
            %subfinished = false;               
      }
      
      //But now, if subfinished is still true, that means we are done with this container, go back up.
      if (%subfinished) //Only do this if we made it to the end of the children.
      {    
         %layerCount--;
         //echo("finishing a subcontainer, type = " @  %container_type @ ", currentElement " @ %currentElement );  
         %currentElement = %layers[%layerCount,0];//-1, or just %layerCount?
         %indent = %layers[%layerCount,1];//This gets saved whether or not we're virtual, some indents just get repeated.
         if (%container_type !$= "Virtual")//But we only add a closing brace if we're not virtual.
         {
            %script = %script @ %indent @ "};\n";
         }
         %layers[%layerCount,3] = true;//Yes, we closed curly braces, or else don't need to if virtual.
      }
      //Finally: run through the whole list, and if type is null for every control, we're DONE done.
      for (%k=0;%k<%count;%k++)
      {
         if (!%results[%k,%c])
         {//exit with finished=false the first time we find a valid type.
            //echo("unfinished control: " @ %results[%k,0]);
            %k = %count;
            continue;
         }
         if (%k==(%count-1))
         {
            echo("finished main loop with all controls defined, sanityCount " @ %sanityCount);
            %finished=true;
         }
      }
   }
   
   //Now, double check that we got all the curly braces - they can be missed when containers back up 
   //onto other containers
   while (%layerCount>=0)
   {
      if (%layers[%layerCount,3]==false)
      {
         %script = %script @ %layers[%layerCount,1] @ "};\n";      
      }      
      %layerCount--;
   }
   
   //if (%test) 
   //   %script = %script @ %onclick_script;
   
   //And then, save it out as a .gui file.
   %filename = %formname @ ".gui";
   %fileObject = new FileObject();
   %fileObject.openForWrite( %filename ); 
   %fileObject.writeLine(%script);
   %fileObject.close();
   %fileObject.delete();
      
   echo("\n\nSCRIPT:\n\n" @ %script);
   
   //////////////////////////////
   eval(%script);  //DO IT!
   //////////////////////////////
   
   EWorldEditor.add(%formname); 
}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
function saveFormXML()
{
   %filename = getSaveXmlName();  
   
   if (strlen(%filename)>0)
      saveSqlGuiXML($formList.getSelected(),%filename);
}

function loadFormXML()
{
   %filename = getOpenXmlName();  
   
   if (strlen(%filename)>0)
      makeXmlGuiForm(%filename);
}

function getOpenXmlName()
{
   //OpenFileDialog();
   
   %dlg = new OpenFileDialog()
   {
      Filters        = "XML Files (*.xml)|*.xml|All Files (*.*)|*.*|";
      DefaultPath    = "";
      ChangePath     = false;
      MustExist      = true;
   };
   if(%dlg.Execute())
   {
      $Pref::DsqDir = filePath( %dlg.FileName );
      %filename = %dlg.FileName;      
      %dlg.delete();
      return %filename;
   }
   %dlg.delete();
   return "";
}

function getSaveXmlName()
{
   %dlg = new SaveFileDialog()
   {
      Filters        = "XML Files (*.xml)|*.xml|All Files (*.*)|*.*|";
      DefaultPath    = "";
      ChangePath     = false;
      OverwritePrompt   = true;
   };
   if(%dlg.Execute())
   {
      $Pref::DsqDir = filePath( %dlg.FileName );
      %filename = %dlg.FileName;      
      %dlg.delete();
      return %filename;
   }
   %dlg.delete();
   return "";   
}

function makeXmlGuiForm(%filename)
{
  echo("MAKE XML GUI FORM!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
   //////////////////////////////////////////////////////////////////////
   //HERE: for xml, need to load results with contents of the file, it will contain exactly the children of this 
   //form and nothing else, so no need for queryies.   
   //////////////////////////////////////////////////////////////////////
   %column_names[0] = "id";//This may not be used, but keep it here so we don't bump all the indices by one.
   %column_names[1] = "parent_id";//This will be a name instead of an ID here, but same field works.
   %column_names[2] = "name";
   %column_names[3] = "width";
   %column_names[4] = "height";
   %column_names[5] = "type";
   %column_names[6] = "path";
   %column_names[7] = "left_anchor";
   %column_names[8] = "right_anchor";
   %column_names[9] = "top_anchor";
   %column_names[10] = "bottom_anchor";
   %column_names[11] = "content";
   %column_names[12] = "command";
   %column_names[13] = "tooltip";
   %column_names[14] = "horiz_align";
   %column_names[15] = "vert_align";
   %column_names[16] = "pos_x";
   %column_names[17] = "pos_y";
   %column_names[18] = "horiz_padding";
   %column_names[19] = "vert_padding";
   %column_names[20] = "horiz_edge_padding";
   %column_names[21] = "vert_edge_padding";
   %column_names[22] = "variable";
   %column_names[23] = "button_type";
   %column_names[24] = "group_num";
   %column_names[25] = "profile";
   %column_names[26] = "value";
   %c = 27;
   %xml = new SimXMLDocument() {};
   %xml.loadFile( %filename );
   
   %xml.pushChildElement("gui");  
   
   %xml.pushChildElement("form");  
   
   %count = 0;
   
   for (%k=0;%k<27;%k++) // first clear the array.
      %results[%count,%k] = "";
      
   %results[%count,0] = %xml.attribute(%column_names[0]);//ID
   if (strlen(%results[%count,0])==0) %results[%count,0] = %count+1;//Plus one, so no object has id=0.
   
   for (%c=2;%c<5;%c++)
      %results[%count,%c] = %xml.attribute(%column_names[%c]);     
   for (%c=5;%c<27;%c++)
   {
      if (%xml.pushFirstChildElement(%column_names[%c]))
      {
         %results[%count,%c] = %xml.getData();   
         %xml.popElement();
      }
   }
   
   %results[%count,1] = 0;//No parent, for we are the top.
   %results[%count,27] = false;//And always mark "finished" flag false.
   %form_id = %results[%count,0];
   
   %container_type = %results[%count,5];//Save this for later. (?)
   //echo("loading form " @ %count @ " " @ %results[%count,2] @ " width " @ %results[%count,3] @ 
   //   " height " @ %results[%count,4] @ " type " @ %container_type @ " pos_x " @ %results[%count,16] @ 
   //   " pos y " @ %results[%count,17] @ " horiz edge padding " @ %results[%count,20] @ 
   //   " vert edge padding " @ %results[%count,21] @ " horiz align " @ %results[%count,14] @ "!!!\n" );
   %count++; 
   
   /////////////////////////////////////////////// 
   //Now, load all the element children, and their children, and their children...
   %elem = %xml.pushFirstChildElement("element");
   while (%elem)
   {      
      for (%k=0;%k<27;%k++) %results[%count,%k] = "";      
      %results[%count,0] = %xml.attribute(%column_names[0]);   
      if (strlen(%results[%count,0])==0) %results[%count,0] = %count+1;
      for (%c=2;%c<5;%c++)
         %results[%count,%c] = %xml.attribute(%column_names[%c]);     
      for (%c=5;%c<27;%c++)
      {
         if (%xml.pushFirstChildElement(%column_names[%c]))
         {
            %results[%count,%c] = %xml.getData();   
            %xml.popElement();
         }
      }
      %results[%count,1] = %form_id;
      %results[%count,27] = false;
      %count++; 
      ///////////////////////////////////////////////
      %elem2 =  %xml.pushFirstChildElement("element");
      if (%elem2)
      {
         %parent2 = %results[%count-1,0];
         while (%elem2)
         {            
            for (%k=0;%k<27;%k++) %results[%count,%k] = "";         
            %results[%count,0] = %xml.attribute(%column_names[0]);   
            if (strlen(%results[%count,0])==0) %results[%count,0] = %count+1;
            for (%c=2;%c<5;%c++)
               %results[%count,%c] = %xml.attribute(%column_names[%c]);     
            for (%c=5;%c<27;%c++)
            {
               if (%xml.pushFirstChildElement(%column_names[%c]))
               {
                  %results[%count,%c] = %xml.getData();   
                  %xml.popElement();
               }
            }
            %results[%count,1] = %parent2;
            %results[%count,27] = false;
            %count++; 
            ///////////////////////////////////////////////            
            %elem3 =  %xml.pushFirstChildElement("element");
            if (%elem3)
            {
               %parent3 = %results[%count-1,0];
               while (%elem3)
               {               
                  for (%k=0;%k<27;%k++) %results[%count,%k] = "";         
                  %results[%count,0] = %xml.attribute(%column_names[0]);  
                  if (strlen(%results[%count,0])==0) %results[%count,0] = %count+1;    
                  for (%c=2;%c<5;%c++)
                     %results[%count,%c] = %xml.attribute(%column_names[%c]);     
                  for (%c=5;%c<27;%c++)
                  {
                     if (%xml.pushFirstChildElement(%column_names[%c]))
                     {
                        %results[%count,%c] = %xml.getData();   
                        %xml.popElement();
                     }
                  }
                  %results[%count,1] = %parent3;                  
                  %results[%count,27] = false;
                  %count++; 
                  ///////////////////////////////////////////////             
                  %elem4 =  %xml.pushFirstChildElement("element");
                  if (%elem4)
                  {
                     %parent4 = %results[%count-1,0];
                     while (%elem4)
                     {                  
                        for (%k=0;%k<27;%k++) %results[%count,%k] = "";
                        %results[%count,0] = %xml.attribute(%column_names[0]);      
                        if (strlen(%results[%count,0])==0) %results[%count,0] = %count+1;         
                        for (%c=2;%c<5;%c++)
                           %results[%count,%c] = %xml.attribute(%column_names[%c]);     
                        for (%c=5;%c<27;%c++)
                        {
                           if (%xml.pushFirstChildElement(%column_names[%c]))
                           {
                              %results[%count,%c] = %xml.getData();   
                              %xml.popElement();
                           }
                        }
                        %results[%count,1] = %parent4;  
                        %results[%count,27] = false;
                        %count++; 

                        ///And... that's as many layers as we get for XML, if you want more use SQL version.
                        
                        %elem4 = %xml.nextSiblingElement("element");
                     }
                     %xml.popElement();
                  }/////////////////////////////////////////////// 
                  %elem3 = %xml.nextSiblingElement("element");
               }
               %xml.popElement();               
            }/////////////////////////////////////////////// 
            %elem2 = %xml.nextSiblingElement("element");
         }
         %xml.popElement();
      }/////////////////////////////////////////////// 
      %elem = %xml.nextSiblingElement("element");
   }/////////////////////////////////////////////// 
      
   //Now: we have all the data, loaded up into %results, but we need to fix the parents and anchors, 
   // to use IDs instead of names, but maintain the sign-flip convention.
   for (%k=0;%k<%count;%k++)
   {
      //First, parent. If we have a parent name, then convert it to the parent ID.
      if (strlen(%results[%k,1])>0)
      {
         for (%j=0;%j<%count;%j++)
            if (%results[%j,2]$=%results[%k,1])
               %results[%k,1] = %results[%j,0];
      }
      //Then, do the same thing for the four anchors, along with the sign-flipping convention.
      for (%d=7;%d<11;%d++)
      {
         if (strlen(%results[%k,%d])>0)
         {
            %flip = 1;
            if (strchrpos(%results[%k,%d],"-")>=0) //If leading hyphen is found in name, flip sign on anchor id.
            { 
               %flip = -1;
               %results[%k,%d] = getSubStr(%results[%k,%d],1);//Remove leading hyphen.
            }
            for (%j=0;%j<%count;%j++)
               if (%results[%j,2]$=%results[%k,%d])//If (name = anchor name) then go back to ID,
                  %results[%k,%d] = %flip * %results[%j,0];//  flipped if necessary.
                  
         }
      }
   }

  
   //////////////////////////////////////////////////////////////////////
   
   %script = "";
   %indent = "";
   %finished = false;
   %formname = "";
   %sanityCount = 0;
   %layers[0,0] = %results[0,0];//First parent is always the form itself.
   %layers[0,1] = %indent;
   %layers[0,2] = false;//Maybe? use this to see if we've added our indent yet.
   %layerCount = 1;//Keeps track of how many layers deep we are in the parent hierarchy.
   %currentElement = %results[0,0];
   while ((%finished == false)&&(%sanityCount++ < 100))
   {      
      //This is now a loop down through each set of children, until we hit the bottom and come back.
      %currentCounter = 0;
      for (%k=0;%k<%count;%k++)
         if (%results[%k,0]==%currentElement)
            %currentCounter = %k;
      
      %currentChildCount = 0;
      for (%k=0;%k<%count;%k++)
      {
         if (%results[%k,1]==%currentElement) // 1 = parent_id
         {
            %currentChildren[%currentChildCount] = %results[%k,0]; // 0 = id
            %currentChildCount++;//TorqueScript: don't put ++ inside brackets, or it will ++ too soon.
         }
      }
      if (%currentChildCount == 0) //Something went wrong, get us out of here.
      { 
         %finished = true;
         continue;
      }
      
      //First, clean up our data for this parent, and grab what we need, a subset of all the fields.
      for (%d=0;%d<%c;%d++)
         if (%results[%currentCounter,%d] $= "NULL") 
            %results[%currentCounter,%d] = "";

      //Then grab the subset of data we need, minus things that are only for leaf nodes.
      %id = %results[%currentCounter,0];
      %parent_id = %results[%currentCounter,1];
      %name = %results[%currentCounter,2];
      %width = %results[%currentCounter,3];
      %height = %results[%currentCounter,4];               
      %type = %results[%currentCounter,5]; 
         
      %container_type = %type;//Save this for later.
      
      %bitmap_path = %results[%currentCounter,6];       
      %left_anchor = %results[%currentCounter,7]; 
      %right_anchor = %results[%currentCounter,8];
      %top_anchor = %results[%currentCounter,9];
      %bottom_anchor = %results[%currentCounter,10];

      %horiz_align = %results[%currentCounter,14];
      %vert_align = %results[%currentCounter,15];
      
      %pos_x = %results[%currentCounter,16]; 
      %pos_y = %results[%currentCounter,17]; 

      %horiz_padding = %results[%currentCounter,18];
      %vert_padding = %results[%currentCounter,19];
      %horiz_edge_padding = %results[%currentCounter,20];
      %vert_edge_padding = %results[%currentCounter,21];

      %profile = %results[%currentCounter,25];
      
      if (%currentElement==%form_id)
      {//We have to keep checking finished marker because we could be coming back up, not our first time here.
         if (!%results[%currentCounter,%c])
         {
            %formname = %name;
            echo("starting form, id=" @ %currentElement @ " name = " @ %name @ "\n");
            %script = %script @ "%guiContent = new " @ %type @ "(" @ %name @ ") {\n";

            %editorExtents = EWorldEditor.getExtent();//FIX: I'm assuming all guis are part of the world editor,
            %editorWidth = getWord(%editorExtents,0);//       which is not a good assumption.
            %editorHeight = getWord(%editorExtents,1);
            %pos_x = %pos_x * %editorWidth;
            %pos_y = %pos_y * %editorHeight;
            %results[%currentCounter,16] = %pos_x; //Oh, this is a little awkward... in the DB, I set up pos x/y
            %results[%currentCounter,17] = %pos_y; // to be in percentages, ie float from {0.0,1.0}, but after 
                                                   // we start writing to the buffer these are in pixels.
         
            %script = %script @ "   position = \"" @ mFloor(%pos_x) @ " " @ mFloor(%pos_y) @ "\";\n";
            %script = %script @ "   extent = \"" @ %width @ " " @ %height @ "\";\n";
            %script = %script @ "   text = \"" @ %name @ "\";\n";
            %script = %script @ "   canClose = \"1\";\n";
            %script = %script @ "   closeCommand = \"" @ %name @ ".setVisible(false);\";\n\n";
            
            %results[%currentCounter,%c] = true;//Set finished marker to true, we're done with this one.
         }
      } 
      else if (!%results[%currentCounter,%c])
      {  //We are not the top form, so we need to find out our anchor situation.
         //echo("adding element: " @ %currentElement @ " left " @ %left_anchor @ " right " @ %right_anchor @ 
         //   " top " @ %top_anchor @ " bottom " @ %bottom_anchor @  " parent id " @ %parent_id @ "\n");
            
         //First, find parent container padding numbers.         
         %i = 0;
         %container_width = 0;
         %container_height = 0;
         %container_pos_x = 0;
         %container_pos_y = 0;         
         %container_horiz_padding = 0;
         %container_vert_padding = 0;
         %container_horiz_edge_padding = 0;
         %container_vert_edge_padding = 0;
         while (%i < %count)
         {
            if (%results[%i,0] == %parent_id)
            {
               %container_width = %results[%i,3];
               %container_height = %results[%i,4];
               %container_pos_x = %results[%i,16];
               %container_pos_y = %results[%i,17];
               %container_horiz_padding = %results[%i,18];
               %container_vert_padding = %results[%i,19];
               %container_horiz_edge_padding = %results[%i,20];
               %container_vert_edge_padding = %results[%i,21];              
            }
            %i++;
         }
         
         echo("container " @ %currentCounter @ " parent " @ %parent_id @ " anchors: l " @ %left_anchor @ " r " @ %right_anchor @ " t " @ %top_anchor @ " b " @ %bottom_anchor );
         %undefined = false;
         %horiz_anchor_flip = false;
         if (%left_anchor < 0)
         {
            %horiz_anchor_flip = true;
            %left_anchor *= -1;
         }
         if (%right_anchor < 0)
         {
            %horiz_anchor_flip = true;
            %right_anchor *= -1;
         }
         if ((strlen(%left_anchor)>0)&&(%left_anchor == %parent_id))
         {
            %i = 0;
            while (%i < %count)
            {
               if (%results[%i,0] == %parent_id)
               {
                  if (%results[%i,5] $= "Virtual")
                  {    
                     %pos_x = %container_pos_x + %container_horiz_edge_padding + %horiz_padding;      
                  } else { 
                     %pos_x = %container_horiz_edge_padding + %horiz_padding;
                  }
               }
               %i++;
            }               
         }
         else if (%left_anchor > %parent_id)
         {
            %i = 0;
            %found = false;
            while (%i < %count)
            {
               if ((%results[%i,0] == %left_anchor) && (%results[%i,%c]))
               {
                  %anchor_pos_x = %results[%i,16];
                  %anchor_width = %results[%i,3];
                  echo("container left anchor pos " @ %anchor_pos_x @ " width " @ %anchor_width);
                  if (%horiz_anchor_flip == false)
                     %pos_x = %anchor_pos_x + %anchor_width + %container_horiz_padding + %horiz_padding;
                  else
                     %pos_x = %anchor_pos_x + %horiz_padding;
                  
                  %found = true;
               }
               %i++;  
            }
            if ( !%found )
               %undefined = true;
         } 
         else if (%right_anchor == %parent_id) 
         {
            echo("container right anchor pos " @ %container_pos_x @ " width " @ %container_width);
            %i = 0;
            while (%i < %count)
            {
               if (%results[%i,0] == %parent_id)
               {
                  
                  if (%results[%i,5] $= "Virtual")
                  {
                     %pos_x = %container_pos_x + %container_width - %width - %container_horiz_edge_padding - %horiz_padding;                     
                  } else {
                     %pos_x = %container_width - %width - %container_horiz_edge_padding - %horiz_padding;
                  }
               }
               %i++;
            }
         } 
         else if (%right_anchor > %parent_id)
         {
            %i = 0;
            %found = false;
            while (%i < %count)
            {
               if ((%results[%i,0] == %right_anchor) && (%results[%i,%c]))
               {
                  %anchor_pos_x = %results[%i,16];
                  %anchor_width = %results[%i,3];
                  if (%horiz_anchor_flip == false)
                     %pos_x = %anchor_pos_x - %width - %container_horiz_padding - %horiz_padding;
                  else
                     %pos_x = %anchor_pos_x + %anchor_width - %width - %horiz_padding;
                  
                  %found = true;
               }
               %i++;  
            }
            if ( !%found )
               %undefined = true;
         }

         ////// top/bottom anchors //////////////
         %vert_anchor_flip = false;
         if (%top_anchor < 0)
         {
            %vert_anchor_flip = true;
            %top_anchor *= -1;
         }
         if (%bottom_anchor < 0)
         {
            %vert_anchor_flip = true;
            %bottom_anchor *= -1;
         }
         if ((strlen(%top_anchor)>0)&&(%top_anchor == %parent_id)) //check for top anchor first, then bottom.
         {
            %i = 0;
            while (%i < %count)
            {
               if (%results[%i,0] == %parent_id)
               {
                  if (%results[%i,5] $= "Virtual")
                  {
                     %pos_y = %container_pos_y + %container_vert_edge_padding + %vert_padding;  
                  } else {
                     %pos_y = %container_vert_edge_padding + %vert_padding;
                  }
               }
               %i++;
            }      
         }
         else if (%top_anchor > %parent_id)
         {
            %i = 0;
            %found = false;
            while (%i < %count)
            {
               if ((%results[%i,0] == %top_anchor) && (%results[%i,%c]))
               {
                  %anchor_pos_y = %results[%i,17];
                  %anchor_height = %results[%i,4];
                  if (%vert_anchor_flip == false)
                     %pos_y = %anchor_pos_y + %anchor_height + %container_vert_padding + %vert_padding;
                  else
                     %pos_y = %anchor_pos_y + %vert_padding;
                  
                  %found = true;
               }
               %i++;  
            }
            if ( !%found )
               %undefined = true;            
         }   
         else if (%bottom_anchor == %parent_id) 
         {
            %i = 0;
            while (%i < %count)
            {
               if (%results[%i,0] == %parent_id)
               {
                  if (%results[%i,5] $= "Virtual")
                  {
                     %pos_y = %container_pos_y + %container_height - %height - %container_vert_edge_padding - %vert_padding; 
                  } else {
                     %pos_y = %container_height - %height - %container_vert_edge_padding - %vert_padding;
                  }
               }
               %i++;
            }             
         } 
         else if (%bottom_anchor > %parent_id)
         {
            %i = 0;
            %found = false;
            while (%i < %count)
            {
               if ((%results[%i,0] == %bottom_anchor) && (%results[%i,%c]))
               {
                  %anchor_pos_y = %results[%i,17];
                  %anchor_height = %results[%i,4];
                  if (%vert_anchor_flip == false)
                     %pos_y = %anchor_pos_y - %height - %container_vert_padding - %vert_padding;
                  else
                     %pos_y = %anchor_pos_y + %anchor_height - %height - %vert_padding;
                  %found = true;
               }
               %i++;  
            }
            if ( !%found )
               %undefined = true;
         }
         
         if (%undefined)
         {
            //This should never happen, we wouldn't be here if we hadn't found our anchors already below.
            echo("OOPS! Gui control has undefined anchors when it is supposed to be a parent! " @ %currentElement);
            return;
         }
         
         //Save these back to the results array so other controls can find them.
         %results[%currentCounter,16] = %pos_x; 
         %results[%currentCounter,17] = %pos_y; 
         //echo("parent control " @ %currentElement @ " found a position: " @ %pos_x @ " " @ %pos_y );
         
         if (%type !$= "Virtual")
         {  
            %script = %script @ %indent @ "new " @ %type @ "() {\n";
            %script = %script @ %indent @ "   position = \"" @ mFloor(%pos_x) @ " " @ mFloor(%pos_y) @ "\";\n";
            %script = %script @ %indent @ "   extent = \"" @ %width @ " " @ %height @ "\";\n";
            if (strlen(%name)>0) %script = %script @ %indent @ "   internalName = \"" @ %name @ "\";\n";            
            if (strlen(%bitmap_path)>0) %script = %script @ %indent @ "   bitmap = \"" @ %bitmap_path @ "\";\n"; 
            if (strlen(%profile)>0) %script = %script @ %indent @ "   profile = \"" @ %profile @ "\";\n"; 
            %script = %script @ "\n";
         }
         %results[%currentCounter,%c] = true;//Whether or not we're virtual, don't come back here.
      }
      
      //////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////
      //Next, run through the children.      
      if ((%container_type !$= "Virtual")&&(%layers[%layerCount-1,2]==false))
      {//AH, but this needs to *not* add a new indent if we are coming back here after the first time.
         %indent = %indent @ "   ";  
         %layers[%layerCount-1,2] = true;
      }
      for (%k=0;%k<%currentChildCount;%k++)
      {
         %childCounter = 0;
         for (%d=0;%d<%count;%d++)
            if (%results[%d,0]==%currentChildren[%k])
               %childCounter = %d;
               
         //First, change the database's returned "NULL" values into actual null strings.
         for (%d=0;%d<%c;%d++)
            if (%results[%childCounter,%d] $= "NULL") 
               %results[%childCounter,%d] = "";
               
         //echo("checking child " @ %k @ " id=" @ %currentChildren[%k] @ ", finished=" @ %results[%childCounter,%c]);
         
         //Then, check for finished flag.
         if (%results[%childCounter,%c])
            continue;
         
         ///////////////////////////////
         
         //Now, check this child for its own children. If so, save current parent to %layers and increment %layerCount.
         %subfinished = true;
         %newChildCount = 0;
         for (%j=1;%j<%count;%j++)//Search through whole array of all results. (Except don't start at zero, that's always the form.)
         {
            if (%results[%j,1]==%currentChildren[%k]) // 1 = parent_id
            {
               %newChildCount++;
            }
         }
         if (%newChildCount>0)
         {
            %layers[%layerCount,0] = %currentElement; 
            %layers[%layerCount,1] = %indent;       
            %layers[%layerCount,2] = false;      
            %currentElement = %currentChildren[%k];
            %k = %currentChildCount; //Go to the end, exit loop.
            %subfinished = false;
            %layerCount++;
            continue;
         } ////////// Full stop if children found. Exit loop and start over, one layer deeper. //////////
            
         ///////////////////////////////
         //Now, if this is a leaf node, go ahead and render it and set type="";

         //First, give results some variable names.
         %id = %results[%childCounter,0];
         %parent_id = %results[%childCounter,1];
         %name = %results[%childCounter,2];
         %width = %results[%childCounter,3];
         %height = %results[%childCounter,4];               
         %type = %results[%childCounter,5]; 
         
         %bitmap_path = %results[%childCounter,6]; 
         %left_anchor = %results[%childCounter,7]; 
         %right_anchor = %results[%childCounter,8];
         %top_anchor = %results[%childCounter,9];
         %bottom_anchor = %results[%childCounter,10];

         %content = %results[%childCounter,11];
         %command = %results[%childCounter,12];
         %tooltip = %results[%childCounter,13];

         %horiz_align = %results[%childCounter,14];
         %vert_align = %results[%childCounter,15];
         
         %pos_x = %results[%childCounter,16]; 
         %pos_y = %results[%childCounter,17]; 

         %horiz_padding = %results[%childCounter,18];
         %vert_padding = %results[%childCounter,19];
         %horiz_edge_padding = %results[%childCounter,20];
         %vert_edge_padding = %results[%childCounter,21];

         %variable = %results[%childCounter,22];
         %button_type = %results[%childCounter,23];
         %group_num = %results[%childCounter,24];
         %profile = %results[%childCounter,25];
         %value = %results[%childCounter,26];
             
         //echo("reading element, name " @ %name @ " type " @ %type @ " id " @ %id );
         //And, UNFORTUNATELY, we (currently) need to repeat the whole block of anchor logic again here. 
         //Edit: REALLY?? Some way to refactor this MUST be possible. Out of time at the moment however.
         //Really not sure how to fix it. If TorqueScript had goto at least... but, nope.
         ///////////////////////////  BEGIN NASTY BLOCK OF REPEATED CODE /////////////////////////
         %i = 0;
         %container_width = 0;
         %container_height = 0;
         %container_pos_x = 0;
         %container_pos_y = 0;         
         %container_horiz_padding = 0;
         %container_vert_padding = 0;
         %container_horiz_edge_padding = 0;
         %container_vert_edge_padding = 0;
         while (%i < %count)
         {
            if (%results[%i,0] == %parent_id)
            {
               %container_width = %results[%i,3];
               %container_height = %results[%i,4];
               %container_pos_x = %results[%i,16];
               %container_pos_y = %results[%i,17];
               %container_horiz_padding = %results[%i,18];
               %container_vert_padding = %results[%i,19];
               %container_horiz_edge_padding = %results[%i,20];
               %container_vert_edge_padding = %results[%i,21];              
            }
            %i++;
         }
         
         %undefined = false;
         %horiz_anchor_flip = false;
         if (%left_anchor < 0)
         {
            %horiz_anchor_flip = true;
            %left_anchor *= -1;
         }
         if (%right_anchor < 0)
         {
            %horiz_anchor_flip = true;
            %right_anchor *= -1;
         }
         if ((strlen(%left_anchor)>0)&&(%left_anchor == %parent_id))
         {
            %i = 0;
            while (%i < %count)
            {
               if (%results[%i,0] == %parent_id)
               {
                  if (%results[%i,5] $= "Virtual")
                  {    
                     %pos_x = %container_pos_x + %container_horiz_edge_padding + %horiz_padding;      
                  } else { 
                     %pos_x = %container_horiz_edge_padding + %horiz_padding;
                  }
               }
               %i++;
            }               
         }
         else if (%left_anchor > %parent_id)
         {
            %i = 0;
            %found = false;
            while (%i < %count)
            {
               if ((%results[%i,0] == %left_anchor) && (%results[%i,%c]))
               {
                  %anchor_pos_x = %results[%i,16];
                  %anchor_width = %results[%i,3];
                  echo("child left anchor pos " @ %anchor_pos_x @ " width " @ %anchor_width);
                  if (%horiz_anchor_flip == false)
                     %pos_x = %anchor_pos_x + %anchor_width + %container_horiz_padding + %horiz_padding;
                  else
                     %pos_x = %anchor_pos_x + %horiz_padding;
                  
                  %found = true;
               }
               %i++;  
            }
            if ( !%found )
               %undefined = true;
         } 
         else if (%right_anchor == %parent_id) 
         {
            %i = 0;
            while (%i < %count)
            {
               if (%results[%i,0] == %parent_id)
               {
                  if (%results[%i,5] $= "Virtual")
                  {
                     %pos_x = %container_pos_x + %container_width - %width - %container_horiz_edge_padding - %horiz_padding;                     
                  } else {
                     %pos_x = %container_width - %width - %container_horiz_edge_padding - %horiz_padding;
                  }
               }
               %i++;
            }
         } 
         else if (%right_anchor > %parent_id)
         {
            %i = 0;
            %found = false;
            while (%i < %count)
            {
               if ((%results[%i,0] == %right_anchor) && (%results[%i,%c]))
               {
                  %anchor_pos_x = %results[%i,16];
                  %anchor_width = %results[%i,3];
                  if (%horiz_anchor_flip == false)
                     %pos_x = %anchor_pos_x - %width - %container_horiz_padding - %horiz_padding;
                  else
                     %pos_x = %anchor_pos_x + %anchor_width - %width - %horiz_padding;
                  
                  %found = true;
               }
               %i++;  
            }
            if ( !%found )
               %undefined = true;
         }

         ////// top/bottom anchors //////////////
         %vert_anchor_flip = false;
         if (%top_anchor < 0)
         {
            %vert_anchor_flip = true;
            %top_anchor *= -1;
         }
         if (%bottom_anchor < 0)
         {
            %vert_anchor_flip = true;
            %bottom_anchor *= -1;
         }
         if ((strlen(%top_anchor)>0)&&(%top_anchor == %parent_id)) //check for top anchor first, then bottom.
         {
            %i = 0;
            while (%i < %count)
            {
               if (%results[%i,0] == %parent_id)
               {
                  if (%results[%i,5] $= "Virtual")
                  {
                     %pos_y = %container_pos_y + %container_vert_edge_padding + %vert_padding;  
                  } else {
                     %pos_y = %container_vert_edge_padding + %vert_padding;
                  }
               }
               %i++;
            }      
         }
         else if (%top_anchor > %parent_id)
         {
            %i = 0;
            %found = false;
            while (%i < %count)
            {
               if ((%results[%i,0] == %top_anchor) && (%results[%i,%c]))
               {
                  %anchor_pos_y = %results[%i,17];
                  %anchor_height = %results[%i,4];
                  if (%vert_anchor_flip == false)
                     %pos_y = %anchor_pos_y + %anchor_height + %container_vert_padding + %vert_padding;
                  else
                     %pos_y = %anchor_pos_y + %vert_padding;
                  
                  %found = true;
               }
               %i++;  
            }
            if ( !%found )
               %undefined = true;            
         }   
         else if (%bottom_anchor == %parent_id) 
         {
            %i = 0;
            while (%i < %count)
            {
               if (%results[%i,0] == %parent_id)
               {
                  if (%results[%i,5] $= "Virtual")
                  {
                     %pos_y = %container_pos_y + %container_height - %height - %container_vert_edge_padding - %vert_padding; 
                  } else {
                     %pos_y = %container_height - %height - %container_vert_edge_padding - %vert_padding;
                  }
               }
               %i++;
            }             
         } 
         else if (%bottom_anchor > %parent_id)
         {
            %i = 0;
            %found = false;
            while (%i < %count)
            {
               if ((%results[%i,0] == %bottom_anchor) && (%results[%i,%c]))
               {
                  %anchor_pos_y = %results[%i,17];
                  %anchor_height = %results[%i,4];
                  if (%vert_anchor_flip == false)
                     %pos_y = %anchor_pos_y - %height - %container_vert_padding - %vert_padding;
                  else
                     %pos_y = %anchor_pos_y + %anchor_height - %height - %vert_padding;
                  %found = true;
               }
               %i++;  
            }
            if ( !%found )
               %undefined = true;
         }
         ////////////////////////////  END NASTY BLOCK OF REPEATED CODE //////////////////////////
         
         if (%undefined) //Bail if we are missing anchors, we will check again later.
         {
            //echo("control " @ %currentChildren[%k] @ " is currently undefined. Skipping for now.");
            continue;
         }
         %results[%childCounter,16] = %pos_x; 
         %results[%childCounter,17] = %pos_y; 
         //echo("Element " @ %currentChildren[%k] @ " found a position: " @ %pos_x @ " " @ %pos_y);

         if (%type !$= "Virtual") //Should never happen, but if someone accidentally adds a Virtual and doesn't
         {                       //give it any children, we still need to not render it.
            %script = %script @ %indent @ "new " @ %type @ "() {\n";
            %script = %script @ %indent @ "   position = \"" @ mFloor(%pos_x) @ " " @ mFloor(%pos_y) @ "\";\n";
            %script = %script @ %indent @ "   extent = \"" @ %width @ " " @ %height @ "\";\n";
            if (strlen(%content)>0) %script = %script @ %indent @ "   text = \"" @ %content @ "\";\n";
            if (strlen(%name)>0) %script = %script @ %indent @ "   internalName = \"" @ %name @ "\";\n";            
            if (strlen(%command)>0) %script = %script @ %indent @ "   command = \"" @ %command @ "\";\n";
            if (strlen(%tooltip)>0) %script = %script @ %indent @ "   tooltip = \"" @ %tooltip @ "\";\n"; 
            if (strlen(%tooltip)>0) %script = %script @ %indent @ "   tooltipprofile = \"GuiToolTipProfile\";\n";
            if (strlen(%bitmap_path)>0) %script = %script @ %indent @ "   bitmap = \"" @ %bitmap_path @ "\";\n"; 
            if (strlen(%variable)>0) %script = %script @ %indent @ "   variable = \"" @ %variable @ "\";\n"; 
            if (strlen(%button_type)>0) %script = %script @ %indent @ "   buttonType = \"" @ %button_type @ "\";\n"; 
            if (strlen(%group_num)>0) %script = %script @ %indent @ "   groupNum = \"" @ %group_num @ "\";\n"; 
            if (strlen(%profile)>0) %script = %script @ %indent @ "   profile = \"" @ %profile @ "\";\n"; 
            %script = %script @ %indent @ "};\n";
            %results[%childCounter,%c] = true; 
         }
         
         //Clear all, so you don't end up with the last control's tooltip because this one doesn't have one...
         %width = "";
         %height = "";
         %left_anchor = "";
         %right_anchor = "";
         %top_anchor = "";
         %bottom_anchor = "";
         %name = "";
         %type = "";
         %content = "";
         %command = "";
         %tooltip = "";
         %bitmap_path = "";
         %variable = "";
         %button_type = "";
         %group_num = "";
         %value = "";
         %profile = "";
         
      }//end of for (0..%currentChildCount) loop.
      
      //Now, double check, did we finish everybody?
      //Two ways we can not be done: either we were sent out of the loop because we need to go into a deeper
      //child layer, or we have children we passed by earlier because of undefined anchors.
      //This is for checking the second case. 
      for (%k=0;%k<%currentChildCount;%k++)
      {
         %childCounter = 0;
         for (%d=0;%d<%count;%d++)
            if (%results[%d,0]==%currentChildren[%k])
               %childCounter = %d;
               
          if (!%results[%childCounter,%c])
            %subfinished = false;               
      }
      
      //But now, if subfinished is still true, that means we are done with this container, go back up.
      if (%subfinished) //Only do this if we made it to the end of the children.
      {    
         %layerCount--;
         %currentElement = %layers[%layerCount,0];//-1, or just %layerCount?
         %indent = %layers[%layerCount,1];       
         if (%container_type !$= "Virtual")
         {
            %script = %script @ %indent @ "};\n";
         }       
      }
      
      //Finally: run through the whole list, and if type is null for every control, we're DONE done.
      for (%k=0;%k<%count;%k++)
      {
         if (!%results[%k,%c])
         {//exit with finished=false the first time we find a valid type.
            %k = %count;
            continue;
         }
         if (%k==(%count-1))
         {
            echo("finished main loop with all controls defined, sanityCount " @ %sanityCount);
            %finished=true;
         }
      }
   }
   
   //%script = %script @ "};\n";
   
   
   //And then, save it out as a .gui file.
   %filename = %formname @ ".gui";
   %fileObject = new FileObject();
   %fileObject.openForWrite( %filename ); 
   %fileObject.writeLine(%script);
   %fileObject.close();
   %fileObject.delete();
      
   echo("\n\n SCRIPT:\n\n" @ %script);
   echo("finished GUI: sanityCount " @ %sanityCount);
   
   //////////////////////////////
   eval(%script);  //DO IT!
   //////////////////////////////
   
   EWorldEditor.add(%formname);
   
}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////


