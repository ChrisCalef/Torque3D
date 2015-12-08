datablock StaticShapeData( StaticShapeBoulder )
{	
   shapeFile = "art/shapes/rocks/boulder.dts";
};

datablock ItemData( ItemBoulder )
{	
   shapeFile = "art/shapes/rocks/boulder.dts";
};

datablock StaticShapeData( LurkerSS )
{	
   shapeFile = "art/shapes/weapons/Lurker/TP_Lurker.DAE";
};

datablock ItemData( LurkerItem )
{	
   shapeFile = "art/shapes/weapons/Lurker/TP_Lurker.DAE";
};

datablock WheeledVehicleData(CustomCheetah : CheetahCar)
{
   nameTag = 'Custom Cheetah';
};

function CustomCheetah::onAdd(%this, %obj)
{
   CheetahCar::onAdd(%this, %obj);
   %obj.unmountImage(%this.turretSlot);
   
   // StaticShape
   %staticRock = new StaticShape() {
      datablock = StaticShapeBoulder;
      scale = "0.2 0.2 0.2";
   };
   %staticRock.setShapeName("StaticShape");
   %obj.staticRock = %staticRock;
   %staticRock.car = %obj;
   %obj.mountObject(%staticRock, %this.turretSlot, "1.4 0 .5 0 0 1 0");
   
   // Item
   %itemRock = new Item() {
      datablock = ItemBoulder;
      scale = "0.2 0.2 0.2";
   };
   %itemRock.setShapeName("Item");
   %obj.itemRock = %itemRock;
   %itemRock.car = %obj;
   %obj.mountObject(%itemRock, %this.turretSlot, "-1.4 0 .5 0 0 1 0");

   // RigidShape
   %rigidRock = new RigidShape() {
      datablock = BouncingBoulder;
      scale = "0.2 0.2 0.2";
   };
   %rigidRock.setShapeName("RigidShape");
   %obj.rigidRock = %rigidRock;
   %rigidRock.car = %obj;
   %obj.mountObject(%rigidRock, %this.turretSlot, "0 1.4 .6 0 0 1 0");

   // Vehicle
   %vehicleMount = new WheeledVehicle() {
      datablock = CheetahCar;
      scale = "0.1 0.1 0.1";
   };
   %vehicleMount.setShapeName("MountedCheetah");
   %obj.vehicleMount = %vehicleMount;
   %vehicleMount.car = %obj;
   %obj.mountObject(%vehicleMount, %this.turretSlot, "0 0 -.17 0 0 1 1.57");

   // TSStatic
   %tsStaticMount = new TSStatic() {
      shapeName = "art/shapes/rocks/boulder.dts";
      scale = "0.1 0.1 0.1";
   };
   %obj.tsStaticMount = %tsStaticMount;
   %tsStaticMount.car = %obj;
   %obj.mountObject(%tsStaticMount, %this.turretSlot, "0 1.4 0 0 0 1 0");

   // TSStatic MountEx
   %tsExMount = new TSStatic() {
      shapeName = "art/shapes/rocks/boulder.dts";
      scale = "0.15 0.15 0.15";
      skin = "MossyRock02=Grid512_OrangeLines_Mat";
   };
   %obj.tsExMount = %tsExMount;
   %tsExMount.car = %obj;
   %obj.mountObjectEx(%tsExMount, "Hub2", "", "-1.2 0 0 0 0 1 0");

   // RigidShape MountEx
   %rigidRock = new RigidShape() {
      datablock = BouncingBoulder;
      scale = "0.15 0.15 0.15";
      skin = "MossyRock02=Grid512_OrangeLines_Mat";
   };
   %rigidRock.setShapeName("RigidShape");
   %obj.rigidRockEx = %itemRock;
   %rigidRock.car = %obj;
   %obj.mountObjectEx(%rigidRock, "Hub3", "", "1.2 0 0 0 0 1 0");

   // StaticShape Lurker MountEx
   %staticLurker = new StaticShape() {
      datablock = LurkerSS;
      scale = "1.5 1.5 1.5";
   };
   %staticLurker.setShapeName("StaticShape");
   %obj.staticLurker = %staticLurker;
   %staticLurker.car = %obj;
   %obj.mountObjectEx(%staticLurker, "Hub1", "MuzzlePoint", "-.85 0 0 1 0 0 1.57");
   
   // Item Lurker MountEx
   %itemLurker = new Item() {
      datablock = LurkerItem;
      scale = "1.5 1.5 1.5";
   };
   %itemLurker.setShapeName("Item");
   %obj.itemLurker = %itemLurker;
   %itemLurker.car = %obj;
   %obj.mountObjectEx(%itemLurker, "Hub0", "MuzzlePoint", ".85 0 0 1 0 0 1.57");

   for (%i = 0; %i < %obj.getMountedObjectCount(); %i++ )
      echo("Object #" @ %i SPC %obj.getMountedObject(%i).getClassName() @ " Mounted to " @ %obj.getMountedObjectNodeEx(%i));
}

function CustomCheetah::onRemove(%this, %obj)
{
   %count = %obj.getMountedObjectCount();
   for (%i = %count-1; %i >= 0; %i--)
   {
      %mountObj = %obj.getMountedObject(%i);
      %obj.unmountObject(%mountObj);
      %mountObj.delete();
   }

   CheetahCar::onRemove(%this, %obj);
}

