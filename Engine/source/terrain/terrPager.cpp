//-----------------------------------------------------------------------------
// Copyright (c) 2015 Chris Calef
//-----------------------------------------------------------------------------

#include "console/engineAPI.h"
#include "platform/platform.h"
#include "terrain/terrPager.h"
#include "console/consoleTypes.h"
#include "core/stream/fileStream.h"
#include "T3D/gameBase/gameConnection.h"
#include "T3D/camera.h"
#include "T3D/player.h"
#include "gfx/bitmap/gBitmap.h"
#include "gui/worldEditor/terrainEditor.h"
#include "sim/dataSource/worldDataSource.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <time.h>

using std::vector;
using std::string;


IMPLEMENT_CONOBJECT( TerrainPager );

ConsoleDocClass( TerrainPager,
	"@brief The TerrainPager class organizes a large grid of terrain tiles and potentially skybox images, and queries a WorldDataSource object for additional data.");

TerrainPager::TerrainPager()
{
	mCurrentTick = 0;
	mTickInterval = 10;
	mSkyboxRes = 800;

	mLastSkyboxTick = 0;
	mSkyboxTickInterval = 360;//FIX!!! GUI!!
	mSkyboxLoadDelay = 8;
	
	mTerrainRequestTick = 0;
	mSkyboxRequestTick = 0;

	mLastForestTick = 0;
	mForestTickInterval = 10;
	mCellGridSize = 64;
	mTreeRadiusMult = 1.0;
	mForestTries = 200;
	mForestRadius = 180;
	mStreetRadius = 320;

	mLoadState = 0;//For now just use numbers, make an enum when you know what the states are going to be.

	mDataSource = NULL;
	mSQL = NULL;
	mSentInitRequests = false;
	mLoadedTileGrid = false;
	mSentTerrainRequest = false;
	mSentSkyboxRequest = false;
	mForest = NULL;
	mForestStarted = false;
	mDoForest = true;
	mDoForestUpdates = true;
	mDoStreets = true;
	mDoStreetUpdates = true;
	mDoStaticShapes = true;
	mDoStaticShapeUpdates = true;
	mForestItemData = NULL;

	mClientPos.zero();
	mStartPos.zero();

	//////////////////////////////
	mD.mMapCenterLongitude = 0.0;
	mD.mMapCenterLatitude = 0.0;
	
	mD.mClientPosLongitude = 0.0f;
	mD.mClientPosLatitude = 0.0f;
	mD.mClientPosAltitude = 0.0f;
	
	mD.mTileLoadRadius = 600.0f;
	mD.mTileDropRadius = 6000.0f;

	mD.mTileWidth = 0.0f;//2550 m squares, by default, in a 256x256 terrain grid with 10m square size.
	mD.mTileWidthLongitude = 0.0f;
	mD.mTileWidthLatitude = 0.0f;
	
	//Now, dip into wide characters for a minute to use GetModuleFileName, in order to retreive the path to our game 
	//directory. (Easier way somewhere?). Meanwhile, note to self, fix this to use wide strings all the time, because unicode.
	wchar_t wtext[MAX_PATH];
	long x = GetModuleFileName(NULL, wtext, MAX_PATH); 
	String gamePath = wtext;
	gamePath.replace("openSimEarth.exe","");//FIX!!! This is why it doesn't work with debug mode! Also you can't change the
	//name of the executable without coming back here. Would be nice to find the name of this executable procedurally.
	//In the meantime, I guess we can do this for the debug build, it won't do anything if it doesn't find it.
	gamePath.replace("openSimEarth_DEBUG.exe","");

	gamePath.replace("\\","/");//not too confident about sending backslashes through the system...

	mD.mTerrainPath = gamePath + "art/terrains/openSimEarth/";//FIX: Hard coded for now,
	mD.mSkyboxPath  = gamePath + "art/skies/night/";//   allow user prefs for these later.
	
	//mD.mTerrainLockfile = mD.mTerrainPath + "lockfile.terrain.tmp";//Revisit this when paths are
	//mD.mSkyboxLockfile = mD.mSkyboxPath + "lockfile.skybox.tmp";//   working from script as they should.

	mD.mSkyboxRes = mSkyboxRes;
	//////////////////////////////
}

TerrainPager::~TerrainPager()
{	
	//Note: make sure to delete all terrainBlock objects, this will only delete pointers themselves I think.
	///Although if they were added to the scene, then they should get deleted automatically on scene exit.
	mTerrains.clear();
	mTerrainGrid.clear();
	if ((mUseDataSource)&&(mDataSource))
		delete mDataSource;

	if (mSQL) 
	{
		mSQL->CloseDatabase();
		delete mSQL;
	}
}

void TerrainPager::initPersistFields()
{
	
	//This should represent a point at the center of the entire region of interest. Necessary to prevent floating point error.
	addField( "mapCenterLongitude", TypeF32, Offset( mMapCenterLongitude, TerrainPager ), "Map Center, Longitude" );
	addField( "mapCenterLatitude", TypeF32, Offset( mMapCenterLatitude, TerrainPager ), "Map Center, Latitude" );
		
	addField( "tileLoadRadius", TypeF32, Offset( mTileLoadRadius, TerrainPager ), "Radius at which to load a tile." );
	addField( "tileDropRadius", TypeF32, Offset( mTileDropRadius, TerrainPager ), "Radius at which to drop a tile." );
		
	addField( "forestRadius", TypeF32, Offset( mForestRadius, TerrainPager ), "Forest radius." );
	addField( "streetRadius", TypeF32, Offset( mStreetRadius, TerrainPager ), "Street radius." );
	addField( "forestTries", TypeS32, Offset( mForestTries, TerrainPager ), 
		"How many tries we take to fill a forest cell (increases density, up to amount allowed by treeRadiusMult)." );
	addField( "treeRadiusMult", TypeF32, Offset( mTreeRadiusMult, TerrainPager ), "Tree radius multiplier, lower number means denser forest." );
	addField( "cellGridSize", TypeS32, Offset( mCellGridSize, TerrainPager ), "Cell grid size." );
	addField( "forestTerrainEdgeDistance", TypeF32, Offset( mForestTerrainEdgeDistance, TerrainPager ), 
												"Forest edge avoidance distance for terrain tiles in lat/long. (Temporary, terrain edge bug workaround.)" );

	addField( "skyboxRes", TypeS32, Offset( mSkyboxRes, TerrainPager ), "Skybox image resolution." );

	addField( "forestTickInterval", TypeS32, Offset( mForestTickInterval, TerrainPager ), "Forest tick interval (30 = 1 second)." );
	
	addField( "useDataSource", TypeBool, Offset(mUseDataSource, TerrainPager ), "Use worldDataSource for terrain/skybox updates.");
	addField( "doForest",  TypeBool, Offset(mDoForest, TerrainPager ), "Set to false to turn off all random forest.");
	addField( "doForestUpdates",  TypeBool, Offset(mDoForestUpdates, TerrainPager ), "Set to false to turn off random forest paging.");
	addField( "doStreets",  TypeBool, Offset(mDoStreets, TerrainPager ), "Set to false to turn off all osm streets.");
	addField( "doStreetUpdates",  TypeBool, Offset(mDoStreetUpdates, TerrainPager ), "Set to false to turn off osm street paging.");
	addField( "doStaticShapes",  TypeBool, Offset(mDoStaticShapes, TerrainPager ), "Set to false to turn off all saved static shapes.");
	addField( "doStaticShapeUpdates",  TypeBool, Offset(mDoStaticShapeUpdates, TerrainPager ), "Set to false to turn off osm static shape paging.");

	Parent::initPersistFields();
}

bool TerrainPager::onAdd()
{
	mD.mMapCenterLongitude = mMapCenterLongitude;
	mD.mMapCenterLatitude = mMapCenterLatitude;

	mD.mTileLoadRadius = mTileLoadRadius;
	mD.mTileDropRadius = mTileDropRadius;

	mD.mSkyboxRes = mSkyboxRes;

	mSQL = new SQLiteObject();

	if (!mSQL->OpenDatabase(Con::getVariable("$pref::OpenSimEarth::OSMDB")))
		Con::printf("TerrainPager: cannot open map db: %s",Con::getVariable("$pref::OpenSimEarth::OSMDB"));
	
	if ( !Parent::onAdd() )
		return false;
	
	
	//terrain_materials.push_back( "city1" );
	//terrain_materials.push_back( "forest1a" );
	//terrain_materials.push_back( "drycrop2" );
	//terrain_materials.push_back( "grass_green_hires" );
	//terrain_materials.push_back( "sand_hires" );
	//terrain_materials.push_back( "water" );
	//terrain_materials.push_back( "asphalt" );

	//FIX!! Need to read materials from terrain block, OR get a list of texture names from FG.
	terrain_materials.push_back( "TT_Gravel_02" );
	terrain_materials.push_back( "TT_Mud_07" );
	terrain_materials.push_back( "TT_Grass_20" );
	terrain_materials.push_back( "TT_Snow_01" );
	terrain_materials.push_back( "TT_Sand_01" );
	terrain_materials.push_back( "TT_Rock_14" );
	terrain_materials.push_back( "forest1a_mat" );
	terrain_materials.push_back( "TT_Grass_01" );

	SimSet* scopeAlwaysSet = Sim::getGhostAlwaysSet();
	const TerrainBlock *tempTerrain;
	for(SimSet::iterator itr = scopeAlwaysSet->begin(); itr != scopeAlwaysSet->end(); itr++)
	{
		TerrainBlock* block = dynamic_cast<TerrainBlock*>(*itr);
		if( block )
		{
			mTerrain = block;
			
			mD.mLightmapRes = block->getLightMapSize();
			mD.mTextureRes = block->mBaseTexSize;
			mD.mHeightmapRes = block->getBlockSize();
			mD.mSquareSize = block->getSquareSize();

			mD.mTileWidth = (mD.mHeightmapRes - 1) * mD.mSquareSize;

			//Following formulas are from http://fmepedia.safe.com/articles/How_To/Calculating-accurate-length-in-meters-for-lat-long-coordinate-systems
			F32 rLat = mDegToRad(mD.mMapCenterLatitude);

			mD.mMetersPerDegreeLatitude = 111132.92 - 559.82 * cos(2 * rLat) + 1.175 * cos(4 * rLat);
			mD.mMetersPerDegreeLongitude = 111412.84 * cos(rLat) - 93.5 * cos(3 * rLat);
						
			mD.mDegreesPerMeterLongitude = 1.0f / mD.mMetersPerDegreeLongitude ;
			mD.mDegreesPerMeterLatitude = 1.0f / mD.mMetersPerDegreeLatitude ;
			
			mD.mTileWidthLongitude = mD.mDegreesPerMeterLongitude * mD.mTileWidth;
			mD.mTileWidthLatitude = mD.mDegreesPerMeterLatitude * mD.mTileWidth;

			//for (U32 i=0;i<mTerrain->mBaseTextures.size();i++)
			//{
			//	Con::printf("material %d: %s",i,mTerrain->mBaseTextures[i].??? // Is there any way to get name back out??
			//}
		}
	}
	//Now, try this again to see if we can find our Forest object. If not this, look for "theForest".
	for(SimSet::iterator itr = scopeAlwaysSet->begin(); itr != scopeAlwaysSet->end(); itr++)
	{
		Forest* kForest = dynamic_cast<Forest*>(*itr);
		if( kForest )
		{
			mForest = kForest;
			mForest->getData()->getDatablocks( &mForestItemData );
			U32 count = mForestItemData.size();
			Con::printf("found a forest with %d datablocks!!!!!!!!!!!!!!!!!!!!! %s %s %s %s",count,mForestItemData[0]->getName(),
				mForestItemData[1]->getName(),mForestItemData[2]->getName(),mForestItemData[3]->getName());
		}
	}
	//delete mTerrain;//Is this safe, at all? What is proper way to remove a terrain block? MissionGroup.remove()?

	//mD.mSkyboxLockfile = mD.mSkyboxPath + "lockfile.skybox.tmp";
	//mD.mTerrainLockfile = mD.mTerrainPath + "lockfile.terrain.tmp";
	
	findClientTile();

	//This can only be 3x3, 5x5, etc.
	mGridSize = 1 + (2 * ((int)(mD.mTileLoadRadius /  mD.mTileWidth) + 1));
	Con::printf("found terrain! tile width: %f tileWidthLat %f tileWidthLong %f mperdeg_lon %f mperdeg_lat %f",
				mD.mTileWidth,mD.mTileWidthLatitude,mD.mTileWidthLongitude,mD.mMetersPerDegreeLongitude,mD.mMetersPerDegreeLatitude);

	//NOW, as soon as you've defined mGridSize, you are ready to set up your sparse Vectors.
	for (int y=0;y<mGridSize;y++) {
		for (int x=0;x<mGridSize;x++) {
			mTerrainGrid.increment();
			mTerrainGrid[y*mGridSize+x] = NULL;
		}
	}

	//mUseDataSource = true;//Get this from persist fields.
	if (mUseDataSource)
	{
		Con::printf("Terrain pager is using a worldDataSource.");
		mDataSource = new worldDataSource(false,&mD);
		//mDataSource = new dataSource(false);
	}

	mCellWidth = mD.mTileWidth / mCellGridSize;
	mCellArea = pow(mCellWidth,2);
	mMinCellArea = mCellArea * 0.1;//start with ten percent...
	Con::printf(" cellGridSize %d  cell width: %f  cell area %f",mCellGridSize,mCellWidth,mCellArea);
	Con::executef(this, "onAdd", getIdString());

	return true;
}

void TerrainPager::processTick()
{
	bool newTile = false;

	//First, establish current position.
	findClientTile();	

	if (mClientPos.len()==0.0)
	{//Check to make sure we have a reasonable player location. [NOTE: Do *not* make player spawn at perfect (0,0,0)!]
		return;//No player yet.
	}
	
	if (mUseDataSource)
	{
		mDataSource->tick();
	}
	if (mLoadState==0)//First pass, just found our first useful client position.
	{
		Con::printf("terrainPager first client info: long/lat %f %f, pos %f %f",
					mD.mClientPosLongitude,mD.mClientPosLatitude,mClientPos.x,mClientPos.y);

		mLastTileStartLong = mTileStartLongitude;
		mLastTileStartLat = mTileStartLatitude;
		
		if (!mUseDataSource) //If not using data source, just page around the finished terrains.
		{
			loadTileGrid();
			mLoadState = 10;//10 is the holding pattern loop.
		} else {
			mLoadState = 1;
		}
	}
	else if (mLoadState==1) //Wait till dataSource is hooked up and ready to go, then send init terrain and skybox requests.
	{	
		if (mDataSource->mReadyForRequests)
		{
			mDataSource->addInitTerrainRequest(&mD,mD.mTerrainPath.c_str());
			mDataSource->addInitSkyboxRequest(mD.mSkyboxRes,0,mD.mSkyboxPath.c_str());
			mSentInitRequests = true;
			mLoadState = 2;  
		} else 
			Con::printf("TerrainPager waiting for worldDataSource to be ready.");
	}
	else if (mLoadState==2) //Call loadTileGrid, which will try to load existing terrains and request 
	{						// data for anything it can't find locally.
		loadTileGrid();
	}
	else if (mLoadState==3) //Waiting for terrain data.
	{
		if (mDataSource->mTerrainDone==true)
		{
			mLoadState=2;
		} else {
			Con::printf("TerrainPager waiting for terrain data...");
			if ((S32)mTerrainRequestTick < ((S32)mCurrentTick - (S32)mSkyboxTickInterval))
			{
				mLoadState=10;//Give up on this load, it got lost in transmission. (?)
			}
		}
	}
	//else if (mLoadState==4) //Waiting for skybox images.
	//{
	//	if (mDataSource->mSkyboxDone==true)
	//	{
	//		Con::printf("reloading skybox!!!!");
	//		reloadSkyboxImages(); //Rotate and flip the raw images from Flightgear to make them work in T3D skybox material.
	//		mSentSkyboxRequest = false;
	//		mLastSkyboxTick = mCurrentTick;
	//		mLoadState=5;
	//	} else {
	//		Con::printf("TerrainPager waiting for skybox images...");
	//	}
	//}
	else if (mLoadState==5) 
	{
		//if ((mCurrentTick - mLastSkyboxTick) > mSkyboxLoadDelay)
		//{
		//Hmm, trying this again, with no delay except one tick, is that enough?
			Con::printf("updating skybox!!!!!!!!!!");
			Con::executef(this,"UpdateSkybox");
			mLoadState = 10;
		//}
	}
	else if (mLoadState==10)
	{
		if (mLastTileStartLong != mTileStartLongitude) newTile = true;
		if (mLastTileStartLat  != mTileStartLatitude) newTile = true;
		if (newTile) 
		{
			Con::printf("new tile: last pos %f %f current pos %f %f",mLastTileStartLong,mTileStartLatitude,
								mTileStartLongitude,mTileStartLatitude);
			loadTileGrid();
		} else {
			if (mCurrentTick++ % mTickInterval == 0) //Hmm, might want to rename mTickInterval to mTileGridInterval, this is the only place it's used.
				checkTileGrid();
		}		
		
		//Con::printf("terrain pager load state: %d sendControls %d skyboxInterval %d currentTick %d",
		//	mLoadState,mDataSource->mSendControls,mSkyboxTickInterval,mCurrentTick);

		if ((mUseDataSource)&&(mDataSource->mSendControls==1)&&
			((S32)mLastSkyboxTick < ((S32)mCurrentTick - (S32)mSkyboxTickInterval))&&
			(mSentSkyboxRequest == false))
		{
			mDataSource->addSkyboxRequest(mTileStartLongitude,mTileStartLatitude,mD.mClientPosLongitude,mD.mClientPosLatitude,mD.mClientPosAltitude);
			//mLoadState = 4;
			mSentSkyboxRequest = true;
			mDataSource->mSkyboxDone = false;
			//mLastSkyboxTick = mCurrentTick;
			mSkyboxRequestTick = mCurrentTick;
			Con::printf("adding a skybox request! ");
		}
		else if ( (mSentSkyboxRequest) && //Either we're done, or we've waited too long and should give up on this one.
				((mDataSource->mSkyboxDone) || ((S32)mSkyboxRequestTick < ((S32)mCurrentTick - (S32)mSkyboxTickInterval))) )
		{
			Con::printf("reloading skybox!!!!");
			reloadSkyboxImages(); //Rotate and flip the raw images from Flightgear to make them work in T3D skybox material.
			mSentSkyboxRequest = false;
			mLastSkyboxTick = mCurrentTick;
			mLoadState=5;
		}

		if ( ((mCurrentTick-mLastForestTick) > mForestTickInterval) )//mDoForest && - now checking doForest later, so we can do streets w/o forest
			checkForest();
	}	
}

//void TerrainPager::updateForest()
//{
//}

void TerrainPager::checkForest()
{
	//Now, we should have a clientPos already, and an mForest object, so all we need to do is A) remove trees outside of
	//or mForestRadius, and then the hard part, add trees to areas inside our mForestRadius that are not already forested.
	//Need to work it by cells for this part.
	//First, let's just remove anything larger than mForestRadius.
	if ( mDoForest && !mForest )
		return;
		
	unsigned long startTime =  clock();

	Vector<ForestItem> items;
	char cellName[20];
	int j;
	
	//getCellCoordsFromName(cellName);
	
	if (!mForestStarted)   	
	{  //So, first thing we need to do is see what forest is already in the level. 
		Con::printf("tile width: %f, cell grid size %d, total area %f cellWidth %f tree count %d",
												mD.mTileWidth,mCellGridSize,mCellArea,mCellWidth,items.size());
	
      //String osmFile( Con::getVariable( "$pref::OpenSimEarth::OSM" ) );
      //String mapDB( Con::getVariable( "$pref::OpenSimEarth::OSMDB" ) );
		//loadOSM(osmFile.c_str(),mapDB.c_str());
		if (mDoStaticShapes)
		{
			makeStaticShapes();
		}

		if (mDoStreets)
		{
			findStreetNodes();
			makeRoads();
		}

		if (mDoForest)
		{			
			mForest->getData()->getItems(&items);
			for (U32 i=0;i<items.size();i++)
			{
				//strcpy(cellName,getCellName(items[i].getPosition()))
				getCellName(items[i].getPosition(),cellName);
				//Point3F longLatPos = convertXYZToLatLong(items[i].getPosition());
				//getCellName(longLatPos.x,longLatPos.y,cellName);
				//std::string celName;//wtf is happening. 
				//celName = getCellName(items[i].getPosition());
				float area = M_PI * pow((items[i].getRadius() * items[i].getScale()),2);//pi times radius squared, with scale.
				area *= 0.5; //Overlap percentage of 0.5, might want to expose this as a script variable.

				if (mCellGrid[cellName]>0.0)
					mCellGrid[cellName] += area;
				else 
					mCellGrid[cellName] = area;

				Con::printf("cell %s area: %f, total %f radius %f  scale %f",
					cellName,area,mCellGrid[cellName],items[i].getRadius(),items[i].getScale());
			}

			Con::printf("Before filling forest, total occupied cells %d",mCellGrid.size());
			j=0;
			for (std::map<std::string,float>::iterator it=mCellGrid.begin(); it!=mCellGrid.end(); ++it)
			{			
				strcpy(cellName,it->first.c_str());
				//Con::printf("cellname %s cell %d  area filled %f  available %f ",
				//	cellName,j++,mCellGrid[cellName],mCellArea-mCellGrid[,]);
			}

			fillForest();

			Con::printf("After filling, total cells size %d",mCellGrid.size());
			j=0;
			for (std::map<std::string,float>::iterator it=mCellGrid.begin(); it!=mCellGrid.end(); ++it)
			{			
				strcpy(cellName,it->first.c_str());
				Con::printf("cellname %s cell %d  area filled %f  available %f ",
					cellName,j++,mCellGrid[cellName],mCellArea-mCellGrid[cellName]);

			}
		}

		mForestStarted = true;
		mLastForestTick = mCurrentTick;
		return;
	} 
	
	//DELETION PASS
	if (1)//if (allow_delete)
	{		
		if ((mDoForest)&&(mDoForestUpdates))
		{
			for (std::map<std::string,float>::iterator it=mCellGrid.begin(); it!=mCellGrid.end(); ++it)
			{			
				strcpy(cellName,it->first.c_str());
				Point2F cellPosLatLong = getCellCoordsFromName(cellName);
				//Con::printf("checking cellname %s %f %f",cellName);
				F32 closestDist = getForestCellClosestDist(cellPosLatLong,mClientPos);
				if (closestDist > (mForestRadius + mCellWidth))
				{
					//Con::printf("CLEARING cellname %s %f %f",cellName,cellPosLatLong.x,cellPosLatLong.y);
					clearForestCell(cellPosLatLong);
				}
				//Con::printf("cellname %s cell %d  area filled %f  available %f ",
				//	cellName,j++,mCellGrid[cellName],mCellArea-mCellGrid[cellName]);
			}
		}

		//if ((mDoStreets)&&(mDoStreetUpdates))
		//	pruneRoads();//Whoops, this is breaking hard, fix it later.

		//mForest->updateCollision();//Costly, only doing this on addition pass, why do it twice.
	}
	
	//ADDITION PASS
	if ((mDoStreets)&&(mDoStreetUpdates))
	{
		findStreetNodes();
		makeRoads();
	}
	if ((mDoForest)&&(mDoForestUpdates))
		fillForest();
	
	//mForest->getData()->getItems(&items);
	//Con::printf("After filling, total cells size %d, tree count %d",mCellGrid.size(),items.size());
	//j=0;
	//for (std::map<std::string,float>::iterator it=mCellGrid.begin(); it!=mCellGrid.end(); ++it)
	//{			
	//	strcpy(cellName,it->first.c_str());
	//	Con::printf("cellname %s cell %d  area filled %f  available %f ",
	//		cellName,j++,mCellGrid[cellName],mCellArea-mCellGrid[cellName]);
	//}
	
	unsigned int latency = clock() - startTime;
	//Con::printf("checkForest took %d milliseconds",latency);
	mLastForestTick = mCurrentTick;
}

void TerrainPager::makeStaticShapes()
{
	Point2F baseCellCoords = getCellCoords(mClientPos);
	Point3F baseCell = convertLatLongToXYZ(Point3F(baseCellCoords.x,baseCellCoords.y,0.0));
	Point3F startCell = baseCell;//This will be moving with every loop.
	S32 loops = 0;
	char cellName[20];
	Vector<std::string> activeCells;

	unsigned long startTime =  clock();

	getCellName(baseCellCoords.x,baseCellCoords.y,cellName);
	activeCells.push_back(cellName);//NOW, we are just going to make a list of cellnames, and submit them all in one query.
	loops++;

	//NOW, to loop around in ever expanding squares until we are entirely clear of streetRadius.
	Point3F iterCell;

	//Note: this is *not* the same as taking a proper vector length from baseCell to startCell - by design, because we need the whole
	//column and row to be outside of forest radius
	while ( ((baseCell.x-startCell.x)<=mForestRadius) || ((baseCell.y-startCell.y)<=mForestRadius) )
	{
		startCell.x -= mCellWidth;
		startCell.y -= mCellWidth;
		iterCell = startCell;

		Point3F cellPosLatLong;
		F32 closestDist;

		for (U32 i=0;i<(loops*2)+1;i++) // Left side, bottom to top.
		{
			if (i>0) iterCell.y += mCellWidth;
			cellPosLatLong = convertXYZToLatLong(iterCell);
			getCellName(cellPosLatLong.x,cellPosLatLong.y,cellName);
			closestDist = getForestCellClosestDist(Point2F(cellPosLatLong.x,cellPosLatLong.y),mClientPos);
			if  (closestDist<mForestRadius)
			{
				activeCells.push_back(cellName);
			}
		}
		for (U32 i=1;i<(loops*2)+1;i++) // Top, left to right.
		{
			if (i>0) iterCell.x += mCellWidth;
			cellPosLatLong = convertXYZToLatLong(iterCell);
			getCellName(cellPosLatLong.x,cellPosLatLong.y,cellName);
			closestDist = getForestCellClosestDist(Point2F(cellPosLatLong.x,cellPosLatLong.y),mClientPos);
			if  (closestDist<mForestRadius)
			{
				activeCells.push_back(cellName);
			}
		}
		for (U32 i=1;i<(loops*2)+1;i++) // Right, top to bottom.
		{
			if (i>0) iterCell.y -= mCellWidth;
			cellPosLatLong = convertXYZToLatLong(iterCell);
			getCellName(cellPosLatLong.x,cellPosLatLong.y,cellName);
			closestDist = getForestCellClosestDist(Point2F(cellPosLatLong.x,cellPosLatLong.y),mClientPos);
			if  (closestDist<mForestRadius)
			{
				activeCells.push_back(cellName);
			}
		}
		for (U32 i=1;i<(loops*2);i++) // Bottom, right to left.
		{
			if (i>0) iterCell.x -= mCellWidth;
			cellPosLatLong = convertXYZToLatLong(iterCell);
			getCellName(cellPosLatLong.x,cellPosLatLong.y,cellName);
			closestDist = getForestCellClosestDist(Point2F(cellPosLatLong.x,cellPosLatLong.y),mClientPos);
			if  (closestDist<mForestRadius)
			{
				activeCells.push_back(cellName);
			}
		}
		loops++;
	}

	//NOW, we have a list of cellnames, let's use them:
	if (mSQL)//->OpenDatabase(Con::getVariable("$pref::OpenSimEarth::OSMDB")))
	{
		long nodeId,fileId,posId,rotId,scaleId,shapeId;
		int result,total_query_len=0;
		double nodeLong,nodeLat;
		std::string shapeFile,name;
		std::ostringstream selectQuery;
		sqlite_resultset *resultSet;
		//"WHERE n.type LIKE 'TSSTatic' AND " <<
			//"n.cellName IN  ( ";
		selectQuery <<
			"SELECT n.id,s.id,n.name,f.path,p.x,p.y,p.z,r.x,r.y,r.z,r.w,sc.x,sc.y,sc.z " <<
			"FROM osmNode n " <<
			"JOIN shape s ON n.id = s.node_id " <<
			"JOIN shapeFile f ON f.id = s.file_id " <<
			"JOIN vector3 p ON p.id = s.pos_id " <<
			"JOIN rotation r ON r.id = s.rot_id " <<
			"JOIN vector3 sc ON sc.id = s.scale_id " <<
			"WHERE n.type LIKE 'TSSTatic' AND " <<
			"n.cell_name IN  ( ";

		for (U32 i=0;i<activeCells.size();i++)
		{
			if (i<(activeCells.size()-1))
				selectQuery << "'" << activeCells[i].c_str() << "', ";
			else
				selectQuery << "'" << activeCells[i].c_str() << "' ";

		}
		selectQuery << " );";

		//U32 queryTime = clock();
		result = mSQL->ExecuteSQL(selectQuery.str().c_str());
		if (result)
		{
			resultSet = mSQL->GetResultSet(result);
			selectQuery.clear();
			selectQuery.str("");
			if (resultSet->iNumRows > 0)
			{
				//Con::printf("found %d staticShapes, query took %d milliseconds",resultSet->iNumRows,clock()-queryTime);
				SimSet* missionGroup = NULL;
				missionGroup = dynamic_cast<SimSet*>(Sim::findObject("MissionGroup"));

				for (U32 i=0;i<resultSet->iNumRows;i++)
				{
					int c=0;
					float x,y,z,w;
					Point3F pos,scale;
					QuatF rot;

					nodeId = dAtol(resultSet->vRows[i]->vColumnValues[c++]);//This has to be long, because it's coming from OSM and might be huge.
					shapeId = dAtoi(resultSet->vRows[i]->vColumnValues[c++]);//This is simply an autoincrement from our own table, unlikely to exceed int limits.
					name =  resultSet->vRows[i]->vColumnValues[c++];
					shapeFile = resultSet->vRows[i]->vColumnValues[c++];
					x = dAtof(resultSet->vRows[i]->vColumnValues[c++]);
					y = dAtof(resultSet->vRows[i]->vColumnValues[c++]);
					z = dAtof(resultSet->vRows[i]->vColumnValues[c++]);
					pos = Point3F(x,y,z);

					x = dAtof(resultSet->vRows[i]->vColumnValues[c++]);
					y = dAtof(resultSet->vRows[i]->vColumnValues[c++]);
					z = dAtof(resultSet->vRows[i]->vColumnValues[c++]);
					w = dAtof(resultSet->vRows[i]->vColumnValues[c++]);
					rot = QuatF(x,y,z,w);

					x = dAtof(resultSet->vRows[i]->vColumnValues[c++]);
					y = dAtof(resultSet->vRows[i]->vColumnValues[c++]);
					z = dAtof(resultSet->vRows[i]->vColumnValues[c++]);
					scale = Point3F(x,y,z);

					if (mStaticShapes.find(shapeId) == mStaticShapes.end())
					{
						TSStatic *shape = new TSStatic();
						MatrixF transform;
						rot.setMatrix(&transform);
						transform.setPosition(pos);
						shape->setTransform(transform);
						shape->setScale(scale);
						shape->mShapeName = StringTable->insert(shapeFile.c_str());
						shape->mOseId = shapeId;

						shape->registerObject( name.c_str() );

						missionGroup->addObject(shape);
						mStaticShapes[shapeId] = shape;

					}
				}
			} 
		} else Con::printf("Static shape query found no results.");
	}

	unsigned int latency = clock() - startTime;
	Con::printf("makeStaticShapes() took %d milliseconds",latency);
}

void TerrainPager::saveStaticShapes()
{//Okay, round two! Did this on the script side first, but for float precision reasons as well as performance,
	//it's time to move back into the engine instead.
	SimSet* missionGroup = NULL;
	missionGroup = dynamic_cast<SimSet*>(Sim::findObject("MissionGroup"));

	unsigned long startTime =  clock();

	if (missionGroup == NULL)
		Con::printf("No mission group found.");

	if (!mSQL)
		Con::printf("mSQL is not valid");

	if ((mSQL)&&(missionGroup))//->OpenDatabase(Con::getVariable("$pref::OpenSimEarth::OSMDB")))
	{
		int nodeId,shapeId,fileId,posId,rotId,scaleId;
		int result,total_query_len=0;
		double nodeLong,nodeLat;
		std::string shapeName;
		std::ostringstream insert_query,update_query,file_id_query,scale_id_query;
		sqlite_resultset *resultSet;
		Point3F pos,latLongPos,scale;
		Point2F cellPos;
		char cellName[20];

		//for(SimSet::iterator itr=missionGroup->begin(); itr!=missionGroup->end(); ++itr)
		for(SimSetIterator itr(missionGroup); *itr; ++itr)
      {
         TSStatic* object = dynamic_cast< TSStatic* >( *itr );
			if (object)
			{
				pos = object->getPosition();
				latLongPos = convertXYZToLatLong(pos);
				cellPos = getCellCoords(pos);
				getCellName(cellPos.x,cellPos.y,cellName);
				Con::printf("TerrainPager::saveStaticShapes found a static. %s %f %f %f id %d",object->mShapeName,pos.x,pos.y,pos.z,object->mOseId); 

				//FIRST: find out if this static is already in the database! I guess this means adding oseId to TSStatic.
				//So, now we have an mOseId in TSStatic, and we can set that here and check if it has a value.
				if (object->mOseId > 0)
				{
					if (object->mIsDirty)
					{
						U32 updateStart = clock();
						//SO, now that we have added mIsDirty to TSStatic, and set it in TSStatic::inspectPostApply, we know that
						//we need to update the object in the db. As it turns out, nothing should need to be added, you only need
						//to update existing pos & scale vectors and rot transform, and the osmNode. Except for the (1,1,1) scale.

						//So first, let's get us a scale_id, and see if it equals one or not.
						
						scale_id_query << "SELECT sc.id FROM vector3 sc JOIN shape sh ON sh.scale_id=sc.id WHERE sh.id=" << 
							object->mOseId << ";";
						result = mSQL->ExecuteSQL(scale_id_query.str().c_str());
						scale_id_query.clear(); scale_id_query.str("");
						if (result)
						{
							resultSet = mSQL->GetResultSet(result);
							if (resultSet->iNumRows == 1) 
							{
								scaleId = dAtoi(resultSet->vRows[0]->vColumnValues[0]);
								Con::printf("static found scale id: %d",scaleId);
							}
						}

						update_query << "UPDATE vector3 SET x=" << pos.x << ",y=" << pos.y << ",z=" << pos.z << 
							" WHERE id=(SELECT pos_id FROM shape WHERE id=" << object->mOseId << ");";
						result = mSQL->ExecuteSQL(update_query.str().c_str());
						update_query.clear(); update_query.str("");

						QuatF rot(object->getTransform());
						update_query << "UPDATE rotation SET x=" << rot.x << ",y=" << rot.y << ",z=" << rot.z << ",w=" << rot.w << 
							" WHERE id=(SELECT rot_id FROM shape WHERE id=" << object->mOseId << ");";
						result = mSQL->ExecuteSQL(update_query.str().c_str());
						update_query.clear(); update_query.str("");

						Point3F scale = object->getScale();
						//Now, make sure we don't change it if it is still (1,1,1), but also that we aren't changing it back to (1,1,1) from something else, which would be just as much time as doing this.
						Point3F diff = scale - Point3F(1,1,1);
						if ((scaleId!=1)||(diff.len()>0.0001))
						{
							if (scaleId==1)
							{//If we used to be (1,1,1) but are changing it, we need an insert.
								insert_query << "INSERT INTO vector3 (x,y,z) VALUES (" << scale.x << "," << scale.y << "," << scale.z << ");";
								result = mSQL->ExecuteSQL(insert_query.str().c_str());
								insert_query.clear(); insert_query.str("");
								if (result)
								{
									scaleId = mSQL->getLastRowId();

									update_query << "UPDATE shape SET scale_id=" << scaleId << " WHERE id=" << object->mOseId << ";";
									result = mSQL->ExecuteSQL(update_query.str().c_str());
									update_query.clear(); update_query.str("");
								}
							} else {
								update_query << "UPDATE vector3 SET x=" << scale.x << ",y=" << scale.y << ",z=" << scale.z << 
									" WHERE id=" << scaleId << ";";
								result = mSQL->ExecuteSQL(update_query.str().c_str());
								update_query.clear(); update_query.str("");
							}
						}
						//Now, osmNode, gets a little more complicated:
						update_query << "UPDATE osmNode SET latitude=" << latLongPos.y << ",longitude=" << latLongPos.x <<
							",name='" << object->getName() << "',cell_name='" << cellName << "' " <<
							"WHERE id=(SELECT node_id FROM shape WHERE id=" << object->mOseId << ");";
						result = mSQL->ExecuteSQL(update_query.str().c_str());
						update_query.clear(); update_query.str("");

						Con::printf("Updated a static shape in %d milliseconds!!!!!!!!!!!!!!!!!!!!!!",clock()-updateStart);
					}
					continue;
				}

				//Else, oseId==0, we're inserting a new object. First check to see if we have the shapefile already.
				fileId = 0;
				file_id_query << "SELECT id FROM shapeFile WHERE path IN ('" << object->mShapeName << "');";
				Con::printf("query: %s",file_id_query.str().c_str());
				result = mSQL->ExecuteSQL(file_id_query.str().c_str());
				file_id_query.clear(); file_id_query.str("");
				if (result)
				{
					resultSet = mSQL->GetResultSet(result);
					if (resultSet->iNumRows == 1) 
						fileId = dAtoi(resultSet->vRows[0]->vColumnValues[0]);
					else if (resultSet->iNumRows > 1) 
						Con::printf("shape has been entered in the database more than once! %s",object->mShapeName);
				}
				if (fileId == 0)
				{
					insert_query << "INSERT INTO shapeFile ( path ) VALUES ('" << object->mShapeName << "');";
					result = mSQL->ExecuteSQL(insert_query.str().c_str());
					fileId = mSQL->getLastRowId();
					insert_query.clear(); insert_query.str("");
				}
				Con::printf("file id: %d",fileId);
				if (fileId==0)
					continue;//something broke, forget this shape, move on.

				if (object->getName())
					insert_query << "INSERT INTO osmNode (longitude,latitude,type,name,cell_name) " <<
						"VALUES (" << latLongPos.x << "," << latLongPos.y << ",'TSStatic','" << object->getName() << "','" << 
						cellName << "')";
				else 	insert_query << "INSERT INTO osmNode (longitude,latitude,type,cell_name) " <<
						"VALUES (" << latLongPos.x << "," << latLongPos.y << ",'TSStatic','" << cellName << "')";

				result = mSQL->ExecuteSQL(insert_query.str().c_str());
				if (result)
					nodeId = mSQL->getLastRowId();
				insert_query.clear();
				insert_query.str("");

				insert_query << "INSERT INTO vector3 (x,y,z) VALUES (" << pos.x << "," << pos.y << "," << pos.z << ");";
				result = mSQL->ExecuteSQL(insert_query.str().c_str());
				if (result)
					posId = mSQL->getLastRowId();
				insert_query.clear();
				insert_query.str("");

				QuatF rot(object->getTransform());
				insert_query << "INSERT INTO rotation (x,y,z,w) VALUES (" << rot.x << "," << rot.y << "," << rot.z <<  "," << rot.w << ");";
				result = mSQL->ExecuteSQL(insert_query.str().c_str());
				if (result)
					rotId = mSQL->getLastRowId();
				insert_query.clear();
				insert_query.str("");

				Point3F scale = object->getScale();
				if ((scale - Point3F(1,1,1)).len() < 0.00001)
				{//SPECIAL: I may be a very bad person for doing this, but: if 99% of the time, scale is just going to be (1,1,1)
					scaleId = 1;//then I can remove a _lot_ of clutter by making an arbitrary rule - the first vector3 in the table
									//is always (1,1,1), and we don't ever change it.
				} else {
					insert_query << "INSERT INTO vector3 (x,y,z) VALUES (" << scale.x << "," << scale.y << "," << scale.z << ");";
					result = mSQL->ExecuteSQL(insert_query.str().c_str());
					insert_query.clear(); insert_query.str("");
					if (result)
						scaleId = mSQL->getLastRowId();
				}

				Con::printf("Trying to insert tsstatic, node %d, file %d, pos %d",nodeId,fileId,posId);

				if ((nodeId>0)&&(fileId>0)&&(posId>0)&&(rotId>0)&&(scaleId>0))
				{
					insert_query << "INSERT INTO shape (node_id, file_id, pos_id, rot_id, scale_id ) VALUES (" <<
						nodeId << "," << fileId << "," << posId << "," << rotId << "," << scaleId << ");";
					result = mSQL->ExecuteSQL(insert_query.str().c_str());
					shapeId = mSQL->getLastRowId();
					insert_query.clear();
					insert_query.str("");
					if (shapeId>0)
						object->mOseId = shapeId;
				}
			}
		}
	}

	unsigned int latency = clock() - startTime;
	Con::printf("saveStaticShapes() took %d milliseconds",latency);
}

DefineConsoleMethod( TerrainPager, saveStaticShapes, void, (), , "" )
{
	object->saveStaticShapes();
	return;
}

void TerrainPager::findStreetNodes()
{
	//Okay, this is really lazy but I'm going to use the same circling out from the middle strategy here
	//as the forest uses, although both cases could probably just be a simple loop by row and column.
	Point2F baseCellCoords = getCellCoords(mClientPos);
	Point3F baseCell = convertLatLongToXYZ(Point3F(baseCellCoords.x,baseCellCoords.y,0.0));
	Point3F startCell = baseCell;//This will be moving with every loop. 
	S32 loops = 0;
	char cellName[20];
	Vector<std::string> activeCells;

	unsigned long startTime =  clock();

	getCellName(baseCellCoords.x,baseCellCoords.y,cellName);
	activeCells.push_back(cellName);//NOW, we are just going to make a list of cellnames, and submit them all in one query.
	loops++;	

	//NOW, to loop around in ever expanding squares until we are entirely clear of streetRadius.
	F32 x,y;
	Point3F iterCell;

	Con::printf("findStreetNodes, roads %d",mRoads.size());
	//Note: this is *not* the same as taking a proper vector length from baseCell to startCell - by design, because we need the whole
	//column and row to be outside of forest radius 
	while ( ((baseCell.x-startCell.x)<=mStreetRadius) || ((baseCell.y-startCell.y)<=mStreetRadius) )
	{
		startCell.x -= mCellWidth;
		startCell.y -= mCellWidth;
		iterCell = startCell;

		Point3F cellPosLatLong;
		F32 closestDist;

		for (U32 i=0;i<(loops*2)+1;i++) // Left side, bottom to top.
		{
			if (i>0) iterCell.y += mCellWidth;
			cellPosLatLong = convertXYZToLatLong(iterCell);
			getCellName(cellPosLatLong.x,cellPosLatLong.y,cellName);
			closestDist = getForestCellClosestDist(Point2F(cellPosLatLong.x,cellPosLatLong.y),mClientPos);
			if  (closestDist<mStreetRadius)
			{
				activeCells.push_back(cellName);
			}
		}
		for (U32 i=1;i<(loops*2)+1;i++) // Top, left to right.
		{
			if (i>0) iterCell.x += mCellWidth;
			cellPosLatLong = convertXYZToLatLong(iterCell);
			getCellName(cellPosLatLong.x,cellPosLatLong.y,cellName);
			closestDist = getForestCellClosestDist(Point2F(cellPosLatLong.x,cellPosLatLong.y),mClientPos);
			if  (closestDist<mStreetRadius)
			{				
				activeCells.push_back(cellName);
			}
		}
		
		for (U32 i=1;i<(loops*2)+1;i++) // Right, top to bottom.
		{
			if (i>0) iterCell.y -= mCellWidth;
			cellPosLatLong = convertXYZToLatLong(iterCell);
			getCellName(cellPosLatLong.x,cellPosLatLong.y,cellName);
			closestDist = getForestCellClosestDist(Point2F(cellPosLatLong.x,cellPosLatLong.y),mClientPos);
			if  (closestDist<mStreetRadius)
			{
				activeCells.push_back(cellName);
			}
		}
		
		for (U32 i=1;i<(loops*2);i++) // Bottom, right to left.
		{
			if (i>0) iterCell.x -= mCellWidth;
			cellPosLatLong = convertXYZToLatLong(iterCell);
			getCellName(cellPosLatLong.x,cellPosLatLong.y,cellName);
			closestDist = getForestCellClosestDist(Point2F(cellPosLatLong.x,cellPosLatLong.y),mClientPos);
			if  (closestDist<mStreetRadius)
			{
				activeCells.push_back(cellName);
			}
		}
		loops++;
	}	
	
	//NOW, we have a list of cellnames, let's use them:
	// SELECT DISTINCT w.osm_id, w.name, w.type FROM osmWay w JOIN osmNode n ...JOIN osmWayNode wn ... WHERE n.cell_name IN {=' << activeCell[i]
	
	if (mSQL)//->OpenDatabase(Con::getVariable("$pref::OpenSimEarth::OSMDB")))
	{
		long wayId;
		int result,total_query_len=0;
		double nodeLong,nodeLat;
		std::string wayName,wayType,wayOsmId;
		std::ostringstream selectQuery,wayQuery;
		sqlite_resultset *resultSet;

		selectQuery << 
			"SELECT DISTINCT w.id, w.osm_id, w.name, w.type " << 
			"FROM osmWay w " << 
			"JOIN osmWayNode wn ON wn.way_id = w.osm_id " <<
			"JOIN osmNode n ON n.osm_id = wn.node_id " <<
			"WHERE n.cell_name IN  ( ";
		for (U32 i=0;i<activeCells.size();i++)
		{
			if (i<(activeCells.size()-1))
				selectQuery << "'" << activeCells[i].c_str() << "', ";
			else
				selectQuery << "'" << activeCells[i].c_str() << "' ";
		}
		selectQuery << " );";

		U32 queryTime = clock();
		//Con::printf("query: %s",selectQuery.str().c_str());
		result = mSQL->ExecuteSQL(selectQuery.str().c_str());
		if (result)
		{
			resultSet = mSQL->GetResultSet(result);
			selectQuery.clear();
			selectQuery.str("");
			if (resultSet->iNumRows > 0)
			{
				Con::printf("findStreetNodes found %d osmWays, query took %d milliseconds",resultSet->iNumRows,clock()-queryTime);
				for (U32 i=0;i<resultSet->iNumRows;i++)
				{
					int c=0;
					wayId = dAtol(resultSet->vRows[i]->vColumnValues[c++]);
					wayOsmId = resultSet->vRows[i]->vColumnValues[c++];
					wayName = resultSet->vRows[i]->vColumnValues[c++];
					wayType = resultSet->vRows[i]->vColumnValues[c++];

					osmWay kWay;
					kWay.name = wayName;
					kWay.type = wayType;

					if (mRoads.find(wayId) == mRoads.end())
					{
						mRoads[wayId].name = wayName;
						mRoads[wayId].type = wayType;
						sprintf(mRoads[wayId].osmId,"%s",wayOsmId.c_str());
						//mRoads[wayId].nodes.push_back(kNode);
						mRoads[wayId].road = NULL;
						Con::printf("pushing back a new road: id %d name %s  type %s  osm_id %s",
											wayId,wayName.c_str(),wayType.c_str(),wayOsmId.c_str());
					}// else {//Okay, this may or may not be useful in the long run. For now we're going to load the whole way
					//mRoads[wayId].nodes.push_back(kNode);//over in makeStreets, so these nodes are irrelevant. Someday, maybe.
					//}			

				}
			}
		}
	}	
	
	unsigned int latency = clock() - startTime;
	Con::printf("findStreetNodes() took %d milliseconds",latency);

}
	



		//selectQuery << 
		//	"SELECT n.osm_id, n.longitude, n.latitude, w.osm_id, w.type, w.name, wn.id " << 
		//	"FROM osmNode n " << 
		//	"JOIN osmWayNode wn ON n.osm_id = wn.node_id " <<
		//	"JOIN osmWay w ON wn.way_id = w.osm_id " <<
		//	"WHERE cell_name LIKE '" << cellName << "';";


void TerrainPager::fillForest()
{
	Point2F baseCellCoords = getCellCoords(mClientPos);
	Point3F baseCell = convertLatLongToXYZ(Point3F(baseCellCoords.x,baseCellCoords.y,0.0));
	Point3F startCell = baseCell;//This will be moving with every loop. 
	S32 loops = 0;
	char cellName[20];
	int treeType = 2;
	
	unsigned int startTime = clock();

	getCellName(baseCellCoords.x,baseCellCoords.y,cellName);

	//if ( (!mCellGrid[cellName]) || (mCellGrid[cellName]==0.0) )// && 
		//((mCellArea - mCellGrid[cellName]) > mMinCellArea) )
	if (mCellGrid.find(cellName) == mCellGrid.end())
	{
		fillForestCell(baseCellCoords);
		loops++;
	}

	//NOW, to loop around in ever expanding squares until we are entirely clear of forestRadius.
	//(loops*2)+1 is the number of squares on a side: 3, 5, 7, ... after the first one we just did.
	//Complicating factor is we are just sticking to the edge, not doing the whole block, so we need
	//four loops, start with lower left and do (loops*2)+1 blocks. Then turn right and do (loops*2),
	//turn down and do (loops*2), and then finally turn right again and do (loops*2)-1 to join back 
	//up with the starting cell.

	F32 x,y;
	Point3F iterCell;

	//Note: this is *not* the same as taking a proper vector length from baseCell to startCell - by design, because we need the whole
	//column and row to be outside of forest radius 
	while ( ((baseCell.x-startCell.x)<=mForestRadius) || ((baseCell.y-startCell.y)<=mForestRadius) )
	{
		startCell.x -= mCellWidth;
		startCell.y -= mCellWidth;
		iterCell = startCell;

		Point3F cellPosLatLong;
		F32 closestDist;
		//Now, time to do four loops, going clockwise, bottom left to top left, top left to top right, etc.
		for (U32 i=0;i<(loops*2)+1;i++) // Left side, bottom to top.
		{
			if (i>0) iterCell.y += mCellWidth;
			cellPosLatLong = convertXYZToLatLong(iterCell);
			getCellName(cellPosLatLong.x,cellPosLatLong.y,cellName);
			closestDist = getForestCellClosestDist(Point2F(cellPosLatLong.x,cellPosLatLong.y),mClientPos);
			if ( (closestDist<mForestRadius) && (!(mCellGrid[cellName]) || (mCellGrid[cellName] == 0.0)))
				//((mCellArea - mCellGrid[cellName]) > mMinCellArea) )
			{
				fillForestCell(Point2F(cellPosLatLong.x,cellPosLatLong.y));
			}
		}
		for (U32 i=1;i<(loops*2)+1;i++) // Top, left to right.
		{
			if (i>0) iterCell.x += mCellWidth;
			cellPosLatLong = convertXYZToLatLong(iterCell);
			getCellName(cellPosLatLong.x,cellPosLatLong.y,cellName);
			closestDist = getForestCellClosestDist(Point2F(cellPosLatLong.x,cellPosLatLong.y),mClientPos);
			if ( (closestDist<mForestRadius) && (!(mCellGrid[cellName]) || (mCellGrid[cellName] == 0.0)))
				//((mCellArea - mCellGrid[cellName]) > mMinCellArea) )
			{
				fillForestCell(Point2F(cellPosLatLong.x,cellPosLatLong.y));
			}
		}		
		for (U32 i=1;i<(loops*2)+1;i++) // Right, top to bottom.
		{
			if (i>0) iterCell.y -= mCellWidth;
			cellPosLatLong = convertXYZToLatLong(iterCell);
			getCellName(cellPosLatLong.x,cellPosLatLong.y,cellName);
			closestDist = getForestCellClosestDist(Point2F(cellPosLatLong.x,cellPosLatLong.y),mClientPos);
			if ( (closestDist<mForestRadius) && (!(mCellGrid[cellName]) || (mCellGrid[cellName] == 0.0)))
				//((mCellArea - mCellGrid[cellName]) > mMinCellArea) )
			{
				fillForestCell(Point2F(cellPosLatLong.x,cellPosLatLong.y));
			}
		}		
		for (U32 i=1;i<(loops*2);i++) // Bottom, right to left.
		{
			if (i>0) iterCell.x -= mCellWidth;
			cellPosLatLong = convertXYZToLatLong(iterCell);
			getCellName(cellPosLatLong.x,cellPosLatLong.y,cellName);
			closestDist = getForestCellClosestDist(Point2F(cellPosLatLong.x,cellPosLatLong.y),mClientPos);
			if ( (closestDist<mForestRadius) && (!(mCellGrid[cellName]) || (mCellGrid[cellName] == 0.0)))
				//((mCellArea - mCellGrid[cellName]) > mMinCellArea) )
			{
				fillForestCell(Point2F(cellPosLatLong.x,cellPosLatLong.y));
			}
		}
		loops++;
	}

	//Now, updateCollision on the forest, but keeping in mind that this is not free, let's not do it
	F32 groundZ;//if we're in free camera mode flying around above the forest. (Really, any time we're in
	Point3F normalVec;//free cam mode, but this could be useful with flying vehicles as well.)
	getGroundAt(mClientPos,&groundZ,&normalVec);
	if (mClientPos.z-groundZ<8)
		mForest->updateCollision();

	//big logic and queries ...
	unsigned int latency = clock() - startTime;
	Con::printf("fillForest() took %d milliseconds",latency);
}

void TerrainPager::fillForestCell(Point2F cellPosLatLong)
{
	float x,y,z,startX,startY;
	float radiusMultiple = mTreeRadiusMult;
	Point3F cellPosLatLong3D = Point3F(cellPosLatLong.x,cellPosLatLong.y,0.0);//(I don't actually care about elevation right now)
	Point3F cellPosXYZ = convertLatLongToXYZ(cellPosLatLong3D);
	
	F32 worldCoordZ;
	Point3F normalVector;
	char cellName[20];
	getCellName(cellPosLatLong.x,cellPosLatLong.y,cellName);
	
	ForestItemData *data = mForestItemData[0];
	F32 scale = 1.0;	
	
	Point3F hitNormal;
	F32 hitHeight;
	StringTableEntry matName;

	S32 lastTree = 0;
	S32 cellTries = mForestTries;
	for (U32 i=0;i<cellTries;i++)
	{
		x = cellPosXYZ.x  + mRandom.randF(0,mCellWidth);
		y = cellPosXYZ.y + mRandom.randF(0,mCellWidth);

		Point3F randPos = Point3F(x,y,0.0);

		Point2F terrainCellCoords = findTileCoords(randPos);
		Point3F randPosCoords = convertXYZToLatLong(randPos);
		F32 xDiff = fabs(randPosCoords.x-terrainCellCoords.x);
		F32 yDiff = fabs(randPosCoords.y-terrainCellCoords.y);
		if ( (xDiff<mForestTerrainEdgeDistance) || ((mD.mTileWidthLongitude-xDiff)<mForestTerrainEdgeDistance) ||
					(yDiff<mForestTerrainEdgeDistance) || ((mD.mTileWidthLatitude-yDiff)<mForestTerrainEdgeDistance) )
		{
			Con::printf("found a position at an edge! height = %f  xDiff %f  yDiff %f tileWidthLong %f",worldCoordZ,xDiff,yDiff,mD.mTileWidthLongitude);
			continue;//for now, just skip this position
		}

		if (!getGroundAt(randPos,&worldCoordZ,&normalVector))
			continue;
			
		//HERE: what's the best way to organize this? I want a group of trees per texture/landcover type, with each
		//tree weighted for probability versus the others in the set.
		//Vector of vectors? A vector of landcover types, each with a distribution vector containing all possible item datas?
		F32 weightSum = 0.0;
		F32 weights[10];
		S32 treeIndex;
		S32 numWeights = 0;
		S32 dataIndex;
		S32 texIndex = getClosestTextureIndex(randPos);
		F32 randWeight = 0.0;
		switch (texIndex)
		{//FIX!!! This all needs to use structures set up from the script side somehow.
			case 0:
				weights[numWeights++] = 20.0;
				weights[numWeights++] = 1.0;
				weights[numWeights++] = 1.0;

				for (U32 j=0;j<numWeights;j++) 
					weightSum += weights[j];
				randWeight = mRandom.randF(0.0,weightSum);
				if (randWeight < weights[0])
					dataIndex = 0;
				else if (randWeight < (weights[0] + weights[1]))
					dataIndex = 5;
				else if (randWeight < (weights[0] + weights[1] + weights[2]))
					dataIndex = 7;

				data = mForestItemData[dataIndex]; 
				break;
			case 1: 
				weights[numWeights++] = 2.0;
				weights[numWeights++] = 1.0;
				weights[numWeights++] = 0.5;

				for (U32 j=0;j<numWeights;j++) 
					weightSum += weights[j];
				randWeight = mRandom.randF(0.0,weightSum);
				if (randWeight < weights[0])
					dataIndex = 2;
				else if (randWeight < (weights[0] + weights[1]))
					dataIndex = 3;
				else if (randWeight < (weights[0] + weights[1] + weights[2]))
					dataIndex = 1;

				data = mForestItemData[dataIndex]; 
				break;
			case 2: 
				data = mForestItemData[5]; 
				break;
			case 3: 
				data = mForestItemData[0]; 
				break;
			case 4: 
				data = mForestItemData[0]; 
				break;
			case 5: 
				data = mForestItemData[0]; 
				break;
			case 6: 
				data = mForestItemData[1]; 
				break;
			case 7: 
				data = mForestItemData[0]; 
				break;
			case 8: 
				data = mForestItemData[0]; 
				break;
		}
		
		//current list:
		// 0  ExampleForestMesh  
		// 1  SequiaForestMesh  
		// 2  ScrubPineForestMesh  
		// 3  ShortPineForestMesh  
		// 4  Sequia02ForestMesh   
		// 5  Acacia01ForestMesh  
		// 6  Acacia02ForestMesh   
		// 7  AfricanMahoganyForestMesh 
		
		randPos.z = worldCoordZ;

		if ( mForest->getData()->getItems( randPos, data->mRadius * radiusMultiple, NULL ) > 0 )//radius / 2.0 because we want overlap.
            continue;
		
		float kScale = 1.0;
		if (scale > 0.0)
			kScale = scale;

		if (data)
		{
			mForest->addItem(data,randPos,mRandom.randF(0.0,360.0),mRandom.randF(kScale*0.7,kScale*1.3));
			
			float radius = (data->mRadius * radiusMultiple ) * kScale;
			if (mCellGrid[cellName]>0.0)
				mCellGrid[cellName] += M_PI * pow(radius,2);
			else
				mCellGrid[cellName] = M_PI * pow(radius,2);

			lastTree = i;
		} 		
	}
}

void TerrainPager::clearForestCell(Point2F cellPosLatLong)
{
	Point3F cellPos = convertLatLongToXYZ(Point3F(cellPosLatLong.x,cellPosLatLong.y,0.0));
	Vector<ForestItem> items;
	mForest->getData()->getItems(&items);
	S32 cellGridSize = mCellGrid.size();
	char cellName[20];
	getCellName(cellPosLatLong.x,cellPosLatLong.y,cellName);
	F32 edgeTrim = mCellWidth * 0.2;//Extra amount to go over the border on all sides, for some reason we're getting an outside
	for (U32 i=0;i<items.size();i++)//fringe of trees hanging on after delete.
	{
		if ((items[i].getPosition().x>=cellPos.x-edgeTrim)&&(items[i].getPosition().y>=cellPos.y-edgeTrim)&&
			 (items[i].getPosition().x<=cellPos.x+mCellWidth+edgeTrim)&&(items[i].getPosition().y<=cellPos.y+mCellWidth+edgeTrim))
		{					
			ForestItemKey key = items[i].getKey();
			mForest->removeItem(key,items[i].getPosition());
		}
	}

	if (mCellGrid.find(cellName) != mCellGrid.end())
	{
		mCellGrid.erase(cellName);
	}
}

F32 TerrainPager::getForestCellClosestDist(Point2F cellPosLatLong,Point3F pos)
{
	Point3F corners[4];
	S32 closest = -1;
	F32 closestDist = FLT_MAX;

	corners[0] = convertLatLongToXYZ(Point3F(cellPosLatLong.x,cellPosLatLong.y,0.0));//SW, bottom left
	corners[1] = corners[0] + Point3F(0.0,mCellWidth,0.0);//NW, upper left
	corners[2] = corners[1] + Point3F(mCellWidth,0.0,0.0);//NE, upper right
	corners[3] = corners[2] + Point3F(0.0,-mCellWidth,0.0);//SE, lower right

	for (U32 i=0;i<4;i++)
	{
		Point3F diff = Point3F(pos.x,pos.y,0.0) - corners[i];
		if (diff.len()<closestDist)
		{
			closest = i;
			closestDist = diff.len();
		}
	}

	return closestDist;
}

F32 TerrainPager::getForestCellFarthestDist(Point2F cellPosLatLong,Point3F pos)
{
	Point3F corners[4];
	S32 farthest = -1;
	F32 farthestDist = 0.0;

	corners[0] = convertLatLongToXYZ(Point3F(cellPosLatLong.x,cellPosLatLong.y,0.0));//SW, bottom left
	corners[1] = corners[0] + Point3F(0.0,mCellWidth,0.0);//NW, upper left
	corners[2] = corners[1] + Point3F(mCellWidth,0.0,0.0);//NE, upper right
	corners[3] = corners[2] + Point3F(0.0,-mCellWidth,0.0);//SE, lower right

	for (U32 i=0;i<4;i++)
	{
		Point3F diff = pos - corners[i];
		if (diff.len()>farthestDist)
		{
			farthest = i;
			farthestDist = diff.len();
		}
	}

	return farthestDist;
}

bool TerrainPager::getGroundAt( const Point3F &worldPt, F32 *zValueOut, VectorF *normalOut )
{
   const U32 mask = TerrainObjectType | StaticObjectType;

   Point3F start( worldPt.x, worldPt.y, worldPt.z + 100000 );
   Point3F end( worldPt.x, worldPt.y, worldPt.z - 100000 );
	U32 nodeIndex=0;

   if ( mForest )
      mForest->disableCollision();

   RayInfo rinfo;   
   bool hit = gServerContainer.castRay( start, end, mask, &rinfo );

   if ( mForest )
      mForest->enableCollision();

	if ( !hit || strcmp(rinfo.object->getClassName(),"TerrainBlock"))
      return false;
   
	Con::printf("fill forest cell, active roads: %d",mActiveRoads.size());
	if (mActiveRoads.size()>0)
	{
		bool roadCollision = false;
		for (std::map<long,osmWay>::iterator it=mActiveRoads.begin(); it!=mActiveRoads.end(); ++it)
		{
			long wayId = it->first;
			osmWay way = it->second;
			if (way.road)
			{
				if (way.road->containsPoint(worldPt,&nodeIndex))
					return false;
			}
		}
	}
   if (zValueOut)
      *zValueOut = rinfo.point.z;

   if (normalOut)
      *normalOut = rinfo.normal;
   return true;
}

bool TerrainPager::getGroundAtInclusive( const Point3F &worldPt, F32 *zValueOut, VectorF *normalOut )
{
   const U32 mask = TerrainObjectType ;

   Point3F start( worldPt.x, worldPt.y, worldPt.z + 100000 );
   Point3F end( worldPt.x, worldPt.y, worldPt.z - 100000 );

   if ( mForest )
      mForest->disableCollision();

   RayInfo rinfo;   
   bool hit = gServerContainer.castRay( start, end, mask, &rinfo );

   if ( mForest )
      mForest->enableCollision();

   if ( !hit )
      return false;

   if (zValueOut)
      *zValueOut = rinfo.point.z;

   if (normalOut)
      *normalOut = rinfo.normal;

   return true;
}

void TerrainPager::interpolateTick(F32 f)
{	//Need this because abstract in parent class.
}

void TerrainPager::advanceTime(F32 f)
{	//Need this because abstract in parent class.
}

bool TerrainPager::checkFileExists(const char *filename)//Actually, this should be up in utils or something.
{//Has the one (potential) drawback of failing to recognize a zero-length file, must contain data.
	FileStream fs;
	if (fs.open(filename,Torque::FS::File::Read))
	{ 
		fs.close();
		return true;
	} else return false;
}

TerrainBlock *TerrainPager::addTerrainBlock(F32 startLong,F32 startLat)
{
	char terrainName[50],heightfilename[256],texturefilename[256],tileName[20];
	TerrainBlock *block;
	bool terrExists = false;

	//HERE: height files are 257K, texture files are 64K. It would take approximately 15 ticks, or half a second, to pass
	//all one tile's worth of this data across the socket instead of writing it to disk. Eight tiles should take four seconds.
	getTileName(startLong,startLat,tileName);
	sprintf(heightfilename,"%shght.%s.bin",mD.mTerrainPath.c_str(),tileName);
	sprintf(texturefilename,"%stext.%s.bin",mD.mTerrainPath.c_str(),tileName);
	sprintf(terrainName,"terrain.%s.ter",tileName);
	FileName terrFileName = mD.mTerrainPath + terrainName;

	//Now, since we're checking previously in loadTerrainGrid or checkTerrainGrid, we are assuming 
	//these files exist and are not calling worldDataSource for them from here.
	if (checkFileExists(terrFileName.c_str())==false)
	{
		Con::printf("trying to make terrain file: %s heightmapres %d ",terrFileName.c_str(),mD.mHeightmapRes );	
		TerrainFile::createByName( &terrFileName, mD.mHeightmapRes, terrain_materials );
	} else 
		terrExists = true;

	Con::printf("adding terrain block! %s",terrFileName.c_str());

	block = new TerrainBlock();

	if( !block->setFile( terrFileName ) )
	{
		Con::errorf( "TerrainBlock::createNew - error creating '%s'", terrFileName.c_str() );
		return 0;
	}

	block->mSquareSize = mD.mSquareSize;
	block->mBaseTexSize = mD.mTextureRes;
	block->mLightMapSize = mD.mLightmapRes;
	block->mLongitude = startLong;
	block->mLatitude = startLat;
	Point3F blockPos = Point3F(((startLong-mD.mMapCenterLongitude)*mD.mMetersPerDegreeLongitude),
						(startLat-mD.mMapCenterLatitude)*mD.mMetersPerDegreeLatitude,0.0);//FIX! need maxHeight/minHeight
	block->setPosition(blockPos );

	mTerrains.increment();
	mTerrains.last() = block;
	
	block->registerObject( terrainName );
	block->addToScene();

	if (terrExists==false)
	{
		if (block->loadTerrainData(heightfilename,texturefilename,mD.mTextureRes,terrain_materials.size(),"treefile.txt"))
			Con::printf("block loaded terrain data: %s",heightfilename);
		else {
			Con::printf("block failed to load terrain data: %s",heightfilename);
		}
	} else {
		Con::printf("block reloaded existing terrain file: %s",terrFileName.c_str());		
	}

   //SimGroup* pMissionGroup = dynamic_cast<SimGroup*>(Sim::findObject("MissionGroup"));
	//pMissionGroup->addObject(block);
	//gServerContainer.addObject(block);//??

	return block;
}


S32 TerrainPager::getClosestTextureIndex(Point3F pos)
{
	S32 index = -1;

	TerrainBlock *block = getTerrainBlock(pos);
	if (block)
	{
		Point3F blockPos = block->getPosition();
		Point3F localPos = pos - blockPos;
		F32 scale = 1.0/block->getWorldBlockSize();
		localPos *= scale;//Scale our local pos down to a 0.0-1.0 range

		Point3F texturePos = localPos * (F32)block->getBlockSize();

		//Wait, why does the literature say I should have a round(float) function in C++, but Visual Studio says I don't?
		S32 x = ((texturePos.x-floor(texturePos.x)<0.5) ? floor(texturePos.x) : ceil(texturePos.x));
		S32 y = ((texturePos.y-floor(texturePos.y)<0.5) ? floor(texturePos.y) : ceil(texturePos.y));
		
		index = block->getFile()->getLayerIndex(x,y);
	}

	return index;
}

TerrainBlock *TerrainPager::getTerrainBlock(Point3F pos)
{
	
	Point2F tileCoords = findTileCoords(pos);

	for (int i=0;i<mTerrains.size();i++)
	{
			Point2F diff = Point2F(mTerrains[i]->mLongitude,mTerrains[i]->mLatitude) - tileCoords;
			if (diff.len()<0.0001)
				return mTerrains[i];
	}

	return NULL;
}

void TerrainPager::dropTerrainBlock(U32 index)
{//NOT WORKING, AT ALL.
	/*
	for (int y=0;y<mGridSize;y++) 
	{
		for (int x=0;x<mGridSize;x++) 
		{
			if (mTerrainGrid[y*mGridSize+x]==mTerrains[index])
				mTerrainGrid[y*mGridSize+x]=NULL;
		}
	}
	//Wait, try this: 
	//mTerrains[index]->unregisterObject();

	//These didn't work
	////mTerrains[index]->safeDeleteObject();
	////mTerrains.erase(index);
	*/
}

void TerrainPager::dropAllTerrains()
{//pretty much just for testing...
	//for (int c=mTerrains.size()-1;c>=0;c--)
	//{
		//mTerrains[c]->safeDeleteObject();//unregisterObject();?
	//}
	//mTerrains.clear();
}

void TerrainPager::dropTerrainBlock(F32 startLong,F32 startLat)
{//NOT WORKING, AT ALL.
	/*
	//Con::printf("trying to drop a terrain: %f %f, mTerrains %d",startLong,startLat,mTerrains.size());
	for (int y=0;y<mGridSize;y++) 
	{
		for (int x=0;x<mGridSize;x++) 
		{
			if (mTerrainGrid[y*mGridSize+x])
			{
				TerrainBlock *tmp = mTerrainGrid[y*mGridSize+x];
				if ((fabs(tmp->mLongitude-startLong)<0.0001f)&&
					(fabs(tmp->mLatitude-startLat)<0.0001f))
					mTerrainGrid[y*mGridSize+x]=NULL;
			}
		}
	}
	for (int c=0;c<mTerrains.size();c++)
	{
		if (mTerrains[c])
		{
			//Con::printf("terrains[%d] longitude=%f, latitude=%f",c,mTerrains[c]->mLongitude,mTerrains[c]->mLatitude);
			if ((fabs(mTerrains[c]->mLongitude-startLong)<0.0001f)&&
				(fabs(mTerrains[c]->mLatitude-startLat)<0.0001f))
			{
				Con::printf("DROPPING A TERRAIN! %f %f",mTerrains[c]->mLongitude,mTerrains[c]->mLatitude);
				//mTerrains[c]->safeDeleteObject();
				mTerrains.erase(c);
			}
		}
	}*/
}

void TerrainPager::getTileName(F32 tileStartPointLong,F32 tileStartPointLat,char *outStr)
{
	char temp[20];
	
	if ((tileStartPointLong<-180.0)||(tileStartPointLong>180.0)|| //First check  for reasonable
					(tileStartPointLat<-90.0)||(tileStartPointLat>90.0))  // coordinate bounds.
		return;

	double tileStartPointLongR,tileStartPointLatR;
	tileStartPointLongR = (float)((int)(tileStartPointLong * 1000.0))/1000.0;//(10 ^ decimalPlaces)
	tileStartPointLatR = (float)((int)(tileStartPointLat * 1000.0))/1000.0;//Maybe?
	char longC,latC;
	if (tileStartPointLong<0.0) longC = 'W';
	else longC = 'E';
	if (tileStartPointLat<0.0) latC = 'S';
	else latC = 'N';

	//NOW, just have to separate out the decimal part from the whole numbers part, and make sure to get
	//preceding zeroes for the decimal part, so it's always three characters.
	int majorLong,minorLong,majorLat,minorLat;//"major" for left of decimal, "minor" for right of decimal.
	std::string majorLongStr,minorLongStr,majorLatStr,minorLatStr;

	majorLong = (int)tileStartPointLongR;
	majorLat = (int)tileStartPointLatR;
	
	minorLong = abs( (int)(tileStartPointLongR * 1000.0) - (majorLong * 1000)  );
	minorLat = abs( (int)(tileStartPointLatR * 1000.0) - (majorLat * 1000)  );

	majorLong = abs(majorLong);
	majorLat = abs(majorLat);
	
	if (majorLong<10) sprintf(temp,"00%d",majorLong);
	else if (majorLong<100) sprintf(temp,"0%d",majorLong);
	else sprintf(temp,"%d",majorLong);
	majorLongStr = temp;

	if (majorLat<10)//Latitude ends at 90 so no worries about three digits.
		 sprintf(temp,"0%d",majorLat);
	else sprintf(temp,"%d",majorLat);
	majorLatStr = temp;

	if (minorLong<10) sprintf(temp,"00%d",minorLong);
	else if (minorLong<100) sprintf(temp,"0%d",minorLong);			
	else sprintf(temp,"%d",minorLong);
	minorLongStr = temp;

	if (minorLat<10) sprintf(temp,"00%d",minorLat);
	else if (minorLat<100) sprintf(temp,"0%d",minorLat);			
	else sprintf(temp,"%d",minorLat);
	minorLatStr = temp;

	sprintf(outStr,"%sd%s%c_%sd%s%c",majorLongStr.c_str(),minorLongStr.c_str(),longC,majorLatStr.c_str(),minorLatStr.c_str(),latC);
	
	return;
}

//FIX: no time for niceties at the moment, this is just an exact copy of getTileName except using four decimal
//places instead of three. 
void TerrainPager::getCellName(F32 tileStartPointLong,F32 tileStartPointLat,char *outStr)//,int decimalPlaces
{
	char temp[20];
	
	if ((tileStartPointLong<-180.0)||(tileStartPointLong>180.0)|| //First check  for reasonable
					(tileStartPointLat<-90.0)||(tileStartPointLat>90.0))  // coordinate bounds.
		return;

	double tileStartPointLongR,tileStartPointLatR;
	tileStartPointLongR = (float)((int)(tileStartPointLong * 10000.0))/10000.0;//(10 ^ decimalPlaces)
	tileStartPointLatR = (float)((int)(tileStartPointLat * 10000.0))/10000.0;
	char longC,latC;
	if (tileStartPointLong<0.0) longC = 'W';
	else longC = 'E';
	if (tileStartPointLat<0.0) latC = 'S';
	else latC = 'N';

	int majorLong,minorLong,majorLat,minorLat;//"major" for left of decimal, "minor" for right of decimal.
	std::string majorLongStr,minorLongStr,majorLatStr,minorLatStr;

	majorLong = (int)tileStartPointLongR;
	majorLat = (int)tileStartPointLatR;
	
	minorLong = abs( (int)(tileStartPointLongR * 10000.0) - (majorLong * 10000)  );
	minorLat = abs( (int)(tileStartPointLatR * 10000.0) - (majorLat * 10000)  );

	majorLong = abs(majorLong);
	majorLat = abs(majorLat);
	
	if (majorLong<10) sprintf(temp,"00%d",majorLong);
	else if (majorLong<100) sprintf(temp,"0%d",majorLong);
	else sprintf(temp,"%d",majorLong);
	majorLongStr = temp;

	if (majorLat<10)//Latitude ends at 90 so no worries about three digits.
		 sprintf(temp,"0%d",majorLat);
	else sprintf(temp,"%d",majorLat);
	majorLatStr = temp;

	if (minorLong<10) sprintf(temp,"000%d",minorLong);
	else if (minorLong<100) sprintf(temp,"00%d",minorLong);		
	else if (minorLong<1000) sprintf(temp,"0%d",minorLong);			
	else sprintf(temp,"%d",minorLong);
	minorLongStr = temp;

	if (minorLat<10) sprintf(temp,"000%d",minorLat);
	else if (minorLat<100) sprintf(temp,"00%d",minorLat);		
	else if (minorLat<1000) sprintf(temp,"0%d",minorLat);		
	else sprintf(temp,"%d",minorLat);
	minorLatStr = temp;

	sprintf(outStr,"%sd%s%c_%sd%s%c",majorLongStr.c_str(),minorLongStr.c_str(),longC,majorLatStr.c_str(),minorLatStr.c_str(),latC);
	
	return;
}

void TerrainPager::getCellName(Point3F position,char *outStr)
{
	float posLongitude = mD.mMapCenterLongitude + (position.x * mD.mDegreesPerMeterLongitude);
	float posLatitude = mD.mMapCenterLatitude + (position.y * mD.mDegreesPerMeterLatitude);

	Point2F coordPos = Point2F(posLongitude,posLatitude);
	Point2F mapCenter = Point2F(mD.mMapCenterLongitude,mD.mMapCenterLatitude);

	float cellWidthLongitude = mD.mTileWidthLongitude / mCellGridSize;//Might be worth storing these once
	float cellWidthLatitude = mD.mTileWidthLatitude / mCellGridSize;		//at load time...

	float cellStartLongitude = (mFloor((coordPos.x-mapCenter.x)/cellWidthLongitude)*cellWidthLongitude)+mapCenter.x;
	float cellStartLatitude = (mFloor((coordPos.y-mapCenter.y)/cellWidthLatitude)*cellWidthLatitude)+mapCenter.y;
	
	return getCellName(cellStartLongitude,cellStartLatitude,outStr);
}

Point2F TerrainPager::getCellCoords(Point3F position)
{
	float posLongitude = mD.mMapCenterLongitude + (position.x * mD.mDegreesPerMeterLongitude);
	float posLatitude = mD.mMapCenterLatitude + (position.y * mD.mDegreesPerMeterLatitude);

	Point2F coordPos = Point2F(posLongitude,posLatitude);
	Point2F mapCenter = Point2F(mD.mMapCenterLongitude,mD.mMapCenterLatitude);

	float cellWidthLongitude = mD.mTileWidthLongitude / mCellGridSize;//Might be worth storing these once
	float cellWidthLatitude = mD.mTileWidthLatitude / mCellGridSize;		//at load time...

	float cellStartLongitude = (mFloor((coordPos.x-mapCenter.x)/cellWidthLongitude)*cellWidthLongitude)+mapCenter.x;
	float cellStartLatitude = (mFloor((coordPos.y-mapCenter.y)/cellWidthLatitude)*cellWidthLatitude)+mapCenter.y;
	
	return (Point2F(cellStartLongitude,cellStartLatitude));
}

Point2F TerrainPager::getCellCoordsFromName(char *cellName)
{//WARNING: this loses accuracy! All coords rounded to the nearest four decimal places. And we have to add/subtract 
	//another small number (0.000005 seems to work) in order to get above a down-one rounding error.
	std::string name(cellName);
	char tempLongMajor[5],tempLongMinor[5],tempLatMajor[5],tempLatMinor[5],tempLong[10],tempLat[10];
	char eastWest[1],northSouth[1];
	double tileStartPointLong,tileStartPointLat;

	strcpy(tempLongMajor,name.substr(0,3).c_str());
	strcpy(tempLongMinor,name.substr(4,4).c_str());
	strcpy(eastWest,name.substr(8,1).c_str());
	strcpy(tempLatMajor,name.substr(10,2).c_str());
	strcpy(tempLatMinor,name.substr(13,4).c_str());
	strcpy(northSouth,name.substr(17,1).c_str());


	sprintf(tempLong,"%s.%s",tempLongMajor,tempLongMinor);
	sprintf(tempLat,"%s.%s",tempLatMajor,tempLatMinor);
	
	tileStartPointLong = atof(tempLong);
	tileStartPointLat = atof(tempLat);

	F32 longMod = 1.0;
	if (!strcmp(eastWest,"W"))
		longMod = -1.0;
	tileStartPointLong *= longMod;

	F32 latMod = 1.0;
	if (!strcmp(northSouth,"S"))
		latMod = -1.0;
	tileStartPointLat *= latMod;
	
	return Point2F(tileStartPointLong+(longMod*0.000005),tileStartPointLat+(latMod*0.000005));//I really hate these float rounding problems.
}

void TerrainPager::findClientPos()
{
	Vector<SceneObject*> kCameras;
	Vector<SceneObject*> kPlayers;

	Box3F bounds;
	bounds.set(Point3F(-FLT_MAX,-FLT_MAX,-FLT_MAX),Point3F(FLT_MAX,FLT_MAX,FLT_MAX));
	gServerContainer.findObjectList(bounds, CameraObjectType, &kCameras);
	gServerContainer.findObjectList(bounds, PlayerObjectType, &kPlayers);

	mClientPos.zero();
	for (U32 i=0;i<kPlayers.size();i++)
	{
		Player *myPlayer = (Player *)(kPlayers[i]);
		Point3F playerPos = myPlayer->getPosition();
		if (kCameras.size()>0)
		{
			Camera *myCamera = dynamic_cast<Camera *>(kCameras[i]);//... sort out which belongs to controlling client.
			Point3F cameraPos = myCamera->getPosition();
			GameConnection *cameraClient = myCamera->getControllingClient();
			GameConnection *playerClient = myPlayer->getControllingClient();
			if (cameraClient) 
			{
				mFreeCamera = true;
				mClientPos = cameraPos;
			} else if (playerClient) {
				mFreeCamera = false;
				mClientPos = playerPos;
			} 
		} else {
			mClientPos = playerPos;
		}
	} 

	mD.mClientPosLongitude = mD.mMapCenterLongitude + (mClientPos.x * mD.mDegreesPerMeterLongitude);
	mD.mClientPosLatitude = mD.mMapCenterLatitude + (mClientPos.y * mD.mDegreesPerMeterLatitude);
	mD.mClientPosAltitude = mClientPos.z;
}

void TerrainPager::findClientTile()
{
	findClientPos();
	
	Point2F clientPos = Point2F(mD.mClientPosLongitude,mD.mClientPosLatitude);
	//FIX: The following is off by one half tile width because of my (perhaps questionable) decision to put 
	//map center in the center of a tile rather than at the lower left corner of that tile. Subject to review.
	Point2F mapCenter = Point2F(mD.mMapCenterLongitude-(mD.mTileWidthLongitude/2.0f),
								mD.mMapCenterLatitude-(mD.mTileWidthLatitude/2.0f));

	mLastTileStartLong = mTileStartLongitude;
	mLastTileStartLat  = mTileStartLatitude;
	mTileStartLongitude = (mFloor((clientPos.x-mapCenter.x)/mD.mTileWidthLongitude)*mD.mTileWidthLongitude)+mapCenter.x;
	mTileStartLatitude = (mFloor((clientPos.y-mapCenter.y)/mD.mTileWidthLatitude)*mD.mTileWidthLatitude)+mapCenter.y;
}

Point2F TerrainPager::findTileCoords(Point3F pos)
{	
	Point3F latLongPos = convertXYZToLatLong(pos);
	//FIX: The following is off by one half tile width because of my (perhaps questionable) decision to put 
	//map center in the center of a tile rather than at the lower left corner of that tile. Subject to review.
	Point2F mapCenter = Point2F(mD.mMapCenterLongitude-(mD.mTileWidthLongitude/2.0f),
								mD.mMapCenterLatitude-(mD.mTileWidthLatitude/2.0f));

	F32 tileStartLongitude = (mFloor((latLongPos.x-mapCenter.x)/mD.mTileWidthLongitude)*mD.mTileWidthLongitude)+mapCenter.x;
	F32 tileStartLatitude = (mFloor((latLongPos.y-mapCenter.y)/mD.mTileWidthLatitude)*mD.mTileWidthLatitude)+mapCenter.y;

	return Point2F(tileStartLongitude,tileStartLatitude);
}

void TerrainPager::loadTileGrid()
{
	bool loaded;
	bool verbose = true;
	char tileName[20],heightfilename[256],texturefilename[256],terrainfilename[256];
	U32 gridMidpoint = (mGridSize-1)/2;
	Vector <loadTerrainData> loadTerrains;

	F32 startLong = mTileStartLongitude - (gridMidpoint * mD.mTileWidthLongitude);
	F32 startLat = mTileStartLatitude - (gridMidpoint * mD.mTileWidthLatitude);
	Con::printf("loading tile grid, client pos %f %f, client tile start %f %f, local grid start %f %f",
		mD.mClientPosLongitude,mD.mClientPosLatitude,mTileStartLongitude,mTileStartLatitude,startLong,startLat);

	//Wait, okay, *first* we need to clear the grid, *then* go ahead and fill it again.
	for (int y=0;y<mGridSize;y++) 
	{
		for (int x=0;x<mGridSize;x++) 
		{
			mTerrainGrid[y*mGridSize+x]=NULL;

			loadTerrains.increment();
			loadTerrainData *kData = &(loadTerrains.last());
			kData->startLongitude = 0.0;
			kData->startLatitude = 0.0;
			kData->tileDistance = FLT_MAX;
		}
	}
	if (verbose) 
		for (int c=0;c<mTerrains.size();c++)
			Con::printf("terrain %d longitude %f latitude %f",c,mTerrains[c]->mLongitude,mTerrains[c]->mLatitude);
	for (int y=0;y<mGridSize;y++) 
	{
		for (int x=0;x<mGridSize;x++) 
		{
			F32 kLong = startLong + (x * mD.mTileWidthLongitude);
			F32 kLat = startLat + (y * mD.mTileWidthLatitude);
			F32 midLong = kLong + (mD.mTileWidthLongitude / 2.0f);
			F32 midLat = kLat + (mD.mTileWidthLatitude / 2.0f);
			Point2F tileCenterDiff = Point2F((mD.mClientPosLongitude-midLong)*mD.mMetersPerDegreeLongitude,
				                          (mD.mClientPosLatitude-midLat)*mD.mMetersPerDegreeLatitude);
			F32 tileDistance = tileCenterDiff.len();

			getTileName(kLong,kLat,tileName);
			sprintf(heightfilename,"%shght.%s.bin",mD.mTerrainPath.c_str(),tileName);
			sprintf(texturefilename,"%stext.%s.bin",mD.mTerrainPath.c_str(),tileName);
			sprintf(terrainfilename,"%sterrain.%s.ter",mD.mTerrainPath.c_str(),tileName);

			if (mTerrainGrid[y*mGridSize+x]==NULL) loaded = false;
			else loaded = true;

			if (verbose) 
				Con::printf("terrain %d %d loaded = %d distance %f kLong %f kLat %f ",x,y,loaded,tileDistance,kLong,kLat);

			if ((tileDistance<mD.mTileLoadRadius)&&(loaded==false))
			{
				for (int c=0;c<mTerrains.size();c++)
				{//Could have based this off tilename comparison, but would rather do it with numbers.
					if ((fabs(mTerrains[c]->mLongitude-kLong)<0.0001)&&(fabs(mTerrains[c]->mLatitude-kLat)<0.0001))
					{//("<0.0001" because "==" doesn't work, floating point error is annoying.)
						loaded=true;
						mTerrainGrid[y*mGridSize+x]=mTerrains[c];
					}

				}
				if (loaded==false)
				{//Here, let's check for the bin file existing firstif (verbose) 
					Con::printf("checking bin file: %s  %s  %s",terrainfilename,heightfilename,texturefilename);
					if ( (checkFileExists(terrainfilename)) || 
						((checkFileExists(heightfilename))&&(checkFileExists(texturefilename))) )
						mTerrainGrid[y*mGridSize+x] = addTerrainBlock(kLong,kLat);
					else if (mUseDataSource)//okay, now we need to make a call to worldDataSource.
					{//HERE: I need to request ONE AT A TIME. And keep coming back here until they're all done.
						loadTerrainData *kData = &(loadTerrains[y*mGridSize+x]);
						kData->startLongitude = kLong;
						kData->startLatitude = kLat;
						kData->tileDistance = tileDistance;
						//mDataSource->addTerrainRequest(kLong,kLat);//have to wait, and only ask for one at a time now.
						mLoadState = 3;//waiting for terrain.
					}
				}
			} else if ((tileDistance>mD.mTileDropRadius)&&(loaded==true)) {
				dropTerrainBlock(kLong,kLat);
				if (verbose) Con::printf("drop this terrain block: %d %d",x,y);
			}
			
			if ((x==gridMidpoint)&&(y==gridMidpoint))
				mTerrain = mTerrainGrid[y*mGridSize+x];
		}
	}
	for (int c=0;c<mTerrains.size();c++)
	{
		if ((mTerrains[c]->mLongitude<startLong)||
			(mTerrains[c]->mLongitude>=(startLong+(mGridSize*mD.mTileWidthLongitude)))||
			(mTerrains[c]->mLatitude<startLat)||
			(mTerrains[c]->mLatitude>=(startLat+(mGridSize*mD.mTileWidthLatitude))))
		{
			dropTerrainBlock(c);
		}
	}
	if (mLoadState==2)//Meaning we didn't set it to 3, ie waiting for data.
	{
		Con::printf("TerrainPager done loading tiles, entering checkTile loop.");
		mLoadState=10;
	} 
	else if (mLoadState==3)//Finally, if we did set load state to three, that means we need more terrains from 
	{			//our dataSource. Which menas the current job is picking the closest one that we still need.
		float closestDist = FLT_MAX;
		int closestIndex = -1;
		loadTerrainData *kData;
		for (int i=0;i<loadTerrains.size();i++)
			for (int j=0;j<loadTerrains.size();j++)
			{
				kData = &(loadTerrains[j]);
				if (kData->tileDistance < closestDist)
				{
					closestDist = kData->tileDistance;
					closestIndex = j;
				}
			}
		if (closestIndex>=0)
		{
			kData = &(loadTerrains[closestIndex]);
			mDataSource->addTerrainRequest(kData->startLongitude,kData->startLatitude);
			mTerrainRequestTick = mCurrentTick;
		}
	}
	loadTerrains.clear();
}

void TerrainPager::checkTileGrid()
{
	bool loaded;
	bool verbose = false;
	char tileName[20],heightfilename[256],texturefilename[256];	
	U32 gridMidpoint = (mGridSize-1)/2;

	F32 startLong = mTileStartLongitude - (gridMidpoint * mD.mTileWidthLongitude);
	F32 startLat = mTileStartLatitude - (gridMidpoint * mD.mTileWidthLatitude);
	
	for (int y=0;y<mGridSize;y++) 
	{
		for (int x=0;x<mGridSize;x++) 
		{
			F32 kLong = startLong + (x * mD.mTileWidthLongitude);
			F32 kLat = startLat + (y * mD.mTileWidthLatitude);
			F32 midLong = kLong + (mD.mTileWidthLongitude / 2.0f);
			F32 midLat = kLat + (mD.mTileWidthLatitude / 2.0f);
			Point2F tileCenterDiff = Point2F((mD.mClientPosLongitude-midLong)*mD.mMetersPerDegreeLongitude,
				                          (mD.mClientPosLatitude-midLat)*mD.mMetersPerDegreeLatitude);
			F32 tileDistance = tileCenterDiff.len();
			getTileName(kLong,kLat,tileName);
			sprintf(heightfilename,"%shght.%s.bin",mD.mTerrainPath.c_str(),tileName);
			sprintf(texturefilename,"%stext.%s.bin",mD.mTerrainPath.c_str(),tileName);

			if (mTerrainGrid[y*mGridSize+x]==NULL) loaded = false;
			else loaded = true;

			if ((tileDistance<mD.mTileLoadRadius)&&(loaded==false))
			{
				if (verbose) Con::printf("tile %d %d should be loaded, but isn't. dist %f, %f %f",x,y,tileDistance,kLong,kLat);

				if ((checkFileExists(heightfilename))&&(checkFileExists(texturefilename)))
					mTerrainGrid[y*mGridSize+x] = addTerrainBlock(kLong,kLat);
				else if (mUseDataSource)
				{	
					mDataSource->addTerrainRequest(kLong,kLat);
					mLoadState = 3;//waiting for terrain.
				}
			} else if ((tileDistance>mD.mTileDropRadius)&&(loaded==true)) {
				dropTerrainBlock(kLong,kLat);
				if (verbose) Con::printf("tile %d %d should not be loaded, but is. dist %f, %f %f",x,y,tileDistance,kLong,kLat);
				//Con::printf("drop this terrain block!!! %d %d",x,y);
			} else if (tileDistance>mD.mTileLoadRadius) {
				if (verbose) Con::printf("tile %d %d should not be loaded, and isn't. dist %f, %f %f",x,y,tileDistance,kLong,kLat);
			} else if ((tileDistance<=mD.mTileLoadRadius)&&(loaded==true)) {
				if (verbose) Con::printf("tile %d %d should be loaded, and is. dist %f, %f %f",x,y,tileDistance,kLong,kLat);
			}
		}
	}

	if (verbose) Con::printf("///////////////////////////////////////////////////////////");
}

void TerrainPager::reloadSkyboxImages()
{
	Con::printf("Reloading skyboxes, skybox path: %s",mD.mSkyboxPath.c_str());

	//HERE:  The one big trick to making skyboxes work in Torque is flipping them all around so they match the cubemap 
	//configuration with the sky in the center and other four images aligned around it, so sky is always "touching".
	//        up
	//        |N
	//    W   v   E
	// up -> SKY <- up
	//        ^
	//        |S
	//        up

	FileName skyboxes[10];//first five for input, second five for output. skybox3_nn vs skybox_nn for first pass.
	//Order is:  up, 00, 90, 180, 270
	skyboxes[0] = mD.mSkyboxPath + String("skybox3_up.png");
	skyboxes[1] = mD.mSkyboxPath + String("skybox3_00.png");
	skyboxes[2] = mD.mSkyboxPath + String("skybox3_90.png");
	skyboxes[3] = mD.mSkyboxPath + String("skybox3_180.png");
	skyboxes[4] = mD.mSkyboxPath + String("skybox3_270.png");

	skyboxes[5] = mD.mSkyboxPath + String("skybox_up.png");
	skyboxes[6] = mD.mSkyboxPath + String("skybox_00.png");
	skyboxes[7] = mD.mSkyboxPath + String("skybox_90.png");
	skyboxes[8] = mD.mSkyboxPath + String("skybox_180.png");
	skyboxes[9] = mD.mSkyboxPath + String("skybox_270.png");
	FileStream fs;
	U32 skyOffset = 0;//20;//Amount we raise all side panels (to compensate for camera height?)
	GBitmap *bitmap = new GBitmap;
	GBitmap *bitmap2 = new GBitmap;
	//We're going to do very different things per bitmap, so not bothering to loop this.
	
	if (fs.open(skyboxes[0].c_str(),Torque::FS::File::Read))// UP
	{
		bitmap->readBitmap("png",fs);
		bitmap2->allocateBitmap(bitmap->getWidth(),bitmap->getHeight());
		fs.close();
		for (U32 x=0;x<bitmap->getWidth();x++) {
			for (U32 y=0;y<bitmap->getHeight();y++) {
				ColorI color;
				bitmap->getColor(x,y,color);
				//bitmap2->setColor(x,y,color);//Direct copy
				bitmap2->setColor(x,(bitmap->getHeight()-1)-y,color);//Flip vertically
			}
		}
		if (fs.open(skyboxes[5].c_str(),Torque::FS::File::Write))
		{
			bitmap2->writeBitmap("png",fs);
			fs.close();
		}
	} else Con::printf("FAILED TO OPEN %s",skyboxes[0].c_str());
	if (fs.open(skyboxes[1].c_str(),Torque::FS::File::Read))// NORTH  00
	{
		bitmap->readBitmap("png",fs);
		fs.close();
		for (U32 x=0;x<bitmap->getWidth();x++) {
			for (U32 y=0;y<bitmap->getHeight();y++) {
				ColorI color;
				U32 newY = y+skyOffset;
				if (newY>bitmap->getHeight()) newY=bitmap->getHeight();
				bitmap->getColor(x,newY,color);
				bitmap2->setColor(x,(bitmap->getHeight()-1)-y,color);//Flip vertically
			}
		}
		if (fs.open(skyboxes[6].c_str(),Torque::FS::File::Write))
		{
			bitmap2->writeBitmap("png",fs);
			fs.close();
		}
	}
	if (fs.open(skyboxes[2].c_str(),Torque::FS::File::Read))// EAST  90
	{
		bitmap->readBitmap("png",fs);
		fs.close();
		for (U32 x=0;x<bitmap->getWidth();x++) {
			for (U32 y=0;y<bitmap->getHeight();y++) {
				ColorI color;
				U32 newY = y+skyOffset;
				if (newY>bitmap->getHeight()) newY=bitmap->getHeight();
				bitmap->getColor(x,newY,color);
				bitmap2->setColor(y,x,color);//Flip diagonally, across axis from upper left to lower right.
			}
		}
		if (fs.open(skyboxes[7].c_str(),Torque::FS::File::Write))
		{
			bitmap2->writeBitmap("png",fs);
			fs.close();
		}
	}
	if (fs.open(skyboxes[3].c_str(),Torque::FS::File::Read))// SOUTH  180
	{
		bitmap->readBitmap("png",fs);
		fs.close();
		for (U32 x=0;x<bitmap->getWidth();x++) {
			for (U32 y=0;y<bitmap->getHeight();y++) {
				ColorI color;
				U32 newY = y+skyOffset;
				if (newY>bitmap->getHeight()) newY=bitmap->getHeight();
				bitmap->getColor(x,newY,color);
				bitmap2->setColor((bitmap->getWidth()-1)-x,y,color);//Flip horizontally
			}
		}
		if (fs.open(skyboxes[8].c_str(),Torque::FS::File::Write))
		{
			bitmap2->writeBitmap("png",fs);
			fs.close();
		}
	}
	if (fs.open(skyboxes[4].c_str(),Torque::FS::File::Read))// WEST  270
	{
		bitmap->readBitmap("png",fs);
		fs.close();
		for (U32 x=0;x<bitmap->getWidth();x++) {
			for (U32 y=0;y<bitmap->getHeight();y++) {
				ColorI color;
				U32 newY = y+skyOffset;
				if (newY>bitmap->getHeight()) newY=bitmap->getHeight();
				bitmap->getColor(x,newY,color);
				bitmap2->setColor((bitmap->getHeight()-1)-y,(bitmap->getWidth()-1)-x,color);//Flip diagonally, across axis from lower left to upper right.
			}
		}
		if (fs.open(skyboxes[9].c_str(),Torque::FS::File::Write))
		{
			bitmap2->writeBitmap("png",fs);
			fs.close();
		}
	}
}

void TerrainPager::updateSkyboxConsole()
{
	Con::executef(this,"UpdateSkybox");
}


Point3F TerrainPager::convertLatLongToXYZ(Point3F pos)
{
	Point3F newPos;

	newPos.x = (pos.x - mD.mMapCenterLongitude) * mD.mMetersPerDegreeLongitude;
	newPos.y = (pos.y - mD.mMapCenterLatitude) * mD.mMetersPerDegreeLatitude;
	newPos.z = pos.z;
	return newPos;
}

Point3F TerrainPager::convertLatLongToXYZ(double longitude,double latitude,float altitude)
{
	Point3F newPos;

	newPos.x = (float)((longitude - (double)mD.mMapCenterLongitude) * (double)mD.mMetersPerDegreeLongitude);
	newPos.y = (float)((latitude - (double)mD.mMapCenterLatitude) * (double)mD.mMetersPerDegreeLatitude);
	newPos.z = altitude;
	return Point3F(newPos);
}

//DefineConsoleMethod( TerrainPager, convertLatLongToXYZ, Point3F, (Point3F pos), , "" )
//{
//	Point3F newPos = object->convertLatLongToXYZ(pos);
//	return newPos;
//}

Point3F TerrainPager::convertXYZToLatLong(Point3F pos)
{
	Point3F newPos;
	newPos.x = mD.mMapCenterLongitude + (pos.x * mD.mDegreesPerMeterLongitude);
	newPos.y = mD.mMapCenterLatitude + (pos.y * mD.mDegreesPerMeterLatitude);
	newPos.z = pos.z;

	return newPos;
}

//DefineConsoleMethod( TerrainPager, convertXYZToLatLong, Point3F, (Point3F pos), , "" )
//{
//	Point3F newPos = object->convertXYZToLatLong(pos);
//	return newPos;
//}

const char *TerrainPager::escapeSingleQuotes(std::string *myString)
{
	U32 pos=0,newpos=0;
	while (pos<myString->length())
	{
		pos = myString->find("'",pos);
		if (pos<myString->length())
		{
			myString->insert(pos+1,"'");
			pos += 2;//To get us past the second quote for next pass, and not just add quotes forever.
		}
	}
	return myString->c_str();
}

void TerrainPager::loadOSM(const char *xml_file, const char *map_db)
{
	
	if (!mSQL->OpenDatabase(map_db))
		return;
	
	mDBName = map_db;

	std::ostringstream insert_query;
	char select_query[512],update_query[255],total_queries[56000];
	int id,result,total_query_len=0;
	
	bool findingTag,foundTag;
	sqlite_resultset *resultSet;
	char cellName[20];
	char nodeId[16];//Have to do this because we are STILL USING VS2010. They added atoll() for long longs in 2011
	char wayId[16];				//But it's okay, because sqlite can convert the char string for us.

	SimXMLDocument *doc = new SimXMLDocument();
	doc->registerObject();

	S32 loaded = doc->loadFile(xml_file);
	if (loaded) 
	{
		bool osmLoad = false;
		bool osmBounds = false;

		F32 version,minlat;
		double nodeLat,nodeLon;
		//long nodeId;		

		osmLoad = doc->pushFirstChildElement("osm");
		if (doc->attributeExists("version") )
			version = atof(doc->attribute("version"));

		osmBounds = doc->pushFirstChildElement("bounds");
		if (doc->attributeExists("minlat") )
			minlat = atof(doc->attribute("minlat"));

		Con::printf("opened the document %s, osm %d version %f bounds %d minLat %f",xml_file,osmLoad,version,osmBounds,minlat);

		//sprintf(insert_query,"BEGIN;\n");
		insert_query << "BEGIN;";
		result = mSQL->ExecuteSQL(insert_query.str().c_str());
		insert_query.clear(); insert_query.str("");
		//Con::printf("result %d: %s",result,insert_query);

		while(doc->nextSiblingElement("node"))
		{
			//nodeId = atol(doc->attribute("id"));//AAAH... turns out what we need is atoll(), and it doesn't happen until 2011
			sprintf(nodeId,"%s",doc->attribute("id"));
			nodeLat = atof(doc->attribute("lat"));
			nodeLon = atof(doc->attribute("lon"));
			Point3F nodePos = convertLatLongToXYZ(nodeLon,nodeLat,0.0); 
			getCellName(nodePos,cellName);

			//sprintf(insert_query,"INSERT INTO osmNode (osm_id,latitude,longitude,cell_name) VALUES (%d,%f,%f,'%s');\n",
			//	nodeId,nodeLat,nodeLon,cellName);
			insert_query.precision(8);
			insert_query << "INSERT INTO osmNode (osm_id,latitude,longitude,cell_name) VALUES (CAST('" << nodeId << "' AS INTEGER)," << 
									nodeLat << "," << nodeLon << ",'" << cellName << "');";
			result = mSQL->ExecuteSQL(insert_query.str().c_str());				
			Con::printf("node lat %f long %f insert query: %s",nodeLat,nodeLon,insert_query.str().c_str());
			insert_query.clear(); insert_query.str("");		
			foundTag = false;
			findingTag = doc->pushFirstChildElement("tag");
			foundTag = findingTag;
			while (findingTag)
			{				
				if (!strcmp(doc->attribute("k"),"name"))
				{//HERE: we REALLY need that escape single quotes function, if not an escape every possible oddity function
					std::string name = doc->attribute("v");
					std::string escapedName = escapeSingleQuotes(&name);
					insert_query << "UPDATE osmNode SET name='" << escapedName.c_str() << "' WHERE osm_id=CAST('" << nodeId << "' AS INTEGER);";
					result = mSQL->ExecuteSQL(insert_query.str().c_str());
					Con::printf("updated node name: %s",insert_query.str().c_str());
					insert_query.clear(); insert_query.str("");

				} else if (!strcmp(doc->attribute("k"),"highway")) {
					insert_query << "UPDATE osmNode SET type='" << doc->attribute("v") << "' WHERE osm_id=CAST('" << nodeId << "' AS INTEGER);";
					result = mSQL->ExecuteSQL(insert_query.str().c_str());
					insert_query.clear(); insert_query.str("");

				} else if (!strcmp(doc->attribute("k"),"building") && !strcmp(doc->attribute("v"),"yes")) {
					insert_query << "UPDATE osmNode SET type='building' WHERE osm_id=CAST('" << nodeId << "' AS INTEGER);";
					result = mSQL->ExecuteSQL(insert_query.str().c_str());
					insert_query.clear(); insert_query.str("");
				}
				findingTag = doc->nextSiblingElement("tag");
			}			
			if (foundTag) doc->popElement();
		}

		doc->popElement();		

		bool findingMyWay = doc->pushFirstChildElement("way");
		while(findingMyWay)
		{
			sprintf(wayId,"%s",doc->attribute("id"));
			insert_query << "INSERT INTO osmWay (osm_id) VALUES (CAST('" << wayId << "' AS INTEGER));";
			//Con::printf("result %d: %s",result,insert_query);
			result = mSQL->ExecuteSQL(insert_query.str().c_str());
			insert_query.clear(); insert_query.str("");

			bool findingNd = doc->pushFirstChildElement("nd");
			while(findingNd)
			{
				sprintf(nodeId,"%s",doc->attribute("ref"));

				//sprintf(insert_query,"INSERT INTO osmWayNode (way_id,node_id) VALUES (%d,%d);\n",wayId,nodeId);
				insert_query << "INSERT INTO osmWayNode (way_id,node_id) VALUES (CAST('" << wayId << "' AS INTEGER),CAST('" << nodeId << "' AS INTEGER));";
				Con::printf("result %d: %s",result,insert_query);
				result = mSQL->ExecuteSQL(insert_query.str().c_str());
				insert_query.clear(); insert_query.str("");

				findingNd = doc->nextSiblingElement("nd");
			}
			doc->popElement();
			bool foundTag = false;
			bool findingTag = doc->pushFirstChildElement("tag");
			foundTag = findingTag;
			while (findingTag)
			{				
				Con::printf("tag %s : %s",doc->attribute("k"),doc->attribute("v"));

				if (!strcmp(doc->attribute("k"),"name"))
				{
					//sprintf(insert_query,"UPDATE osmWay SET name='%s' WHERE osm_id=%d;\n",doc->attribute("v"),way_id);
					std::string name = doc->attribute("v");
					std::string escapedName = escapeSingleQuotes(&name);
					insert_query << "UPDATE osmWay SET name='" << escapedName.c_str() << "' WHERE osm_id=CAST('" << wayId << "' AS INTEGER);";
					
					result = mSQL->ExecuteSQL(insert_query.str().c_str());
					insert_query.clear(); insert_query.str("");

				} else if (!strcmp(doc->attribute("k"),"highway")) {
					//sprintf(insert_query,"UPDATE osmWay SET type='%s' WHERE osm_id=%d;\n",doc->attribute("v"),wayId);
					insert_query << "UPDATE osmWay SET type='" << doc->attribute("v") << "' WHERE osm_id=CAST('" << wayId << "' AS INTEGER);";
					result = mSQL->ExecuteSQL(insert_query.str().c_str());
					insert_query.clear(); insert_query.str("");

				} else if (!strcmp(doc->attribute("k"),"building") && !strcmp(doc->attribute("v"),"yes")) {
					//sprintf(insert_query,"UPDATE osmWay SET type='building' WHERE osm_id=%d;\n",wayId);
					insert_query << "UPDATE osmWay SET type='building' WHERE osm_id=CAST('" << wayId << "' AS INTEGER);";
					result = mSQL->ExecuteSQL(insert_query.str().c_str());
					insert_query.clear(); insert_query.str("");
				}

				findingTag = doc->nextSiblingElement("tag");
			}				
			if (foundTag) doc->popElement();
			findingMyWay = doc->nextSiblingElement("way");
		}

		//sprintf(insert_query,"COMMIT;\n");
		insert_query << "COMMIT;";
		result = mSQL->ExecuteSQL(insert_query.str().c_str());
		insert_query.clear(); insert_query.str("");

	} else Con::errorf("Failed to load OpenStreetMap export file: %s",xml_file);

	doc->deleteObject();

	mSQL->CloseDatabase();

	return;
}

DefineConsoleMethod(TerrainPager, loadOSM, void, (const char *xml_file,const char *map_db), , "" )
{
	object->loadOSM(xml_file,map_db);
}

void TerrainPager::makeRoads()
{
	Con::printf("makeRoads starting");
	if (mRoads.size()<=0)
		return;

	SimGroup *missionGroup;
	if ( !Sim::findObject( "MissionGroup", missionGroup ) )               
		Con::errorf( "TerrainPager - could not find MissionGroup to add new DecalRoad" );
	
	F32 worldZ;
	Point3F normalVec;
	F32 width = 10.0;
	//F32 depth = 3.0;
	
	//long nodeId,wayId,wayNodeId;
	char nodeId[16];
	long wayId;
	long wayNodeId;//This one can still safely be treated as a number, up to a couple of billion rows.
	double nodeLong,nodeLat;
	unsigned long startTime =  clock();

	for (std::map<long,osmWay>::iterator it=mRoads.begin(); it!=mRoads.end(); ++it)
	{
		wayId = it->first;
		osmWay way = it->second;
		//Con::printf("looking for way id: %ld",wayId);
		if (mActiveRoads.find(wayId)!=mActiveRoads.end())
			continue;

		//FIX: take some real world measurements here, at least from air photos. No idea what these widths really are.
		const char *wayType = way.type.c_str();
		if ( !strcmp(wayType,"residential") || !strcmp(wayType,"service") )
			width = 8.0;
		else if ( !strcmp(wayType,"tertiary") || !strcmp(wayType,"trunk_link") )
			width = 12.0;
		else if ( !strcmp(wayType,"trunk") || !strcmp(wayType,"motorway_link") || !strcmp(wayType,"secondary") )
			width = 18.0;
		else if ( !strcmp(wayType,"motorway")  )
			width = 24.0;
		else if ( !strcmp(wayType,"footway") )
			width = 2.5;
		else if ( !strcmp(wayType,"path") )
			width = 4.5;
		//else if (strlen(wayType)==0) 
		//	width = 2.0;
		//else if (!strcmp(wayType,"NULL"))
		//	width = 3.5;
		else
		{
			//Con::printf("Street rejected for unknown/unsupported type: %s",wayType);
			continue;
		}

		std::ostringstream select_query;
		select_query << 
			"SELECT wn.id , n.osm_id, n.longitude, n.latitude " <<
			"FROM osmWayNode wn " <<
			"LEFT JOIN osmNode n  ON n.osm_id = wn.node_id " <<
			"WHERE wn.way_id = CAST('" << way.osmId << "' AS INTEGER) ORDER BY wn.id;";

		int result;
		sqlite_resultset *resultSet;

		U32 queryTime = clock();
		result = mSQL->ExecuteSQL(select_query.str().c_str());
		if (result)
		{
			resultSet = mSQL->GetResultSet(result);
			select_query.clear();
			select_query.str("");
			//Con::printf("makeRoads: way %d nodes %d  name %s type %s query time %d ms",
			//	wayId,resultSet->iNumRows,way.name.c_str(),way.type.c_str(),clock()-queryTime);
			if (resultSet->iNumRows > 1)
			{			
				 //FIX! add a variable to specify mesh or decal road, and split a way into new smaller roads 
						//whenever you change from one to the other.
				/*
				MeshRoad *newRoad = new MeshRoad();	

				sprintf(newRoad->mOsmId,"%s",way.osmId);

				//HERE: hook these to a script object
				if ( !strcmp(wayType,"footway") ||  !strcmp(wayType,"path") )
				{
					newRoad->mMaterialName[0] = "DefaultRoadMaterialPath";
					newRoad->mMaterialName[1] = "DefaultRoadMaterialPath";
					newRoad->mMaterialName[2] = "DefaultRoadMaterialPath";
				} else {
					newRoad->mMaterialName[0] = "DefaultRoadMaterialTop";
					newRoad->mMaterialName[1] = "DefaultRoadMaterialOther";
					newRoad->mMaterialName[2] = "DefaultRoadMaterialOther";
				}
				*/
				DecalRoad *newRoad = new DecalRoad();
				
				if ( !strcmp(wayType,"footway") ||  !strcmp(wayType,"path") )
				{
					newRoad->mMaterialName = "DefaultRoadMaterialPath";
				} else {
					newRoad->mMaterialName = "DefaultRoadMaterialTop";
				}

				if (strlen(way.name.c_str())>0)
				{
					newRoad->setInternalName(way.name.c_str());

				}


				newRoad->registerObject();
				
				way.road = newRoad;
				//Con::printf("adding a new road! %d %s",wayId,way.name.c_str());
				
				mActiveRoads[wayId] = way;

				if (missionGroup)
					missionGroup->addObject( newRoad );

				for (int i=0;i<resultSet->iNumRows;i++)
				{
					S32 c=0;

					wayNodeId = dAtol(resultSet->vRows[i]->vColumnValues[c++]);
					sprintf(nodeId,"%s",resultSet->vRows[i]->vColumnValues[c++]);
					nodeLong = dAtod(resultSet->vRows[i]->vColumnValues[c++]);
					nodeLat = dAtod(resultSet->vRows[i]->vColumnValues[c++]);

					Point3F posLatLong(nodeLong,nodeLat,0.0);//Convert to doubles?
					Point3F posXYZ = convertLatLongToXYZ(posLatLong);

					if (!getGroundAtInclusive(posXYZ,&worldZ,&normalVec))//Whoops! Fixed getGroundAt to avoid roads, for the forest, but now I can't 
						continue;//put down road nodes where there are other road nodes, which is a bit of a problem.

					//posXYZ.z = worldZ + (depth * 0.3);//Need enough distance to prevent Z fighting, especially far from the origin.

					//newRoad->insertNode( posXYZ, width, depth, Point3F(0,0,1), 0, 0, nodeId );	
					newRoad->insertNode( posXYZ, width, 0, nodeId );	

					osmNode n;
					sprintf(n.osmId,"%s",nodeId);
					n.longitude = nodeLong;
					n.latitude = nodeLat;
					way.nodes.push_back(n);
				}
			}
		}
	}
	
	unsigned int latency = clock() - startTime;
	Con::printf("makeRoads() took %d milliseconds",latency);

	return;
}

//Now, if a whole way's nodes are all out of our search area, delete the road.
void TerrainPager::pruneRoads()
{
	unsigned long startTime =  clock();
	Con::printf("prune roads starting with %d roads",mRoads.size());
	SimGroup *missionGroup;
	if ( !Sim::findObject( "MissionGroup", missionGroup ) )               
		Con::errorf( "TerrainPager - could not find MissionGroup in pruneRoads()" );
	Vector <int> removedWays;

	for (std::map<long,osmWay>::iterator it=mActiveRoads.begin(); it!=mActiveRoads.end(); ++it)
	{
		int wayId = it->first;
		osmWay way = it->second;
		
		Point3F southWestXYZ,northEastXYZ;//bounding box corners
		Point3F southWestLatLong,northEastLatLong;//bounding box corners
		southWestXYZ = mClientPos + Point3F(-mStreetRadius*1.25,-mStreetRadius*1.25,0.0);
		northEastXYZ = mClientPos + Point3F( mStreetRadius*1.25, mStreetRadius*1.25,0.0);
		southWestLatLong = convertXYZToLatLong(southWestXYZ);
		northEastLatLong = convertXYZToLatLong(northEastXYZ);
		
		bool allOutside = true; 
		for (U32 i=0;i<way.nodes.size();i++)
		{
			osmNode n = way.nodes[i];
			if ( (n.longitude>=southWestLatLong.x)&&(n.longitude<=northEastLatLong.x)&&
					   (n.latitude>=southWestLatLong.y)&&(n.latitude<=northEastLatLong.y) )
				allOutside = false;
		}
		if (allOutside)
		{
			if (missionGroup) 
				missionGroup->removeObject(way.road);
			way.road->unregisterObject();
			removedWays.push_back(wayId);
		}
	}
	for (U32 i=0;i<removedWays.size();i++)
	{//Now, remove them from mActiveRoads, now that we're not iterating through it.
		mActiveRoads.erase(removedWays[i]);
		mRoads.erase(removedWays[i]);
	}

	unsigned int latency = clock() - startTime;
	Con::printf("pruneRoads() took %d milliseconds, ended with %d roads",latency,mRoads.size());
}

void TerrainPager::saveRoads()//WIP: need to step through the road's nodes, and do insert or update logic
{
	Con::printf("TerrainPager::saveRoads");
	SimSet* missionGroup = NULL;
	missionGroup = dynamic_cast<SimSet*>(Sim::findObject("MissionGroup"));

	unsigned long startTime =  clock();

	if (missionGroup == NULL)
		Con::printf("No mission group found.");

	if (!mSQL)
		Con::printf("mSQL is not valid");

	if ((mSQL)&&(missionGroup))//->OpenDatabase(Con::getVariable("$pref::OpenSimEarth::OSMDB")))
	{
		int nodeId;
		int result,total_query_len=0;
		double nodeLong,nodeLat;
		std::ostringstream insert_query,update_query;
		sqlite_resultset *resultSet;
		Point3F pos,latLongPos;
		Point2F cellPos;
		char cellName[20];

		for(SimSetIterator itr(missionGroup); *itr; ++itr)
      {
         MeshRoad* object = dynamic_cast< MeshRoad* >( *itr );
			if (object)
			{				
				pos = object->getPosition();
				latLongPos = convertXYZToLatLong(pos);
				cellPos = getCellCoords(pos);
				getCellName(cellPos.x,cellPos.y,cellName);

				Con::printf("TerrainPager::saveRoads found a MeshRoad %s. pos %f %f %f Nodes %d",object->mOsmId,pos.x,pos.y,pos.z,object->getSegmentCount()); 
				for (U32 i=0;i<object->getSegmentCount();i++)
				{
					Con::printf("found a node, osm_id %s",object->getNodeOsmId(i));
				}
			}
		}
	}
}

DefineConsoleMethod( TerrainPager, saveRoads, void, (), , "" )
{
	object->saveRoads();
	return;
}

	/*
	unsigned long startTime =  clock();
	//big logic and queries ...
	unsigned int latency = clock() - startTime;
	Con::printf("() took %d milliseconds",latency);
	*/


// pointWithinPoly function courtesy of Kevin Ryan of Top Meadow Inc.
// What this does is goes along each side of the polygon and for each edge it creates  
// a new polygon that is perpendicular to the original polygon.  Then we do a check  
// with the point to see which side of the polygon it is on.  If the point is on the  
// wrong side of any of the polys then it is not within the polygon.  

/*
bool TerrainPager::pointWithinPoly(const ConcretePolyList::Poly &poly, const Point3F &point)  
{  //whoops, polyList hasn't been defined...

	Point3F v2 = polyList.mVertexList[polyList.mIndexList[poly.vertexStart + poly.vertexCount - 1]];  

	for (U32 i = 0; i < poly.vertexCount; i++)  
	{  
		Point3F v1 = polyList.mVertexList[polyList.mIndexList[i + poly.vertexStart]];  

		// build a plane normal for this polygon edge  
		PlaneF p(v1 + poly.plane, v1, v2);  
		v2 = v1;  
		F32 dist = p.distToPlane(point);  
		if (dist < 0)  
			return(false);  
	}  
	
	return(true);  
} */ 



DefineConsoleMethod(TerrainPager, makeStreets, void, (), , "" )
{
	object->makeRoads();
}

DefineConsoleMethod(TerrainPager, callUpdateSkybox, void, (), , "" )
{
	Con::printf("calling update skybox");
	object->updateSkyboxConsole();
}

DefineConsoleMethod(TerrainPager, reloadSkybox, void, (), , "" )
{
	Con::printf("calling reload skybox");
	object->reloadSkyboxImages();
}

DefineConsoleMethod( TerrainPager, getTileWidth, F32, (), , "" )
{
	return object->mD.mTileWidth;
}

DefineConsoleMethod( TerrainPager, getTileWidthLatitude, F32, (), , "" )
{
	return object->mD.mTileWidthLatitude;
}

DefineConsoleMethod( TerrainPager, getTileWidthLongitude, F32, (), , "" )
{
	return object->mD.mTileWidthLongitude;
}

DefineConsoleMethod( TerrainPager, getHeightmapRes, S32, (), , "" )
{
	return object->mD.mHeightmapRes;
}

DefineConsoleMethod( TerrainPager, getTextureRes, S32, (), , "" )
{
	return object->mD.mTextureRes;
}

DefineConsoleMethod( TerrainPager, getLightmapRes, S32, (), , "" )
{
	return object->mD.mLightmapRes;
}

DefineConsoleMethod( TerrainPager, getMetersPerDegreeLatitude, F32, (), , "" )
{
	return object->mD.mMetersPerDegreeLatitude;
}

DefineConsoleMethod( TerrainPager, getMetersPerDegreeLongitude, F32, (), , "" )
{
	return object->mD.mMetersPerDegreeLongitude;
}

DefineConsoleMethod( TerrainPager, getDegreesPerMeterLatitude, F32, (), , "" )
{
	return object->mD.mDegreesPerMeterLatitude;
}

DefineConsoleMethod( TerrainPager, getDegreesPerMeterLongitude, F32, (), , "" )
{
	return object->mD.mDegreesPerMeterLongitude;
}

DefineConsoleMethod( TerrainPager, getClientPosLatitude, F32, (), , "" )
{
	object->findClientPos();
	return object->mD.mClientPosLatitude;
}

DefineConsoleMethod( TerrainPager, getClientPosLongitude, F32, (), , "" )
{
	object->findClientPos();
	return object->mD.mClientPosLongitude;
}

DefineConsoleMethod( TerrainPager, getClientLongLat, const char*, (), , "" )
{
	object->findClientPos();
	char posLongLat[50];
	sprintf(posLongLat,"%f %f",object->mD.mClientPosLongitude,object->mD.mClientPosLatitude);
	return posLongLat;
}

DefineConsoleMethod( TerrainPager, getClientPos, Point3F, (), , "" )
{
	object->findClientPos();
	return object->mClientPos;
}

DefineConsoleMethod( TerrainPager, findClientTile, void, (), , "" )
{
	object->findClientTile();
	return;
}

DefineConsoleMethod( TerrainPager, getClientTileName, const char*, (), , "" )
{
	char tileName[20];
	object->findClientTile();
	object->getTileName(object->mTileStartLongitude,object->mTileStartLatitude,tileName);
	return tileName;//Is this safe, because it's a console method? Returning local char array?
}

DefineConsoleMethod( TerrainPager, getClientTilePos, Point2F, (), , "" )
{
	object->findClientTile();
	Point2F tilePos = Point2F(object->mTileStartLongitude,object->mTileStartLatitude);
	return tilePos;
}

DefineConsoleMethod( TerrainPager, getTileLoadRadius, F32, (), , "" )
{
	return object->mD.mTileLoadRadius;
}

DefineConsoleMethod( TerrainPager, getTileDropRadius, F32, (), , "" )
{
	return object->mD.mTileDropRadius;
}

DefineConsoleMethod( TerrainPager, openListenSocket, void, (), , "" )
{
	if (object->mUseDataSource)
		object->mDataSource->openListenSocket();
	return;
}

DefineConsoleMethod( TerrainPager, connectListenSocket, void, (), , "" )
{
	if (object->mUseDataSource)
		object->mDataSource->connectListenSocket();
	return;
}

DefineConsoleMethod( TerrainPager, listenForPacket, void, (), , "" )
{
	if (object->mUseDataSource)
		object->mDataSource->listenForPacket();
	return;
}

DefineConsoleMethod( TerrainPager, connectSendSocket, void, (), , "" )
{
	if (object->mUseDataSource)
		object->mDataSource->connectSendSocket();
	return;
}

DefineConsoleMethod( TerrainPager, sendPacket, void, (), , "" )
{
	if (object->mUseDataSource)
		object->mDataSource->sendPacket();
	return;
}

DefineConsoleMethod( TerrainPager, dropAll, void, (), , "" )
{
	object->dropAllTerrains();
	return;
}

DefineConsoleMethod( TerrainPager, setDBName, void , (const char* name), , "" )
{
	object->mDBName = name;
	return;
}

DefineConsoleMethod( TerrainPager, convertLatLongToXYZ, Point3F, (Point3F pos), , "" )
{
	Point3F xyzPos = object->convertLatLongToXYZ(pos);
	return xyzPos;
}

DefineConsoleMethod( TerrainPager, convertXYZToLatLong, Point3F, (Point3F pos), , "" )
{
	Point3F latLongPos = object->convertXYZToLatLong(pos);
	return latLongPos;
}

DefineConsoleMethod( TerrainPager, getTileName, const char*, (Point3F pos), , "" )
{
	char tileName[20];
	object->getTileName(pos.x,pos.y,tileName);
	return tileName;//Is this safe, because it's a console method? Returning local char array?
}

DefineConsoleMethod( TerrainPager, getCellName, const char*, (Point3F pos), , "" )
{
	char cellName[20];

	object->getCellName(pos,cellName);

	return cellName;//?
}

DefineConsoleMethod( TerrainPager, getCellCoords, Point2F, (Point3F pos), , "" )
{
	Point2F coords = object->getCellCoords(pos);
	Con::printf("Found cell coords: %f %f",coords.x,coords.y);
	return coords;
}

///////////////  Disregard obsolete code from here down except as reference ///////////

/*

DefineConsoleMethod( TerrainPager, getTileName, const char*, (F32 longitude,F32 latitude), , "" )
{
	return object->getTileName(longitude,latitude);
}
DefineConsoleMethod( TerrainPager, getTileNameByIndex, const char*, (S32 index), , "" )
{
	if ((index>0)&&(index<object->mTileNames.size()))
		return object->mTileNames[index].c_str();
	else
		return "";
}
DefineConsoleMethod( TerrainPager, loadTileNames, void, (), , "" )
{
	object->loadTileNames();
	return;
}

void TerrainPager::loadTileNames()
{
	S32 centerTile,tilesFromCenterLong,tilesFromCenterLat;
	F32 blockLong,blockLat;
	char fileName[512];

	centerTile = (mGridSize-1)/2;
	
	findClientPos();

	tilesFromCenterLong = mFloor((mClientPos.x + (mD.mTileWidth/2.0f)) / mD.mTileWidth) - centerTile;
	tilesFromCenterLat = mFloor((mClientPos.y + (mD.mTileWidth/2.0f)) / mD.mTileWidth) - centerTile;
	Con::printf("tiles from center lat %d  long %d",tilesFromCenterLat,tilesFromCenterLong);

	if (mTileNames.size()>0)
		mTileNames.clear();

	for (S32 y = 0;y < mD.mGridSize; y++) {
      for (S32 x = 0;x < mD.mGridSize; x++) {
		  blockLong = mD.mMapCenterLongitude - (mD.mTileWidthLongitude / 2.0f) +  ((tilesFromCenterLong + x) * mD.mTileWidthLongitude);
		  blockLat = mD.mMapCenterLatitude - (mD.mTileWidthLatitude / 2.0f) +  ((tilesFromCenterLat + y) * mD.mTileWidthLatitude);
		  mTileNames.increment();
		  sprintf(fileName,"%shght.%s.bin",mD.mTerrainPath.c_str(),getTileName(blockLong,blockLat));
		  mTileNames.last() = fileName;
		  Con::printf("tilename %s ",mTileNames.last().c_str());
	  }
   }
        
	//Well, meant to do this in script, but foiled by three digit float accuracy in scripts, apparently.
}


DefineConsoleMethod( TerrainPager, getGridSize, S32, (), , "" )
{
	return object->mD.mGridSize;
}
DefineConsoleMethod( TerrainPager, setGridSize, void, (U32 gridSize), , "" )
{
	if (gridSize<3) object->mD.mGridSize = 3;
	else if (gridSize>9) object->mD.mGridSize = 9;
	else if ((gridSize==4)||(gridSize==6)||(gridSize==8))
		object->mD.mGridSize = gridSize - 1;
	else object->mD.mGridSize = gridSize;
	return; 
}
*/
	/*
	//HERE: we need to find the correct set of nine lat/long combos to load given the player position, and 
	//then run them through getTileName().
	F32 start_long,start_lat;
	//for (int x=0;x<3;x++) 
	//{
		//for (int y=0;y<3;y++) 
	//{
	start_long = mStartLongitude + (x * mTileWidthLongitude);
	start_lat = mStartLatitude + (y * mTileWidthLatitude);
	F32 data;
	char heightsfilename[255],texturesfilename[255];

	mTerrainHeightsBinFile = mTerrainPath + "hght." + getTileName(start_long,start_lat) + ".bin";
	mTerrainTexturesBinFile = mTerrainPath + "text." + getTileName(start_long,start_lat) + ".bin";

	Con::printf("heights bin %d %d: %s",x,y,getTileName(start_long,start_lat));

	//FILE *fp = fopen(mTerrainHeightsBinFile.c_str(),"r+b");
	FileStream fs;
	if (fs.open(mTerrainHeightsBinFile.c_str(),Torque::FS::File::Read))
	{
		fs.setPosition(0*sizeof(float)); fs.read(&data);
		F32 fileLong = data;
		fs.setPosition(1*sizeof(float)); fs.read(&data);
		F32 fileLat = data;
		fs.setPosition(2*sizeof(float)); fs.read(&data);
		F32 fileTileWidth = data;
		fs.setPosition(3*sizeof(float)); fs.read(&data);
		U32 fileHeightmapRes = (U32)data;
		fs.setPosition(4*sizeof(float)); fs.read(&data);
		U32 fileTextureRes = (U32)data;
		if (fileHeightmapRes!=mHeightmapRes)
		{
			Con::printf("Wrong heightmap resolution in file: %s",heightsfilename);
			//for(int i=0; i<mTerrainsXCount; i++)//First time, we're going to have to load all terrains for sure.
			//	for(int j=0; j<mTerrainsZCount; j++)
			//		mLoadTerrains[j][i] = true; 
			//pingWorldServer(true);
			return;
		}
		Con::printf("Loading terrain data from:  %s, long/lat: %f %f  heightmapres %d  tileWidth %f",
			heightsfilename,fileLong,fileLat,fileHeightmapRes,mTileWidth);
		for (U32 xx=0;xx<mHeightmapRes;xx++)
		{	
			for (U32 yy=0;yy<mHeightmapRes;yy++)
			{
				fs.setPosition((yy*mHeightmapRes*sizeof(float)) + (xx*sizeof(float)) + (mNumHeightBinArgs*sizeof(float)) );
				fs.read(&data);
				mTerrain->setHeight(Point2I(xx,yy),data);
				//mTerrainsLayout[y][x]->setHeight(Point2I(xx,yy),data);					
			}
		}
		fs.close();

		//Now, set terrain position to where it should be based on our previously determined startPos.
		Point3F terrPos(mStartPos.x+((float)(x)*(mTileWidth-mSquareSize)),mStartPos.y+((float)(y)*(mTileWidth-mSquareSize)),0.0);
		//Hmm, not sure if it's a bug in Torque or not, but WorldBlockSize should be 2550 should it not?  Because
		//the last data point is shared with the next terrain, it doesn't have a square of its own.
		mTerrainsLayout[y][x]->setPosition(terrPos);
	}

	//Here, not sure if it will help to do this twice, but it seems like once isn't enough.  Maybe another function...
	mTerrainsLayout[y][x]->updateGrid(Point2I(0,0),
		Point2I(mTerrainsLayout[y][x]->getBlockSize()-1,mTerrainsLayout[y][x]->getBlockSize()-1),true);

	U8 texData;
	if (fs.open(mTerrainTexturesBinFile.c_str(),Torque::FS::File::Read))
	{
		TerrainFile *file = mTerrainsLayout[y][x]->getFile();
		for (U32 xx=0;xx<mTextureRes;xx++)
		{	
			for (U32 yy=0;yy<mTextureRes;yy++)
			{
				fs.setPosition((yy*mTextureRes*sizeof(U8)) + (xx*sizeof(U8)));
				fs.read(&texData);
				U8 index = texData - 1;//Flightgear puts out 1-based numbers, here we are 0-based.
				if ((index>=0)&&(index<=2))
					file->setLayerIndex( xx, yy, index );
			}
		}
		fs.close();
	}

	//Here, second time's a charm?
	mTerrainsLayout[y][x]->updateGrid(Point2I(0,0),
		Point2I(mTerrainsLayout[y][x]->getBlockSize()-1,mTerrainsLayout[y][x]->getBlockSize()-1),true);
	
	//}
	//}
	}


	//So, btw, this doesn't go anywhere near here but this is a snippet for creating the IG interface style of socket communication.
	char *returnBuffer;
	returnBuffer = new char[packetsize];
	float *outArray;
	outArray = new float[packetsize/sizeof(float)];
	int numSendControls = 1;

	outArray[0] = (float)numSendControls;//May become relevant if we start returning more complex data.
	outArray[1] = 101.0;//CIGI code for StartOfFrame
	outArray[2] = (float)igFrame++;//Mandatory IG frame argument.
	returnBuffer = reinterpret_cast<char*>(outArray);
	n = send(newsockfd,returnBuffer,packetsize,0);

	*/

/*
	if (mUseDataSource)
	{
		FileStream fs;
		if (fs.open(heightfilename,Torque::FS::File::Read))
		{ 
			fs.close();
		} else if (mDataSource->mReadyForRequests && mSentInitRequests) {
			Con::printf("Can't find height file %s, adding request to world data source. ready %d",heightfilename,mDataSource->mReadyForRequests);
			if (mSentTerrainRequest==false)//(checkTerrainLock()==false) 
			{
				Con::printf("requesting terrain tile: %f %f",startLong,startLat);
				//mDataSource->addTerrainRequest(startLong,startLat);
				mDataSource->addSkyboxRequest(startLong,startLat,mD.mClientPosLongitude,mD.mClientPosLatitude);
				mSentTerrainRequest = true;
			} else Con::printf("can't request terrain, source busy");
			return false;//Don't add anything until we can open the file. But first check for lockfile?
		} else {
			Con::printf("Can't find height file %s, not ready for requests.",heightfilename);
			return false;//else Con::printf("Can't add terrain block, readyForRequests %d",mDataSource->mReadyForRequests);
		}
	}
	
*/