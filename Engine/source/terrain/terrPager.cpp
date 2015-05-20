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


#include <iostream>
#include <sstream>
#include <fstream>


#include <vector>
#include <string>

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
	mSkyboxTickInterval = 1200;// 30/sec * 60, about a minute
	mSkyboxLoadDelay = 60;//two seconds

	mLoadState = 0;//For now just use numbers, make an enum when you know what the states are going to be.

	mWorldDataSource = NULL;
	mSentInitRequests = false;
	mLoadedTileGrid = false;
	mSentTerrainRequest = false;
	mSentSkyboxRequest = false;

	mClientPos.zero();
	mStartPos.zero();

	//////////////////////////////
	mD.mMapCenterLongitude = 0.0;
	mD.mMapCenterLatitude = 0.0;
	
	mD.mClientPosLongitude = 0.0f;
	mD.mClientPosLatitude = 0.0f;
	mD.mClientPosAltitude = 0.0f;
	
	mD.mTileLoadRadius = 4800.0f;//Arbitrary values, just make sure the drop
	mD.mTileDropRadius = 6200.0f;//value is always larger than the load value!

	mD.mTileWidth = 0.0f;//2550 m squares, by default, in a 256x256 terrain grid with 10m square size.
	mD.mTileWidthLongitude = 0.0f;
	mD.mTileWidthLatitude = 0.0f;
	
	//Would like to receive these through console, but TypeString seems to be failing for me.
	mD.mTerrainPath = "art/terrains/terrainFiles/";
	mD.mSkyboxPath = "art/skies/night/";

	mD.mTerrainLockfile = "art/terrains/terrainFiles/lockfile.terrain.tmp";//Revisit this when paths are
	mD.mSkyboxLockfile = "art/skies/night/lockfile.skybox.tmp";//   working from script as they should.

	mD.mSkyboxRes = mSkyboxRes;
	//////////////////////////////
}

TerrainPager::~TerrainPager()
{	
	//Note: make sure to delete all terrainBlock objects, this will only delete pointers themselves I think.
	///Although if they were added to the scene, then they should get deleted automatically on scene exit.
	mTerrains.clear();
	mTerrainGrid.clear();
	if ((mUseDataSource)&&(mWorldDataSource))
		delete mWorldDataSource;

	//mSQL->CloseDatabase();
	//delete mSQL;
}

void TerrainPager::initPersistFields()
{
	
	//This should represent a point at the center of the entire region of interest. Necessary to prevent floating point error.
	addField( "mapCenterLongitude", TypeF32, Offset( mMapCenterLongitude, TerrainPager ), "Map Center, Longitude" );
	addField( "mapCenterLatitude", TypeF32, Offset( mMapCenterLatitude, TerrainPager ), "Map Center, Latitude" );
		
	addField( "tileLoadRadius", TypeF32, Offset( mTileLoadRadius, TerrainPager ), "Radius at which to load a tile." );
	addField( "tileDropRadius", TypeF32, Offset( mTileDropRadius, TerrainPager ), "Radius at which to drop a tile." );
		
	addField( "skyboxRes", TypeS32, Offset( mSkyboxRes, TerrainPager ), "Skybox image resolution." );
	
	addField( "useDataSource", TypeBool, Offset(mUseDataSource, TerrainPager ), "Use worldDataSource for terrain/skybox updates.");
	
	Parent::initPersistFields();
}

	//What the hell? TypeString just does not work?
	//addField("skyboxPath", TypeString, Offset( mSkyboxPath, TerrainPager ));
	//addField("terrainPath", TypeString, Offset( mTerrainPath, TerrainPager ));

	//addField( "minHeight", TypeF32, Offset( mMinHeight, TerrainMaster ), "Terrain Matrix min height" );
	//addField( "maxHeight", TypeF32, Offset( mMaxHeight, TerrainMaster ), "Terrain Matrix max height" );
	
	//addField( "tileWidth", TypeF32, Offset( mTileWidth, TerrainPager ), "Tile Width in Meters" );
	//addField( "squareSize", TypeF32, Offset( mSquareSize, TerrainPager ), "Square Size in Meters" );
	
	//addField( "gridSize", TypeS32, Offset( mGridSize, TerrainPager ), "Number (per side) of neighboring terrains in grid surrounding base terrain. Must be an odd integer greater than one." );

	//This perhaps could be automatically calculated using latitude of map center.
	//addField( "tileWidthLongitude", TypeF32, Offset( mTileWidthLongitude, TerrainPager ), "Map Center, Longitude" );
	//addField( "tileWidthLatitude", TypeF32, Offset( mTileWidthLatitude, TerrainPager ), "Map Center, Latitude" );

bool TerrainPager::onAdd()
{
	//First, load up the decisions that have to come from the user.
	mD.mMapCenterLongitude = mMapCenterLongitude;
	mD.mMapCenterLatitude = mMapCenterLatitude;

	mD.mTileLoadRadius = mTileLoadRadius;
	mD.mTileDropRadius = mTileDropRadius;

	mD.mSkyboxRes = mSkyboxRes;
	
	//Con::printf("calling TerrainPager::onAdd() mTileLoadRadius %f, mTileDropRadius %f",
	//	mTileLoadRadius,mTileDropRadius);
	
	if ( !Parent::onAdd() )
		return false;
	
	
	//terrain_materials.push_back( "city1" );
	//terrain_materials.push_back( "forest1a" );
	//terrain_materials.push_back( "drycrop2" );
	//terrain_materials.push_back( "grass_green_hires" );
	//terrain_materials.push_back( "sand_hires" );
	//terrain_materials.push_back( "water" );
	//terrain_materials.push_back( "asphalt" );

	//FIX!! Need to read materials from terrain block
	terrain_materials.push_back( "TT_Gravel_02" );
	terrain_materials.push_back( "TT_Grass_01" );
	terrain_materials.push_back( "TT_Grass_20" );
	terrain_materials.push_back( "TT_Rock_14" );
	terrain_materials.push_back( "TT_Sand_01" );
	terrain_materials.push_back( "TT_Snow_01" );
	terrain_materials.push_back( "TT_Snow_02" );
	terrain_materials.push_back( "TT_Mud_07" );

	Con::printf("Terrain pager skybox path: %s",mD.mSkyboxPath.c_str());

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

			Con::printf("terrain materials: %d",mTerrain->mBaseTextures.size());
			//for (U32 i=0;i<mTerrain->mBaseTextures.size();i++)
			//{
			//	Con::printf("material %d: %s",i,mTerrain->mBaseTextures[i].??? // Is there any way to get name back out??
			//}
		}
	}
	
	//delete mTerrain;//Is this safe, at all? What is proper way to remove a terrain block? MissionGroup.remove()?

	//mD.mSkyboxLockfile = mD.mSkyboxPath + "lockfile.skybox.tmp";
	//mD.mTerrainLockfile = mD.mTerrainPath + "lockfile.terrain.tmp";
	
	findClientTile();

	//This can only be 3x3, 5x5, etc.
	mGridSize = 1 + (2 * ((int)(mD.mTileLoadRadius /  mD.mTileWidth) + 1));
	Con::printf("found terrain! tile width: %f tileWidthLat %f tileWidthLong %f gridSize %d textureRes %d",
				mD.mTileWidth,mD.mTileWidthLatitude,mD.mTileWidthLongitude,mGridSize,mD.mTextureRes);

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
		mWorldDataSource = new worldDataSource(false,&mD);
	}

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

	if (mLoadState==0)
	{//Just found our first useful client position.
		Con::printf("terrainPager first client info: long/lat %f %f, pos %f %f",
					mD.mClientPosLongitude,mD.mClientPosLatitude,mClientPos.x,mClientPos.y);

		mLastTileStartLong = mTileStartLongitude;
		mLastTileStartLat = mTileStartLatitude;
		
		if (!mUseDataSource) //Fortunately, the path is short if we're not.
		{
			loadTileGrid();//Don't do this till we're ready to send requests, if we have a dataSource.
			mLoadState = 10;//arbitrary, 10 = done with initial terrain loads, go to steady checking from here.
			return;
		} else {
			mLoadState = 1;
			return;
		}
	}
	else if (mLoadState==1) 
	{//Next tick after finding client pos, here we have to split up based on whether we're using a dataSource or not.
		 if (mWorldDataSource->mReadyForRequests)
		{
			mWorldDataSource->addInitTerrainRequest(&mD,mD.mTerrainPath.c_str());
			mWorldDataSource->addInitSkyboxRequest(mD.mSkyboxRes,0,mD.mSkyboxPath.c_str());
			Con::printf("TerrainPager sending init requests.");
			mSentInitRequests = true;
			mLoadState = 2;
			return;
		} else Con::printf("TerrainPager waiting for worldDataSource to be ready.");
	}
	else if (mLoadState==2) 
	{//first time in a new position, ready for fresh load.
		Con::printf("TerrainPager loading tile grid");
		loadTileGrid();
		//mLoadState=10;//arbitrary, 10 = done with initial terrain loads, go to steady checking from here.
		//Whoops, can't assign a loadState here, because loadTileGrid needs to make that decision.
		return;
	}
	else if ((mLoadState==3) &&(mUseDataSource))//Should never get here without mUseDataSource, but for safety..
	{
		if (mWorldDataSource->checkTerrainLock()==false)
		{
			mLoadState=2;//If terrain is no longer locked, then go back and try load again.
		} else {			
			mWorldDataSource->tick();
			if (mCurrentTick++ % mTickInterval == 0)
				Con::printf("waiting for terrain lock...");
		}
		return;//otherwise just keep coming here.
	}
	else if ((mLoadState==4) &&(mUseDataSource))//Should never get here without mUseDataSource, but for safety..
	{//Be very careful, could get stuck here forever and quietly stop checking terrain, if skybox lockfile not deleted.
		if (mWorldDataSource->checkSkyboxLock()==false)
		{
			Con::printf("reloading skybox!!!!");
			reloadSkybox();
			mSentSkyboxRequest = false;
			mLastSkyboxTick = mCurrentTick;
			mLoadState=10;
		} else {
			mWorldDataSource->tick();
			//if (mCurrentTick++ % mTickInterval == 0)
			Con::printf("waiting for skybox lock...");
		}
		return;//otherwise just keep coming here.
	}
	//else if ((mLoadState==5)&&(mUseDataSource))//Should never get here without mUseDataSource, but for safety..
	//{
	//	if ((mCurrentTick - mLastSkyboxTick) > mSkyboxLoadDelay)
	//	{
	//		Con::executef(this,"UpdateSkybox");
	//		mLoadState = 10;
	//	}
	//}
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
			if (mCurrentTick++ % mTickInterval == 0)
				checkTileGrid();
		}		

		//if (mCurrentTick % mTickInterval == 0)
		//{
		//	Con::printf("lastSkyboxTick %d, currentTick %d return controls %d, sent request %d",
		//		mLastSkyboxTick,mCurrentTick,mWorldDataSource->mNumReturnControls,mSentSkyboxRequest);
		//}

		if ((mUseDataSource)&&(mWorldDataSource->mNumReturnControls==1)&&
			((S32)mLastSkyboxTick < ((S32)mCurrentTick - (S32)mSkyboxTickInterval))&&
			(mSentSkyboxRequest == false))
		{
			mWorldDataSource->addSkyboxRequest(mTileStartLongitude,mTileStartLatitude,mD.mClientPosLongitude,mD.mClientPosLatitude,mD.mClientPosAltitude);
			mLoadState = 4;
			mSentSkyboxRequest = true;
		}
	}
	
	if (mUseDataSource)
	{
		mWorldDataSource->tick();
	}
	
	if ((mCurrentTick % 60 == 0)&&(mUseDataSource))
	{
		Con::executef(this,"UpdateSkybox");//hell with it, this takes almost no time, do it every couple of seconds for now.
	}
	


	/*
	//Now, do this in the second tick, or whenever we're ready.
	if (mUseDataSource && !mLoadedTileGrid && mWorldDataSource->mReadyForRequests)
		loadTileGrid();

	//Now, if we're past the first couple of ticks, find out if we've changed tiles.	
	if (mLastTileStartLong != mTileStartLongitude) newTile = true;
	if (mLastTileStartLat  != mTileStartLatitude) newTile = true;
	if (newTile) 
	{
		Con::printf("new tile: last pos %f %f current pos %f %f",mLastTileStartLong,mTileStartLatitude,
								mTileStartLongitude,mTileStartLatitude);

		loadTileGrid();
	} else {
		checkTileGrid();
	}
	mLastTileStartLong = mTileStartLongitude;
	mLastTileStartLat = mTileStartLatitude;
	
	//Con::printf("found %d terrains that need to be loaded!",mLoadTiles.size());
	Con::executef(this, "onTick", getIdString());

	if (mUseDataSource)
	{  //First time we can, let's send init requests to set up data source for terrains and skyboxes.
		if (!mSentInitRequests && mWorldDataSource->mReadyForRequests)
		{
			Con::printf("trying to send init requests!");
			mWorldDataSource->addInitTerrainRequest(&mD,mD.mTerrainPath.c_str());
			mWorldDataSource->addInitSkyboxRequest(mD.mSkyboxRes,0,mD.mSkyboxPath.c_str());
			mSentInitRequests = true;
			return;
		} else { //Otherwise just give it a regular tick.
			mWorldDataSource->tick();
		}
	}
	*/
	//mCurrentTick++;
}

void TerrainPager::interpolateTick(F32 f)
{
	//Need this because abstract in parent class.
}

void TerrainPager::advanceTime(F32 f)
{
	//Need this because abstract in parent class.
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
	char terrainName[50],heightfilename[256],texturefilename[256];
	TerrainBlock *block;
	bool terrExists = false;

	sprintf(heightfilename,"%shght.%s.bin",mD.mTerrainPath.c_str(),getTileName(startLong,startLat));
	sprintf(texturefilename,"%stext.%s.bin",mD.mTerrainPath.c_str(),getTileName(startLong,startLat));
	sprintf(terrainName,"terrain.%s.ter",getTileName(startLong,startLat));
	FileName terrFileName = mD.mTerrainPath + terrainName;

	//Now, since we're checking previously in loadTerrainGrid or checkTerrainGrid, we are assuming 
	//these files exist and are not calling worldDataSource for them from here.
	if (checkFileExists(terrFileName.c_str())==false)
	{
		//Vector<String> materials;
		//materials.push_back( "city1" );
		//materials.push_back( "forest1a" );
		//materials.push_back( "drycrop2" );
		//materials.push_back( "grass_green_hires" );
		//materials.push_back( "sand_hires" );
		//materials.push_back( "water" );
		//materials.push_back( "asphalt" );
		//Con::printf("trying to make terrain file: %s heightmapres %d  materials[0] %s",terrFileName.c_str(),mD.mHeightmapRes, materials[0].c_str());	
		TerrainFile::createByName( &terrFileName, mD.mHeightmapRes, terrain_materials );
	} else 
		terrExists = true;

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
	//Con::printf("added a terrainblock, long %f lat %f start %f %f",block->mLongitude,block->mLatitude,startLong,startLat);
	Point3F blockPos = Point3F(((startLong-mD.mMapCenterLongitude)*mD.mMetersPerDegreeLongitude),
						(startLat-mD.mMapCenterLatitude)*mD.mMetersPerDegreeLatitude,0);//FIX! need maxHeight/minHeight
	//Con::printf("Setting new block position: %f %f %f, long/lat %f %f",blockPos.x,blockPos.y,blockPos.z,startLong,startLat);
	block->setPosition(blockPos );

	mTerrains.increment();
	mTerrains.last() = block;

	
	block->registerObject( terrainName );
	block->addToScene();

	//Con::printf("added a terrainblock, %f %f %f, long/lat %f %f mTerrains %d",
	//	blockPos.x,blockPos.y,blockPos.z,startLong,startLat,mTerrains.size());
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


	return block;
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
	//mTerrains[index]->safeDeleteObject();
	mTerrains.erase(index);
	*/
}

void TerrainPager::dropAllTerrains()
{//pretty much just for testing...
	//for (int c=mTerrains.size()-1;c>=0;c--)
	//{
		//mTerrains[c]->safeDeleteObject();
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

const char* TerrainPager::getTileName(F32 tileStartPointLong,F32 tileStartPointLat)
{
	String returnStr;
	//returnStr = "123d115_43d988";
	
	char temp[255];
	
	double tileStartPointLongR,tileStartPointLatR;
	tileStartPointLongR = (float)((int)(tileStartPointLong * 1000.0))/1000.0;//Maybe?
	tileStartPointLatR = (float)((int)(tileStartPointLat * 1000.0))/1000.0;//Maybe?
	//Debug.Log("Pre rounding longitude: " + tileStartPointLong + ", post rounding: " + tileStartPointLongR + 
	//			", pre latitude " + tileStartPointLat + " post " + tileStartPointLatR);
	char longC,latC;
	if (tileStartPointLong<0.0) longC = 'W';
	else longC = 'E';
	if (tileStartPointLat<0.0) latC = 'S';
	else latC = 'N';
	//NOW, just have to separate out the decimal part from the whole numbers part, and make sure to get
	//preceding zeroes for the decimal part, so it's always three characters.
	//NOTE: there is a Convert class in  C#, with various ToString functions, but this would all be C# specific,
	//might as well do it myself the hard way and it will at least be easy to convert it over to C++ as well.
	//string testString = Convert.ToString(tileStartPointLongR,??);
	//string longitudeName = 
	int majorLong,minorLong,majorLat,minorLat;//"major" for left of decimal, "minor" for right of decimal.
	std::string majorLongStr,minorLongStr,majorLatStr,minorLatStr;

	majorLong = (int)tileStartPointLongR;
	majorLat = (int)tileStartPointLatR;
	
	//minorLong = Math.Abs((int)((tileStartPointLongR - (float)majorLong) * 1000.0f));//NOPE!  float errors creeping in...
	minorLong = abs( (int)(tileStartPointLongR * 1000.0) - (majorLong * 1000)  );//Turns out doubles fixed it.
	//minorLat = Math.Abs((int)((tileStartPointLatR - (float)majorLat) * 1000.0));
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

	//returnStr = majorLongStr + "d" + minorLongStr + longC + "_" + majorLatStr + "d" + minorLatStr + latC;
	sprintf(temp,"%sd%s%c_%sd%s%c",majorLongStr.c_str(),minorLongStr.c_str(),longC,majorLatStr.c_str(),minorLatStr.c_str(),latC);
	returnStr = temp;
	
	return temp;// returnStr.c_str();
}

void TerrainPager::findClientPos()
{
	Vector<SceneObject*> kCameras;
	Vector<SceneObject*> kPlayers;

	Box3F bounds;
	bounds.set(Point3F(-1024000,-1024000,0),Point3F(1024000,1024000,10000));
	gServerContainer.findObjectList(bounds, CameraObjectType, &kCameras);
	gServerContainer.findObjectList(bounds, PlayerObjectType, &kPlayers);

	//Con::printf("seeking client pos... cameras %d  players %d",kCameras.size(),kPlayers.size());
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
	//The following is off by one half tile width because of my (perhaps questionable) decision to put 
	//map center in the center of a tile rather than at the lower left corner of that tile. Subject to review.
	Point2F mapCenter = Point2F(mD.mMapCenterLongitude-(mD.mTileWidthLongitude/2.0f),
								mD.mMapCenterLatitude-(mD.mTileWidthLatitude/2.0f));

	mLastTileStartLong = mTileStartLongitude;
	mLastTileStartLat  = mTileStartLatitude;
	mTileStartLongitude = (mFloor((clientPos.x-mapCenter.x)/mD.mTileWidthLongitude)*mD.mTileWidthLongitude)+mapCenter.x;
	mTileStartLatitude = (mFloor((clientPos.y-mapCenter.y)/mD.mTileWidthLatitude)*mD.mTileWidthLatitude)+mapCenter.y;
	//Con::printf("Finding tile start: clientPos.y %f mapCenter.y %f  tileWidthlat %f tileStartLat %f",
	//	clientPos.y,mapCenter.y,mD.mTileWidthLatitude,mTileStartLatitude);
}

void TerrainPager::loadTileGrid()
{
	bool loaded;
	bool verbose = false;
	char tileName[20],heightfilename[256],texturefilename[256],terrainfilename[256];
	U32 gridMidpoint = (mGridSize-1)/2;

	F32 startLong = mTileStartLongitude - (gridMidpoint * mD.mTileWidthLongitude);
	F32 startLat = mTileStartLatitude - (gridMidpoint * mD.mTileWidthLatitude);
	Con::printf("loading tile grid, client pos %f %f, client tile start %f %f, local grid start %f %f\n",
		mD.mClientPosLongitude,mD.mClientPosLatitude,mTileStartLongitude,mTileStartLatitude,startLong,startLat);
	//Wait, okay, *first* we need to clear the grid, *then* go ahead and fill it again.
	for (int y=0;y<mGridSize;y++) 
	{
		for (int x=0;x<mGridSize;x++) 
		{
			mTerrainGrid[y*mGridSize+x]=NULL;
		}
	}
	if (verbose) 
	{
		for (int c=0;c<mTerrains.size();c++)
		{
			Con::printf("terrain %d longitude %f latitude %f",c,mTerrains[c]->mLongitude,mTerrains[c]->mLatitude);
		}
	}
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
			sprintf(tileName,getTileName(kLong,kLat));
			sprintf(heightfilename,"%shght.%s.bin",mD.mTerrainPath.c_str(),tileName);
			sprintf(texturefilename,"%stext.%s.bin",mD.mTerrainPath.c_str(),tileName);
			sprintf(terrainfilename,"%sterrain.%s.ter",mD.mTerrainPath.c_str(),tileName);

			if (mTerrainGrid[y*mGridSize+x]==NULL) loaded = false;
			else loaded = true;
			if (verbose) Con::printf("terrain %d %d loaded = %d distance %f kLong %f kLat %f ",x,y,loaded,tileDistance,kLong,kLat);
			//Now, to the meat of the issue:
			if ((tileDistance<mD.mTileLoadRadius)&&(loaded==false))
			{
				//HERE: okay, the difference is that first, you need to check your mTerrains.
				for (int c=0;c<mTerrains.size();c++)
				{//Could have based this off tilename comparison, but would rather do it with numbers.
					if ((fabs(mTerrains[c]->mLongitude-kLong)<0.0001)&&(fabs(mTerrains[c]->mLatitude-kLat)<0.0001))
					{//(Even though the floating point error is annoying.)
						if (verbose) Con::printf("found terrain already loaded! %f %f",kLong,kLat);
						loaded=true;
						mTerrainGrid[y*mGridSize+x]=mTerrains[c];
					}

				}
				if (loaded==false)
				{//Here, let's check for the bin file existing first, and if not handle request here, don't call.
					if ((checkFileExists(terrainfilename)) || 
						((checkFileExists(heightfilename))&&(checkFileExists(texturefilename))))
						mTerrainGrid[y*mGridSize+x] = addTerrainBlock(kLong,kLat);
					else if (mUseDataSource)//okay, now we need to make a call to worldDataSource. We should be able to safely assume 
					{	//we're ready for packets and have already sent our init requests.
						mWorldDataSource->addTerrainRequest(kLong,kLat);
						mLoadState = 3;//waiting for terrain.
					}
				}
				
				//loadTerrainData kData;
				//kData.startLongitude = kLong;
				//kData.startLatitude = kLat;
				//kData.loadPriority = 1.0 / tileDistance;
				//mLoadTiles.last() = kData;

			} else if ((tileDistance>mD.mTileDropRadius)&&(loaded==true)) {
				dropTerrainBlock(kLong,kLat);
				if (verbose) Con::printf("drop this terrain block: %d %d",x,y);
			}
			
			if ((x==gridMidpoint)&&(y==gridMidpoint))
				mTerrain = mTerrainGrid[y*mGridSize+x];
		}
	}
	//HERE: 
	for (int c=0;c<mTerrains.size();c++)
	{
		if ((mTerrains[c]->mLongitude<startLong)||
			(mTerrains[c]->mLongitude>=(startLong+(mGridSize*mD.mTileWidthLongitude)))||
			(mTerrains[c]->mLatitude<startLat)||
			(mTerrains[c]->mLatitude>=(startLat+(mGridSize*mD.mTileWidthLatitude))))
		{
			//Con::printf("REMOVING A TERRAIN! %f %f",mTerrains[c]->mLongitude,mTerrains[c]->mLatitude);
			dropTerrainBlock(c);
		}
	}
	//mLoadedTileGrid = true;
	if (mLoadState==2)//Meaning we didn't set it to 3, ie waiting for data.
	{
		Con::printf("TerrainPager done loading tiles, entering checkTile loop.");
		mLoadState=10;
	} 
	else if (mLoadState==3)//If we did set it to three, meaning we requested data, then make sure
	{ // right now that we have a lockfile, so we don't think we're done already on the next tick.
		if (mWorldDataSource->checkTerrainLock()==false)
			mWorldDataSource->makeTerrainLock();
	}
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
			sprintf(tileName,getTileName(kLong,kLat));
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
					mWorldDataSource->addTerrainRequest(kLong,kLat);
					mLoadState = 3;//waiting for terrain.
				}
				//loadTerrainData kData;
				//kData.startLongitude = kLong;
				//kData.startLatitude = kLat;
				//kData.loadPriority = 1.0 / tileDistance;
				//mLoadTiles.last() = kData;
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

void TerrainPager::reloadSkybox()
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
	skyboxes[0] = mD.mSkyboxPath + String("skybox3_up.png");//mTerrainPath
	//skyboxes[0] = mTerrainPath + "SouthernWillamette.jpg";
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
	U32 skyOffset = 20;//Amount we raise all side panels (to compensate for camera height?)
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
	//Con::executef(this,"UpdateSkybox");//Hmm, it appears there is a delay, it takes the system longer
	//to write the actual files than it does to get out of this function. Might have to schedule the
	//update for some ticks in the future.
	//WS_SkyboxCubemap.updateFaces();
}

void TerrainPager::updateSkyboxConsole()
{
	Con::executef(this,"UpdateSkybox");
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

DefineConsoleMethod(TerrainPager, callUpdateSkybox, void, (), , "" )
{
	Con::printf("calling update skybox");
	object->updateSkyboxConsole();
}
DefineConsoleMethod(TerrainPager, reloadSkybox, void, (), , "" )
{
	Con::printf("calling reload skybox");
	object->reloadSkybox();
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
	object->findClientTile();
	return object->getTileName(object->mTileStartLongitude,object->mTileStartLatitude);
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
		object->mWorldDataSource->openListenSocket();
	return;
}
DefineConsoleMethod( TerrainPager, connectListenSocket, void, (), , "" )
{
	if (object->mUseDataSource)
		object->mWorldDataSource->connectListenSocket();
	return;
}
DefineConsoleMethod( TerrainPager, listenForPacket, void, (), , "" )
{
	if (object->mUseDataSource)
		object->mWorldDataSource->listenForPacket();
	return;
}

DefineConsoleMethod( TerrainPager, connectSendSocket, void, (), , "" )
{
	if (object->mUseDataSource)
		object->mWorldDataSource->connectSendSocket();
	return;
}

DefineConsoleMethod( TerrainPager, sendPacket, void, (), , "" )
{
	if (object->mUseDataSource)
		object->mWorldDataSource->sendPacket();
	return;
}

DefineConsoleMethod( TerrainPager, dropAll, void, (), , "" )
{
	object->dropAllTerrains();
	return;
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
	int numReturnControls = 1;

	outArray[0] = (float)numReturnControls;//May become relevant if we start returning more complex data.
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
		} else if (mWorldDataSource->mReadyForRequests && mSentInitRequests) {
			Con::printf("Can't find height file %s, adding request to world data source. ready %d",heightfilename,mWorldDataSource->mReadyForRequests);
			if (mSentTerrainRequest==false)//(checkTerrainLock()==false) 
			{
				Con::printf("requesting terrain tile: %f %f",startLong,startLat);
				//mWorldDataSource->addTerrainRequest(startLong,startLat);
				mWorldDataSource->addSkyboxRequest(startLong,startLat,mD.mClientPosLongitude,mD.mClientPosLatitude);
				mSentTerrainRequest = true;
			} else Con::printf("can't request terrain, source busy");
			return false;//Don't add anything until we can open the file. But first check for lockfile?
		} else {
			Con::printf("Can't find height file %s, not ready for requests.",heightfilename);
			return false;//else Con::printf("Can't add terrain block, readyForRequests %d",mWorldDataSource->mReadyForRequests);
		}
	}
	
*/