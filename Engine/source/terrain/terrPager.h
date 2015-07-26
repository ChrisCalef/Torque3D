//-----------------------------------------------------------------------------
// Copyright (c) 2015 Chris Calef
//-----------------------------------------------------------------------------

#ifndef _TERRPAGER_H_
#define _TERRPAGER_H_

#ifndef _SIMBASE_H_
#include "console/simBase.h"
#endif
#ifndef _TERRDATA_H_
#include "terrain/terrData.h"
#endif
#ifndef _ITICKABLE_H_
#include "core/iTickable.h"
#endif

#include "console/SQLiteObject.h"
#include "console/SimXMLDocument.h"

class worldDataSource;
class dataSource;

//This is meant to be a plain portable C++ encapsulation of all data necessary to run the terrainPager, so
//it can be included on the data source (flightgear) side.
struct terrainPagerData
{	
	String mSkyboxPath;
	String mTerrainPath;
	String mSkyboxLockfile;//Keep in WorldDataSource? Or not?
	String mTerrainLockfile;
	String mTerrainHeightsBinFile;//One tile's worth of heights
	String mTerrainTexturesBinFile;//and textures
	String mTerrainTreesBinFile;//and trees.
	String skybox_files[5];//Filenames for the five skybox textures, because we 
	//are going to have to flip and rotate them before they can be used in Torque.
	
	float mMapCenterLongitude;
	float mMapCenterLatitude;
	
	float mMetersPerDegreeLongitude;
	float mDegreesPerMeterLongitude;
	float mMetersPerDegreeLatitude;
	float mDegreesPerMeterLatitude;
	
	float mClientPosLongitude;
	float mClientPosLatitude;
	float mClientPosAltitude;

	float mTileLoadRadius;
	float mTileDropRadius;

	float mTileWidth;
	float mTileWidthLongitude;
	float mTileWidthLatitude;
	float mSquareSize;

	unsigned int mGridSize;	
	unsigned int mHeightmapRes;
	unsigned int mTextureRes;
	unsigned int mLightmapRes;
	unsigned int mSkyboxRes;	
	unsigned int mSkyboxCacheMode;
};

struct loadTerrainData
{
	float startLongitude;
	float startLatitude;
	float loadPriority;
};

/// The TerrainPager organizes large numbers of stored terrain tiles and optionally skyboxes, as well
/// the capability to maintain a WorldDataSource connection to an external process for realtime data updates.
class TerrainPager : public SimObject, public virtual ITickable
{
	typedef SimObject Parent;

public:
	
	DECLARE_CONOBJECT( TerrainPager );
	
	terrainPagerData mD;//This is a standalone data struct for easy portability.

	SQLiteObject *mSQL;

	Vector <TerrainBlock *>mTerrains;//This is the short list of actually loaded terrains. 
	Vector <TerrainBlock *>mTerrainGrid;//This is the sparse array in the shape of a grid centered on player.
	Vector <loadTerrainData> mRequestTiles;//Ever changing list of tiles that need to be requested.
	TerrainBlock *mTerrain;//This is the terrain the player or camera is currently occupying. 
	Vector <String> terrain_materials;

	bool mUseDataSource;
	worldDataSource *mDataSource;
	bool mSentInitRequests;
	bool mSentTerrainRequest;
	bool mSentSkyboxRequest;
	bool mLoadedTileGrid;
	

	//Vector <String> mTileNames;//Still need this? Don't think so.	

	Point3F mClientPos;

	F32 mTileStartLongitude;//These refer to the bottom left corner of the tile
	F32 mTileStartLatitude;  //the client is currently standing on or over.

	Point3F mStartPos;

	U32 mCurrentTick;
	U32 mTickInterval;

	U32 mLastSkyboxTick;
	U32 mSkyboxTickInterval;
	U32 mSkyboxLoadDelay;

	U32 mLoadState;

	/////////////////////////////////////////////////////////////////////
	//These are the only overlapping items between terrainPager and terrainPagerData - they are here because  
	//they need to be exposed to script for the terrainPager creation block in the mission.
	F32 mMapCenterLongitude;
	F32 mMapCenterLatitude;
	F32 mTileLoadRadius;//At a future time these could be weighted by axes, to get an eliptical area
	F32 mTileDropRadius;//instead of a circle, but definitely not necessary for first pass.
	U32 mSkyboxRes;
	///////////////////////
	
	U32 mGridSize;//Now, this is no longer a solid 3x3 or 5x5 grid that we have loaded at all times,
					//but rather a 3x3, 5x5, 7x7 etc grid that we potentially *could* load tiles from. Actual
					//tiles loaded will be on a one by one basis as resources allow.	
	
	F32 mLastTileStartLong;//For determining when we've crossed a tile border and 
	F32 mLastTileStartLat;  //need to reset the grid.

	bool mFreeCamera;

	TerrainPager();
	virtual ~TerrainPager();
	
	bool onAdd();
	static void initPersistFields();	
	void processTick();
	void interpolateTick(F32);
	void advanceTime(F32);

	const char* getTileName(F32 longitude, F32 latitude);

	void findClientPos();
	void findClientTile();
	void loadTileNames();

	TerrainBlock *addTerrainBlock(F32 startLong,F32 startLat);

	void dropTerrainBlock(U32 index);
	void dropTerrainBlock(F32 startLong,F32 startLat);
	void dropAllTerrains();

	void loadTileGrid();
	void checkTileGrid();

	bool checkFileExists(const char*);

	void reloadSkybox();

	void updateSkyboxConsole();

	void loadOSM(const char*);
	void makeStreets();

	Point3F convertLatLongToXYZ(Point3F pos);
	Point3F convertLatLongToXYZ(double longitude,double latitude, float altitude);
	Point3F convertXYZToLatLong(Point3F pos);


};
/*
	////////////////////////////////////////////////////////////
	/////These will hopefully all be obsolete, if I can pull of the same functionality in script...//////
	Vector<TerrainBlock *> mActiveTerrains;//This is a filled vector array containing numRows*numColumns entries,
	//all of which will start as NULL pointers, and are filled by position in the grid as new terrains are added.
	Vector<bool> mLoadingTerrains;//This also sparse array of numRows*numColumns.
	Vector<TerrainBlock *> mLoadedTerrains;//This will contain only as many as are currently loaded.
	//TerrainBlock *mMegaTerrain;//For far distance only, a ten times scale terrain
	//with the local blocks cut out of it.
	TerrainBlock *mTerrains[3][3];//HERE: just hard coding this for the first pass.
	TerrainBlock *mTerrainsLayout[3][3];//If people really want 5x5 or more, fix it later.
	bool mLoadTerrains[3][3];
	//////End hopefully obsolete section... /////////////////////////////////////////////
	*/
#endif // _TERRPAGER_H_
