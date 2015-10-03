//-----------------------------------------------------------------------------
// Copyright (c) 2015 Chris Calef
//-----------------------------------------------------------------------------

#ifndef _WORLDDATASOURCE_H_
#define _WORLDDATASOURCE_H_

#include "sim/dataSource/dataSource.h"
#include "terrain/terrPager.h"

#define OPCODE_INIT_TERRAIN		2
#define OPCODE_TERRAIN			3
#define OPCODE_INIT_SKYBOX		4
#define OPCODE_SKYBOX			5

/// Base class for various kinds of data sources, first one being worldDataSource, for terrain, sky, weather and map information.
class worldDataSource : public dataSource 
{
   public:
	   terrainPagerData mD;

	   bool mFullRebuild;

	   bool mTerrainDone;
	   bool mSkyboxDone;

	   unsigned int mSkyboxStage;

	   worldDataSource(bool listening=false,terrainPagerData *data=NULL);
	   ~worldDataSource();
	   
	   void tick();

	   void listenForPacket();
	   void readPacket();

	   void addInitTerrainRequest(terrainPagerData *data,const char *path);
	   void addTerrainRequest(float playerLong,float playerLat);
	   void addInitSkyboxRequest(unsigned int skyboxRes,int cacheMode,const char *path);
	   void addSkyboxRequest(float tileLong,float tileLat,float playerLong,float playerLat,float playerAlt);
	   //void skyboxSocketDraw();//source/server side only, ie flightgear

};

#endif // _WORLDDATASOURCE_H_
