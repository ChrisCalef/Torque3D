//-----------------------------------------------------------------------------
// Copyright (c) 2015 Chris Calef
//-----------------------------------------------------------------------------

#ifndef _WORLDDATASOURCE_H_
#define _WORLDDATASOURCE_H_

#include "sim/dataSource/dataSource.h"
#include "terrain/terrPager.h"

/// Base class for various kinds of data sources, first one being worldDataSource, for terrain, sky, weather and map information.
class worldDataSource : public dataSource 
{
   public:
	   terrainPagerData mD;

	   bool mFullRebuild;

	   unsigned int mSkyboxStage;

	   worldDataSource(bool listening=false,terrainPagerData *data=NULL);
	   ~worldDataSource();
	   
	   void tick();

	   void listenForPacket();
	   void connectSendSocket();
	   void sendPacket();
	   void clearPacket();

	   void addInitTerrainRequest(terrainPagerData *data,const char *path);
	   //void addTerrainRequest(loadTerrainData *ltData);
	   void addTerrainRequest(float playerLong,float playerLat);
	   void addInitSkyboxRequest(unsigned int skyboxRes,int cacheMode,const char *path);
	   void addSkyboxRequest(float tileLong,float tileLat,float playerLong,float playerLat,float playerAlt);
	   void skyboxSocketDraw();//source side only, ie flightgear

	   void makeTerrainLock();
	   void removeTerrainLock();
	   bool checkTerrainLock();

	   void makeSkyboxLock();
	   void removeSkyboxLock();
	   bool checkSkyboxLock();

};

#endif // _WORLDDATASOURCE_H_
