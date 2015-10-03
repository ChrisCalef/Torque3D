//-----------------------------------------------------------------------------
// Copyright (c) 2015 Chris Calef
//-----------------------------------------------------------------------------

#include "sim/dataSource/worldDataSource.h"
#include "console/consoleTypes.h"//Torque specific, should do #ifdef TORQUE or something
#include "core/stream/fileStream.h"
#include "time.h"

worldDataSource::worldDataSource(bool server,terrainPagerData *data)
{
	mSkyboxStage = 0;

	if (data)
	{
		mD.mTerrainPath = data->mTerrainPath;
		//mD.mTerrainLockfile = data->mTerrainLockfile;
		mD.mSkyboxPath = data->mSkyboxPath;
		//mD.mSkyboxLockfile = data->mSkyboxLockfile;
		mD.mTileWidth = data->mTileWidth;
		mD.mTileWidthLongitude = data->mTileWidthLongitude;
		mD.mTileWidthLatitude = data->mTileWidthLatitude;
		mD.mHeightmapRes = data->mHeightmapRes;
		mD.mTextureRes = data->mTextureRes;
		mD.mLightmapRes = data->mLightmapRes;
		mD.mSquareSize = data->mSquareSize;
		mD.mMapCenterLongitude = data->mMapCenterLongitude;
		mD.mMapCenterLatitude = data->mMapCenterLatitude;
		mD.mTileLoadRadius = data->mTileLoadRadius;
		mD.mTileDropRadius = data->mTileDropRadius;
		mD.mGridSize = data->mGridSize;
	}
		
	if (server)
	{
		mServer = true;
		mListening = true;
	}

	mTerrainDone = false;
	mSkyboxDone = false;

	Con::printf("New world data source object!!! Source IP: %s packetSize %d mAlternating %d\n",
		mSourceIP,mPacketSize,mAlternating);
}

worldDataSource::~worldDataSource()
{
	disconnectSockets();//Is this redundant with dataSource::~dataSource() doing the same thing?
}

void worldDataSource::tick()
{
	if ((mCurrentTick++ % mTickInterval == 0)&&
		(mCurrentTick > mStartDelay)) 
	{ 
		if (mConnectionEstablished == false)
		{
			trySockets();
		} else {
			if (mListening) {
				listenForPacket();
				if (mAlternating) {
					mListening = false;
					addBaseRequest();
					if (mServer) tick();
				}
			} else {
				unsigned long currentTime =  clock();
				unsigned int latencyMS = currentTime - mLastSendTimeMS;
				mLastSendTimeMS = currentTime;
				writeInt(latencyMS);

				sendPacket();

				if (mAlternating) {
					mListening = true;
					if (!mServer) tick();
				} else 
					addBaseRequest();
			}
		}
	}
}

void worldDataSource::listenForPacket()
{ 	
	mPacketCount = 0;

	int n = recv(mWorkSockfd,mReturnBuffer,mPacketSize,0);
	if (n<=0) {
		Con::printf(".");
		clearReturnPacket();
		return;
	}

	readPacket();

	//HERE: not sure how/where to do this, but this is how you suck up mPacketCount's worth of data packets in a single tick.
	//for (int j=0;j<mPacketCount;j++)
	//{
	//	n = recv(mWorkSockfd,mReturnBuffer,mPacketSize,0);
	//	if (n<=0) j--;
	//	else {
	//		readPacket();
	//	}
	//}
}


void worldDataSource::readPacket()
{
	short opcode,controlCount;//,packetCount;

	controlCount = readShort();
	for (short i=0;i<controlCount;i++)
	{		
		opcode = readShort();
		if (opcode == OPCODE_BASE) {   ////  keep contact, but no request /////////////////////////
			//Con::printf("worldDataSource got a base request");
			handleBaseRequest();
		} else if (opcode == OPCODE_INIT_TERRAIN) { //initTerrainRequest finished
			Con::printf("initTerrainRequest finished");
			//HERE: set a variable! 
		} else if (opcode == OPCODE_TERRAIN) {//terrainRequest finished
			Con::printf("terrainRequest finished");
			mTerrainDone = true;
		} else if (opcode == OPCODE_INIT_SKYBOX) {//initSkyboxRequest finished
			Con::printf("initSkyboxRequest finished");
		} else if (opcode == OPCODE_SKYBOX) {//skybox finished
			Con::printf("skyboxRequest finished");
			mSkyboxDone = true;
		}

		//And later, "here is the skybox image data", if we don't just render it to the buffer directly or 
		//pass it as shared memory / shared pointer

		//else if (opcode==22) { // send us some number of packets after this one
		//	packetCount = readShort();
		//	if ((packetCount>0)&&(packetCount<=mMaxPackets))
		//		mPacketCount = packetCount;
		//}
	}
	
	//Might want to ditch this, or plug it in optionally, it is useful at times.
	unsigned int latency = readInt();
	//if (mServer) Con::printf("client latency: %d",latency);
	//else Con::printf("server latency: %d",latency);
	
	clearReturnPacket();
}

//These functions can be called from the terrainPager, just make sure all the data variables are set first.
void worldDataSource::addInitTerrainRequest(terrainPagerData *data,const char *path)
{
	char* localPath = new char[mPacketSize];
	strcpy(localPath,path);

	mSendControls++;//Increment mNumSendControls every time you add a control.

	writeShort(OPCODE_INIT_TERRAIN);
	writeFloat(data->mTileWidth);
	writeInt(data->mHeightmapRes);
	writeInt(data->mTextureRes);
	writeFloat(data->mMapCenterLongitude);
	writeFloat(data->mMapCenterLatitude);
	writeString(localPath);

	delete localPath;

	Con::printf("adding init terrain request, numControls %d, numSendBytes %d currentTick %d",
		mSendControls,mSendByteCounter,mCurrentTick);
}

//void worldDataSource::addTerrainRequest(loadTerrainData *data)
void worldDataSource::addTerrainRequest(float playerLong,float playerLat)
{
	mSendControls++;//Increment mNumSendControls every time you add a control.

	writeShort(OPCODE_TERRAIN);
	writeFloat(playerLong);
	writeFloat(playerLat);

	mTerrainDone = false;

	Con::printf("adding terrain request, playerLong %f, playerLat %f currentTick %d",
		playerLong,playerLat,mCurrentTick);
}

void worldDataSource::addInitSkyboxRequest(unsigned int skyboxRes,int cacheMode,const char *path)
{
	char* localPath = new char[mPacketSize];
	strcpy(localPath,path);

	mSendControls++;//Increment mNumSendControls every time you add a control.

	writeShort(OPCODE_INIT_SKYBOX);
	writeInt(skyboxRes);
	writeInt(cacheMode);
	writeString(localPath);

	delete localPath;

	Con::printf("adding init skybox request, currentTick %d",mCurrentTick);
}

void worldDataSource::addSkyboxRequest(float tileLong,float tileLat,float playerLong,float playerLat,float playerAlt)
{
	mSendControls++;//Increment mNumSendControls every time you add a control.
	
	writeShort(OPCODE_SKYBOX);
	writeFloat(tileLong);
	writeFloat(tileLat);
	writeFloat(playerLong);
	writeFloat(playerLat);
	writeFloat(playerAlt);

	mSkyboxDone = false;

	Con::printf("adding skybox request, playerLong %f playerLat %f, currentTick %d",
		playerLong,playerLat,mCurrentTick);
}

//void worldDataSource::skyboxSocketDraw()
//{	//(This is only implemented on the source/server side.)
//}
