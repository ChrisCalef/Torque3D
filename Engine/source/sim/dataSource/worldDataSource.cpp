//-----------------------------------------------------------------------------
// Copyright (c) 2015 Chris Calef
//-----------------------------------------------------------------------------

#include "sim/dataSource/worldDataSource.h"

#include "console/consoleTypes.h"//Torque specific, should do #ifdef TORQUE or something

#include "core/stream/fileStream.h"

worldDataSource::worldDataSource(bool listening,terrainPagerData *data)
{
	mPacketSize = 1024;
	mSocketTimeout = 0;
	mPort = 9934;
	mCurrentTick = 0;
	mSkyboxStage = 0;
	mLastSendTick = 0;
	mTickInterval = 30;
	mTalkInterval = 20;
	mStartDelay = 50;
	sprintf(mSourceIP,"127.0.0.1");
	
	if (listening)
	{
		mListening = true;
		mSending = false;
	} else {
		mListening = false;
		mSending = true;
	}

	if (data)
	{
		mD.mTerrainPath = data->mTerrainPath;
		mD.mTerrainLockfile = data->mTerrainLockfile;
		mD.mSkyboxPath = data->mSkyboxPath;
		mD.mSkyboxLockfile = data->mSkyboxLockfile;
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

	Con::printf("New world data source object!!! Source IP: %s\n",mSourceIP);
}

worldDataSource::~worldDataSource()
{
	
	//delete [] mReturnBuffer;
	//delete [] mFloatBuffer;
	//deletePacket();
	disconnectSockets();
}

void worldDataSource::tick()
{	
    if ((mCurrentTick++ % mTickInterval == 0)&&
		(mCurrentTick > mStartDelay)) 
	{ 
		if (mListening)
		{
			if (mListenSockfd == INVALID_SOCKET)
				openListenSocket();
			else if (mWorkSockfd == INVALID_SOCKET)
				connectListenSocket();
			else
				listenForPacket();
		} else if (mSending) {
			if (mWorkSockfd == INVALID_SOCKET)
			{
				connectSendSocket();
			} else {
				sendPacket();
			}
		}
	} else if (mCurrentTick % mTalkInterval == 0) {
		//Con::printf("\nworldDataSource current tick: %d\n",mCurrentTick);
	}
}

void worldDataSource::listenForPacket()
{
	//char *buffer;
	//buffer = new char[mPacketSize];//,buffer2[packetsize];//I do need a number for my input buffer size though.
	
	struct sockaddr_in client_addr;
	int n=0;
	int i=0;
	mByteCounter = 0;
	//char floatBytes[256];//sizeof(float)?
	float OPCODE,hostFrame;
	int num_args,pathLength;

	//on dataSource side, do pre-receive logic here like drawing skyboxes, if necessary

	n = recv(mWorkSockfd,mReturnBuffer,mPacketSize,0);
	if (n>0) {
		Con::printf("\nwork socket received %d bytes!\n",n);
	} else {
		Con::printf(".");
		return;
	}

	//char *bytes = &(buffer[0]);
	//for (int i=0;i<sizeof(float);i++) floatBytes[i] = bytes[i];
	//mByteCounter += sizeof(float);
	
	/*
	float *argArray
	mFloatBuffer = getFloatBytes(1);
	float controlCount = mFloatBuffer[0];

	//HERE: might pay to split these out into separate functions, for readability.
	Con::printf("control count = %d\n",(int)controlCount);
	for (int i=0;i<(int)controlCount;i++)
	{
		mFloatBuffer = getFloatBytes(1);
		OPCODE = mFloatBuffer[0];
		printf("dealing with control type: %d\n",(int)OPCODE);

		if (OPCODE==1.0) {//Basic contact, no request.
			mFloatBuffer = getFloatBytes(1);
			hostFrame = mFloatBuffer[0];
			Con::printf("Host frame: %d\n",(int)hostFrame);
		} 
	}*/

	//delete [] buffer;
}

void worldDataSource::connectSendSocket()
{
	Con::printf("worldDataSource::connectSendSocket\n");
	struct sockaddr_in source_addr;
	
	mReturnBuffer = new char[mPacketSize];
	//mFloatBuffer = new float[mPacketSize/sizeof(float)];
	
	mReadyForRequests = true;

	addBaseRequest();//Do this just so there's always something to send...

	mWorkSockfd = socket(AF_INET, SOCK_STREAM,IPPROTO_TCP);
	if (mWorkSockfd < 0) {
		Con::printf("ERROR opening send socket\n");
		return;
	} else { Con::printf("SUCCESS opening send socket\n"); }
	
	u_long iMode=1;
	ioctlsocket(mWorkSockfd,FIONBIO,&iMode);//Make it a non-blocking socket.

	ZeroMemory((char *) &source_addr, sizeof(source_addr));
    source_addr.sin_family = AF_INET;
	source_addr.sin_addr.s_addr = inet_addr( mSourceIP );
    source_addr.sin_port = htons(mPort);
	
	int result = connect(mWorkSockfd,(struct sockaddr *) &source_addr,sizeof(source_addr));
	if ( result < 0) 
	{
        Con::printf("worldDataSource: ERROR connecting send socket, errno %d error %s\n",errno,strerror(errno));
		return;
	}
	
	Con::printf("world data source connection successful! Port %d\n",mPort);
}

void worldDataSource::sendPacket()
{
	//char *returnBuffer;
	//mReturnBuffer = new char[mPacketSize];
	//float *outArray;
	//mFloatBuffer = new float[mPacketSize/sizeof(float)];
	//mNumReturnControls = 1;

	//mFloatBuffer[0] = (float)mNumReturnControls;//May become relevant if we start returning more complex data.
	//mFloatBuffer[1] = 103.0f;//CIGI code for StartOfFrame
	//mFloatBuffer[2] = (float)mCurrentTick;//Mandatory frame argument, so we know where we're at.
	
	//mReturnBuffer = reinterpret_cast<char*>(mFloatBuffer);

	int n = send(mWorkSockfd,mReturnBuffer,mPacketSize,0);
	Con::printf("worldDataSource sent packet, n=%d",n);
	
	clearPacket();

	addBaseRequest();//Do this just so there's always something to send... unless/until we move to not send
	//anything at all until there is a request waiting.
	

	//delete [] mReturnBuffer;
	//delete [] mFloatBuffer;

}

void worldDataSource::clearPacket()
{
	memset((void *)(mReturnBuffer),NULL,1024);	
	mReturnControls = 0;
	mByteCounter = 0;	
}

//These functions can be called from the terrainPager, just make sure all the data variables are set first.
void worldDataSource::addInitTerrainRequest(terrainPagerData *data,const char *path)
{
	float OPCODE = 101.0f;
	mReturnControls++;//Increment mNumReturnControls every time you add a control.
	/*
	mFloatBuffer[0] = (float)mNumReturnControls;

	mByteCounter++;
	mFloatBuffer[mByteCounter] = OPCODE;
	//num args = 6
	mByteCounter++;
	mFloatBuffer[mByteCounter] = data->mTileWidth;
	mByteCounter++;
	mFloatBuffer[mByteCounter] = (float)(data->mHeightmapRes);
	mByteCounter++;
	mFloatBuffer[mByteCounter] = (float)(data->mTextureRes);
	mByteCounter++;
	mFloatBuffer[mByteCounter] = data->mMapCenterLongitude;
	mByteCounter++;
	mFloatBuffer[mByteCounter] = data->mMapCenterLatitude;
	mByteCounter++;
	mFloatBuffer[mByteCounter] = (float)(strlen(path));
	//And then send the actual path...
	*/
	Con::printf("adding init terrain request, numControls %d, numReturnBytes %d\n",mReturnControls,mByteCounter);
}

//void worldDataSource::addTerrainRequest(loadTerrainData *data)
void worldDataSource::addTerrainRequest(float playerLong,float playerLat)
{
	float OPCODE = 102.0f;
	mReturnControls++;//Increment mNumReturnControls every time you add a control.
	/*
	mFloatBuffer[0] = (float)mNumReturnControls;

	mByteCounter++;
	mFloatBuffer[mByteCounter] = OPCODE;

	mByteCounter++;
	mFloatBuffer[mByteCounter] = playerLong;//data->startLongitude
	mByteCounter++;
	mFloatBuffer[mByteCounter] = playerLat;//data->startLatitude
	//mByteCounter++;
	//mFloatBuffer[mByteCounter] = data->loadPriority;

	Con::printf("adding terrain request, numControls %d, long %f lat %f\n",mNumReturnControls,playerLong,playerLat);
		*/
}

void worldDataSource::addInitSkyboxRequest(unsigned int skyboxRes,int cacheMode,const char *path)
{
	float OPCODE = 201.0f;
	mReturnControls++;//Increment mNumReturnControls every time you add a control.

	/*
	mFloatBuffer[0] = (float)mNumReturnControls;

	mByteCounter++;
	mFloatBuffer[mByteCounter] = OPCODE;

	mByteCounter++;
	mFloatBuffer[mByteCounter] = (float)skyboxRes;
	mByteCounter++;
	mFloatBuffer[mByteCounter] = (float)cacheMode;
	mByteCounter++;
	mFloatBuffer[mByteCounter] = (float)(strlen(path));

	Con::printf("adding init skybox request, numControls %d, numReturnBytes %d\n",mNumReturnControls,mByteCounter);
		*/
}

void worldDataSource::addSkyboxRequest(float tileLong,float tileLat,float playerLong,float playerLat,float playerAlt)
{
	float OPCODE = 202.0f;
	mReturnControls++;//Increment mNumReturnControls every time you add a control.
	/*
	mFloatBuffer[0] = (float)mNumReturnControls;

	mByteCounter++;
	mFloatBuffer[mByteCounter] = OPCODE;

	mByteCounter++;
	mFloatBuffer[mByteCounter] = tileLong;
	mByteCounter++;
	mFloatBuffer[mByteCounter] = tileLat;
	mByteCounter++;
	mFloatBuffer[mByteCounter] = playerLong;
	mByteCounter++;
	mFloatBuffer[mByteCounter] = playerLat;
	mByteCounter++;
	mFloatBuffer[mByteCounter] = playerAlt;

	Con::printf("adding skybox request, numControls %d, tileLong %f tileLat %f\n",mNumReturnControls,tileLong,tileLat);
		*/
}

void worldDataSource::skyboxSocketDraw()
{
	//(This is only on the dataSource side.)
}

void worldDataSource::makeTerrainLock()
{
	FILE *fs;
	if (fs = fopen(mD.mTerrainLockfile.c_str(),"w"))
	{
		fprintf(fs,"%d",1);//A byte, so we can open for read.
	}
	fclose(fs);
}

void worldDataSource::removeTerrainLock()
{
	remove(mD.mTerrainLockfile.c_str());
}

bool worldDataSource::checkTerrainLock()
{
	FileStream fs;
	if (fs.open(mD.mTerrainLockfile.c_str(),Torque::FS::File::Read))
	{ 
		fs.close();
		return true;
	} else return false;
}

void worldDataSource::makeSkyboxLock()
{
	FILE *fs;
	if (fs = fopen(mD.mSkyboxLockfile.c_str(),"w"))
	{
		fprintf(fs,"%d",1);//A byte, so we can open for read.
	}
	fclose(fs);
}

void worldDataSource::removeSkyboxLock()
{
	Con::printf("removing skybox lockfile: %s",mD.mSkyboxLockfile.c_str());
	remove(mD.mSkyboxLockfile.c_str());
}

bool worldDataSource::checkSkyboxLock()
{
	FileStream fs;
	Con::printf("checking skybox lock file: %s  %s",mD.mSkyboxLockfile.c_str());
	if (fs.open(mD.mSkyboxLockfile.c_str(),Torque::FS::File::Read))
	{ 
		fs.close();
		return true;
	} else return false;
}


