//-----------------------------------------------------------------------------
// Copyright (c) 2015 Chris Calef
//-----------------------------------------------------------------------------

#include "sim/dataSource/vehicleDataSource.h"

#include "console/consoleTypes.h"//Torque specific, should do #ifdef TORQUE or something

#include "core/stream/fileStream.h"


//Swap function for undoing big endian values

float float_swap(float value){
   int temp = htonl(*(unsigned int*)&value);
   return *(float*)&temp;
};

//double double_swap(double value){
  // int temp = htonll(*(unsigned __int64*)&value);
   //return *(double*)&temp;
//};

vehicleDataSource::vehicleDataSource(bool listening)
{
	mPacketSize = 1024;
	mSocketTimeout = 0;
	mPort = 9934;
	mCurrentTick = 0;
	mLastSendTick = 0;
	mTickInterval = 30;
	sprintf(mSourceIP,"127.0.0.1");
	mListening = false;
	if (listening)
		mListening = true;
	
	mTerrainPager = NULL;
	TerrainPager* pager = dynamic_cast<TerrainPager*>(Sim::findObject("TheTP"));
	if( pager )
		mTerrainPager = pager;
	
	if (mTerrainPager)
		Con::printf("New vehicle data source object!!! listening: %d terrain pager mClientPos %f %f %f\n",listening,
		mTerrainPager->mClientPos.x,mTerrainPager->mClientPos.y,mTerrainPager->mClientPos.z);
	else
		Con::printf("New vehicle data source object!!! Terrain Pager NULL");
}

vehicleDataSource::~vehicleDataSource()
{
	
}

void vehicleDataSource::openListenSocket()
{
	struct sockaddr_in source_addr,client_addr;
    int n;

    mListenSockfd = socket(AF_INET, SOCK_DGRAM,IPPROTO_UDP);
    if (mListenSockfd < 0) {
        printf("ERROR opening listen socket \n");
        return;
    } else {
        printf("SUCCESS opening listen socket \n");
    }
   
    u_long iMode=1;
    ioctlsocket(mListenSockfd,FIONBIO,&iMode);//Make it a non-blocking socket.
   
    ZeroMemory((char *) &client_addr, sizeof(client_addr));
    client_addr.sin_family = AF_INET;
    client_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    client_addr.sin_port = htons(9934);//Any port will do, just match it up to your XML file for FG.

    if (bind(mListenSockfd, (struct sockaddr *) &client_addr, sizeof(client_addr)) < 0)
        printf("ERROR on binding mListenSockfd error %d\n",WSAGetLastError());
    else
    {
        printf("SUCCESS binding mListenSockfd\n");
    }
}

void vehicleDataSource::listenForPacket()
{
	if (!mTerrainPager)
		return;

    struct sockaddr_in source_addr;
    int n=-1;

    ZeroMemory((char *) &source_addr, sizeof(source_addr));
    int addrSize = sizeof(source_addr);

    flightgear_packet buff;
    memset(&buff, 0, sizeof(buff));

    n = recvfrom(SOCKET(mListenSockfd),(char *)(&buff),sizeof(buff),0,(struct sockaddr*)&source_addr,&addrSize);
	//Con::printf("receiving bytes: %d",n);
    while (n>0) {//Do a loop here in case we have multiple packets piled up waiting.
		//Con::printf("lat %3.8f long %3.8f",buff.latitude,buff.longitude);
		if ((buff.latitude!=0.0)||(buff.longitude!=0.0)) //((float_swap(buff.latitude)!=0.0)||(float_swap(buff.longitude)!=0.0)) 
		{
			//mFGPacket.latitude = buff.latitude;//double_swap(buff.latitude);
			//mFGPacket.longitude = buff.longitude;//double_swap(buff.longitude);
			mFGPacket.latitude = buff.latitude;//float_swap(buff.latitude);
			mFGPacket.longitude = buff.longitude;//float_swap(buff.longitude);
			mFGPacket.altitude = buff.altitude * (12.0/39.37);//convert to meters
			mFGPacket.airspeed = buff.airspeed;//ntohl(buff.airspeed);
			mFGPacket.roll = buff.roll;//float_swap(buff.roll);
			mFGPacket.pitch = buff.pitch;//float_swap(buff.pitch);
			mFGPacket.heading = buff.heading;//float_swap(buff.heading);
			U32 latency =  Platform::getRealMilliseconds() - mLastSendTimeMS;      
			mLastSendTimeMS = mLastSendTimeMS = Platform::getRealMilliseconds();
			//Con::printf("lat %f long %f alt %f roll %f heading %f latency %d",mFGPacket.latitude,mFGPacket.longitude,mFGPacket.altitude,buff.roll,buff.heading,latency);

			MatrixF oriMat,pitchMat,rollMat,headMat;
			QuatF oriQuat;
			oriMat.identity();
			pitchMat = MatrixF(EulerF(-1.0 * mDegToRad(mFGPacket.pitch),0,0));
			rollMat =  MatrixF(EulerF(0,-1.0 * mDegToRad(mFGPacket.roll),0));
			headMat =  MatrixF(EulerF(0,0,mDegToRad(mFGPacket.heading)));
			oriMat = headMat;
			oriMat.mul(pitchMat);
			oriMat.mul(rollMat);

			mFGTransform = oriMat;

			Point3F curPos = Point3F((float)mFGPacket.longitude,(float)mFGPacket.latitude,mFGPacket.altitude);
			//Point3F newPos = mTerrainPager->convertLatLongToXYZ(curPos);
			Point3F newPos = mTerrainPager->convertLatLongToXYZ(mFGPacket.longitude,mFGPacket.latitude,mFGPacket.altitude);

			mFGTransform.setPosition(newPos);
		}
		n = recvfrom(SOCKET(mListenSockfd),(char *)(&buff),sizeof(buff),0,(struct sockaddr*)&source_addr,&addrSize);
	}

	return;
}

void vehicleDataSource::tick()
{
	if (mListening)
	{
		if (mListenSockfd == INVALID_SOCKET)
			openListenSocket();
		else
			listenForPacket();
	}
}

