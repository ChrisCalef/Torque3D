//-----------------------------------------------------------------------------
// Copyright (c) 2015 Chris Calef
//-----------------------------------------------------------------------------

#ifndef _DATASOURCE_H_
#define _DATASOURCE_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <iostream>
#include "winsock2.h"

#define OPCODE_BASE		1

/// Base class for various kinds of data sources, first one being worldDataSource, for terrain, sky, weather and map information.
class dataSource 
{
   public:
	   char mSourceIP[16];
	   unsigned int mPort;

	   unsigned int mCurrentTick;
	   unsigned int mLastSendTick;//Last time we sent a packet.
	   unsigned int mLastSendTimeMS;//Last time we sent a packet.
	   unsigned int mTickInterval;
	   unsigned int mTalkInterval;
	   unsigned int mStartDelay;
	   unsigned int mPacketCount;
	   unsigned int mMaxPackets;
	   unsigned int mPacketSize;

	   bool mReadyForRequests;//flag to user class (eg terrainPager) that we can start adding requests.

	   SOCKET mListenSockfd;
	   SOCKET mWorkSockfd;

	   fd_set mMasterFDS;
	   fd_set mReadFDS;

	   int mSocketTimeout;
	   
	   bool mServer;
	   bool mListening;
	   bool mAlternating;
	   bool mConnectionEstablished;

	   char *mReturnBuffer;
	   char *mSendBuffer;
	   char *mStringBuffer;

	   short mSendControls;
	   short mReturnByteCounter;
	   short mSendByteCounter;

	   dataSource(bool listening=false);
	   ~dataSource();

	   void tick();
	   
	   void openListenSocket();
	   void connectListenSocket();
	   void listenForPacket();
	   void readPacket();
	   void clearReturnPacket();

	   void connectSendSocket();
	   void sendPacket();
	   void clearSendPacket();
	   
	   void trySockets();
	   void disconnectSockets();	  

	   void writeShort(short);
	   void writeInt(int);
	   void writeFloat(float);
	   void writeDouble(double);
	   void writeString(char *);
	   //void writePointer(void *);//Someday? Using boost?

	   short readShort();
	   int readInt();
	   float readFloat();
	   double readDouble();
	   char *readString();
	   //void *readPointer();
	   
	   void clearString();

	   void addBaseRequest();
	   void handleBaseRequest();
};

#endif // _DATASOURCE_H_
