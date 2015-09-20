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
	   unsigned int mStartDelay;//startup delay time.

	   bool mReadyForRequests;//flag to user class (eg terrainPager) that we can start adding requests.

	   SOCKET mListenSockfd;
	   SOCKET mWorkSockfd;

	   fd_set mMasterFDS;
	   fd_set mReadFDS;

	   unsigned int mPacketSize;
	   int mSocketTimeout;

	   bool mListening;//Atually could be one variable, because this isn't really going to be  
	   bool mSending;//about sending and receiving, but only about who initially connects to whom.

	   char *mReturnBuffer;
	   char *mStringBuffer;

	   unsigned int mReturnControls;
	   unsigned int mByteCounter;

	   dataSource(bool listening=false);
	   ~dataSource();

	   void tick();
	   
	   void openListenSocket();
	   void connectListenSocket();
	   void listenForPacket();

	   void connectSendSocket();
	   void sendPacket();
	   void clearPacket();
	   
	   void disconnectSockets();	   

	   void writeShort(short);
	   void writeInt(int);
	   void writeFloat(float);
	   void writeDouble(double);
	   void writeString(char *);

	   short readShort();
	   int readInt();
	   float readFloat();
	   double readDouble();
	   char *readString();
	   
	   void addBaseRequest();
};

#endif // _DATASOURCE_H_
