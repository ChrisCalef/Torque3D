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
	   float *mFloatBuffer;

	   unsigned int mNumReturnControls;
	   unsigned int mNumReturnBytes;
	   unsigned int mByteCounter;

	   dataSource(bool listening=false);
	   ~dataSource();

	   virtual void tick();
	   
	   virtual void openListenSocket();
	   virtual void connectListenSocket();
	   virtual void listenForPacket();

	   virtual void connectSendSocket();
	   virtual void sendPacket();
	   virtual void clearPacket();
	   
	   void addBaseRequest();

	   float *getFloatBytes(unsigned int num_args);

	   void disconnectSockets();
};

#endif // _DATASOURCE_H_
