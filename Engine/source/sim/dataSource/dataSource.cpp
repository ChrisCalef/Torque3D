//-----------------------------------------------------------------------------
// Copyright (c) 2015 Chris Calef
//-----------------------------------------------------------------------------

#include "sim/dataSource/dataSource.h"

#include "console/consoleTypes.h"//Torque specific, should do #ifdef TORQUE or something

dataSource::dataSource(bool server)
{
	mPacketSize = 1024;
	mSocketTimeout = 0;
	mPort = 9934;
	mCurrentTick = 0;
	mLastSendTick = 0;
	mLastSendTimeMS = 0;
	mTickInterval = 1;//45;
	mTalkInterval = 20;
	mStartDelay = 50;
	mPacketCount = 0;
	mMaxPackets = 20;
	mSendControls = 0;
	mReturnByteCounter = 0;
	mSendByteCounter = 0;
	sprintf(mSourceIP,"127.0.0.1");
	mListenSockfd = INVALID_SOCKET;
	mWorkSockfd = INVALID_SOCKET;
	mReturnBuffer = NULL;
	mSendBuffer = NULL;
	mStringBuffer = NULL;
	mReadyForRequests = false;
	mServer = false;
	mListening = false;	
	mAlternating = true;
	mConnectionEstablished = false;
	if (server)
	{
		mServer = true;
		mListening = true;
	}
	Con::printf("New data source object!!! Source IP: %s listening %d\n",mSourceIP,server);
}

dataSource::~dataSource()
{
	disconnectSockets();
}

////////////////////////////////////////////////////////////////////////////////

void dataSource::tick()
{	
	Con::printf("datasource tick %d",mCurrentTick);
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

/////////////////////////////////////////////////////////////////////////////////

void dataSource::openListenSocket()
{
	struct sockaddr_in source_addr;
	//int n;

	Con::printf("connecting listen socket\n");
	mListenSockfd = socket(AF_INET, SOCK_STREAM,IPPROTO_TCP);
	if (mListenSockfd < 0) {
		Con::printf("ERROR opening listen socket \n");
		return;
	} else {
		Con::printf("SUCCESS opening listen socket \n");
	}
	
	BOOL bOptVal = TRUE;
	//// lose the pesky "Address already in use" error message  - only for listen socket?
	if (setsockopt(mListenSockfd,SOL_SOCKET,SO_REUSEADDR,(char *) &bOptVal,sizeof(BOOL)) == -1) {
		Con::printf("FAILED to set socket options\n");
		return;
	} 
		
	u_long iMode=1;
	ioctlsocket(mListenSockfd,FIONBIO,&iMode);//Make it a non-blocking socket.

	ZeroMemory((char *) &source_addr, sizeof(source_addr));
    source_addr.sin_family = AF_INET;
	source_addr.sin_addr.s_addr = inet_addr( mSourceIP );
    source_addr.sin_port = htons(mPort);

	if (bind(mListenSockfd, (struct sockaddr *) &source_addr,
		sizeof(source_addr)) < 0) 
		Con::printf("ERROR on binding mListenSockfd\n");
	else Con::printf("SUCCESS binding mListenSockfd\n");

}

void dataSource::connectListenSocket()
{
	int n;
	
	n = listen(mListenSockfd,10);
	if (n == SOCKET_ERROR)
	{
		Con::printf(" listen socket error!\n");
		return;
	}
	
	mWorkSockfd = accept(mListenSockfd,NULL,NULL);
	
	if (mWorkSockfd == INVALID_SOCKET)
	{
		Con::printf(".");
		return;
	} else {
		Con::printf("\nlisten accept succeeded!\n");
		mReturnBuffer = new char[mPacketSize];
		memset((void *)(mReturnBuffer),NULL,mPacketSize);	
		mReturnBuffer = new char[mPacketSize];
		memset((void *)(mSendBuffer),NULL,mPacketSize);
		mStringBuffer = new char[mPacketSize];
		memset((void *)(mStringBuffer),NULL,mPacketSize);	
	}
}

void dataSource::listenForPacket()
{
	Con::printf("dataSource listenForPacket");
	mPacketCount = 0;

	int n = recv(mWorkSockfd,mReturnBuffer,mPacketSize,0);
	if (n<=0) {
		Con::printf(".");
		return;
	}

	readPacket();
}

	//for (int j=0;j<mPacketCount;j++)
	//{
	//	n = recv(mWorkSockfd,mReturnBuffer,mPacketSize,0);
	//	if (n<=0) j--;
	//	else {
	//		readPacket();
	//	}
	//}


void dataSource::readPacket()
{
	short opcode,controlCount;//,packetCount;

	controlCount = readShort();
	for (short i=0;i<controlCount;i++)
	{		
		opcode = readShort();
		if (opcode==1) {   ////  keep contact, but no request /////////////////////////
			int tick = readInt();				
			//if (mServer) Con::printf("dataSource clientTick = %d, my tick %d",tick,mCurrentTick);
			//else Con::printf("dataSource serverTick = %d, my tick %d",tick,mCurrentTick);
		}// else if (opcode==22) { // send us some number of packets after this one
		//	packetCount = readShort();
		//	if ((packetCount>0)&&(packetCount<=mMaxPackets))
		//		mPacketCount = packetCount;
		//}
	}
	
	clearReturnPacket();
}

/////////////////////////////////////////////////////////////////////////////////

void dataSource::connectSendSocket()
{
	struct sockaddr_in source_addr;
	
	mReturnBuffer = new char[mPacketSize];	
	mSendBuffer = new char[mPacketSize];		
	mStringBuffer = new char[mPacketSize];
	memset((void *)(mReturnBuffer),NULL,mPacketSize);
	memset((void *)(mSendBuffer),NULL,mPacketSize);
	memset((void *)(mStringBuffer),NULL,mPacketSize);		

	mReadyForRequests = true;

	addBaseRequest();

	mWorkSockfd = socket(AF_INET, SOCK_STREAM,IPPROTO_TCP);
	if (mWorkSockfd < 0) {
		Con::printf("ERROR opening send socket\n");
		return;
	}
	
	u_long iMode=1;
	ioctlsocket(mWorkSockfd,FIONBIO,&iMode);//Make it a non-blocking socket.

	ZeroMemory((char *) &source_addr, sizeof(source_addr));
    source_addr.sin_family = AF_INET;
	source_addr.sin_addr.s_addr = inet_addr( mSourceIP );
    source_addr.sin_port = htons(mPort);
	
	int connectCode = connect(mWorkSockfd,(struct sockaddr *) &source_addr,sizeof(source_addr));
	if (connectCode < 0) 
	{
        Con::printf("dataSource: ERROR connecting send socket: %d\n",connectCode);
		return;
	} else {		
        Con::printf("dataSource: SUCCESS connecting send socket: %d\n",connectCode);
	}
}

void dataSource::sendPacket()
{ 
	memset((void *)(mStringBuffer),NULL,mPacketSize);	
	memcpy((void*)mStringBuffer,reinterpret_cast<void*>(&mSendControls),sizeof(short));
	memcpy((void*)&mStringBuffer[sizeof(short)],(void*)mSendBuffer,mPacketSize-sizeof(short));
	send(mWorkSockfd,mStringBuffer,mPacketSize,0);	
	mLastSendTick = mCurrentTick;
	
	clearSendPacket();
}

void dataSource::clearSendPacket()
{	
	memset((void *)(mSendBuffer),NULL,mPacketSize);
	memset((void *)(mStringBuffer),NULL,mPacketSize);	

	mSendControls = 0;
	mSendByteCounter = 0;
}

void dataSource::clearReturnPacket()
{
	memset((void *)(mReturnBuffer),NULL,mPacketSize);	
	memset((void *)(mStringBuffer),NULL,mPacketSize);	

	mReturnByteCounter = 0;	
}
/////////////////////////////////////////////////////////////////////////////////

void dataSource::trySockets()
{
	if (mServer)
	{
		if (mListenSockfd == INVALID_SOCKET) {
			openListenSocket();
		} else if (mWorkSockfd == INVALID_SOCKET) {
			connectListenSocket();
		} else {
			listenForPacket();
			mConnectionEstablished = true;
		}
	} else {
		if (mWorkSockfd == INVALID_SOCKET) {
			connectSendSocket();
		} else {
			sendPacket();
			mConnectionEstablished = true;
		}
	}
}

void dataSource::disconnectSockets()
{
	closesocket(mListenSockfd);
	closesocket(mWorkSockfd);
}

/////////////////////////////////////////////////////////////////////////////////

void dataSource::writeShort(short value)
{
	memcpy((void*)&mSendBuffer[mSendByteCounter],reinterpret_cast<void*>(&value),sizeof(short));
	mSendByteCounter += sizeof(short);	
}

void dataSource::writeInt(int value)
{
	memcpy((void*)&mSendBuffer[mSendByteCounter],reinterpret_cast<void*>(&value),sizeof(int));
	mSendByteCounter += sizeof(int);	
}

void dataSource::writeFloat(float value)
{
	memcpy((void*)&mSendBuffer[mSendByteCounter],reinterpret_cast<void*>(&value),sizeof(float));
	mSendByteCounter += sizeof(float);	
}

void dataSource::writeDouble(double value)
{
	memcpy((void*)&mSendBuffer[mSendByteCounter],reinterpret_cast<void*>(&value),sizeof(double));
	mSendByteCounter += sizeof(double);	
}

void dataSource::writeString(char *content)
{
	int length = strlen(content);
	writeInt(length);
	strncpy(&mSendBuffer[mSendByteCounter],content,length);
	mSendByteCounter += length;
}

//void dataSource::writePointer(void *pointer) //Maybe, someday? using boost, shared pointer or shared memory? 
//{  //Or can it be done in a more brute force way with global scale pointers? 
//}

//////////////////////////////////////////////

short dataSource::readShort()
{
	char bytes[sizeof(short)];
	for (int i=0;i<sizeof(short);i++) bytes[i] = mReturnBuffer[mReturnByteCounter+i];
	mReturnByteCounter+=sizeof(short);
	short *ptr = reinterpret_cast<short*>(bytes);
	return *ptr;	
}

int dataSource::readInt()
{
	char bytes[sizeof(int)];
	for (int i=0;i<sizeof(int);i++) bytes[i] = mReturnBuffer[mReturnByteCounter+i];
	mReturnByteCounter+=sizeof(int);
	int *ptr = reinterpret_cast<int*>(bytes);
	return *ptr;
}

float dataSource::readFloat()
{
	char bytes[sizeof(float)];
	for (int i=0;i<sizeof(float);i++) bytes[i] = mReturnBuffer[mReturnByteCounter+i];
	mReturnByteCounter+=sizeof(float);
	float *ptr = reinterpret_cast<float*>(bytes);
	return *ptr;
}

double dataSource::readDouble()
{
	char bytes[sizeof(double)];
	for (int i=0;i<sizeof(double);i++) bytes[i] = mReturnBuffer[mReturnByteCounter+i];
	mReturnByteCounter+=sizeof(double);
	double *ptr = reinterpret_cast<double*>(bytes);
	return *ptr;
}

char *dataSource::readString()
{
	int length = readInt();
	strncpy(mStringBuffer,&mReturnBuffer[mReturnByteCounter],length);
	mReturnByteCounter += length;
	return mStringBuffer;
}

void dataSource::clearString()
{
	if (mStringBuffer!=NULL)	
		memset((void *)(mStringBuffer),NULL,mPacketSize);
}

//void dataSource::readPointer()
//{
//}

/////////////////////////////////////////////////////////////////////////////////

void dataSource::addBaseRequest()
{	
	short opcode = 1;//base request
	mSendControls++;//Increment this every time you add a control.
	writeShort(opcode);
	writeInt(mCurrentTick);//For a baseRequest, do nothing but send a tick value to make sure there's a connection.
}

void dataSource::handleBaseRequest()
{	
	int tick = readInt();				
	//if (mServer) Con::printf("dataSource clientTick = %d, my tick %d",tick,mCurrentTick);
	//else Con::printf("dataSource serverTick = %d, my tick %d",tick,mCurrentTick);
}
