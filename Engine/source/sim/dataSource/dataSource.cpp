//-----------------------------------------------------------------------------
// Copyright (c) 2015 Chris Calef
//-----------------------------------------------------------------------------

#include "sim/dataSource/dataSource.h"

#include "console/consoleTypes.h"//Torque specific, should do #ifdef TORQUE or something


dataSource::dataSource(bool listening)
{
	mPacketSize = 1024;
	mSocketTimeout = 0;
	mPort = 9934;
	mCurrentTick = 0;
	mLastSendTick = 0;
	mTickInterval = 45;
	mTalkInterval = 20;
	mStartDelay = 50;
	mReturnControls = 0;
	mByteCounter = 0;
	sprintf(mSourceIP,"127.0.0.1");
	mListenSockfd = INVALID_SOCKET;
	mWorkSockfd = INVALID_SOCKET;
	mReturnBuffer = NULL;
	mReadyForRequests = false;
	
	if (listening)
	{
		mListening = true;
		mSending = false;
	} else {
		mListening = false;
		mSending = true;
	}
	Con::printf("New data source object!!! Source IP: %s listening %d\n",mSourceIP,listening);
}

dataSource::~dataSource()
{
	disconnectSockets();
}


/////////////////////////////////////////////////////////////////////////////////

void dataSource::tick()
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
				connectSendSocket();
			else
				sendPacket();
		}
	}// else if (mCurrentTick % mTalkInterval == 0) {
	//	Con::printf("\ndataSource current tick: %d\n",mCurrentTick);
	//}
}


/////////////////////////////////////////////////////////////////////////////////

void dataSource::openListenSocket()
{
	struct sockaddr_in source_addr;
	int n;

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
		memset((void *)(mReturnBuffer),NULL,1024);	
		mStringBuffer = new char[mPacketSize];
		memset((void *)(mStringBuffer),NULL,1024);	
	}
}

void dataSource::listenForPacket()
{
	struct sockaddr_in client_addr;
	int n=0;
	int i=0;
	mByteCounter = 0;
	char floatBytes[256];//sizeof(float)?
	float OPCODE,hostFrame;
	int num_args;

	n = recv(mWorkSockfd,mReturnBuffer,mPacketSize,0);
	if (n>0) {
		Con::printf("\nlisten socket received %d bytes!\n",n);
	} else {
		Con::printf(".");
		return;
	}

	char *bytes = &(mReturnBuffer[0]);
	for (int i=0;i<sizeof(float);i++) floatBytes[i] = bytes[i];
	mByteCounter += sizeof(float);

	float *argArray = reinterpret_cast<float*>(floatBytes);
	float controlCount = argArray[0];

	//HERE: might pay to split these out into separate functions, for readability.
	Con::printf("control count = %d\n",(int)controlCount);
	for (int i=0;i<(int)controlCount;i++)
	{
		for (int i=0;i<sizeof(float);i++) floatBytes[i] = bytes[mByteCounter+i];
		mByteCounter += sizeof(float);
		argArray = reinterpret_cast<float*>(floatBytes);
		OPCODE = argArray[0];
		Con::printf("dealing with control type: %d\n",(int)OPCODE);
		if (OPCODE==101.0) {//IG Control
			num_args = 1;
			for (int i=0;i<sizeof(float)*num_args;i++) floatBytes[i] = bytes[mByteCounter+i];
			mByteCounter += sizeof(float)*num_args;
			argArray = reinterpret_cast<float*>(floatBytes);

			hostFrame = argArray[0];
			Con::printf("Host frame: %d\n",(int)hostFrame);
		} 
	}
}
/////////////////////////////////////////////////////////////////////////////////

void dataSource::connectSendSocket()
{
	struct sockaddr_in source_addr;
	
	mReturnBuffer = new char[mPacketSize];
	
	memset((void *)(mReturnBuffer),NULL,mPacketSize);	

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
	send(mWorkSockfd,mReturnBuffer,mPacketSize,0);	
	clearPacket();
}

void dataSource::clearPacket()
{
	memset((void *)(mReturnBuffer),NULL,1024);	
	mReturnControls = 0;
	mByteCounter = 0;	
}

/////////////////////////////////////////////////////////////////////////////////

void dataSource::disconnectSockets()
{
	closesocket(mListenSockfd);
	closesocket(mWorkSockfd);
}

/////////////////////////////////////////////////////////////////////////////////

void dataSource::writeShort(short value)
{
	memcpy((void*)&mReturnBuffer[mByteCounter],reinterpret_cast<void*>(&value),sizeof(short));
	mByteCounter += sizeof(short);	
}

void dataSource::writeInt(int value)
{
	memcpy((void*)&mReturnBuffer[mByteCounter],reinterpret_cast<void*>(&value),sizeof(int));
	mByteCounter += sizeof(int);	
}

void dataSource::writeFloat(float value)
{
	memcpy((void*)&mReturnBuffer[mByteCounter],reinterpret_cast<void*>(&value),sizeof(float));
	mByteCounter += sizeof(float);	
}

void dataSource::writeDouble(double value)
{
	memcpy((void*)&mReturnBuffer[mByteCounter],reinterpret_cast<void*>(&value),sizeof(double));
	mByteCounter += sizeof(double);	
}

void dataSource::writeString(char *content)
{
	int length = strlen(content);
	writeInt(length);
	strncpy(&mReturnBuffer[mByteCounter],content,length);
	mByteCounter += length;
}

//////////////////////////////////////////////

short dataSource::readShort()
{
	char bytes[sizeof(short)];
	for (int i=0;i<sizeof(short);i++) bytes[i] = mReturnBuffer[mByteCounter+i];
	mByteCounter+=sizeof(short);
	short *ptr = reinterpret_cast<short*>(bytes);
	return *ptr;	
}

int dataSource::readInt()
{
	char bytes[sizeof(int)];
	for (int i=0;i<sizeof(int);i++) bytes[i] = mReturnBuffer[mByteCounter+i];
	mByteCounter+=sizeof(int);
	int *ptr = reinterpret_cast<int*>(bytes);
	return *ptr;
}

float dataSource::readFloat()
{
	char bytes[sizeof(float)];
	for (int i=0;i<sizeof(float);i++) bytes[i] = mReturnBuffer[mByteCounter+i];
	mByteCounter+=sizeof(float);
	float *ptr = reinterpret_cast<float*>(bytes);
	return *ptr;
}

double dataSource::readDouble()
{
	char bytes[sizeof(double)];
	for (int i=0;i<sizeof(double);i++) bytes[i] = mReturnBuffer[mByteCounter+i];
	mByteCounter+=sizeof(double);
	double *ptr = reinterpret_cast<double*>(bytes);
	return *ptr;
}

char *dataSource::readString()
{
	int length = readInt();
	strncpy(mStringBuffer,&mReturnBuffer[mByteCounter],length);
	mByteCounter += length;
	return mStringBuffer;
}

/////////////////////////////////////////////////////////////////////////////////

void dataSource::addBaseRequest()
{	
	mReturnControls++;//Increment this every time you add a control.
	writeShort(1);//base request opcode
	writeInt(mCurrentTick);//For a baseRequest, do nothing but send a tick value to make sure there's a connection.
}

/////////////////////////////////////////////////////////////////////////////////
