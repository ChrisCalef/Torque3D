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
	mNumReturnControls = 0;
	mNumReturnBytes = 0;	
	mByteCounter = 0;
	sprintf(mSourceIP,"127.0.0.1");
	mListenSockfd = INVALID_SOCKET;
	mWorkSockfd = INVALID_SOCKET;
	mReturnBuffer = NULL;
	mFloatBuffer = NULL;
	mReadyForRequests = false;
	
	if (listening)
	{
		mListening = true;
		mSending = false;
	} else {
		mListening = false;
		mSending = true;
	}
	Con::printf("New data source object!!! Source IP: %s\n",mSourceIP);
}

dataSource::~dataSource()
{
	//delete [] mReturnBuffer;
	//delete [] mFloatBuffer;
	disconnectSockets();
}

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
	} else if (mCurrentTick % mTalkInterval == 0) {
		Con::printf("\ndataSource current tick: %d\n",mCurrentTick);
	}
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
	} else {
		//printf(" listen socket success!\n");
	}
	
	mWorkSockfd = accept(mListenSockfd,NULL,NULL);
	
	if (mWorkSockfd == INVALID_SOCKET)
	{
		Con::printf(".");
		return;
	} else {
		Con::printf("\nlisten accept succeeded!\n");
		mReturnBuffer = new char[mPacketSize];
	}
}

void dataSource::listenForPacket()
{
	//char *buffer;
	//buffer = new char[mPacketSize];//,buffer2[packetsize];//I do need a number for my input buffer size though.
	
	struct sockaddr_in client_addr;
	int n=0;
	int i=0;
	mByteCounter = 0;
	char floatBytes[256];//sizeof(float)?
	float OPCODE,hostFrame;
	int num_args;

	n = recv(mWorkSockfd,mReturnBuffer,mPacketSize,0);
	//n = recv(mListenSockfd,buffer,mPacketSize,0);
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
	//delete [] buffer;
}

/////////////////////////////////////////////////////////////////////////////////

void dataSource::connectSendSocket()
{
	struct sockaddr_in source_addr;
	
	mReturnBuffer = new char[mPacketSize];
	mFloatBuffer = new float[mPacketSize/sizeof(float)];

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
        Con::printf("ERROR connecting send socket: %d\n",connectCode);
		return;
	} else {		
        Con::printf("SUCCESS connecting send socket: %d\n",connectCode);
	}
}

void dataSource::sendPacket()
{
	//char *returnBuffer;
	//mReturnBuffer = new char[mPacketSize];
	//float *outArray;
	//mFloatBuffer = new float[mPacketSize/sizeof(float)];
	mNumReturnControls = 1;

	mFloatBuffer[0] = (float)mNumReturnControls;
	mFloatBuffer[1] = 1.0f;
	mFloatBuffer[2] = (float)mCurrentTick;//One argument sent in base packet, current frame.
	mReturnBuffer = reinterpret_cast<char*>(mFloatBuffer);

	int n = send(mWorkSockfd,mReturnBuffer,mPacketSize,0);
	Con::printf("dataSource sent packet, n=%d",n);
	
	clearPacket();

	//delete [] mReturnBuffer;
	//delete [] mFloatBuffer;//Weird, thought this was how it was done, but crash burn fail.
}

void dataSource::clearPacket()
{
	memset((void *)(mFloatBuffer),NULL,1024);	
	mNumReturnControls = 0;
	mNumReturnBytes = 0;	
}

void dataSource::addBaseRequest()
{
	mNumReturnControls++;//Increment mNumReturnControls every time you add a control.
	mFloatBuffer[0] = (float)mNumReturnControls;

	mNumReturnBytes++;
	mFloatBuffer[mNumReturnBytes] = 1.0f;
	mNumReturnBytes++;
	mFloatBuffer[mNumReturnBytes] = (float)mCurrentTick;//Send only one argument in base request: current tick.

	Con::printf("adding base request, currentTick %d, numReturnBytes %d\n",mCurrentTick,mNumReturnBytes);
}

float *dataSource::getFloatBytes(unsigned int num_args)
{
	char floatBytes[1024];
	for (int i=0;i<sizeof(float)*num_args;i++) 
		floatBytes[i] = mReturnBuffer[mByteCounter+i];
	mByteCounter += sizeof(float)*num_args;
	return reinterpret_cast<float*>(floatBytes);
}
/////////////////////////////////////////////////////////////////////////////////

void dataSource::disconnectSockets()
{
	closesocket(mListenSockfd);
	closesocket(mWorkSockfd);
}

/////////////////////////////////////////////////////////////////////////////////

