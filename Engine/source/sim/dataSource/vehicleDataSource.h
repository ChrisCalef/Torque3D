//-----------------------------------------------------------------------------
// Copyright (c) 2015 Chris Calef
//-----------------------------------------------------------------------------

#ifndef _VEHICLEDATASOURCE_H_
#define _VEHICLEDATASOURCE_H_

#include "sim/dataSource/dataSource.h"
#include "terrain/terrPager.h"


///////////////////////////////////////////////////////////////////////////////////
typedef struct
{//HERE: I think latitude and longitude need to be doubles, but gotta work that out coming from flightgear and flipping endianness.
    double latitude;
    double longitude;
    float altitude;
    int airspeed;
    float roll;
    float pitch;
    float heading;
} flightgear_packet;
///////////////////////////////////////////////////////////////////////////////////


/// Base class for various kinds of data sources, first one being worldDataSource, for terrain, sky, weather and map information.
class vehicleDataSource : public dataSource 
{
public:

	flightgear_packet mFGPacket;
	MatrixF mFGTransform;

	TerrainPager *mTerrainPager;

	vehicleDataSource(bool listening);
	~vehicleDataSource();

	void listenForPacket();
	void openListenSocket();

	void tick();
};

#endif // _VEHICLEDATASOURCE_H_
