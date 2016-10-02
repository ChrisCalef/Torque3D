//-----------------------------------------------------------------------------
// Copyright (c) 2015 Chris Calef
//-----------------------------------------------------------------------------

#ifndef _VEHICLEDATASOURCE_H_
#define _VEHICLEDATASOURCE_H_

#include "sim/dataSource/dataSource.h"
#include "terrain/terrPager.h"


///////////////////////////////////////////////////////////////////////////////////
typedef struct
{
    double latitude;
    double longitude;
    float altitude;
    int airspeed;
    float roll;
    float pitch;
    float heading;	
    float left_aileron;
    float right_aileron;
    float elevator;
    float rudder;
    float gear;
    float throttle;
    float engine_rpm;
    float rotor_rpm;
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
