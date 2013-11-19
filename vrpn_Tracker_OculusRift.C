/** @file vrpn_Tracker_OculusRift.C
	@brief Implementation

	@date 2013

	@author
	Jerald Thomas
	<thoma891@d.umn.edu> and <jeraldamo@gmail.com>
	http://www.d.umn.edu/~thoma891
	
	University of Minnesota Duluth 
	Simulation and Interaction in Virtual Environments (SIVE) Lab
    http://wind.d.umn.edu
*/

//          Copyright University of Minnesota Duluth 2013.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file README.legal or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

// Internal Includes
#include "vrpn_Tracker_OculusRift.h"
#include "vrpn_SendTextMessageStreamProxy.h"  // for operator<<, etc

// Library/third-party includes
// - none

// Standard includes
#include <sstream>                      // for operator<<, basic_ostream, etc
#include <string>                       // for char_traits, basic_string, etc
#include <stddef.h>                     // for size_t
#include <stdio.h>                      // for fprintf, NULL, stderr
#include <string.h>                     // for memset

vrpn_Tracker_OculusRift::vrpn_Tracker_OculusRift(const char * name, vrpn_Connection * con)
	: vrpn_Tracker(name, con)
	, status(RIFT_WAITING_FOR_CONNECT)
{
	
	
	vrpn_Tracker::num_sensors = 1;

	/// Initialize all data
	//memset(buttons, 0, sizeof(buttons));
	//memset(lastbuttons, 0, sizeof(lastbuttons));
	//memset(channel, 0, sizeof(channel));
	//memset(last, 0, sizeof(last));

	vrpn_gettimeofday(&_timestamp, NULL);
	
	// Initialize LibOVR Core
	status = RIFT_INITIALIZING_CORE;
	System::Init();
    
	status = RIFT_INITIALIZING_HMD;
	pRiftManager = *DeviceManager::Create();
	
	// No need to create HMD device, go straight to creating sensor device
	pRiftSensor = *pRiftManager->EnumerateDevices<SensorDevice>().CreateDevice();
	if (pRiftSensor)
	{
	    fprintf(stderr, "Located Rift Sensor\n");
	    pRiftSFusion.AttachToSensor(pRiftSensor);
	}
    
    usleep(1000 * 1000 * 10);
    
    fprintf(stderr, "Ready to accept requests...\n");

    _set_reporting();

}

vrpn_Tracker_OculusRift::~vrpn_Tracker_OculusRift() 
{	
	pRiftSensor.Clear();
	pRiftManager.Clear();
	//pRiftSFusion = 0;
	//delete pRiftSFusion;
	System::Destroy();
}

void vrpn_Tracker_OculusRift::_set_reporting()
{
    status = RIFT_REPORTING;
    _connected = _timestamp;
}

void vrpn_Tracker_OculusRift::mainloop() 
{
	// server update
	vrpn_Tracker::server_mainloop();
	
	
	// update report
	if (status == RIFT_REPORTING)
	{
	    _generate_report(0);
	}

}

bool vrpn_Tracker_OculusRift::reconnect() 
{
	status = RIFT_WAITING_FOR_CONNECT;
	return false;
}

void vrpn_Tracker_OculusRift::_generate_report(int sensorNum) 
{
	if (!d_connection) 
	{
		return;
	}

	d_sensor = sensorNum;
    
    // Keep position set to 0, position may be added later by Oculus.
	pos[0] = 0;
	pos[1] = 0;
	pos[2] = 0;

    // Set d_quat
    //fprintf(stderr, "Got here\n");
    Quatf tmpOrientation = pRiftSFusion.GetOrientation();
    //fprintf(stderr, "Got here too\n");
    
	d_quat[Q_W] = tmpOrientation.w;
	d_quat[Q_X] = tmpOrientation.x;
	d_quat[Q_Y] = tmpOrientation.y;
	d_quat[Q_Z] = tmpOrientation.z;
	q_normalize(d_quat, d_quat);

    // Replace old positon with current position
	//q_vec_copy(_old_position[sensorNum], pos);

	char msgbuf[512];
	int len = vrpn_Tracker::encode_to(msgbuf);
	if (d_connection->pack_message(len, _timestamp, position_m_id, d_sender_id, msgbuf,
	                               vrpn_CONNECTION_LOW_LATENCY)) 
	{
		fprintf(stderr, "vrpn_Tracker_OculusRift: cannot write message: tossing\n");
	}
}
