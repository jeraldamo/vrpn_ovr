/** @file vrpn_Tracker_OculusRift.h
	@brief Header

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

#pragma once

// Internal Includes
#include "quat.h"                       // for q_vec_type
#include "vrpn_Tracker.h"               // for vrpn_Tracker

// Library/third-party includes
#include "OVR.h"

// Standard includes
// - none

class VRPN_API vrpn_Tracker_OculusRift: public vrpn_Tracker {
	public:
		vrpn_Tracker_OculusRift(const char * name, vrpn_Connection * trackercon);
		~vrpn_Tracker_OculusRift();

		virtual void mainloop();

		virtual bool reconnect();

	private:
		enum RiftStatus {
			RIFT_WAITING_FOR_CONNECT,
			RIFT_INITIALIZING_CORE,
			RIFT_INITIALIZING_HMD,
			RIFT_REPORTING
		};
		RiftStatus status;

		void _set_reporting();
		void _generate_report(int sensorNum);

		struct timeval _timestamp;
		struct timeval _connected;

		q_vec_type _old_position[1];
		
		OVR::DeviceManager* riftManager;
		OVR::HMDDevice* riftHMD;
		OVR::SensorDevice* riftSensor;
		OVR::SensorFusion riftSFusion;
		

};

