/**
ROS/IOP Bridge
Copyright (c) 2017 Fraunhofer

This program is dual licensed; you can redistribute it and/or
modify it under the terms of the GNU General Public License
version 2 as published by the Free Software Foundation, or
enter into a proprietary license agreement with the copyright
holder.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; or you can read the full license at
<http://www.gnu.de/documents/gpl-2.0.html>
*/

/** \author Alexander Tiderko */


#ifndef PLATFORMMODE_RECEIVEFSM_H
#define PLATFORMMODE_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_iop_PlatformMode/Messages/MessageSet.h"
#include "urn_jaus_jss_iop_PlatformMode/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControl/AccessControl_ReceiveFSM.h"


#include "PlatformMode_ReceiveFSM_sm.h"

#include <ros/ros.h>
#include <std_msgs/UInt8.h>

namespace urn_jaus_jss_iop_PlatformMode
{

class DllExport PlatformMode_ReceiveFSM : public JTS::StateMachine
{
public:
	PlatformMode_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM);
	virtual ~PlatformMode_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void SendAction(std::string arg0, Receive::Body::ReceiveRec transportData);
	virtual void SetPlatformModeAction(SetPlatformMode msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods
	virtual bool isControllingClient(Receive::Body::ReceiveRec transportData);


	PlatformMode_ReceiveFSMContext *context;

protected:
    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;

	std::vector<int> p_supported_modes;
	int platform_mode;
	ros::Publisher p_pub_mode;
	ros::Subscriber p_sub_mode;

	void pRosMode(const std_msgs::UInt8::ConstPtr& msg);
	std::map<int, std::string> platform_mode_map();

};

};

#endif // PLATFORMMODE_RECEIVEFSM_H
