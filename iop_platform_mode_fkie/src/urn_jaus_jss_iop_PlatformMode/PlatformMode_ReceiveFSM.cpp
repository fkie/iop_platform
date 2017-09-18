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

#include "urn_jaus_jss_iop_PlatformMode/PlatformMode_ReceiveFSM.h"

#include <iostream>



using namespace std;
using namespace JTS;

namespace urn_jaus_jss_iop_PlatformMode
{


PlatformMode_ReceiveFSM::PlatformMode_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new PlatformMode_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
	p_pnh = ros::NodeHandle("~");
	p_supported_modes.push_back(0);  // only Standard operation is supported by default
	platform_mode = 0;
}


PlatformMode_ReceiveFSM::~PlatformMode_ReceiveFSM()
{
	delete context;
}

void PlatformMode_ReceiveFSM::setupNotifications()
{
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled", ieHandler, "InternalStateChange_To_PlatformMode_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_Controlled", ieHandler, "InternalStateChange_To_PlatformMode_ReceiveFSM_Receiving_Ready_Controlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_PlatformMode_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_PlatformMode_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_NotControlled", "PlatformMode_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_Controlled", "PlatformMode_ReceiveFSM");
	registerNotification("Receiving_Ready", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready", "PlatformMode_ReceiveFSM");
	registerNotification("Receiving", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving", "PlatformMode_ReceiveFSM");

	int platformmode;
	p_pnh.param("platform_mode", platformmode, 0);
	platform_mode = platformmode;
	std::vector<double> multipliers;
	p_pnh.param("supported_modes", p_supported_modes, p_supported_modes);
	if (p_supported_modes.size() == 0) {
		p_supported_modes.push_back(0);
	}
	p_pub_mode = p_pnh.advertise<std_msgs::UInt8>("platform_mode", 5, true);
	std_msgs::UInt8 rosmsg;
	rosmsg.data = platform_mode;
	p_pub_mode.publish(rosmsg);
	p_sub_mode = p_pnh.subscribe<std_msgs::UInt8>("set_platform_mode", 1, &PlatformMode_ReceiveFSM::pRosMode, this);

}

void PlatformMode_ReceiveFSM::pRosMode(const std_msgs::UInt8::ConstPtr& msg)
{
	platform_mode = msg->data;
}

void PlatformMode_ReceiveFSM::SendAction(std::string arg0, Receive::Body::ReceiveRec transportData)
{
	int16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	int mode_status;
	ros::NodeHandle pnh("~");
	ReportPlatformMode response;
	ReportPlatformMode::Body::ReportPlatformModeRec comp;
	if(strcmp(arg0.c_str(), "ReportPlatformMode") == 0)
	{
		comp.setStatus(1);  // 0="Initializing", 1="Active", 2="Exiting"
		comp.setPlatformMode(platform_mode);
		response.getBody()->setReportPlatformModeRec(comp);
		sendJausMessage(response, sender);
	}
	else if	(strcmp(arg0.c_str(),"ReportSupportedPlatformModes") == 0)
	{
		ReportSupportedPlatformModes responses;
		for (unsigned int i = 0; i < p_supported_modes.size(); i++) {
			ReportSupportedPlatformModes::Body::SupportedPlatformModesList::PlatformModeRec comps;
			comps.setPlatformMode(p_supported_modes[i]);
			responses.getBody()->getSupportedPlatformModesList()->addElement(comps);
		}
		sendJausMessage(responses, sender);
	}
}

void PlatformMode_ReceiveFSM::SetPlatformModeAction(SetPlatformMode msg,Receive::Body::ReceiveRec transportData)
{
	platform_mode =  msg.getBody()->getPlatformModeRec()->getPlatformMode();
	std_msgs::UInt8 rosmsg;
	rosmsg.data = platform_mode;
	p_pub_mode.publish(rosmsg);
}

bool PlatformMode_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
	//// By default, inherited guards call the parent function.
	//// This can be replaced or modified as needed.
	return pAccessControl_ReceiveFSM->isControllingClient(transportData );
}

};
