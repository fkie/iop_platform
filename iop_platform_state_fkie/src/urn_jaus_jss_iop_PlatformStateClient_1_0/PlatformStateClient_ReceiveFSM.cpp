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


#include "urn_jaus_jss_iop_PlatformStateClient_1_0/PlatformStateClient_ReceiveFSM.h"




using namespace JTS;

namespace urn_jaus_jss_iop_PlatformStateClient_1_0
{

unsigned char PlatformStateClient_ReceiveFSM::PLATFORM_STATE_INIT = 0;
unsigned char PlatformStateClient_ReceiveFSM::PLATFORM_STATE_OPERATIONAL = 1;
unsigned char PlatformStateClient_ReceiveFSM::PLATFORM_STATE_SHUTDOWN = 2;
unsigned char PlatformStateClient_ReceiveFSM::PLATFORM_STATE_SYSTEM_ABORT = 3;
unsigned char PlatformStateClient_ReceiveFSM::PLATFORM_STATE_EMERGENCY = 4;
unsigned char PlatformStateClient_ReceiveFSM::PLATFORM_STATE_RENDER_USELESS = 5;
unsigned char PlatformStateClient_ReceiveFSM::PLATFORM_STATE_UNKNOWN = 255;


PlatformStateClient_ReceiveFSM::PlatformStateClient_ReceiveFSM(urn_jaus_jss_core_Transport_1_0::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient_1_0::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient_1_0::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new PlatformStateClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	p_state = PLATFORM_STATE_UNKNOWN;
}



PlatformStateClient_ReceiveFSM::~PlatformStateClient_ReceiveFSM()
{
	delete context;
}

void PlatformStateClient_ReceiveFSM::setupNotifications()
{
	pAccessControlClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_PlatformStateClient_ReceiveFSM_Receiving_Ready", "AccessControlClient_ReceiveFSM");
	pAccessControlClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_PlatformStateClient_ReceiveFSM_Receiving_Ready", "AccessControlClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving_Ready", "PlatformStateClient_ReceiveFSM");
	registerNotification("Receiving", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving", "PlatformStateClient_ReceiveFSM");

}

void PlatformStateClient_ReceiveFSM::handleConfirmPlatformStateRequestAction(ConfirmPlatformStateRequest msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	if (msg.getBody()->getConfirmPlatformStateRec()->getResponseCode() == 0) {
		p_state = msg.getBody()->getConfirmPlatformStateRec()->getPlatformState();
	}
	ROS_DEBUG_NAMED("PlatformStateClient", "ConfirmPlatformStateRequest, state: %d", p_state);
	if (!p_class_interface_callback.empty()) {
		ROS_DEBUG_NAMED("PlatformStateClient", "  forward to handler");
		p_class_interface_callback(sender, p_state);
	}

}

void PlatformStateClient_ReceiveFSM::handleReportPlatformStateAction(ReportPlatformState msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	p_state = msg.getBody()->getPlatformStateRec()->getPlatformState();
	ROS_DEBUG_NAMED("PlatformStateClient", "ReportPlatformState, state: %d", p_state);
	if (!p_class_interface_callback.empty()) {
		ROS_DEBUG_NAMED("ManagementClient", "  forward to handler");
		p_class_interface_callback(sender, p_state);
	}
}

void PlatformStateClient_ReceiveFSM::query_state(JausAddress address)
{
  QueryPlatformState msg;
  sendJausMessage( msg, address );
}

void PlatformStateClient_ReceiveFSM::set_state(JausAddress address, unsigned char state)
{
  SetPlatformState msg;
  msg.getBody()->getPlatformStateRec()->setPlatformState(state);
  sendJausMessage( msg, address);
//  query_state(address);
}



};
