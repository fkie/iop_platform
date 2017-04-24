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


#include "urn_jaus_jss_iop_PlatformState_1_0/PlatformState_ReceiveFSM.h"

#include <ros/console.h>


using namespace JTS;

namespace urn_jaus_jss_iop_PlatformState_1_0
{



PlatformState_ReceiveFSM::PlatformState_ReceiveFSM(urn_jaus_jss_core_Transport_1_0::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events_1_0::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl_1_0::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new PlatformState_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
}



PlatformState_ReceiveFSM::~PlatformState_ReceiveFSM()
{
	delete context;
}

void PlatformState_ReceiveFSM::setupNotifications()
{
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled", ieHandler, "InternalStateChange_To_PlatformState_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_Controlled", ieHandler, "InternalStateChange_To_PlatformState_ReceiveFSM_Receiving_Ready_Controlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_PlatformState_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_PlatformState_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_NotControlled", "PlatformState_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_Controlled", "PlatformState_ReceiveFSM");
	registerNotification("Receiving_Ready", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready", "PlatformState_ReceiveFSM");
	registerNotification("Receiving", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving", "PlatformState_ReceiveFSM");

}

void PlatformState_ReceiveFSM::sendReportPlatformStateAction(Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	/**
	<value_enum enum_index="0" enum_const="Initialize"/>
	<value_enum enum_index="1" enum_const="Operational"/>
	<value_enum enum_index="2" enum_const="Shutdown"/>
	<value_enum enum_index="3" enum_const="System_Abort"/>
	<value_enum enum_index="4" enum_const="Emergency"/>
	<value_enum enum_index="5" enum_const="Render_Useless"/>
	 */
	std::string state_str("Operational");
	ROS_DEBUG_NAMED("PlatformState", "sendReportPlatformStateAction to %d.%d.%d, code: %s (currently the one state )", subsystem_id, node_id, component_id, state_str.c_str());
	ReportPlatformState report;
	report.getBody()->getPlatformStateRec()->setPlatformState(1);
	// Now send it to the requesting component
	sendJausMessage( report, sender );
}

void PlatformState_ReceiveFSM::storeRequesterAction(Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	p_requestor = sender;
	ROS_DEBUG_NAMED("PlatformState", "storeRequesterAction %d.%d.%d", subsystem_id, node_id, component_id);
}

void PlatformState_ReceiveFSM::triggerEmergencyAction()
{
	ConfirmPlatformStateRequest msg;
	msg.getBody()->getConfirmPlatformStateRec()->setPlatformState(1);
	// set to InvalidState, we support only operational
	msg.getBody()->getConfirmPlatformStateRec()->setResponseCode(1);
	ROS_DEBUG_NAMED("PlatformState", "triggerEmergencyAction is invalid");
	sendJausMessage( msg, p_requestor );
}

void PlatformState_ReceiveFSM::triggerRecoverEmergencyAction()
{
	ConfirmPlatformStateRequest msg;
	msg.getBody()->getConfirmPlatformStateRec()->setPlatformState(1);
	// set to InvalidState, we support only operational
	msg.getBody()->getConfirmPlatformStateRec()->setResponseCode(0);
	ROS_DEBUG_NAMED("PlatformState", "triggerRecoverEmergencyAction is invalid");
	sendJausMessage( msg, p_requestor );
}

void PlatformState_ReceiveFSM::triggerRenderUselessAction()
{
	ConfirmPlatformStateRequest msg;
	msg.getBody()->getConfirmPlatformStateRec()->setPlatformState(1);
	// set to InvalidState, we support only operational
	msg.getBody()->getConfirmPlatformStateRec()->setResponseCode(1);
	ROS_DEBUG_NAMED("PlatformState", "triggerRenderUselessAction is invalid");
	sendJausMessage( msg, p_requestor );
}

void PlatformState_ReceiveFSM::triggerResetAction()
{
	ConfirmPlatformStateRequest msg;
	msg.getBody()->getConfirmPlatformStateRec()->setPlatformState(1);
	// set to InvalidState, we support only operational
	msg.getBody()->getConfirmPlatformStateRec()->setResponseCode(1);
	ROS_DEBUG_NAMED("PlatformState", "triggerEmergencyAction is invalid");
	sendJausMessage( msg, p_requestor );
}

void PlatformState_ReceiveFSM::triggerShutdownAction()
{
	ConfirmPlatformStateRequest msg;
	msg.getBody()->getConfirmPlatformStateRec()->setPlatformState(1);
	// set to InvalidState, we support only operational
	msg.getBody()->getConfirmPlatformStateRec()->setResponseCode(1);
	ROS_DEBUG_NAMED("PlatformState", "triggerShutdownAction is invalid");
	sendJausMessage( msg, p_requestor );
}



bool PlatformState_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
	//// By default, inherited guards call the parent function.
	//// This can be replaced or modified as needed.
	return pAccessControl_ReceiveFSM->isControllingClient(transportData );
}

bool PlatformState_ReceiveFSM::setToEmergency(SetPlatformState msg)
{
	/// Insert User Code HERE
	ROS_WARN("PlatformState: setToEmergency not implemented!");
	return false;
}

bool PlatformState_ReceiveFSM::setToInitialize(SetPlatformState msg)
{
	/// Insert User Code HERE
	ROS_WARN("PlatformState: setToInitialize not implemented!");
	return false;
}

bool PlatformState_ReceiveFSM::setToOperational(SetPlatformState msg)
{
	/// Insert User Code HERE
	return true;
}

bool PlatformState_ReceiveFSM::setToRenderUseless(SetPlatformState msg)
{
	/// Insert User Code HERE
	ROS_WARN("PlatformState: setToRenderUseless not implemented!");
	return false;
}

bool PlatformState_ReceiveFSM::setToShutdown(SetPlatformState msg)
{
	/// Insert User Code HERE
	ROS_WARN("PlatformState: setToShutdown not implemented!");
	return false;
}



};
