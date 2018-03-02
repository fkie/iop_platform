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


#include "urn_jaus_jss_iop_PlatformState/PlatformState_ReceiveFSM.h"

#include <ros/console.h>
#include <iop_component_fkie/iop_component.h>
#include <iop_discovery_fkie/DiscoveryComponent.h>

using namespace JTS;
using namespace urn_jaus_jss_core_Discovery;

namespace urn_jaus_jss_iop_PlatformState
{



PlatformState_ReceiveFSM::PlatformState_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM)
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
	this->p_discovery_srv = NULL;
	p_init_platform_state = 1;  //starts with operational state
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
	p_own_address = *(jausRouter->getJausAddress());
	pEvents_ReceiveFSM->get_event_handler().register_query(QueryPlatformState::ID);
	iop::Config cfg("~PlatformState");
	cfg.param("init_platform_state", p_init_platform_state, p_init_platform_state);
	std::vector<std::string> s_sup_default;
	s_sup_default.push_back("OPERATIONAL");
	s_sup_default.push_back("EMERGENCY");
	cfg.param<std::vector<std::string> >("supported_states", p_supported_states, s_sup_default);
	// normalize string to lower case
	std::vector<std::string> nomalizedlist;
	std::vector<std::string>::iterator it;
	for (it = p_supported_states.begin(); it != p_supported_states.end(); it++) {
		std::string normstr = *it;
		std::transform(normstr.begin(), normstr.end(), normstr.begin(), ::tolower);
		nomalizedlist.push_back(normstr);
	}
	p_supported_states = nomalizedlist;
	p_current_state.getBody()->getPlatformStateRec()->setPlatformState(p_get_current_state());
	p_sub_state = cfg.subscribe<std_msgs::UInt8>("robot_platform_state", 1, &PlatformState_ReceiveFSM::pRosNewState, this);
	p_sub_state_str = cfg.subscribe<std_msgs::String>("robot_platform_state_str", 1, &PlatformState_ReceiveFSM::pRosNewStateStr, this);
	p_pub_state = cfg.advertise<std_msgs::UInt8>("set_platform_state", 10);
	p_pub_state_str = cfg.advertise<std_msgs::String>("set_platform_state_str", 10);
	pEvents_ReceiveFSM->get_event_handler().set_report(QueryPlatformState::ID, &p_current_state);
}

Discovery_ReceiveFSM* PlatformState_ReceiveFSM::p_get_discovery()
{
	if (p_discovery_srv == NULL) {
		iop::Component &cmp = iop::Component::get_instance();
		DiscoveryService *discovery_srv = static_cast<DiscoveryService*>(cmp.get_service("Discovery"));
		if (discovery_srv != NULL) {
			p_discovery_srv = discovery_srv->pDiscovery_ReceiveFSM;
		} else {
			throw std::runtime_error("[Slave] no Discovery found! Please include its plugin!");
		}
	}
	return p_discovery_srv;
}

void PlatformState_ReceiveFSM::pRosNewState(const std_msgs::UInt8::ConstPtr& msg)
{
	p_current_state.getBody()->getPlatformStateRec()->setPlatformState(msg->data);
	pEvents_ReceiveFSM->get_event_handler().set_report(QueryPlatformState::ID, &p_current_state);
}

void PlatformState_ReceiveFSM::pRosNewStateStr(const std_msgs::String::ConstPtr& msg)
{
	p_current_state.getBody()->getPlatformStateRec()->setPlatformState(p_state2int(msg->data));
	pEvents_ReceiveFSM->get_event_handler().set_report(QueryPlatformState::ID, &p_current_state);
}

void PlatformState_ReceiveFSM::sendReportPlatformStateAction(Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	ROS_DEBUG_NAMED("PlatformState", "sendReportPlatformStateAction to %s, state: %s [%d]", sender.str().c_str(), p_get_current_state_str().c_str(), p_get_current_state());
	ReportPlatformState report;
	report.getBody()->getPlatformStateRec()->setPlatformState(p_get_current_state());
	// Now send it to the requesting component
	sendJausMessage( report, sender );
}

void PlatformState_ReceiveFSM::storeRequesterAction(Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	p_requestor = sender;
	ROS_DEBUG_NAMED("PlatformState", "storeRequesterAction %s", sender.str().c_str());
}

void PlatformState_ReceiveFSM::triggerEmergencyAction()
{
	// set emergency for all components
	int resp_code = p_publish_state(EMERGENCY);
	ROS_DEBUG_NAMED("PlatformState", "triggerEmergency from %s response code %d  (0=TRANSITIONING, 1=INVALID_STATE)", p_requestor.str().c_str(), resp_code);
	if (p_is_supported_state(EMERGENCY)) {
		std::vector<iop::DiscoveryComponent> mcp = p_get_discovery()->getComponents("urn:jaus:jss:core:Management");
		std::vector<iop::DiscoveryComponent>::iterator it;
		SetEmergency jsmsg;
		jsmsg.getBody()->getSetEmergencyRec()->setEmergencyCode(1);
		for (it = mcp.begin(); it != mcp.end(); it++) {
			ROS_DEBUG_NAMED("PlatformState", "  forward SetEmergency to %s", it->address.str().c_str());
			sendJausMessage(jsmsg, it->address);
			resp_code = TRANSITIONING;
		}
	}
	ConfirmPlatformStateRequest msg;
	msg.getBody()->getConfirmPlatformStateRec()->setPlatformState(EMERGENCY);
	msg.getBody()->getConfirmPlatformStateRec()->setResponseCode(resp_code);
	sendJausMessage(msg, p_requestor);
}

void PlatformState_ReceiveFSM::triggerRecoverEmergencyAction()
{
	int resp_code = p_publish_state(OPERATIONAL);
	ROS_DEBUG_NAMED("PlatformState", "triggerRecoverEmergency from %s response code %d  (0=TRANSITIONING, 1=INVALID_STATE)", p_requestor.str().c_str(), resp_code);
	// recover emergency for all components
	if (p_is_supported_state(OPERATIONAL)) {
		std::vector<iop::DiscoveryComponent> mcp = p_get_discovery()->getComponents("urn:jaus:jss:core:Management");
		std::vector<iop::DiscoveryComponent>::iterator it;
		ClearEmergency jsmsg;
		jsmsg.getBody()->getClearEmergencyRec()->setEmergencyCode(1);
		for (it = mcp.begin(); it != mcp.end(); it++) {
			ROS_DEBUG_NAMED("PlatformState", "  forward ClearEmergency to %s", it->address.str().c_str());
			sendJausMessage(jsmsg, it->address);
			resp_code = TRANSITIONING;
		}
	}
	ConfirmPlatformStateRequest msg;
	msg.getBody()->getConfirmPlatformStateRec()->setPlatformState(OPERATIONAL);
	msg.getBody()->getConfirmPlatformStateRec()->setResponseCode(resp_code);
	sendJausMessage(msg, p_requestor);
}

void PlatformState_ReceiveFSM::triggerRenderUselessAction()
{
	int resp_code = p_publish_state(RENDER_USELESS);
	ConfirmPlatformStateRequest msg;
	msg.getBody()->getConfirmPlatformStateRec()->setPlatformState(RENDER_USELESS);
	msg.getBody()->getConfirmPlatformStateRec()->setResponseCode(resp_code);
	ROS_DEBUG_NAMED("PlatformState", "triggerRenderUseless from %s response code %d  (0=TRANSITIONING, 1=INVALID_STATE)", p_requestor.str().c_str(), resp_code);
	sendJausMessage( msg, p_requestor );
}

void PlatformState_ReceiveFSM::triggerResetAction()
{
	int resp_code = p_publish_state(INITIALIZE);
	ConfirmPlatformStateRequest msg;
	msg.getBody()->getConfirmPlatformStateRec()->setPlatformState(INITIALIZE);
	msg.getBody()->getConfirmPlatformStateRec()->setResponseCode(resp_code);
	ROS_DEBUG_NAMED("PlatformState", "triggerReset from %s response code %d  (0=TRANSITIONING, 1=INVALID_STATE)", p_requestor.str().c_str(), resp_code);
	sendJausMessage( msg, p_requestor );
}

void PlatformState_ReceiveFSM::triggerShutdownAction()
{
	int resp_code = p_publish_state(SHUTDOWN);
	ConfirmPlatformStateRequest msg;
	msg.getBody()->getConfirmPlatformStateRec()->setPlatformState(SHUTDOWN);
	msg.getBody()->getConfirmPlatformStateRec()->setResponseCode(resp_code);
	ROS_DEBUG_NAMED("PlatformState", "triggerShutdown from %s response code %d  (0=TRANSITIONING, 1=INVALID_STATE)", p_requestor.str().c_str(), resp_code);
	sendJausMessage( msg, p_requestor );
}


int PlatformState_ReceiveFSM::p_publish_state(int state) {
	int resp_code = INVALID_STATE;
	bool supported = p_is_supported_state(state);
	if (supported) {
		if (p_pub_state.getNumSubscribers() > 0 || p_pub_state_str.getNumSubscribers() > 0) {
			resp_code = TRANSITIONING;
			if (p_pub_state.getNumSubscribers() > 0) {
				std_msgs::UInt8 ros_msg;
				ros_msg.data = state;
				p_pub_state.publish(ros_msg);
			}
			if (p_pub_state_str.getNumSubscribers() > 0) {
				std_msgs::String ros_msg;
				ros_msg.data = p_state2str(state);
				p_pub_state_str.publish(ros_msg);
			}
		}
	}
	return resp_code;
}

bool PlatformState_ReceiveFSM::p_is_supported_state(int state)
{
	std::string normstr = p_state2str(state);
	std::vector<std::string>::iterator it = std::find(p_supported_states.begin(), p_supported_states.end(), p_state2str(state));
	return (it != p_supported_states.end());
}

bool PlatformState_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
	//// By default, inherited guards call the parent function.
	//// This can be replaced or modified as needed.
	return pAccessControl_ReceiveFSM->isControllingClient(transportData );
}

bool PlatformState_ReceiveFSM::isIDStored(Receive::Body::ReceiveRec transportData)
{
	bool result = (transportData.getAddress() == p_requestor);
	Receive::Body::ReceiveRec td;
	td.setSrcSubsystemID(p_own_address.getSubsystemID());
	td.setSrcNodeID(p_own_address.getNodeID());
	td.setSrcComponentID(p_own_address.getComponentID());
	result &= pAccessControl_ReceiveFSM->isEmergencyClient(td);
	return result;
}

bool PlatformState_ReceiveFSM::setToEmergency(SetPlatformState msg)
{
	return (msg.getBody()->getPlatformStateRec()->getPlatformState() == EMERGENCY);
}

bool PlatformState_ReceiveFSM::setToInitialize(SetPlatformState msg)
{
	return (msg.getBody()->getPlatformStateRec()->getPlatformState() == INITIALIZE);
}

bool PlatformState_ReceiveFSM::setToOperational(SetPlatformState msg)
{
	return (msg.getBody()->getPlatformStateRec()->getPlatformState() == OPERATIONAL);
}

bool PlatformState_ReceiveFSM::setToRenderUseless(SetPlatformState msg)
{
	return (msg.getBody()->getPlatformStateRec()->getPlatformState() == RENDER_USELESS);
}

bool PlatformState_ReceiveFSM::setToShutdown(SetPlatformState msg)
{
	return (msg.getBody()->getPlatformStateRec()->getPlatformState() == SHUTDOWN);
}

int PlatformState_ReceiveFSM::p_get_current_state()
{
	return p_current_state.getBody()->getPlatformStateRec()->getPlatformState();
}

std::string PlatformState_ReceiveFSM::p_get_current_state_str()
{
	return p_state2str(p_current_state.getBody()->getPlatformStateRec()->getPlatformState());
}

std::string PlatformState_ReceiveFSM::p_state2str(int state)
{
	std::string result = "UNKNOWN";
	switch (state) {
		case INITIALIZE: {
			result = "initialize";
			break;
		}
		case OPERATIONAL: {
			result = "operational";
			break;
		}
		case SHUTDOWN: {
			result = "shutdown";
			break;
		}
		case SYSTEM_ABORT: {
			result = "system_abort";
			break;
		}
		case EMERGENCY: {
			result = "emergency";
			break;
		}
		case RENDER_USELESS: {
			result = "render_useless";
			break;
		}
	}
	return result;
}

int PlatformState_ReceiveFSM::p_state2int(std::string state)
{
	std::string normstr = state;
	std::transform(normstr.begin(), normstr.end(), normstr.begin(), ::tolower);
	if (normstr.compare("initialize") == 0) {
		return 0;
	}
	if (normstr.compare("operational") == 0) {
		return 1;
	}
	if (normstr.compare("shutdown") == 0) {
		return 2;
	}
	if (normstr.compare("system_abort") == 0) {
		return 3;
	}
	if (normstr.compare("emergency") == 0) {
		return 4;
	}
	if (normstr.compare("render_useless") == 0) {
		return 5;
	}
	return -1;
}

};
