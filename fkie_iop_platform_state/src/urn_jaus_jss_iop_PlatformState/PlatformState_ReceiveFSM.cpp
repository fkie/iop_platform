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
#include <fkie_iop_component/iop_config.hpp>
#include <fkie_iop_discovery/DiscoveryComponent.h>

using namespace JTS;
using namespace urn_jaus_jss_core_Discovery;

namespace urn_jaus_jss_iop_PlatformState
{



PlatformState_ReceiveFSM::PlatformState_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: logger(cmp->get_logger().get_child("PlatformState"))
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new PlatformState_ReceiveFSMContext(*this);

	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
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
}


void PlatformState_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "PlatformState");
	cfg.declare_param<uint8_t>("init_platform_state", p_init_platform_state, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER,
		"Default state on start.",
		"Default: 1; 0:initialize, 1:operational, 2:shutdown, 3:system_abort, 4:emergency, 5:render_useless");
	cfg.declare_param<std::vector<std::string> >("supported_states", p_supported_states, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY,
		"A list with supported states.",
		"Default: [OPERATIONAL, EMERGENCY]; Possible entries: initialize, operational, shutdown, system_abort, emergency, render_useless");

	p_own_address = *(jausRouter->getJausAddress());
	pEvents_ReceiveFSM->get_event_handler().register_query(QueryPlatformState::ID);
	std::map<uint8_t, std::string> ps_names;
	ps_names[0] = "initialize";
	ps_names[1] = "operational";
	ps_names[2] = "shutdown";
	ps_names[3] = "system_abort";
	ps_names[4] = "emergency";
	ps_names[5] = "render_useless";
	cfg.param_named("init_platform_state", p_init_platform_state, p_init_platform_state, ps_names);
	std::vector<std::string> s_sup_default;
	s_sup_default.push_back("OPERATIONAL");
	s_sup_default.push_back("EMERGENCY");
	cfg.param_vector<std::vector<std::string> >("supported_states", p_supported_states, s_sup_default);
	// normalize string to lower case
	std::vector<std::string> nomalizedlist;
	std::vector<std::string>::iterator it;
	for (it = p_supported_states.begin(); it != p_supported_states.end(); it++) {
		std::string normstr = *it;
		std::transform(normstr.begin(), normstr.end(), normstr.begin(), ::tolower);
		nomalizedlist.push_back(normstr);
	}
	p_supported_states = nomalizedlist;
	p_current_state.getBody()->getPlatformStateRec()->setPlatformState(p_init_platform_state);
	p_sub_state = cfg.create_subscription<std_msgs::msg::UInt8>("robot_platform_state", 1, std::bind(&PlatformState_ReceiveFSM::pRosNewState, this, std::placeholders::_1));
	p_sub_state_str = cfg.create_subscription<std_msgs::msg::String>("robot_platform_state_str", 1, std::bind(&PlatformState_ReceiveFSM::pRosNewStateStr, this, std::placeholders::_1));
	p_pub_state = cfg.create_publisher<std_msgs::msg::UInt8>("set_platform_state", 10);
	p_pub_state_str = cfg.create_publisher<std_msgs::msg::String>("set_platform_state_str", 10);
	pEvents_ReceiveFSM->get_event_handler().set_report(QueryPlatformState::ID, &p_current_state);
}

Discovery_ReceiveFSM* PlatformState_ReceiveFSM::p_get_discovery()
{
	if (p_discovery_srv == NULL) {
		DiscoveryService *discovery_srv = static_cast<DiscoveryService*>(cmp->get_service("DiscoveryService"));
		if (discovery_srv != NULL) {
			p_discovery_srv = discovery_srv->pDiscovery_ReceiveFSM;
		} else {
			throw std::runtime_error("[Slave] no Discovery found! Please include its plugin!");
		}
	}
	return p_discovery_srv;
}

void PlatformState_ReceiveFSM::pRosNewState(const std_msgs::msg::UInt8::SharedPtr msg)
{
	p_current_state.getBody()->getPlatformStateRec()->setPlatformState(msg->data);
	pEvents_ReceiveFSM->get_event_handler().set_report(QueryPlatformState::ID, &p_current_state);
}

void PlatformState_ReceiveFSM::pRosNewStateStr(const std_msgs::msg::String::SharedPtr msg)
{
	p_current_state.getBody()->getPlatformStateRec()->setPlatformState(p_state2int(msg->data));
	pEvents_ReceiveFSM->get_event_handler().set_report(QueryPlatformState::ID, &p_current_state);
}

void PlatformState_ReceiveFSM::sendReportPlatformStateAction(Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	RCLCPP_DEBUG(logger, "sendReportPlatformStateAction to %s, state: %s [%d]", sender.str().c_str(), p_get_current_state_str().c_str(), p_get_current_state());
	ReportPlatformState report;
	report.getBody()->getPlatformStateRec()->setPlatformState(p_get_current_state());
	// Now send it to the requesting component
	sendJausMessage( report, sender );
}

void PlatformState_ReceiveFSM::storeRequesterAction(Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	p_requestor = sender;
	RCLCPP_DEBUG(logger, "storeRequesterAction %s", sender.str().c_str());
}

void PlatformState_ReceiveFSM::triggerEmergencyAction()
{
	// set emergency for all components
	int resp_code = p_publish_state(EMERGENCY);
	RCLCPP_DEBUG(logger, "triggerEmergency from %s response code %d  (0=TRANSITIONING, 1=INVALID_STATE)", p_requestor.str().c_str(), resp_code);
	if (p_is_supported_state(EMERGENCY)) {
		std::vector<iop::DiscoveryComponent> mcp = p_get_discovery()->getComponents("urn:jaus:jss:core:Management");
		std::vector<iop::DiscoveryComponent>::iterator it;
		SetEmergency jsmsg;
		jsmsg.getBody()->getSetEmergencyRec()->setEmergencyCode(1);
		for (it = mcp.begin(); it != mcp.end(); it++) {
			RCLCPP_DEBUG(logger, "  forward SetEmergency to %s", it->address.str().c_str());
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
	RCLCPP_DEBUG(logger, "triggerRecoverEmergency from %s response code %d  (0=TRANSITIONING, 1=INVALID_STATE)", p_requestor.str().c_str(), resp_code);
	// recover emergency for all components
	if (p_is_supported_state(OPERATIONAL)) {
		std::vector<iop::DiscoveryComponent> mcp = p_get_discovery()->getComponents("urn:jaus:jss:core:Management");
		std::vector<iop::DiscoveryComponent>::iterator it;
		ClearEmergency jsmsg;
		jsmsg.getBody()->getClearEmergencyRec()->setEmergencyCode(1);
		for (it = mcp.begin(); it != mcp.end(); it++) {
			RCLCPP_DEBUG(logger, "  forward ClearEmergency to %s", it->address.str().c_str());
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
	RCLCPP_DEBUG(logger, "triggerRenderUseless from %s response code %d  (0=TRANSITIONING, 1=INVALID_STATE)", p_requestor.str().c_str(), resp_code);
	sendJausMessage( msg, p_requestor );
}

void PlatformState_ReceiveFSM::triggerResetAction()
{
	int resp_code = p_publish_state(INITIALIZE);
	ConfirmPlatformStateRequest msg;
	msg.getBody()->getConfirmPlatformStateRec()->setPlatformState(INITIALIZE);
	msg.getBody()->getConfirmPlatformStateRec()->setResponseCode(resp_code);
	RCLCPP_DEBUG(logger, "triggerReset from %s response code %d  (0=TRANSITIONING, 1=INVALID_STATE)", p_requestor.str().c_str(), resp_code);
	sendJausMessage( msg, p_requestor );
}

void PlatformState_ReceiveFSM::triggerShutdownAction()
{
	int resp_code = p_publish_state(SHUTDOWN);
	ConfirmPlatformStateRequest msg;
	msg.getBody()->getConfirmPlatformStateRec()->setPlatformState(SHUTDOWN);
	msg.getBody()->getConfirmPlatformStateRec()->setResponseCode(resp_code);
	RCLCPP_DEBUG(logger, "triggerShutdown from %s response code %d  (0=TRANSITIONING, 1=INVALID_STATE)", p_requestor.str().c_str(), resp_code);
	sendJausMessage( msg, p_requestor );
}


int PlatformState_ReceiveFSM::p_publish_state(int state) {
	int resp_code = INVALID_STATE;
	bool supported = p_is_supported_state(state);
	if (supported) {
		if (p_pub_state->get_subscription_count() > 0 || p_pub_state_str->get_subscription_count() > 0) {
			resp_code = TRANSITIONING;
			if (p_pub_state->get_subscription_count() > 0) {
				auto ros_msg = std_msgs::msg::UInt8();
				ros_msg.data = state;
				p_pub_state->publish(ros_msg);
			}
			if (p_pub_state_str->get_subscription_count() > 0) {
				auto ros_msg = std_msgs::msg::String();
				ros_msg.data = p_state2str(state);
				p_pub_state_str->publish(ros_msg);
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

}
