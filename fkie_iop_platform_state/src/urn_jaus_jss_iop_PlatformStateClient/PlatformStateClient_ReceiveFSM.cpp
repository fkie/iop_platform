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


#include "urn_jaus_jss_iop_PlatformStateClient/PlatformStateClient_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>


using namespace JTS;

namespace urn_jaus_jss_iop_PlatformStateClient
{

PlatformStateClient_ReceiveFSM::PlatformStateClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: SlaveHandlerInterface(cmp, "PlatformStateClient", 10.0),
  logger(cmp->get_logger().get_child("PlatformStateClient"))
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new PlatformStateClient_ReceiveFSMContext(*this);

	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
	p_state = PLATFORM_STATE_UNKNOWN;
	p_hz = 0.0;
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


void PlatformStateClient_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "PlatformStateClient");
	cfg.param("hz", p_hz, p_hz, false);
	p_sub_state = cfg.create_subscription<std_msgs::msg::UInt8>("cmd_platform_state", 1, std::bind(&PlatformStateClient_ReceiveFSM::pRosNewCmdState, this, std::placeholders::_1));
	p_sub_state_str = cfg.create_subscription<std_msgs::msg::String>("cmd_platform_state_str", 1, std::bind(&PlatformStateClient_ReceiveFSM::pRosNewCmdStateStr, this, std::placeholders::_1));
	p_pub_state = cfg.create_publisher<std_msgs::msg::UInt8>("platform_state", 10);
	p_pub_state_str = cfg.create_publisher<std_msgs::msg::String>("platform_state_str", 10);
	// initialize the control layer, which handles the access control staff
	this->set_rate(p_hz);
	this->set_supported_service(*this, "urn:jaus:jss:iop:PlatformState", 1, 255);
	this->set_event_name("platform state");
}

void PlatformStateClient_ReceiveFSM::register_events(JausAddress remote_addr, double hz)
{
	pEventsClient_ReceiveFSM->create_event(*this, remote_addr, p_query_platform_state_msg, p_hz);
}

void PlatformStateClient_ReceiveFSM::unregister_events(JausAddress remote_addr)
{
	pEventsClient_ReceiveFSM->cancel_event(*this, remote_addr, p_query_platform_state_msg);
	stop_query(remote_addr);
}

void PlatformStateClient_ReceiveFSM::send_query(JausAddress remote_addr)
{
	sendJausMessage(p_query_platform_state_msg, remote_addr);
}

void PlatformStateClient_ReceiveFSM::stop_query(JausAddress remote_addr)
{
	p_state = PLATFORM_STATE_UNKNOWN;
	p_publish_state(p_state);
}

void PlatformStateClient_ReceiveFSM::event(JausAddress sender, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata)
{
	ReportPlatformState report;
	report.decode(reportdata);
	Receive::Body::ReceiveRec transport_data;
	transport_data.setSrcSubsystemID(sender.getSubsystemID());
	transport_data.setSrcNodeID(sender.getNodeID());
	transport_data.setSrcComponentID(sender.getComponentID());
	handleReportPlatformStateAction(report, transport_data);
}

void PlatformStateClient_ReceiveFSM::handleConfirmPlatformStateRequestAction(ConfirmPlatformStateRequest msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	int state = msg.getBody()->getConfirmPlatformStateRec()->getPlatformState();
	if (msg.getBody()->getConfirmPlatformStateRec()->getResponseCode() == INVALID_STATE) {
		RCLCPP_WARN(logger, "%s can't change to platform state: %d (%s)", sender.str().c_str(), state, p_state2str(state).c_str());
	} else {
		p_state = state;
		RCLCPP_DEBUG(logger, "%s confirm change to platform state: %d (%s)", sender.str().c_str(), p_state, p_state2str(p_state).c_str());
		p_publish_state(p_state);
		if (p_class_interface_callback != nullptr) {
			RCLCPP_DEBUG(logger, "  forward to handler");
			p_class_interface_callback(sender, p_state);
		}
	}
}

void PlatformStateClient_ReceiveFSM::handleReportPlatformStateAction(ReportPlatformState msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	int state = msg.getBody()->getPlatformStateRec()->getPlatformState();
	if (state != p_state) {
		p_state = state;
		RCLCPP_DEBUG(logger, "new platform state on %s, state: %d (%s)", sender.str().c_str(), p_state, p_state2str(p_state).c_str());
		p_publish_state(p_state);
		if (p_class_interface_callback != nullptr) {
			RCLCPP_DEBUG(logger, "  forward to handler");
			p_class_interface_callback(sender, p_state);
		}
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
}

void PlatformStateClient_ReceiveFSM::pRosNewCmdState(const std_msgs::msg::UInt8::SharedPtr msg)
{
	if ((has_access() || msg->data == OPERATIONAL) && has_remote_addr()) {
		RCLCPP_DEBUG(logger, "set new state (%d) '%s'", msg->data, p_state2str(msg->data).c_str());
		set_state(p_remote_addr, msg->data);
	} else {
		RCLCPP_WARN(logger, "has no access rights to set a new state");
	}
}

void PlatformStateClient_ReceiveFSM::pRosNewCmdStateStr(const std_msgs::msg::String::SharedPtr msg)
{
	int state = p_state2int(msg->data);
	if ((has_access() || state == OPERATIONAL) && has_remote_addr()) {
		RCLCPP_DEBUG(logger, "set new state '%s'(%d)", msg->data.c_str(), state);
		set_state(p_remote_addr, state);
	} else {
		RCLCPP_WARN(logger, "has no access rights to set a new state");
	}
}

void PlatformStateClient_ReceiveFSM::p_publish_state(int state)
{
	if (p_pub_state->get_subscription_count() > 0 || p_pub_state_str->get_subscription_count() > 0) {
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

std::string PlatformStateClient_ReceiveFSM::p_state2str(int state)
{
	std::string result = "UNKNOWN";
	switch (state) {
		case INITIALIZE: {
			result = "Initialize";
			break;
		}
		case OPERATIONAL: {
			result = "Operational";
			break;
		}
		case SHUTDOWN: {
			result = "Shutdown";
			break;
		}
		case SYSTEM_ABORT: {
			result = "System_Abort";
			break;
		}
		case EMERGENCY: {
			result = "Emergency";
			break;
		}
		case RENDER_USELESS: {
			result = "Render_Useless";
			break;
		}
	}
	return result;
}

int PlatformStateClient_ReceiveFSM::p_state2int(std::string state)
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
