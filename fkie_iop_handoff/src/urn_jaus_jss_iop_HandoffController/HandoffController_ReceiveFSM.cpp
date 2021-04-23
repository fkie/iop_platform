#include <algorithm>

#include "urn_jaus_jss_iop_HandoffController/HandoffController_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"
#include <fkie_iop_component/iop_component.hpp>
#include <fkie_iop_component/iop_config.hpp>
#include <fkie_iop_ocu_slavelib/Slave.h>


using namespace JTS;
using namespace urn_jaus_jss_core_AccessControlClient;

namespace urn_jaus_jss_iop_HandoffController
{



HandoffController_ReceiveFSM::HandoffController_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: logger(cmp->get_logger().get_child("HandoffController")),
  p_timer(std::chrono::seconds(10), std::bind(&HandoffController_ReceiveFSM::p_timeout, this), false)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new HandoffController_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
	this->p_accesscontrol_client = nullptr;
	p_enhanced_timeout = 10;
	p_auto_request = false;
	p_auto_authority = 255;
	p_auto_explanation = "";
}



HandoffController_ReceiveFSM::~HandoffController_ReceiveFSM()
{
	p_timer.stop();
	delete context;
}

void HandoffController_ReceiveFSM::setupNotifications()
{
	pTransport_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_HandoffController_ReceiveFSM_Receiving", "Transport_ReceiveFSM");
	registerNotification("Receiving", pTransport_ReceiveFSM->getHandler(), "InternalStateChange_To_Transport_ReceiveFSM_Receiving", "HandoffController_ReceiveFSM");
}


void HandoffController_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "HandoffController");
	cfg.declare_param<uint8_t>("enhanced_timeout", p_enhanced_timeout, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER,
		"Timeout in seconds.",
		"Default: 10 sec");
	cfg.declare_param<bool>("auto_request", p_auto_request, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_BOOL,
		"Requests automatically handoff on INSUFFICIENT_AUTHORITY.",
		"Default: false");
	cfg.declare_param<uint8_t>("auto_authority", p_auto_authority, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER,
		"Authority for auto requests.",
		"Default: 255");
	cfg.declare_param<std::string>("auto_explanation", p_auto_explanation, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"Explanation text for auto requests.",
		"Default: \"\"");

	uint64_t etimeout = p_enhanced_timeout;
	cfg.param("enhanced_timeout", etimeout, etimeout);
	p_enhanced_timeout = etimeout;
	cfg.param("auto_request", p_auto_request, p_auto_request);
	uint8_t eauthority = p_auto_authority;
	cfg.param("auto_authority", eauthority, eauthority);
	p_auto_authority = eauthority;
	cfg.param("auto_explanation", p_auto_explanation, p_auto_explanation);
	//create ROS subscriber
	p_pub_handoff_response = cfg.create_publisher<fkie_iop_msgs::msg::HandoffResponse>("handoff_remote_response", 10);
	p_pub_handoff_request = cfg.create_publisher<fkie_iop_msgs::msg::HandoffRequest>("handoff_remote_request", 10);
	p_sub_handoff_request = cfg.create_subscription<fkie_iop_msgs::msg::HandoffRequest>("handoff_own_request", 10, std::bind(&HandoffController_ReceiveFSM::p_ros_handoff_request, this, std::placeholders::_1));
	p_sub_handoff_response = cfg.create_subscription<fkie_iop_msgs::msg::HandoffResponse>("handoff_own_response", 10, std::bind(&HandoffController_ReceiveFSM::p_ros_handoff_response, this, std::placeholders::_1));
	std::shared_ptr<iop::ocu::Slave> slave = iop::ocu::Slave::get_instance(cmp);
	slave->set_supported_handoff(true);
	p_subscribe_accesscontrolclient();
	if (p_auto_request && p_enhanced_timeout > 0) {
		p_timer.set_rate(p_enhanced_timeout / 3.0);
	} else {
		p_timer.set_rate(10);
	}
}

void HandoffController_ReceiveFSM::processConfirmHandoffRequestAction(ConfirmHandoffRequest msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	auto ros_msg = fkie_iop_msgs::msg::HandoffResponse();
	unsigned char code = msg.getBody()->getConfirmHandoffRequestRec()->getResponseCode();
	unsigned char id = msg.getBody()->getConfirmHandoffRequestRec()->getID();
	RCLCPP_DEBUG(logger, "received handoff confirm: %s (%d) from %s", p_code2str(code).c_str(), code, sender.str().c_str());
	ros_msg.code = code;
	ros_msg.request_id = id;
	ros_msg.component.subsystem_id = sender.getSubsystemID();
	ros_msg.component.node_id = sender.getNodeID();
	ros_msg.component.component_id = sender.getComponentID();
	p_pub_handoff_response->publish(ros_msg);
	bool remove = true;
	if (code == 7) {  // where is no code for WAIT in the confirm message, we use a new one!
		// do nothing, still send requests
		remove = false;
	} else if (code == 4) {  // QUEUED
		// do nothing, still send requests
		remove = false;
	} else if (code == 0) {
		// GRANTED, send access control request
		if (p_accesscontrol_client != NULL) {
			p_accesscontrol_client->requestAccess(sender, p_auto_authority);
		}
	}  // on all other codes we remove from requests
	if (remove) {
		lock_type lock(p_mutex);
		// remove from requests, which are send on timeout
		p_checked_remove(p_in_request, sender);
		p_checked_remove(p_rejected_addresses, sender);
	}
}

void HandoffController_ReceiveFSM::processHandoffRequestsAction(RequestReleaseControl msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	std::vector<JausAddress> current_list;
	for (unsigned int i = 0; i < msg.getBody()->getRequestReleaseControlList()->getNumberOfElements(); i++) {
		RequestReleaseControl::Body::RequestReleaseControlList::RequestReleaseControlRec *rr_rec = msg.getBody()->getRequestReleaseControlList()->getElement(i);
		auto ros_msg = fkie_iop_msgs::msg::HandoffRequest();
		ros_msg.request = true;
		ros_msg.authority_code = rr_rec->getAuthorityCode();
		ros_msg.explanation = rr_rec->getExplanation();
		ros_msg.request_id = rr_rec->getID();
		RCLCPP_DEBUG(logger, "received handoff request for %d.%d.%d, auth: %d from %s, explanation: %s",
				rr_rec->getSrcSubsystemID(), rr_rec->getSrcNodeID(), rr_rec->getSrcComponentID(), rr_rec->getAuthorityCode(), sender.str().c_str(), rr_rec->getExplanation().c_str());
		ros_msg.component.subsystem_id = transportData.getSrcSubsystemID();
		ros_msg.component.node_id = transportData.getSrcNodeID();
		ros_msg.component.component_id = transportData.getSrcComponentID();
		ros_msg.ocu.subsystem_id = rr_rec->getSrcSubsystemID();
		ros_msg.ocu.node_id = rr_rec->getSrcNodeID();
		ros_msg.ocu.component_id = rr_rec->getSrcComponentID();
		JausAddress ocu_addr(rr_rec->getSrcSubsystemID(), rr_rec->getSrcNodeID(), rr_rec->getSrcComponentID());
		current_list.push_back(ocu_addr);
		p_pub_handoff_request->publish(ros_msg);
	}
	// publish for each removed request a handoff request with request=false
	std::vector<JausAddress>::iterator it;
	for (it = p_remote_requests.begin(); it != p_remote_requests.end(); ++it) {
		std::vector<JausAddress>::iterator search_it = std::find(current_list.begin(), current_list.end(), *it);
		if (search_it == current_list.end()) {
			auto ros_msg = fkie_iop_msgs::msg::HandoffRequest();
			ros_msg.request = false;
			ros_msg.authority_code = 255;
			ros_msg.explanation = "not in the queue list anymore";
			ros_msg.request_id = 255;
			RCLCPP_DEBUG(logger, "  remove handoff request for %s, from %s", it->str().c_str(), sender.str().c_str());
			ros_msg.component.subsystem_id = transportData.getSrcSubsystemID();
			ros_msg.component.node_id = transportData.getSrcNodeID();
			ros_msg.component.component_id = transportData.getSrcComponentID();
			ros_msg.ocu.subsystem_id = it->getSubsystemID();
			ros_msg.ocu.node_id = it->getNodeID();
			ros_msg.ocu.component_id = it->getComponentID();
			p_pub_handoff_request->publish(ros_msg);
		}
	}
	p_remote_requests = std::vector<JausAddress>(current_list);
}

void HandoffController_ReceiveFSM::processReportEnhancedTimeoutAction(ReportEnhancedTimeout msg, Receive::Body::ReceiveRec transportData)
{
	unsigned char timeoput = msg.getBody()->getReportEnhancedTimeoutRec()->getTimeout();
	if (p_enhanced_timeout > msg.getBody()->getReportEnhancedTimeoutRec()->getTimeout()) {
		JausAddress sender = transportData.getAddress();
		RCLCPP_DEBUG(logger, "update EnhancedTimeout to %d from %s", timeoput, sender.str().c_str());
		p_enhanced_timeout = timeoput;
		// restart timer, if already running
		p_timer.stop();
		if (p_enhanced_timeout != 0) {
			RCLCPP_DEBUG(logger, "create timer with new period %.2f", p_enhanced_timeout / 3.0);
			p_timer.set_rate(p_enhanced_timeout / 3.0);
			p_timer.start();
		}
	}
}

void HandoffController_ReceiveFSM::processReportHandoffTimeoutAction(ReportHandoffTimeout msg, Receive::Body::ReceiveRec transportData)
{
	/// this should not happen
}

void HandoffController_ReceiveFSM::p_subscribe_accesscontrolclient()
{
	if (p_accesscontrol_client == nullptr) {
		AccessControlClientService *accesscontrol_srv = static_cast<AccessControlClientService*>(cmp->get_service("AccessControlClientService"));
		if (accesscontrol_srv != NULL) {
			p_accesscontrol_client = accesscontrol_srv->pAccessControlClient_ReceiveFSM;
			p_accesscontrol_client->add_reply_handler(&HandoffController_ReceiveFSM::p_accesscontrolclient_reply_handler, this);
		} else {
			RCLCPP_WARN_ONCE(logger, "no AccessControlClient found! Please include its plugin first (in the list), if you needs one!");
		}
	}
}

void HandoffController_ReceiveFSM::p_accesscontrolclient_reply_handler(JausAddress &address, uint8_t code)
{
	lock_type lock(p_mutex);
	RCLCPP_DEBUG(logger, "access control reply from %s, code: %d", address.str().c_str(), (int)code);
	if (code == AccessControlClient_ReceiveFSM::ACCESS_STATE_INSUFFICIENT_AUTHORITY) {
		RCLCPP_DEBUG(logger, "code %d is ACCESS_STATE_INSUFFICIENT_AUTHORITY", (int)code);
		if (std::find(p_rejected_addresses.begin(), p_rejected_addresses.end(), address) == p_rejected_addresses.end()) {
			RCLCPP_DEBUG(logger, "add to rejected addresses %s", address.str().c_str());
			p_rejected_addresses.push_back(address);
			if (p_auto_request) {
				p_checked_add(p_in_request, address);
			}
		}
	} else {
		p_checked_remove(p_rejected_addresses, address);
		if (p_auto_request) {
			// remove from requests
			p_checked_remove(p_in_request, address);
		}
	}

}

void HandoffController_ReceiveFSM::p_ros_handoff_request(const fkie_iop_msgs::msg::HandoffRequest::SharedPtr msg)
{
	lock_type lock(p_mutex);
	if (msg->request) {
		p_auto_request = true;
		p_auto_authority = msg->authority_code;
		p_auto_explanation = msg->explanation;
		// p_in_request = std::vector<JausAddress>(p_rejected_addresses);
		JausAddress component(msg->component.subsystem_id, msg->component.node_id, msg->component.component_id);
		RCLCPP_DEBUG(logger, "received handoff request from ROS for %s", component.str().c_str());
		p_checked_add(p_in_request, component);
		p_update_handoff_requests();
		if (p_enhanced_timeout != 0) {
			RCLCPP_DEBUG(logger, "create timer with new period %.2f", p_enhanced_timeout / 3.0);
			p_timer.set_rate(p_enhanced_timeout / 3.0);
			p_timer.start();
		}
	} else {
		RCLCPP_DEBUG(logger, "received request to cancel handoff requests, stop timer. No RemoveHandoffRequest is send!");
		p_auto_request = false;
		p_timer.stop();
		// remove requests
		// we do not send RemoveHandoffRequest, because we did not saved ID. The request should be removed after a timeout by EnhancedAccessControl.
		p_in_request.clear();
	}
}

void HandoffController_ReceiveFSM::p_ros_handoff_response(const fkie_iop_msgs::msg::HandoffResponse::SharedPtr msg)
{
	ConfirmReleaseControl reply;
	ConfirmReleaseControl::Body::ReleaseControlList::ReleaseControlRec rcrec;
	rcrec.setID(msg->request_id);
	if (msg->code == 0) {
		rcrec.setResponseCode(0);  // GRANTED in ConfirmReleaseControl
	} else if (msg->code == 7 || msg->code == 4) {
		rcrec.setResponseCode(2);  // WAIT in ConfirmReleaseControl
	} else {
		rcrec.setResponseCode(1);  // DENIED in ConfirmReleaseControl
	}
	reply.getBody()->getReleaseControlList()->addElement(rcrec);
	JausAddress address(msg->component.subsystem_id, msg->component.node_id, msg->component.component_id);
	RCLCPP_DEBUG(logger, "send handoff response to %s, code: %d", address.str().c_str(), rcrec.getID());
	sendJausMessage(reply, address);
}

std::string HandoffController_ReceiveFSM::p_code2str(uint8_t code)
{
	std::string result = "UNKNOWN CODE";
	switch (code) {
	case 0: result = "GRANTED";
			break;
	case 1: result = "NOT_AVAILABLE";
			break;
	case 2: result = "TIMEOUT";
			break;
	case 3: result = "DENIED";
			break;
	case 4: result = "QUEUED";
			break;
	case 5: result = "DEFERRED";
			break;
	case 6: result = "INSUFFICIENT_AUTHORITY";
			break;
	case 7: result = "WAIT";
			break;
	}
	return result;
}

void HandoffController_ReceiveFSM::p_timeout()
{
	p_update_handoff_requests();
}

void HandoffController_ReceiveFSM::p_update_handoff_requests()
{
	lock_type lock(p_mutex);
	if (p_auto_request) {
		RequestHandoff request;
		request.getBody()->getRequestHandoffRec()->setAuthorityCode(p_auto_authority);
		request.getBody()->getRequestHandoffRec()->setExplanation(p_auto_explanation);
		std::vector<JausAddress>::iterator it;
		for (it = p_in_request.begin(); it != p_in_request.end(); it++) {
			QueryEnhancedTimeout qt;
			sendJausMessage(qt, *it);
			RCLCPP_DEBUG(logger, "send handoff request to %s, authority: %d, explanation: %s", it->str().c_str(), (int)p_auto_authority, p_auto_explanation.c_str());
			this->sendJausMessage(request, *it);
		}
	}
}

}
