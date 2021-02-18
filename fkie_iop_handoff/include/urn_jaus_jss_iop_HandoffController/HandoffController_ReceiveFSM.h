

#ifndef HANDOFFCONTROLLER_RECEIVEFSM_H
#define HANDOFFCONTROLLER_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_iop_HandoffController/Messages/MessageSet.h"
#include "urn_jaus_jss_iop_HandoffController/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include <urn_jaus_jss_core_AccessControlClient/AccessControlClientService.h>

#include "HandoffController_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>
#include <fkie_iop_component/timer.hpp>

#include <fkie_iop_msgs/msg/handoff_request.hpp>
#include <fkie_iop_msgs/msg/handoff_response.hpp>

namespace urn_jaus_jss_iop_HandoffController
{

class DllExport HandoffController_ReceiveFSM : public JTS::StateMachine
{
public:
	HandoffController_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~HandoffController_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
	virtual void processConfirmHandoffRequestAction(ConfirmHandoffRequest msg, Receive::Body::ReceiveRec transportData);
	virtual void processHandoffRequestsAction(RequestReleaseControl msg, Receive::Body::ReceiveRec transportData);
	virtual void processReportEnhancedTimeoutAction(ReportEnhancedTimeout msg, Receive::Body::ReceiveRec transportData);
	virtual void processReportHandoffTimeoutAction(ReportHandoffTimeout msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods



	HandoffController_ReceiveFSMContext *context;

protected:

	/// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM *p_accesscontrol_client;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;
	typedef std::recursive_mutex mutex_type;
	typedef std::unique_lock<mutex_type> lock_type;
	mutable mutex_type p_mutex;

	JausAddress p_remote_addr;
	iop::Timer p_timer;
	rclcpp::Subscription<fkie_iop_msgs::msg::HandoffRequest>::SharedPtr p_sub_handoff_request;
	rclcpp::Subscription<fkie_iop_msgs::msg::HandoffResponse>::SharedPtr p_sub_handoff_response;
	rclcpp::Publisher<fkie_iop_msgs::msg::HandoffResponse>::SharedPtr p_pub_handoff_response;
	rclcpp::Publisher<fkie_iop_msgs::msg::HandoffRequest>::SharedPtr p_pub_handoff_request;
	std::vector<JausAddress> p_rejected_addresses;
	std::vector<JausAddress> p_in_request;
	std::vector<JausAddress> p_remote_requests;
	/// Clients must re-request handoff to prevent being denied handoff request when the timeout expires. A value of zero indicates this feature is disabled.
	int64_t p_enhanced_timeout;
	bool p_auto_request;
	uint8_t p_auto_authority;
	std::string p_auto_explanation;

	void p_accesscontrolclient_reply_handler(JausAddress &address, uint8_t code);
	void p_subscribe_accesscontrolclient();

	void p_ros_handoff_request(const fkie_iop_msgs::msg::HandoffRequest::SharedPtr msg);
	void p_ros_handoff_response(const fkie_iop_msgs::msg::HandoffResponse::SharedPtr msg);

	std::string p_code2str(uint8_t code);
	void p_timeout();
	void p_update_handoff_requests();

	template<class T>
	void p_checked_add(std::vector<T> &list, T &item) {
		typename std::vector<T>::iterator it = std::find(list.begin(), list.end(), item);
		if (it == list.end()) {
			list.push_back(item);
		}
	}
	template<class T>
	void p_checked_remove(std::vector<T> &list, T &item) {
		typename std::vector<T>::iterator it = std::find(list.begin(), list.end(), item);
		if (it != list.end()) {
			list.erase(it);
		}
	}
};

}

#endif // HANDOFFCONTROLLER_RECEIVEFSM_H
