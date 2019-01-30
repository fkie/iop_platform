

#ifndef HANDOFFCONTROLLER_RECEIVEFSM_H
#define HANDOFFCONTROLLER_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_iop_HandoffController/Messages/MessageSet.h"
#include "urn_jaus_jss_iop_HandoffController/InternalEvents/InternalEventsSet.h"
#include "urn_jaus_jss_iop_EnhancedAccessControl/Messages/MessageSet.h"
#include "urn_jaus_jss_iop_EnhancedAccessControl/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include <urn_jaus_jss_core_AccessControlClient/AccessControlClientService.h>


#include <boost/thread/recursive_mutex.hpp>
#include <ros/ros.h>
#include <fkie_iop_msgs/HandoffRequest.h>
#include <fkie_iop_msgs/HandoffResponse.h>

#include "HandoffController_ReceiveFSM_sm.h"

namespace urn_jaus_jss_iop_HandoffController
{

class DllExport HandoffController_ReceiveFSM : public JTS::StateMachine
{
public:
	HandoffController_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~HandoffController_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

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

	typedef boost::recursive_mutex mutex_type;
	typedef boost::unique_lock<mutex_type> lock_type;
	mutable mutex_type p_mutex;

	JausAddress p_remote_addr;
	ros::NodeHandle p_nh;
	ros::Timer p_timeout_timer;
	ros::Subscriber p_sub_handoff_request;
	ros::Subscriber p_sub_handoff_response;
	ros::Publisher p_pub_handoff_response;
	ros::Publisher p_pub_handoff_request;
	std::vector<JausAddress> p_rejected_addresses;
	std::vector<JausAddress> p_in_request;
	std::vector<JausAddress> p_remote_requests;
	/// Clients must re-request handoff to prevent being denied handoff request when the timeout expires. A value of zero indicates this feature is disabled.
	unsigned char p_enhanced_timeout;
	bool p_auto_request;
	unsigned char p_auto_authority;
	std::string p_auto_explanation;

	void p_accesscontrolclient_reply_handler(JausAddress &address, unsigned char code);
	void p_subscribe_accesscontrolclient();

	void p_ros_handoff_request(const fkie_iop_msgs::HandoffRequest::ConstPtr msg);
	void p_ros_handoff_response(const fkie_iop_msgs::HandoffResponse::ConstPtr msg);

	std::string p_code2str(unsigned char code);
	void p_timeout(const ros::TimerEvent& event);
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

};

#endif // HANDOFFCONTROLLER_RECEIVEFSM_H
