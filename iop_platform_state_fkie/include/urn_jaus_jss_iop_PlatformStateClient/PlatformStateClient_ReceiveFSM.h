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


#ifndef PLATFORMSTATECLIENT_RECEIVEFSM_H
#define PLATFORMSTATECLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_iop_PlatformStateClient/Messages/MessageSet.h"
#include "urn_jaus_jss_iop_PlatformStateClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"

#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <iop_ocu_slavelib_fkie/SlaveHandlerInterface.h>
#include <iop_events_fkie/EventHandlerInterface.h>

#include "PlatformStateClient_ReceiveFSM_sm.h"

namespace urn_jaus_jss_iop_PlatformStateClient
{

const int INITIALIZE = 0;
const int OPERATIONAL = 1;
const int SHUTDOWN = 2;
const int SYSTEM_ABORT = 3;
const int EMERGENCY = 4;
const int RENDER_USELESS = 5;
const int PLATFORM_STATE_UNKNOWN = 255;

const int TRANSITIONING = 0;
const int INVALID_STATE = 1;

class DllExport PlatformStateClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface, public iop::EventHandlerInterface
{
public:
	PlatformStateClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM);
	virtual ~PlatformStateClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void handleConfirmPlatformStateRequestAction(ConfirmPlatformStateRequest msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportPlatformStateAction(ReportPlatformState msg, Receive::Body::ReceiveRec transportData);


	template<class T>
	void set_state_handler(void(T::*handler)(JausAddress &, unsigned char state), T*obj) {
		p_class_interface_callback = boost::bind(handler, obj, _1, _2);
	}
	void query_state(JausAddress address);
	/**
	 * sets the platform the new state:
	 * 0: Initialize
	 * 1: Operational
	 * 2: Shutdown
	 * 3: System_Abort
	 * 4: Emergency
	 * 5: Render_Useless
	 */
	void set_state(JausAddress address, unsigned char state);

	/// Guard Methods

	/// EventHandlerInterface Methods
	void event(JausAddress reporter, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata);

	/// SlaveHandlerInterface Methods
	void control_allowed(std::string service_uri, JausAddress component, unsigned char authority);
	void enable_monitoring_only(std::string service_uri, JausAddress component);
	void access_deactivated(std::string service_uri, JausAddress component);
	void create_events(std::string service_uri, JausAddress component, bool by_query=false);
	void cancel_events(std::string service_uri, JausAddress component, bool by_query=false);


	PlatformStateClient_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;

	boost::function<void (JausAddress &, unsigned char state)> p_class_interface_callback;

	ros::NodeHandle p_nh;
	ros::Timer p_query_timer;
	ros::Subscriber p_sub_state;
	ros::Subscriber p_sub_state_str;
	ros::Publisher p_pub_state;
	ros::Publisher p_pub_state_str;
	int p_state;
	double p_hz;
	bool p_has_access;

	JausAddress p_remote_addr;
	QueryPlatformState p_query_platform_state_msg;


	void pRosNewCmdState(const std_msgs::UInt8::ConstPtr& msg);
	void pRosNewCmdStateStr(const std_msgs::String::ConstPtr& msg);
	void pQueryCallback(const ros::TimerEvent& event);
	void p_publish_state(int state);
	std::string p_state2str(int state);
	int p_state2int(std::string state);
};

};

#endif // PLATFORMSTATECLIENT_RECEIVEFSM_H
