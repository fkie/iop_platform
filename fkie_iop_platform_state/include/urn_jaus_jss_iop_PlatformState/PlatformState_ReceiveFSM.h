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


#ifndef PLATFORMSTATE_RECEIVEFSM_H
#define PLATFORMSTATE_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_iop_PlatformState/Messages/MessageSet.h"
#include "urn_jaus_jss_iop_PlatformState/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_AccessControl/AccessControl_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"


#include "PlatformState_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>

#include <urn_jaus_jss_core_Discovery/DiscoveryService.h>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace urn_jaus_jss_iop_PlatformState
{

const int INITIALIZE = 0;
const int OPERATIONAL = 1;
const int SHUTDOWN = 2;
const int SYSTEM_ABORT = 3;
const int EMERGENCY = 4;
const int RENDER_USELESS = 5;

const int TRANSITIONING = 0;
const int INVALID_STATE = 1;


class DllExport PlatformState_ReceiveFSM : public JTS::StateMachine
{
public:

	PlatformState_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~PlatformState_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
	virtual void sendReportPlatformStateAction(Receive::Body::ReceiveRec transportData);
	virtual void storeRequesterAction(Receive::Body::ReceiveRec transportData);
	virtual void triggerEmergencyAction();
	virtual void triggerRecoverEmergencyAction();
	virtual void triggerRenderUselessAction();
	virtual void triggerResetAction();
	virtual void triggerShutdownAction();


	/// Guard Methods
	virtual bool isControllingClient(Receive::Body::ReceiveRec transportData);
	virtual bool isIDStored(Receive::Body::ReceiveRec transportData);
	virtual bool setToEmergency(SetPlatformState msg);
	virtual bool setToInitialize(SetPlatformState msg);
	virtual bool setToOperational(SetPlatformState msg);
	virtual bool setToRenderUseless(SetPlatformState msg);
	virtual bool setToShutdown(SetPlatformState msg);



	PlatformState_ReceiveFSMContext *context;

protected:

	/// References to parent FSMs
	urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;

	urn_jaus_jss_core_Discovery::Discovery_ReceiveFSM *p_discovery_srv;
	JausAddress p_own_address;
	ReportPlatformState p_current_state;
	JausAddress p_requestor;
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr p_sub_state;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr p_sub_state_str;
	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr p_pub_state;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr p_pub_state_str;
	uint8_t p_init_platform_state;
	std::vector<std::string> p_supported_states;


	urn_jaus_jss_core_Discovery::Discovery_ReceiveFSM* p_get_discovery();
	void pRosNewState(const std_msgs::msg::UInt8::SharedPtr msg);
	void pRosNewStateStr(const std_msgs::msg::String::SharedPtr msg);

	int p_publish_state(int state);
	int p_get_current_state();
	std::string p_get_current_state_str();
	std::string p_state2str(int state);
	int p_state2int(std::string state);
	bool p_is_supported_state(int state);
};

}

#endif // PLATFORMSTATE_RECEIVEFSM_H
