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

typedef JTS::Receive Receive;
typedef JTS::Send Send;

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include "PlatformStateClient_ReceiveFSM_sm.h"

namespace urn_jaus_jss_iop_PlatformStateClient
{

class DllExport PlatformStateClient_ReceiveFSM : public JTS::StateMachine
{
public:
	static unsigned char PLATFORM_STATE_INIT;
	static unsigned char PLATFORM_STATE_OPERATIONAL;
	static unsigned char PLATFORM_STATE_SHUTDOWN;
	static unsigned char PLATFORM_STATE_SYSTEM_ABORT;
	static unsigned char PLATFORM_STATE_EMERGENCY;
	static unsigned char PLATFORM_STATE_RENDER_USELESS;
	static unsigned char PLATFORM_STATE_UNKNOWN;

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



	PlatformStateClient_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;

	boost::function<void (JausAddress &, unsigned char state)> p_class_interface_callback;

	unsigned char p_state;
};

};

#endif // PLATFORMSTATECLIENT_RECEIVEFSM_H
