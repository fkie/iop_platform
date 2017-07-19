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

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControl/AccessControl_ReceiveFSM.h"


#include "PlatformState_ReceiveFSM_sm.h"

namespace urn_jaus_jss_iop_PlatformState
{

class DllExport PlatformState_ReceiveFSM : public JTS::StateMachine
{
public:
	PlatformState_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM);
	virtual ~PlatformState_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

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
	virtual bool setToEmergency(SetPlatformState msg);
	virtual bool setToInitialize(SetPlatformState msg);
	virtual bool setToOperational(SetPlatformState msg);
	virtual bool setToRenderUseless(SetPlatformState msg);
	virtual bool setToShutdown(SetPlatformState msg);



	PlatformState_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;

	JausAddress p_requestor;

};

};

#endif // PLATFORMSTATE_RECEIVEFSM_H
