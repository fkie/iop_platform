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


#ifndef PLATFORMSTATE_PLATFORMSTATEFSM_H
#define PLATFORMSTATE_PLATFORMSTATEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_iop_PlatformState_1_0/Messages/MessageSet.h"
#include "urn_jaus_jss_iop_PlatformState_1_0/InternalEvents/InternalEventsSet.h"

typedef JTS::Receive_1_0 Receive;
typedef JTS::Send_1_0 Send;



#include "PlatformState_PlatformStateFSM_sm.h"

namespace urn_jaus_jss_iop_PlatformState_1_0
{

class DllExport PlatformState_PlatformStateFSM : public JTS::StateMachine
{
public:
	PlatformState_PlatformStateFSM();
	virtual ~PlatformState_PlatformStateFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void SendAction(std::string arg0, std::string arg1);
	virtual void transitionPlatformStateAction(std::string arg0);


	/// Guard Methods



	PlatformState_PlatformStateFSMContext *context;

protected:

    /// References to parent FSMs


};

};

#endif // PLATFORMSTATE_PLATFORMSTATEFSM_H
