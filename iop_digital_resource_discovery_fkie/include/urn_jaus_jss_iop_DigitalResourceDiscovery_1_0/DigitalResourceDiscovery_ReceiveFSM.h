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


#ifndef DIGITALRESOURCEDISCOVERY_RECEIVEFSM_H
#define DIGITALRESOURCEDISCOVERY_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_iop_DigitalResourceDiscovery_1_0/Messages/MessageSet.h"
#include "urn_jaus_jss_iop_DigitalResourceDiscovery_1_0/InternalEvents/InternalEventsSet.h"

typedef JTS::Receive_1_0 Receive;
typedef JTS::Send_1_0 Send;

#include "urn_jaus_jss_core_Transport_1_0/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events_1_0/Events_ReceiveFSM.h"


#include "DigitalResourceDiscovery_ReceiveFSM_sm.h"
#include "DigitalResourceEndpoint.h"

namespace urn_jaus_jss_iop_DigitalResourceDiscovery_1_0
{

class DllExport DigitalResourceDiscovery_ReceiveFSM : public JTS::StateMachine
{
public:
	DigitalResourceDiscovery_ReceiveFSM(urn_jaus_jss_core_Transport_1_0::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events_1_0::Events_ReceiveFSM* pEvents_ReceiveFSM);
	virtual ~DigitalResourceDiscovery_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void addAndConfirmDigitalResourceEndpointAction(RegisterDigitalResourceEndpoint msg, Receive::Body::ReceiveRec transportData);
	virtual void removeAndConfirmDigitalResourceEndpointAction(RemoveDigitalResourceEndpoint msg, Receive::Body::ReceiveRec transportData);
	virtual void reportDigitalResourceEndpointAction(Receive::Body::ReceiveRec transportData);


	/// Guard Methods



	DigitalResourceDiscovery_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport_1_0::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_Events_1_0::Events_ReceiveFSM* pEvents_ReceiveFSM;
	std::map<unsigned char, digital_resource_endpoint::DigitalResourceEndpoint> p_known_endpoints;
	bool pHasEndpoint(digital_resource_endpoint::DigitalResourceEndpoint endpoint);
	unsigned char pGetFreeID();
	unsigned char pGetEndpointById(JausAddress iop_id, unsigned short int resource_id);

};

};

#endif // DIGITALRESOURCEDISCOVERY_RECEIVEFSM_H
