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
#include "urn_jaus_jss_iop_DigitalResourceDiscovery/Messages/MessageSet.h"
#include "urn_jaus_jss_iop_DigitalResourceDiscovery/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"


#include "DigitalResourceDiscovery_ReceiveFSM_sm.h"
#include <iop_digital_resource_discovery_fkie/DigitalResourceEndpoint.h>

namespace urn_jaus_jss_iop_DigitalResourceDiscovery
{

class DllExport DigitalResourceDiscovery_ReceiveFSM : public JTS::StateMachine
{
public:
	DigitalResourceDiscovery_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM);
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
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	std::map<unsigned char, digital_resource_endpoint::DigitalResourceEndpoint> p_known_endpoints;
	bool pHasEndpoint(digital_resource_endpoint::DigitalResourceEndpoint endpoint);
	unsigned char pGetFreeID();
	unsigned char pGetEndpointById(JausAddress iop_id, unsigned short int resource_id);

};

};

#endif // DIGITALRESOURCEDISCOVERY_RECEIVEFSM_H
