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


#ifndef DIGITALRESOURCEDISCOVERYCLIENT_RECEIVEFSM_H
#define DIGITALRESOURCEDISCOVERYCLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_iop_DigitalResourceDiscoveryClient/Messages/MessageSet.h"
#include "urn_jaus_jss_iop_DigitalResourceDiscoveryClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"


#include "DigitalResourceDiscoveryClient_ReceiveFSM_sm.h"
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <fkie_iop_digital_resource_discovery/DigitalResourceEndpoint.h>
#include <ros/ros.h>
#include <fkie_iop_msgs/DigitalResourceEndpoints.h>
#include <fkie_iop_msgs/QueryByAddr.h>


namespace urn_jaus_jss_iop_DigitalResourceDiscoveryClient
{

class DllExport DigitalResourceDiscoveryClient_ReceiveFSM : public JTS::StateMachine
{
public:
	DigitalResourceDiscoveryClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM);
	virtual ~DigitalResourceDiscoveryClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void confirmDigitalResourceEndpointAction(ConfirmDigitalResourceEndpoint msg, Receive::Body::ReceiveRec transportData);
	virtual void reportDigitalResourceEndpointAction(ReportDigitalResourceEndpoint msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods
	void registerEndpoint(digital_resource_endpoint::DigitalResourceEndpoint endpoint, const JausAddress digital_resource_discovery_service);
	void unregisterEndpoint(digital_resource_endpoint::DigitalResourceEndpoint endpoint, const JausAddress digital_resource_discovery_service);
	template<class T>
	void set_discovery_handler(void(T::*handler)(std::vector<digital_resource_endpoint::DigitalResourceEndpoint>, JausAddress &), T*obj) {
		class_discovery_callback_ = boost::bind(handler, obj, _1, _2);
	}
	void discoverEndpoints(const JausAddress digital_resource_discovery_service);

	DigitalResourceDiscoveryClient_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	std::deque<digital_resource_endpoint::DigitalResourceEndpoint> p_toregister_endpoints;
	std::deque<digital_resource_endpoint::DigitalResourceEndpoint> p_tounregister_endpoints;
	std::deque<digital_resource_endpoint::DigitalResourceEndpoint> p_registered_endpoints;
	unsigned char p_request_id;
	boost::function<void (std::vector<digital_resource_endpoint::DigitalResourceEndpoint>, JausAddress &)> class_discovery_callback_;
	bool p_enable_ros_interface;
	ros::NodeHandle p_nh;
	ros::Publisher p_pub_endoints;
	ros::ServiceServer p_srv_update_endpoints;

	bool pUpdateEndpointsSrv(fkie_iop_msgs::QueryByAddr::Request  &req, fkie_iop_msgs::QueryByAddr::Response &res);
	bool pHasEndpoint(digital_resource_endpoint::DigitalResourceEndpoint &endpoint);

};

};

#endif // DIGITALRESOURCEDISCOVERYCLIENT_RECEIVEFSM_H
