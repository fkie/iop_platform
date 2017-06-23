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

#include <algorithm>

#include "urn_jaus_jss_iop_DigitalResourceDiscoveryClient/DigitalResourceDiscoveryClient_ReceiveFSM.h"


#include <ros/console.h>

using namespace JTS;
using namespace digital_resource_endpoint;

namespace urn_jaus_jss_iop_DigitalResourceDiscoveryClient
{



DigitalResourceDiscoveryClient_ReceiveFSM::DigitalResourceDiscoveryClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new DigitalResourceDiscoveryClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	p_request_id = 0;
	p_enable_ros_interface = false;
	p_pnh = ros::NodeHandle("~");
}



DigitalResourceDiscoveryClient_ReceiveFSM::~DigitalResourceDiscoveryClient_ReceiveFSM()
{
	delete context;
}

void DigitalResourceDiscoveryClient_ReceiveFSM::setupNotifications()
{
	pEventsClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_DigitalResourceDiscoveryClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
	pEventsClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_DigitalResourceDiscoveryClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving_Ready", "DigitalResourceDiscoveryClient_ReceiveFSM");
	registerNotification("Receiving", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving", "DigitalResourceDiscoveryClient_ReceiveFSM");
	p_pnh.param("enable_ros_interface", p_enable_ros_interface, p_enable_ros_interface);
	ROS_INFO_STREAM("[DigitalResourceDiscoveryClient] enable_ros_interface: " << p_enable_ros_interface);
	if (p_enable_ros_interface) {
		p_pub_endoints = p_nh.advertise<iop_msgs_fkie::DigitalResourceEndpoints>("endpoints", 1, true);
		p_srv_update_endpoints = p_nh.advertiseService("update_endpoints", &DigitalResourceDiscoveryClient_ReceiveFSM::pUpdateEndpointsSrv, this);
	}
}

bool DigitalResourceDiscoveryClient_ReceiveFSM::pUpdateEndpointsSrv(iop_msgs_fkie::QueryByAddr::Request	&req, iop_msgs_fkie::QueryByAddr::Response &res)
{
	ROS_DEBUG_NAMED("DigitalResourceDiscoveryClient", "update endpoints of %d.%d.%d", req.address.subsystem_id, req.address.node_id, req.address.component_id);
	discoverEndpoints(JausAddress(req.address.subsystem_id, req.address.node_id, req.address.component_id));
	return true;
}

void DigitalResourceDiscoveryClient_ReceiveFSM::confirmDigitalResourceEndpointAction(ConfirmDigitalResourceEndpoint msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	bool confirmed = false;
	unsigned char server_id = msg.getBody()->getConfirmDigitalResourceEndpointRec()->getID();
	unsigned char request_id = msg.getBody()->getConfirmDigitalResourceEndpointRec()->getRequestID();
	// check, is it a registration confirmation
	for (std::deque<DigitalResourceEndpoint>::iterator it = p_toregister_endpoints.begin(); it!=p_toregister_endpoints.end(); ++it) {
		if (it->request_id == request_id) {
			DigitalResourceEndpoint reg = *it;
			reg.request_id = server_id;
			p_registered_endpoints.push_back(reg);
			p_toregister_endpoints.erase(it);
			ROS_INFO_NAMED("DigitalResourceDiscoveryClient", "registration for %s confirmed!", reg.server_url.c_str());
			confirmed = true;
			break;
		}
	}
	// check, is it a unregistration confirmation
	for (std::deque<DigitalResourceEndpoint>::iterator it = p_tounregister_endpoints.begin(); it!=p_tounregister_endpoints.end(); ++it) {
		if (it->request_id == request_id) {
			DigitalResourceEndpoint reg = *it;
			p_tounregister_endpoints.erase(it);
			ROS_INFO_NAMED("DigitalResourceDiscoveryClient", "unregistration for %s confirmed!", reg.server_url.c_str());
			confirmed = true;
			break;
		}
	}
	if (!confirmed) {
		ROS_WARN_NAMED("DigitalResourceDiscoveryClient", "received confirmation for wrong request_id: %d", request_id);
	}
}

void DigitalResourceDiscoveryClient_ReceiveFSM::reportDigitalResourceEndpointAction(ReportDigitalResourceEndpoint msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_DEBUG_NAMED("DigitalResourceDiscoveryClient", "reportDigitalResourceEndpointAction from %d.%d.%d",
			subsystem_id, node_id, component_id);
	ReportDigitalResourceEndpoint::Body::ReportFileTransferEndpointList *endpoints = msg.getBody()->getReportFileTransferEndpointList();
	std::vector<digital_resource_endpoint::DigitalResourceEndpoint> v_endpoints;
	for (unsigned int ei = 0; ei < endpoints->getNumberOfElements(); ei++) {
		DigitalResourceEndpoint endpoint;
		ReportDigitalResourceEndpoint::Body::ReportFileTransferEndpointList::DigitalResourceEndpointRec *ep = endpoints->getElement(ei);
		endpoint.server_url = ep->getServerURL();
		endpoint.server_type = ep->getServerType();
		endpoint.resource_id = ep->getResourceID();
		endpoint.iop_id = JausAddress(ep->getIOP_ID()->getSubsystemID(), ep->getIOP_ID()->getNodeID(), ep->getIOP_ID()->getComponentID());
		v_endpoints.push_back(endpoint);
	}
	iop_msgs_fkie::DigitalResourceEndpoints ros_msg;
	for (unsigned int i = 0; i < v_endpoints.size(); i++) {
		iop_msgs_fkie::DigitalResourceEndpoint ep;
		ep.server_url = v_endpoints[i].server_url;
		ep.server_type = v_endpoints[i].server_type;
		ep.resource_id = v_endpoints[i].resource_id;
		ep.address.subsystem_id = v_endpoints[i].iop_id.getSubsystemID();
		ep.address.node_id = v_endpoints[i].iop_id.getNodeID();
		ep.address.component_id = v_endpoints[i].iop_id.getComponentID();
		ros_msg.endpoints.push_back(ep);
	}
	p_pub_endoints.publish(ros_msg);
	if (!class_discovery_callback_.empty()) {
		class_discovery_callback_(v_endpoints, sender);
	}
}

void DigitalResourceDiscoveryClient_ReceiveFSM::registerEndpoint(DigitalResourceEndpoint endpoint, const JausAddress digital_resource_discovery_service)
{
	if (pHasEndpoint(endpoint)) {
		ROS_WARN_NAMED("DigitalResourceDiscoveryClient", "endpoint %s already registered, skip registration", endpoint.server_url.c_str());
		return;
	}
	ROS_INFO_NAMED("DigitalResourceDiscoveryClient", "register endpoint: %s", endpoint.server_url.c_str());
	// register the digital video endpoint by DigitalResourceDiscovery service
	RegisterDigitalResourceEndpoint msg;
	endpoint.request_id = p_request_id;
	p_request_id++;
	msg.getBody()->getRegisterDigitalResourceSeq()->getRequestIDRec()->setRequestID(endpoint.request_id);
	RegisterDigitalResourceEndpoint::Body::RegisterDigitalResourceSeq::DigitalResourceEndpointRec *digital_resource;
	digital_resource = msg.getBody()->getRegisterDigitalResourceSeq()->getDigitalResourceEndpointRec();
	digital_resource->setServerType(endpoint.server_type);
	digital_resource->setServerURL(endpoint.server_url);
	RegisterDigitalResourceEndpoint::Body::RegisterDigitalResourceSeq::DigitalResourceEndpointRec::IOP_ID iop_id;
	iop_id.setSubsystemID(endpoint.iop_id.getSubsystemID());
	iop_id.setNodeID(endpoint.iop_id.getNodeID());
	iop_id.setComponentID(endpoint.iop_id.getComponentID());
	digital_resource->setIOP_ID(iop_id);
	digital_resource->setResourceID(endpoint.resource_id);
	if (std::find(p_toregister_endpoints.begin(), p_toregister_endpoints.end(), endpoint) == p_toregister_endpoints.end()) {
		p_toregister_endpoints.push_back(endpoint);
	}
	sendJausMessage(msg, digital_resource_discovery_service);
}

void DigitalResourceDiscoveryClient_ReceiveFSM::unregisterEndpoint(DigitalResourceEndpoint endpoint, const JausAddress digital_resource_discovery_service)
{
	std::deque<DigitalResourceEndpoint>::iterator deg = std::find(p_registered_endpoints.begin(), p_registered_endpoints.end(), endpoint);
	if (deg == p_registered_endpoints.end()) {
		ROS_WARN_NAMED("DigitalResourceDiscoveryClient", "endpoint %s is not registered, skip unregistration", endpoint.server_url.c_str());
		return;
	}
	ROS_INFO_NAMED("DigitalResourceDiscoveryClient", "unregister endpoint: %s", endpoint.server_url.c_str());
	// unregister the digital video endpoint by DigitalResourceDiscovery service
	RemoveDigitalResourceEndpoint msg;
	msg.getBody()->getRemoveDigitalResourceEndpointRec()->setID(deg->request_id);
	deg->request_id = p_request_id;
	p_request_id++;
	msg.getBody()->getRemoveDigitalResourceEndpointRec()->setRequestID(deg->request_id);
	p_tounregister_endpoints.push_back(*deg);
	p_registered_endpoints.erase(deg);
	sendJausMessage(msg, digital_resource_discovery_service);
}

void DigitalResourceDiscoveryClient_ReceiveFSM::discoverEndpoints(const JausAddress digital_resource_discovery_service)
{
	QueryDigitalResourceEndpoint qmsg;
	sendJausMessage(qmsg, digital_resource_discovery_service);
}

bool DigitalResourceDiscoveryClient_ReceiveFSM::pHasEndpoint(DigitalResourceEndpoint &endpoint)
{
	return (std::find(p_registered_endpoints.begin(), p_registered_endpoints.end(), endpoint) != p_registered_endpoints.end());
}


};
