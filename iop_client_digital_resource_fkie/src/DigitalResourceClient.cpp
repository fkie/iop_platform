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


#include "DigitalResourceClient.h"
#include <iop_ocu_slavelib_fkie/Slave.h>


using namespace iop;
using namespace urn_jaus_jss_core_Transport;
using namespace urn_jaus_jss_core_EventsClient;
using namespace urn_jaus_jss_iop_DigitalResourceDiscoveryClient;


DigitalResourceClient::DigitalResourceClient(JTS::JausRouter* jausRouter, TransportService* pTransportService, EventsClientService* pEventsClientService, DigitalResourceDiscoveryClientService* pDigitalResourceDiscoveryClientService)
{

	this->jausRouter = jausRouter;
	this->pDigitalResourceDiscoveryClientService = pDigitalResourceDiscoveryClientService;
}

DigitalResourceClient::~DigitalResourceClient()
{
}

/**
 *	This is the function that will actually be run by the thread at start-up.
 *  We override it from EventReceiver in order to handle any entry
 *  actions defined by the initial state.
 */
void DigitalResourceClient::run()
{

	/// Perform any entry actions specified by the start state.
	p_pub_endoints = p_nh.advertise<iop_msgs_fkie::DigitalResourceEndpoints>("digital_endpoints", 10, true);
	pDigitalResourceDiscoveryClientService->pDigitalResourceDiscoveryClient_ReceiveFSM->set_discovery_handler(&DigitalResourceClient::p_discovered_endpoints, this);
	ocu::Slave &slave = ocu::Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:iop:DigitalResourceDiscovery", 1, 0);

	/// Kick-off the receive loop...
	EventReceiver::run();
}

bool DigitalResourceClient::processTransitions(JTS::InternalEvent* ie)
{
	return false;
}
bool DigitalResourceClient::defaultTransitions(JTS::InternalEvent* ie)
{
	return false;
}

void DigitalResourceClient::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:iop:DigitalResourceDiscovery") == 0) {
		p_control_addr = component;
		ROS_INFO_NAMED("DigitalResourceClient", "control requested, update endpoints of %d.%d.%d",
				component.getSubsystemID(), component.getNodeID(), component.getComponentID());
		pDigitalResourceDiscoveryClientService->pDigitalResourceDiscoveryClient_ReceiveFSM->discoverEndpoints(p_control_addr);
	} else {
		ROS_WARN_STREAM("[DigitalResourceClient] unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void DigitalResourceClient::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	ROS_INFO_NAMED("DigitalResourceClient", "view requested, update endpoints of %d.%d.%d",
			component.getSubsystemID(), component.getNodeID(), component.getComponentID());
	pDigitalResourceDiscoveryClientService->pDigitalResourceDiscoveryClient_ReceiveFSM->discoverEndpoints(component);
}

void DigitalResourceClient::access_deactivated(std::string service_uri, JausAddress component)
{
	p_control_addr = JausAddress(0);
}

void DigitalResourceClient::p_discovered_endpoints(std::vector<digital_resource_endpoint::DigitalResourceEndpoint> endpoints, JausAddress &address)
{
	iop_msgs_fkie::DigitalResourceEndpoints ros_msg;
	for (unsigned int i = 0; i < endpoints.size(); i++) {
		iop_msgs_fkie::DigitalResourceEndpoint ep;
		ep.server_url = endpoints[i].server_url;
		ep.server_type = endpoints[i].server_type;
		ep.resource_id = endpoints[i].resource_id;
		ep.address.subsystem_id = endpoints[i].iop_id.getSubsystemID();
		ep.address.node_id = endpoints[i].iop_id.getNodeID();
		ep.address.component_id = endpoints[i].iop_id.getComponentID();
		ros_msg.endpoints.push_back(ep);
	}
	p_pub_endoints.publish(ros_msg);
}
