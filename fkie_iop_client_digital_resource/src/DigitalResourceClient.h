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


#ifndef DIGITAL_RESOURCE_CLIENT_H
#define DIGITAL_RESOURCE_CLIENT_H

#include <string>
#include "Transport/JausAddress.h"

#include "urn_jaus_jss_core_Transport/TransportService.h"
#include "urn_jaus_jss_core_EventsClient/EventsClientService.h"
#include "urn_jaus_jss_iop_DigitalResourceDiscoveryClient/DigitalResourceDiscoveryClientService.h"

#include <ros/ros.h>
#include <fkie_iop_ocu_slavelib/SlaveHandlerInterface.h>
#include <fkie_iop_msgs/DigitalResourceEndpoints.h>

namespace iop
{

class DigitalResourceClient : public JTS::Service, public iop::ocu::SlaveHandlerInterface
{
public:
	DigitalResourceClient(JTS::JausRouter* jausRouter , urn_jaus_jss_core_Transport::TransportService* pTransportService, urn_jaus_jss_core_EventsClient::EventsClientService* pEventsClientService, urn_jaus_jss_iop_DigitalResourceDiscoveryClient::DigitalResourceDiscoveryClientService* pDigitalResourceDiscoveryClientService);
	virtual ~DigitalResourceClient();

	virtual bool processTransitions(JTS::InternalEvent* ie);
	virtual bool defaultTransitions(JTS::InternalEvent* ie);

	/// SlaveHandlerInterface Methods
	void control_allowed(std::string service_uri, JausAddress component, unsigned char authority);
	void enable_monitoring_only(std::string service_uri, JausAddress component);
	void access_deactivated(std::string service_uri, JausAddress component);
	void create_events(std::string service_uri, JausAddress component, bool by_query=false);
	void cancel_events(std::string service_uri, JausAddress component, bool by_query=false);

protected:
	JTS::JausRouter* jausRouter;
	urn_jaus_jss_iop_DigitalResourceDiscoveryClient::DigitalResourceDiscoveryClientService* pDigitalResourceDiscoveryClientService;

	ros::NodeHandle p_nh;
	ros::WallTimer p_query_timer;
	ros::Publisher p_pub_endoints;
	JausAddress p_remote_addr;
	bool has_access;

	virtual void run();
	void p_discovered_endpoints(std::vector<digital_resource_endpoint::DigitalResourceEndpoint>, JausAddress &);
	void pQueryCallback(const ros::WallTimerEvent& event);

};

};

#endif // DIGITAL_RESOURCE_ENDPOINT_H
