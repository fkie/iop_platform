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

#include <HealthMonitorPlugin.h>
#include <pluginlib/class_list_macros.h>

using namespace iop;
using namespace urn_jaus_jss_exp_aeodrs_HealthMonitor;
using namespace urn_jaus_jss_core_AccessControl;
using namespace urn_jaus_jss_core_Transport;
using namespace urn_jaus_jss_core_Events;



HealthMonitorPlugin::HealthMonitorPlugin()
{
	p_my_service = NULL;
	p_base_service = NULL;
	p_transport_service = NULL;
	p_events_service = NULL;
}

JTS::Service* HealthMonitorPlugin::get_service()
{
	return p_my_service;
}

void HealthMonitorPlugin::create_service(JTS::JausRouter* jaus_router)
{
	p_base_service = static_cast<AccessControlService *>(get_base_service());
	p_events_service = static_cast<EventsService *>(get_base_service(2));
	p_transport_service = static_cast<TransportService *>(get_base_service(3));
	p_my_service = new HealthMonitorService(jaus_router, p_transport_service, p_events_service, p_base_service);
}

PLUGINLIB_EXPORT_CLASS(iop::HealthMonitorPlugin, iop::PluginInterface)
