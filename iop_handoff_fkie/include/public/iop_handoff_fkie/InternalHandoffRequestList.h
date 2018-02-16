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


#ifndef IOPINTERNALHANDOFFREQUESTLIST_H
#define IOPINTERNALHANDOFFREQUESTLIST_H

#include "Transport/JausTransport.h"
#include "urn_jaus_jss_iop_EnhancedAccessControl/Messages/MessageSet.h"
#include "urn_jaus_jss_iop_EnhancedAccessControl/InternalEvents/InternalEventsSet.h"

#include <boost/thread/recursive_mutex.hpp>
#include <ros/ros.h>
#include <string>

namespace iop
{

class HandoffRequest {
public:
	JausAddress requestor;
	unsigned char authority;
	unsigned char id;
	std::string explanation;
	/// time stamp in seconds of last request. No requests are defined by 0.
	unsigned int ts_enhanced_request;

	HandoffRequest(unsigned char id, JausAddress requestor, unsigned char authority, std::string explanation) {
		this->id = id;
		this->requestor = requestor;
		this->authority = authority;
		this->explanation = explanation;
		ts_enhanced_request = ros::Time::now().nsec;
	}

	HandoffRequest() {
		this->id = 0;
		this->authority = 255;
		ts_enhanced_request = 0;
	}

	bool valid() {
		return requestor.get() != 0;
	}
};


class InternalHandoffRequestList {
public:
	InternalHandoffRequestList(unsigned char enhanced_timeout=10, unsigned char handoff_timeout=60);
	~InternalHandoffRequestList();

	bool contains(JausAddress requestor);
	boost::shared_ptr<iop::HandoffRequest> get(JausAddress requestor);
	boost::shared_ptr<iop::HandoffRequest> get(unsigned char id);
	/** Does not test for existing handoff requests from the same requestor.*/
	boost::shared_ptr<iop::HandoffRequest> add(JausAddress requestor, unsigned char authority, std::string explanation);
	boost::shared_ptr<iop::HandoffRequest> update(JausAddress requestor, unsigned int current_ts=0);
	boost::shared_ptr<iop::HandoffRequest> update(JausAddress requestor, unsigned char authority, std::string explanation, unsigned int current_ts=0);
	boost::shared_ptr<iop::HandoffRequest> remove(unsigned char id);
	boost::shared_ptr<iop::HandoffRequest> remove(JausAddress requestor);
	boost::shared_ptr<iop::HandoffRequest> get_first_expired_enhanced_request(unsigned int current_ts=0);
	/** Returns a copy of all HandofRequests. */
	std::vector<HandoffRequest> get_all();

	/** Tests if given `ts_handoff_request` has expired.
	 * :param current_ts: current time stamp in seconds. If 0 the time is determine by this method.
	 * :type current_ts: unsigend int */
	bool expired_handoff_request(unsigned int current_ts=0);

	/** Tests if given `ts_enhanced_request` has expired.
	 * :param ts_enhanced_request: time stamp of last request in seconds. Returns `false` on 0.
	 * :type ts_enhanced_request: unsigend int
	 * :param current_ts: current time stamp in seconds. If 0 the time is determine by this method.
	 * :type current_ts: unsigend int */
	bool expired_enhanced_request(unsigned int ts_enhanced_request, unsigned int current_ts=0);

	/// Clients must re-request handoff to prevent being denied handoff request when the timeout expires. A value of zero indicates this feature is disabled.
	unsigned char enhanced_timeout;
	/// The handoff timeout is the amount of time that must pass from when this service first requests a handoff from the current controlling client before the requester is notified that the handoff failed due to a timeout.
	unsigned char handoff_timeout;

	/// time stamp in seconds of last ReuestReleaseControl request. No requests are defined by 0.
	unsigned int ts_handoff_request;


protected:
	typedef boost::recursive_mutex mutex_type;
	typedef boost::unique_lock<mutex_type> lock_type;
	mutable mutex_type p_mutex;

	unsigned char p_current_id;
	std::vector<boost::shared_ptr<HandoffRequest> > p_requests;
	unsigned int p_resolve_ts(unsigned int ts);
};

};

#endif
