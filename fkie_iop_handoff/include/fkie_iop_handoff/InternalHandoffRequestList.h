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

#include <string>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>

namespace iop
{

class HandoffRequest {
public:
	JausAddress requestor;
	unsigned char authority;
	unsigned char id;
	std::string explanation;
	/// time stamp in seconds of last request. No requests are defined by 0.
	int64_t ts_enhanced_request;

	HandoffRequest(unsigned char id, JausAddress requestor, unsigned char authority, std::string explanation) {
		this->id = id;
		this->requestor = requestor;
		this->authority = authority;
		this->explanation = explanation;
		ts_enhanced_request = iop::Component::now_secs();
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
	InternalHandoffRequestList(std::shared_ptr<iop::Component> cmp, unsigned char enhanced_timeout=10, unsigned char handoff_timeout=60);
	~InternalHandoffRequestList();

	bool contains(JausAddress requestor);
	std::shared_ptr<iop::HandoffRequest> get(JausAddress requestor);
	std::shared_ptr<iop::HandoffRequest> get(unsigned char id);
	/** Does not test for existing handoff requests from the same requestor.*/
	std::shared_ptr<iop::HandoffRequest> add(JausAddress requestor, unsigned char authority, std::string explanation);
	std::shared_ptr<iop::HandoffRequest> update(JausAddress requestor, int64_t current_ts=0);
	std::shared_ptr<iop::HandoffRequest> update(JausAddress requestor, unsigned char authority, std::string explanation, int64_t current_ts=0);
	std::shared_ptr<iop::HandoffRequest> remove(unsigned char id);
	std::shared_ptr<iop::HandoffRequest> remove(JausAddress requestor);
	std::shared_ptr<iop::HandoffRequest> get_first_expired_enhanced_request(int64_t current_ts=0);
	/** Returns a copy of all HandofRequests. */
	std::vector<HandoffRequest> get_all();

	/** Tests if given `ts_handoff_request` has expired.
	 * :param current_ts: current time stamp in seconds. If 0 the time is determine by this method.
	 * :type current_ts: int64_t */
	bool expired_handoff_request(int64_t current_ts=0);

	/** Tests if given `ts_enhanced_request` has expired.
	 * :param ts_enhanced_request: time stamp of last request in seconds. Returns `false` on 0.
	 * :type ts_enhanced_request: int64_t
	 * :param current_ts: current time stamp in seconds. If 0 the time is determine by this method.
	 * :type current_ts: int64_t */
	bool expired_enhanced_request(int64_t ts_enhanced_request, int64_t current_ts=0);

	/// Clients must re-request handoff to prevent being denied handoff request when the timeout expires. A value of zero indicates this feature is disabled.
	unsigned char enhanced_timeout;
	/// The handoff timeout is the amount of time that must pass from when this service first requests a handoff from the current controlling client before the requester is notified that the handoff failed due to a timeout.
	unsigned char handoff_timeout;

	/// time stamp in seconds of last ReuestReleaseControl request. No requests are defined by 0.
	int64_t ts_handoff_request;


protected:
	rclcpp::Logger logger;
	typedef std::recursive_mutex mutex_type;
	typedef std::unique_lock<mutex_type> lock_type;
	mutable mutex_type p_mutex;

	unsigned char p_current_id;
	std::vector<std::shared_ptr<HandoffRequest> > p_requests;
	int64_t p_resolve_ts(int64_t ts);
};

}

#endif
