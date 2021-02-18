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


#include <fkie_iop_handoff/InternalHandoffRequestList.h>
#include <fkie_iop_component/iop_component.hpp>

using namespace iop;

InternalHandoffRequestList::InternalHandoffRequestList(std::shared_ptr<iop::Component> cmp, unsigned char enhanced_timeout, unsigned char handoff_timeout)
: logger(cmp->get_logger().get_child("InternalHandoffRequestList"))
{
	this->enhanced_timeout = enhanced_timeout;
	this->handoff_timeout = handoff_timeout;
	p_current_id = 0;
	ts_handoff_request = 0;
}

InternalHandoffRequestList::~InternalHandoffRequestList()
{

}

bool InternalHandoffRequestList::contains(JausAddress requestor)
{
	lock_type lock(p_mutex);
	std::vector<std::shared_ptr<HandoffRequest> >::iterator it;
	for (it = p_requests.begin(); it != p_requests.end(); ++it) {
		if (requestor == it->get()->requestor) {
			return true;
		}
	}
	return false;
}

std::shared_ptr<iop::HandoffRequest> InternalHandoffRequestList::get(JausAddress requestor)
{
	lock_type lock(p_mutex);
	std::vector<std::shared_ptr<HandoffRequest> >::iterator it;
	for (it = p_requests.begin(); it != p_requests.end(); ++it) {
		if (requestor == it->get()->requestor) {
			return *it;
		}
	}
	std::shared_ptr<HandoffRequest> nullPtr;
	return nullPtr;
}

std::shared_ptr<iop::HandoffRequest> InternalHandoffRequestList::get(unsigned char id)
{
	lock_type lock(p_mutex);
	std::vector<std::shared_ptr<HandoffRequest> >::iterator it;
	for (it = p_requests.begin(); it != p_requests.end(); ++it) {
		if (id == it->get()->id) {
			return *it;
		}
	}
	std::shared_ptr<HandoffRequest> nullPtr;
	return nullPtr;
}

std::shared_ptr<iop::HandoffRequest> InternalHandoffRequestList::add(JausAddress requestor, unsigned char authority, std::string explanation)
{
	lock_type lock(p_mutex);
	p_current_id++;
	std::shared_ptr<HandoffRequest> request(std::make_shared<HandoffRequest>(p_current_id, requestor, authority, explanation));
	p_requests.push_back(request);
	return p_requests[p_requests.size()-1];
}


std::shared_ptr<iop::HandoffRequest> InternalHandoffRequestList::update(JausAddress requestor, int64_t current_ts)
{
	lock_type lock(p_mutex);
	std::vector<std::shared_ptr<HandoffRequest> >::iterator it;
	for (it = p_requests.begin(); it != p_requests.end(); ++it) {
		if (requestor == it->get()->requestor) {
			it->get()->ts_enhanced_request = p_resolve_ts(current_ts);
			return *it;
		}
	}
	std::shared_ptr<HandoffRequest> nullPtr;
	return nullPtr;
}

std::shared_ptr<iop::HandoffRequest> InternalHandoffRequestList::update(JausAddress requestor, unsigned char authority, std::string explanation, int64_t current_ts)
{
	lock_type lock(p_mutex);
	std::vector<std::shared_ptr<HandoffRequest> >::iterator it;
	for (it = p_requests.begin(); it != p_requests.end(); ++it) {
		if (requestor == it->get()->requestor) {
			it->get()->authority = authority;
			it->get()->explanation = explanation;
			it->get()->ts_enhanced_request = p_resolve_ts(current_ts);
			return *it;
		}
	}
	std::shared_ptr<HandoffRequest> nullPtr;
	return nullPtr;
}

std::shared_ptr<iop::HandoffRequest> InternalHandoffRequestList::remove(unsigned char id)
{
	lock_type lock(p_mutex);
	std::shared_ptr<HandoffRequest> result;
	std::vector<std::shared_ptr<HandoffRequest> >::iterator it;
	for (it = p_requests.begin(); it != p_requests.end(); ++it) {
		if (id == it->get()->id) {
			result = *it;
			p_requests.erase(it);
			return result;
		}
	}
	return result;
}

std::shared_ptr<iop::HandoffRequest> InternalHandoffRequestList::remove(JausAddress requestor)
{
	lock_type lock(p_mutex);
	std::shared_ptr<HandoffRequest> result;
	std::vector<std::shared_ptr<HandoffRequest> >::iterator it;
	for (it = p_requests.begin(); it != p_requests.end(); ++it) {
		if (requestor == it->get()->requestor) {
			result = *it;
			p_requests.erase(it);
			return result;
		}
	}
	return result;
}

std::shared_ptr<iop::HandoffRequest> InternalHandoffRequestList::get_first_expired_enhanced_request(int64_t current_ts)
{
	lock_type lock(p_mutex);
	std::shared_ptr<HandoffRequest> result;
	std::vector<std::shared_ptr<HandoffRequest> >::iterator it;
	for (it = p_requests.begin(); it != p_requests.end(); ++it) {
		if (expired_enhanced_request(it->get()->ts_enhanced_request, current_ts)) {
			result = *it;
			return result;
		}
	}
	return result;
}

std::vector<HandoffRequest> InternalHandoffRequestList::get_all()
{
	lock_type lock(p_mutex);
	std::vector<HandoffRequest> result;
	std::vector<std::shared_ptr<HandoffRequest> >::iterator it;
	for (it = p_requests.begin(); it != p_requests.end(); ++it) {
		result.push_back(*(it->get()));
	}
	return result;
}

bool InternalHandoffRequestList::expired_handoff_request(int64_t current_ts)
{
	if (ts_handoff_request > 0) {
		return p_resolve_ts(current_ts) > ts_handoff_request + handoff_timeout;
	}
	return false;
}

bool InternalHandoffRequestList::expired_enhanced_request(int64_t ts_enhanced_request, int64_t current_ts)
{
	if (ts_enhanced_request > 0) {
		RCLCPP_DEBUG(logger, "COMPARE ts %d, %d, timeout: %d", (int)ts_enhanced_request, (int)p_resolve_ts(current_ts), enhanced_timeout);
		return p_resolve_ts(current_ts) > ts_enhanced_request + enhanced_timeout;
	}
	return false;
}

int64_t InternalHandoffRequestList::p_resolve_ts(int64_t ts)
{
	if (ts == 0) {
		return iop::Component::now_secs();
	}
	return ts;
}
