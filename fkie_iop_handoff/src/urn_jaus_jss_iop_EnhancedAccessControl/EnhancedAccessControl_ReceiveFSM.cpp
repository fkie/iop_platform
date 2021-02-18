

#include "urn_jaus_jss_iop_EnhancedAccessControl/EnhancedAccessControl_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>


using namespace JTS;

namespace urn_jaus_jss_iop_EnhancedAccessControl
{



EnhancedAccessControl_ReceiveFSM::EnhancedAccessControl_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: logger(cmp->get_logger().get_child("EnhancedAccessControl")),
  p_timer(std::chrono::seconds(1), std::bind(&EnhancedAccessControl_ReceiveFSM::p_timeout, this), false),
  p_requests(cmp)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new EnhancedAccessControl_ReceiveFSMContext(*this);

	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
}



EnhancedAccessControl_ReceiveFSM::~EnhancedAccessControl_ReceiveFSM()
{
	p_timer.stop();
	delete context;
}

void EnhancedAccessControl_ReceiveFSM::setupNotifications()
{
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled", ieHandler, "InternalStateChange_To_EnhancedAccessControl_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_Controlled", ieHandler, "InternalStateChange_To_EnhancedAccessControl_ReceiveFSM_Receiving_Ready_Controlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_EnhancedAccessControl_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_EnhancedAccessControl_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_NotControlled", "EnhancedAccessControl_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_Controlled", "EnhancedAccessControl_ReceiveFSM");
	registerNotification("Receiving_Ready", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready", "EnhancedAccessControl_ReceiveFSM");
	registerNotification("Receiving", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving", "EnhancedAccessControl_ReceiveFSM");
}


void EnhancedAccessControl_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "EnhancedAccessControl");
	int etimeout = p_requests.enhanced_timeout;
	cfg.param("enhanced_timeout", etimeout, etimeout);
	p_requests.enhanced_timeout = etimeout;
	int htimeout = p_requests.handoff_timeout;
	cfg.param("handoff_timeout", htimeout, htimeout);
	p_requests.handoff_timeout = htimeout;
	p_timer.start();
}

void EnhancedAccessControl_ReceiveFSM::processHandoffResponseAction(ConfirmReleaseControl msg, Receive::Body::ReceiveRec transportData)
{
	p_requests.ts_handoff_request = 0;
	JausAddress address = transportData.getAddress();
	for (unsigned int i = 0;i < msg.getBody()->getReleaseControlList()->getNumberOfElements(); i++) {
		ConfirmReleaseControl::Body::ReleaseControlList::ReleaseControlRec *item = msg.getBody()->getReleaseControlList()->getElement(i);
		unsigned char id = item->getID();
		unsigned char code = item->getResponseCode();
		std::shared_ptr<iop::HandoffRequest> requester = p_requests.get(id);
		RCLCPP_DEBUG(logger, "handoff response for id %d, code %s from %s", (int)id, p_crc_code2str(code).c_str(), address.str().c_str());
		if (requester) {
			ConfirmHandoffRequest reply;
			bool remove = true;
			if (code == 2) {  // WAIT
				// do nothing, still send requests
				remove = false;
				reply.getBody()->getConfirmHandoffRequestRec()->setResponseCode(4);
			} else if (code == 0) {
				pAccessControl_ReceiveFSM->sendRejectControlToControllerAction("CONTROL_RELEASED");
				reply.getBody()->getConfirmHandoffRequestRec()->setResponseCode(0);
			} else { // DENIED
				reply.getBody()->getConfirmHandoffRequestRec()->setResponseCode(3);
			}
			reply.getBody()->getConfirmHandoffRequestRec()->setID(id);
			sendJausMessage(reply, requester->requestor);
			if (remove) {
				p_requests.remove(id);
			}
		} else {
			RCLCPP_WARN(logger, "request for handoff response with id %d from %s not found", (int)id, address.str().c_str());
		}
	}
}

void EnhancedAccessControl_ReceiveFSM::queueHandoffRequestAction(RequestHandoff msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress address = transportData.getAddress();
	unsigned char authority = msg.getBody()->getRequestHandoffRec()->getAuthorityCode();
	std::string explanation = msg.getBody()->getRequestHandoffRec()->getExplanation();
	RCLCPP_DEBUG(logger, "queue handoff request from %s, authority %d, explanation %s", address.str().c_str(), authority, explanation.c_str());
	std::shared_ptr<iop::HandoffRequest> added = p_requests.add(address, authority, explanation);
	if (added) {
		added->ts_enhanced_request = iop::Component::now_secs();
		// Send a ConfirmHandoffRequest message to querying client
		ConfirmHandoffRequest reply;
		reply.getBody()->getConfirmHandoffRequestRec()->setResponseCode(p_get_code("QUEUED"));
		reply.getBody()->getConfirmHandoffRequestRec()->setID(added->id);
		sendJausMessage(reply, address);
	} else {
		// Send a ConfirmHandoffRequest message to querying client
		RCLCPP_WARN(logger, "queue handoff request from %s, authority %d, explanation `%s` failed!", address.str().c_str(), authority, explanation.c_str());
		ConfirmHandoffRequest reply;
		reply.getBody()->getConfirmHandoffRequestRec()->setResponseCode(p_get_code("NOT_AVAILABLE"));
		sendJausMessage(reply, address);
	}
}

void EnhancedAccessControl_ReceiveFSM::removeHandoffRequestAction(RemoveHandoffRequest msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress address = transportData.getAddress();
	unsigned char id = msg.getBody()->getRemoveHandoffRequestRec()->getID();
	RCLCPP_DEBUG(logger, "remove handoff request by id %d from %s", id, address.str().c_str());
	std::shared_ptr<iop::HandoffRequest> removed = p_requests.remove(id);
	// inform the controller about queue changes
	if (removed) {
		p_send_request_release_control();
	}
}

void EnhancedAccessControl_ReceiveFSM::removeHandoffRequestAction(RequestHandoff msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress address = transportData.getAddress();
	RCLCPP_DEBUG(logger, "remove handoff request by address %s", address.str().c_str());
	std::shared_ptr<iop::HandoffRequest> removed = p_requests.remove(address);
	// inform the controller about queue changes
	if (removed) {
		p_send_request_release_control();
	}
}

void EnhancedAccessControl_ReceiveFSM::sendConfirmHandoffRequestAction(std::string arg0, Receive::Body::ReceiveRec transportData)
{
	JausAddress address = transportData.getAddress();
	unsigned char code = p_get_code(arg0);
	RCLCPP_DEBUG(logger, "send ConfirmHandoffRequest %s[%d] to %s", arg0.c_str(), code, address.str().c_str());
	ConfirmHandoffRequest msg;
	msg.getBody()->getConfirmHandoffRequestRec()->setID(255);
	msg.getBody()->getConfirmHandoffRequestRec()->setResponseCode(code);
	sendJausMessage(msg, address);
}

void EnhancedAccessControl_ReceiveFSM::sendReportEnhancedTimeoutAction(QueryEnhancedTimeout msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress address = transportData.getAddress();
	RCLCPP_DEBUG(logger, "report enhanced timeout %d sec to %s", (int)p_requests.enhanced_timeout, address.str().c_str());
	ReportEnhancedTimeout reply;
	reply.getBody()->getReportEnhancedTimeoutRec()->setTimeout(p_requests.enhanced_timeout);
	sendJausMessage(reply, address);
}

void EnhancedAccessControl_ReceiveFSM::sendReportHandoffTimeoutAction(QueryHandoffTimeout msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress address = transportData.getAddress();
	RCLCPP_DEBUG(logger, "report handoff timeout %d sec to %s", (int)p_requests.handoff_timeout, address.str().c_str());
	ReportHandoffTimeout reply;
	reply.getBody()->getReportHandoffTimeoutRec()->setTimeout(p_requests.handoff_timeout);
	sendJausMessage(reply, address);
}

void EnhancedAccessControl_ReceiveFSM::sendRequestReleaseControlAction()
{
	p_send_request_release_control();
}

void EnhancedAccessControl_ReceiveFSM::setAuthorityAction(RequestHandoff msg)
{
	/// Insert User Code HERE
}

void EnhancedAccessControl_ReceiveFSM::updateHandoffRequestAction(RequestHandoff msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress address = transportData.getAddress();
	unsigned char authority = msg.getBody()->getRequestHandoffRec()->getAuthorityCode();
	std::string explanation = msg.getBody()->getRequestHandoffRec()->getExplanation();
	RCLCPP_DEBUG(logger, "update handoff request from %s, authority: %d, explanation: %s", address.str().c_str(), (int)authority, explanation.c_str());
	p_requests.update(address, authority, explanation);
}

bool EnhancedAccessControl_ReceiveFSM::isControlAvailable()
{
	//// By default, inherited guards call the parent function.
	//// This can be replaced or modified as needed.
	return pAccessControl_ReceiveFSM->isControlAvailable( );
}

bool EnhancedAccessControl_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
	//// By default, inherited guards call the parent function.
	//// This can be replaced or modified as needed.
	return pAccessControl_ReceiveFSM->isControllingClient(transportData );
}

bool EnhancedAccessControl_ReceiveFSM::isDefaultAuthorityGreater(RequestHandoff msg)
{
	urn_jaus_jss_core_AccessControl::RequestControl rc;
	rc.getBody()->getRequestControlRec()->setAuthorityCode(msg.getBody()->getRequestHandoffRec()->getAuthorityCode());
	return pAccessControl_ReceiveFSM->isDefaultAuthorityGreater(rc);
}

bool EnhancedAccessControl_ReceiveFSM::isQueued(Receive::Body::ReceiveRec transportData)
{
	JausAddress address = transportData.getAddress();
	return p_requests.contains(address);
}

std::string EnhancedAccessControl_ReceiveFSM::p_crc_code2str(unsigned char code)
{
	std::string result = "UNKNOWN CODE";
	switch (code) {
	case 0: result = "GRANTED";
			break;
	case 1: result = "DENIED";
			break;
	case 2: result = "WAIT";
			break;
	}
	return result;
}


unsigned char EnhancedAccessControl_ReceiveFSM::p_get_code(std::string response)
{
	unsigned char result = 1;
	if(strcmp(response.c_str(), "GRANTED") == 0)
	{
		result = 0;
	} else if(strcmp(response.c_str(), "NOT_AVAILABLE") == 0)
	{
		result = 1;
	} else if(strcmp(response.c_str(), "TIMEOUT") == 0)
	{
		result = 2;
	} else if(strcmp(response.c_str(), "DENIED") == 0)
	{
		result = 3;
	} else if(strcmp(response.c_str(), "QUEUED") == 0)
	{
		result = 4;
	} else if(strcmp(response.c_str(), "DEFERRED") == 0)
	{
		result = 5;
	} else if(strcmp(response.c_str(), "INSUFFICIENT_AUTHORITY") == 0)
	{
		result = 6;
	} else if(strcmp(response.c_str(), "WAIT") == 0)
	{
		result = 7;
	}
	return result;
}


std::string EnhancedAccessControl_ReceiveFSM::p_code2str(unsigned char code)
{
	std::string result = "UNKNOWN CODE";
	switch (code) {
	case 0: result = "GRANTED";
			break;
	case 1: result = "NOT_AVAILABLE";
			break;
	case 2: result = "TIMEOUT";
			break;
	case 3: result = "DENIED";
			break;
	case 4: result = "QUEUED";
			break;
	case 5: result = "DEFERRED";
			break;
	case 6: result = "INSUFFICIENT_AUTHORITY";
			break;
	case 7: result = "WAIT";
			break;
	}
	return result;
}

void EnhancedAccessControl_ReceiveFSM::p_timeout()
{
	int64_t secs =iop::Component::now_secs();
	std::shared_ptr<iop::HandoffRequest> enh_req = p_requests.get_first_expired_enhanced_request(secs);
	if (enh_req) {
		RCLCPP_WARN(logger, "update for request handoff from ocu %s expired, remove request!", enh_req->requestor.str().c_str());
		// Removes the request for handoff
		p_requests.remove(enh_req->requestor);
		// Send a ConfirmHandoffRequest message to querying client
		ConfirmHandoffRequest reply;
		reply.getBody()->getConfirmHandoffRequestRec()->setResponseCode(p_get_code("DENIED"));
		reply.getBody()->getConfirmHandoffRequestRec()->setID(enh_req->id);
		sendJausMessage(reply, enh_req->requestor);
		// notify the current controller about changed request queue
		p_send_request_release_control();
	}
	if (p_requests.expired_handoff_request(secs)) {
		p_requests.ts_handoff_request = 0;
		std::vector<iop::HandoffRequest> requests = p_requests.get_all();
		std::vector<iop::HandoffRequest>::iterator it;
		for (it = requests.begin(); it != requests.end(); ++it) {
			RCLCPP_WARN(logger, "request handoff to controlled ocu %s expired, inform the requested OCU: %s!",
					pAccessControl_ReceiveFSM->current_controller().str().c_str(), it->requestor.str().c_str());
			ConfirmHandoffRequest reply;
			reply.getBody()->getConfirmHandoffRequestRec()->setResponseCode(p_get_code("TIMEOUT"));
			reply.getBody()->getConfirmHandoffRequestRec()->setID(it->id);
			sendJausMessage(reply, it->requestor);
		}
	}
}

bool EnhancedAccessControl_ReceiveFSM::p_send_request_release_control()
{
	if (pAccessControl_ReceiveFSM->current_controller().get() == 0) {
		return false;
	}
	std::vector<iop::HandoffRequest> requests = p_requests.get_all();
	std::vector<iop::HandoffRequest>::iterator it;
	urn_jaus_jss_iop_EnhancedAccessControl::RequestReleaseControl request;
	for (it = requests.begin(); it != requests.end(); ++it) {
		RCLCPP_DEBUG(logger, "send request release control to controlled ocu %s for requested OCU: %s, authority: %d, explanation: `%s`",
				pAccessControl_ReceiveFSM->current_controller().str().c_str(), it->requestor.str().c_str(), it->authority, it->explanation.c_str());
		RequestReleaseControl::Body::RequestReleaseControlList::RequestReleaseControlRec rrcrec;
		rrcrec.setAuthorityCode(it->authority);
		rrcrec.setExplanation(it->explanation);
		rrcrec.setID(it->id);
		rrcrec.setSrcSubsystemID(it->requestor.getSubsystemID());
		rrcrec.setSrcNodeID(it->requestor.getNodeID());
		rrcrec.setSrcComponentID(it->requestor.getComponentID());
		request.getBody()->getRequestReleaseControlList()->addElement(rrcrec);
	}
	if (p_requests.ts_handoff_request == 0) {
		p_requests.ts_handoff_request = iop::Component::now_secs();
	}
	sendJausMessage(request, pAccessControl_ReceiveFSM->current_controller());
	return true;
}

}
