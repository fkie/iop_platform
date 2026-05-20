

#include "urn_jaus_jss_iop_EnhancedAccessControl/EnhancedAccessControl_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>

using namespace JTS;

namespace urn_jaus_jss_iop_EnhancedAccessControl {

EnhancedAccessControl_ReceiveFSM::EnhancedAccessControl_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
    : logger(cmp->get_logger().get_child("EnhancedAccessControl"))
    , p_timer(std::chrono::seconds(1), std::bind(&EnhancedAccessControl_ReceiveFSM::p_timeout, this), false)
    , p_requests(cmp)
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
    pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled", ieHandler, "InternalStateChange_To_EnhancedAccessControl_ReceiveFSM_Receiving_Ready_NotControlled_Available", "AccessControl_ReceiveFSM");
    pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_Controlled", ieHandler, "InternalStateChange_To_EnhancedAccessControl_ReceiveFSM_Receiving_Ready_Controlled_Available", "AccessControl_ReceiveFSM");
    pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_EnhancedAccessControl_ReceiveFSM_Receiving_Ready_NotControlled_Available", "AccessControl_ReceiveFSM");
    pAccessControl_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_EnhancedAccessControl_ReceiveFSM_Receiving_Ready_NotControlled_Available", "AccessControl_ReceiveFSM");
    registerNotification("Receiving_Ready_NotControlled_NotAvailable", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_NotControlled", "EnhancedAccessControl_ReceiveFSM");
    registerNotification("Receiving_Ready_NotControlled_Available", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_NotControlled", "EnhancedAccessControl_ReceiveFSM");
    registerNotification("Receiving_Ready_NotControlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_NotControlled", "EnhancedAccessControl_ReceiveFSM");
    registerNotification("Receiving_Ready_Controlled_NotAvailable", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_Controlled", "EnhancedAccessControl_ReceiveFSM");
    registerNotification("Receiving_Ready_Controlled_Available", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_Controlled", "EnhancedAccessControl_ReceiveFSM");
    registerNotification("Receiving_Ready_Controlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_Controlled", "EnhancedAccessControl_ReceiveFSM");
    registerNotification("Receiving_Ready", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready", "EnhancedAccessControl_ReceiveFSM");
    registerNotification("Receiving", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving", "EnhancedAccessControl_ReceiveFSM");
}

void EnhancedAccessControl_ReceiveFSM::setupIopConfiguration()
{
    iop::Config cfg(cmp, "EnhancedAccessControl");
    int dTimeout = p_requests.decision_timeout;
    cfg.param("decision_timeout", dTimeout, dTimeout);
    p_requests.decision_timeout = dTimeout;
    int rTimeout = p_requests.request_timeout;
    cfg.param("request_timeout", rTimeout, rTimeout);
    p_requests.request_timeout = rTimeout;
    p_timer.start();
}

void EnhancedAccessControl_ReceiveFSM::processHandoffResponseAction(ConfirmReleaseControl msg, Receive::Body::ReceiveRec transportData)
{
    p_requests.ts_request = 0;
    JausAddress address = transportData.getAddress();
    ConfirmReleaseControl::Body::ReleaseControlRec* item = msg.getBody()->getReleaseControlRec();
    unsigned char id = item->getID();
    unsigned char code = item->getResponseCode();
    std::shared_ptr<iop::HandoffRequest> requester = p_requests.get(id);
    RCLCPP_DEBUG(logger, "handoff response for id %d, code %s from %s", (int)id, p_code2str(code).c_str(), address.str().c_str());
    if (requester) {
        ConfirmHandoffRequest reply;
        bool remove = true;
        if (code == 2) { // WAIT
            // do nothing, still send requests
            remove = false;
            reply.getBody()->getConfirmHandoffRequestRec()->setResponseCode(4);
        } else if (code == 0) {
            pAccessControl_ReceiveFSM->sendRejectControlToControllerAction("CONTROL_RELEASED");
            reply.getBody()->getConfirmHandoffRequestRec()->setResponseCode(0);
        } else { // DENIED
            reply.getBody()->getConfirmHandoffRequestRec()->setResponseCode(3);
        }
        reply.getBody()->getConfirmHandoffRequestRec()->setRequestID(id);
        sendJausMessage(reply, requester->requestor);
        if (remove) {
            p_requests.remove(id);
        }
    } else {
        RCLCPP_WARN(logger, "request for handoff response with id %d from %s not found", (int)id, address.str().c_str());
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
        added->ts_request = iop::Component::now_secs();
        // Send a ConfirmHandoffRequest message to querying client
        ConfirmHandoffRequest reply;
        reply.getBody()->getConfirmHandoffRequestRec()->setResponseCode(p_get_code("QUEUED"));
        reply.getBody()->getConfirmHandoffRequestRec()->setRequestID(added->id);
        sendJausMessage(reply, address);
    } else {
        // Send a ConfirmHandoffRequest message to querying client
        RCLCPP_WARN(logger, "queue handoff request from %s, authority %d, explanation `%s` failed!", address.str().c_str(), authority, explanation.c_str());
        ConfirmHandoffRequest reply;
        reply.getBody()->getConfirmHandoffRequestRec()->setResponseCode(p_get_code("NOT_AVAILABLE"));
        sendJausMessage(reply, address);
    }
}

void EnhancedAccessControl_ReceiveFSM::removeAllHandoffRequestsAction()
{
    /// Insert User Code HERE
}

void EnhancedAccessControl_ReceiveFSM::removeHandoffRequestAction(Receive::Body::ReceiveRec transportData)
{
    JausAddress address = transportData.getAddress();
    RCLCPP_DEBUG(logger, "remove handoff request from %s", address.str().c_str());
    std::shared_ptr<iop::HandoffRequest> removed = p_requests.remove(address);
    // inform the controller about queue changes
    if (removed) {
        p_send_request_release_control();
    }
}

void EnhancedAccessControl_ReceiveFSM::resetHandoffRequestTimerAction(Receive::Body::ReceiveRec transportData)
{
    JausAddress address = transportData.getAddress();
    RCLCPP_DEBUG(logger, "reset request timer for %s", address.str().c_str());
    p_requests.update(address);
}

void EnhancedAccessControl_ReceiveFSM::sendConfirmHandoffRequestAction(std::string arg0, Receive::Body::ReceiveRec transportData)
{
    JausAddress address = transportData.getAddress();
    unsigned char code = p_get_code(arg0);
    RCLCPP_DEBUG(logger, "send ConfirmHandoffRequest %s[%d] to %s", arg0.c_str(), code, address.str().c_str());
    ConfirmHandoffRequest msg;
    msg.getBody()->getConfirmHandoffRequestRec()->setRequestID(255);
    msg.getBody()->getConfirmHandoffRequestRec()->setResponseCode(code);
    sendJausMessage(msg, address);
}

void EnhancedAccessControl_ReceiveFSM::sendConfirmHandoffRequestToAllAction(std::string arg0)
{
    /// Insert User Code HERE
    /// Send a ConfirmHandoffRequest message to all pending clients
}

void EnhancedAccessControl_ReceiveFSM::sendReportHandoffDecisionTimeoutAction(Receive::Body::ReceiveRec transportData)
{
    JausAddress address = transportData.getAddress();
    RCLCPP_DEBUG(logger, "report decision timeout %d sec to %s", (int)p_requests.decision_timeout, address.str().c_str());
    ReportHandoffDecisionTimeout reply;
    reply.getBody()->getReportHandoffDecisionTimeoutRec()->setHandoffDecisionTimeout(p_requests.decision_timeout);
    sendJausMessage(reply, address);
}

void EnhancedAccessControl_ReceiveFSM::sendReportHandoffRequestTimeoutAction(Receive::Body::ReceiveRec transportData)
{
    JausAddress address = transportData.getAddress();
    RCLCPP_DEBUG(logger, "report request timeout %d sec to %s", (int)p_requests.request_timeout, address.str().c_str());
    ReportHandoffRequestTimeout reply;
    reply.getBody()->getReportHandoffRequestTimeoutRec()->setHandoffRequestTimeout(p_requests.request_timeout);
    sendJausMessage(reply, address);
}

void EnhancedAccessControl_ReceiveFSM::sendRequestReleaseControlAction(RequestHandoff msg, Receive::Body::ReceiveRec transportData)
{
    /// Insert User Code HERE
    /// Sends a HandoffController.RequestReleaseControl message to the current controller
    p_send_request_release_control();
}

void EnhancedAccessControl_ReceiveFSM::setAuthorityAction(RequestHandoff msg)
{
    /// Insert User Code HERE
    this->pAccessControl_ReceiveFSM->setAuthority(msg.getBody()->getRequestHandoffRec()->getAuthorityCode());
}

void EnhancedAccessControl_ReceiveFSM::updateHandoffRequestAction(RequestHandoff msg, Receive::Body::ReceiveRec transportData)
{
    JausAddress address = transportData.getAddress();
    unsigned char authority = msg.getBody()->getRequestHandoffRec()->getAuthorityCode();
    std::string explanation = msg.getBody()->getRequestHandoffRec()->getExplanation();
    RCLCPP_DEBUG(logger, "update handoff request from %s, authority: %d, explanation: %s", address.str().c_str(), (int)authority, explanation.c_str());
    p_requests.update(address, authority, explanation);
}

bool EnhancedAccessControl_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
    //// By default, inherited guards call the parent function.
    //// This can be replaced or modified as needed.
    return pAccessControl_ReceiveFSM->isControllingClient(transportData);
}

bool EnhancedAccessControl_ReceiveFSM::isDefaultAuthorityGreater(RequestHandoff msg)
{
    urn_jaus_jss_core_AccessControl::RequestControl rc;
    rc.getBody()->getRequestControlRec()->setAuthorityCode(msg.getBody()->getRequestHandoffRec()->getAuthorityCode());
    return pAccessControl_ReceiveFSM->isDefaultAuthorityGreater(rc);
}

bool EnhancedAccessControl_ReceiveFSM::isDeferred(Receive::Body::ReceiveRec transportData)
{
    /// Insert User Code HERE
    return false;
}

bool EnhancedAccessControl_ReceiveFSM::isQueued(Receive::Body::ReceiveRec transportData)
{
    JausAddress address = transportData.getAddress();
    return p_requests.contains(address);
}

unsigned char EnhancedAccessControl_ReceiveFSM::p_get_code(std::string response)
{
    unsigned char result = 1;
    if (strcmp(response.c_str(), "GRANTED") == 0) {
        result = 0;
    } else if (strcmp(response.c_str(), "NOT_AVAILABLE") == 0) {
        result = 1;
    } else if (strcmp(response.c_str(), "HANDOFF_REQUEST_TIMEOUT") == 0) {
        result = 2;
    } else if (strcmp(response.c_str(), "DENIED") == 0) {
        result = 3;
    } else if (strcmp(response.c_str(), "QUEUED") == 0) {
        result = 4;
    } else if (strcmp(response.c_str(), "DEFERRED") == 0) {
        result = 5;
    } else if (strcmp(response.c_str(), "INSUFFICIENT_AUTHORITY") == 0) {
        result = 6;
    } else if (strcmp(response.c_str(), "HANDOFF_DECISION_TIMEOUT") == 0) {
        result = 7;
    } else if (strcmp(response.c_str(), "SERVICE_NOT_CONTROLLED") == 0) {
        result = 8;
    } else if (strcmp(response.c_str(), "REMOVED") == 0) {
        result = 9;
    }
    return result;
}

std::string EnhancedAccessControl_ReceiveFSM::p_code2str(unsigned char code)
{
    std::string result = "UNKNOWN CODE";
    switch (code) {
    case 0:
        result = "GRANTED";
        break;
    case 1:
        result = "NOT_AVAILABLE";
        break;
    case 2:
        result = "HANDOFF_REQUEST_TIMEOUT";
        break;
    case 3:
        result = "DENIED";
        break;
    case 4:
        result = "QUEUED";
        break;
    case 5:
        result = "DEFERRED";
        break;
    case 6:
        result = "INSUFFICIENT_AUTHORITY";
        break;
    case 7:
        result = "HANDOFF_DECISION_TIMEOUT";
        break;
    case 8:
        result = "SERVICE_NOT_CONTROLLED";
        break;
    case 9:
        result = "REMOVED";
        break;
    }
    return result;
}

void EnhancedAccessControl_ReceiveFSM::p_timeout()
{
    int64_t secs = iop::Component::now_secs();
    std::shared_ptr<iop::HandoffRequest> enh_req = p_requests.get_first_expired_request(secs);
    if (enh_req) {
        RCLCPP_WARN(logger, "update for request handoff from ocu %s expired, remove request!", enh_req->requestor.str().c_str());
        // Removes the request for handoff
        p_requests.remove(enh_req->requestor);
        // Send a ConfirmHandoffRequest message to querying client
        ConfirmHandoffRequest reply;
        reply.getBody()->getConfirmHandoffRequestRec()->setResponseCode(p_get_code("DENIED"));
        reply.getBody()->getConfirmHandoffRequestRec()->setRequestID(enh_req->id);
        sendJausMessage(reply, enh_req->requestor);
        // notify the current controller about changed request queue
        p_send_request_release_control();
    }
    if (p_requests.expired_decision_request(secs)) {
        p_requests.ts_request = 0;
        std::vector<iop::HandoffRequest> requests = p_requests.get_all();
        std::vector<iop::HandoffRequest>::iterator it;
        for (it = requests.begin(); it != requests.end(); ++it) {
            RCLCPP_WARN(logger, "request handoff to controlled ocu %s expired, inform the requested OCU: %s!",
                pAccessControl_ReceiveFSM->current_controller().str().c_str(), it->requestor.str().c_str());
            ConfirmHandoffRequest reply;
            reply.getBody()->getConfirmHandoffRequestRec()->setResponseCode(p_get_code("HANDOFF_DECISION_TIMEOUT"));
            reply.getBody()->getConfirmHandoffRequestRec()->setRequestID(it->id);
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
        request.getBody()->getRequestReleaseControlRec()->setAuthorityCode(it->authority);
        request.getBody()->getRequestReleaseControlRec()->setExplanation(it->explanation);
        request.getBody()->getRequestReleaseControlRec()->setID(it->id);
        request.getBody()->getRequestReleaseControlRec()->setSrcSubsystemID(it->requestor.getSubsystemID());
        request.getBody()->getRequestReleaseControlRec()->setSrcNodeID(it->requestor.getNodeID());
        request.getBody()->getRequestReleaseControlRec()->setSrcComponentID(it->requestor.getComponentID());
    }
    if (p_requests.ts_request == 0) {
        p_requests.ts_request = iop::Component::now_secs();
    }
    sendJausMessage(request, pAccessControl_ReceiveFSM->current_controller());
    return true;
}
}
