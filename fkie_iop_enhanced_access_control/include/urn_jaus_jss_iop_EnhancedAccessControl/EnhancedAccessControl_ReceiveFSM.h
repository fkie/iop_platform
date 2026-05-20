

#ifndef ENHANCEDACCESSCONTROL_RECEIVEFSM_H
#define ENHANCEDACCESSCONTROL_RECEIVEFSM_H

#include "InternalEvents/InternalEventHandler.h"
#include "JTSStateMachine.h"
#include "JausUtils.h"
#include "Transport/JausTransport.h"
#include "urn_jaus_jss_iop_EnhancedAccessControl/InternalEvents/InternalEventsSet.h"
#include "urn_jaus_jss_iop_EnhancedAccessControl/Messages/MessageSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_AccessControl/AccessControl_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"

#include "EnhancedAccessControl_ReceiveFSM_sm.h"
#include <fkie_iop_component/iop_component.hpp>
#include <fkie_iop_component/timer.hpp>
#include <fkie_iop_enhanced_access_control/InternalHandoffRequestList.h>
#include <rclcpp/rclcpp.hpp>

namespace urn_jaus_jss_iop_EnhancedAccessControl {

class DllExport EnhancedAccessControl_ReceiveFSM : public JTS::StateMachine {
public:
    EnhancedAccessControl_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
    virtual ~EnhancedAccessControl_ReceiveFSM();
    /// Handle notifications on parent state changes
    virtual void setupNotifications();
    virtual void setupIopConfiguration();

    /// Action Methods
    virtual void processHandoffResponseAction(ConfirmReleaseControl msg, Receive::Body::ReceiveRec transportData);
    virtual void queueHandoffRequestAction(RequestHandoff msg, Receive::Body::ReceiveRec transportData);
    virtual void removeAllHandoffRequestsAction();
    virtual void removeHandoffRequestAction(Receive::Body::ReceiveRec transportData);
    virtual void resetHandoffRequestTimerAction(Receive::Body::ReceiveRec transportData);
    virtual void sendConfirmHandoffRequestAction(std::string arg0, Receive::Body::ReceiveRec transportData);
    virtual void sendConfirmHandoffRequestToAllAction(std::string arg0);
    virtual void sendReportHandoffDecisionTimeoutAction(Receive::Body::ReceiveRec transportData);
    virtual void sendReportHandoffRequestTimeoutAction(Receive::Body::ReceiveRec transportData);
    virtual void sendRequestReleaseControlAction(RequestHandoff msg, Receive::Body::ReceiveRec transportData);
    virtual void setAuthorityAction(RequestHandoff msg);
    virtual void updateHandoffRequestAction(RequestHandoff msg, Receive::Body::ReceiveRec transportData);

    /// Guard Methods
    virtual bool isControllingClient(Receive::Body::ReceiveRec transportData);
    virtual bool isDefaultAuthorityGreater(RequestHandoff msg);
    virtual bool isDeferred(Receive::Body::ReceiveRec transportData);
    virtual bool isQueued(Receive::Body::ReceiveRec transportData);

    EnhancedAccessControl_ReceiveFSMContext* context;

protected:
    /// References to parent FSMs
    urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;
    urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
    urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

    std::shared_ptr<iop::Component> cmp;
    rclcpp::Logger logger;

    iop::Timer p_timer;
    iop::InternalHandoffRequestList p_requests;

    unsigned char p_get_code(std::string response);
    /** Returns descriptions for codes in ConfirmReleaseControl */
    std::string p_code2str(unsigned char code);
    void p_timeout();
    bool p_send_request_release_control();
};

}

#endif // ENHANCEDACCESSCONTROL_RECEIVEFSM_H
