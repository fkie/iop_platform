

#ifndef ENHANCEDACCESSCONTROL_RECEIVEFSM_H
#define ENHANCEDACCESSCONTROL_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_iop_EnhancedAccessControl/Messages/MessageSet.h"
#include "urn_jaus_jss_iop_EnhancedAccessControl/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControl/AccessControl_ReceiveFSM.h"

#include <iop_handoff_fkie/InternalHandoffRequestList.h>
#include <ros/ros.h>

#include "EnhancedAccessControl_ReceiveFSM_sm.h"

namespace urn_jaus_jss_iop_EnhancedAccessControl
{

class DllExport EnhancedAccessControl_ReceiveFSM : public JTS::StateMachine
{
public:
	EnhancedAccessControl_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM);
	virtual ~EnhancedAccessControl_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void processHandoffResponseAction(ConfirmReleaseControl msg, Receive::Body::ReceiveRec transportData);
	virtual void queueHandoffRequestAction(RequestHandoff msg, Receive::Body::ReceiveRec transportData);
	virtual void removeHandoffRequestAction(RemoveHandoffRequest msg, Receive::Body::ReceiveRec transportData);
	virtual void removeHandoffRequestAction(RequestHandoff msg, Receive::Body::ReceiveRec transportData);
	virtual void sendConfirmHandoffRequestAction(std::string arg0, Receive::Body::ReceiveRec transportData);
	virtual void sendReportEnhancedTimeoutAction(QueryEnhancedTimeout msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportHandoffTimeoutAction(QueryHandoffTimeout msg, Receive::Body::ReceiveRec transportData);
	virtual void sendRequestReleaseControlAction();
	virtual void setAuthorityAction(RequestHandoff msg);
	virtual void updateHandoffRequestAction(RequestHandoff msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods
	virtual bool isControlAvailable();
	virtual bool isControllingClient(Receive::Body::ReceiveRec transportData);
	virtual bool isDefaultAuthorityGreater(RequestHandoff msg);
	virtual bool isQueued(Receive::Body::ReceiveRec transportData);



	EnhancedAccessControl_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;

	ros::NodeHandle p_nh;
	ros::Timer p_timeout_timer;
	iop::InternalHandoffRequestList p_requests;

	/** Returns descriptions for codes in ConfirmReleaseControl */
	std::string p_crc_code2str(unsigned char code);
	unsigned char p_get_code(std::string response);
	std::string p_code2str(unsigned char code);
	void p_timeout(const ros::TimerEvent& event);
	bool p_send_request_release_control();
};

};

#endif // ENHANCEDACCESSCONTROL_RECEIVEFSM_H
