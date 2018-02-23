

#ifndef UNSOLICITEDHEARTBEAT_RECEIVEFSM_H
#define UNSOLICITEDHEARTBEAT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_iop_UnsolicitedHeartbeat/Messages/MessageSet.h"
#include "urn_jaus_jss_iop_UnsolicitedHeartbeat/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_Liveness/Liveness_ReceiveFSM.h"

#include <ros/ros.h>
#include "UnsolicitedHeartbeat_ReceiveFSM_sm.h"

namespace urn_jaus_jss_iop_UnsolicitedHeartbeat
{

class DllExport UnsolicitedHeartbeat_ReceiveFSM : public JTS::StateMachine
{
public:
	UnsolicitedHeartbeat_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Liveness::Liveness_ReceiveFSM* pLiveness_ReceiveFSM);
	virtual ~UnsolicitedHeartbeat_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void broadcastReportHeartBeatPulseAction();


	/// Guard Methods
	virtual bool hasJAUS_ID();



	UnsolicitedHeartbeat_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_Liveness::Liveness_ReceiveFSM* pLiveness_ReceiveFSM;

	double p_hz;
	ros::NodeHandle p_nh;
	ros::Timer p_timeout_timer;
	JTS::InternalEvent *p_timeout_event;
	JausAddress p_destination;
	void p_timeout(const ros::TimerEvent& event);
};

};

#endif // UNSOLICITEDHEARTBEAT_RECEIVEFSM_H
