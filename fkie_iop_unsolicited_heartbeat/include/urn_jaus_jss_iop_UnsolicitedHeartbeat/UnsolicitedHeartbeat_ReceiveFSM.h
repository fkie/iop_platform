

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

#include "urn_jaus_jss_core_Liveness/Liveness_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"


#include "UnsolicitedHeartbeat_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>
#include <fkie_iop_component/timer.hpp>

namespace urn_jaus_jss_iop_UnsolicitedHeartbeat
{

class DllExport UnsolicitedHeartbeat_ReceiveFSM : public JTS::StateMachine
{
public:
	UnsolicitedHeartbeat_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_Liveness::Liveness_ReceiveFSM* pLiveness_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~UnsolicitedHeartbeat_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
	virtual void broadcastReportHeartBeatPulseAction();


	/// Guard Methods
	virtual bool hasJAUS_ID();



	UnsolicitedHeartbeat_ReceiveFSMContext *context;

protected:

	/// References to parent FSMs
	urn_jaus_jss_core_Liveness::Liveness_ReceiveFSM* pLiveness_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;
	double p_hz;
	iop::Timer p_timeout_timer;
	JTS::InternalEvent *p_timeout_event;
	JausAddress p_destination;
	void p_timeout();
};

}

#endif // UNSOLICITEDHEARTBEAT_RECEIVEFSM_H
