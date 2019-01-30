

#include "urn_jaus_jss_iop_UnsolicitedHeartbeat/UnsolicitedHeartbeat_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.h>



using namespace JTS;

namespace urn_jaus_jss_iop_UnsolicitedHeartbeat
{



UnsolicitedHeartbeat_ReceiveFSM::UnsolicitedHeartbeat_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Liveness::Liveness_ReceiveFSM* pLiveness_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new UnsolicitedHeartbeat_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pLiveness_ReceiveFSM = pLiveness_ReceiveFSM;
	p_timeout_event = new InternalEvent("PeriodicTimerTrigger", "PeriodicTimeout");
	p_hz = 1.0;
}



UnsolicitedHeartbeat_ReceiveFSM::~UnsolicitedHeartbeat_ReceiveFSM()
{
	delete p_timeout_event;
	delete context;
}

void UnsolicitedHeartbeat_ReceiveFSM::setupNotifications()
{
	pLiveness_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_UnsolicitedHeartbeat_ReceiveFSM_Receiving_Ready", "Liveness_ReceiveFSM");
	pLiveness_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_UnsolicitedHeartbeat_ReceiveFSM_Receiving_Ready", "Liveness_ReceiveFSM");
	registerNotification("Receiving_Ready", pLiveness_ReceiveFSM->getHandler(), "InternalStateChange_To_Liveness_ReceiveFSM_Receiving_Ready", "UnsolicitedHeartbeat_ReceiveFSM");
	registerNotification("Receiving", pLiveness_ReceiveFSM->getHandler(), "InternalStateChange_To_Liveness_ReceiveFSM_Receiving", "UnsolicitedHeartbeat_ReceiveFSM");
	p_destination = JausAddress(this->jausRouter->getJausAddress()->getSubsystemID(), 255, 255);
	iop::Config cfg("~UnsolicitedHeartbeat");
	cfg.param("hz", p_hz, p_hz);
	if (p_hz > 0) {
		p_timeout_timer = p_nh.createTimer(ros::Duration(1.0 / p_hz), &UnsolicitedHeartbeat_ReceiveFSM::p_timeout, this);
	} else {
		ROS_INFO_NAMED("UnsolicitedHeartbeat", "periodic heartbeat disabled");
	}
}

void UnsolicitedHeartbeat_ReceiveFSM::broadcastReportHeartBeatPulseAction()
{
	ROS_DEBUG_NAMED("UnsolicitedHeartbeat", "send periodic heartbeat @ %.2fHz", p_hz);
	ReportHeartbeatPulse response;
	sendJausMessage(response, p_destination);
}



bool UnsolicitedHeartbeat_ReceiveFSM::hasJAUS_ID()
{
	return p_destination.get() != 0;
}

void UnsolicitedHeartbeat_ReceiveFSM::p_timeout(const ros::TimerEvent& event)
{
	this->getHandler()->invoke(p_timeout_event);
	// create a new event, since the InternalEventHandler deletes the given.
	p_timeout_event = new InternalEvent("PeriodicTimerTrigger", "PeriodicTimeout");
}

};
