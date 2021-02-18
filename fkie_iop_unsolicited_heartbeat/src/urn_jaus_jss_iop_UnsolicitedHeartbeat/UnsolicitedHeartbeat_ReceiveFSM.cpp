

#include "urn_jaus_jss_iop_UnsolicitedHeartbeat/UnsolicitedHeartbeat_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>


using namespace JTS;

namespace urn_jaus_jss_iop_UnsolicitedHeartbeat
{



UnsolicitedHeartbeat_ReceiveFSM::UnsolicitedHeartbeat_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_Liveness::Liveness_ReceiveFSM* pLiveness_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: logger(cmp->get_logger().get_child("UnsolicitedHeartbeat")),
  p_timeout_timer(std::chrono::seconds(1), std::bind(&UnsolicitedHeartbeat_ReceiveFSM::p_timeout, this), false)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new UnsolicitedHeartbeat_ReceiveFSMContext(*this);

	this->pLiveness_ReceiveFSM = pLiveness_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
	p_timeout_event = new InternalEvent("PeriodicTimerTrigger", "PeriodicTimeout");
	p_hz = 1.0;
}



UnsolicitedHeartbeat_ReceiveFSM::~UnsolicitedHeartbeat_ReceiveFSM()
{
	p_timeout_timer.stop();
	delete p_timeout_event;
	delete context;
}

void UnsolicitedHeartbeat_ReceiveFSM::setupNotifications()
{
	pLiveness_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_UnsolicitedHeartbeat_ReceiveFSM_Receiving_Ready", "Liveness_ReceiveFSM");
	pLiveness_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_UnsolicitedHeartbeat_ReceiveFSM_Receiving_Ready", "Liveness_ReceiveFSM");
	registerNotification("Receiving_Ready", pLiveness_ReceiveFSM->getHandler(), "InternalStateChange_To_Liveness_ReceiveFSM_Receiving_Ready", "UnsolicitedHeartbeat_ReceiveFSM");
	registerNotification("Receiving", pLiveness_ReceiveFSM->getHandler(), "InternalStateChange_To_Liveness_ReceiveFSM_Receiving", "UnsolicitedHeartbeat_ReceiveFSM");
}


void UnsolicitedHeartbeat_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "UnsolicitedHeartbeat");
	p_destination = JausAddress(this->jausRouter->getJausAddress()->getSubsystemID(), 255, 255);
	cfg.declare_param<double>("hz", p_hz, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"Specifies the rate at which the unsolicited Report Heartbeat Pulse message is send, in Hertz. 0 disables the send.",
		"Default: 1");
	cfg.param<double>("hz", p_hz, p_hz, true);
	if (p_hz > 0) {
		p_timeout_timer.set_rate(p_hz);
		p_timeout_timer.start();
	} else {
		RCLCPP_INFO(logger, "periodic heartbeat disabled");
	}
}

void UnsolicitedHeartbeat_ReceiveFSM::broadcastReportHeartBeatPulseAction()
{
	RCLCPP_DEBUG(logger, "send periodic heartbeat @ %.2fHz", p_hz);
	ReportHeartbeatPulse response;
	sendJausMessage(response, p_destination);
}



bool UnsolicitedHeartbeat_ReceiveFSM::hasJAUS_ID()
{
	return p_destination.get() != 0;
}

void UnsolicitedHeartbeat_ReceiveFSM::p_timeout()
{
	this->getHandler()->invoke(p_timeout_event);
	// create a new event, since the InternalEventHandler deletes the given.
	p_timeout_event = new InternalEvent("PeriodicTimerTrigger", "PeriodicTimeout");
}

}
