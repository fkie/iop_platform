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


#include "urn_jaus_jss_iop_PlatformState/PlatformState_PlatformStateFSM.h"

#include <ros/console.h>


using namespace JTS;

namespace urn_jaus_jss_iop_PlatformState
{



PlatformState_PlatformStateFSM::PlatformState_PlatformStateFSM()
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new PlatformState_PlatformStateFSMContext(*this);

}



PlatformState_PlatformStateFSM::~PlatformState_PlatformStateFSM()
{
	delete context;
}

void PlatformState_PlatformStateFSM::setupNotifications()
{

}

void PlatformState_PlatformStateFSM::SendAction(std::string arg0, std::string arg1)
{
	/// Insert User Code HERE
	ROS_WARN("PlatformState: SendAction '%s, %s' not implemented!", arg0.c_str(), arg1.c_str());
}

void PlatformState_PlatformStateFSM::transitionPlatformStateAction(std::string arg0)
{
	/// Insert User Code HERE
	ROS_WARN("PlatformState: transitionPlatformStateAction '%s' not implemented!", arg0.c_str());
}





};
