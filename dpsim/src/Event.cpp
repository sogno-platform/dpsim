/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/Event.h>

using namespace DPsim;
using namespace CPS;

void EventQueue::addEvent(Event::Ptr e) {
	mEvents.push(e);
}

void EventQueue::handleEvents(Real currentTime) {
	Event::Ptr e;

	while (!mEvents.empty()) {
		e = mEvents.top();
		// if current time larger or equal to event time, execute event
		if ( currentTime > e->mTime || (e->mTime - currentTime) < 100e-9) {
			e->execute();
			/// FIXME: Use a logger!
			std::cout << std::scientific << currentTime << ": Handle event time" << std::endl;
			//std::cout << std::scientific << e->mTime << ": Original event time" << std::endl;
			//std::cout << std::scientific << (e->mTime - currentTime)*1e9 << ": Difference to specified event time in ns" << std::endl;
			mEvents.pop();
		} else {
			break;
		}
	}
}
