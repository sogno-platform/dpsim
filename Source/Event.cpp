/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
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
		if (e->mTime >= currentTime)
			break;

		e->execute();
		std::cout << currentTime << ": Handle event" << std::endl;
		mEvents.pop();
	}
}
