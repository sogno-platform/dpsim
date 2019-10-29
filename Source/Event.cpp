/** Event system
 *
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
