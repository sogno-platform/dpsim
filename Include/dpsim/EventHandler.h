/** EventHandler
 *
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
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

#pragma once

#include <vector>
#include "cps/Config.h"
#include "cps/Definitions.h"
#include "cps/Logger.h"

using namespace CPS;

namespace DPsim {

	class Event {
	public:
		std::function<void()> mfunction;
	};

	class EventHandler {

	protected:
		std::map<Real, std::vector<Event> > mEvents;

	public:
		/// Creates new reader with a name for logging.
		/// The first log level is for the reader and the second for the generated components.
		EventHandler(String name,			
			Logger::Level logLevel = Logger::Level::NONE);

		addEvent(Real startTime, Event newEvent, Int iterations = 1, Real period = 0) {
			std::vector<Event> events;
			events.push_back(newEvent);
			if (!mEvents.insert(std::make_pair(startTime, events)).second) {
				mEvents[startTime].push_back(newEvent);
			}
		}

	};
}

