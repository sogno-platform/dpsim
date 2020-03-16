/**
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
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

#include <cps/IdentifiedObject.h>
#include <cps/MathUtils.h>
#include <cps/PtrFactory.h>

namespace CPS {
	/// Base class for all signal type components
	/// that have only unidirectional connections
	class TopologicalSignalComp : public IdentifiedObject {
	protected:
		/// Component logger
		Logger::Log mSLog;
		/// Component logger control for internal variables
		Logger::Level mLogLevel;
	public:
		typedef std::shared_ptr<TopologicalSignalComp> Ptr;
		typedef std::vector<Ptr> List;

		///
		TopologicalSignalComp(String uid, String name, Logger::Level logLevel = Logger::Level::off)
			: IdentifiedObject(uid, name), mLogLevel(logLevel) {
			mSLog = Logger::get(name, logLevel);
		}
		///
		TopologicalSignalComp(String name, Logger::Level logLevel = Logger::Level::off)
			: TopologicalSignalComp(name, name, logLevel) { }
		///
		virtual ~TopologicalSignalComp() { }
	};
}
