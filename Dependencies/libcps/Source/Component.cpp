/**
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

#include <cps/Component.h>
#include <cps/Logger.h>
#include <iostream>

using namespace CPS;

Component::Component(String uid, String name, Logger::Level logLevel)
	: IdentifiedObject(uid, name),
	mLogLevel(logLevel) {

	mSLog = Logger::get(name, logLevel);
}

Component::Component(String name, Logger::Level logLevel)
	: Component(name, name, logLevel) { }
