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

#include <cps/TopologicalTerminal.h>

using namespace CPS;

TopologicalTerminal::TopologicalTerminal(String uid, String name, PhaseType phase)
	: IdentifiedObject(uid, name) {
	setPhaseType(phase);
}

Complex TopologicalTerminal::singlePower() {
	if (mPhaseType == PhaseType::B)
		return mPower(1,0);
	else if (mPhaseType == PhaseType::C)
		return mPower(2,0);
	else // mPhaseType == PhaseType::Single || mPhaseType == PhaseType::A
		return mPower(0,0);
}

MatrixComp TopologicalTerminal::initialVoltage() {
	if (mPhaseType == PhaseType::Single || mPhaseType == PhaseType::A)
		return topologicalNodes()->initialVoltage().block(0,0,1,1);
	else if (mPhaseType == PhaseType::B)
		return topologicalNodes()->initialVoltage().block(1,0,1,1);
	else if (mPhaseType == PhaseType::C)
		return topologicalNodes()->initialVoltage().block(2,0,1,1);
	else
		return topologicalNodes()->initialVoltage();
}

