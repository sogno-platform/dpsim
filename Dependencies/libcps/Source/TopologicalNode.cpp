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

#include <cps/TopologicalNode.h>

using namespace CPS;

TopologicalNode::TopologicalNode(String uid, String name,
	PhaseType phaseType, std::vector<Complex> initialVoltage) 
	: IdentifiedObject(uid, name) {

	mPhaseType = phaseType;
	if (phaseType == PhaseType::ABC) {				
		//mSimNode = simNode;
		mInitialVoltage = MatrixComp::Zero(3, 1);						
		mInitialVoltage << initialVoltage[0], initialVoltage[1], initialVoltage[2];
	}
	else {
		//mSimNode = { simNode[0] };
		mInitialVoltage = MatrixComp::Zero(1, 1);				
		mInitialVoltage << initialVoltage[0];
	}

	addAttribute<MatrixComp>("voltage_init", &mInitialVoltage, Flags::read);			
}

Complex TopologicalNode::initialSingleVoltage(PhaseType phaseType) {		
	if (phaseType == PhaseType::B)
		return mInitialVoltage(1,0);
	else if (phaseType == PhaseType::C)
		return mInitialVoltage(2,0);
	else // phaseType == PhaseType::Single || mPhaseType == PhaseType::A
		return mInitialVoltage(0,0);
}