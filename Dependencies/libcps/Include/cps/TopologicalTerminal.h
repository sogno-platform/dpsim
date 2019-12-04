/**
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

#include <cps/PtrFactory.h>
#include <cps/TopologicalNode.h>
#include <cps/IdentifiedObject.h>

namespace CPS {

	class TopologicalTerminal : public IdentifiedObject {
	public:
		typedef std::shared_ptr<TopologicalTerminal> Ptr;
		typedef std::vector<Ptr> List;
		/// Determines the connection between Component and Node
		PhaseType mPhaseType;
		/// Power through the Terminal
		MatrixComp mPower;
		///
		TopologicalTerminal(String uid, String name, PhaseType phase = PhaseType::Single);
		///
		virtual ~TopologicalTerminal() { }
		/// Returns reference to TopologicalNode
		virtual TopologicalNode::Ptr topologicalNodes() = 0;
		/// Returns Power as complex matrix, where the size depends on the number of phases
		MatrixComp power() { return mPower; }
		/// Returns single complex number for power
		Complex singlePower();
		///
		void setPower(Complex power) { mPower(0,0) = power; }
		///
		void setPower(MatrixComp power) { mPower = power; }
		///
		void setPhaseType(PhaseType type) {
			mPhaseType = type;
			if (mPhaseType == PhaseType::ABC)
				mPower = MatrixComp::Zero(3, 1);
			else
				mPower = MatrixComp::Zero(1, 1);
		}
		///
		Real singleActivePower() { return singlePower().real(); }
		///
		Real singleReactivePower() { return singlePower().imag(); }
		///
		Complex initialSingleVoltage() { return topologicalNodes()->initialSingleVoltage(mPhaseType); }
		///
		MatrixComp initialVoltage();
		///
		UInt simNode() { return topologicalNodes()->simNode(mPhaseType); }
		///
		std::vector<UInt> simNodes() { return topologicalNodes()->simNodes(); }
	};
}
