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
#include <cps/IdentifiedObject.h>
#include <cps/MathUtils.h>

namespace CPS {

	class TopologicalNode : public IdentifiedObject {
	protected:
		PhaseType mPhaseType = PhaseType::Single;
		MatrixComp mInitialVoltage;
		Bool mIsGround = false;
	public:
		typedef std::shared_ptr<TopologicalNode> Ptr;
		typedef std::vector<Ptr> List;

		TopologicalNode() { }
		/// This very general constructor is used by other constructors.
		TopologicalNode(String uid, String name,
			PhaseType phaseType, std::vector<Complex> initialVoltage);
		///
		virtual ~TopologicalNode() { }

		///
		Bool isGround() { return mIsGround; }
		///
		MatrixComp initialVoltage() { return mInitialVoltage; }
		///
		void setInitialVoltage(MatrixComp voltage) { mInitialVoltage = voltage; }
		///
		void setInitialVoltage(Complex voltage) { mInitialVoltage(0,0) = voltage; }
		///
		void setInitialVoltage(Complex voltage, Int phaseIndex) {
			mInitialVoltage(phaseIndex, 0) = voltage;
		}
		///
		Complex initialSingleVoltage(PhaseType phaseType = PhaseType::Single);
		///
		PhaseType phaseType() { return mPhaseType; }
		///
		virtual UInt simNode(PhaseType phaseType = PhaseType::Single) = 0;
		///
		virtual std::vector<UInt> simNodes() = 0;
		///
		virtual void setSimNode(UInt phase, UInt simNode) = 0;
	};
}
