/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
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
		UInt matrixNodeIndex() { return topologicalNodes()->matrixNodeIndex(mPhaseType); }
		///
		std::vector<UInt> matrixNodeIndices() { return topologicalNodes()->matrixNodeIndices(); }
	};
}
