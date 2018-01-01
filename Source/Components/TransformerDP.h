/** Ideal Transformer DP
*
* @file
* @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
* @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
* @license GNU General Public License (version 3)
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

#pragma once

#include "BaseComponent.h"
#include "RxLineDP.h"
#include "IdealTransformerDP.h"
#include "InductorDP.h"

namespace DPsim {

	/// Transformer that includes an inductance and resistance
	class TransformerDP : public BaseComponent {
	private:	
		/// Transformer ratio
		Complex mRatio;
		Real mRatioAbs;
		Real mRatioPhase;
		/// Voltage [V]
		Real mSvVoltage;
		/// Resistance [Ohm]
		Real mResistance;
		/// Conductance [S]
		Real mConductance;
		/// Reactance [Ohm]
		Real mReactance;
		/// Inductance [H]
		Real mInductance;
		/// Internal ideal transformer
		shared_ptr<IdealTransformerDP> mIdealTransformer;
		/// Internal RX-line to model losses
		shared_ptr<RxLineDP> mLine;
		/// Internal inductor to model losses
		shared_ptr<InductorDP> mInductor;

	public:
		TransformerDP() { };
		TransformerDP(String name, Int node1, Int node2, Real ratioAbs, Real ratioPhase, Real resistance, Real inductance);

		void init(Real om, Real dt);
		void applySystemMatrixStamp(SystemModel& system);
		void applyRightSideVectorStamp(SystemModel& system) { };
		void step(SystemModel& system, Real time);
		void postStep(SystemModel& system);
	};
}

