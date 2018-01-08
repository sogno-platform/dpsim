/** Ideal Transformer DP
*
* @file
* @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
* @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
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

#include "Base.h"
#include "DP_Line_Rx.h"
#include "DP_Transformer_Ideal.h"
#include "DP_Inductor.h"

namespace DPsim {
namespace Components {
namespace DP {

	/// Transformer that includes an inductance and resistance
	class Transformer : public Components::Base {

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
		std::shared_ptr<Components::DP::TransformerIdeal> mIdealTransformer;
		/// Internal RX-line to model losses
		std::shared_ptr<Components::DP::RxLine> mLine;
		/// Internal inductor to model losses
		std::shared_ptr<Components::DP::Inductor> mInductor;

	public:
		Transformer(String name, Int node1, Int node2, Real ratioAbs, Real ratioPhase, Real resistance, Real inductance);

		void init(Real om, Real dt);
		void applySystemMatrixStamp(SystemModel& system);
		void applyRightSideVectorStamp(SystemModel& system) { };
		void step(SystemModel& system, Real time);
		void postStep(SystemModel& system);
	};
}
}
}
