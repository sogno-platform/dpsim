/** Real voltage source freq (EMT)
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

namespace DPsim {
namespace Component {
namespace EMT {

	class VoltageSourceFreq : public Component::Base {
	protected:
		Real mVoltageAmp;
		Real mVoltagePhase;
		Real mSwitchTime;
		Real mVoltageDiff;
		Real mResistance;
		Real mConductance;
		Real mCurrent;
		Real mOmegaSource;
		Real mRampTime;

	public:
		VoltageSourceFreq() { };
		VoltageSourceFreq(String name, Int src, Int dest, Real voltage, Real phase, Real resistance, Real omegaSource, Real switchTime, Real rampTime);

		void init(Real om, Real dt) { }
		void applySystemMatrixStamp(SystemModel& system);
		void applyRightSideVectorStamp(SystemModel& system);
		void step(SystemModel& system, Real time);
		void postStep(SystemModel& system) { }
	};

}
}
}
