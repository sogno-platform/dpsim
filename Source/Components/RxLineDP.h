/** RX Line
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

#include "../Components.h"

namespace DPsim {

	class RxLine : public BaseComponent {
	protected:
		Real mResistance;
		Real mConductance;
		Real mVoltageAtNode1Re;
		Real mVoltageAtNode1Im;
		Real mVoltageAtNode2Re;
		Real mVoltageAtNode2Im;

		Real mInductance;
		Real mDeltaVre;
		Real mDeltaVim;
		Real mCurrRe;
		Real mCurrIm;
		Real mCurEqRe;
		Real mCurEqIm;
		Real mGlr;
		Real mGli;
		Real mPrevCurFacRe;
		Real mPrevCurFacIm;

		LineTypes type;

		double correctr, correcti;
		double cureqr_ind, cureqi_ind;
		double deltavr_ind;
		double deltavi_ind;
		double glr_ind, gli_ind;
		double currr_ind;
		double curri_ind;

	public:
		RxLine() { };
		RxLine(std::string name, int node1, int node2, Real resistance, Real inductance);
		RxLine(std::string name, int node1, int node2, int node3, Real resistance, Real inductance);

		virtual void init(Real om, Real dt);
		void applySystemMatrixStamp(SystemModel& system);
		void applyRightSideVectorStamp(SystemModel& system) { }
		void step(SystemModel& system, Real time);
		void postStep(SystemModel& system);
		Complex getCurrent(SystemModel& system);
	};
}
