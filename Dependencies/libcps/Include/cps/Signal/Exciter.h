/** Exciter
 *
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

#include <cps/IdentifiedObject.h>
#include <cps/Logger.h>

namespace CPS {
namespace Signal {
	/// Exciter model
	class Exciter : public IdentifiedObject {
	protected:
		Real mTa;
		Real mKa;
		Real mKe;
		Real mTe;
		Real mKf;
		Real mTf;
		Real mTr;

		Real mLad;
		Real mRfd;

		/// Reference voltage
		Real mVref = 0;
		/// Output of voltage transducer
		Real mVm = 0;
		/// Input of voltage transducer
		Real mVh = 0;
		/// Output of stablizing feedback
		Real mVis = 0;
		/// Output of se function
		Real mVse = 0;
		/// Regulator output
		Real mVr = 0;
		/// Exciter output
		Real mVf = 0;
		/// Auxiliar variable
		Real mVfl = 0;
		/// Initial field votlage
		Real mVf_init;

	public:
		///
		Exciter(String name) : IdentifiedObject(name) { }

		/// Initializes exciter parameters
		void setParameters(Real Ta, Real Ka, Real Te, Real Ke, Real Tf, Real Kf, Real Tr, Real Lad, Real Rfd);
		/// Initializes exciter variables
		void initialize(Real Vh_init, Real Vf_init);
		/// Performs an step to update field voltage value
		Real step(Real mVd, Real mVq, Real Vref, Real dt);
	};
}
}
